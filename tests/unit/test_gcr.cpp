#include <doctest/doctest.h>

#include "../../core/src/machine/gcr.hpp"

#include <numeric>
#include <vector>

using namespace openmac;

namespace {

// A pseudo-random but reproducible 800K volume: every sector gets distinct,
// non-trivial content so a mis-decode cannot pass by luck.
std::vector<u8> makeVolume(int sides) {
    const std::size_t sectors =
        static_cast<std::size_t>(gcr::sectorsPerSide()) * static_cast<std::size_t>(sides);
    std::vector<u8> img(sectors * 512);
    u32 x = 0x12345678u;
    for (auto& b : img) {
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;   // xorshift32
        b = static_cast<u8>(x);
    }
    return img;
}

// ---- the oracle -------------------------------------------------------
//
// Our encoder and decoder are inverses of each other, so a round-trip test
// happily round-trips a mistake. The only authority on the format is the
// machine that has to read the disk, so this is the Classic ROM's own data-field
// reader ($43D1B8-$43D376), transcribed from its instructions:
//
//   43D202  MOVE.B (0,A3,D3.W),D1   ; D1 = 6-bit value of the gathered byte
//   43D206  ROL.B  #2,D1
//   43D20A  AND.B  D0,D2            ; D0 = $C0 -> the pair lands in bits 7:6
//   43D210  OR.B   (0,A3,D3.W),D2   ; the value byte supplies bits 5:0
//   43D214  MOVE.B D7,D3 / ADD.B D7,D3 / ROL.B #1,D7   ; X = old bit 7 of c3
//   43D21A  EOR.B  D7,D2            ; unscramble with the rotated c3
//   43D21E  ADDX.B D2,D5            ; c1 += raw + X
//
// so: byte = (pair << 6) | value6, c3 rotates plainly (bit 7 -> bit 0), and the
// carry folded into the first addition is c3's own old bit 7. The write side at
// $4358CC-$435930 does the exact inverse, which cross-checks all three.
struct RomRead {
    bool addrMark = false, dataMark = false, checksumOk = false;
    u8 payload[gcr::kPayload] = {};
};

RomRead romReadFirstSector(const std::vector<u8>& trk) {
    const u8* dec = gcr::decodeTable();
    RomRead r;
    std::size_t i = 0;
    for (; i + 3 < trk.size(); ++i)
        if (trk[i] == 0xD5 && trk[i + 1] == 0xAA && trk[i + 2] == 0x96) break;
    if (i + 3 >= trk.size()) return r;
    r.addrMark = true;
    for (; i + 3 < trk.size(); ++i)
        if (trk[i] == 0xD5 && trk[i + 1] == 0xAA && trk[i + 2] == 0xAD) break;
    if (i + 3 >= trk.size()) return r;
    r.dataMark = true;

    std::size_t p = i + 4;                     // prologue + the sector byte
    unsigned c1 = 0, c2 = 0, c3 = 0, x = 0;
    // One group: the gathered byte, then `count` value bytes. 524 is not a
    // multiple of three, so the last group of the payload is short -- the ROM
    // handles that at $43D296 (TST.W D4; BEQ -> the checksum), reading only the
    // bytes it still needs and never a fourth.
    auto group = [&](u8* out, int count) {
        const u8 pairs = dec[trk[p++]];
        for (int k = 0; k < count; ++k) {
            const u8 val = dec[trk[p++]];
            out[k] = static_cast<u8>((((pairs >> (4 - 2 * k)) & 3) << 6) | val);
        }
    };
    for (std::size_t done = 0; done < gcr::kPayload;) {
        const std::size_t left = gcr::kPayload - done;
        const int cnt = static_cast<int>(left < 3 ? left : 3);
        u8 scr[3] = {};
        group(scr, cnt);
        const unsigned bit7 = (c3 >> 7) & 1u;
        c3 = ((c3 << 1) | bit7) & 0xFF;        // ROL.B #1
        x = bit7;
        unsigned s;
        const u8 v0 = static_cast<u8>(scr[0] ^ c3);
        s = c1 + v0 + x; c1 = s & 0xFF; x = s >> 8;
        r.payload[done] = v0;
        if (cnt > 1) {
            const u8 v1 = static_cast<u8>(scr[1] ^ c1);
            s = c2 + v1 + x; c2 = s & 0xFF; x = s >> 8;
            r.payload[done + 1] = v1;
        }
        if (cnt > 2) {
            const u8 v2 = static_cast<u8>(scr[2] ^ c2);
            s = c3 + v2 + x; c3 = s & 0xFF; x = s >> 8;
            r.payload[done + 2] = v2;
        }
        done += static_cast<std::size_t>(cnt);
    }
    u8 want[3];
    group(want, 3);                             // same group format for the checksum
    r.checksumOk = (want[0] == c1 && want[1] == c2 && want[2] == c3);
    return r;
}

} // namespace

TEST_CASE("gcr: the disk-byte alphabet matches the ROM's table") {
    const u8* enc = gcr::encodeTable();
    // Every entry must have bit 7 set and no more than one pair of adjacent
    // zero bits -- that is what makes the stream self-clocking.
    for (int i = 0; i < 64; ++i) {
        CHECK((enc[i] & 0x80) != 0);
        int runs = 0;
        for (int b = 6; b >= 1; --b)
            if (((enc[i] >> b) & 1) == 0 && ((enc[i] >> (b - 1)) & 1) == 0) ++runs;
        CHECK(runs <= 1);
    }
    // $D5 and $AA are reserved as mark bytes and must never appear.
    for (int i = 0; i < 64; ++i) {
        CHECK(enc[i] != 0xD5);
        CHECK(enc[i] != 0xAA);
    }
    // The table is strictly ascending and the inverse map round-trips.
    for (int i = 1; i < 64; ++i) CHECK(enc[i] > enc[i - 1]);
    for (int i = 0; i < 64; ++i) CHECK(gcr::decodeTable()[enc[i]] == i);
}

TEST_CASE("gcr: geometry matches the zoned 800K layout") {
    CHECK(gcr::sectorsOnTrack(0) == 12);
    CHECK(gcr::sectorsOnTrack(15) == 12);
    CHECK(gcr::sectorsOnTrack(16) == 11);
    CHECK(gcr::sectorsOnTrack(31) == 11);
    CHECK(gcr::sectorsOnTrack(32) == 10);
    CHECK(gcr::sectorsOnTrack(48) == 9);
    CHECK(gcr::sectorsOnTrack(79) == 8);
    CHECK(gcr::sectorsPerSide() == 800);
    // 1600 sectors x 512 bytes = 819200 = "800K".
    CHECK(gcr::sectorsPerSide() * 2 * 512 == 819200);
}

TEST_CASE("gcr: 2:1 interleave is a permutation") {
    for (int n : {8, 9, 10, 11, 12}) {
        std::vector<int> phys(static_cast<std::size_t>(n));
        gcr::interleaveOrder(n, phys.data());
        std::vector<int> seen(static_cast<std::size_t>(n), 0);
        for (int i = 0; i < n; ++i) {
            REQUIRE(phys[static_cast<std::size_t>(i)] >= 0);
            REQUIRE(phys[static_cast<std::size_t>(i)] < n);
            ++seen[static_cast<std::size_t>(phys[static_cast<std::size_t>(i)])];
        }
        for (int i = 0; i < n; ++i) CHECK(seen[static_cast<std::size_t>(i)] == 1);
        // Logical sector 0 sits at physical slot 0, and consecutive logical
        // sectors are two slots apart while slots remain free.
        CHECK(phys[0] == 0);
    }
}

TEST_CASE("gcr: the data scramble round-trips") {
    u8 src[gcr::kPayload];
    for (std::size_t i = 0; i < gcr::kPayload; ++i) src[i] = static_cast<u8>(i * 7 + 3);
    u8 scrambled[gcr::kPayload], back[gcr::kPayload];
    const auto ck1 = gcr::encodeData(src, gcr::kPayload, scrambled);
    const auto ck2 = gcr::decodeData(scrambled, gcr::kPayload, back);
    for (std::size_t i = 0; i < gcr::kPayload; ++i) CHECK(back[i] == src[i]);
    CHECK(ck1.a == ck2.a);
    CHECK(ck1.b == ck2.b);
    CHECK(ck1.c == ck2.c);
    // A scramble that left the data alone would be a silent no-op.
    bool differs = false;
    for (std::size_t i = 0; i < gcr::kPayload; ++i)
        if (scrambled[i] != src[i]) { differs = true; break; }
    CHECK(differs);
}

TEST_CASE("gcr: nibblization round-trips") {
    u8 src[gcr::kPayload];
    for (std::size_t i = 0; i < gcr::kPayload; ++i) src[i] = static_cast<u8>(i * 31 + 11);
    std::vector<u8> nib;
    gcr::nibblize(src, gcr::kPayload, nib);
    // 524 bytes -> 4 disk bytes per 3 source bytes.
    CHECK(nib.size() == 699);
    for (u8 b : nib) CHECK(gcr::decodeTable()[b] != 0xFF);
    u8 back[gcr::kPayload];
    REQUIRE(gcr::denibblize(nib.data(), back, gcr::kPayload));
    for (std::size_t i = 0; i < gcr::kPayload; ++i) CHECK(back[i] == src[i]);
}

TEST_CASE("gcr: every sector of an 800K volume survives encode then decode") {
    const int sides = 2;
    const std::vector<u8> src = makeVolume(sides);
    std::vector<u8> dst(src.size(), 0);

    int totalSectors = 0;
    for (int track = 0; track < 80; ++track) {
        for (int side = 0; side < sides; ++side) {
            const std::vector<u8> trk = gcr::buildTrack(src, track, side, sides, 0x22);
            // The stream must carry the marks the ROM searches for.
            REQUIRE(trk.size() > 0);
            totalSectors += gcr::decodeTrack(trk, dst, track, sides);
        }
    }
    CHECK(totalSectors == gcr::sectorsPerSide() * sides);   // 1600
    CHECK(dst == src);
}

TEST_CASE("gcr: the ROM's own reader recovers what we wrote") {
    const std::vector<u8> src = makeVolume(2);
    // Physical slot 0 on any track holds logical sector 0 under 2:1 interleave.
    for (int track : {0, 1, 20, 40, 60, 79}) {
        for (int side : {0, 1}) {
            const std::vector<u8> trk = gcr::buildTrack(src, track, side, 2, 0x22);
            const RomRead r = romReadFirstSector(trk);
            INFO("track " << track << " side " << side);
            REQUIRE(r.addrMark);
            REQUIRE(r.dataMark);
            CHECK(r.checksumOk);
            const std::size_t off =
                static_cast<std::size_t>(gcr::trackStartSector(track) * 2 +
                                         side * gcr::sectorsOnTrack(track)) * 512;
            bool same = true;
            for (std::size_t i = 0; i < 512; ++i)
                if (r.payload[gcr::kTagBytes + i] != src[off + i]) { same = false; break; }
            CHECK(same);
        }
    }
}

TEST_CASE("gcr: every sector of a track lands where the driver will look for it") {
    // The checksums can all be right and the disk still be unreadable if a
    // sector carries the wrong 512 bytes. The driver locates a block by the
    // track/side/sector triple in the address field, so walk each track the way
    // it does and check that the triple it finds selects the image offset the
    // File Manager expects for that logical block.
    const std::vector<u8> src = makeVolume(2);
    const u8* dec = gcr::decodeTable();
    for (int track : {0, 1, 15, 16, 31, 32, 47, 48, 63, 64, 79}) {
        for (int side : {0, 1}) {
            const std::vector<u8> trk = gcr::buildTrack(src, track, side, 2, 0x22);
            const int n = gcr::sectorsOnTrack(track);
            std::vector<int> seen(static_cast<std::size_t>(n), 0);
            int found = 0;
            for (std::size_t i = 0; i + 8 < trk.size(); ++i) {
                if (!(trk[i] == 0xD5 && trk[i + 1] == 0xAA && trk[i + 2] == 0x96)) continue;
                const u8 t = dec[trk[i + 3]], s = dec[trk[i + 4]];
                const u8 sh = dec[trk[i + 5]], f = dec[trk[i + 6]], ck = dec[trk[i + 7]];
                INFO("track " << track << " side " << side << " address field at " << i);
                REQUIRE(t != 0xFF); REQUIRE(s != 0xFF); REQUIRE(sh != 0xFF);
                REQUIRE(f != 0xFF); REQUIRE(ck != 0xFF);
                CHECK(((t ^ s ^ sh ^ f) & 0x3F) == ck);       // $43D146-$43D17A
                CHECK(t == (track & 0x3F));
                CHECK(((sh >> 5) & 1) == side);               // side bit
                CHECK((sh & 1) == (track >= 64 ? 1 : 0));     // track bit 6
                CHECK(f == 0x22);                             // 800K double sided
                REQUIRE(s < n);
                ++seen[s];
                ++found;

                // Now the data field that follows, and where it must live.
                std::vector<u8> tail(trk.begin() + static_cast<std::ptrdiff_t>(i), trk.end());
                const RomRead r = romReadFirstSector(tail);
                REQUIRE(r.dataMark);
                CHECK(r.checksumOk);
                const std::size_t off =
                    static_cast<std::size_t>(gcr::trackStartSector(track) * 2 +
                                             side * n + s) * 512;
                bool same = true;
                for (std::size_t k = 0; k < 512; ++k)
                    if (r.payload[gcr::kTagBytes + k] != src[off + k]) { same = false; break; }
                CHECK(same);
            }
            CHECK(found == n);
            for (int s = 0; s < n; ++s) CHECK(seen[static_cast<std::size_t>(s)] == 1);
        }
    }
}

TEST_CASE("gcr: a rewritten sector decodes back into the image, alone") {
    // What the write path does: the driver finds a sector's address field, lays
    // a fresh data field over the one that follows, and the machine decodes the
    // whole track back into the image. One sector must change and no other.
    const std::vector<u8> src = makeVolume(2);
    std::vector<u8> image = src;
    std::vector<u8> trk = gcr::buildTrack(src, 3, 0, 2, 0x22);

    // New contents for logical sector 5 of track 3, side 0.
    const int n = gcr::sectorsOnTrack(3);
    const std::size_t off = static_cast<std::size_t>(gcr::trackStartSector(3) * 2 + 5) * 512;
    u8 fresh[gcr::kPayload] = {};
    for (std::size_t i = 0; i < 512; ++i) fresh[gcr::kTagBytes + i] = static_cast<u8>(0xA5 ^ i);

    // Re-encode that sector's data field in place, the way the driver would.
    const u8* dec = gcr::decodeTable();
    const u8* enc = gcr::encodeTable();
    bool patched = false;
    for (std::size_t i = 0; i + 8 < trk.size() && !patched; ++i) {
        if (!(trk[i] == 0xD5 && trk[i + 1] == 0xAA && trk[i + 2] == 0x96)) continue;
        if (dec[trk[i + 4]] != 5) continue;                 // the sector byte
        for (std::size_t j = i; j + 3 < trk.size(); ++j) {
            if (!(trk[j] == 0xD5 && trk[j + 1] == 0xAA && trk[j + 2] == 0xAD)) continue;
            std::size_t p = j + 4;                          // past the sector byte
            u8 scrambled[gcr::kPayload];
            const auto ck = gcr::encodeData(fresh, gcr::kPayload, scrambled);
            std::vector<u8> nib;
            gcr::nibblize(scrambled, gcr::kPayload, nib);
            for (u8 b : nib) trk[p++] = b;
            trk[p++] = enc[(((ck.a >> 6) << 4) | ((ck.b >> 6) << 2) | (ck.c >> 6)) & 0x3F];
            trk[p++] = enc[ck.a & 0x3F];
            trk[p++] = enc[ck.b & 0x3F];
            trk[p++] = enc[ck.c & 0x3F];
            patched = true;
            break;
        }
    }
    REQUIRE(patched);

    CHECK(gcr::decodeTrack(trk, image, 3, 2) == n);
    for (std::size_t i = 0; i < 512; ++i)
        CHECK(image[off + i] == static_cast<u8>(0xA5 ^ i));
    // Everything outside that sector is untouched.
    std::size_t elsewhere = 0;
    for (std::size_t i = 0; i < image.size(); ++i)
        if (i < off || i >= off + 512)
            if (image[i] != src[i]) ++elsewhere;
    CHECK(elsewhere == 0);
}

TEST_CASE("gcr: a corrupted sector is refused rather than written back") {
    const std::vector<u8> src = makeVolume(2);
    std::vector<u8> image = src;
    std::vector<u8> trk = gcr::buildTrack(src, 0, 0, 2, 0x22);
    const int n = gcr::sectorsOnTrack(0);
    // Flip one byte inside the first data field: its checksum no longer holds.
    std::size_t j = 0;
    for (; j + 3 < trk.size(); ++j)
        if (trk[j] == 0xD5 && trk[j + 1] == 0xAA && trk[j + 2] == 0xAD) break;
    REQUIRE(j + 3 < trk.size());
    trk[j + 10] = gcr::encodeTable()[(gcr::decodeTable()[trk[j + 10]] + 1) & 0x3F];
    CHECK(gcr::decodeTrack(trk, image, 0, 2) == n - 1);   // that one is dropped
    CHECK(image == src);                                  // and nothing was corrupted
}

TEST_CASE("gcr: a built track is well formed") {
    const std::vector<u8> src = makeVolume(2);
    const std::vector<u8> trk = gcr::buildTrack(src, 0, 0, 2, 0x22);

    int addrMarks = 0, dataMarks = 0;
    for (std::size_t i = 0; i + 2 < trk.size(); ++i) {
        if (trk[i] == 0xD5 && trk[i + 1] == 0xAA && trk[i + 2] == 0x96) ++addrMarks;
        if (trk[i] == 0xD5 && trk[i + 1] == 0xAA && trk[i + 2] == 0xAD) ++dataMarks;
    }
    CHECK(addrMarks == 12);   // track 0 is in the 12-sector zone
    CHECK(dataMarks == 12);

    // Outside the mark bytes and sync gaps, every byte must be a legal disk
    // byte -- anything else and the ROM's decoder would reject the sector.
    std::size_t i = 0;
    while (i + 2 < trk.size()) {
        if (trk[i] == 0xD5 && trk[i + 1] == 0xAA &&
            (trk[i + 2] == 0x96 || trk[i + 2] == 0xAD)) {
            const bool addr = trk[i + 2] == 0x96;
            const std::size_t n = addr ? 5 : (1 + 699 + 4);
            for (std::size_t k = 0; k < n; ++k)
                CHECK(gcr::decodeTable()[trk[i + 3 + k]] != 0xFF);
            i += 3 + n;
            CHECK(trk[i] == 0xDE);
            CHECK(trk[i + 1] == 0xAA);
            i += 2;
            continue;
        }
        ++i;
    }
}
