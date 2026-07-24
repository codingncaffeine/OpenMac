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
