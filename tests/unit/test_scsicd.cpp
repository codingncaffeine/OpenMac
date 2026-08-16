#include <doctest/doctest.h>

#include "../../core/src/machine/cdmedia.hpp"
#include "../../core/src/machine/scsicd.hpp"

#include <openmac/quadra.hpp>

#include <cstring>
#include <string>
#include <vector>

using namespace openmac;

namespace {

// Run one CDB straight against the target (the 5380 routing is covered by
// test_scsi.cpp; these tests are about the drive's answers).
struct Cmd {
    u8 status;
    std::vector<u8> data;
    u32 writeBytes;
};

Cmd run(ScsiCdRom& cd, std::initializer_list<u8> bytes) {
    u8 cdb[12] = {};
    std::size_t i = 0;
    for (u8 b : bytes) cdb[i++] = b;
    Cmd r{};
    r.status = cd.execute(cdb, r.data, r.writeBytes);
    return r;
}

// Sense key / ASC as REQUEST SENSE reports them.
std::pair<u8, u8> sense(ScsiCdRom& cd) {
    auto r = run(cd, {0x03, 0, 0, 0, 18, 0});
    REQUIRE(r.data.size() >= 13);
    return {static_cast<u8>(r.data[2] & 0x0F), r.data[12]};
}

std::vector<u8> discOf(std::size_t sectors2048) {
    std::vector<u8> d(sectors2048 * 2048u);
    for (std::size_t i = 0; i < d.size(); ++i) d[i] = static_cast<u8>(i * 7);
    return d;
}

} // namespace

TEST_CASE("the drive answers INQUIRY as a removable SCSI-2 CD-ROM") {
    ScsiCdRom cd;
    cd.setAttached(true, 3);
    auto r = run(cd, {0x12, 0, 0, 0, 36, 0});
    REQUIRE(r.status == 0x00);
    REQUIRE(r.data.size() == 36);
    CHECK(r.data[0] == 0x05);   // CD-ROM
    CHECK(r.data[1] == 0x80);   // removable
    CHECK(std::string(r.data.begin() + 8, r.data.begin() + 16) == "SONY    ");
    CHECK(std::string(r.data.begin() + 16, r.data.begin() + 32) == "CD-ROM CDU-8003A");
    CHECK(cd.id() == 3);
    CHECK(cd.present());
    cd.setAttached(false, 3);
    CHECK_FALSE(cd.present());
}

TEST_CASE("discs appear through TEST UNIT READY the way the Apple driver polls") {
    ScsiCdRom cd;
    cd.setAttached(true, 3);

    // Empty tray: NOT READY, medium not present.
    CHECK(run(cd, {0x00, 0, 0, 0, 0, 0}).status == 0x02);
    auto [k1, a1] = sense(cd);
    CHECK(k1 == 0x02);
    CHECK(a1 == 0x3A);

    // Insertion: one UNIT ATTENTION (medium changed), then GOOD.
    cd.insert(discOf(4));
    CHECK(run(cd, {0x00, 0, 0, 0, 0, 0}).status == 0x02);
    auto [k2, a2] = sense(cd);
    CHECK(k2 == 0x06);
    CHECK(a2 == 0x28);
    CHECK(run(cd, {0x00, 0, 0, 0, 0, 0}).status == 0x00);
}

TEST_CASE("reads serve the flat image at the selected block size") {
    ScsiCdRom cd;
    cd.setAttached(true, 3);
    auto img = discOf(10);
    cd.insert(img);
    run(cd, {0x00, 0, 0, 0, 0, 0});   // absorb the unit attention

    // READ CAPACITY at the power-on 2048.
    auto cap = run(cd, {0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    REQUIRE(cap.data.size() == 8);
    const u32 lastLba = (u32(cap.data[0]) << 24) | (u32(cap.data[1]) << 16) |
                        (u32(cap.data[2]) << 8) | cap.data[3];
    const u32 bs = (u32(cap.data[4]) << 24) | (u32(cap.data[5]) << 16) |
                   (u32(cap.data[6]) << 8) | cap.data[7];
    CHECK(lastLba == 9);
    CHECK(bs == 2048);

    // READ(10) of sector 2 matches the image bytes.
    auto rd = run(cd, {0x28, 0, 0, 0, 0, 2, 0, 0, 1, 0});
    REQUIRE(rd.status == 0x00);
    REQUIRE(rd.data.size() == 2048);
    CHECK(std::memcmp(rd.data.data(), img.data() + 2 * 2048, 2048) == 0);

    // MODE SELECT down to 512-byte logical blocks (the HFS-era size)...
    auto ms = run(cd, {0x15, 0, 0, 0, 12, 0});
    CHECK(ms.status == 0x00);
    CHECK(ms.writeBytes == 12);
    const std::vector<u8> params = {0, 0, 0, 8,  0, 0, 0, 0,  0, 0x00, 0x02, 0x00};
    cd.acceptWrite(params);
    CHECK(cd.blockSize() == 512);

    // ...and the same bytes come back in 512-byte frames.
    auto rd512 = run(cd, {0x08, 0, 0, 5, 1, 0});   // LBA 5 at 512 = byte 2560
    REQUIRE(rd512.data.size() == 512);
    CHECK(std::memcmp(rd512.data.data(), img.data() + 5 * 512, 512) == 0);
    auto cap512 = run(cd, {0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    CHECK(cap512.data[3] == static_cast<u8>((10 * 2048 / 512) - 1));

    // Reading past the end is an ILLEGAL REQUEST, not a wedge.
    CHECK(run(cd, {0x28, 0, 0, 1, 0, 0, 0, 0, 1, 0}).status == 0x02);
    auto [k, a] = sense(cd);
    CHECK(k == 0x05);
    CHECK(a == 0x21);
}

TEST_CASE("READ TOC reports one data track and the lead-out, MSF and LBA") {
    ScsiCdRom cd;
    cd.setAttached(true, 3);
    cd.insert(discOf(100));
    run(cd, {0x00, 0, 0, 0, 0, 0});

    // MSF form: lead-out at 100 sectors + 150-frame pregap = 00:03:25.
    auto msf = run(cd, {0x43, 0x02, 0, 0, 0, 0, 0, 0, 40, 0});
    REQUIRE(msf.status == 0x00);
    REQUIRE(msf.data.size() == 20);
    CHECK(msf.data[2] == 1);           // first track
    CHECK(msf.data[3] == 1);           // last track
    CHECK(msf.data[6] == 1);           // track 1...
    CHECK(msf.data[5] == 0x14);        // ...a data track
    CHECK(msf.data[14] == 0xAA);       // lead-out
    CHECK(msf.data[17] == 0);          // minutes
    CHECK(msf.data[18] == 3);          // seconds
    CHECK(msf.data[19] == 25);         // frames

    // LBA form: lead-out at exactly 100.
    auto lba = run(cd, {0x43, 0x00, 0, 0, 0, 0, 0, 0, 40, 0});
    REQUIRE(lba.data.size() == 20);
    CHECK(lba.data[19] == 100);
}

TEST_CASE("the Finder's eject reaches the host and writes are refused") {
    ScsiCdRom cd;
    cd.setAttached(true, 3);
    cd.insert(discOf(4));
    run(cd, {0x00, 0, 0, 0, 0, 0});

    // WRITE(10): it is a CD-ROM.
    CHECK(run(cd, {0x2A, 0, 0, 0, 0, 0, 0, 0, 1, 0}).status == 0x02);
    auto [k, a] = sense(cd);
    CHECK(k == 0x07);
    CHECK(a == 0x27);

    // PREVENT MEDIUM REMOVAL holds the disc in against an eject.
    run(cd, {0x1E, 0, 0, 0, 1, 0});
    CHECK(run(cd, {0x1B, 0, 0, 0, 0x02, 0}).status == 0x02);
    CHECK(cd.discPresent());
    run(cd, {0x1E, 0, 0, 0, 0, 0});

    // START STOP UNIT with LoEj: disc leaves, the host hears about it once.
    CHECK(run(cd, {0x1B, 0, 0, 0, 0x02, 0}).status == 0x00);
    CHECK_FALSE(cd.discPresent());
    CHECK(cd.takeEjectRequest());
    CHECK_FALSE(cd.takeEjectRequest());
}

TEST_CASE("cd media containers normalize to the flat run the drive serves") {
    SUBCASE("ISO 9660") {
        std::vector<u8> f(20 * 2048u, 0);
        std::memcpy(f.data() + 16 * 2048 + 1, "CD001", 5);
        auto m = cd::normalize(f);
        REQUIRE(m.ok);
        CHECK(m.data.size() == f.size());
        CHECK(std::string(m.desc).find("ISO 9660") != std::string::npos);
    }
    SUBCASE("raw 2352 MODE1 extracts the user data") {
        std::vector<u8> f(3 * 2352u, 0);
        static const u8 sync[12] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
        for (int s = 0; s < 3; ++s) {
            std::memcpy(f.data() + s * 2352, sync, 12);
            f[s * 2352 + 15] = 1;   // MODE1
            for (int i = 0; i < 2048; ++i)
                f[s * 2352 + 16 + i] = static_cast<u8>(s + i);
        }
        auto m = cd::normalize(f);
        REQUIRE(m.ok);
        REQUIRE(m.data.size() == 3 * 2048u);
        CHECK(m.data[0] == 0);
        CHECK(m.data[2048] == 1);
        CHECK(m.data[2 * 2048 + 5] == 7);
    }
    SUBCASE("raw 2352 MODE2 takes the user data behind the XA subheader") {
        std::vector<u8> f(2 * 2352u, 0);
        static const u8 sync[12] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
        for (int s = 0; s < 2; ++s) {
            std::memcpy(f.data() + s * 2352, sync, 12);
            f[s * 2352 + 15] = 2;   // MODE2
            for (int i = 0; i < 2048; ++i)
                f[s * 2352 + 24 + i] = static_cast<u8>(0x40 + s + i);
        }
        auto m = cd::normalize(f);
        REQUIRE(m.ok);
        REQUIRE(m.data.size() == 2 * 2048u);
        CHECK(m.data[0] == 0x40);
        CHECK(m.data[2048 + 3] == 0x44);
        CHECK(std::string(m.desc).find("MODE2") != std::string::npos);
    }
    SUBCASE("raw 2448 (subchannel behind every sector) is framed by its stride") {
        std::vector<u8> f(2 * 2448u, 0);
        static const u8 sync[12] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
        for (int s = 0; s < 2; ++s) {
            std::memcpy(f.data() + s * 2448, sync, 12);
            f[s * 2448 + 15] = 1;
            f[s * 2448 + 16] = static_cast<u8>(0x70 + s);
        }
        auto m = cd::normalize(f);
        REQUIRE(m.ok);
        REQUIRE(m.data.size() == 2 * 2048u);
        CHECK(m.data[0] == 0x70);
        CHECK(m.data[2048] == 0x71);
        CHECK(std::string(m.desc).find("2448") != std::string::npos);
    }
    SUBCASE("MODE0 raw sectors are refused with their nature named") {
        std::vector<u8> f(2352, 0);
        static const u8 sync[12] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
        std::memcpy(f.data(), sync, 12);
        f[15] = 0;
        auto m = cd::normalize(f);
        CHECK_FALSE(m.ok);
        CHECK(std::string(m.desc).find("MODE0") != std::string::npos);
    }
    SUBCASE("MODE2/2336 (no sync) is found by the layout behind the subheader") {
        std::vector<u8> f(20 * 2336u, 0);
        std::memcpy(f.data() + 16 * 2336 + 8 + 1, "CD001", 5);
        f[8] = 0x11;                                     // user byte 0 of sector 0
        auto m = cd::normalize(f);
        REQUIRE(m.ok);
        REQUIRE(m.data.size() == 20 * 2048u);
        CHECK(m.data[0] == 0x11);
        CHECK(std::memcmp(m.data.data() + 16 * 2048 + 1, "CD001", 5) == 0);
        CHECK(std::string(m.desc).find("2336") != std::string::npos);
    }
    SUBCASE("HFS Plus is refused by name") {
        std::vector<u8> f(4 * 512u, 0);
        f[1024] = 'H'; f[1025] = '+';
        auto m = cd::normalize(f);
        CHECK_FALSE(m.ok);
        CHECK(std::string(m.desc).find("HFS Plus") != std::string::npos);
    }
    SUBCASE("Apple-partitioned and bare HFS masters pass through flat") {
        std::vector<u8> apm(4 * 512u, 0);
        apm[0] = 0x45; apm[1] = 0x52;
        CHECK(cd::normalize(apm).ok);

        std::vector<u8> hfs(4 * 512u, 0);
        hfs[1024] = 0x42; hfs[1025] = 0x44;
        CHECK(cd::normalize(hfs).ok);
    }
    SUBCASE("something else is refused") {
        CHECK_FALSE(cd::normalize(std::vector<u8>(4096, 0xAB)).ok);
    }
}


// ---- UDIF (.dmg) --------------------------------------------------------
namespace {

void be32at(std::vector<u8>& v, std::size_t at, u32 x) {
    v[at] = u8(x >> 24); v[at + 1] = u8(x >> 16); v[at + 2] = u8(x >> 8); v[at + 3] = u8(x);
}
void be64at(std::vector<u8>& v, std::size_t at, u64 x) {
    be32at(v, at, static_cast<u32>(x >> 32));
    be32at(v, at + 4, static_cast<u32>(x));
}
std::string base64(const std::vector<u8>& in) {
    static const char* t = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    std::size_t i = 0;
    for (; i + 2 < in.size(); i += 3) {
        const u32 v = (u32(in[i]) << 16) | (u32(in[i + 1]) << 8) | in[i + 2];
        out += t[(v >> 18) & 63]; out += t[(v >> 12) & 63];
        out += t[(v >> 6) & 63]; out += t[v & 63];
    }
    if (i + 1 == in.size()) {
        const u32 v = u32(in[i]) << 16;
        out += t[(v >> 18) & 63]; out += t[(v >> 12) & 63]; out += "==";
    } else if (i + 2 == in.size()) {
        const u32 v = (u32(in[i]) << 16) | (u32(in[i + 1]) << 8);
        out += t[(v >> 18) & 63]; out += t[(v >> 12) & 63]; out += t[(v >> 6) & 63]; out += '=';
    }
    return out;
}

// A zlib stream produced by an INDEPENDENT DEFLATE implementation (.NET's
// DeflateStream), decompressing to 512 bytes of
// "OpenPowerMac reads discs. " x19 + "OpenPowerMac reads".
const u8 kZlib512[] = {
    0x78, 0x9c, 0xf3, 0x2f, 0x48, 0xcd, 0x0b, 0xc8, 0x2f, 0x4f, 0x2d,
    0xf2, 0x4d, 0x4c, 0x56, 0x28, 0x4a, 0x4d, 0x4c, 0x29, 0x56, 0x48,
    0xc9, 0x2c, 0x4e, 0x2e, 0xd6, 0x53, 0xf0, 0x1f, 0x95, 0x51, 0x18,
    0xfe, 0x61, 0x00, 0x00, 0xaf, 0x71, 0xb9, 0x39};

std::vector<u8> zlibPlain512() {
    std::string t;
    for (int k = 0; k < 19; ++k) t += "OpenPowerMac reads discs. ";
    t += "OpenPowerMac reads";
    return std::vector<u8>(t.begin(), t.end());
}

// A UDIF whose device is: sector 0 = the zlib text, sector 1 = a raw
// pattern, sector 2 = a zero chunk (its bytes 0..1 are where an HFS master's
// 'BD' lives -- put there through a raw chunk instead when wanted), then a
// hole to the koly sector count.
std::vector<u8> buildUdif(bool hfsMaster, u32 zlibType = 0x80000005u) {
    std::vector<u8> fork(kZlib512, kZlib512 + sizeof kZlib512);
    std::vector<u8> rawSec(512, 0);
    for (u32 j = 0; j < 512; ++j) rawSec[j] = static_cast<u8>(j * 5 + 9);
    fork.insert(fork.end(), rawSec.begin(), rawSec.end());
    std::vector<u8> mdb(512, 0);
    if (hfsMaster) { mdb[0] = 'B'; mdb[1] = 'D'; }
    fork.insert(fork.end(), mdb.begin(), mdb.end());

    std::vector<u8> mish(204 + 5 * 40, 0);
    be32at(mish, 0, 0x6D697368u);          // 'mish'
    be32at(mish, 4, 1);
    be64at(mish, 8, 0);                    // first sector of this table
    be64at(mish, 16, 3);                   // sectors the chunks below cover
    be64at(mish, 24, 0);                   // data offset
    be32at(mish, 200, 5);
    auto chunk = [&](u32 idx, u32 type, u64 sec, u64 cnt, u64 off, u64 len) {
        const std::size_t at = 204 + idx * 40;
        be32at(mish, at, type);
        be64at(mish, at + 8, sec);
        be64at(mish, at + 16, cnt);
        be64at(mish, at + 24, off);
        be64at(mish, at + 32, len);
    };
    chunk(0, 0x7FFFFFFEu, 0, 0, 0, 0);                             // comment
    chunk(1, zlibType, 0, 1, 0, sizeof kZlib512);                  // zlib (or a refused codec)
    chunk(2, 0x00000001u, 1, 1, sizeof kZlib512, 512);             // raw
    chunk(3, hfsMaster ? 0x00000001u : 0x00000000u, 2, 1,
          sizeof kZlib512 + 512, hfsMaster ? 512 : 0);             // raw MDB / zero
    chunk(4, 0xFFFFFFFFu, 3, 0, 0, 0);                             // terminator

    const std::string xml =
        "<?xml version=\"1.0\"?><plist><dict><key>resource-fork</key>"
        "<dict><key>blkx</key><array><dict><key>Data</key><data>\n" +
        base64(mish) + "\n</data></dict></array></dict></dict></plist>";

    std::vector<u8> file(fork);
    file.insert(file.end(), xml.begin(), xml.end());
    std::vector<u8> koly(512, 0);
    std::memcpy(koly.data(), "koly", 4);
    be32at(koly, 4, 4);                    // version
    be32at(koly, 8, 512);                  // header size
    be64at(koly, 24, 0);                   // data fork offset
    be64at(koly, 32, fork.size());
    be64at(koly, 216, fork.size());        // XML offset
    be64at(koly, 224, xml.size());
    be64at(koly, 492, 4);                  // device sectors, hole included
    file.insert(file.end(), koly.begin(), koly.end());
    return file;
}

} // namespace

TEST_CASE("zlib inflate: an independent compressor's stream comes back exact") {
    std::vector<u8> out;
    REQUIRE(cd::zlibInflate(kZlib512, sizeof kZlib512, out, 512));
    CHECK(out == zlibPlain512());
    // The 1-byte classic: zlib.compress(b"a") -- fixed Huffman.
    static const u8 kA[] = {0x78, 0x9c, 0x4b, 0x04, 0x00, 0x00, 0x62, 0x00, 0x62};
    REQUIRE(cd::zlibInflate(kA, sizeof kA, out, 16));
    REQUIRE(out.size() == 1);
    CHECK(out[0] == 'a');
    // A stored block, hand-made: header, BFINAL+BTYPE=00, LEN/NLEN, bytes, adler32.
    static const u8 kStored[] = {0x78, 0x01, 0x01, 0x03, 0x00, 0xFC, 0xFF, 'h', 'i', '!',
                                 0x02, 0x2E, 0x00, 0xF3};
    REQUIRE(cd::zlibInflate(kStored, sizeof kStored, out, 16));
    CHECK(std::string(out.begin(), out.end()) == "hi!");
    // A wrong checksum is refused: a trusted defect makes silently wrong sectors.
    u8 bad[sizeof kA]; std::memcpy(bad, kA, sizeof kA); bad[sizeof kA - 1] ^= 1;
    CHECK_FALSE(cd::zlibInflate(bad, sizeof bad, out, 16));
}

TEST_CASE("UDIF dmg: zlib, raw, zero and hole chunks land, and the disc inside is seen") {
    const std::vector<u8> file = buildUdif(false);
    REQUIRE(cd::isUdif(file));
    std::vector<u8> device;
    std::string why;
    REQUIRE_MESSAGE(cd::udifDecode(file, device, why), why);
    REQUIRE(device.size() == 4 * 512u);
    CHECK(std::memcmp(device.data(), zlibPlain512().data(), 512) == 0);
    for (u32 j = 0; j < 512; ++j) REQUIRE(device[512 + j] == static_cast<u8>(j * 5 + 9));
    for (u32 j = 1024; j < 2048; ++j) REQUIRE(device[j] == 0);
    // Through normalize: the text is not a disc, so it is refused -- but with
    // the UDIF named, so the person knows the container was understood.
    auto m = cd::normalize(file);
    CHECK_FALSE(m.ok);
    CHECK(std::string(m.desc).find("UDIF") != std::string::npos);
    // With an HFS master inside, the drive takes it and serves the device.
    const std::vector<u8> hfsDmg = buildUdif(true);
    auto h = cd::normalize(hfsDmg);
    REQUIRE_MESSAGE(h.ok, h.desc);
    CHECK(h.data.size() == 4 * 512u);
    CHECK(h.data[1024] == 'B');
    CHECK(std::string(h.desc).find("UDIF") != std::string::npos);
    CHECK(std::string(h.desc).find("HFS CD master") != std::string::npos);
    // A codec this decoder does not carry is refused by name.
    auto b = cd::normalize(buildUdif(true, 0x80000006u));
    CHECK_FALSE(b.ok);
    CHECK(std::string(b.desc).find("bzip2") != std::string::npos);
}

// A drive that answers every page code with page 01 is telling the same lie an
// echoing register stub tells: the initiator gets a well-formed reply that
// describes something else, and nothing is logged because the command
// "succeeded". Report the page that was asked for, or say it is not there.
TEST_CASE("MODE SENSE serves the page it was asked for, and refuses the rest") {
    ScsiCdRom cd;
    cd.setAttached(true, 3);
    cd.insert(std::vector<u8>(4 * 2048, 0x11));
    run(cd, {0x00});                      // clear the insertion unit attention

    // Page 01, read error recovery: header, block descriptor, then the page.
    Cmd p1 = run(cd, {0x1A, 0x00, 0x01, 0x00, 0xFF, 0x00});
    CHECK(p1.status == 0x00);
    REQUIRE(p1.data.size() >= 14);
    CHECK(p1.data[3] == 8);               // block descriptor length
    CHECK(p1.data[12] == 0x01);           // the page code that was asked for
    CHECK(p1.data[0] == p1.data.size() - 1);   // mode data length excludes itself
    // The block descriptor still carries the block size the drive is set to.
    const u32 bs = (u32(p1.data[9]) << 16) | (u32(p1.data[10]) << 8) | p1.data[11];
    CHECK(bs == cd.blockSize());

    // Page 2A, CD capabilities -- a page a CD driver actually asks for.
    Cmd p2a = run(cd, {0x1A, 0x00, 0x2A, 0x00, 0xFF, 0x00});
    CHECK(p2a.status == 0x00);
    REQUIRE(p2a.data.size() >= 14);
    CHECK(p2a.data[12] == 0x2A);

    // Page 3F is "all of them", so both show up in one answer.
    Cmd all = run(cd, {0x1A, 0x00, 0x3F, 0x00, 0xFF, 0x00});
    CHECK(all.status == 0x00);
    CHECK(all.data.size() > p1.data.size());
    CHECK(all.data[12] == 0x01);

    // And a page this drive does not have is refused, not answered with
    // something else.
    Cmd bad = run(cd, {0x1A, 0x00, 0x10, 0x00, 0xFF, 0x00});
    CHECK(bad.status == 0x02);            // CHECK CONDITION
    Cmd sense = run(cd, {0x03, 0x00, 0x00, 0x00, 0x12, 0x00});
    REQUIRE(sense.data.size() >= 13);
    CHECK((sense.data[2] & 0x0F) == 0x05);   // ILLEGAL REQUEST
    CHECK(sense.data[12] == 0x24);           // invalid field in CDB
}

// A Mac CD master puts an Apple partition map at the front and the volume
// inside an "Apple_HFS" partition -- and the map's entries are addressed in
// 512-byte blocks even when the disc's sectors are 2048, which is the detail
// that decides whether the driver reads a volume or reads the map again.
TEST_CASE("the HFS partition is found where an Apple CD master puts it") {
    // A bare HFS master starts at zero: MDB two blocks in, nothing in front.
    std::vector<u8> bare(64 * 1024, 0);
    bare[1024] = 'B'; bare[1025] = 'D';
    CHECK(QuadraMachine::findHfsPartition(bare) == 0);

    // A partitioned master: DDR at block 0, map entries from block 1, and the
    // Apple_HFS entry naming a start in 512-byte blocks.
    std::vector<u8> apm(256 * 1024, 0);
    apm[0] = 'E'; apm[1] = 'R';
    auto entry = [&](std::size_t blk, const char* type, u32 start) {
        u8* p = apm.data() + blk * 512;
        p[0] = 'P'; p[1] = 'M';
        p[8] = u8(start >> 24); p[9] = u8(start >> 16);
        p[10] = u8(start >> 8); p[11] = u8(start);
        std::memcpy(p + 48, type, std::strlen(type) + 1);
    };
    entry(1, "Apple_partition_map", 1);
    entry(2, "Apple_Driver", 64);
    entry(3, "Apple_HFS", 128);
    CHECK(QuadraMachine::findHfsPartition(apm) == 128u * 512u);

    // A disc with a map but no HFS in it reads as "starts at zero", which is
    // what an ISO 9660 disc is: the mount then fails, rather than reading the
    // partition map as if it were a volume.
    std::vector<u8> isoOnly(256 * 1024, 0);
    isoOnly[0] = 'E'; isoOnly[1] = 'R';
    CHECK(QuadraMachine::findHfsPartition(isoOnly) == 0);
}

// The same finder serves both machines from cdmedia.hpp, and hasHfsVolume is
// the question the IIfx asks before it promises the user a mount: a partition
// map with an Apple_HFS entry whose volume actually carries the MDB signature,
// or a bare master -- but not an ISO-only disc, whose byte 1024 is nothing.
TEST_CASE("cd::findHfsPartition and hasHfsVolume agree with the Quadra's finder") {
    std::vector<u8> apm(256 * 1024, 0);
    apm[0] = 'E'; apm[1] = 'R';
    auto entry = [&](std::size_t blk, const char* type, u32 start) {
        u8* p = apm.data() + blk * 512;
        p[0] = 'P'; p[1] = 'M';
        p[8] = u8(start >> 24); p[9] = u8(start >> 16);
        p[10] = u8(start >> 8); p[11] = u8(start);
        std::memcpy(p + 48, type, std::strlen(type) + 1);
    };
    entry(1, "Apple_partition_map", 1);
    entry(2, "Apple_HFS", 128);
    CHECK(cd::findHfsPartition(apm) == QuadraMachine::findHfsPartition(apm));
    CHECK(cd::findHfsPartition(apm) == 128u * 512u);
    CHECK_FALSE(cd::hasHfsVolume(apm));            // the partition is empty
    apm[128 * 512 + 1024] = 'B'; apm[128 * 512 + 1025] = 'D';
    CHECK(cd::hasHfsVolume(apm));

    std::vector<u8> bare(64 * 1024, 0);
    CHECK_FALSE(cd::hasHfsVolume(bare));
    bare[1024] = 'B'; bare[1025] = 'D';
    CHECK(cd::hasHfsVolume(bare));

    std::vector<u8> tiny(600, 0);                  // shorter than the MDB
    CHECK_FALSE(cd::hasHfsVolume(tiny));
}
