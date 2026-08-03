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
    SUBCASE("MODE2 is refused with its nature named") {
        std::vector<u8> f(2352, 0);
        static const u8 sync[12] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
        std::memcpy(f.data(), sync, 12);
        f[15] = 2;
        auto m = cd::normalize(f);
        CHECK_FALSE(m.ok);
        CHECK(std::string(m.desc).find("MODE2") != std::string::npos);
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
