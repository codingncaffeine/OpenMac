#include <doctest/doctest.h>

#include "../../core/src/machine/iwm.hpp"
#include "../../core/src/machine/mfm.hpp"
#include "../../core/src/machine/sony.hpp"

#include <vector>

using namespace openmac;

namespace {

// A drive with a disk in it, spun up, and a 64-byte track under the head whose
// bytes are self-identifying ($80 | index).
struct SpunUp {
    SonyDrive drive;
    std::vector<u8> media{std::vector<u8>(1024, 0)};
    std::vector<u8> track{std::vector<u8>(64)};
    u64 now = 0;

    SpunUp() {
        drive.installed = true;
        drive.image = &media;
        for (std::size_t i = 0; i < track.size(); ++i)
            track[i] = static_cast<u8>(0x80 | i);
        drive.setTrackData(track);
        drive.command(0x4, false, now);   // MOTORON, data 0 = motor on
        now += 4000000;                   // past the 400 ms spin-up
    }
};

} // namespace

TEST_CASE("swim: handshake keeps RDDATA and SENSE independent") {
    Iwm swim;
    swim.reset();
    swim.forceIsm();

    // Port 15 is the ISM Handshake register. CRC is nonzero after reset, so
    // bit 1 remains set alongside the two independently sampled input pins.
    swim.readDataHigh = true;
    swim.senseHigh = false;
    CHECK((swim.access(15, false, 0) & 0x0E) == 0x06);

    swim.readDataHigh = false;
    swim.senseHigh = true;
    CHECK((swim.access(15, false, 0) & 0x0E) == 0x0A);

    swim.readDataHigh = true;
    CHECK((swim.access(15, false, 0) & 0x0E) == 0x0E);
}

TEST_CASE("swim: read FIFO preserves two bytes and reports exact occupancy") {
    Iwm swim;
    swim.reset();
    swim.forceIsm();

    REQUIRE(swim.ismPushReadByte(0xA1, true));
    CHECK(swim.ismFifoOccupancy() == 1);
    CHECK((swim.access(15, false, 0) & 0xC1u) == 0x81u);

    REQUIRE(swim.ismPushReadByte(0xFE, false));
    CHECK(swim.ismFifoOccupancy() == 2);
    CHECK((swim.access(15, false, 0) & 0xC1u) == 0xC1u);
    swim.access(7, true, 0x08); // ACTION: register reads now address the FIFO

    // Mark register consumes the marked head without setting Mark-in-Data.
    CHECK(swim.access(9, false, 0) == 0xA1);
    CHECK(swim.ismFifoOccupancy() == 1);
    CHECK((swim.access(15, false, 0) & 0xC1u) == 0x80u);
    CHECK(swim.access(8, false, 0) == 0xFE);
    CHECK(swim.ismFifoOccupancy() == 0);
    CHECK((swim.access(15, false, 0) & 0xC0u) == 0);
}

TEST_CASE("swim: CRC handshake follows the newest byte in the read FIFO") {
    Iwm swim;
    swim.reset();
    swim.forceIsm();

    const u8 addressField[8] = {0xA1, 0xA1, 0xA1, 0xFE, 0, 0, 1, 2};
    const u16 fieldCrc = mfm::crc16(addressField, sizeof addressField);
    swim.ismCrcReset();
    for (const u8 byte : addressField) swim.ismCrcAdd(byte);

    const u8 crcHigh = static_cast<u8>(fieldCrc >> 8);
    const u8 crcLow = static_cast<u8>(fieldCrc);
    swim.ismCrcAdd(crcHigh);
    REQUIRE(swim.ismPushReadByte(crcHigh, false));
    REQUIRE(swim.diagnosticFifoHeadCrc() != 0);
    swim.ismCrcAdd(crcLow);
    REQUIRE(swim.ismPushReadByte(crcLow, false));

    CHECK(swim.ismFifoHeadData() == crcHigh);
    CHECK(swim.ismFifoTailData() == crcLow);
    CHECK(swim.diagnosticFifoHeadCrc() != 0);
    CHECK(swim.diagnosticCrc() == 0);
    CHECK((swim.diagnosticHandshake() & 0x02u) == 0);
}

TEST_CASE("swim: illegal FIFO reads set the documented error bits") {
    Iwm swim;
    swim.reset();
    swim.forceIsm();
    REQUIRE(swim.ismPushReadByte(0xA1, true));
    swim.access(7, true, 0x08);

    CHECK(swim.access(8, false, 0) == 0xA1); // mark through Data
    CHECK(swim.access(10, false, 0) == 0x02);
    CHECK(swim.access(10, false, 0) == 0x00); // Error read clears it

    static_cast<void>(swim.access(8, false, 0)); // faster than available
    CHECK(swim.access(10, false, 0) == 0x04);
}

TEST_CASE("sony: a fast reader gets every byte of the track, in order") {
    SpunUp s;
    REQUIRE(s.drive.motorRunning(s.now));

    // Sync to wherever the surface happens to be, then poll far faster than the
    // byte rate, the way the ROM's read loop does (it polls roughly every 40
    // cycles against a 128-cycle byte time).
    const u8 first = s.drive.readNibble(s.now);
    REQUIRE(first != 0);
    std::vector<u8> got{first};
    for (int guard = 0; guard < 200000 && got.size() < 200; ++guard) {
        s.now += 4;
        if (const u8 b = s.drive.readNibble(s.now)) got.push_back(b);
    }
    REQUIRE(got.size() == 200);

    // Every byte, no skips, wrapping at the end of the track. Handing out every
    // other byte still looks like "data is flowing" but no address mark can ever
    // match, which is exactly how it hid before.
    const std::size_t start = static_cast<std::size_t>(first & 0x3F);
    for (std::size_t i = 0; i < got.size(); ++i)
        CHECK(got[i] == s.track[(start + i) % s.track.size()]);
}

TEST_CASE("sony: one byte per byte time, no more") {
    SpunUp s;
    s.drive.readNibble(s.now);            // sync
    // Exactly one byte time later there is exactly one new byte, and no byte
    // before it.
    for (int i = 0; i < 8; ++i) {
        CHECK(s.drive.readNibble(s.now + SonyDrive::kCyclesPerByte - 1) == 0);
        s.now += SonyDrive::kCyclesPerByte;
        CHECK(s.drive.readNibble(s.now) != 0);
    }
}

TEST_CASE("sony: a slow reader skips bytes but never repeats or reorders") {
    SpunUp s;
    const u8 first = s.drive.readNibble(s.now);
    REQUIRE(first != 0);
    // Poll once every 10 byte times: the surface keeps turning, so each byte we
    // do get must be 10 further along.
    std::size_t expect = static_cast<std::size_t>(first & 0x3F);
    for (int i = 0; i < 20; ++i) {
        s.now += SonyDrive::kCyclesPerByte * 10;
        expect = (expect + 10) % s.track.size();
        CHECK(s.drive.readNibble(s.now) == s.track[expect]);
    }
}

TEST_CASE("sony: a stopped motor delivers nothing") {
    SpunUp s;
    s.drive.command(0x4, true, s.now);    // MOTORON, data 1 = motor off
    for (int i = 0; i < 8; ++i) {
        s.now += SonyDrive::kCyclesPerByte * 4;
        CHECK(s.drive.readNibble(s.now) == 0);
    }
    // ...and when it spins up again the stream resumes cleanly.
    s.drive.command(0x4, false, s.now);
    s.now += 4000000;
    CHECK(s.drive.readNibble(s.now) != 0);
}

TEST_CASE("sony: the write head takes one byte per byte time") {
    SpunUp s;
    s.drive.readNibble(s.now);                    // sync to the surface
    // The driver polls the handshake until the buffer empties, so readiness has
    // to follow the same byte clock the read side does.
    CHECK(s.drive.writeReady(s.now + SonyDrive::kCyclesPerByte - 1) == false);
    CHECK(s.drive.writeReady(s.now + SonyDrive::kCyclesPerByte) == true);

    // Bytes land under the head and the head moves on, so a run of writes ends
    // up contiguous on the track.
    const std::size_t start = s.drive.bytePos();
    for (int i = 0; i < 16; ++i) {
        s.now += SonyDrive::kCyclesPerByte;
        REQUIRE(s.drive.writeReady(s.now));
        s.drive.writeNibble(s.now, static_cast<u8>(0xC0 | i));
    }
    CHECK(s.drive.trackDirty());
    for (int i = 0; i < 16; ++i)
        CHECK(s.drive.trackData()[(start + static_cast<std::size_t>(i)) % s.track.size()] ==
              static_cast<u8>(0xC0 | i));
}

TEST_CASE("sony: a locked disk takes no writes at all") {
    SpunUp s;
    s.drive.readOnly = true;
    const std::vector<u8> before = s.drive.trackData();
    for (int i = 0; i < 8; ++i) {
        s.now += SonyDrive::kCyclesPerByte;
        CHECK(s.drive.writeReady(s.now) == false);
        s.drive.writeNibble(s.now, 0xFF);
    }
    CHECK(s.drive.trackData() == before);
    CHECK(s.drive.trackDirty() == false);
    CHECK(s.drive.sense(0x3, s.now) == false);    // WRTPRT: 0 = write protected
}

TEST_CASE("sony: an eject is a mechanism, not a strobe width") {
    SpunUp s;
    s.drive.command(0x6, true, s.now);            // EJECT: the driver's pulse is short
    CHECK(s.drive.takeEjectRequest() == false);   // ...and nothing happens yet
    s.now += 7833600ull * 700 / 1000;             // 700 ms: still on its way out
    s.drive.tickEject(s.now);
    CHECK(s.drive.takeEjectRequest() == false);
    s.now += 7833600ull * 100 / 1000;             // past 750 ms
    s.drive.tickEject(s.now);
    CHECK(s.drive.takeEjectRequest() == true);

    // A cancelled eject stays cancelled, however long the machine runs after.
    SpunUp t;
    t.drive.command(0x6, true, t.now);
    t.drive.command(0x6, false, t.now + 1000);
    t.now += 7833600ull * 5;                      // five seconds later
    t.drive.tickEject(t.now);
    CHECK(t.drive.takeEjectRequest() == false);
}

TEST_CASE("sony: status lines answer the way the ROM's driver expects") {
    SpunUp s;
    // Active low throughout, so false means the condition is asserted.
    CHECK(s.drive.sense(0x1, s.now) == false);   // CSTIN: disk in place
    CHECK(s.drive.sense(0xE, s.now) == false);   // INSTALLED: drive present
    // $F is the high-density aperture, not DRVIN: low only when the medium is
    // high density. Answering it from the drive rather than the disk makes a
    // SWIM-mode driver refuse an 800K disk with gcrOnMFMErr.
    CHECK(s.drive.sense(0xF, s.now) == true);    // an 800K disk is not HD
    s.drive.hdMedia = true;
    CHECK(s.drive.sense(0xF, s.now) == false);   // a 1.4MB disk is
    s.drive.hdMedia = false;
    CHECK(s.drive.sense(0xC, s.now) == true);    // SIDES: high = double sided
    CHECK(s.drive.sense(0x5, s.now) == false);   // TK0: at track 0
    CHECK(s.drive.sense(0x4, s.now) == false);   // MOTORON: motor running

    // An unconnected drive floats every line high; that is what tells the ROM's
    // drive scan to move on rather than wait on a drive that cannot answer.
    SonyDrive absent;
    for (int a = 0; a < 16; ++a) CHECK(absent.sense(a, 0) == true);
}

TEST_CASE("sony: stepping walks the head and tracks the direction bit") {
    SpunUp s;
    CHECK(s.drive.track == 0);
    s.drive.command(0x0, false, s.now);          // DIRTN: data 0 = toward track 79
    for (int i = 0; i < 5; ++i) {
        s.drive.command(0x2, false, s.now);      // STEP: data 0 = step
        s.now += SonyDrive::kCyclesPerByte * 1000;
    }
    CHECK(s.drive.track == 5);
    s.drive.command(0x0, true, s.now);           // DIRTN: data 1 = toward track 0
    for (int i = 0; i < 3; ++i) {
        s.drive.command(0x2, false, s.now);
        s.now += SonyDrive::kCyclesPerByte * 1000;
    }
    CHECK(s.drive.track == 2);
    // The head cannot walk off either end of the disk.
    for (int i = 0; i < 100; ++i) s.drive.command(0x2, false, s.now);
    CHECK(s.drive.track == 0);
}

TEST_CASE("sony: STEP handshake releases after 80 microseconds") {
    SpunUp s;
    s.drive.command(0x0, false, s.now);          // toward track 79
    s.drive.command(0x2, false, s.now);

    CHECK(s.drive.track == 1);
    CHECK(s.drive.sense(0x2, s.now) == false);  // /STEP asserted/busy
    CHECK(s.drive.sense(0x2,
        s.now + 7833600ull * 79u / 1000000u) == false);
    CHECK(s.drive.sense(0x2,
        s.now + 7833600ull * 80u / 1000000u) == true);
}

TEST_CASE("sony: the disk-switched line is high until the driver acknowledges it") {
    SpunUp s;
    std::vector<u8> other(1024, 0);

    // Seating a disk latches the line. It is active HIGH, unlike every other
    // status line on the drive: the ROM's per-VBL drive poll turns it into a
    // flag with SNE ($435C98) and announces a disk insertion when it reads high.
    s.drive.insert(&s.media, false, false);
    CHECK(s.drive.sense(0x6, s.now) == true);

    // Reading it does not clear it. If it did, the line would answer "a disk was
    // just swapped in" for as long as the disk sat in the drive, and the ROM
    // would post a disk-inserted event for a volume that is already mounted --
    // which the Finder answers by flushing that volume and ejecting it.
    CHECK(s.drive.sense(0x6, s.now) == true);
    CHECK(s.drive.sense(0x6, s.now) == true);

    // Control register 001 with data 1 is the acknowledgement ($435866-$435868,
    // issued only after the line has read high), and it is what clears it.
    s.drive.command(0x1, true, s.now);
    CHECK(s.drive.sense(0x6, s.now) == false);

    // ...and it stays clear, so an idle drive never announces anything.
    s.now += SonyDrive::kCyclesPerByte * 100000;
    CHECK(s.drive.sense(0x6, s.now) == false);

    // A real disk change latches it again, which is how an insertion into a
    // drive that already held one gets noticed at all.
    s.drive.insert(&other, false, false);
    CHECK(s.drive.sense(0x6, s.now) == true);
    s.drive.command(0x1, true, s.now);
    CHECK(s.drive.sense(0x6, s.now) == false);

    // So does taking one out.
    s.drive.removeDisk();
    CHECK(s.drive.sense(0x6, s.now) == true);
}

TEST_CASE("sony: acknowledging a disk switch does not touch the motor") {
    SpunUp s;
    REQUIRE(s.drive.motorRunning(s.now));
    s.drive.command(0x4, true, s.now);           // MOTORON, data 1 = motor off
    CHECK(s.drive.motorRunning(s.now) == false);

    // Register 001 was read as a motor enable for a while, because the drive
    // poll strobes it constantly -- which it only did because the latch it
    // acknowledges never cleared. Spinning the disk up here would start the
    // motor every time the poll ran.
    s.drive.command(0x1, true, s.now);
    CHECK(s.drive.motorRunning(s.now) == false);
    s.now += SonyDrive::kCyclesPerByte * 100000;
    CHECK(s.drive.motorRunning(s.now) == false);
}
