#include <doctest/doctest.h>

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

TEST_CASE("sony: status lines answer the way the ROM's driver expects") {
    SpunUp s;
    // Active low throughout, so false means the condition is asserted.
    CHECK(s.drive.sense(0x1, s.now) == false);   // CSTIN: disk in place
    CHECK(s.drive.sense(0xE, s.now) == false);   // INSTALLED: drive present
    CHECK(s.drive.sense(0xF, s.now) == false);   // DRVIN: drive connected
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
