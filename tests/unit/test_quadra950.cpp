#include <doctest/doctest.h>

#include "../../core/src/machine/quadra950/memctl.hpp"

using namespace openmac;

namespace {

constexpr u32 MB = 1024 * 1024;

// The ROM's way of loading the controller: the value goes to the first
// register, is shifted right one place, goes to the next, and so on.
void load(MemoryController950& mc, u32 firstOffset, u32 value, int count) {
    for (int i = 0; i < count; ++i) {
        mc.write(firstOffset + static_cast<u32>(i) * 4, value);
        value >>= 1;
    }
}

// What the ROM writes once it has sized the banks: one boundary at a time,
// six registers each, the running end of RAM in units of 4 MB.
void setBoundaries(MemoryController950& mc, u32 endA, u32 endB, u32 endC) {
    load(mc, 0x00, endA / (4 * MB), 6);
    load(mc, 0x18, endB / (4 * MB), 6);
    load(mc, 0x30, endC / (4 * MB), 6);
}

} // namespace

TEST_CASE("950 memory controller: registers are one bit wide, loaded by shifting") {
    MemoryController950 mc({4, 4, 0, 0});
    load(mc, 0x00, 0x138B0810u, 32);                  // the ROM's reset value
    CHECK(mc.boundaryBytes(0) == 64 * MB);
    CHECK(mc.boundaryBytes(1) == 128 * MB);
    CHECK(mc.boundaryBytes(2) == 192 * MB);
    CHECK(mc.read(0x10) == 1);                        // bit 4 of the value
    CHECK(mc.read(0x00) == 0);
    CHECK(mc.read(0x48) == 0);                        // bit 18, the one the ROM reads
    CHECK(mc.read(0x4C) == 1);                        // bit 19
}

TEST_CASE("950 memory controller: a bank repeats at its own size inside its range") {
    MemoryController950 mc({4, 16, 0, 0});
    load(mc, 0x00, 0x138B0810u, 32);                  // four 64 MB ranges
    // Bank A, 4 MB, at $0000 0000: the top of its range reads the same cell
    // as the top of its first 4 MB.
    CHECK(mc.translate(0x00000000u) == 0);
    CHECK(mc.translate(64 * MB - 4) == 4 * MB - 4);
    CHECK(mc.translate(4 * MB) == 0);
    // Bank B, 16 MB, at $0400 0000, stored after bank A in the flat array.
    CHECK(mc.translate(64 * MB) == 4 * MB);
    CHECK(mc.translate(64 * MB + 16 * MB) == 4 * MB);
    CHECK(mc.translate(128 * MB - 4) == 4 * MB + 16 * MB - 4);
    // Banks C and D are empty.
    CHECK(mc.translate(128 * MB) == MemoryController950::kNoRam);
    CHECK(mc.translate(192 * MB) == MemoryController950::kNoRam);
    CHECK_FALSE(mc.identityMapped());
}

TEST_CASE("950 memory controller: the settled boundaries put RAM back to back") {
    SUBCASE("two 4 MB banks") {
        MemoryController950 mc({4, 4, 0, 0});
        setBoundaries(mc, 4 * MB, 8 * MB, 8 * MB);
        CHECK(mc.flatBytes() == 8 * MB);
        CHECK(mc.translate(0) == 0);
        CHECK(mc.translate(4 * MB) == 4 * MB);
        CHECK(mc.translate(8 * MB - 1) == 8 * MB - 1);
        CHECK(mc.translate(8 * MB) == MemoryController950::kNoRam);   // bank D, empty
        CHECK(mc.identityMapped());
    }
    SUBCASE("four 64 MB banks") {
        MemoryController950 mc({64, 64, 64, 64});
        setBoundaries(mc, 64 * MB, 128 * MB, 192 * MB);
        CHECK(mc.flatBytes() == 256 * MB);
        CHECK(mc.translate(200 * MB + 12) == 200 * MB + 12);
        CHECK(mc.identityMapped());
    }
    SUBCASE("an empty bank between two fitted ones") {
        MemoryController950 mc({4, 0, 4, 0});
        setBoundaries(mc, 4 * MB, 4 * MB, 8 * MB);
        CHECK(mc.translate(4 * MB + 100) == 4 * MB + 100);   // bank C follows A
        CHECK(mc.identityMapped());
    }
    SUBCASE("a small bank below a large one is rotated, not lost") {
        MemoryController950 mc({4, 16, 0, 0});
        setBoundaries(mc, 4 * MB, 20 * MB, 20 * MB);
        // Bank B covers 4-20 MB and answers at (address mod 16 MB): every
        // one of its bytes has exactly one address, but not in flat order.
        CHECK(mc.translate(4 * MB) == 4 * MB + 4 * MB);
        CHECK(mc.translate(16 * MB) == 4 * MB);
        CHECK(mc.translate(20 * MB - 1) == 4 * MB + 4 * MB - 1);
        CHECK_FALSE(mc.identityMapped());
        bool seen[5] = {false, false, false, false, false};
        for (u32 a = 4 * MB; a < 20 * MB; a += 4 * MB) {
            const u32 unit = (mc.translate(a) >> 22);
            REQUIRE(unit >= 1);
            REQUIRE(unit <= 4);
            CHECK_FALSE(seen[unit]);
            seen[unit] = true;
        }
    }
}

TEST_CASE("950 memory controller: the registers above the boundaries are kept, not acted on") {
    MemoryController950 mc({8, 0, 0, 0});
    setBoundaries(mc, 8 * MB, 8 * MB, 8 * MB);
    mc.write(0xA0, 0xFFFF);
    mc.write(0x84, 1);
    load(mc, 0x48, 0x4E2, 12);
    CHECK(mc.read(0xA0) == 0xFFFF);
    CHECK(mc.read(0x84) == 1);
    CHECK(mc.boundaryBytes(0) == 8 * MB);
    CHECK(mc.translate(8 * MB - 1) == 8 * MB - 1);
    CHECK(mc.identityMapped());
}

TEST_CASE("950 memory controller: only whole SIMM sets are bank sizes") {
    CHECK(MemoryController950::validBankMb(0));
    CHECK(MemoryController950::validBankMb(4));
    CHECK(MemoryController950::validBankMb(64));
    CHECK_FALSE(MemoryController950::validBankMb(12));
    CHECK_FALSE(MemoryController950::validBankMb(128));
    MemoryController950 mc({4, 12, 0, 0});            // 12 MB is not a bank: left empty
    CHECK(mc.flatBytes() == 4 * MB);
}

// ---- the microcontroller (clock, parameter RAM, power) ------------------------

#include "../../core/src/machine/quadra950/egret.hpp"

#include <vector>

namespace {

// The host's half of the conversation, in the order the 950's ROM does it.
struct Host {
    EgretController& c;
    u8 sr = 0;            // VIA1's shift register
    bool flag = false;    // its interrupt flag

    explicit Host(EgretController& controller) : c(controller) {
        c.hostShiftOut = [this] { return sr; };
        c.completeShift = [this](bool input, u8 value) {
            if (input) sr = value;
            flag = true;
        };
    }

    void send(const std::vector<u8>& bytes) {
        c.setHostLines(true, false);                 // system session
        for (u8 b : bytes) {
            sr = b;
            flag = false;
            c.setHostLines(true, true);              // VIA full
            REQUIRE(flag);                           // the controller took it
            c.setHostLines(true, false);
        }
        flag = false;
        c.setHostLines(false, false);                // end of packet
    }

    // Take a reply (or a tick): the attention byte is thrown away, then up to
    // `max` bytes, stopping early the way the ROM does when it has enough.
    std::vector<u8> receive(std::size_t max = 100000) {
        std::vector<u8> out;
        REQUIRE(flag);
        REQUIRE_FALSE(c.xcvrSessionLine());          // held low: bytes are waiting
        flag = false;
        c.setHostLines(true, false);                 // system session: first byte
        for (;;) {
            REQUIRE(flag);
            out.push_back(sr);
            const bool last = c.xcvrSessionLine();   // released with the last byte
            c.setHostLines(true, true);              // acknowledge
            if (last || out.size() >= max) break;
            flag = false;
            c.setHostLines(true, false);             // next byte
        }
        c.setHostLines(false, true);
        c.setHostLines(false, false);
        return out;
    }

    std::vector<u8> ask(const std::vector<u8>& bytes, std::size_t max = 100000) {
        send(bytes);
        return receive(max);
    }
};

} // namespace

TEST_CASE("950 microcontroller: parameter RAM is written and read back through packets") {
    EgretController c;
    Host host(c);
    const auto ack = host.ask({0x01, 0x0C, 0x00, 0x10, 0xAA, 0xBB});
    CHECK(ack == std::vector<u8>{0x01, 0x00, 0x0C});
    CHECK(c.pram()[0x10] == 0xAA);
    CHECK(c.pram()[0x11] == 0xBB);
    CHECK(c.xcvrSessionLine());                      // released
    CHECK(c.idle());

    const auto all = host.ask({0x01, 0x07, 0x00, 0x10});
    REQUIRE(all.size() == 3 + (256 - 0x10));         // header, then to the end of PRAM
    CHECK(all[0] == 0x01);
    CHECK(all[1] == 0x00);
    CHECK(all[2] == 0x07);
    CHECK(all[3] == 0xAA);
    CHECK(all[4] == 0xBB);
    CHECK(all[5] == 0x00);
}

TEST_CASE("950 microcontroller: the host may stop a reply early") {
    EgretController c;
    Host host(c);
    c.pram()[0xFC] = 0x5A;
    // The ROM's diagnostic reader takes the header and one byte, then lowers
    // system session and waits for the controller to let go.
    const auto one = host.ask({0x01, 0x07, 0x00, 0xFC}, 4);
    CHECK(one == std::vector<u8>{0x01, 0x00, 0x07, 0x5A});
    CHECK(c.xcvrSessionLine());
    CHECK(c.idle());
    // And the next packet starts clean.
    const auto again = host.ask({0x01, 0x07, 0x00, 0xFF});
    CHECK(again.size() == 4);
}

TEST_CASE("950 microcontroller: parameter RAM is the controller's memory from $0100") {
    EgretController c;
    Host host(c);
    host.ask({0x01, 0x08, 0x01, 0x0C, 0x4E, 0x75, 0x4D, 0x63});   // what the ROM writes: 'NuMc'
    CHECK(c.pram()[0x0C] == 0x4E);
    CHECK(c.pram()[0x0F] == 0x63);
    const auto mem = host.ask({0x01, 0x02, 0x01, 0x0C}, 3 + 4);
    CHECK(mem == std::vector<u8>{0x01, 0x00, 0x02, 0x4E, 0x75, 0x4D, 0x63});
    host.ask({0x01, 0x08, 0x00, 0x20, 0x77});                     // below $0100: not PRAM
    for (u8 b : c.pram()) CHECK(b != 0x77);
}

TEST_CASE("950 microcontroller: the clock is set, counts seconds and is read") {
    EgretController c;
    Host host(c);
    host.ask({0x01, 0x09, 0x12, 0x34, 0x56, 0x78});
    c.tickSecond();
    c.tickSecond();
    const auto time = host.ask({0x01, 0x03});
    CHECK(time == std::vector<u8>{0x01, 0x00, 0x03, 0x12, 0x34, 0x56, 0x7A});
    CHECK(c.seconds() == 0x1234567Au);
}

TEST_CASE("950 microcontroller: the once-a-second tick is sent only when asked for and only on idle lines") {
    EgretController c;
    Host host(c);
    c.tickSecond();
    c.poll();
    CHECK(c.xcvrSessionLine());                      // one-second mode is off: nothing offered
    CHECK_FALSE(host.flag);

    host.ask({0x01, 0x1B, 0x03});
    CHECK(c.oneSecondMode() == 3);
    c.tickSecond();
    c.poll();
    CHECK_FALSE(c.xcvrSessionLine());
    CHECK(host.receive() == std::vector<u8>{0x03, 0x00, 0x00});
    c.poll();
    CHECK(c.xcvrSessionLine());                      // one tick per second, not a stream

    // A tick that falls due while the host is sending waits for the lines.
    c.tickSecond();
    c.setHostLines(true, false);
    c.poll();
    CHECK(c.xcvrSessionLine());
    host.sr = 0x01; c.setHostLines(true, true); c.setHostLines(true, false);
    host.sr = 0x03; c.setHostLines(true, true); c.setHostLines(true, false);
    host.flag = false;
    c.setHostLines(false, false);
    CHECK(host.receive().size() == 3 + 4);           // the reply to "read time" comes first
    c.poll();
    CHECK(host.receive() == std::vector<u8>{0x03, 0x00, 0x00});

    host.ask({0x01, 0x1B, 0x00});
    c.tickSecond();
    c.poll();
    CHECK(c.xcvrSessionLine());
}

TEST_CASE("950 microcontroller: power down and restart reach the board; unknown packets are reported") {
    EgretController c;
    Host host(c);
    int off = 0, restart = 0, unknown = 0;
    c.onPowerDown = [&] { ++off; };
    c.onResetSystem = [&] { ++restart; };
    c.onDiag = [&](const char*) { ++unknown; };
    host.ask({0x01, 0x0A});
    host.ask({0x01, 0x11});
    CHECK(off == 1);
    CHECK(restart == 1);
    CHECK(unknown == 0);
    const auto reply = host.ask({0x01, 0x7E, 0x55});
    CHECK(unknown == 1);
    CHECK(reply == std::vector<u8>{0x01, 0x00, 0x7E});
}

TEST_CASE("950 microcontroller: a machine reset drops what was in flight and keeps clock and PRAM") {
    EgretController c;
    Host host(c);
    host.ask({0x01, 0x0C, 0x00, 0x40, 0x99});
    host.ask({0x01, 0x09, 0x00, 0x00, 0x10, 0x00});
    host.ask({0x01, 0x1B, 0x03});
    host.send({0x01, 0x07, 0x00, 0x00});             // a reply is on offer...
    CHECK_FALSE(c.xcvrSessionLine());
    c.reset();                                       // ...when the machine is reset
    CHECK(c.xcvrSessionLine());
    CHECK(c.idle());
    CHECK(c.oneSecondMode() == 0);
    CHECK(c.pram()[0x40] == 0x99);
    CHECK(c.seconds() == 0x1000u);
    host.flag = false;
    const auto after = host.ask({0x01, 0x07, 0x00, 0x40}, 4);
    CHECK(after == std::vector<u8>{0x01, 0x00, 0x07, 0x99});
}
