#include <doctest/doctest.h>

#include "../../core/src/machine/quadra/dafb.hpp"
#include "../../core/src/machine/quadra/ncr53c96.hpp"
#include "../../core/src/machine/quadra950/memctl.hpp"
#include "openmac/quadra.hpp"

#include <cstring>
#include <vector>

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

// ---- video: the stand-alone chip ---------------------------------------------------

namespace {

// What the ROM leaves in the chip for one display and depth: the clock
// generator's sixteen registers (one hex digit each, register 0 first), the
// timing registers that place the picture, and the DAC's control register.
// Read out of a start of the Quadra 950's ROM on each display.
struct ProgrammedMode {
    const char* name;
    const char* clock;
    u32 hal, hfp, hpix, vhline, val, vfp, control;
    int width, height;
    u32 dotClockHz;       // the display's nominal dot clock
    u32 scansMilliHz;     // and vertical rate
};

constexpr ProgrammedMode kModes[] = {
    {"12-inch, 256 colors",   "E500F00003564100", 0x063, 0x263, 0x27E, 0x32E, 0x2A, 0x32A, 0x98,  512, 384,  15667200, 60150},
    {"13-inch, 256 colors",   "EF00510003564100", 0x093, 0x313, 0x35E, 0x41A, 0x52, 0x412, 0x98,  640, 480,  30240000, 66670},
    {"portrait, 256 colors",  "E700610001564100", 0x048, 0x188, 0x19E, 0x72C, 0x58, 0x724, 0xB8,  640, 870,  57283200, 75000},
    {"16-inch, 256 colors",   "E700B00002564100", 0x089, 0x229, 0x23E, 0x536, 0x52, 0x532, 0xB8,  832, 624,  57283200, 74550},
    {"16-inch, millions",     "E700B00002564100", 0x113, 0x453, 0x47E, 0x536, 0x52, 0x532, 0x3C,  832, 624,  57283200, 74550},
    {"19-inch, 256 colors",   "8F00F10001564100", 0x03F, 0x13F, 0x14A, 0x648, 0x40, 0x640, 0xD8, 1024, 768,  80000000, 74930},
    {"21-inch, 256 colors",   "EB00620000564100", 0x03F, 0x15F, 0x16A, 0x726, 0x52, 0x71E, 0xD8, 1152, 870, 100000000, 75060},
    {"21-inch, thousands",    "EB00620000564100", 0x07F, 0x2BF, 0x2D6, 0x726, 0x52, 0x71E, 0xBE, 1152, 870, 100000000, 75060},
};

void program(Dafb& chip, const ProgrammedMode& m) {
    for (u32 r = 0; r < 16; ++r) {
        const char c = m.clock[r];
        chip.write(0x300 + r * 0x10, static_cast<u32>(c <= '9' ? c - '0' : c - 'A' + 10));
    }
    chip.write(0x140, m.hal);
    chip.write(0x144, m.hfp);
    chip.write(0x148, m.hpix);
    chip.write(0x14C, m.vhline);
    chip.write(0x15C, m.val);
    chip.write(0x160, m.vfp);
    chip.write(0x200, 0);
    chip.write(0x220, m.control);
}

void setControl(Dafb& chip, u32 first, u32 second) {
    chip.write(0x200, 0);
    chip.write(0x220, 0x06);       // the second register answers only behind this
    chip.write(0x200, 1);
    chip.write(0x220, second);
    chip.write(0x200, 0);
    chip.write(0x220, first);
}

// The ROM's video RAM sizing: a different word at the top of each half
// megabyte, highest first, then a look at which are still there.
int sizedKilobytes(Dafb& chip) {
    chip.vramWrite32(0x1FFFFC, 0x324D4547);   // '2MEG'
    chip.vramWrite32(0x17FFFC, 0x312E354D);   // '1.5M'
    chip.vramWrite32(0x0FFFFC, 0x316D6567);   // '1meg'
    chip.vramWrite32(0x07FFFC, 0x3531324B);   // '512K'
    if (chip.vramRead32(0x0FFFFC) != 0x316D6567) return 512;
    if (chip.vramRead32(0x17FFFC) != 0x312E354D) return 1024;
    if (chip.vramRead32(0x1FFFFC) != 0x324D4547) return 1536;
    return 2048;
}

} // namespace

TEST_CASE("950 video: the ROM's sizing probe finds the video RAM that is fitted") {
    Dafb two(Dafb::Variant::Standalone, 0x200000);
    Dafb one(Dafb::Variant::Standalone, 0x100000);
    CHECK(sizedKilobytes(two) == 2048);
    CHECK(sizedKilobytes(one) == 1024);
    // An empty socket is an open bus: nothing written there comes back, and
    // nothing lands in the megabyte that is fitted.
    one.vramWrite8(0x100000, 0x12);
    CHECK(one.vramRead8(0x100000) == Dafb::kOpenBus);
    one.vramWrite8(0x000000, 0x34);
    one.vramWrite8(0x100000, 0x56);
    CHECK(one.vramRead8(0x000000) == 0x34);
    // The memory controller's cell repeats its megabyte instead.
    Dafb cell;
    cell.vramWrite8(0x000010, 0x77);
    CHECK(cell.vramRead8(0x100010) == 0x77);
    CHECK(sizedKilobytes(cell) == 1024);
}

TEST_CASE("950 video: the DAC's second control register carries the AC842a's version") {
    Dafb chip(Dafb::Variant::Standalone, 0x200000);
    setControl(chip, 0x9E, 0xC0);
    chip.write(0x200, 1);
    CHECK(chip.read(0x220) == 0xC1);
    chip.write(0x200, 0);
    CHECK(chip.read(0x220) == 0x9E);
    // The ROM's test for a two-register DAC: 6 in the first, 0 in the
    // second, and the 6 must still be in the first.
    chip.write(0x200, 0);
    chip.write(0x220, 6);
    chip.write(0x200, 1);
    chip.write(0x220, 0);
    chip.write(0x200, 0);
    CHECK((chip.read(0x220) & 0x0F) == 6);
    // The Quadra 650's DAC returns its second register as written.
    Dafb cell;
    setControl(cell, 0x9E, 0xC0);
    cell.write(0x200, 1);
    CHECK(cell.read(0x220) == 0xC0);
}

TEST_CASE("950 video: pixel depth follows both control registers") {
    for (Dafb::Variant v : {Dafb::Variant::Standalone, Dafb::Variant::Memc}) {
        Dafb chip(v, 0x200000);
        setControl(chip, 0x80, 0x00); CHECK(chip.bpp() == 1);
        setControl(chip, 0x88, 0x01); CHECK(chip.bpp() == 2);
        setControl(chip, 0x90, 0x01); CHECK(chip.bpp() == 4);
        setControl(chip, 0x98, 0x01); CHECK(chip.bpp() == 8);
        setControl(chip, 0x9E, 0xC0); CHECK(chip.bpp() == 16);
        setControl(chip, 0xBE, 0xC0); CHECK(chip.bpp() == 16);
        // 32-bit pixels: the same depth bits without the 16-bit mode's two.
        setControl(chip, 0x9C, 0x01); CHECK(chip.bpp() == 24);
        setControl(chip, 0x3C, 0x01); CHECK(chip.bpp() == 24);
        setControl(chip, 0x9C, 0xC0); CHECK(chip.bpp() == 24);
        setControl(chip, 0x9E, 0x01); CHECK(chip.bpp() == 24);
    }
}

TEST_CASE("950 video: the programmed raster, dot clock and refresh are each display's own") {
    for (const ProgrammedMode& m : kModes) {
        CAPTURE(m.name);
        Dafb chip(Dafb::Variant::Standalone, 0x200000);
        CHECK(chip.refreshMilliHz() == 0);            // nothing programmed yet
        program(chip, m);
        CHECK(chip.timingWidth() == m.width);
        CHECK(chip.timingHeight() == m.height);
        const double clockError = (static_cast<double>(chip.pixelClockHz()) - m.dotClockHz) / m.dotClockHz;
        CHECK(clockError > -0.001);
        CHECK(clockError < 0.001);
        const int rateError = static_cast<int>(chip.refreshMilliHz()) - static_cast<int>(m.scansMilliHz);
        CHECK(rateError > -100);
        CHECK(rateError < 100);
    }
    // The Quadra 650's cell has another clock generator; this one's
    // registers mean nothing there.
    Dafb cell;
    program(cell, kModes[1]);
    CHECK(cell.pixelClockHz() == 0);
    CHECK(cell.refreshMilliHz() == 0);
}

TEST_CASE("950 video: direct-color pixels pass through the color table, channel by channel") {
    for (Dafb::Variant v : {Dafb::Variant::Standalone, Dafb::Variant::Memc}) {
        Dafb chip(v, 0x200000);
        Dafb::MonitorWiring monitor;
        monitor.width = 4;
        monitor.height = 1;
        chip.setMonitor(monitor);
        // A table that is different in every channel of every entry.
        chip.write(0x200, 0);
        for (u32 i = 0; i < 256; ++i) {
            chip.write(0x210, i ^ 0x55);              // red
            chip.write(0x210, 255 - i);               // green
            chip.write(0x210, (i * 7) & 0xFF);        // blue
        }
        chip.write(0x00, 0);
        chip.write(0x04, 0);
        chip.write(0x08, 4);                          // 16 bytes a row
        u32 out[4];

        setControl(chip, 0x9C, 0x01);                 // 32-bit: x, red, green, blue
        chip.vramWrite32(0, 0xEE123456);
        chip.vramWrite32(4, 0x00FF0080);
        chip.render(out);
        CHECK(out[0] == (0xFF000000u | ((0x12u ^ 0x55u) << 16) | ((255u - 0x34u) << 8) | ((0x56u * 7u) & 0xFFu)));
        CHECK(out[1] == (0xFF000000u | ((0xFFu ^ 0x55u) << 16) | (255u << 8) | ((0x80u * 7u) & 0xFFu)));

        setControl(chip, 0x9E, 0xC0);                 // 16-bit: x rrrrr ggggg bbbbb
        chip.vramWrite8(0, 0x7C);                     // red 31, green 0, blue 0
        chip.vramWrite8(1, 0x00);
        chip.vramWrite8(2, 0x04);                     // red 1, green 2, blue 30
        chip.vramWrite8(3, 0x5E);
        chip.render(out);
        // A 5-bit value reaches the entry its top bits repeat into: 31 -> 255,
        // 1 -> 8, 2 -> 16, 30 -> 247.
        CHECK(out[0] == (0xFF000000u | ((255u ^ 0x55u) << 16) | (255u << 8) | 0u));
        CHECK(out[1] == (0xFF000000u | ((8u ^ 0x55u) << 16) | ((255u - 16u) << 8) | ((247u * 7u) & 0xFFu)));
    }
}

TEST_CASE("950 video: one-bit pixels use the table's first two entries on the 950, plain black on white on the 650") {
    Dafb::MonitorWiring monitor;
    monitor.width = 8;
    monitor.height = 1;
    const auto load = [&](Dafb& chip) {
        chip.setMonitor(monitor);
        chip.write(0x200, 0);
        for (u32 v : {0xF0u, 0xE0u, 0xD0u, 0x05u, 0x06u, 0x07u}) chip.write(0x210, v);
        chip.write(0x00, 0);
        chip.write(0x04, 0);
        chip.write(0x08, 4);
        setControl(chip, 0x80, 0x00);
        chip.vramWrite8(0, 0x80);
    };
    u32 out[8];
    Dafb chip(Dafb::Variant::Standalone, 0x100000);
    load(chip);
    chip.render(out);
    CHECK(out[0] == 0xFF050607u);
    CHECK(out[1] == 0xFFF0E0D0u);
    Dafb cell;
    load(cell);
    cell.render(out);
    CHECK(out[0] == 0xFF000000u);
    CHECK(out[1] == 0xFFFFFFFFu);
}

TEST_CASE("950 video: the sense lines come up released, and a switched-off display is black") {
    Dafb::MonitorWiring thirteen;
    thirteen.grounded = 1;
    thirteen.width = 2;
    thirteen.height = 1;
    Dafb chip(Dafb::Variant::Standalone, 0x100000);
    chip.setMonitor(thirteen);
    CHECK(chip.read(0x1C) == 1);                      // read before any write, as the ROM does
    chip.write(0x1C, 5);
    chip.reset();
    CHECK(chip.read(0x1C) == 1);
    Dafb cell;
    cell.setMonitor(thirteen);
    CHECK(cell.read(0x1C) == 7);                      // the cell holds all three low until written

    chip.write(0x200, 0);
    for (u32 v : {0xFFu, 0xFFu, 0xFFu}) chip.write(0x210, v);
    setControl(chip, 0x98, 0x01);
    u32 out[2];
    chip.write(0x100, 0xFF2);
    chip.render(out);
    CHECK(out[0] == 0xFFFFFFFFu);
    chip.write(0x100, 0x001);                         // what the ROM writes with no display attached
    chip.render(out);
    CHECK(out[0] == 0xFF000000u);
    CHECK(out[1] == 0xFF000000u);
}

TEST_CASE("displays: each listed display presents Apple's sense code") {
    // Apple's three-bit code (sense 2, 1, 0) reads back inverted, one bit
    // set per grounded line: 12-inch RGB 010, 13-inch 110, color portrait
    // 101, 21-inch color 000; the 16- and 19-inch ground nothing (111) and
    // are told apart by which lines follow a driven one.
    struct Want { const char* name; u32 passive; };
    const Want want[] = {{"12-inch RGB", 5}, {"13-inch RGB", 1}, {"15-inch Portrait", 2},
                         {"16-inch Color", 0}, {"19-inch Color", 0}, {"21-inch Color", 7}};
    int n = 0;
    const QuadraMachine::DisplayInfo* list = QuadraMachine::displays(n);
    REQUIRE(n == 6);
    u32 extended[6] = {};
    for (int i = 0; i < n; ++i) {
        CAPTURE(list[i].name);
        CHECK(std::strcmp(list[i].name, want[i].name) == 0);
        Dafb chip(Dafb::Variant::Standalone, 0x100000);
        Dafb::MonitorWiring m;
        m.grounded = static_cast<u8>(list[i].grounded);
        m.pairs = static_cast<u8>(list[i].pairs);
        chip.setMonitor(m);
        CHECK(chip.read(0x1C) == want[i].passive);
        for (u32 line = 0; line < 3; ++line) {        // drive one line low, read all three
            chip.write(0x1C, 7u & ~(1u << line));
            extended[i] = (extended[i] << 3) | chip.read(0x1C);
        }
    }
    CHECK(extended[3] != extended[4]);
}

// ---- SCSI: the data request in the video chip, the byte-wide data port ----

TEST_CASE("950 SCSI: the video chip's two SCSI registers carry each bus's data request in bit 9") {
    Dafb chip(Dafb::Variant::Standalone, 0x200000);
    bool request[2] = {false, false};
    chip.scsiDrq = [&](int bus) { return request[bus]; };
    chip.write(0x24, 0x041);      // what the ROM writes to both on a 950
    chip.write(0x28, 0x1EC);      // and on the slower board: both wait bits set
    CHECK(chip.read(0x24) == 0x041);
    CHECK(chip.read(0x28) == 0x1EC);
    request[0] = true;
    CHECK(chip.read(0x24) == 0x241);
    CHECK(chip.read(0x28) == 0x1EC);
    request[0] = false;
    request[1] = true;
    CHECK(chip.read(0x24) == 0x041);
    CHECK(chip.read(0x28) == 0x3EC);

    // The wait bits are what the board acts on at the data ports.
    CHECK((chip.scsiControl(0) & (Dafb::kScsiWaitRead | Dafb::kScsiWaitWrite)) == 0);
    CHECK((chip.scsiControl(1) & Dafb::kScsiWaitRead) != 0);
    CHECK((chip.scsiControl(1) & Dafb::kScsiWaitWrite) != 0);

    // Bit 9 is the line, not a stored bit.
    request[1] = false;
    chip.write(0x28, 0x3EC);
    CHECK(chip.read(0x28) == 0x1EC);

    // The Quadra 650's video cell has no SCSI block: plain storage there.
    Dafb cell;
    cell.scsiDrq = [](int) { return true; };
    cell.write(0x24, 0x041);
    CHECK(cell.read(0x24) == 0x041);
}

TEST_CASE("950 SCSI: single bytes through the data port arrive once each") {
    // A disk the bus will show: block 0 starts with the driver map's 'ER'.
    std::vector<u8> image(64 * 512, 0);
    image[0] = 0x45;
    image[1] = 0x52;
    ScsiDisk disk;
    disk.attach(&image, 0);
    Ncr53c96 chip;
    chip.addTarget(&disk);
    chip.reset();
    u8 lastInterrupt = 0;
    const auto interrupt = [&] {
        REQUIRE(chip.irqAsserted());
        lastInterrupt = chip.read(5);
    };

    // The ROM's command: five bytes into the FIFO, the sixth through the data
    // port once the chip asks for it, with a count of one. WRITE(6), block 9.
    const u8 cdb[6] = {0x0A, 0x00, 0x00, 0x09, 0x01, 0x00};
    chip.write(0, 1);
    chip.write(1, 0);
    chip.write(4, 0);              // target 0
    chip.write(3, 0xC1);           // select without ATN, the tail by data request
    for (int i = 0; i < 5; ++i) chip.write(2, cdb[i]);
    REQUIRE(chip.drq());
    chip.dma8Write(cdb[5]);
    interrupt();
    CHECK(lastInterrupt == 0x18);  // function complete and bus service
    CHECK(chip.lastCdbLen == 6);
    CHECK(std::memcmp(chip.lastCdb, cdb, 6) == 0);
    CHECK((chip.read(4) & 7) == Ncr53c96::kDataOut);

    // The block, one byte per access. A byte taken twice would end the
    // transfer early with the wrong contents.
    chip.write(0, 0x00);
    chip.write(1, 0x02);           // 512
    chip.write(3, 0x90);           // transfer information, by data request
    for (int i = 0; i < 512; ++i) {
        REQUIRE(chip.drq());
        chip.dma8Write(static_cast<u8>(i * 7 + 3));
    }
    CHECK_FALSE(chip.drq());
    interrupt();
    CHECK((chip.read(4) & 7) == Ncr53c96::kStatus);
    bool same = true;
    for (int i = 0; i < 512; ++i) same = same && image[9 * 512 + static_cast<std::size_t>(i)] == static_cast<u8>(i * 7 + 3);
    CHECK(same);
    CHECK(image[10 * 512] == 0);   // and nothing ran past the block
}
