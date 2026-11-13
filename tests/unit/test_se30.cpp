#include <doctest/doctest.h>

#include "../../core/src/machine/se30/video.hpp"
#include "openmac/se30.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

using namespace openmac;

namespace {

constexpr u32 MB = 1024 * 1024;

// Device addresses as the ROM uses them.
constexpr u32 kVia1 = 0x50F00000u, kVia2 = 0x50F02000u;
constexpr u32 kOrb = 0x0000, kDdrb = 0x0400, kDdra = 0x0600, kIfr = 0x1A00, kIer = 0x1C00, kOra = 0x1E00;

// A ROM that is nothing but ORI.B #0,D0: the processor walks through it and
// touches nothing, so a frame of machine time can pass. A few marker bytes
// let a test tell ROM from RAM.
std::vector<u8> quietRom() {
    std::vector<u8> rom(0x40000, 0);
    rom[0x1000] = 0xC3;
    rom[0x1001] = 0x5A;
    return rom;
}

std::vector<u8> videoRomImage() {
    std::vector<u8> v(0x2000, 0);
    v[0x0000] = 0x11;
    v[0x1FFF] = 0x0F;      // the format block's last byte: byte lanes
    return v;
}

Se30Machine::Config banks(u32 a, u32 b) {
    Se30Machine::Config c;
    c.bankAMb = a;
    c.bankBMb = b;
    return c;
}

// What the ROM does after its checksum: VIA1 port A's direction register
// gets $3D with the output latch at 0, which drives bit 4 low.
void overlayOff(Se30Machine& m) { m.write8(kVia1 + kDdra, 0x3D); }

// What the ROM's sizing does: bits 7-6 of VIA2 port A become outputs and
// carry the size of bank A's SIMMs.
void setSizeBits(Se30Machine& m, u8 bits) {
    m.write8(kVia2 + kDdra, 0xC0);
    m.write8(kVia2 + kOra, static_cast<u8>(bits << 6));
}

} // namespace

TEST_CASE("SE/30 RAM: the RAM-size bits pick one address line as the bank select") {
    Se30Machine m(quietRom(), banks(1, 1));
    overlayOff(m);
    setSizeBits(m, 0);                                 // 256 KB SIMMs in bank A: A20 selects
    CHECK(m.boardState().bankBStart == 1 * MB);
    CHECK(m.ramIndex(0) == 0);
    CHECK(m.ramIndex(1 * MB) == 1 * MB);               // bank B, stored after bank A
    // The line above the select line is not decoded: 2 MB is bank A again,
    // 3 MB bank B again. The ROM's sizing counts on the first (it finds
    // 2 MB by seeing its write at 2 MB land on address 0).
    CHECK(m.ramIndex(2 * MB) == 0);
    CHECK(m.ramIndex(3 * MB) == 1 * MB);
    m.write32(0, 0);
    m.write32(2 * MB, 0xA5B4C3D2u);
    CHECK(m.read32(0) == 0xA5B4C3D2u);

    setSizeBits(m, 1);                                 // told 1 MB SIMMs: A22 selects
    CHECK(m.boardState().bankBStart == 4 * MB);
    CHECK(m.ramIndex(1 * MB) == 0);                    // still bank A, which repeats by its 1 MB
    CHECK(m.ramIndex(4 * MB) == 1 * MB);
    setSizeBits(m, 2);
    CHECK(m.boardState().bankBStart == 16 * MB);
    setSizeBits(m, 3);
    CHECK(m.boardState().bankBStart == 64 * MB);
}

TEST_CASE("SE/30 RAM: a bank repeats by its own size and an empty bank does not answer") {
    Se30Machine m(quietRom(), banks(4, 1));
    overlayOff(m);
    setSizeBits(m, 1);
    // Bank B holds 1 MB: 5 MB is the same cell as 4 MB. The ROM's sizing
    // finds 5 MB by seeing its write at 5 MB change what it left at 4 MB.
    CHECK(m.ramIndex(4 * MB) == 4 * MB);
    CHECK(m.ramIndex(5 * MB) == 4 * MB);
    m.write32(4 * MB, 0);
    m.write32(5 * MB, 0x12345678u);
    CHECK(m.read32(4 * MB) == 0x12345678u);

    Se30Machine alone(quietRom(), banks(4, 0));
    overlayOff(alone);
    setSizeBits(alone, 1);
    CHECK(alone.ramIndex(4 * MB) == Se30Machine::kNoRam);
    alone.write32(4 * MB, 0x12345678u);                // lost
    CHECK(alone.read32(4 * MB) != 0x12345678u);
    CHECK(alone.read32(0) == 0);                       // and it did not land in bank A
    CHECK(alone.ramBytes() == 4 * MB);
}

TEST_CASE("SE/30 RAM: thirteen sizes are offered, each a legal pair of banks; 64 MB is not one") {
    const auto& sizes = Se30Machine::ramSizesMb();
    const u32 want[13] = {1, 2, 4, 5, 8, 16, 17, 20, 32, 65, 68, 80, 128};
    REQUIRE(sizes.size() == 13);
    for (std::size_t i = 0; i < sizes.size(); ++i) {
        CHECK(sizes[i] == want[i]);
        u32 a = 0, b = 0;
        REQUIRE(Se30Machine::banksForTotal(sizes[i], a, b));
        CHECK(a + b == sizes[i]);
        CHECK((a == 1 || a == 4 || a == 16 || a == 64));
        CHECK((b == 0 || b == 1 || b == 4 || b == 16 || b == 64));
        CHECK(a >= b);                                 // the larger SIMMs go in bank A
    }
    u32 a = 7, b = 7;
    CHECK_FALSE(Se30Machine::banksForTotal(64, a, b)); // fails the ROM's own address test
    CHECK_FALSE(Se30Machine::banksForTotal(12, a, b)); // 2 MB SIMMs: not in the ROM's table
    CHECK_FALSE(Se30Machine::banksForTotal(0, a, b));
    CHECK(a == 7);
    // A bank size no SIMM set makes is an empty bank.
    Se30Machine odd(quietRom(), banks(4, 2));
    CHECK(odd.ramBytes() == 4 * MB);
}

TEST_CASE("SE/30 overlay: the ROM answers in the RAM space until VIA1 port A bit 4 is driven low") {
    Se30Machine m(quietRom(), banks(1, 0));
    CHECK(m.overlayActive());
    CHECK(m.read8(0x1000) == 0xC3);                    // the ROM, at RAM's addresses
    CHECK(m.read16(0x1000) == 0xC35A);
    CHECK(m.read8(0x40001000u) == 0xC3);               // and at its own
    CHECK(m.read8(0x40801000u) == 0xC3);               // 256 KB repeating: where the ROM runs
    m.write8(0x1000, 0x77);                            // RAM is not reachable under the overlay
    overlayOff(m);
    CHECK_FALSE(m.overlayActive());
    setSizeBits(m, 0);
    CHECK(m.read8(0x1000) == 0x00);                    // RAM, and the write did not reach it
    m.write8(0x1000, 0x77);
    CHECK(m.read8(0x1000) == 0x77);
    CHECK(m.read8(0x40801000u) == 0xC3);
    // The line is the pin: driven high again, the ROM is back over RAM.
    m.write8(kVia1 + kOra, 0x10);
    CHECK(m.overlayActive());
    CHECK(m.read8(0x1000) == 0xC3);
    m.write8(kVia1 + kOra, 0x00);
    CHECK(m.read8(0x1000) == 0x77);
}

TEST_CASE("SE/30 address map: every device answers and the gaps end in a bus error") {
    Se30Machine::Config c = banks(1, 0);
    c.videoRom = videoRomImage();
    Se30Machine m(quietRom(), c);
    // The ROM reads these with a handler that would retry forever.
    CHECK_NOTHROW(m.read8(0x50F10010u));               // SCSI register 1
    CHECK_NOTHROW(m.read8(0x50F17C00u));               // floppy chip
    CHECK_NOTHROW(m.read8(0x50F04002u));               // serial chip, A control
    CHECK(m.read8(0x50F14800u) == 0x00);               // sound chip version: first generation
    CHECK_NOTHROW(m.read8(0x50F06000u));               // SCSI data with handshake
    CHECK_NOTHROW(m.read8(0x50F12000u));               // SCSI data
    // The device window repeats: the table in the cards book gives $5000 xxxx.
    m.write8(0x50000000u + kDdra, 0x3D);
    CHECK(m.read8(kVia1 + kDdra) == 0x3D);
    CHECK(m.read8(0x50F20000u + kDdra) == 0x3D);
    // A long word from an 8-bit port is four cycles at four addresses, which
    // for a VIA are one register (the ROM reads VIA2's port A this way).
    setSizeBits(m, 1);
    CHECK(m.read32(kVia2 + kOra) == 0x7F7F7F7Fu);

    CHECK_THROWS_AS(m.read8(0x58000000u), BusFault);   // the direct slot, empty
    CHECK_THROWS_AS(m.read32(0x58000000u), BusFault);  // the ROM's look for a diagnostic ROM
    CHECK_THROWS_AS(m.read8(0x50F08000u), BusFault);   // named in the ROM, never used
    CHECK_THROWS_AS(m.read8(0x50F18000u), BusFault);
    CHECK_THROWS_AS(m.read8(0x50F1C000u), BusFault);
    CHECK_THROWS_AS(m.write8(0x50F18000u, 0), BusFault);
    for (u32 slot = 0xF9; slot <= 0xFD; ++slot)        // the Slot Manager's five empty slots
        CHECK_THROWS_AS(m.read8((slot << 24) | 0x00FFFFFFu), BusFault);
    CHECK_THROWS_AS(m.read8(0x60000000u), BusFault);
    CHECK_THROWS_AS(m.read8(0xFF000000u), BusFault);

    // Slot $E: the video ROM at the top, byte after byte.
    CHECK(m.read8(0xFEFFFFFFu) == 0x0F);
    CHECK(m.read8(0xFEFFE000u) == 0x11);
    CHECK_NOTHROW(m.write8(0xFEFFFFFFu, 0));
    CHECK(m.read8(0xFEFFFFFFu) == 0x0F);
    Se30Machine bare(quietRom(), banks(1, 0));         // no video ROM: slot $E is empty
    CHECK_THROWS_AS(bare.read8(0xFEFFFFFFu), BusFault);
}

TEST_CASE("SE/30 identity: the two lines the ROM reads say SE/30") {
    Se30Machine m(quietRom(), banks(1, 0));
    // VIA1 port A bit 6 high; VIA2 port B bit 3 low with the line an input
    // (ROM $3F768: high and low is machine number 3).
    CHECK((m.read8(kVia1 + kOra) & 0x40) != 0);
    m.write8(kVia2 + kDdrb, 0x80);
    CHECK((m.read8(kVia2 + kOrb) & 0x08) == 0);
    CHECK((m.read8(kVia2 + kOrb) & 0x40) == 0);        // sound always stereo
    CHECK((m.read8(kVia2 + kOrb) & 0x30) == 0);        // no slot transfer error
}

TEST_CASE("SE/30 screen: video RAM repeats through slot $E and port A bit 6 picks the buffer") {
    Se30Machine m(quietRom(), banks(1, 0));
    // The driver's address and the one a 24-bit access arrives at are the
    // same cell.
    m.write8(0xFEE08040u, 0x80);
    CHECK(m.read8(0xFE008040u) == 0x80);
    CHECK(m.read8(0xFE7F8040u) == 0x80);
    m.write32(0xFE000040u, 0x40000000u);               // the alternate buffer's first long
    CHECK(m.read8(0xFEE00040u) == 0x40);

    std::vector<u32> px(static_cast<std::size_t>(m.screenWidth()) * static_cast<std::size_t>(m.screenHeight()));
    REQUIRE(m.screenWidth() == 512);
    REQUIRE(m.screenHeight() == 342);
    m.renderScreen(px.data());
    CHECK(m.boardState().mainScreenBuffer);            // the line reads high until it is driven
    CHECK(px[0] == 0xFF000000u);                       // a 1 bit is black, most significant bit first
    CHECK(px[1] == 0xFFFFFFFFu);
    // The video ROM makes bit 6 an output; low shows the alternate buffer.
    m.write8(kVia1 + kDdra, 0x7D);
    m.write8(kVia1 + kOra, 0x00);
    CHECK_FALSE(m.boardState().mainScreenBuffer);
    m.renderScreen(px.data());
    CHECK(px[0] == 0xFFFFFFFFu);
    CHECK(px[1] == 0xFF000000u);
    m.write8(kVia1 + kOra, 0x40);
    CHECK(m.boardState().mainScreenBuffer);
    // The last dot of the last row is the buffer's last bit.
    m.write8(0xFEE08040u + Se30Video::kBufferBytes - 1, 0x01);
    m.renderScreen(px.data());
    CHECK(px.back() == 0xFF000000u);
    CHECK(px[px.size() - 2] == 0xFFFFFFFFu);
}

TEST_CASE("SE/30 screen: the interrupt is set at vertical blanking and held until port B bit 6 goes high") {
    Se30Machine m(quietRom(), banks(1, 0));
    m.write8(kVia2 + kIer, 0x7F);                      // nothing enabled: only the flags are watched
    m.write8(kVia1 + kIer, 0x7F);
    // Off at reset: frames pass and nothing is asked.
    m.runFrame();
    m.runFrame();
    CHECK(m.boardState().screenInterrupts == 0);
    CHECK((m.read8(kVia2 + kOra) & 0x20) != 0);        // slot $E's line rests high
    CHECK((m.read8(kVia2 + kIfr) & 0x02) == 0);

    // The driver's Open: bit 6 an output, low.
    m.write8(kVia1 + kDdrb, 0x40);
    m.write8(kVia1 + kOrb, 0x00);
    CHECK(m.boardState().screenInterruptEnabled);
    CHECK_FALSE(m.boardState().screenInterruptRequested);   // not before the next blanking
    m.runFrame();
    CHECK(m.boardState().screenInterrupts == 1);
    CHECK(m.boardState().screenInterruptRequested);
    CHECK((m.read8(kVia2 + kOra) & 0x20) == 0);        // slot $E asks
    CHECK((m.read8(kVia2 + kIfr) & 0x02) != 0);        // and the slot line's edge was seen
    // Unanswered, it stays one request: the line is a latch, not a level
    // that follows the blanking period.
    m.runFrame();
    m.runFrame();
    CHECK(m.boardState().screenInterrupts == 1);

    // The handler: bit 6 high, then low again.
    m.write8(kVia1 + kOrb, 0x40);
    CHECK_FALSE(m.boardState().screenInterruptRequested);
    CHECK((m.read8(kVia2 + kOra) & 0x20) != 0);
    m.write8(kVia1 + kOrb, 0x00);
    CHECK_FALSE(m.boardState().screenInterruptRequested);
    m.write8(kVia2 + kIfr, 0x02);
    m.runFrame();
    CHECK(m.boardState().screenInterrupts == 2);       // one a frame when answered
    CHECK((m.read8(kVia2 + kIfr) & 0x02) != 0);
}

TEST_CASE("SE/30 screen: the chip alone -- no request while off, none lost while pending") {
    Se30Video v;
    int asked = 0, cleared = 0;
    v.onInterrupt = [&](bool asking) { ++(asking ? asked : cleared); };
    v.reset();
    v.verticalBlank();
    CHECK(asked == 0);
    v.setInterruptEnabled(true);
    CHECK(asked == 0);
    v.verticalBlank();
    v.verticalBlank();
    CHECK(asked == 1);
    CHECK(v.interruptRequested());
    v.setInterruptEnabled(false);
    CHECK(cleared == 1);
    CHECK_FALSE(v.interruptRequested());
    // A reset clears the request and the enable, and leaves the picture.
    v.setInterruptEnabled(true);
    v.verticalBlank();
    v.write(0x8040, 0x55);
    v.selectMainBuffer(false);
    v.reset();
    CHECK_FALSE(v.interruptRequested());
    CHECK_FALSE(v.interruptEnabled());
    CHECK(v.mainBufferShown());
    CHECK(v.read(0x8040) == 0x55);
}

TEST_CASE("SE/30 timing: a frame is 370 lines of 704 clocks and VIA2's timer makes the 60.15 Hz tick") {
    Se30Machine m(quietRom(), banks(1, 0));
    const u64 before = m.totalCycles();
    m.runFrame();
    const u64 took = m.totalCycles() - before;
    CHECK(took >= 260480);
    CHECK(took < 260480 + 64);                         // one instruction's overshoot at most
    CHECK(m.frameCount() == 1);
    // The ROM's set-up: timer 1 free-running with its output on port B bit 7,
    // latch $196E. The output flips every 6,512 VIA clocks, and each fall is
    // VIA1's CA1 flag: 60.15 of them a second.
    m.write8(kVia1 + kIer, 0x7F);
    m.write8(kVia2 + 0x1600, 0xC0);                    // auxiliary control
    m.write8(kVia2 + kDdrb, 0x80);
    m.write8(kVia2 + 0x0800, 0x6E);                    // timer 1 latch low
    m.write8(kVia2 + 0x0A00, 0x19);                    // high: starts it
    int ticks = 0;
    for (int f = 0; f < 120; ++f) {
        for (int slice = 0; slice < 8; ++slice) {
            const u64 until = m.totalCycles() + 260480 / 8;
            while (m.totalCycles() < until) m.stepInstruction();
            if (m.read8(kVia1 + kIfr) & 0x02) {
                ++ticks;
                m.write8(kVia1 + kIfr, 0x02);
            }
        }
    }
    CHECK(ticks >= 119);                               // 120 frames are 120 ticks, give or take the phase
    CHECK(ticks <= 121);
}

TEST_CASE("SE/30 parameter RAM: asked without a buffer the machine gives the size, and what it saves loads back") {
    Se30Machine m(quietRom(), banks(1, 0));
    CHECK(m.savePram(nullptr, 0) == Se30Machine::kPramBlobBytes);
    u8 small[16];
    CHECK(m.savePram(small, sizeof small) == Se30Machine::kPramBlobBytes);
    std::vector<u8> blob(Se30Machine::kPramBlobBytes);
    for (u32 i = 0; i < 256; ++i) blob[8 + i] = static_cast<u8>(i * 7 + 1);
    std::memcpy(blob.data(), "PRAM", 4);
    blob[7] = 1;
    blob[264] = 0x12; blob[265] = 0x34; blob[266] = 0x56; blob[267] = 0x78;
    REQUIRE(m.loadPram(blob.data(), static_cast<u32>(blob.size()), 2));
    CHECK(m.pram()[0] == 1);
    CHECK(m.pram()[255] == static_cast<u8>(255 * 7 + 1));
    std::vector<u8> back(Se30Machine::kPramBlobBytes);
    CHECK(m.savePram(back.data(), static_cast<u32>(back.size())) == Se30Machine::kPramBlobBytes);
    CHECK(std::equal(blob.begin(), blob.begin() + 264, back.begin()));
    CHECK(back[267] == 0x7A);                          // the clock went on by the two seconds given
    // A reset keeps both; a wrong file is refused.
    m.reset();
    CHECK(m.pram()[255] == static_cast<u8>(255 * 7 + 1));
    blob[0] = 'X';
    CHECK_FALSE(m.loadPram(blob.data(), static_cast<u32>(blob.size()), 0));
    CHECK_FALSE(m.loadPram(back.data(), 100, 0));
}
