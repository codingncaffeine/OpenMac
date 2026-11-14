#include <doctest/doctest.h>

#include "../../core/src/machine/asc.hpp"
#include "../../core/src/machine/iifx/nubus_video.hpp"
#include "../../core/src/machine/se30/video.hpp"
#include "../../core/src/machine/se30/video_rom.hpp"
#include "openmac/se30.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

using namespace openmac;

namespace {

constexpr u32 MB = 1024 * 1024;

// Device addresses as the ROM uses them.
constexpr u32 kVia1 = 0x50F00000u, kVia2 = 0x50F02000u;
constexpr u32 kOrb = 0x0000, kDdrb = 0x0400, kDdra = 0x0600, kPcr = 0x1800, kIfr = 0x1A00, kIer = 0x1C00, kOra = 0x1E00;
// The SCSI chip's three faces: registers (n at n * $10), data, data with handshake.
constexpr u32 kScsi = 0x50F10000u, kScsiData = 0x50F12000u, kScsiHandshake = 0x50F06000u;

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

u32 scsiReg(int n) { return kScsi + static_cast<u32>(n) * 0x10u; }

// The first hard-disk seat with a disk of zeros behind OpenMac's partition
// map, and a READ(6) of block 0 issued the way the ROM does it through the
// register face; the chip is left in Data In with DMA receive started. Block
// 0 is the driver descriptor map: 'E', 'R', the block size 512, the block
// count.
void startBlockZeroRead(Se30Machine& m) {
    m.insertHardDisk(std::vector<u8>(256 * 512, 0));
    m.write8(scsiReg(0), 0x81);                        // the Macintosh's ID and target 0 on the data bus
    m.write8(scsiReg(1), 0x04);                        // /SEL without /BSY
    m.write8(scsiReg(1), 0x00);
    REQUIRE((m.read8(scsiReg(4)) & 0x40) != 0);        // /BSY: the disk answered
    const u8 read6[6] = {0x08, 0x00, 0x00, 0x00, 0x01, 0x00};
    for (u8 b : read6) {
        m.write8(scsiReg(0), b);
        m.write8(scsiReg(1), 0x11);                    // data bus and /ACK
        m.write8(scsiReg(1), 0x00);
    }
    REQUIRE((m.read8(scsiReg(4)) & 0x1C) == 0x04);     // Data In
    m.write8(scsiReg(3), 0x01);                        // Target Command: Data In
    m.write8(scsiReg(2), 0x02);                        // Mode: DMA
    m.write8(scsiReg(7), 0x00);                        // start DMA receive
}

} // namespace

TEST_CASE("SE/30 SCSI: registers at $50F1 0000, data at $50F1 2000, data held for the chip at $50F0 6000") {
    Se30Machine m(quietRom(), banks(1, 0));
    // Nothing selected: the handshake face has no byte to wait for.
    CHECK_THROWS_AS(m.read8(kScsiHandshake + 0x60), BusFault);
    startBlockZeroRead(m);
    CHECK((m.read8(scsiReg(5)) & 0x40) != 0);          // Bus and Status: data request

    // The data face completes at once and takes the byte, whatever the
    // offset (the ROM reads at +$60 and writes at +0).
    CHECK(m.read8(kScsiData + 0x60) == 'E');
    CHECK(m.read8(kScsiData) == 'R');
    // Register 6 on the register face shows the byte and takes nothing.
    CHECK(m.read8(scsiReg(6)) == 0x02);
    CHECK(m.read8(scsiReg(6)) == 0x02);
    // The handshake face: a long word is four cycles, each given its byte.
    CHECK(m.read32(kScsiHandshake + 0x60) == 0x02000000u);   // block size 512, then the count's top bytes
    for (int i = 6; i < 512; ++i) CHECK_NOTHROW(m.read8(kScsiHandshake + 0x60));
    const Se30Machine::ScsiCounts before = m.scsiCounts();
    CHECK(before.bytesIn == 512);
    CHECK(before.plainDataCycles == 2);
    CHECK(before.handshakeDataCycles == 511);          // one refused before the read, 510 served
    CHECK(before.handshakeTimeouts == 1);

    // The disk is at Status now: no more data will be asked for. The
    // handshake face ends in a bus error and takes nothing; the data face
    // answers and takes nothing either.
    REQUIRE((m.read8(scsiReg(4)) & 0x1C) == 0x0C);
    CHECK_THROWS_AS(m.read8(kScsiHandshake + 0x60), BusFault);
    CHECK_THROWS_AS(m.read32(kScsiHandshake + 0x60), BusFault);
    CHECK_THROWS_AS(m.write8(kScsiHandshake, 0x55), BusFault);
    CHECK_NOTHROW(m.read8(kScsiData + 0x60));
    CHECK((m.read8(scsiReg(4)) & 0x1C) == 0x0C);       // still Status
    CHECK(m.scsiCounts().handshakeTimeouts == 4);
    CHECK(m.scsiCounts().bytesIn == 512);
}

TEST_CASE("SE/30 bus timing: what each cycle costs beyond the two clocks the processor counts") {
    Se30Machine m(quietRom(), banks(1, 0));
    overlayOff(m);
    setSizeBits(m, 0);                                 // 1 MB in bank A, every byte at its own address
    (void)m.takeCyclePenalty();
    const auto cost = [&] { return m.takeCyclePenalty(); };

    // RAM and ROM: one cycle of four clocks whatever the operand's size.
    (void)m.read32(0x1000);            CHECK(cost() == 2);
    (void)m.read8(0x1000);             CHECK(cost() == 2);
    m.write16(0x1000, 0);              CHECK(cost() == 2);
    (void)m.read32(0x40800000u);       CHECK(cost() == 2);
    CHECK(cost() == 0);                                // taken once
    // Devices are 8-bit ports: a cycle a byte, each with its wait states.
    (void)m.read8(scsiReg(1));         CHECK(cost() == 2);     // SCSI chip: 4 clocks
    (void)m.read8(0x50F17C00u);        CHECK(cost() == 2);     // floppy chip: 4
    (void)m.read8(0x50F14800u);        CHECK(cost() == 3);     // sound chip, a read: 5
    m.write8(0x50F14000u, 0);          CHECK(cost() == 2);     // a write: 4
    (void)m.read32(0xFE008040u);       CHECK(cost() == 14);    // the screen, a long word: four cycles of 4
    m.write32(0xFE008040u, 0);         CHECK(cost() == 14);
    m.write16(0xFE008040u, 0);         CHECK(cost() == 6);
    // The serial chip: 4 clocks, and a second access is held until 2.2
    // microseconds (35 clocks) after the first.
    (void)m.read8(0x50F04002u);        CHECK(cost() == 2);
    (void)m.read8(0x50F04002u);        CHECK(cost() == 2 + 4 + 35);
    // A VIA cycle starts on the VIA's clock (20 processor clocks) and ends
    // half of one later: here the machine stands on an edge.
    REQUIRE(m.totalCycles() % 20 == 0);
    (void)m.read8(kVia1 + kIfr);       CHECK(cost() == 10 - 2);
    // A cycle nobody answers lasts until the watchdog's 44 microseconds.
    CHECK_THROWS_AS(m.read8(0x58000000u), BusFault);
    CHECK(cost() == 689);
    CHECK_THROWS_AS(m.read8(kScsiHandshake + 0x60), BusFault);   // no data request: the same
    CHECK(cost() == 689);
}

TEST_CASE("SE/30 bus timing: an instruction fetched from memory is a memory cycle, one from the cache is not") {
    // The quiet ROM, with a loop of three instructions at $08-$11.
    std::vector<u8> rom = quietRom();
    rom[0x10] = 0x60;                                  // BRA.S $08
    rom[0x11] = 0xF6;
    const auto clocksFor = [](Se30Machine& m, int instructions) {
        const u64 at = m.totalCycles();
        for (int i = 0; i < instructions; ++i) m.stepInstruction();
        return static_cast<int>(m.totalCycles() - at);
    };
    // Caches off: the processor model fetches a word at a time.
    Se30Machine off(rom, banks(1, 0));
    (void)clocksFor(off, 12);                          // into the loop
    const int uncached = clocksFor(off, 30);           // ten times round
    // Instruction cache on: after the first time round nothing is fetched.
    Se30Machine on(rom, banks(1, 0));
    on.cpu().cacr = 0x00000001;
    (void)clocksFor(on, 12);
    const int cached = clocksFor(on, 30);
    // Five words a round, two clocks a word: four for a long word of code.
    CHECK(uncached - cached == 10 * 5 * 2);
    // The first time through fresh code with the cache on, a long word (one
    // ORI.B #0,D0) is one cycle of four clocks, none of which the
    // processor's own time for the instruction holds.
    Se30Machine cold(quietRom(), banks(1, 0));
    cold.cpu().cacr = 0x00000001;
    (void)clocksFor(cold, 4);
    const int coldTen = clocksFor(cold, 10);
    Se30Machine warm(rom, banks(1, 0));                // the same instruction, from the cache
    warm.cpu().cacr = 0x00000001;
    (void)clocksFor(warm, 12);
    int oriCached = 0;
    for (int i = 0; i < 3; ++i) {                      // one round: two ORIs and the branch
        const u32 pc = warm.cpu().pc;
        const int c = clocksFor(warm, 1);
        if (pc == 0x08) oriCached = c;
    }
    REQUIRE(oriCached > 0);
    CHECK(coldTen == 10 * (oriCached + 4));
}

TEST_CASE("SE/30 SCSI: the chip's data request is VIA2's CA2 and its interrupt CB2, both high when asserted") {
    Se30Machine m(quietRom(), banks(1, 0));
    CHECK((m.read8(kVia2 + kIfr) & 0x09) == 0);        // at rest, and no edge from the reset
    m.write8(kVia2 + kPcr, 0x44);                      // both lines flag a rising edge
    startBlockZeroRead(m);
    CHECK((m.read8(kVia2 + kIfr) & 0x09) == 0x01);     // data request: CA2
    for (int i = 0; i < 512; ++i) (void)m.read8(kScsiData + 0x60);
    // The disk went on to Status with the chip in DMA mode: its interrupt.
    CHECK((m.read8(scsiReg(5)) & 0x10) != 0);
    CHECK((m.read8(kVia2 + kIfr) & 0x08) != 0);        // CB2
    m.write8(kVia2 + kIfr, 0x09);                      // flags cleared
    (void)m.read8(scsiReg(7));                         // the chip's interrupt reset: the line falls
    m.write8(scsiReg(2), 0x00);
    CHECK((m.read8(kVia2 + kIfr) & 0x09) == 0);        // falling edges flag nothing here
    m.write8(kVia2 + kPcr, 0x00);                      // the ROM's setting: falling edges
    m.write8(scsiReg(2), 0x02);                        // DMA mode with Status pending: no data request
    CHECK((m.read8(kVia2 + kIfr) & 0x01) == 0);
}

TEST_CASE("SE/30 SCSI: disks at IDs 0 and 1, the CD-ROM drive at 3 and the network adapter at 4 share the bus") {
    Se30Machine m(quietRom(), banks(1, 0));
    const auto answers = [&](int id) {
        m.write8(scsiReg(1), 0x80);                    // bus reset, released
        m.write8(scsiReg(1), 0x00);
        m.write8(scsiReg(0), static_cast<u8>(0x80u | (1u << id)));
        m.write8(scsiReg(1), 0x04);
        m.write8(scsiReg(1), 0x00);
        return (m.read8(scsiReg(4)) & 0x40) != 0;
    };
    for (int id = 0; id < 7; ++id) CHECK_FALSE(answers(id));   // an empty bus
    m.insertHardDisk(std::vector<u8>(256 * 512, 0));
    m.insertHardDisk2(std::vector<u8>(256 * 512, 0));
    m.attachCdRom(true, 3);
    m.attachNet(true);
    for (int id = 0; id < 7; ++id) CHECK(answers(id) == (id == 0 || id == 1 || id == 3 || id == 4));
    m.detachHardDisk2();
    m.attachNet(false);
    CHECK_FALSE(answers(1));
    CHECK_FALSE(answers(4));
    CHECK(answers(0));
}

TEST_CASE("SE/30 host calls: this ROM's System heap begins at $1E00, and $1EFC is no addressing-mode flag") {
    Se30Machine m(quietRom(), banks(4, 0));
    overlayOff(m);
    setSizeBits(m, 1);
    // The Macintosh II family's 256K ROMs: SysZone ($02A6) = $1E00, so the
    // byte at $1EFC is heap data. Whatever it holds, the System is a 24-bit
    // one, and the MMU's mode (MMU32Bit, $0CB2) alone says whether it has
    // been switched away from that.
    m.write32(0x02A6, 0x00001E00);
    m.write8(0x0CB2, 0x00);
    m.write8(0x1EFC, 0x01);
    CHECK_FALSE(m.addressingModeSwapped());
    m.write8(0x0CB2, 0x01);
    CHECK(m.addressingModeSwapped());
    // A ROM whose low memory reaches past it (SysZone = $2000): the byte is
    // MMFlags, bit 0 the mode the System started in.
    m.write32(0x02A6, 0x00002000);
    CHECK_FALSE(m.addressingModeSwapped());            // 32-bit System, 32-bit now
    m.write8(0x0CB2, 0x00);
    CHECK(m.addressingModeSwapped());
    m.write8(0x1EFC, 0x00);
    CHECK_FALSE(m.addressingModeSwapped());
}

TEST_CASE("SE/30 sound: the chip gives the jack its two channels and the speaker their mix") {
    Asc asc;
    constexpr u8 tableValues[4] = {0x10, 0x20, 0x30, 0x40};
    for (u32 voice = 0; voice < 4; ++voice) asc.write(voice * Asc::kTableBytes, tableValues[voice]);
    asc.write(0x806, 0xE0);                    // full output level
    asc.write(0x801, 2);                       // four-voice synthesis, one channel
    Asc::MixedSample s = asc.pullMixedSample();
    CHECK(s.mono == 0xA0);                     // all four voices
    CHECK(s.left == 0xA0);
    CHECK(s.right == 0xA0);

    asc.write(0x802, 0x02);                    // stereo: two voices a channel
    s = asc.pullMixedSample();
    CHECK(s.mono == 0xA0);                     // the speaker's mixer still hears all four
    CHECK(s.left == 0x70);                     // $10 + $20 about the channel's rest at $40, given about $80
    CHECK(s.right == 0xB0);                    // $30 + $40

    asc.write(0x801, 1);                       // sampled sound, a buffer a channel
    asc.write(0x000, 0x90);
    asc.write(0x400, 0x20);
    s = asc.pullMixedSample();
    CHECK(s.left == 0x90);
    CHECK(s.right == 0x20);
    CHECK(s.mono == 0x58);                     // their mean
    s = asc.pullMixedSample();                 // buffers empty: the last sample is held
    CHECK(s.left == 0x90);
    CHECK(s.right == 0x20);
    CHECK(s.mono == 0x58);

    asc.write(0x806, 0x00);                    // output level 0: silence everywhere
    s = asc.pullMixedSample();
    CHECK(s.left == 0x80);
    CHECK(s.right == 0x80);
    CHECK(s.mono == 0x80);
}

TEST_CASE("SE/30 sound: the speaker's samples and the jack's are kept and drained apart") {
    Se30Machine m(quietRom(), banks(1, 0));
    std::vector<u8> speaker, jack;
    m.runFrame();
    m.drainAudio(speaker);
    CHECK(speaker.size() == 370);              // a sample a line
    m.drainAudio(speaker);
    CHECK(speaker.empty());
    m.drainAudioStereo(jack);                  // the same frame, still there
    CHECK(jack.size() == 2 * 370);
    CHECK(std::all_of(jack.begin(), jack.end(), [](u8 v) { return v == 0x80; }));
    // Nobody listening: each keeps 8192 sample periods and no more.
    for (int f = 0; f < 30; ++f) m.runFrame();
    m.drainAudio(speaker);
    m.drainAudioStereo(jack);
    CHECK(speaker.size() == 8192);
    CHECK(jack.size() == 2 * 8192);
}

TEST_CASE("SE/30 clock: a one-second pulse the System leaves unanswered is counted, with its frame") {
    Se30Machine m(quietRom(), banks(1, 0));
    for (int f = 0; f < 61; ++f) m.runFrame();           // the first pulse: its flag is raised
    CHECK((m.read8(kVia1 + kIfr) & 0x01) != 0);
    CHECK(m.boardState().secondsUnanswered == 0);
    for (int f = 0; f < 60; ++f) m.runFrame();           // the second finds it standing
    Se30Machine::BoardState b = m.boardState();
    CHECK(b.secondsUnanswered == 1);
    CHECK(b.unansweredAtFrame[0] == 120);
    m.write8(kVia1 + kIfr, 0x01);                        // the System answers
    for (int f = 0; f < 60; ++f) m.runFrame();
    CHECK(m.boardState().secondsUnanswered == 1);        // the third pulse was the first of a new count
}

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
    CHECK_NOTHROW(m.read8(0x50F12000u));               // SCSI data
    // SCSI data with handshake: the cycle waits for the chip's data request,
    // and with none coming the watchdog ends it.
    CHECK_THROWS_AS(m.read8(0x50F06000u), BusFault);
    CHECK_THROWS_AS(m.write8(0x50F06000u, 0), BusFault);
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
    Se30Machine::Config empty = banks(1, 0);
    empty.emptyVideoSlot = true;                       // no ROM in slot $E at all
    Se30Machine bare(quietRom(), empty);
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

namespace {

u64 fnv64(const std::vector<u8>& bytes) {
    u64 h = 1469598103934665603ull;
    for (u8 b : bytes) h = (h ^ b) * 1099511628211ull;
    return h;
}

u32 long_at(const std::vector<u8>& rom, std::size_t at) {
    return (u32(rom[at]) << 24) | (u32(rom[at + 1]) << 16) | (u32(rom[at + 2]) << 8) | rom[at + 3];
}

// A resource list's entry with this ID: where it is, or kNoEntry when the list ends without one.
constexpr std::size_t kNoEntry = ~std::size_t(0);
std::size_t entry_at(const std::vector<u8>& rom, std::size_t list, u8 id) {
    for (std::size_t at = list; at + 4 <= rom.size(); at += 4) {
        if (rom[at] == 0xFF) return kNoEntry;
        if (rom[at] == id) return at;
    }
    return kNoEntry;
}

// Where an offset entry points: its 24 bits are signed and count from the entry itself.
std::size_t target_of(const std::vector<u8>& rom, std::size_t entry) {
    const s32 offset = static_cast<s32>(long_at(rom, entry) << 8) >> 8;
    return static_cast<std::size_t>(static_cast<s64>(entry) + offset);
}

} // namespace

TEST_CASE("SE/30 own video ROM: the format block, the two resources and the mode the System reads") {
    const std::vector<u8> rom = Se30VideoRom::build();
    REQUIRE(rom.size() == 0x2000);
    // The format block is the ROM's last 20 bytes: the directory's offset, the length, the
    // checksum, revision 1, Apple's format, the test pattern, a zero and the byte lanes.
    const std::size_t format = rom.size() - 20;
    CHECK(long_at(rom, format + 4) == 0x2000);
    CHECK(rom[format + 12] == 1);
    CHECK(rom[format + 13] == 1);
    CHECK(long_at(rom, format + 14) == 0x5A932BC7u);
    CHECK(rom[format + 18] == 0);
    CHECK(rom[format + 19] == 0x0F);
    // The checksum the Slot Manager computes: rotate left one bit, add each byte, the
    // checksum's own four bytes left out.
    u32 sum = 0;
    for (std::size_t i = 0; i < rom.size(); ++i) {
        sum = (sum << 1) | (sum >> 31);
        if (i < format + 8 || i >= format + 12) sum += rom[i];
    }
    CHECK(sum == long_at(rom, format + 8));
    CHECK(rom[format] == 0);                                // the offset is 24 bits under a zero byte
    const std::size_t directory = target_of(rom, format);
    REQUIRE(directory < format);

    const std::size_t boardEntry = entry_at(rom, directory, 1);
    const std::size_t videoEntry = entry_at(rom, directory, 0x80);
    REQUIRE(boardEntry != kNoEntry);
    REQUIRE(videoEntry != kNoEntry);
    const std::size_t board = target_of(rom, boardEntry);
    const std::size_t video = target_of(rom, videoEntry);
    // The board: type 1 0 0 0, board ID $000C, a start-up routine for a 68020.
    const std::size_t boardType = target_of(rom, entry_at(rom, board, 1));
    CHECK(long_at(rom, boardType) == 0x00010000u);
    CHECK(long_at(rom, boardType + 4) == 0);
    CHECK((long_at(rom, entry_at(rom, board, 32)) & 0x00FFFFFFu) == 0x000C);
    const std::size_t startUp = target_of(rom, entry_at(rom, board, 34));
    CHECK(rom[startUp + 4] == 2);
    CHECK(rom[startUp + 5] == 2);
    CHECK(long_at(rom, startUp + 8) == 4);
    CHECK(long_at(rom, startUp) > 12);
    // The screen: display, video, Apple, hardware 9; one mode, $80.
    const std::size_t videoType = target_of(rom, entry_at(rom, video, 1));
    CHECK(long_at(rom, videoType) == 0x00030001u);
    CHECK(long_at(rom, videoType + 4) == 0x00010009u);
    CHECK(std::string(reinterpret_cast<const char*>(&rom[target_of(rom, entry_at(rom, video, 2))])) ==
          "Display_Video_Apple_MacSE/30 Video");
    CHECK(long_at(rom, target_of(rom, entry_at(rom, video, 10))) == 0);
    CHECK(long_at(rom, target_of(rom, entry_at(rom, video, 11))) == 0x8040u + 342u * 64u);
    CHECK(entry_at(rom, video, 0x81) == kNoEntry);
    const std::size_t mode = target_of(rom, entry_at(rom, video, 0x80));
    CHECK((long_at(rom, entry_at(rom, mode, 3)) & 0x00FFFFFFu) == 2);      // two pages
    CHECK((long_at(rom, entry_at(rom, mode, 4)) & 0x00FFFFFFu) == 1);      // a fixed color table
    const std::size_t params = target_of(rom, entry_at(rom, mode, 1));
    CHECK(long_at(rom, params) == 46);
    CHECK(long_at(rom, params + 4) == 0x8040);                             // the main buffer
    CHECK(long_at(rom, params + 8) == 0x00400000u);                        // 64 bytes a row, top 0
    CHECK(long_at(rom, params + 12) == 0x00000156u);                       // left 0, bottom 342
    CHECK((long_at(rom, params + 16) >> 16) == 512);
    CHECK((long_at(rom, params + 36) & 0xFFFFu) == 1);                     // one bit a dot
    // The driver: its header's four routines lie inside it, under its name.
    const std::size_t drivers = target_of(rom, entry_at(rom, video, 4));
    const std::size_t driver = target_of(rom, entry_at(rom, drivers, 2)) + 4;
    const u32 driverBytes = long_at(rom, driver - 4) - 4;
    CHECK(((u32(rom[driver]) << 8) | rom[driver + 1]) == 0x4C00);
    for (std::size_t at : {8u, 12u, 14u, 16u}) {
        const u32 routine = (u32(rom[driver + at]) << 8) | rom[driver + at + 1];
        CHECK(routine > 18);
        CHECK(routine < driverBytes);
    }
    CHECK(rom[driver + 18] == 35);
    CHECK(std::string(reinterpret_cast<const char*>(&rom[driver + 19]), 35) == ".Display_Video_Apple_MacSE/30 Video");
}

TEST_CASE("SE/30 own video ROM: it is the image whose answers were compared with the real ROM's") {
    // Any change to the routines changes this sum. Run scratch/se30/vrom/calls.sh again (the
    // driver's answers and the pictures under System 6.0.8 and 7.5, against the real ROM's)
    // before giving it a new one.
    CHECK(fnv64(Se30VideoRom::build()) == 0xAE9A43F548485997ull);
}

TEST_CASE("SE/30 own video ROM: a machine without a ROM file has it in slot $E") {
    Se30Machine m(quietRom(), banks(1, 0));
    CHECK(m.ownVideoRom());
    const std::vector<u8> rom = Se30VideoRom::build();
    for (u32 at : {0u, 0x123u, 0x1FEBu, 0x1FFFu}) CHECK(m.read8(0xFEFFE000u + at) == rom[at]);
    Se30Machine::Config file = banks(1, 0);
    file.videoRom = videoRomImage();
    Se30Machine real(quietRom(), file);
    CHECK_FALSE(real.ownVideoRom());
    CHECK(real.read8(0xFEFFE000u) == 0x11);
}

TEST_CASE("declaration ROM builders: the IIfx's generated video ROM is what it was before they moved") {
    const IifxNuBusVideo video;
    CHECK(video.declarationRom().size() == 65536);
    CHECK(fnv64(video.declarationRom()) == 0xDD91F3059CF7DB01ull);
}
