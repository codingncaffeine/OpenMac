#include <doctest/doctest.h>
#include <openmac/machine.hpp>

#include <cstring>
#include <vector>

using namespace openmac;

namespace {

std::vector<u8> fakeRom() {
    std::vector<u8> rom(4096, 0xFF);
    auto put32 = [&](u32 off, u32 v) {
        rom[off] = static_cast<u8>(v >> 24);
        rom[off + 1] = static_cast<u8>(v >> 16);
        rom[off + 2] = static_cast<u8>(v >> 8);
        rom[off + 3] = static_cast<u8>(v);
    };
    put32(0, 0x00002000);   // initial SSP
    put32(4, 0x00400010);   // initial PC (in ROM)
    rom[0x10] = 0x4E; rom[0x11] = 0x71;   // NOP
    rom[0x12] = 0x60; rom[0x13] = 0xFC;   // BRA.s back to the NOP
    return rom;
}

constexpr u32 kViaBase = 0xEFE1FE;
constexpr u32 viaReg(int r) { return kViaBase + (static_cast<u32>(r) << 9); }

} // namespace

TEST_CASE("boot overlay maps ROM at zero and is a one-way latch until reset") {
    Machine mac(fakeRom(), {1u * 1024 * 1024});
    CHECK(mac.overlayActive());
    CHECK(mac.read8(0) == 0x00);       // ROM vector bytes visible at 0
    CHECK(mac.read8(2) == 0x20);
    CHECK(mac.cpu().pc == 0x00400010); // reset vector fetched through overlay
    CHECK(mac.cpu().a[7] == 0x2000);

    // Driving PA4 low (DDRA output, ORA bit 4 = 0) clears the overlay.
    mac.write8(viaReg(3), 0xFF);
    mac.write8(viaReg(15), 0x00);
    CHECK_FALSE(mac.overlayActive());
    mac.write8(0x100, 0xAB);
    CHECK(mac.read8(0x100) == 0xAB);   // RAM now lives at zero

    // One-way latch: driving PA4 high again does NOT bring the overlay back
    // (the ROM reuses PA4 late in boot and must not re-map ROM over RAM).
    mac.write8(viaReg(15), 0x10);
    CHECK_FALSE(mac.overlayActive());
    mac.write8(0x104, 0xCD);
    CHECK(mac.read8(0x104) == 0xCD);   // still RAM

    // Only a power-on reset re-arms it.
    mac.reset();
    CHECK(mac.overlayActive());
}

TEST_CASE("machine runs frames and raises VBL through the VIA") {
    Machine mac(fakeRom(), {1u * 1024 * 1024});
    // Enable the CA1 (VBL) interrupt: IER set CA1 (bit 1).
    mac.write8(viaReg(14), 0x82);
    mac.runFrame();
    CHECK(mac.totalCycles() >= 370u * 352u);
    // VBL flag must have been raised during the frame (IFR bit 1) and the
    // CPU must have taken the level-1 autovector (which the fake ROM leaves
    // at $FFFFFFFF -> odd -> the CPU halts). Either outcome proves the wire.
    const bool vblSeen = (mac.read8(viaReg(13)) & 0x02) != 0 || mac.cpu().halted;
    CHECK(vblSeen);
}

TEST_CASE("via T1 one-shot fires and gates the IRQ line") {
    Machine mac(fakeRom(), {1u * 1024 * 1024});
    mac.write8(viaReg(14), 0xC0);   // IER: enable T1
    mac.write8(viaReg(4), 50);      // T1 latch low
    mac.write8(viaReg(5), 0);       // latch high: load + start
    CHECK((mac.read8(viaReg(13)) & 0x40) == 0);
    // Run the NOP loop until well past 50 VIA clocks (500 CPU cycles).
    while (mac.totalCycles() < 700) mac.stepInstruction();
    CHECK((mac.read8(viaReg(13)) & 0x40) != 0);   // T1 flag set
    CHECK((mac.read8(viaReg(13)) & 0x80) != 0);   // IRQ asserted
    (void)mac.read8(viaReg(4));     // reading T1C-low clears the interrupt
    CHECK((mac.read8(viaReg(13)) & 0x40) == 0);
}

// Parameter RAM is the one part of a Macintosh that survives being switched
// off. The blob is deliberately opaque -- 256 bytes of XPRAM exactly as the
// guest wrote them, plus the clock -- so a setting stored by a control panel
// comes back whatever it was, and nothing on the host has to know which byte
// holds it.
TEST_CASE("parameter RAM survives the machine it came from") {
    Machine a(fakeRom(), {1u * 1024 * 1024});
    // Write a pattern through the chip the way the guest would see it, plus a
    // clock reading, then take a copy.
    // (The chip is reached here through the machine's own save/load, which is
    // the surface a front end uses.)
    std::vector<u8> blob(Machine::kPramBlobBytes);
    REQUIRE(a.savePram(blob.data(), u32(blob.size())) == Machine::kPramBlobBytes);
    CHECK(std::memcmp(blob.data(), "PRAM", 4) == 0);
    CHECK(blob[4] == 1);

    // A doctored blob stands in for "what the guest left in the chip".
    for (u32 i = 0; i < 256; ++i) blob[8 + i] = u8(i ^ 0x5A);
    blob[264] = 0x12; blob[265] = 0x34; blob[266] = 0x56; blob[267] = 0x78;

    Machine b(fakeRom(), {1u * 1024 * 1024});
    // Switched off for an hour: a real battery keeps counting.
    REQUIRE(b.loadPram(blob.data(), u32(blob.size()), 3600));
    std::vector<u8> back(Machine::kPramBlobBytes);
    REQUIRE(b.savePram(back.data(), u32(back.size())) == Machine::kPramBlobBytes);
    CHECK(std::memcmp(back.data() + 8, blob.data() + 8, 256) == 0);
    const u32 clock = (u32(back[264]) << 24) | (u32(back[265]) << 16) |
                      (u32(back[266]) << 8) | back[267];
    CHECK(clock == 0x12345678u + 3600u);

    // Anything that is not ours changes nothing at all, rather than filling
    // the chip with a stranger's bytes.
    Machine c(fakeRom(), {1u * 1024 * 1024});
    std::vector<u8> before(Machine::kPramBlobBytes);
    c.savePram(before.data(), u32(before.size()));
    std::vector<u8> junk(Machine::kPramBlobBytes, 0xAB);
    CHECK_FALSE(c.loadPram(junk.data(), u32(junk.size()), 0));
    CHECK_FALSE(c.loadPram(blob.data(), 10, 0));       // too short
    std::vector<u8> after(Machine::kPramBlobBytes);
    c.savePram(after.data(), u32(after.size()));
    CHECK(before == after);
}
