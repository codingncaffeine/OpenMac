#include <doctest/doctest.h>
#include <openmac/machine.hpp>

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

TEST_CASE("boot overlay maps ROM at zero until PA4 clears it") {
    Machine mac(fakeRom(), {1u * 1024 * 1024});
    CHECK(mac.overlayActive());
    CHECK(mac.read8(0) == 0x00);       // ROM vector bytes visible at 0
    CHECK(mac.read8(2) == 0x20);
    CHECK(mac.cpu().pc == 0x00400010); // reset vector fetched through overlay
    CHECK(mac.cpu().a[7] == 0x2000);

    // Drive PA4 low through the VIA: DDRA all output, ORA with PA4 clear.
    mac.write8(viaReg(3), 0xFF);
    mac.write8(viaReg(15), 0x00);
    CHECK_FALSE(mac.overlayActive());
    mac.write8(0x100, 0xAB);
    CHECK(mac.read8(0x100) == 0xAB);   // RAM now lives at zero

    // Flip it back on: reads at 0 return ROM again.
    mac.write8(viaReg(15), 0x10);
    CHECK(mac.overlayActive());
    CHECK(mac.read8(0) == 0x00);
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
