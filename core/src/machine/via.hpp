#pragma once

// Synertek/Rockwell 6522 VIA as wired in the compact Macs. Clocked at
// 783.36 kHz (CPU clock / 10). CA1 = vertical blanking, CA2 = RTC one-second
// tick, shift register = ADB (P3). IRQ output feeds CPU interrupt level 1.

#include "openmac/types.hpp"

#include <functional>

namespace openmac {

class Via6522 {
public:
    void reset();

    u8   read(int reg);
    void write(int reg, u8 value);

    // Advance by VIA clocks (host divides CPU cycles by 10).
    void tick(int viaClocks);

    // Input edges from the machine.
    void setCA1(bool level);
    void setCA2(bool level);

    bool irqAsserted() const { return (ifr_ & ier_ & 0x7F) != 0; }

    // Port callbacks: the machine supplies what input bits read as, and
    // reacts to output changes (overlay, RTC lines, sound switches...).
    std::function<u8()> inA;
    std::function<u8()> inB;
    std::function<void(u8 value, u8 ddr)> outA;
    std::function<void(u8 value, u8 ddr)> outB;

    // Externally-clocked shift register (ADB transceiver): srArmed fires
    // when the CPU arms a transfer (input = shift-in); the machine calls
    // completeShift when/if the transceiver actually clocks the byte.
    std::function<void(bool input)> srArmed;
    std::function<void()> srDisarmed;   // ACR left external-shift mode

    u8 shiftValue() const { return sr_; }
    void completeShift(bool input, u8 inValue) {
        if (input) sr_ = inValue;
        ifr_ |= 0x04;   // SR interrupt flag
    }

    u8 ora() const { return ora_; }
    u8 orb() const { return orb_; }
    u8 ddra() const { return ddra_; }
    u8 ddrb() const { return ddrb_; }

private:
    void setIFR(u8 bit);
    void clearIFR(u8 bit);
    void portAWritten();
    void portBWritten();

    u8 orb_ = 0, ora_ = 0;
    u8 ddrb_ = 0, ddra_ = 0;
    u16 t1c_ = 0xFFFF, t1l_ = 0xFFFF;
    u16 t2c_ = 0xFFFF;
    u8 t2ll_ = 0;
    bool t1Running_ = false;
    bool t2Running_ = false;
    u8 sr_ = 0;
    u8 acr_ = 0, pcr_ = 0;
    u8 ifr_ = 0, ier_ = 0;
    bool ca1_ = false, ca2_ = false;
    int srTicks_ = 0;      // countdown to shift-register completion
    bool srInput_ = false; // completing a shift-in (data arrives from outside)
};

} // namespace openmac
