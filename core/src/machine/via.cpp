#include "via.hpp"

namespace openmac {

// IFR bits
namespace {
constexpr u8 kIrqCA2 = 0x01;
constexpr u8 kIrqCA1 = 0x02;
constexpr u8 kIrqSR  = 0x04;
constexpr u8 kIrqCB2 = 0x08;
constexpr u8 kIrqCB1 = 0x10;
constexpr u8 kIrqT2  = 0x20;
constexpr u8 kIrqT1  = 0x40;
} // namespace

void Via6522::reset() {
    orb_ = ora_ = 0;
    ddrb_ = ddra_ = 0;
    t1c_ = t1l_ = 0xFFFF;
    t2c_ = 0xFFFF;
    t1Running_ = t2Running_ = false;
    sr_ = 0;
    acr_ = pcr_ = 0;
    ifr_ = ier_ = 0;
    ca1_ = ca2_ = false;
}

void Via6522::setIFR(u8 bit) { ifr_ |= bit; }
void Via6522::clearIFR(u8 bit) { ifr_ = static_cast<u8>(ifr_ & ~bit); }

void Via6522::portAWritten() { if (outA) outA(ora_, ddra_); }
void Via6522::portBWritten() { if (outB) outB(orb_, ddrb_); }

u8 Via6522::read(int reg) {
    switch (reg & 15) {
    case 0: { // ORB/IRB
        clearIFR(kIrqCB1 | kIrqCB2);
        const u8 in = inB ? inB() : 0xFF;
        return static_cast<u8>((orb_ & ddrb_) | (in & ~ddrb_));
    }
    case 1: // ORA/IRA with handshake
        clearIFR(kIrqCA1 | kIrqCA2);
        [[fallthrough]];
    case 15: { // ORA without handshake
        const u8 in = inA ? inA() : 0xFF;
        return static_cast<u8>((ora_ & ddra_) | (in & ~ddra_));
    }
    case 2: return ddrb_;
    case 3: return ddra_;
    case 4: // T1C low: clears T1 interrupt
        clearIFR(kIrqT1);
        return static_cast<u8>(t1c_ & 0xFF);
    case 5: return static_cast<u8>(t1c_ >> 8);
    case 6: return static_cast<u8>(t1l_ & 0xFF);
    case 7: return static_cast<u8>(t1l_ >> 8);
    case 8: // T2C low: clears T2 interrupt
        clearIFR(kIrqT2);
        return static_cast<u8>(t2c_ & 0xFF);
    case 9: return static_cast<u8>(t2c_ >> 8);
    case 10: {
        clearIFR(kIrqSR);
        const u8 v = sr_;
        const int mode = (acr_ >> 2) & 7;
        if (mode == 3 || mode == 7) {   // external clock: the device decides
            if (srArmed) srArmed(true);
        } else if (mode != 0) {
            srTicks_ = 16;
            srInput_ = true;
        }
        return v;
    }
    case 11: return acr_;
    case 12: return pcr_;
    case 13: // IFR with bit 7 = any enabled interrupt active
        return static_cast<u8>(ifr_ | (irqAsserted() ? 0x80 : 0));
    case 14: return static_cast<u8>(ier_ | 0x80);
    default: return 0;
    }
}

void Via6522::write(int reg, u8 value) {
    switch (reg & 15) {
    case 0:
        orb_ = value;
        clearIFR(kIrqCB1 | kIrqCB2);
        portBWritten();
        break;
    case 1:
        clearIFR(kIrqCA1 | kIrqCA2);
        [[fallthrough]];
    case 15:
        ora_ = value;
        portAWritten();
        break;
    case 2: ddrb_ = value; portBWritten(); break;
    case 3: ddra_ = value; portAWritten(); break;
    case 4: // T1 latch low
    case 6:
        t1l_ = static_cast<u16>((t1l_ & 0xFF00) | value);
        break;
    case 5: // T1 latch high + load counter + start
        t1l_ = static_cast<u16>((t1l_ & 0x00FF) | (value << 8));
        t1c_ = t1l_;
        t1Running_ = true;
        clearIFR(kIrqT1);
        break;
    case 7:
        t1l_ = static_cast<u16>((t1l_ & 0x00FF) | (value << 8));
        clearIFR(kIrqT1);
        break;
    case 8:
        t2ll_ = value;
        break;
    case 9:
        t2c_ = static_cast<u16>((value << 8) | t2ll_);
        t2Running_ = true;
        clearIFR(kIrqT2);
        break;
    case 10: {
        sr_ = value;
        clearIFR(kIrqSR);
        const int mode = (acr_ >> 2) & 7;
        if (mode == 3 || mode == 7) {   // external clock: the device decides
            if (srArmed) srArmed(mode == 3);
        } else if (mode != 0) {
            srTicks_ = 16;
            srInput_ = (mode & 4) == 0;
        }
        break;
    }
    case 11: {
        const int oldMode = (acr_ >> 2) & 7;
        acr_ = value;
        const int newMode = (acr_ >> 2) & 7;
        const bool wasExt = oldMode == 3 || oldMode == 7;
        const bool isExt = newMode == 3 || newMode == 7;
        if (wasExt && !isExt && srDisarmed) srDisarmed();
        break;
    }
    case 12: pcr_ = value; break;
    case 13: // IFR: writing 1s clears those bits
        ifr_ = static_cast<u8>(ifr_ & ~(value & 0x7F));
        break;
    case 14: // IER: bit 7 selects set or clear
        if (value & 0x80) ier_ = static_cast<u8>(ier_ | (value & 0x7F));
        else              ier_ = static_cast<u8>(ier_ & ~(value & 0x7F));
        break;
    default: break;
    }
}

void Via6522::tick(int viaClocks) {
    while (viaClocks-- > 0) {
        if (srTicks_ > 0 && --srTicks_ == 0) {
            if (srInput_) sr_ = 0xFF;   // internal-clock modes: idle line
            setIFR(kIrqSR);
        }
        // T1: free-running counter; interrupts only while armed
        if (t1c_ == 0) {
            if (t1Running_) {
                setIFR(kIrqT1);
                if (acr_ & 0x40) {
                    t1c_ = t1l_;             // continuous: reload from latch
                } else {
                    t1Running_ = false;      // one-shot fired
                    t1c_ = 0xFFFF;
                }
            } else {
                t1c_ = 0xFFFF;
            }
        } else {
            --t1c_;
        }
        // T2 timed mode (pulse-counting mode is unused on the Mac)
        if (!(acr_ & 0x20)) {
            if (t2c_ == 0) {
                if (t2Running_) {
                    setIFR(kIrqT2);
                    t2Running_ = false;
                }
                t2c_ = 0xFFFF;
            } else {
                --t2c_;
            }
        }
    }
}

void Via6522::setCA1(bool level) {
    const bool positive = (pcr_ & 0x01) != 0;
    if (ca1_ != level) {
        if ((positive && !ca1_ && level) || (!positive && ca1_ && !level)) {
            setIFR(kIrqCA1);
        }
        ca1_ = level;
    }
}

void Via6522::setCA2(bool level) {
    if (pcr_ & 0x08) { ca2_ = level; return; }   // CA2 in output mode: ignore
    const bool positive = (pcr_ & 0x04) != 0;    // input modes: bit2 polarity
    if (ca2_ != level) {
        if ((positive && !ca2_ && level) || (!positive && ca2_ && !level)) {
            setIFR(kIrqCA2);
        }
        ca2_ = level;
    }
}

} // namespace openmac
