#pragma once

// The Quadra 610/650/800 "VIA2": not real silicon. IOSB provides a software
// register facade that acts like a 6522 with no timers and no shift register.
// Registers sit 0x200 apart in the $50002000 window (index = offset >> 9):
// PB=0, PA=1 (alias 15), DDRB=2, DDRA=3, IFR=13, IER=14.
//
// The IFR aggregates: bit0 = SCSI DRQ, bit1 = "any slot" (SONIC, NuBus C/D/E,
// DAFB VBL), bit3 = SCSI IRQ, bit4 = ASC/EASC (edge-triggered on this
// variant). Meaningful mask is $1B. IFR resets to $1B with IER 0, so nothing
// reaches the CPU until the ROM enables sources. Output: one line toward the
// machine's interrupt aggregator (CPU IPL 2).
//
// Reference: behavioral facts from the Quadra 650 hardware dossier (IOSB /
// pseudo-VIA sections; Apple Developer Notes for the board). Clean-room.

#include "openmac/types.hpp"

#include <functional>

namespace openmac {

class PseudoVia {
public:
    void reset() {
        // The real chip's power-on IFR reads $1B, but those bits are not
        // genuine pending events -- treating them as pending fires a phantom
        // slot interrupt (DS 51 sad Mac) the moment the ROM opens the IER.
        ifr_ = 0;
        ier_ = 0;
        ora_ = orb_ = 0;
        ddra_ = ddrb_ = 0;
        update();
    }

    u8 read(int reg) {
        switch (reg) {
        case 0:  return orb_;
        case 1: case 15: return ina ? ina() : ora_;
        case 2:  return ddrb_;
        case 3:  return ddra_;
        case 13: {
            const u8 v = static_cast<u8>(effIfr() | (irqOut_ ? 0x80 : 0));
            if (onDiag && rdBudget_ > 0) {
                --rdBudget_;
                char b[32];
                std::snprintf(b, sizeof b, "VIFR rd=%02X", v);
                onDiag(b);
            }
            return v;
        }
        case 14: return static_cast<u8>(ier_ | 0x80);
        default: return 0;
        }
    }

    void write(int reg, u8 v) {
        switch (reg) {
        case 0:
            orb_ = v;
            if (outB) outB(v, ddrb_);
            break;
        case 1: case 15:
            ora_ = v;
            break;
        case 2:  ddrb_ = v; break;
        case 3:  ddra_ = v; break;
        case 13:
            ifr_ = static_cast<u8>(ifr_ & ~(v & 0x7F));   // write-1-to-clear
            update();
            break;
        case 14:
            if (v & 0x80) ier_ = static_cast<u8>(ier_ | (v & 0x7F));
            else          ier_ = static_cast<u8>(ier_ & ~(v & 0x7F));
            update();
            break;
        default: break;
        }
    }

    // Interrupt inputs from the machine. Edge semantics: a rising edge sets
    // the IFR bit; clearing is software's job (write-1-to-clear).
    // SCSI DRQ and the SCSI interrupt read as LIVE LEVELS in the IFR: the
    // manager's select and transfer loops poll these bits as ready lines,
    // and a latched stale bit satisfies the poll with old state (the exact
    // failure: every select then evaluates garbage). The power-on $1B in
    // the latch still covers the ROM's first read; once cleared, bits 0
    // and 3 mirror the wires.
    void setScsiDrq(bool level)
    {
        if (onDiag && drqDiagBudget_ > 0 && level != scsiDrqPrev_) {
            --drqDiagBudget_;
            char b[24];
            std::snprintf(b, sizeof b, "DRQ->%d", level ? 1 : 0);
            onDiag(b);
        }
        scsiDrqPrev_ = level;
        update();
    }
    void setSlotIrq(bool level)  { edge(0x02, level, slotPrev_); }
    void setScsiIrq(bool level)  { scsiIrqPrev_ = level; update(); }
    void setAscIrq(bool level)   { edge(0x10, level, ascPrev_); }

    bool irqAsserted() const { return irqOut_; }

    // Port callbacks (VIA2 port A/B carry DFAC control and NuBus lines).
    std::function<u8()> ina;
    std::function<void(u8 value, u8 ddr)> outB;
    std::function<void(bool level)> onIrq;
    std::function<void(const char* msg)> onDiag;
    void rearmDiag(int reads, int edges) { rdBudget_ = reads; drqDiagBudget_ = edges; }

private:
    int rdBudget_ = 0;
    int drqDiagBudget_ = 0;

    void edge(u8 bit, bool level, bool& prev) {
        if (level && !prev) {
            ifr_ = static_cast<u8>(ifr_ | bit);
            update();
        }
        prev = level;
    }

    // The latch plus the live SCSI lines: what a guest IFR read sees.
    u8 effIfr() const {
        return static_cast<u8>(ifr_ | (scsiDrqPrev_ ? 0x01 : 0) |
                               (scsiIrqPrev_ ? 0x08 : 0));
    }

    void update() {
        const bool now = (effIfr() & ier_ & 0x1B) != 0;
        if (now != irqOut_) {
            irqOut_ = now;
            if (onIrq) onIrq(now);
        }
    }

    u8 ifr_ = 0x1B, ier_ = 0;
    u8 ora_ = 0, orb_ = 0, ddra_ = 0, ddrb_ = 0;
    bool irqOut_ = false;
    bool scsiDrqPrev_ = false, slotPrev_ = false;
    bool scsiIrqPrev_ = false, ascPrev_ = false;
};

} // namespace openmac
