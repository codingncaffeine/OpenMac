#pragma once

// IWM (Integrated Woz Machine) / SWIM floppy disk controller, as the Macintosh
// Classic wires it.
//
// Address decode: the chip lives at base $DFE1FF with a 512-byte register stride,
// so soft switch n is at $DFE1FF + n*$200 and register = (addr >> 9) & 15. The
// ROM confirms this itself -- low-memory global IWM ($01E0) holds $00DFE1FF and
// the .Sony driver addresses every access as $XX00(A0) off that pointer. The chip
// sits on the low byte of the data bus, so only odd byte accesses reach it.
//
// The 16 addresses control 8 latches; the odd address of each pair sets its latch
// and the even one clears it (SWIM Chip User's Reference p.10):
//
//   0/1   PHASE0 (CA0)      8/9   ENABLE / MotorOn
//   2/3   PHASE1 (CA1)     10/11  drive select (0 = internal, 1 = external)
//   4/5   PHASE2 (CA2)     12/13  Q6  (L6)
//   6/7   PHASE3 (LSTRB)   14/15  Q7  (L7)
//
// L7/L6/MotorOn then decode to one of six registers. Reads must be made from a
// state where the selected bit is 0 and writes from one where it is 1:
//
//   L7 L6 Mot   register
//    0  0  0    Read All Ones   (always $FF)
//    0  0  1    Read Data
//    0  1  x    Read Status
//    1  0  x    Read Write-Handshake
//    1  1  0    Set Mode        (write)
//    1  1  1    Write Data
//
// The Classic actually carries a SWIM, not a bare IWM (Macintosh Classic Developer
// Note pp.2, 5: "internal 1.4 MB SuperDrive with Super Woz Integrated Machine
// (SWIM) interface"), and the ROM's .Sony driver probes for it at $435A22 by
// switching to the ISM register set and round-tripping the Phase register. Until
// that path is implemented we answer as a plain IWM, which the ROM handles: the
// probe mismatches and it falls back to IWM/GCR mode of its own accord.
//
// Reference: SWIM Chip User's Reference Rev 1.5 (Apple, 1988) pp.10-12; Inside
// Macintosh Vol. III pp.33-44; Guide to the Macintosh Family Hardware 2nd ed.
// Ch.9; Macintosh Classic Developer Note. Clean-room from the specs.

#include "openmac/types.hpp"

#include <functional>

namespace openmac {

class Iwm {
public:
    // Latch bits in lines_, in soft-switch order.
    enum : u8 {
        kCa0    = 0x01,
        kCa1    = 0x02,
        kCa2    = 0x04,
        kLstrb  = 0x08,
        kMotor  = 0x10,   // ENABLE
        kExtDrv = 0x20,   // drive select: set = external
        kQ6     = 0x40,
        kQ7     = 0x80,
    };

    void reset() {
        lines_ = 0;
        mode_ = 0;
    }

    u8 lines() const { return lines_; }
    u8 mode() const { return mode_; }
    bool motorOn() const { return (lines_ & kMotor) != 0; }
    bool externalDrive() const { return (lines_ & kExtDrv) != 0; }
    bool lstrb() const { return (lines_ & kLstrb) != 0; }

    // The Sony drive multiplexes its status lines onto one wire; which one is
    // selected by a 4-bit address CA2:CA1:CA0:SEL, where CA0-CA2 are phase lines
    // and SEL is VIA port A bit 5. Confirmed against the ROM's own .Sony driver:
    // its address packer at $435154 unpacks D0 as CA1:CA0:SEL:CA2, and its Prime
    // routine asks for D0=2 before I/O (-> CSTIN, disk in place) and D0=6 before a
    // write (-> WRTPRT, write protect), which is exactly this table.
    int driveRegister(bool sel) const {
        return ((lines_ & kCa2) ? 8 : 0) |
               ((lines_ & kCa1) ? 4 : 0) |
               ((lines_ & kCa0) ? 2 : 0) |
               (sel ? 1 : 0);
    }

    // Which of the six registers L7/L6/MotorOn currently selects.
    enum class Reg { AllOnes, Data, Status, Handshake, SetMode, WriteData };
    Reg selected() const {
        const bool q7 = (lines_ & kQ7) != 0, q6 = (lines_ & kQ6) != 0;
        if (q7) return q6 ? ((lines_ & kMotor) ? Reg::WriteData : Reg::SetMode)
                          : Reg::Handshake;
        return q6 ? Reg::Status : ((lines_ & kMotor) ? Reg::Data : Reg::AllOnes);
    }

    // One soft-switch access. `write` distinguishes a Mac write cycle from a read;
    // the returned byte is what a read cycle sees. Every access toggles its latch
    // first, so the register a read returns is the one the NEW line state selects
    // (SWIM ref p.10: "If an operation occurs that changes the state of one of
    // these bits, the new state will select the register to be accessed").
    u8 access(int reg, bool write, u8 data) {
        switch (reg & 15) {
            case 0x0: lines_ &= ~kCa0;    break;
            case 0x1: lines_ |=  kCa0;    break;
            case 0x2: lines_ &= ~kCa1;    break;
            case 0x3: lines_ |=  kCa1;    break;
            case 0x4: lines_ &= ~kCa2;    break;
            case 0x5: lines_ |=  kCa2;    break;
            case 0x6: lines_ &= ~kLstrb;  break;
            case 0x7: lines_ |=  kLstrb;  break;
            case 0x8: lines_ &= ~kMotor;  break;
            case 0x9: lines_ |=  kMotor;  break;
            case 0xA: lines_ &= ~kExtDrv; break;
            case 0xB: lines_ |=  kExtDrv; break;
            case 0xC: lines_ &= ~kQ6;     break;
            case 0xD: lines_ |=  kQ6;     break;
            case 0xE: lines_ &= ~kQ7;     break;
            case 0xF: lines_ |=  kQ7;     break;
        }
        if (write) {
            if (selected() == Reg::SetMode) mode_ = data;
            return data;
        }
        return read();
    }

    // Read the currently selected register.
    u8 read() const {
        switch (selected()) {
            case Reg::AllOnes:   return 0xFF;
            case Reg::Status:    return status();
            case Reg::Handshake: return handshake();
            case Reg::Data:      return readData();
            default:             return 0;
        }
    }

    // Reading the Data register consumes the byte currently assembled under the
    // head. A byte is only valid once its most significant bit is a one -- that
    // is how the controller finds byte boundaries in a self-clocking GCR stream,
    // and the ROM's read loops spin on exactly that (SWIM ref p.5). Returning 0
    // while no byte is ready is what makes those loops wait rather than consume
    // garbage.
    u8 readData() const {
        const u8 b = dataByte;
        dataByte = 0;          // consumed; the drive supplies the next one
        return b;
    }

    // The byte currently under the head, or 0 if none is ready. Driven by the
    // machine from the rotating track buffer.
    mutable u8 dataByte = 0;

    // Status: bit 7 = the selected drive sense line, bit 5 = a drive enable is
    // active, bits 4-0 echo the low five bits of Mode (SWIM ref p.11).
    u8 status() const {
        u8 s = senseHigh ? 0x80 : 0x00;
        if (lines_ & kMotor) s |= 0x20;
        return static_cast<u8>(s | (mode_ & 0x1F));
    }

    // Write-Handshake: bit 7 = write buffer empty, bit 6 = write state (cleared by
    // an underrun), bits 5-0 always read as ones (SWIM ref p.11). With no write
    // path yet, report "ready, never underran".
    u8 handshake() const { return 0xFF; }

    // Sampled by status(): the level of whichever drive status line the phase
    // lines currently address. Owned by the machine, which knows the drives.
    bool senseHigh = false;

private:
    u8 lines_ = 0;
    u8 mode_  = 0;
};

} // namespace openmac
