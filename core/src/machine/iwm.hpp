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
        writeReady = false;
        writeLatched = false;
        writeData = 0;
        ism_ = false;
        unlock_ = 0;
        unlockN_ = 0;
        ismMode_ = 0;
        ismPhase_ = 0xF0;   // reset: all four phases outputs, all low
        ismSetup_ = 0;
        ismError_ = 0;
        ismParamIdx_ = 0;
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
        if (ism_) return ismAccess(reg & 15, write, data);
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
            switch (selected()) {
                case Reg::SetMode:   mode_ = data; noteModeWrite(data); break;
                // The write buffer is one byte deep. The machine takes the byte
                // straight after this call and hands it to the drive, which is
                // what "the buffer emptied" means to the handshake register.
                case Reg::WriteData: writeData = data; writeLatched = true; break;
                default: break;
            }
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

    // Write-Handshake: bit 7 = write buffer empty, bit 6 = write state (cleared
    // by an underrun), bits 5-0 always read as ones (SWIM ref p.11). The driver
    // spins on bit 7 before handing over each byte ($4358D0), so this is what
    // paces a write to the surface's byte rate; bit 6 stays set because the
    // machine only accepts a byte when the surface is ready for one, which is
    // the condition an underrun reports having missed.
    u8 handshake() const { return static_cast<u8>((writeReady ? 0x80 : 0x00) | 0x7F); }

    // Owned by the machine, like senseHigh: whether the drive can take a byte.
    bool writeReady = false;
    // A byte the CPU has just written to the Data register, waiting to go down.
    bool writeLatched = false;
    u8 writeData = 0;

    // Sampled by status(): the level of whichever drive status line the phase
    // lines currently address. Owned by the machine, which knows the drives.
    bool senseHigh = false;

    // ---- ISM (SWIM) register set ---------------------------------------
    //
    // The same sixteen soft switches address a completely different register
    // file once the chip is unlocked into ISM mode, and the address lines now
    // select a register directly rather than toggling latches: A3 is the
    // read/write line, so register n is written at +512*n and read at
    // +512*(n+8) (SWIM ref p.13). The phase lines stop being latches too --
    // they are driven from the Phase register.
    //
    // Reference: SWIM Chip User's Reference Rev 1.5 pp.12-13, 20-25.
    // Whether the chip will unlock into ISM mode at all. The ROM's probe decides
    // between the GCR path and the MFM path on the strength of it, so answering
    // "SWIM" commits the driver to the ISM register set for every disk -- GCR
    // media included. Off until the ISM data path can carry them.
    bool swimEnabled = false;

    bool ismSelected() const { return ism_; }
    u8 ismMode() const { return ismMode_; }
    u8 ismSetup() const { return ismSetup_; }
    bool ismGcr() const { return (ismSetup_ & 0x04) != 0; }   // Setup bit 2: 1 = GCR
    bool ismAction() const { return (ismMode_ & 0x08) != 0; } // Mode bit 3
    bool ismWriting() const { return (ismMode_ & 0x10) != 0; }// Mode bit 4: 1 = write

    // ISM data path, driven by the machine the way dataByte/writeReady are in
    // IWM mode: the FIFO holds what the framer has produced or wants.
    mutable u8 ismData = 0;      // next byte the FIFO would return
    bool ismDataReady = false;   // a byte is available to read / room to write
    bool ismMarkNext = false;    // the next byte to read is a mark byte
    bool ismCrcOk = true;        // running CRC is zero
    mutable bool ismMarkLatched = false;   // last read came from the Mark register
    mutable bool ismDataTaken = false;     // the CPU consumed a FIFO byte
    mutable bool ismWroteData = false;     // the CPU pushed a byte at the FIFO
    mutable u8 ismWritten = 0;
    mutable bool ismWroteMark = false;
    mutable bool ismWroteCrc = false;

private:
    // IWM Mode bit 6 written 1,0,1,1 in four consecutive writes unlocks the ISM
    // register set (SWIM ref p.12). Nothing else in the chip cares about the
    // sequence, so it is tracked as a four-deep shift of that one bit.
    void noteModeWrite(u8 data) {
        unlock_ = static_cast<u8>(((unlock_ << 1) | ((data >> 6) & 1)) & 0x0F);
        if (!swimEnabled) return;   // answer as a plain IWM until the ISM data path lands
        if (++unlockN_ >= 4 && unlock_ == 0x0B) {   // 1,0,1,1
            ism_ = true;
            ismMode_ = 0x40;                        // ISM select bit set
            ismPhase_ = 0xF0;                       // reset: all outputs, all low
            ismSetup_ = 0;
            ismError_ = 0;
            ismParamIdx_ = 0;
            unlockN_ = 0;
        }
    }

    u8 ismAccess(int reg, bool write, u8 data) {
        if (!(reg & 8)) {                            // A3 = 0: write registers
            if (!write) return 0;                    // a read cycle here latches nothing
            switch (reg & 7) {
                case 0: ismWritten = data; ismWroteData = true; break;   // Data
                case 1: ismWritten = data; ismWroteMark = true; break;   // Mark
                case 2:                                                  // CRC / IWM cfg
                    if (ismAction()) ismWroteCrc = true; else ismIwmCfg_ = data;
                    break;
                case 3: ismParam_[ismParamIdx_ & 15] = data; ++ismParamIdx_; break;
                case 4: ismPhase_ = data; syncPhaseLines(); break;
                case 5: ismSetup_ = data; break;
                case 6:                                  // Mode: clear the set bits
                    ismMode_ = static_cast<u8>(ismMode_ & ~data);
                    ismParamIdx_ = 0;                    // any Mode-0 access resets it
                    if (!(ismMode_ & 0x40)) leaveIsm();
                    break;
                case 7: ismMode_ = static_cast<u8>(ismMode_ | data); break;  // Mode: set
            }
            return data;
        }
        switch (reg & 7) {                           // A3 = 1: read registers
            case 0:                                  // Data (or Correction)
                ismDataTaken = true;
                return ismData;
            case 1:                                  // Mark
                ismDataTaken = true;
                ismMarkLatched = true;
                return ismData;
            case 2: { const u8 e = ismError_; ismError_ = 0; return e; }   // Error
            case 3: return ismParam_[(ismParamIdx_++) & 15];
            case 4: return ismPhase_;
            case 5: return ismSetup_;
            case 6: return ismMode_;                 // Status = the Mode register
            case 7: return ismHandshake();
        }
        return 0;
    }

    void leaveIsm() {
        ism_ = false;
        unlock_ = 0;
        unlockN_ = 0;
    }

    // In ISM mode the phase pins are driven by the Phase register, so the drive
    // still sees an address on CA0-CA2 and LSTRB. Only lines configured as
    // outputs drive; the rest keep their level.
    void syncPhaseLines() {
        const u8 dir = static_cast<u8>(ismPhase_ >> 4), st = static_cast<u8>(ismPhase_ & 0x0F);
        const u8 map[4] = {kCa0, kCa1, kCa2, kLstrb};
        for (int i = 0; i < 4; ++i) {
            if (!((dir >> i) & 1)) continue;
            if ((st >> i) & 1) lines_ |= map[i];
            else               lines_ &= static_cast<u8>(~map[i]);
        }
    }

    u8 ismHandshake() const {
        u8 h = 0;
        if (ismMarkNext) h |= 0x01;
        if (!ismCrcOk)   h |= 0x02;
        if (senseHigh)   h |= 0x08;
        if (ismMode_ & 0x80) h |= 0x10;              // MotorOn
        if (ismError_)   h |= 0x20;
        if (ismDataReady) h |= 0xC0;                 // 1-or-2 bytes, and 2 bytes
        return h;
    }

    u8 lines_ = 0;
    u8 mode_  = 0;
    bool ism_ = false;
    u8 unlock_ = 0;
    int unlockN_ = 0;
    u8 ismMode_ = 0, ismPhase_ = 0xF0, ismSetup_ = 0, ismError_ = 0, ismIwmCfg_ = 0;
    u8 ismParam_[16] = {};
    int ismParamIdx_ = 0;
};

} // namespace openmac
