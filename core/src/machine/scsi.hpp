#pragma once

// NCR 5380 SCSI Interface Controller, as the Macintosh Classic wires it.
//
// The compact Macs decode a 512 KB block at $580000-$5FFFFF for the 5380; the
// Classic ROM addresses it near the top, at base $5FF000. Within the block:
//   register = (addr >> 4) & 7      (stride 0x10)
//   read  register on an even address ( !(addr & 1) )
//   write register on an odd  address (  (addr & 1) )
// so a read comes through Machine::read8 and a write through Machine::write8.
//
// Phase 0 (this file): the 8-register file, the read/write banks, and reset, with
// no bus/target yet. The bus phase state machine and a disk target come next.
//
// Reference: NCR 5380 Design Manual (1985) §6 (register map + bit semantics);
// Guide to the Macintosh Family Hardware 2nd ed. Ch.1/2/11. Clean-room: modelled
// from the datasheet, not transcribed from any emulator.

#include "openmac/types.hpp"

namespace openmac {

class Ncr5380 {
public:
    void reset() {
        odr_ = icr_ = mr_ = tcr_ = ser_ = 0;
    }

    // Read a register. reg = (addr >> 4) & 7 for an even (read) Mac address.
    u8 read(int reg) const {
        switch (reg & 7) {
            case 0: return currentData();   // Current SCSI Data (CSD)
            case 1: return readIcr();       // Initiator Command (ICR)
            case 2: return mr_;             // Mode (MR)
            case 3: return tcr_;            // Target Command (TCR)
            case 4: return readCsr();       // Current SCSI Bus Status (CSR)
            case 5: return readBsr();       // Bus and Status (BSR)
            case 6: return currentData();   // Input Data (IDR)
            case 7: return 0;               // Reset Parity/Interrupts (read clears; none pending)
        }
        return 0;
    }

    // Write a register. reg = (addr >> 4) & 7 for an odd (write) Mac address.
    void write(int reg, u8 v) {
        switch (reg & 7) {
            case 0: odr_ = v; break;        // Output Data (ODR)
            case 1: icr_ = v; break;        // Initiator Command (ICR)
            case 2: mr_  = v; break;        // Mode (MR)
            case 3: tcr_ = v; break;        // Target Command (TCR)
            case 4: ser_ = v; break;        // Select Enable (SER)
            case 5: case 6: case 7: break;  // Start DMA Send/TargetRecv/InitRecv (no bus yet)
        }
    }

private:
    // With no target on the bus, the data lines carry the initiator's Output Data
    // only while it drives them (ICR b0 = Assert Data Bus); otherwise they float 0.
    u8 currentData() const { return (icr_ & 0x01) ? odr_ : 0; }

    // ICR read: b7 /RST, b6 AIP, b5 LA, b4 /ACK, b3 /BSY, b2 /SEL, b1 /ATN, b0 data.
    // The initiator-driven bits mirror the write value; AIP/LA (arbitration) are 0
    // with no bus, and the write-only Test-Mode bit (b6) reads back as AIP=0.
    u8 readIcr() const { return static_cast<u8>(icr_ & 0x9F); }

    // CSR read mirrors the live bus control lines. Idle bus, no target: nothing is
    // asserted except /RST if the initiator drives it (ICR b7).
    u8 readCsr() const { return static_cast<u8>((icr_ & 0x80) ? 0x80 : 0x00); }

    // BSR read: b7 End-of-DMA, b6 DRQ, b5 ParityErr, b4 IRQ, b3 PhaseMatch, b2
    // BusyErr, b1 /ATN, b0 /ACK. No transfer, no target -> nothing asserted.
    u8 readBsr() const { return 0; }

    u8 odr_ = 0;   // Output Data Register (write)
    u8 icr_ = 0;   // Initiator Command Register
    u8 mr_  = 0;   // Mode Register
    u8 tcr_ = 0;   // Target Command Register
    u8 ser_ = 0;   // Select Enable Register (write)
};

} // namespace openmac
