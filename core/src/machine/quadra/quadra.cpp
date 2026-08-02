#include "openmac/quadra.hpp"

#include "../adb.hpp"
#include "../rtc.hpp"
#include "../scsi.hpp"
#include "../scsicd.hpp"
#include "../scsiimage.hpp"
#include "../scsinet.hpp"
#include "../via.hpp"
#include "dafb.hpp"
#include "easc.hpp"
#include "ncr53c96.hpp"
#include "pseudovia.hpp"

#include <cstdio>

// Machine wiring and address decode per the Quadra 650 hardware dossier:
// djMEMC owns the map (RAM, ROM alias/overlay, DAFB), IOSB's I/O window is
// carved out at $50000000 with the documented sub-decodes and mirrors, empty
// NuBus space bus-errors. VIA1 is the Classic's real 6522; VIA2 is IOSB's
// pseudo-VIA. The ADB modem talks through VIA1's shift register exactly like
// the Classic's transceiver, and the RTC/PRAM chip is the same 343-0042
// protocol the Classic bit-bangs, so both devices are reused as-is.

namespace openmac {

namespace {

constexpr u64 kCpuHz = 33333333;         // 68040 @ 33.33 MHz
constexpr int kSlicesPerFrame = 370;     // audio rides the slice cadence
constexpr int kCyclesPerSlice = 1498;    // ~60.15 Hz tick frame
// ~694 us from ADB shift arm to completion, as measured on the Classic.
constexpr int kAdbShiftCycles = 23150;

// The 4-bit machine-ID strap on VIA1 port A: Quadra 650 = PA6|PA4|PA1.
constexpr u8 kMachineId = 0x52;

u8 bitswapNibbles(u8 v) {
    // MAC PROM bytes store the low nibble's bits reversed against the high:
    // bit order 0,1,2,3,7,6,5,4 of the source byte.
    u8 r = 0;
    const int order[8] = {0, 1, 2, 3, 7, 6, 5, 4};
    for (int i = 0; i < 8; ++i) {
        if (v & (1u << order[i])) r |= static_cast<u8>(1u << (7 - i));
    }
    return r;
}

} // namespace

QuadraMachine::QuadraMachine(std::vector<u8> rom, const Config& cfg)
    : ram_(cfg.ramSize, 0),
      rom_(std::move(rom)),
      via1_(std::make_unique<Via6522>()),
      via2_(std::make_unique<PseudoVia>()),
      rtc_(std::make_unique<Rtc>()),
      adb_(std::make_unique<AdbTransceiver>()),
      dafb_(std::make_unique<Dafb>()),
      easc_(std::make_unique<Easc>()),
      scsi_(std::make_unique<Ncr53c96>()),
      disk_(std::make_unique<ScsiDisk>()),
      disk2_(std::make_unique<ScsiDisk>()),
      cdrom_(std::make_unique<ScsiCdRom>()),
      netdev_(std::make_unique<ScsiEthernet>()),
      cpu_(*this) {
    u32 rs = 1;
    while (rs < rom_.size()) rs <<= 1;
    rom_.resize(rs, 0xFF);
    romMask_ = rs - 1;

    scsi_->addTarget(disk_.get());
    scsi_->addTarget(disk2_.get());
    scsi_->addTarget(cdrom_.get());
    scsi_->addTarget(netdev_.get());

    // Ethernet MAC PROM: Apple OUI 08:00:07 + a fixed station id, each byte
    // nibble-bit-swizzled, byte 7 = inverted XOR checksum.
    const u8 mac[6] = {0x08, 0x00, 0x07, 0x2A, 0x65, 0x50};
    u8 x = 0;
    for (int i = 0; i < 6; ++i) {
        macProm_[i] = bitswapNibbles(mac[i]);
        x ^= macProm_[i];
    }
    macProm_[6] = 0;
    macProm_[7] = static_cast<u8>(x ^ 0xFF);

    wireDevices();
    reset();
}

QuadraMachine::QuadraMachine(std::vector<u8> rom)
    : QuadraMachine(std::move(rom), Config{}) {}

QuadraMachine::~QuadraMachine() = default;

void QuadraMachine::wireDevices() {
    via1_->inA = [] {
        // The machine-ID strap: bits 1/2/4/6 carry the board code and the
        // rest read low, so the ROM's masked read sees exactly $52.
        return kMachineId;
    };
    via1_->inB = [this] {
        u8 v = 0xFF;
        if (!rtc_->dataOut()) v = static_cast<u8>(v & ~0x01);
        if (!adb_->intLine()) v = static_cast<u8>(v & ~0x08);   // PB3, active low
        return v;
    };
    via1_->outA = [](u8, u8) { /* PA5 = floppy HDSEL; nothing modeled yet */ };
    via1_->outB = [this](u8 value, u8 ddr) {
        const u8 eff = static_cast<u8>(value | ~ddr);
        rtc_->setLines((eff & 0x01) != 0, (eff & 0x02) != 0, (eff & 0x04) != 0);
        const int prev = adb_->state();
        adb_->setState((eff >> 4) & 3);   // PB4/PB5 = "newaction"
        if (adb_->state() != prev) adbMaybeClock();
        updateIpl();
    };
    via1_->srArmed = [this](bool input) {
        adbArmed_ = true;
        adbArmedInput_ = input;
        adbMaybeClock();
    };
    via1_->srDisarmed = [this] { adbArmed_ = false; };

    via2_->outB = [this](u8 value, u8 ddr) {
        const u8 eff = static_cast<u8>(value | ~ddr);
        // DFAC 3-wire: PB0 latch, PB3 data, PB4 clock.
        easc_->dfacLines((eff & 0x01) != 0, (eff & 0x08) != 0, (eff & 0x10) != 0);
    };
    via2_->onIrq = [this](bool) { updateIpl(); };

    dafb_->onIrq = [this](bool level) { via2_->setSlotIrq(level); };
    easc_->onIrq = [this](bool level) { via2_->setAscIrq(level); };
    easc_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    scsi_->onIrq = [this](bool level) { via2_->setScsiIrq(level); };
    scsi_->onDrq = [this](bool level) { via2_->setScsiDrq(level); };
    scsi_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    scsi_->onCmd = [this](u8 cmd, u32 fifoLevel, u8 phase) {
        if (cdbDiagBudget_ <= 0) return;
        char b[80];
        std::snprintf(b, sizeof b, "SCMD %02X fifo=%u phase=%d", cmd, fifoLevel, phase);
        if (onDiag) onDiag(b);
    };
    scsi_->onCdb = [this](int id, const u8* cdb, int len) {
        // The 8th CDB is the probe's boot-block read; re-arm the register
        // trace there so the storm before it cannot exhaust the budget.
        if (++cdbSeen_ == 8) cdbDiagBudget_ = 5000;
        if (cdbListBudget_ <= 0) return;
        --cdbListBudget_;
        char b[112];
        int n = std::snprintf(b, sizeof b, "CDB id%d:", id);
        for (int i = 0; i < len && n < 100; ++i)
            n += std::snprintf(b + n, sizeof b - static_cast<size_t>(n), " %02X", cdb[i]);
        if (onDiag) onDiag(b);
    };

    netdev_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };

    cpu_.onResetInstruction = [this] {
        via1_->reset();
        via2_->reset();
        rtc_->reset();
        adb_->reset();
        scsi_->reset();
        easc_->reset();
        adbArmed_ = false;
        adbPending_ = 0;
    };
}

void QuadraMachine::reset() {
    overlay_ = true;
    for (auto& r : djmemcRegs_) r = 0;
    for (auto& r : iosbRegs_) r = 0;
    for (auto& r : sonicRegs_) r = 0;
    sonicRegs_[0x29] = 6;                 // silicon revision
    sonicRegs_[0x00] = 0x0094;            // CR: RST | STP | RXDIS
    sccPtr_ = 0;
    swimMode_ = 0x40;
    via1_->reset();
    via2_->reset();
    rtc_->reset();
    adb_->reset();
    dafb_->reset();
    easc_->reset();
    scsi_->reset();
    adbArmed_ = false;
    adbPending_ = 0;
    totalCycles_ = 0;
    via1Remainder_ = 0;
    secondAcc_ = 0;
    vblAcc_ = 0;
    bootTraceFrame_ = 0;
    cpu_.reset();
}

void QuadraMachine::updateIpl() {
    // SCC would be level 4; the stub never interrupts. VIA2 aggregate is
    // level 2, VIA1 level 1.
    int ipl = 0;
    if (via2_->irqAsserted()) ipl = 2;
    else if (via1_->irqAsserted()) ipl = 1;
    if (ipl == 2 && iplDiagBudget_ > 0) {
        --iplDiagBudget_;
        char b[96];
        std::snprintf(b, sizeof b, "IPL2 asserted: via2 ifr=%02X ier=%02X pc=%08X",
                      via2_->read(13), via2_->read(14), cpu_.pc);
        if (onDiag) onDiag(b);
    }
    cpu_.setIrqLevel(ipl);
}

// ---------------------------------------------------------------- bus decode

void QuadraMachine::logAccess(const char* what, u32 addr, bool write, u32 value) {
    if (accessLog_.size() >= 4000) return;
    char buf[96];
    std::snprintf(buf, sizeof buf, "%s %c %08X = %X (pc %08X)", what,
                  write ? 'W' : 'R', addr, value, cpu_.pc);
    accessLog_.emplace_back(buf);
}

u8 QuadraMachine::read8(u32 addr) {
    if (addr < 0x40000000u) {
        if (overlay_ && addr < 0x400000u) return rom_[addr & romMask_];
        if (addr < ram_.size()) return ram_[addr];
        return 0xFF;   // unpopulated RAM: the sizing probe reads open bus
    }
    if (addr < 0x50000000u) {
        overlay_ = false;   // first native-ROM read swaps RAM in at zero
        return rom_[addr & romMask_];
    }
    if (addr < 0x60000000u) return ioRead8(addr);
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) return dafb_->vramRead8(off);
        if (off >= 0x800000u && off < 0x800400u) {
            const u32 v = dafb_->read(off & 0x3FFu & ~3u);
            return static_cast<u8>(v >> (8 * (3 - (off & 3))));
        }
    }
    throw BusFault{addr, true, 1};
}

void QuadraMachine::write8(u32 addr, u8 value) {
    if (addr < 0x40000000u) {
        if (addr < ram_.size()) ram_[addr] = value;
        return;   // writes to holes vanish, as on the real bus
    }
    if (addr < 0x50000000u) return;   // ROM
    if (addr < 0x60000000u) { ioWrite8(addr, value); return; }
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) {
            if (onVramWrite) onVramWrite(off, cpu_.pc);
            dafb_->vramWrite8(off, value);
            return;
        }
        if (off >= 0x800000u && off < 0x800400u) {
            const u32 reg = off & 0x3FFu;
            if ((reg >> 8) == 2) {
                // RAMDAC: each byte write is one step of the CLUT's 3-phase
                // R,G,B cycle -- pass it through untouched (a read-modify-
                // write here would advance the read phase and mangle it).
                dafb_->write(reg & ~3u, value);
                return;
            }
            const u32 r32 = reg & ~3u;
            const int shift = 8 * (3 - (off & 3));
            const u32 old = dafb_->read(r32);
            dafb_->write(r32, (old & ~(0xFFu << shift)) |
                                  (static_cast<u32>(value) << shift));
            return;
        }
    }
    throw BusFault{addr, false, 1};
}

u16 QuadraMachine::read16(u32 addr) {
    if (addr < 0x40000000u) {
        if (overlay_ && addr < 0x400000u) {
            const u32 a = addr & romMask_;
            return static_cast<u16>((rom_[a] << 8) | rom_[(a + 1) & romMask_]);
        }
        if (addr + 1 < ram_.size())
            return static_cast<u16>((ram_[addr] << 8) | ram_[addr + 1]);
        return 0xFFFF;
    }
    if (addr < 0x50000000u) {
        overlay_ = false;
        const u32 a = addr & romMask_;
        return static_cast<u16>((rom_[a] << 8) | rom_[(a + 1) & romMask_]);
    }
    if (addr < 0x60000000u) {
        const u32 off = addr & 0x0003FFFFu;
        if (off >= 0x0A000u && off < 0x0B100u) {   // SONIC: 16-bit lanes
            return sonicRegs_[(off >> 2) & 0x3F];
        }
        // The SCSI pseudo-DMA port is a true 16-bit lane: a word read moves
        // two data bytes (the manager bursts MOVE.W/MOVE.L from it).
        if (off >= 0x10100u && off < 0x10104u) return scsi_->dma16Read();
        // 8-bit peripherals answer 16-bit reads with the byte in both lanes
        // (the SCC glue's documented behavior; safe for the VIAs too since
        // it decodes the register only once).
        const u8 v = ioRead8(addr);
        return static_cast<u16>((v << 8) | v);
    }
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) {
            return static_cast<u16>((dafb_->vramRead8(off) << 8) |
                                    dafb_->vramRead8(off + 1));
        }
        if (off >= 0x800000u && off < 0x800400u) {
            const u32 v = dafb_->read(off & 0x3FFu & ~3u);
            return static_cast<u16>(v >> ((off & 2) ? 0 : 16));
        }
    }
    throw BusFault{addr, true, 2};
}

void QuadraMachine::write16(u32 addr, u16 value) {
    if (addr < 0x40000000u) {
        if (addr + 1 < ram_.size()) {
            ram_[addr] = static_cast<u8>(value >> 8);
            ram_[addr + 1] = static_cast<u8>(value);
        }
        return;
    }
    if (addr < 0x50000000u) return;
    if (addr < 0x60000000u) {
        const u32 off = addr & 0x0003FFFFu;
        if (off >= 0x0A000u && off < 0x0B100u) {
            sonicWrite((off >> 2) & 0x3F, value);
            return;
        }
        if (off >= 0x0C000u && off < 0x0E000u) {
            ioWrite8(addr, static_cast<u8>(value >> 8));   // SCC: high byte
            return;
        }
        // The SCSI pseudo-DMA port takes both bytes of a word write.
        if (off >= 0x10100u && off < 0x10104u) {
            scsi_->dma16Write(value);
            return;
        }
        ioWrite8(addr, static_cast<u8>(value >> 8));
        return;
    }
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) {
            dafb_->vramWrite8(off, static_cast<u8>(value >> 8));
            dafb_->vramWrite8(off + 1, static_cast<u8>(value));
            return;
        }
        if (off >= 0x800000u && off < 0x800400u) {
            const u32 reg = off & 0x3FFu & ~3u;
            const int shift = (off & 2) ? 0 : 16;
            const u32 old = dafb_->read(reg);
            dafb_->write(reg, (old & ~(0xFFFFu << shift)) |
                                  (static_cast<u32>(value) << shift));
            return;
        }
    }
    throw BusFault{addr, false, 2};
}

u32 QuadraMachine::read32(u32 addr) {
    if (addr < 0x40000000u) {
        if (overlay_ && addr < 0x400000u) {
            const u32 a = addr & romMask_;
            return (static_cast<u32>(rom_[a]) << 24) |
                   (static_cast<u32>(rom_[(a + 1) & romMask_]) << 16) |
                   (static_cast<u32>(rom_[(a + 2) & romMask_]) << 8) |
                   rom_[(a + 3) & romMask_];
        }
        if (addr + 3 < ram_.size()) {
            return (static_cast<u32>(ram_[addr]) << 24) |
                   (static_cast<u32>(ram_[addr + 1]) << 16) |
                   (static_cast<u32>(ram_[addr + 2]) << 8) | ram_[addr + 3];
        }
        return 0xFFFFFFFFu;
    }
    if (addr < 0x50000000u) {
        overlay_ = false;
        const u32 a = addr & romMask_;
        return (static_cast<u32>(rom_[a]) << 24) |
               (static_cast<u32>(rom_[(a + 1) & romMask_]) << 16) |
               (static_cast<u32>(rom_[(a + 2) & romMask_]) << 8) |
               rom_[(a + 3) & romMask_];
    }
    if (addr < 0x60000000u) return ioRead32(addr);
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) return dafb_->vramRead32(off);
        if (off >= 0x800000u && off < 0x800400u) return dafb_->read(off & 0x3FFu);
    }
    throw BusFault{addr, true, 4};
}

void QuadraMachine::write32(u32 addr, u32 value) {
    if (addr < 0x40000000u) {
        if (addr + 3 < ram_.size()) {
            ram_[addr] = static_cast<u8>(value >> 24);
            ram_[addr + 1] = static_cast<u8>(value >> 16);
            ram_[addr + 2] = static_cast<u8>(value >> 8);
            ram_[addr + 3] = static_cast<u8>(value);
        }
        return;
    }
    if (addr < 0x50000000u) return;
    if (addr < 0x60000000u) { ioWrite32(addr, value); return; }
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) {
            if (onVramWrite) onVramWrite(off, cpu_.pc);
            dafb_->vramWrite32(off, value);
            return;
        }
        if (off >= 0x800000u && off < 0x800400u) {
            dafb_->write(off & 0x3FFu, value);
            return;
        }
    }
    throw BusFault{addr, false, 4};
}

// ---- the $50000000 I/O window, mirrors folded to 256KB ----

u8 QuadraMachine::ioRead8(u32 addr) {
    if ((addr & 0x00FF0000u) == 0x00FF0000u) {
        // IOSB chip id, every read in the top 64KB window.
        const u32 v = 0xA55A2BADu;
        return static_cast<u8>(v >> (8 * (3 - (addr & 3))));
    }
    const u32 off = addr & 0x0003FFFFu;
    if (off < 0x02000u) return via1_->read((off >> 9) & 0xF);
    if (off < 0x04000u) return via2_->read((off >> 9) & 0xF);
    if (off >= 0x08000u && off < 0x08008u) return macProm_[off & 7];
    if (off >= 0x0A000u && off < 0x0B100u) {
        const u16 v = sonicRegs_[(off >> 2) & 0x3F];
        return static_cast<u8>((off & 1) ? v : (v >> 8));
    }
    if (off >= 0x0C000u && off < 0x0E000u) {   // SCC (functional stub)
        if ((off & 0x2) == 0) {                // control
            const int reg = sccPtr_;
            sccPtr_ = 0;
            if (reg == 0) return 0x54;         // Tx empty | underrun | hunt
            if (reg == 1) return 0x01;         // all sent
            return sccRegs_[reg & 15];
        }
        return 0;                              // data
    }
    if (off >= 0x0E000u && off < 0x10000u) {
        const u32 v = djmemcRegs_[(off >> 2) & 0xF];
        return static_cast<u8>(v >> (8 * (3 - (off & 3))));
    }
    if (off >= 0x10000u && off < 0x10100u) {
        const u8 v = scsi_->read((off >> 4) & 0xF);
        if (scsiDiagBudget_ > 0) {
            --scsiDiagBudget_;
            logAccess("SCSI", addr, false, v);
        }
        return v;
    }
    if (off >= 0x10100u && off < 0x10104u) {
        // A byte-wide pseudo-DMA read hands over exactly one byte.
        return scsi_->dma8Read();
    }
    if (off >= 0x14000u && off < 0x15000u) {
        const u8 v = easc_->read(off & 0xFFF);
        if ((off & 0xFFF) >= 0x800 && eascDiagBudget_ > 0) {
            --eascDiagBudget_;
            logAccess("EASC", addr, false, v);
        }
        return v;
    }
    if (off >= 0x18000u && off < 0x1A000u) {
        const u16 v = iosbRegs_[(off >> 2) & 0x1F];
        return static_cast<u8>((off & 1) ? v : (v >> 8));
    }
    if (off >= 0x1E000u && off < 0x20000u) {   // SWIM2 stub
        const int reg = static_cast<int>((off >> 9) & 7);
        switch (reg) {
        case 2: return 0;                      // error, reads clear
        case 3: return swimParams_[swimParamPtr_++ & 15];
        case 4: return swimPhases_;            // presence probe reads back
        case 5: return swimSetup_;
        case 6: return swimMode_;
        case 7: return 0xFF;                   // handshake: an empty port floats
                                               // every line high (no drive)
        default: return 0xFF;                  // empty FIFO / floating
        }
    }
    logAccess("IO?", addr, false, 0);
    return 0xFF;
}

void QuadraMachine::ioWrite8(u32 addr, u8 v) {
    if ((addr & 0x00FF0000u) == 0x00FF0000u) return;
    const u32 off = addr & 0x0003FFFFu;
    if (off < 0x02000u) { via1_->write((off >> 9) & 0xF, v); updateIpl(); return; }
    if (off < 0x04000u) {
        const int reg = (off >> 9) & 0xF;
        if ((reg == 13 || reg == 14) && via2DiagBudget_ > 0) {
            --via2DiagBudget_;
            char b[96];
            std::snprintf(b, sizeof b, "VIA2 reg%d W %02X (pc %08X)", reg, v, cpu_.pc);
            if (onDiag) onDiag(b);
        }
        via2_->write(reg, v);
        updateIpl();
        return;
    }
    if (off >= 0x0C000u && off < 0x0E000u) {
        if ((off & 0x2) == 0) {
            if (sccPtr_ == 0) sccPtr_ = v & 0x0F;
            else { sccRegs_[sccPtr_] = v; sccPtr_ = 0; }
        }
        return;
    }
    if (off >= 0x0E000u && off < 0x10000u) {
        // djMEMC: echo what was written (bank config readback matters).
        u32& r = djmemcRegs_[(off >> 2) & 0xF];
        const int shift = 8 * (3 - (off & 3));
        r = (r & ~(0xFFu << shift)) | (static_cast<u32>(v) << shift);
        return;
    }
    if (off >= 0x10000u && off < 0x10100u) {
        const u32 reg = (off >> 4) & 0xF;
        if (reg <= 4 && cdbDiagBudget_ > 0) {
            --cdbDiagBudget_;
            char b[64];
            std::snprintf(b, sizeof b, "SREG%u W %02X", reg, v);
            if (onDiag) onDiag(b);
        }
        scsi_->write(static_cast<int>(reg), v);
        updateIpl();
        return;
    }
    if (off >= 0x10100u && off < 0x10104u) {
        if (cdbDiagBudget_ > 0) {
            --cdbDiagBudget_;
            char b[48];
            std::snprintf(b, sizeof b, "SDMA8 W %02X", v);
            if (onDiag) onDiag(b);
        }
        scsi_->dma16Write(static_cast<u16>((v << 8) | v));
        return;
    }
    if (off >= 0x14000u && off < 0x15000u) { easc_->write(off & 0xFFF, v); return; }
    if (off >= 0x18000u && off < 0x1A000u) {
        u16& r = iosbRegs_[(off >> 2) & 0x1F];
        if (off & 1) r = static_cast<u16>((r & 0xFF00) | v);
        else r = static_cast<u16>((r & 0x00FF) | (v << 8));
        return;
    }
    if (off >= 0x1E000u && off < 0x20000u) {
        const int reg = static_cast<int>((off >> 9) & 7);
        switch (reg) {
        case 3: swimParams_[swimParamPtr_++ & 15] = v; break;
        case 4: swimPhases_ = v; break;
        case 5: swimSetup_ = v; break;
        case 6: swimMode_ = static_cast<u8>(swimMode_ & ~v); break;   // mode clear
        case 7: swimMode_ = static_cast<u8>(swimMode_ | v); break;    // mode set
        default: break;
        }
        return;
    }
    logAccess("IO?", addr, true, v);
}

u32 QuadraMachine::ioRead32(u32 addr) {
    if ((addr & 0x00FF0000u) == 0x00FF0000u) return 0xA55A2BADu;
    const u32 off = addr & 0x0003FFFFu;
    if (off >= 0x0E000u && off < 0x10000u) return djmemcRegs_[(off >> 2) & 0xF];
    if (off >= 0x10100u && off < 0x10104u) {
        const u32 hi = scsi_->dma16Read();
        const u32 lo = scsi_->dma16Read();
        return (hi << 16) | lo;
    }
    if (off >= 0x18000u && off < 0x1A000u) return iosbRegs_[(off >> 2) & 0x1F];
    if (off >= 0x0A000u && off < 0x0B100u) return sonicRegs_[(off >> 2) & 0x3F];
    // Everything else decomposes into a single device access replicated.
    const u16 hi = read16(addr);
    const u16 lo = read16(addr + 2);
    return (static_cast<u32>(hi) << 16) | lo;
}

void QuadraMachine::ioWrite32(u32 addr, u32 v) {
    if ((addr & 0x00FF0000u) == 0x00FF0000u) return;
    const u32 off = addr & 0x0003FFFFu;
    if (off >= 0x0E000u && off < 0x10000u) {
        djmemcRegs_[(off >> 2) & 0xF] = v;
        return;
    }
    if (off >= 0x10100u && off < 0x10104u) {
        if (cdbDiagBudget_ > 0) {
            --cdbDiagBudget_;
            char b[48];
            std::snprintf(b, sizeof b, "SDMA32 W %08X", v);
            if (onDiag) onDiag(b);
        }
        scsi_->dma16Write(static_cast<u16>(v >> 16));
        scsi_->dma16Write(static_cast<u16>(v));
        return;
    }
    if (off >= 0x18000u && off < 0x1A000u) {
        iosbRegs_[(off >> 2) & 0x1F] = static_cast<u16>(v);
        return;
    }
    if (off >= 0x0A000u && off < 0x0B100u) {
        sonicWrite((off >> 2) & 0x3F, static_cast<u16>(v));
        return;
    }
    write16(addr, static_cast<u16>(v >> 16));
    write16(addr + 2, static_cast<u16>(v));
}

void QuadraMachine::sonicWrite(u32 reg, u16 v) {
    if (reg == 0) {
        // CR: writing RST low releases reset; ST/STP/RXEN bits echo.
        if (!(v & 0x80)) v = static_cast<u16>(v & ~0x80);
        sonicRegs_[0] = v;
        return;
    }
    sonicRegs_[reg & 0x3F] = v;
}

// ---------------------------------------------------------------- time

void QuadraMachine::adbMaybeClock() {
    const int st = adb_->state();
    const bool canClock = st != 3 || adb_->responsePending();
    if (canClock && adbArmed_ && adbPending_ == 0) {
        adbArmed_ = false;
        adbPending_ = kAdbShiftCycles;
        adbPendingInput_ = adbArmedInput_;
    }
}

void QuadraMachine::tickDevices(int cpuCycles) {
    totalCycles_ += static_cast<u64>(cpuCycles);
    if (adbPending_ > 0) {
        adbPending_ -= cpuCycles;
        if (adbPending_ <= 0) {
            adbPending_ = 0;
            if (adbPendingInput_) {
                const u8 v = adb_->cpuShiftIn();
                via1_->completeShift(true, v);
            } else {
                const u8 v = via1_->shiftValue();
                adb_->cpuShiftOut(v);
                via1_->completeShift(false, 0);
            }
            updateIpl();
        }
    }
    // VIA1's 783.36 kHz clock is CPU/42.55 here (the Classic's was CPU/10);
    // accumulate in hundredths so the ratio holds over time.
    via1Remainder_ += cpuCycles * 100;
    while (via1Remainder_ >= 4255) {
        via1_->tick(1);
        via1Remainder_ -= 4255;
    }
    scsi_->tick(cpuCycles);
    secondAcc_ += static_cast<u64>(cpuCycles);
    if (secondAcc_ >= kCpuHz) {
        secondAcc_ -= kCpuHz;
        rtc_->tickSecond();
        via1_->setCA2(true);
        ca2PulseSlices_ = 2;
    }
    // The whole machine cadence rides device time so that frame-driven and
    // single-stepped execution behave identically: audio samples per slice,
    // the 60.15 Hz VIA1 CA1 tick, the ADB idle wake, and the DAFB vertical
    // blank all come from this accumulator.
    audioAcc_ += cpuCycles;
    while (audioAcc_ >= kCyclesPerSlice) {
        audioAcc_ -= kCyclesPerSlice;
        const u8 sample = easc_->pullSample();
        if (audioOut_.size() < 8192) audioOut_.push_back(sample);

        const int slice = sliceInFrame_;
        if (slice == 0) via1_->setCA1(true);    // tick, rising edge
        if (slice == 8) via1_->setCA1(false);
        if (ca2PulseSlices_ > 0 && --ca2PulseSlices_ == 0) via1_->setCA2(false);
        if (slice == 4) adbIdleWake();

        if (++sliceInFrame_ >= kSlicesPerFrame) {
            sliceInFrame_ = 0;
            ++frameCounter_;
            // DAFB VBL at ~66.67 Hz against the 60.15 Hz tick frame.
            vblAcc_ += 6667;
            dafb_->vblank();
            if (vblAcc_ >= 6015 * 2) {
                vblAcc_ -= 6015;
                dafb_->vblank();
            }
            vblAcc_ %= 6015;
        }
    }
    updateIpl();
}

void QuadraMachine::adbIdleWake() {
    const u32 polls = adb_->mousePolls() + adb_->kbdPolls();
    if (polls != adbLastPollTotal_) {
        adbLastPollTotal_ = polls;
        adbWakeStreak_ = 0;
    }
    if (adbPending_ == 0 && adb_->state() == 3 && adb_->wakeWorthPoking()) {
        if (adbWakeStreak_ < 32) {
            adb_->reStageLastTalk();
            via1_->completeShift(true, 0xFF);
            ++adbWakeStreak_;
        } else {
            adb_->flushStaleInput();
            adbWakeStreak_ = 0;
        }
    } else {
        adbWakeStreak_ = 0;
    }
}

int QuadraMachine::stepInstruction() {
    const int c = cpu_.step();
    tickDevices(c);
    return c;
}

void QuadraMachine::runFrame() {
    // One 60.15 Hz frame's worth of machine time; the cadence itself lives
    // in tickDevices so stepping and frame-running are identical.
    const u64 target = totalCycles_ +
                       static_cast<u64>(kSlicesPerFrame) * kCyclesPerSlice;
    while (totalCycles_ < target) {
        stepInstruction();
        if (cpu_.halted) return;
    }

    if (bootTraceFrame_ < 3000) {
        if (onDiag && (bootTraceFrame_ % 60) == 0) {
            char b[128];
            std::snprintf(b, sizeof b,
                          "boot f=%u pc=%08X sr=%04X ov=%d scsiW=%u scr=%dx%d%s",
                          bootTraceFrame_, cpu_.pc, cpu_.getSR(), overlay_ ? 1 : 0,
                          scsi_->diagWrites, screenWidth(), screenHeight(),
                          cpu_.halted ? " HALTED" : "");
            onDiag(b);
        }
        ++bootTraceFrame_;
    }
}

// ---------------------------------------------------------------- frontend

QuadraMachine::ScsiDiag QuadraMachine::scsiDiag() const {
    ScsiDiag d{};
    d.writes = scsi_->diagWrites;
    d.selects = scsi_->diagSelects;
    d.commands = scsi_->diagCommands;
    d.lastCdbLen = scsi_->lastCdbLen;
    for (int i = 0; i < 12; ++i) d.lastCdb[i] = scsi_->lastCdb[i];
    return d;
}

int QuadraMachine::screenWidth() const { return dafb_->width(); }
int QuadraMachine::screenHeight() const { return dafb_->height(); }
void QuadraMachine::renderScreen(u32* argbOut) const { dafb_->render(argbOut); }

void QuadraMachine::drainAudio(std::vector<u8>& out) {
    out = std::move(audioOut_);
    audioOut_.clear();
}

void QuadraMachine::mouseMove(int dx, int dy, bool button) {
    adb_->injectMouse(dx, dy, button);
}

void QuadraMachine::keyEvent(u8 adbCode, bool down) {
    adb_->injectKey(adbCode, down);
}

void QuadraMachine::insertHardDisk(std::vector<u8> image, bool readOnly) {
    hd_ = std::move(image);
    hdRO_ = readOnly;
    if (hd_.empty()) {
        disk_->detach();
        scsiImage_.clear();
        return;
    }
    scsiImage_ = scsi::buildAppleScsiDisk(hd_, scsi::buildScsiDriverPortable(0, 4, 1));
    hfsImageOffset_ = static_cast<u32>(scsiImage_.size() - hd_.size());
    disk_->attach(&scsiImage_, 0);
    disk_->readOnly = readOnly;
}

const std::vector<u8>& QuadraMachine::hardDiskImage() const {
    if (!scsiImage_.empty() &&
        static_cast<std::size_t>(hfsImageOffset_) + hd_.size() <= scsiImage_.size())
        hd_.assign(scsiImage_.begin() + hfsImageOffset_,
                   scsiImage_.begin() + hfsImageOffset_ + hd_.size());
    return hd_;
}

void QuadraMachine::insertHardDisk2(std::vector<u8> image, bool readOnly) {
    hd2_ = std::move(image);
    if (hd2_.empty()) {
        disk2_->detach();
        scsiImage2_.clear();
        return;
    }
    scsiImage2_ = scsi::buildAppleScsiDisk(hd2_, scsi::buildScsiDriverPortable(1, 5, 33));
    hfsImageOffset2_ = static_cast<u32>(scsiImage2_.size() - hd2_.size());
    disk2_->attach(&scsiImage2_, 1);
    disk2_->readOnly = readOnly;
}

void QuadraMachine::detachHardDisk2() {
    hd2_.clear();
    scsiImage2_.clear();
    disk2_->detach();
}

const std::vector<u8>& QuadraMachine::hardDisk2Image() const {
    if (!scsiImage2_.empty() &&
        static_cast<std::size_t>(hfsImageOffset2_) + hd2_.size() <= scsiImage2_.size())
        hd2_.assign(scsiImage2_.begin() + hfsImageOffset2_,
                    scsiImage2_.begin() + hfsImageOffset2_ + hd2_.size());
    return hd2_;
}

void QuadraMachine::attachCdRom(bool attached, int busId) {
    cdrom_->setAttached(attached, busId);
}
bool QuadraMachine::cdRomAttached() const { return cdrom_->attachedState(); }
int QuadraMachine::insertCd(std::vector<u8> image) {
    cdrom_->insert(std::move(image));
    return 1;
}
void QuadraMachine::ejectCd() { cdrom_->eject(); }
bool QuadraMachine::cdPresent() const { return cdrom_->discPresent(); }

void QuadraMachine::attachNet(bool attached, int busId) {
    netdev_->setAttached(attached, busId);
}
bool QuadraMachine::netAttached() const { return netdev_->attachedState(); }
bool QuadraMachine::netInject(const u8* frame, u32 len) {
    return netdev_->injectFrame(frame, len);
}
bool QuadraMachine::netDrain(std::vector<u8>& out) {
    return netdev_->drainFrame(out);
}

} // namespace openmac
