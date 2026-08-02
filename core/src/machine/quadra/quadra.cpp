#include "openmac/quadra.hpp"

#include "../adb.hpp"
#include "../dc42.hpp"
#include "../macbinary.hpp"
#include "../rtc.hpp"
#include "../scsi.hpp"
#include "../scsicd.hpp"
#include "../scsiimage.hpp"
#include "../scsinet.hpp"
#include "../sony.hpp"
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

// The ROM's .Sony DRVR (header at $4086C3E0): Prime = header + $26. The
// driver executes through the 24-bit alias as well, so both faces hook.
constexpr u32 kSonyPrime = 0x4086C406u;
constexpr u32 kSonyPrimeAlias = 0x0086C406u;

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
      fd_(std::make_unique<SonyDrive>()),
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

    fd_->installed = true;      // the internal SuperDrive is always fitted
    fd_->superDrive = true;
    fd_->doubleSided = true;

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
    via1_->outA = [this](u8 value, u8 ddr) {
        // PA5 is the floppy mechanism's SEL line (the fourth sense-address
        // bit); the .Sony driver drives it while walking the drive status.
        const u8 eff = static_cast<u8>(value | ~ddr);
        floppySel_ = (eff & 0x20) != 0;
    };
    via1_->outB = [this](u8 value, u8 ddr) {
        const u8 eff = static_cast<u8>(value | ~ddr);
        rtc_->setLines((eff & 0x01) != 0, (eff & 0x02) != 0, (eff & 0x04) != 0);
        const int prev = adb_->state();
        adb_->setState((eff >> 4) & 3);   // PB4/PB5 = "newaction"
        if (adb_->state() != prev) {
            // Push model, as the real transceiver behaves: entering a data
            // state clocks the device's next byte ~700 us later whether or not
            // the CPU has armed the shift register. The System's PATCHED RAM
            // ADB manager (the one running once the desktop is up) does NOT
            // pre-read the SR, so an arm-driven input shift never completes
            // under it -- which is why the autopolled mouse reported motion
            // (12 reports) that never reached the guest. Listen transfers stay
            // CPU-driven: a pushed byte there would collide with the write.
            const int st = adb_->state();
            if ((st == 1 || st == 2) && !adb_->listening()) {
                adbPending_ = kAdbShiftCycles;
                adbPendingInput_ = true;
                adbArmed_ = false;
            } else if (st == 3 && adb_->responsePending()) {
                // The settled poll collects a flush byte at idle before it
                // reads the data states; serve it the same pushed way.
                adbPending_ = kAdbShiftCycles;
                adbPendingInput_ = true;
                adbArmed_ = false;
            } else if (st == 3) {
                adbArmed_ = false;   // idle, nothing staged: the transaction is over
                adbPending_ = 0;
            } else if (adbPendingInput_) {
                adbPending_ = 0;     // command window: cancel a stale input shift
            }
            adbMaybeClock();         // output shifts armed by SR writes still clock
        }
        updateIpl();
    };
    via1_->srArmed = [this](bool input) {
        // Input bytes are PUSHED by state changes (see outB); an SR read is
        // just the CPU collecting the latched byte. Only OUTPUT shifts -- the
        // CPU writing a command or Listen data -- arm a transfer here.
        if (input) return;
        adbArmed_ = true;
        adbArmedInput_ = false;
        adbMaybeClock();
    };
    via1_->srDisarmed = [this] { adbArmed_ = false; };

    via2_->outB = [this](u8 value, u8 ddr) {
        const u8 eff = static_cast<u8>(value | ~ddr);
        // DFAC 3-wire: PB0 latch, PB3 data, PB4 clock.
        easc_->dfacLines((eff & 0x01) != 0, (eff & 0x08) != 0, (eff & 0x10) != 0);
    };
    via2_->onIrq = [this](bool) { updateIpl(); };
    via2_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    via2_->ina = [this]() {
        // Port A carries the per-slot interrupt lines, ACTIVE LOW (an idle
        // bus reads $FF). The ROM's slot dispatcher reads this to learn which
        // source interrupted; internal video (the DAFB) rides bit 6. Serving
        // zeros here reads as "every slot at once" and the dispatcher can
        // neither identify nor acknowledge anything.
        return static_cast<u8>(0xFFu & ~(dafbIrq_ ? 0x40u : 0u));
    };

    dafb_->onIrq = [this](bool level) {
        dafbIrq_ = level;
        via2_->setSlotIrq(level);
    };
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
        if (watchLen_ && addr - watchAddr_ < watchLen_ && watchBudget_ > 0 && onDiag) {
            --watchBudget_;
            char b[64];
            std::snprintf(b, sizeof b, "WATCH w8 %08X=%02X pc=%08X", addr, value, cpu_.pc);
            onDiag(b);
        }
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
        // two data bytes (the manager bursts MOVE.W/MOVE.L from it), and
        // touching it without DRQ bus-errors (the not-ready handshake).
        if (off >= 0x10100u && off < 0x10104u) {
            if (!scsi_->drq()) throw BusFault{addr, true, 2};
            if (dataInPcBudget_ > 0 && onDiag) {
                --dataInPcBudget_;
                char b[48];
                std::snprintf(b, sizeof b, "PDMArd16 pc=%08X", cpu_.pc);
                onDiag(b);
            }
            return scsi_->dma16Read();
        }
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
        if (watchLen_ && addr - watchAddr_ < watchLen_ + 1 && watchBudget_ > 0 && onDiag) {
            --watchBudget_;
            char b[64];
            std::snprintf(b, sizeof b, "WATCH w16 %08X=%04X pc=%08X", addr, value, cpu_.pc);
            onDiag(b);
        }
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
        // The SCSI pseudo-DMA port takes both bytes of a word write, and
        // faults when the chip isn't requesting (the not-ready handshake).
        if (off >= 0x10100u && off < 0x10104u) {
            if (!scsi_->drq()) throw BusFault{addr, false, 2};
            scsi_->dma16Write(value);
            return;
        }
        // (The EASC is an 8-bit device on IOSB's byte-steered lane: a wide
        // write delivers ONE byte, like every other 8-bit peripheral here.
        // The ROM's chime player only ever byte-writes the FIFOs anyway.)
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
        if (watchLen_ && addr - watchAddr_ < watchLen_ + 3 && watchBudget_ > 0 && onDiag) {
            --watchBudget_;
            char b[72];
            std::snprintf(b, sizeof b, "WATCH w32 %08X=%08X pc=%08X", addr, value, cpu_.pc);
            onDiag(b);
        }
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
    if (off < 0x02000u) {
        const int vreg = (off >> 9) & 0xF;
        const u8 v = via1_->read(vreg);
        if (vreg == 10 && adbSrTraceBudget_ > 0) {
            --adbSrTraceBudget_;
            char b[48];
            std::snprintf(b, sizeof b, "SRread pc=%08X v=%02X", cpu_.pc, v);
            if (onDiag) onDiag(b);
        }
        return v;
    }
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
        const int sreg = (off >> 4) & 0xF;
        const u8 v = scsi_->read(sreg);
        if (sreg == 2 && dataInPcBudget_ > 0 && onDiag) {
            --dataInPcBudget_;
            char b[48];
            std::snprintf(b, sizeof b, "FIFOrd pc=%08X v=%02X", cpu_.pc, v);
            onDiag(b);
        }
        if (scsiDiagBudget_ > 0) {
            --scsiDiagBudget_;
            logAccess("SCSI", addr, false, v);
        }
        return v;
    }
    if (off >= 0x10100u && off < 0x10104u) {
        // Pseudo-DMA handshake: touching the port without DRQ is a real
        // bus error on this hardware -- the fault IS the not-ready signal,
        // and the SCSI Manager paces its blind transfers on it.
        if (!scsi_->drq()) throw BusFault{addr, true, 1};
        // A byte-wide pseudo-DMA read hands over exactly one byte.
        if (dataInPcBudget_ > 0 && onDiag) {
            --dataInPcBudget_;
            char b[48];
            std::snprintf(b, sizeof b, "PDMArd8 pc=%08X", cpu_.pc);
            onDiag(b);
        }
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
        u8 rv;
        switch (reg) {
        case 2: rv = 0; break;                 // error, reads clear
        case 3: rv = swimParams_[swimParamPtr_++ & 15]; break;
        case 4: rv = swimPhases_; break;       // presence probe reads back
        case 5: rv = swimSetup_; break;
        case 6: rv = swimMode_; break;
        case 7: {
            // Handshake: the drive's multiplexed sense line answers on bit 3.
            // Sense address = CA2:CA1:CA0:SEL -- CA lines from the phases
            // register's driven nibble, SEL from VIA1 PA5.
            const int sense = ((swimPhases_ & 4) << 1) | ((swimPhases_ & 2) << 1) |
                              ((swimPhases_ & 1) << 1) | (floppySel_ ? 1 : 0);
            rv = static_cast<u8>(0xF7 | (fd_->sense(sense, totalCycles_) ? 0x08 : 0));
            break;
        }
        default: rv = 0xFF; break;             // empty FIFO / floating
        }
        if (swimDiagBudget_ > 0) {
            --swimDiagBudget_;
            char b[64];
            std::snprintf(b, sizeof b, "SWIM R reg%d=%02X pc=%08X", reg, rv, cpu_.pc);
            if (onDiag) onDiag(b);
        }
        return rv;
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
        if (!scsi_->drq()) throw BusFault{addr, false, 1};
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
        if (swimDiagBudget_ > 0) {
            --swimDiagBudget_;
            char b[64];
            std::snprintf(b, sizeof b, "SWIM W reg%d=%02X pc=%08X", reg, v, cpu_.pc);
            if (onDiag) onDiag(b);
        }
        switch (reg) {
        case 3: swimParams_[swimParamPtr_++ & 15] = v; break;
        case 4: {
            // Phase lines: bits 0-2 = CA0-CA2, bit 3 = LSTRB. A rising
            // strobe latches a drive command -- register CA1:CA0:SEL, data
            // level CA2 (the classic Sony control convention).
            const u8 prev = swimPhasePrev_;
            swimPhases_ = v;
            swimPhasePrev_ = static_cast<u8>(v & 0xF);
            if ((v & 0x08) && !(prev & 0x08)) {
                const int reg35 = ((v & 2) << 1) | ((v & 1) << 1) |
                                  (floppySel_ ? 1 : 0);
                fd_->command(reg35, (v & 4) != 0, totalCycles_);
            }
            break;
        }
        case 5: swimSetup_ = v; break;
        case 6:
            swimMode_ = static_cast<u8>(swimMode_ & ~v);   // mode clear
            fd_->setEnabled((swimMode_ & 0x80) && ((swimMode_ >> 1) & 3) == 1,
                            totalCycles_);
            break;
        case 7:
            swimMode_ = static_cast<u8>(swimMode_ | v);    // mode set
            fd_->setEnabled((swimMode_ & 0x80) && ((swimMode_ >> 1) & 3) == 1,
                            totalCycles_);
            break;
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
        if (!scsi_->drq()) throw BusFault{addr, true, 4};
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
        if (!scsi_->drq()) throw BusFault{addr, false, 4};
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
    if (fd_->takeEjectRequest()) {
        floppy_.clear();
        fd_->removeDisk();
        if (onDiag) onDiag("floppy: guest ejected the disk");
    }
    secondAcc_ += static_cast<u64>(cpuCycles);
    if (secondAcc_ >= kCpuHz) {
        secondAcc_ -= kCpuHz;
        rtc_->tickSecond();
        via1_->setCA2(true);
        ca2PulseSlices_ = 2;
        announceHardDisks();
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
            // Claim the disk driver's Prime as soon as the ROM's boot scan has
            // installed it -- BEFORE the System's startup mount runs. Hooked
            // late, that mount goes through the SCSI Manager's discard drain,
            // fails, and the volume is written off for the rest of the session.
            if ((!hd_.empty() && !diskPrimePc_[0]) || (!hd2_.empty() && !diskPrimePc_[1]))
                findDiskDriverPrime();
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

// The ROM's .Sony Prime, served from the image at driver level: the real
// routine would run the SWIM2's MFM engine; this one moves the bytes and
// returns through jIODone exactly as the hardware path would have.
void QuadraMachine::servePrime() {
    // The System passes these with Memory Manager flag bits riding the high
    // byte; guestPtr drops them without truncating a real address (see there).
    const u32 pb = guestPtr(cpu_.a[0]);
    const u32 dce = guestPtr(cpu_.a[1]);
    const u16 trap = read16(pb + 0x06);
    const u32 buf = read32(pb + 0x20);
    const u32 req = read32(pb + 0x24);
    const u16 posMode = read16(pb + 0x2C);
    const u32 posOff = read32(pb + 0x2E);
    const u32 dctlPos = read32(dce + 0x10);

    // All four positioning modes, not just the two the File Manager usually
    // sends: a request that means "from the mark" is not the same as "at the
    // mark", and serving it as the latter reads the wrong part of the disk.
    u32 pos = dctlPos;
    switch (posMode & 3) {
    case 1: pos = posOff; break;                            // fsFromStart
    case 2: pos = static_cast<u32>(floppy_.size()) + posOff; break;   // fsFromLEOF
    case 3: pos = dctlPos + posOff; break;                  // fsFromMark
    default: break;                                         // fsAtMark
    }
    s16 result = 0;
    u32 done = 0;
    if (onDiag && primeDiagBudget_ > 0) {
        --primeDiagBudget_;
        char b[96];
        std::snprintf(b, sizeof b, "PRIME %s pos=%u req=%u buf=%08X",
                      (trap & 1) ? "write" : "read", pos, req, buf);
        onDiag(b);
    }
    if (!fd_->hasDisk()) {
        result = -65;                              // offLinErr
    } else if ((trap & 1) && fd_->readOnly) {
        result = -44;                              // wPrErr
    } else {
        const u32 size = static_cast<u32>(floppy_.size());
        if (pos >= size) {
            result = -39;                          // eofErr
        } else {
            u32 n = req;
            if (pos + n > size) { n = size - pos; result = -39; }
            if (trap & 1) {
                for (u32 i = 0; i < n; ++i) floppy_[pos + i] = read8(buf + i);
            } else {
                for (u32 i = 0; i < n; ++i) write8(buf + i, floppy_[pos + i]);
            }
            done = n;
        }
    }
    if (result != 0 && onDiag && hdErrBudget_ > 0) {
        --hdErrBudget_;
        char b[128];
        std::snprintf(b, sizeof b,
                      "floppy %s FAILED err=%d pos=%u req=%u mode=%u (size=%u)",
                      (trap & 1) ? "write" : "read", result, pos, req, posMode,
                      static_cast<u32>(floppy_.size()));
        onDiag(b);
    }
    write32(pb + 0x28, done);                      // ioActCount
    write32(dce + 0x10, pos + done);               // dCtlPosition
    write16(pb + 0x10, static_cast<u16>(result));  // ioResult (IODone rewrites)
    cpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    // Return through jIODone: A1 = DCE, D0 = result, jump via ($08FC).
    const u32 target = read32(0x08FC);
    cpu_.a[0] = target;
    cpu_.pc = target;
}

// Is a volume already on line for this drive? Walk the VCB queue ($0356 is the
// header, qHead at $0358); each VCB carries its drive number at vcbDrvNum
// (+$6E). This is how we tell "the System mounted it itself" from "nobody has".
bool QuadraMachine::volumeMountedFor(u16 drive) {
    u32 vcb = guestPtr(read32(0x0358));
    for (int n = 0; vcb && vcb + 80 < ram_.size() && n < 16; ++n) {
        if (read16(vcb + 72) == drive) return true;   // vcbDrvNum
        vcb = guestPtr(read32(vcb));                  // qLink
    }
    return false;
}

// Locate the Prime entry of each installed .ScsiHD driver, so the machine can
// serve its requests directly. The unit table ($011C) holds a handle per unit;
// the DCE's dCtlDriver points at the DRVR, whose header carries the Prime
// offset. Cached once the System has installed the driver.
void QuadraMachine::findDiskDriverPrime() {
    static const int kUnits[2] = {1, 33};
    const u32 uTable = guestPtr(read32(0x011C));
    if (!uTable || uTable + 256 >= ram_.size()) return;
    for (int i = 0; i < 2; ++i) {
        if (diskPrimePc_[i]) continue;
        const u32 h = guestPtr(read32(uTable + static_cast<u32>(kUnits[i]) * 4u));
        if (!h || h + 4 >= ram_.size()) continue;
        const u32 dce = guestPtr(read32(h));
        if (!dce || dce + 4 >= ram_.size()) continue;
        const u32 drvr = guestPtr(read32(dce));       // dCtlDriver
        if (!drvr || drvr + 0x20 >= ram_.size()) continue;
        // Only OUR driver: its name field is ".ScsiHD".
        if (read8(drvr + 0x12) != 7 || read8(drvr + 0x13) != '.' ||
            read8(drvr + 0x14) != 'S' || read8(drvr + 0x15) != 'c')
            continue;
        const u32 prime = drvr + read16(drvr + 0x0A);
        diskPrimePc_[i] = prime;
        if (onDiag) {
            char b[72];
            std::snprintf(b, sizeof b, "hd: driver unit %d Prime at %08X",
                          kUnits[i], prime);
            onDiag(b);
        }
    }
}

// The on-disk SCSI driver's Prime, served from the image the way the .Sony
// hook serves floppies. The ROM's old-API SCSI Manager takes our driver's
// read down its DISCARD drain (measured: every byte of a correct MDB read out
// at ROM $D1CCC and thrown away), so the volume never mounts. Moving the
// bytes here keeps the guest's own driver, DCE and completion path intact --
// only the transfer itself is done by the machine -- and gives the writes the
// installer needs as well.
void QuadraMachine::serveDiskPrime(int unit) {
    const u32 pb = guestPtr(cpu_.a[0]);
    const u32 dce = guestPtr(cpu_.a[1]);
    const u16 trap = read16(pb + 0x06);
    const u32 buf = guestPtr(read32(pb + 0x20));
    const u32 req = read32(pb + 0x24);
    const u16 posMode = read16(pb + 0x2C);
    const u32 posOff = read32(pb + 0x2E);
    const u32 dctlPos = read32(dce + 0x10);

    std::vector<u8>& img = unit ? hd2_ : hd_;
    const bool ro = unit ? false : hdRO_;
    const u32 base = (unit ? hfsImageOffset2_ : hfsImageOffset_);
    std::vector<u8>& backing = unit ? scsiImage2_ : scsiImage_;

    u32 pos = dctlPos;
    switch (posMode & 3) {
    case 1: pos = posOff; break;                                   // fsFromStart
    case 2: pos = static_cast<u32>(img.size()) + posOff; break;    // fsFromLEOF
    case 3: pos = dctlPos + posOff; break;                         // fsFromMark
    default: break;                                                // fsAtMark
    }
    s16 result = 0;
    u32 done = 0;
    if (img.empty() || backing.empty()) {
        result = -65;                                  // offLinErr
    } else if ((trap & 1) && ro) {
        result = -44;                                  // wPrErr
    } else {
        const u32 size = static_cast<u32>(img.size());
        if (pos >= size) {
            result = -39;                              // eofErr
        } else {
            u32 n = req;
            if (pos + n > size) { n = size - pos; result = -39; }
            // The guest addresses the VOLUME; the backing store is the whole
            // partitioned disk, so the partition offset goes on here.
            const u32 at = base + pos;
            if (trap & 1) {
                for (u32 i = 0; i < n; ++i) backing[at + i] = read8(buf + i);
            } else {
                for (u32 i = 0; i < n; ++i) write8(buf + i, backing[at + i]);
            }
            done = n;
        }
    }
    if (trap & 1) ++hdWriteCount_; else ++hdReadCount_;
    if (result != 0 && onDiag && hdErrBudget_ > 0) {
        --hdErrBudget_;
        char b[128];
        std::snprintf(b, sizeof b,
                      "hd %s FAILED err=%d pos=%u req=%u buf=%08X (size=%u)",
                      (trap & 1) ? "write" : "read", result, pos, req, buf,
                      static_cast<u32>(img.size()));
        onDiag(b);
    }
    write32(pb + 0x28, done);                          // ioActCount
    write32(dce + 0x10, pos + done);                   // dCtlPosition
    write16(pb + 0x10, static_cast<u16>(result));      // ioResult
    cpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const u32 target = read32(0x08FC);                 // jIODone
    cpu_.a[0] = target;
    cpu_.pc = target;
}

// Run one A-line trap to completion on the guest CPU: the opcode goes in a
// scratch cell at the top of RAM, the PC points at it, and stepping continues
// until control returns to the following word (the dispatcher adjusts the
// return PC past the A-line). The Classic's proven injection shape.
void QuadraMachine::execute68kTrap(u16 trap) {
    // ApplScratch, the twelve low-memory bytes reserved for applications. The
    // Classic put this word at the top of RAM, which works on a machine with
    // no MMU -- but here the '040 runs the System's own address space, and
    // fetching from the far top of physical RAM lands somewhere the guest has
    // not mapped: the CPU faults, the guard loop spins on the fault, and the
    // machine ends up executing address 10. Low memory is identity-mapped on
    // every Macintosh, because the vector table and the globals live there.
    const u32 scratch = 0x0A78;
    write8(scratch, static_cast<u8>(trap >> 8));
    write8(scratch + 1, static_cast<u8>(trap & 0xFF));
    const u32 savedPc = cpu_.pc;
    const u16 savedSr = cpu_.getSR();
    cpu_.pc = scratch;
    for (int guard = 0; guard < 64000000 && cpu_.pc != scratch + 2 && !cpu_.halted;
         ++guard) {
        stepInstruction();
    }
    cpu_.pc = savedPc;
    cpu_.setSR(savedSr);
}

// A drive that has sat in the queue since the ROM scan never gets mounted:
// its disk-inserted announcement fell in the window startup flushes. Posting
// the event once the System is up makes the insertion the same event a user's
// insertion is -- the System mounts the volume and the Finder shows it.
void QuadraMachine::announceHardDisks() {
    if (announceInFlight_) return;
    if (!hdAnnouncePending_ && !hd2AnnouncePending_) return;
    if (read32(0x0358) == 0) return;              // no live VCB: System not up
    if (read8(0x0360) & 1) return;                // FSBusy: a file op is running
    if ((cpu_.getSR() & 0x0700) != 0) return;     // mid-interrupt: not now
    // Never from inside a driver request we are serving: the guest's own
    // driver call is still in flight and a nested File Manager call there is
    // the re-entrancy the Classic learned to refuse.
    if (inDriver_) return;
    // One second's grace for the System's own startup mount, then do it
    // ourselves. Waiting longer means an application is already running and
    // has enumerated its disks -- the 7.5 Installer builds its destination
    // list at launch, so a volume mounted after that never appears in it.
    if (++hdAnnounceDelay_ < 2) return;
    // If the System already mounted the volume by itself -- which it does once
    // the driver's Prime is hooked before startup -- there is nothing to
    // announce. Injecting anyway disturbs whatever is running: the 7.5
    // Installer loses its window and hangs on a watch cursor (measured).
    if (volumeMountedFor(4)) hdAnnouncePending_ = false;
    if (volumeMountedFor(5)) hd2AnnouncePending_ = false;
    if (!hdAnnouncePending_ && !hd2AnnouncePending_) return;
    // The File Manager's own queue must be initialised, or _MountVol enqueues
    // into a header the System has not built yet (the Classic address-errored
    // on exactly this).
    if ((((static_cast<u32>(read16(0x0360)) << 16) | read16(0x0362)) == 0xFFFFFFFFu))
        return;
    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
    // A param block from the System heap, not a corner of RAM we picked: the
    // trap keeps it only for the call, but the heap is the one place nothing
    // else claims. Allocated once, reused for every retry.
    if (!hdMountPb_) {
        cpu_.d[0] = 80;
        execute68kTrap(0xA71E);                   // _NewPtr,Sys,Clear
        hdMountPb_ = cpu_.a[0] & 0x00FFFFFFu;
    }
    // Mount the volume, THEN announce it. Posting the event alone is not
    // enough: on a real insertion the File Manager mounts the volume before
    // the driver's event reaches the application, and the message carries the
    // drive number in its high word with the mount's result in the low word.
    // The Finder recovers from a bare event by mounting the drive itself,
    // which is why the desktop showed the disk -- but the Installer runs in
    // the Finder's place off the install floppy and does not, so the disk it
    // was told about was never a volume it could see.
    auto mountDrive = [&](u16 drive) -> bool {
        if (!hdMountPb_) return false;
        for (u32 i = 0; i < 80; ++i) write8(hdMountPb_ + i, 0);
        write16(hdMountPb_ + 22, drive);          // ioVRefNum = drive number
        cpu_.a[0] = hdMountPb_;
        execute68kTrap(0xA00F);                   // _MountVol
        const s16 res = static_cast<s16>(cpu_.d[0] & 0xFFFF);
        cpu_.a[0] = 7;                            // diskEvt
        cpu_.d[0] = (static_cast<u32>(drive) << 16) | static_cast<u16>(res);
        execute68kTrap(0xA02F);                   // _PostEvent
        if (onDiag) {
            char b[72];
            std::snprintf(b, sizeof b, "hd: drive %u mount result %d", drive, res);
            onDiag(b);
        }
        // 0 = mounted, -55 = volOnLinErr = already on line. Either way, done.
        return res == 0 || res == -55;
    };
    ++hdMountTries_;
    if (hdAnnouncePending_ && mountDrive(4)) hdAnnouncePending_ = false;
    if (hd2AnnouncePending_ && mountDrive(5)) hd2AnnouncePending_ = false;
    if (hdMountTries_ >= 15) { hdAnnouncePending_ = hd2AnnouncePending_ = false; }
    for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
}

int QuadraMachine::stepInstruction() {
    if (countPc_ && (cpu_.pc | 0x40000000u) == (countPc_ | 0x40000000u)) ++countPcHits_;
    // The ROM's .Sony driver runs through both its 32-bit face and the 24-bit
    // alias; hook Prime at either and serve the request from the image.
    if ((diskPrimePc_[0] && cpu_.pc == diskPrimePc_[0]) ||
        (diskPrimePc_[1] && cpu_.pc == diskPrimePc_[1])) {
        const int unit = (diskPrimePc_[0] && cpu_.pc == diskPrimePc_[0]) ? 0 : 1;
        inDriver_ = true;
        try { serveDiskPrime(unit); } catch (const BusFault&) {
            cpu_.d[0] = static_cast<u32>(-36);
            cpu_.pc = read32(0x08FC);
            cpu_.a[0] = cpu_.pc;
        }
        inDriver_ = false;
        tickDevices(40);
        return 40;
    }
    if (cpu_.pc == kSonyPrime || cpu_.pc == kSonyPrimeAlias) {
        try {
            servePrime();
        } catch (const BusFault&) {
            // A malformed parameter block must not take the emulator down;
            // answer as a driver would and let the caller cope.
            cpu_.d[0] = static_cast<u32>(-36);   // ioErr
            cpu_.pc = read32(0x08FC);
            cpu_.a[0] = cpu_.pc;
        }
        tickDevices(40);
        return 40;
    }
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

u32 QuadraMachine::adbMousePolls() const { return adb_->mousePolls(); }
u32 QuadraMachine::adbKbdPolls() const { return adb_->kbdPolls(); }
u32 QuadraMachine::adbMouseReports() const { return adb_->mouseReports(); }
bool QuadraMachine::dafbVblEnabled() const { return dafb_->vblIntEnabled(); }
u32 QuadraMachine::dafbSwatchReg(int i) const { return dafb_->swatchReg(i); }
u32 QuadraMachine::adbMouseBytesRead() const { return adb_->mouseBytesRead(); }
std::vector<u8> QuadraMachine::adbMouseBytesLog() const { return adb_->mouseBytesLog(); }
void QuadraMachine::adbClearCmdTrace() { adb_->clearCmdTrace(); }
std::vector<u8> QuadraMachine::adbCmdTrace() const { return adb_->cmdTrace(); }

int QuadraMachine::insertFloppy(std::vector<u8> image, bool readOnly) {
    // Peel the wrappers the archived media wears -- MacBinary around DiskCopy
    // 4.2 around the sectors, either or both -- down to the raw disk data the
    // .Sony Prime hook serves. Most 7.1 install floppies are DiskCopy 4.2: an
    // 84-byte header in front of the sectors, so a 1.44 MB disk is 1,474,644
    // bytes on disk rather than 1,474,560, and read raw every sector lands 84
    // bytes out of place and nothing mounts.
    const char* container = "raw image";
    if (macbinary::isMacBinary(image)) {
        macbinary::split(image);
        container = "MacBinary";
    }
    if (dc42::isDiskCopy(image)) {
        dc42::split(image);
        container = "DiskCopy 4.2";
    }

    // The three geometries a Macintosh SuperDrive frames. Size alone settles
    // the medium once the wrappers are off.
    const std::size_t n = image.size();
    const bool hd = n == 1474560;
    const bool dsGCR = n == 819200;   // 800K double-sided GCR
    const bool ssGCR = n == 409600;   // 400K single-sided GCR
    if (!hd && !dsGCR && !ssGCR) {
        if (onDiag) {
            char b[128];
            std::snprintf(b, sizeof b,
                          "floppy refused: %zu bytes (%s) is no floppy geometry -- "
                          "need 400K/800K/1.44 MB of sectors", n, container);
            onDiag(b);
        }
        return 0;
    }

    floppy_ = std::move(image);
    fd_->doubleSided = !ssGCR;
    fd_->insert(&floppy_, readOnly, hd);
    if (onDiag) {
        char b[128];
        std::snprintf(b, sizeof b, "floppy: %s disk in the internal drive (%s)",
                      hd ? "1.44 MB" : dsGCR ? "800K" : "400K", container);
        onDiag(b);
    }
    return 1;
}

void QuadraMachine::ejectFloppy() {
    fd_->removeDisk();
    floppy_.clear();
}

bool QuadraMachine::floppyPresent() const { return fd_->hasDisk(); }

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
    hdAnnouncePending_ = true;
    hdAnnounceDelay_ = 0;
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
    hd2AnnouncePending_ = true;
    hdAnnounceDelay_ = 0;
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
