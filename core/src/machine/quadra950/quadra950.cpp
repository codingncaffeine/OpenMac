#include "openmac/quadra950.hpp"

#include "../iop.hpp"
#include "../quadra/dafb.hpp"
#include "../quadra/easc.hpp"
#include "../quadra/ncr53c96.hpp"
#include "../quadra/scc.hpp"
#include "../scsi.hpp"
#include "../scsicd.hpp"
#include "../via.hpp"
#include "egret.hpp"
#include "memctl.hpp"

#include <cstdio>
#include <cstring>

// Macintosh Quadra 950. See the header for scope and sources; the comments
// here name the dossier section a decision rests on.
//
// Address map (dossier, "Address map"):
//   $0000 0000-$0FFF FFFF  RAM, four banks placed by the memory controller
//   $4000 0000-$4FFF FFFF  ROM (also at zero until the first read here)
//   $5000 0000-$53FF FFFF  I/O, one $4 0000 window repeated
//     +$00000 VIA1        +$02000 VIA2        +$08000 Ethernet address PROM
//     +$0A000 SONIC       +$0C000 serial I/O processor
//     +$0E000 memory controller
//     +$0F000 SCSI 0      +$0F400 SCSI 1      (register n at +n*$10)
//     +$14000 sound       +$1E000 floppy/ADB I/O processor
//     +$28000 NuBus controller
//   $F900 0000 video RAM  $F980 0000 DAFB registers
// Everything else ends in a bus error, which is what the ROM's hardware
// detection and the Slot Manager's scan expect.

namespace openmac {

namespace {

constexpr u64 kCpuHz = 33333333;         // 68040 at 33.33 MHz
// The I/O processors' clock. The 950's I/O bus runs faster than the 900's
// (developer note); what the processors themselves are clocked at is not
// documented. The IIfx value is kept until the firmware's timing says
// otherwise (dossier, "Open points").
constexpr u64 kIopClockHz = 15667200;
constexpr int kSlicesPerFrame = 370;     // audio rides the slice cadence
constexpr int kCyclesPerSlice = 1498;    // ~60.15 Hz frame

// VIA1 port A as the ROM reads it with the port set to input: PA6, PA4,
// PA2, PA1 = 0, 1, 0, 0 identify the 950 (ROM product record: mask $56,
// match $10); PA0 high keeps the ROM out of its diagnostic mode; PA7 high.
constexpr u8 kVia1PortA = 0x91;

// The address PROM holds each byte bit-reversed.
u8 reverseBits(u8 v) {
    u8 r = 0;
    for (int i = 0; i < 8; ++i)
        if (v & (1u << i)) r = static_cast<u8>(r | (0x80u >> i));
    return r;
}

u32 be32(const u8* p) {
    return (static_cast<u32>(p[0]) << 24) | (static_cast<u32>(p[1]) << 16) |
           (static_cast<u32>(p[2]) << 8) | p[3];
}

} // namespace

Quadra950Machine::Quadra950Machine(std::vector<u8> rom, const Config& cfg)
    : MacHostServices(*this, cpu_, ram_),
      rom_(std::move(rom)),
      memctl_(std::make_unique<MemoryController950>(cfg.bankMb)),
      via1_(std::make_unique<Via6522>()),
      via2_(std::make_unique<Via6522>()),
      egret_(std::make_unique<EgretController>()),
      sccIop_(std::make_unique<IoProcessor>(kCpuHz, kIopClockHz)),
      swimIop_(std::make_unique<IoProcessor>(kCpuHz, kIopClockHz)),
      scc_(std::make_unique<Scc8530>()),
      easc_(std::make_unique<Easc>()),
      scsi0_(std::make_unique<Ncr53c96>()),
      scsi1_(std::make_unique<Ncr53c96>()),
      dafb_(std::make_unique<Dafb>()),
      cpu_(*this) {
    ram_.assign(memctl_->flatBytes(), 0);
    u32 rs = 1;
    while (rs < rom_.size()) rs <<= 1;
    rom_.resize(rs, 0xFF);
    romMask_ = rs - 1;
    romEntry_ = be32(&rom_[4]);

    // All three devices on the internal bus for now; the external bus is
    // present and empty.
    scsi0_->addTarget(disk_.get());
    scsi0_->addTarget(disk2_.get());
    scsi0_->addTarget(cdrom_.get());
    cdrom_->setStartupDrive(true);

    const u8 mac[6] = {0x08, 0x00, 0x07, 0x2A, 0x95, 0x50};
    u8 x = 0;
    for (int i = 0; i < 6; ++i) {
        macProm_[i] = reverseBits(mac[i]);
        x ^= macProm_[i];
    }
    macProm_[6] = 0;
    macProm_[7] = static_cast<u8>(x ^ 0xFF);

    wireDevices();
    reset();
}

Quadra950Machine::Quadra950Machine(std::vector<u8> rom)
    : Quadra950Machine(std::move(rom), Config{}) {}

Quadra950Machine::~Quadra950Machine() = default;

void Quadra950Machine::wireDevices() {
    // ---- VIA1 (dossier, "VIA1") ---------------------------------------------
    via1_->inA = [] { return kVia1PortA; };
    via1_->inB = [this] {
        u8 v = 0xFF;
        if (!egret_->xcvrSessionLine()) v = static_cast<u8>(v & ~0x08);   // PB3
        return v;
    };
    via1_->outB = [this](u8 value, u8 ddr) {
        // PB5 system session and PB4 VIA full count only while the port
        // drives them: at reset every line is an input and both are idle.
        const u8 driven = static_cast<u8>(value & ddr);
        egret_->setHostLines((driven & 0x20) != 0, (driven & 0x10) != 0);
        updateIpl();
    };
    egret_->hostShiftOut = [this] { return via1_->shiftValue(); };
    egret_->completeShift = [this](bool input, u8 value) {
        via1_->completeShift(input, value);
        updateIpl();
    };
    egret_->onPowerDown = [this] { powerDownRequested = true; };
    egret_->onResetSystem = [this] { resetPending_ = true; };
    egret_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };

    // ---- VIA2 (dossier, "VIA2") ---------------------------------------------
    via2_->inA = [this] {
        // PA0 Ethernet, PA1-PA5 slots $A-$E, PA6 built-in video: low = active.
        return static_cast<u8>(0xFFu & ~(dafbIrq_ ? 0x40u : 0u));
    };
    via2_->inB = [] { return static_cast<u8>(0xCF); };   // no NuBus transfer error

    // ---- I/O processors (dossier, "I/O processors") -------------------------
    sccIop_->bypassRead = [this](u32 offset) { return scc_->read(offset & 6u); };
    sccIop_->bypassWrite = [this](u32 offset, u8 value) { scc_->write(offset & 6u, value); };
    sccIop_->peripheralRead = [this](u8 reg) { return scc_->read(static_cast<u32>(reg & 3u) * 2u); };
    sccIop_->peripheralWrite = [this](u8 reg, u8 value) {
        scc_->write(static_cast<u32>(reg & 3u) * 2u, value);
    };
    sccIop_->onInterruptChange = [this] { updateIpl(); };
    // The floppy/ADB processor's peripherals arrive with the keyboard, mouse
    // and floppy: until then the SWIM reads as an open bus and the ADB line
    // idles high.
    swimIop_->peripheralRead = [](u8) { return static_cast<u8>(0xFF); };
    swimIop_->peripheralWrite = [](u8, u8) {};
    swimIop_->gpioRead = [] { return static_cast<u8>(0xFF); };
    swimIop_->gpioWrite = [](int, bool) {};
    swimIop_->onInterruptChange = [this] {
        via2_->setCA2(!swimIop_->irqAsserted());
        updateIpl();
    };

    scc_->onDiag = [this](const char* s) { if (onDiag) onDiag(s); };
    scc_->enableLoopbackAndZeroCount(true);   // the ROM's serial self-tests need both
    scc_->setClocks(kCpuHz, 7833600);
    easc_->onIrq = [this](bool level) {
        via2_->setCB1(!level);
        updateIpl();
    };
    easc_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    easc_->enableSelfTestRegisters(true);     // the ROM's sound self-test needs them
    const auto scsiIrq = [this](int chip) {
        return [this, chip](bool level) {
            scsiIrq_[chip] = level;
            via2_->setCB2(!(scsiIrq_[0] || scsiIrq_[1]));   // both chips share the line
            updateIpl();
        };
    };
    scsi0_->onIrq = scsiIrq(0);
    scsi1_->onIrq = scsiIrq(1);
    scsi0_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    scsi1_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    dafb_->onDiag = [this](const char* s) { if (onDiag) onDiag(s); };
    dafb_->onIrq = [this](bool level) {
        dafbIrq_ = level;
        via2_->setCA1(!level);
        updateIpl();
    };
    cdrom_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };

    cpu_.onResetInstruction = [this] { resetPeripherals(); };
}

// The RESET line: what the CPU's RESET instruction pulses. The I/O
// processors are left alone -- the ROM executes RESET with their firmware
// loaded -- and so is the microcontroller, which is not on that line.
void Quadra950Machine::resetPeripherals() {
    via1_->reset();
    via2_->reset();
    // Interrupt inputs idle high; set after the reset so no edge is latched.
    via2_->setCA1(true);
    via2_->setCA2(!swimIop_->irqAsserted());
    via2_->setCB1(true);
    via2_->setCB2(true);
    via2Pb7_ = via2_->pb7();
    egret_->setHostLines(false, false);
    scc_->reset();
    easc_->reset();
    scsi0_->reset();
    scsi1_->reset();
    scsiIrq_[0] = scsiIrq_[1] = false;
    updateIpl();
}

void Quadra950Machine::reset() {
    overlay_ = true;
    resetPending_ = false;
    memctl_->reset();
    for (auto& r : sonicRegs_) r = 0;
    sonicRegs_[0x29] = 6;                 // silicon revision
    sonicRegs_[0x00] = 0x0094;            // CR: RST | STP | RXDIS
    for (auto& r : nubusCtlRegs_) r = 0;
    egret_->reset();
    sccIop_->reset();
    swimIop_->reset();
    dafb_->reset();
    dafbIrq_ = false;
    resetPeripherals();
    totalCycles_ = 0;
    viaRemainder_ = 0;
    secondAcc_ = 0;
    audioAcc_ = 0;
    sliceInFrame_ = 0;
    vblAcc_ = 0;
    cpu_.reset();
}

void Quadra950Machine::updateIpl() {
    // Level 4 serial (its I/O processor), 2 VIA2, 1 VIA1 (DCD3 table 16-7).
    // In bypass mode the serial processor is transparent, and that includes
    // the SCC's interrupt: the ROM's serial interrupt self-test takes a level
    // 4 interrupt from the chip with the processor in that mode.
    int ipl = 0;
    if (sccIop_->irqAsserted() || (sccIop_->bypassed() && scc_->irqAsserted())) ipl = 4;
    else if (via2_->irqAsserted()) ipl = 2;
    else if (via1_->irqAsserted()) ipl = 1;
    cpu_.setIrqLevel(ipl);
}

// ---- RAM ------------------------------------------------------------------------

u8 Quadra950Machine::ramRead8(u32 addr) const {
    const u32 at = memctl_->translate(addr);
    return at == MemoryController950::kNoRam ? MemoryController950::kFloatingBus : ram_[at];
}

void Quadra950Machine::ramWrite8(u32 addr, u8 v) {
    const u32 at = memctl_->translate(addr);
    if (at != MemoryController950::kNoRam) ram_[at] = v;
}

// ---- bus ------------------------------------------------------------------------

u8 Quadra950Machine::read8(u32 addr) {
    if (addr < MemoryController950::kSpaceBytes) {
        if (overlay_ && addr <= romMask_) return rom_[addr];
        return ramRead8(addr);
    }
    if (addr >= 0x40000000u && addr < 0x50000000u) {
        overlay_ = false;   // the first read of the ROM proper puts RAM at zero
        return rom_[addr & romMask_];
    }
    if (addr >= 0x50000000u && addr < 0x54000000u) return ioRead8(addr & 0x3FFFFu, addr);
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) return dafb_->vramRead8(off);
        if (off >= 0x800000u && off < 0x800400u) {
            const u32 v = dafb_->read(off & 0x3FCu);
            return static_cast<u8>(v >> (8 * (3 - (off & 3))));
        }
    }
    throw BusFault{addr, true, 1};
}

void Quadra950Machine::write8(u32 addr, u8 value) {
    if (addr < MemoryController950::kSpaceBytes) {
        ramWrite8(addr, value);   // writes under the overlay reach RAM
        return;
    }
    if (addr >= 0x40000000u && addr < 0x50000000u) return;   // ROM
    if (addr >= 0x50000000u && addr < 0x54000000u) {
        ioWrite8(addr & 0x3FFFFu, value, addr);
        return;
    }
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) { dafb_->vramWrite8(off, value); return; }
        if (off >= 0x800000u && off < 0x800400u) {
            const u32 reg = off & 0x3FCu;
            const int shift = static_cast<int>(8 * (3 - (off & 3)));
            const u32 old = dafb_->read(reg);
            dafb_->write(reg, (old & ~(0xFFu << shift)) | (static_cast<u32>(value) << shift));
            return;
        }
    }
    throw BusFault{addr, false, 1};
}

u16 Quadra950Machine::read16(u32 addr) {
    if (!overlay_ && memctl_->identityMapped() && directRam(addr, 2))
        return static_cast<u16>((ram_[addr] << 8) | ram_[addr + 1]);
    if (addr >= 0x40000000u && addr < 0x50000000u) {
        overlay_ = false;
        const u32 o = addr & romMask_;
        return static_cast<u16>((rom_[o] << 8) | rom_[(o + 1) & romMask_]);
    }
    try {
        const u16 hi = read8(addr);
        return static_cast<u16>((hi << 8) | read8(addr + 1));
    } catch (const BusFault&) {
        throw BusFault{addr, true, 2};
    }
}

void Quadra950Machine::write16(u32 addr, u16 value) {
    if (memctl_->identityMapped() && directRam(addr, 2)) {
        ram_[addr] = static_cast<u8>(value >> 8);
        ram_[addr + 1] = static_cast<u8>(value);
        return;
    }
    try {
        write8(addr, static_cast<u8>(value >> 8));
        write8(addr + 1, static_cast<u8>(value));
    } catch (const BusFault&) {
        throw BusFault{addr, false, 2};
    }
}

u32 Quadra950Machine::read32(u32 addr) {
    if (!overlay_ && memctl_->identityMapped() && directRam(addr, 4)) return be32(&ram_[addr]);
    if (addr >= 0x40000000u && addr < 0x50000000u) {
        overlay_ = false;
        const u32 o = addr & romMask_;
        if (o + 3 <= romMask_) return be32(&rom_[o]);
    }
    if (addr >= 0x50000000u && addr < 0x54000000u) {
        const u32 off = addr & 0x3FFFFu;
        if (off >= 0x0E000u && off < 0x0F000u) return memctl_->read(off & 0xFFu);
        if (off >= 0x0A000u && off < 0x0C000u) return sonicRegs_[(off >> 2) & 0x3F];
        if (off >= 0x28000u && off < 0x2A000u) return nubusCtlRegs_[(off >> 2) & 7];
    }
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off + 3 < 0x200000u) return dafb_->vramRead32(off);
        if (off >= 0x800000u && off < 0x800400u) return dafb_->read(off & 0x3FFu);
    }
    try {
        const u32 hi = read16(addr);
        return (hi << 16) | read16(addr + 2);
    } catch (const BusFault&) {
        throw BusFault{addr, true, 4};
    }
}

void Quadra950Machine::write32(u32 addr, u32 value) {
    if (memctl_->identityMapped() && directRam(addr, 4)) {
        ram_[addr] = static_cast<u8>(value >> 24);
        ram_[addr + 1] = static_cast<u8>(value >> 16);
        ram_[addr + 2] = static_cast<u8>(value >> 8);
        ram_[addr + 3] = static_cast<u8>(value);
        return;
    }
    if (addr >= 0x50000000u && addr < 0x54000000u) {
        const u32 off = addr & 0x3FFFFu;
        if (off >= 0x0E000u && off < 0x0F000u) { memctl_->write(off & 0xFFu, value); return; }
        if (off >= 0x0A000u && off < 0x0C000u) {
            const u32 reg = (off >> 2) & 0x3F;
            // CR: RST stays until it is written low; the other bits echo.
            sonicRegs_[reg] = static_cast<u16>(value);
            return;
        }
        if (off >= 0x28000u && off < 0x2A000u) { nubusCtlRegs_[(off >> 2) & 7] = value; return; }
    }
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off + 3 < 0x200000u) { dafb_->vramWrite32(off, value); return; }
        if (off >= 0x800000u && off < 0x800400u) { dafb_->write(off & 0x3FFu, value); return; }
    }
    try {
        write16(addr, static_cast<u16>(value >> 16));
        write16(addr + 2, static_cast<u16>(value));
    } catch (const BusFault&) {
        throw BusFault{addr, false, 4};
    }
}

// ---- I/O window -----------------------------------------------------------------

u8 Quadra950Machine::ioRead8(u32 off, u32 addr) {
    if (off < 0x02000u) return via1_->read(static_cast<int>((off >> 9) & 0xF));
    if (off < 0x04000u) return via2_->read(static_cast<int>((off >> 9) & 0xF));
    if (off >= 0x08000u && off < 0x0A000u) return macProm_[off & 7];
    if (off >= 0x0A000u && off < 0x0C000u) {
        const u16 v = sonicRegs_[(off >> 2) & 0x3F];
        return (off & 3) == 2 ? static_cast<u8>(v >> 8) : (off & 3) == 3 ? static_cast<u8>(v) : 0;
    }
    if (off >= 0x0C000u && off < 0x0E000u) return sccIop_->read(off & 0x3Fu);
    if (off >= 0x0E000u && off < 0x0F000u) {
        const u32 v = memctl_->read(off & 0xFCu);
        return static_cast<u8>(v >> (8 * (3 - (off & 3))));
    }
    if (off >= 0x0F000u && off < 0x0F800u) {
        Ncr53c96& chip = off < 0x0F400u ? *scsi0_ : *scsi1_;
        return chip.read(static_cast<int>((off >> 4) & 0xF));
    }
    if (off >= 0x14000u && off < 0x16000u) return easc_->read(off & 0xFFFu);
    if (off >= 0x1E000u && off < 0x20000u) return swimIop_->read(off & 0x3Fu);
    if (off >= 0x28000u && off < 0x2A000u) {
        const u32 v = nubusCtlRegs_[(off >> 2) & 7];
        return static_cast<u8>(v >> (8 * (3 - (off & 3))));
    }
    throw BusFault{addr, true, 1};
}

void Quadra950Machine::ioWrite8(u32 off, u8 v, u32 addr) {
    if (off < 0x02000u) { via1_->write(static_cast<int>((off >> 9) & 0xF), v); updateIpl(); return; }
    if (off < 0x04000u) { via2_->write(static_cast<int>((off >> 9) & 0xF), v); updateIpl(); return; }
    if (off >= 0x08000u && off < 0x0A000u) return;
    if (off >= 0x0A000u && off < 0x0C000u) {
        u16& reg = sonicRegs_[(off >> 2) & 0x3F];
        if ((off & 3) == 2) reg = static_cast<u16>((reg & 0x00FF) | (v << 8));
        else if ((off & 3) == 3) reg = static_cast<u16>((reg & 0xFF00) | v);
        return;
    }
    if (off >= 0x0C000u && off < 0x0E000u) { sccIop_->write(off & 0x3Fu, v); updateIpl(); return; }
    if (off >= 0x0E000u && off < 0x0F000u) {
        const u32 reg = off & 0xFCu;
        const int shift = static_cast<int>(8 * (3 - (off & 3)));
        memctl_->write(reg, (memctl_->read(reg) & ~(0xFFu << shift)) | (static_cast<u32>(v) << shift));
        return;
    }
    if (off >= 0x0F000u && off < 0x0F800u) {
        Ncr53c96& chip = off < 0x0F400u ? *scsi0_ : *scsi1_;
        chip.write(static_cast<int>((off >> 4) & 0xF), v);
        return;
    }
    if (off >= 0x14000u && off < 0x16000u) { easc_->write(off & 0xFFFu, v); return; }
    if (off >= 0x1E000u && off < 0x20000u) { swimIop_->write(off & 0x3Fu, v); updateIpl(); return; }
    if (off >= 0x28000u && off < 0x2A000u) {
        u32& reg = nubusCtlRegs_[(off >> 2) & 7];
        const int shift = static_cast<int>(8 * (3 - (off & 3)));
        reg = (reg & ~(0xFFu << shift)) | (static_cast<u32>(v) << shift);
        return;
    }
    throw BusFault{addr, false, 1};
}

// ---- time -----------------------------------------------------------------------

void Quadra950Machine::tickDevices(int cpuCycles) {
    totalCycles_ += static_cast<u64>(cpuCycles);
    // Both VIAs count the 783.36 kHz E clock: one count per 42.55 CPU clocks.
    viaRemainder_ += cpuCycles * 100;
    while (viaRemainder_ >= 4255) {
        via1_->tick(1);
        via2_->tick(1);
        viaRemainder_ -= 4255;
        // VIA2's timer 1 output (PB7) is the 60.15 Hz tick on VIA1's CA1.
        const bool pb7 = via2_->pb7();
        if (pb7 != via2Pb7_) {
            via2Pb7_ = pb7;
            via1_->setCA1(pb7);
        }
    }
    sccIop_->tick(cpuCycles);
    swimIop_->tick(cpuCycles);
    scc_->tick(static_cast<s32>(cpuCycles));
    sccIop_->setPeripheralInterrupt(scc_->irqAsserted());
    scsi0_->tick(cpuCycles);
    scsi1_->tick(cpuCycles);

    secondAcc_ += static_cast<u64>(cpuCycles);
    if (secondAcc_ >= kCpuHz) {
        secondAcc_ -= kCpuHz;
        egret_->tickSecond();
        storageSecondTick();
    }
    egret_->poll();

    audioAcc_ += cpuCycles;
    while (audioAcc_ >= kCyclesPerSlice) {
        audioAcc_ -= kCyclesPerSlice;
        const u8 sample = easc_->pullSample();
        if (audioOut_.size() < 8192) audioOut_.push_back(sample);
        if (++sliceInFrame_ >= kSlicesPerFrame) {
            sliceInFrame_ = 0;
            ++frameCounter_;
            storageFrameTick();
            // DAFB vertical blank at ~66.67 Hz against the 60.15 Hz frame.
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

int Quadra950Machine::stepInstruction() {
    if (resetPending_) {
        // The microcontroller pulled the reset line (its "reset system").
        if (onDiag) onDiag("microcontroller: system reset");
        reset();
    }
    if (cpu_.pc == 0x4080000Au || (overlay_ && cpu_.pc == romEntry_)) machineStarting();
    if (serveStorageHooks(frameCounter_)) {
        tickDevices(40);
        return 40;
    }
    const int c = cpu_.step();
    tickDevices(c);
    return c;
}

void Quadra950Machine::runFrame() {
    const u64 target = totalCycles_ + static_cast<u64>(kSlicesPerFrame) * kCyclesPerSlice;
    while (totalCycles_ < target) {
        stepInstruction();
        if (cpu_.halted) return;
    }
}

// ---- front-end surface ----------------------------------------------------------

int Quadra950Machine::screenWidth() const { return dafb_->width(); }
int Quadra950Machine::screenHeight() const { return dafb_->height(); }
void Quadra950Machine::renderScreen(u32* argbOut) const { dafb_->render(argbOut); }

void Quadra950Machine::drainAudio(std::vector<u8>& out) {
    out.insert(out.end(), audioOut_.begin(), audioOut_.end());
    audioOut_.clear();
}

const std::array<u8, 256>& Quadra950Machine::pram() const { return egret_->pram(); }

u32 Quadra950Machine::bankBoundary(int k) const { return memctl_->boundaryBytes(k); }

// 'PRAM', a version, the 256 bytes, then the clock: the Quadra 650's layout,
// so a front end stores both machines' files the same way.
u32 Quadra950Machine::savePram(u8* out, u32 cap) const {
    if (cap < kPramBlobBytes) return 0;
    std::memcpy(out, "PRAM", 4);
    out[4] = 0; out[5] = 0; out[6] = 0; out[7] = 1;
    std::memcpy(out + 8, egret_->pram().data(), 256);
    const u32 s = egret_->seconds();
    out[264] = static_cast<u8>(s >> 24);
    out[265] = static_cast<u8>(s >> 16);
    out[266] = static_cast<u8>(s >> 8);
    out[267] = static_cast<u8>(s);
    return kPramBlobBytes;
}

bool Quadra950Machine::loadPram(const u8* data, u32 len, u32 addSeconds) {
    if (len != kPramBlobBytes || std::memcmp(data, "PRAM", 4) != 0) return false;
    if (be32(data + 4) != 1) return false;
    std::memcpy(egret_->pram().data(), data + 8, 256);
    egret_->setSeconds(be32(data + 264) + addSeconds);
    return true;
}

std::string Quadra950Machine::diagnosticReport() const {
    std::string out;
    char line[200];
    const auto add = [&](const char* fmt, auto... args) {
        std::snprintf(line, sizeof line, fmt, args...);
        out += line;
        out += '\n';
    };
    add("Quadra 950: frame %llu pc=%08X sr=%04X overlay=%d%s",
        static_cast<unsigned long long>(frameCounter_), cpu_.pc, cpu_.getSR(), overlay_ ? 1 : 0,
        cpu_.halted ? " HALTED" : cpu_.stopped ? " STOPPED" : "");
    add("   D0=%08X D1=%08X D2=%08X D3=%08X D6=%08X D7=%08X", cpu_.d[0], cpu_.d[1], cpu_.d[2],
        cpu_.d[3], cpu_.d[6], cpu_.d[7]);
    add("   A0=%08X A1=%08X A2=%08X A5=%08X A6=%08X A7=%08X", cpu_.a[0], cpu_.a[1], cpu_.a[2],
        cpu_.a[5], cpu_.a[6], cpu_.a[7]);
    std::string ring = "   recent pc:";
    for (int back = 11; back >= 0; --back) {
        std::snprintf(line, sizeof line, " %08X", cpu_.recentPc(back));
        ring += line;
    }
    out += ring + '\n';
    add("   RAM %u MB fitted, boundaries %u/%u/%u MB, %s", static_cast<unsigned>(ram_.size() >> 20),
        memctl_->boundaryBytes(0) >> 20, memctl_->boundaryBytes(1) >> 20, memctl_->boundaryBytes(2) >> 20,
        memctl_->identityMapped() ? "settled" : "not settled");
    add("   VIA1 ifr=%02X ier=%02X acr=%02X pcr=%02X orb=%02X ddrb=%02X", via1_->ifr(), via1_->ier(),
        via1_->acr(), via1_->pcr(), via1_->orb(), via1_->ddrb());
    add("   VIA2 ifr=%02X ier=%02X acr=%02X pcr=%02X orb=%02X ddrb=%02X t1=%04X", via2_->ifr(), via2_->ier(),
        via2_->acr(), via2_->pcr(), via2_->orb(), via2_->ddrb(), via2_->t1Counter());
    add("   microcontroller: %s, one-second mode %u, clock %08X", egret_->idle() ? "idle" : "busy",
        egret_->oneSecondMode(), egret_->seconds());
    add("   serial processor: %s%s status=%02X | floppy/ADB processor: %s%s status=%02X",
        sccIop_->running() ? "running" : "held", sccIop_->bypassed() ? " bypass" : "", sccIop_->status(),
        swimIop_->running() ? "running" : "held", swimIop_->firmwareAlive() ? " alive" : "",
        swimIop_->status());
    add("   video %dx%d %d bpp", dafb_->width(), dafb_->height(), dafb_->bpp());
    return out;
}

} // namespace openmac
