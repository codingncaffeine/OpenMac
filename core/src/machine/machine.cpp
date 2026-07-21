#include "openmac/machine.hpp"

#include "adb.hpp"
#include "rtc.hpp"
#include "via.hpp"

#include <cstdio>

namespace openmac {

namespace {
constexpr u32 kCpuHz = 7833600;
constexpr size_t kMaxLogEntries = 400;
} // namespace

Machine::Machine(std::vector<u8> rom, const Config& cfg)
    : ram_(cfg.ramSize, 0),
      rom_(std::move(rom)),
      via_(std::make_unique<Via6522>()),
      rtc_(std::make_unique<Rtc>()),
      adb_(std::make_unique<AdbTransceiver>()),
      cpu_(*this) {
    ramMask_ = cfg.ramSize - 1;
    // ROM sizes are powers of two (Classic: 512K); mirror across its window.
    u32 rs = 1;
    while (rs < rom_.size()) rs <<= 1;
    rom_.resize(rs, 0xFF);
    romMask_ = rs - 1;
    wireVia();
    reset();
}

// Config lives inside Machine, so its default member initializer cannot be
// used in an in-class default argument; delegate from here where the class
// is complete.
Machine::Machine(std::vector<u8> rom) : Machine(std::move(rom), Config{}) {}

Machine::~Machine() = default;

void Machine::wireVia() {
    cpu_.onException = [this](int vector, u32 pc) {
        // Log only the crash-class exceptions; A-line/traps are normal.
        if (vector == 2 || vector == 3 || vector == 4 || vector == 8 ||
            vector == 11) {
            logAccess("EXC", pc, false, static_cast<u32>(vector));
        }
    };
    cpu_.onResetInstruction = [this] {
        // RESET asserts /RSTO to the peripherals only. It does NOT relatch the
        // boot overlay — that flip-flop is set by power-on reset and cleared by
        // the ROM via VIA PA4, and must stay as the ROM left it here.
        logAccess("RSET", cpu_.pc, true, 0);
        via_->reset();
        rtc_->reset();      // protocol state only; time and PRAM survive
        adb_->reset();
        adbArmed_ = false;
        adbPending_ = 0;
    };
    rtc_->onByte = [this](const char* what, u8 v) {
        logAccess(what, 0, false, v);   // RTC wire bytes, addr not meaningful
    };
    // Undriven port lines float high; PA0 low would mean a factory test jig.
    via_->inA = [] { return u8(0xFF); };
    via_->inB = [this] {
        u8 v = 0xFF;
        if (!rtc_->dataOut()) v = static_cast<u8>(v & ~0x01);
        if (!adb_->intLine()) v = static_cast<u8>(v & ~0x08);   // PB3
        return v;
    };
    via_->outA = [this](u8 value, u8 ddr) {
        // The boot overlay is a one-way latch: power-on sets it, the ROM's
        // first driven PA4=0 clears it, and after that PA4 can never set it
        // again (only a power-on reset does). The ROM later drives PA4 high
        // for its own purposes; the hardware ignores that for the overlay.
        if ((ddr & 0x10) && !(value & 0x10)) overlay_ = false;
        if (ddr & 0x40) screenAlt_ = (value & 0x40) == 0;   // PA6: 0 = alt buffer
    };
    via_->outB = [this](u8 value, u8 ddr) {
        const u8 eff = static_cast<u8>(value | ~ddr);
        rtc_->setLines((eff & 0x01) != 0, (eff & 0x02) != 0, (eff & 0x04) != 0);
        const int prev = adb_->state();
        adb_->setState((eff >> 4) & 3);                          // PB4/PB5
        if (adb_->state() != prev) {
            logAccess("ADBs", static_cast<u32>(adb_->state()), true, 0);
            adbMaybeClock();   // the rule inside decides if this clocks
        }
    };
    via_->srArmed = [this](bool input) {
        adbArmed_ = true;
        adbArmedInput_ = input;
        adbMaybeClock();
    };
    via_->srDisarmed = [this] {
        adbArmed_ = false;
        adbPending_ = 0;
    };
}

void Machine::reset() {
    overlay_ = true;
    via_->reset();
    rtc_->reset();
    adb_->reset();
    adbPending_ = 0;
    cpu_.reset();
    lineTarget_ = 0;
    viaRemainder_ = 0;
    secondAcc_ = 0;
}

void Machine::mouseMove(int dx, int dy, bool button) {
    adb_->injectMouse(dx, dy, button);
}

void Machine::keyEvent(u8 adbCode, bool down) {
    adb_->injectKey(adbCode, down);
}

bool Machine::keyHeld(u8 adbCode) const {
    return adb_->keyHeld(adbCode);
}

u8 Machine::adbLastCommand() const {
    return adb_->lastCommand();
}

Machine::AdbStats Machine::adbStats() const {
    return {adb_->mousePolls(), adb_->kbdPolls(), adb_->mouseReports()};
}

u32 Machine::screenBase() const {
    return static_cast<u32>(ram_.size()) - (screenAlt_ ? 0xD900u : 0x5900u);
}

void Machine::renderScreen(u32* argbOut) const {
    const u32 base = screenBase();
    for (int y = 0; y < kScreenH; ++y) {
        const u32 row = base + static_cast<u32>(y) * (kScreenW / 8);
        for (int xb = 0; xb < kScreenW / 8; ++xb) {
            const u8 bits = ram_[(row + static_cast<u32>(xb)) & ramMask_];
            for (int b = 0; b < 8; ++b) {
                const bool black = (bits & (0x80 >> b)) != 0;
                argbOut[y * kScreenW + xb * 8 + b] = black ? 0xFF000000u : 0xFFFFFFFFu;
            }
        }
    }
}

void Machine::logAccess(const char* what, u32 addr, bool write, u32 value) {
    if (accessLog_.size() >= kMaxLogEntries) {   // rolling window
        accessLog_.erase(accessLog_.begin());
    }
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s %s @%06X val=%02X pc=%06X",
                  write ? "W" : "R", what, addr, value, cpu_.pc);
    accessLog_.emplace_back(buf);
}

u8 Machine::read8(u32 addr) {
    addr &= 0xFFFFFF;
    if (addr < 0x400000) {
        // The overlay maps only the ROM-sized window at zero, reads only;
        // RAM above it (and all writes) behave normally.
        if (overlay_ && addr <= romMask_) return rom_[addr];
        return ram_[addr & ramMask_];
    }
    if (addr < 0x580000) return rom_[addr & romMask_];
    if (addr < 0x600000) {          // NCR 5380 SCSI (P4)
        logAccess("SCSI", addr, false, 0);
        return 0;
    }
    if (addr < 0x800000) {          // RAM alias while the overlay is up
        if (overlay_) return ram_[addr & ramMask_];
        logAccess("HOLE", addr, false, 0);
        return 0xFF;
    }
    if (addr < 0xC00000) {          // Z8530 SCC: control reads via pointer
        if ((addr & 0x4) == 0) {    // control (base +0/+2); +4/+6 = data
            const int reg = sccPtr_;
            sccPtr_ = 0;
            u8 v = 0;
            if (reg == 0) v = 0x04;      // RR0: transmit buffer empty
            else if (reg == 1) v = 0x01; // RR1: all sent
            return v;
        }
        logAccess("SCCd", addr, false, 0);
        return 0;
    }
    if (addr < 0xE00000) {          // IWM/SWIM (P6)
        logAccess("IWM", addr, false, 0x1F);
        return 0x1F;
    }
    if (addr < 0xF00000) {
        return via_->read((addr >> 9) & 15);
    }
    logAccess("HI", addr, false, 0);
    return 0xFF;
}

void Machine::write8(u32 addr, u8 value) {
    addr &= 0xFFFFFF;
    if (addr < 0x400000) {
        ram_[addr & ramMask_] = value;   // writes reach RAM even under overlay
        return;
    }
    if (addr < 0x580000) {
        logAccess("ROMW", addr, true, value);
        return;
    }
    if (addr < 0x600000) {
        logAccess("SCSI", addr, true, value);
        return;
    }
    if (addr < 0x800000) {
        if (overlay_) ram_[addr & ramMask_] = value;
        else logAccess("HOLE", addr, true, value);
        return;
    }
    if (addr < 0xC00000) {
        if ((addr & 0x4) == 0) {    // control write: pointer, then register
            if (sccPtr_ == 0) {
                sccPtr_ = value & 0x0F;
                if ((value & 0xF0) != 0) sccPtr_ = value & 0x0F; // commands fold in
            } else {
                sccRegs_[sccPtr_] = value;
                sccPtr_ = 0;
            }
        } else {
            logAccess("SCCd", addr, true, value);
        }
        return;
    }
    if (addr < 0xE00000) {
        logAccess("IWM", addr, true, value);
        return;
    }
    if (addr < 0xF00000) {
        via_->write((addr >> 9) & 15, value);
        return;
    }
    logAccess("HI", addr, true, value);
}

u16 Machine::read16(u32 addr) {
    return static_cast<u16>((read8(addr) << 8) | read8(addr + 1));
}

void Machine::write16(u32 addr, u16 value) {
    write8(addr, static_cast<u8>(value >> 8));
    write8(addr + 1, static_cast<u8>(value & 0xFF));
}

void Machine::adbMaybeClock() {
    const int st = adb_->state();
    const bool canClock =
        st == 0 || ((st == 1 || st == 2) && adb_->transactionOpen());
    if (canClock && adbArmed_ && adbPending_ == 0) {
        adbArmed_ = false;
        adbPending_ = 300;
        adbPendingInput_ = adbArmedInput_;
    }
}

void Machine::tickDevices(int cpuCycles) {
    totalCycles_ += static_cast<u64>(cpuCycles);
    if (adbPending_ > 0) {
        adbPending_ -= cpuCycles;
        if (adbPending_ <= 0) {
            adbPending_ = 0;
            if (adbPendingInput_) {
                const u8 v = adb_->cpuShiftIn();
                logAccess("ADBi", static_cast<u32>(adb_->state()), false, v);
                via_->completeShift(true, v);
            } else {
                const u8 v = via_->shiftValue();
                logAccess("ADBo", static_cast<u32>(adb_->state()), true, v);
                adb_->cpuShiftOut(v);
                via_->completeShift(false, 0);
            }
        }
    }
    viaRemainder_ += cpuCycles;
    if (viaRemainder_ >= 10) {
        via_->tick(viaRemainder_ / 10);
        viaRemainder_ %= 10;
    }
    secondAcc_ += static_cast<u64>(cpuCycles);
    if (secondAcc_ >= kCpuHz) {
        secondAcc_ -= kCpuHz;
        rtc_->tickSecond();
        via_->setCA2(true);          // one-second tick pulse
        ca2PulseLines_ = 2;
    }
}

int Machine::stepInstruction() {
    cpu_.setIrqLevel(via_->irqAsserted() ? 1 : 0);
    const int c = cpu_.step();
    tickDevices(c);
    return c;
}

void Machine::runFrame() {
    u64 target = totalCycles_;
    for (int line = 0; line < kLinesPerFrame; ++line) {
        // /VBL is active-low: high while the beam draws, low during blanking.
        if (line == 0) via_->setCA1(true);
        if (line == kScreenH) via_->setCA1(false);
        if (ca2PulseLines_ > 0 && --ca2PulseLines_ == 0) via_->setCA2(false);
        target += kCyclesPerLine;
        while (totalCycles_ < target) {
            stepInstruction();
            if (cpu_.halted) return;
        }
    }
}

} // namespace openmac
