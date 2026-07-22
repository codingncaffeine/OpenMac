#include "openmac/machine.hpp"

#include "adb.hpp"
#include "rtc.hpp"
#include "scsi.hpp"
#include "scsiimage.hpp"
#include "via.hpp"

#include <cstdio>

namespace openmac {

namespace {
constexpr u32 kCpuHz = 7833600;
constexpr size_t kMaxLogEntries = 400;

// Device Manager / Sony disk driver structure offsets and result codes
// (Inside Macintosh: Devices / Files). Used by the replacement .Sony driver.
enum {
    ioTrap = 6, ioResult = 16, ioNamePtr = 18, ioVRefNum = 22, ioPermssn = 27, ioBuffer = 32,
    ioReqCount = 36, ioActCount = 40, ioPosOffset = 46,
    dCtlPosition = 16, dCtlQHdr = 6,
    dsWriteProt = 2, dsDiskInPlace = 3, dsInstalled = 4, dsSides = 5,
    dsQLink = 6, dsQType = 10, dsTwoSideFmt = 18, dsNewIntf = 19,
    dsMFMDrive = 22, dsMFMDisk = 23, dsTwoMegFmt = 24, SIZEOF_DrvSts = 30,
    csCode = 26, csParam = 28,
};
constexpr int kNoErr = 0, kControlErr = -17, kReadErr = -19, kWritErr = -20,
              kWPrErr = -44, kParamErr = -50, kOffLinErr = -65;
constexpr int kSonyType = 0;      // dsQType value for a Sony (floppy) drive
constexpr int kARdCmd = 2;        // low byte of ioTrap for a Read
constexpr int kSonyRefNum = -5;   // .Sony driver reference number
constexpr int kHdRefNum   = -2;   // hard disk's own driver refNum (unit-table alias)
constexpr u16 kTrapNewPtrSysClear = 0xA71E;
constexpr u16 kTrapAddDrive = 0xA04E;
constexpr u16 kTrapInsTime = 0xA058;
constexpr u16 kTrapPostEvent = 0xA02F;
constexpr u16 kTrapMountVol = 0xA00F;
constexpr u16 kTrapEject = 0xA017;
constexpr u16 kTrapOpen = 0xA000;
} // namespace

Machine::Machine(std::vector<u8> rom, const Config& cfg)
    : ram_(cfg.ramSize, 0),
      rom_(std::move(rom)),
      via_(std::make_unique<Via6522>()),
      rtc_(std::make_unique<Rtc>()),
      adb_(std::make_unique<AdbTransceiver>()),
      scsi_(std::make_unique<Ncr5380>()),
      cpu_(*this) {
    ramMask_ = cfg.ramSize - 1;
    // ROM sizes are powers of two (Classic: 512K); mirror across its window.
    u32 rs = 1;
    while (rs < rom_.size()) rs <<= 1;
    rom_.resize(rs, 0xFF);
    romMask_ = rs - 1;
    wireVia();
    if (u32 drvr = findSonyDriver()) {
        auto off = [&](u32 h) {
            return (static_cast<u32>(rom_[h & romMask_]) << 8) | rom_[(h + 1) & romMask_];
        };
        const u32 hoff = drvr & romMask_;
        sonyOpenPc_    = drvr + off(hoff + 8);
        sonyPrimePc_   = drvr + off(hoff + 10);
        sonyControlPc_ = drvr + off(hoff + 12);
        sonyStatusPc_  = drvr + off(hoff + 14);
    }
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
    //
    // PA3 is the Mac Classic's "boot the built-in ROM disk" sense line. The
    // startup code at ROM $43F770 makes PA3 an input (BCLR #3,DDRA) and, when it
    // reads LOW, stores $0B into low-memory $0CB3. That flag gates the entire
    // internal-EDisk open/boot path -- the decision point at $43F82A is
    // `CMPI.B #$0B,$0CB3; BNE skip`. With PA3 floating high the flag stays $FF,
    // so the ROM never opens the .EDisk driver (DRVR id 51), the drive queue is
    // left empty, and the machine lands on the flashing-? screen. Pull PA3 low
    // when the ROM disk is explicitly forced, or while the documented
    // Cmd-Opt-X-O startup combo is held, and the ROM boots System 6.0.3 from ROM
    // through its own driver (which scans the ROM window at $43E256 for the
    // "EDisk" signature and serves reads straight out of ROM).
    via_->inA = [this] {
        u8 v = 0xFF;
        if (forceRomDisk_ || romDiskComboHeld()) v = static_cast<u8>(v & ~0x08u);  // PA3 = 0
        return v;
    };
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
            if (onAdbEvent) onAdbEvent("state", adb_->state(), adb_->lastCommand());
            if (adb_->state() == 3) {   // idle ends the transaction: a shift
                adbArmed_ = false;      // still pending would deliver a stale
                adbPending_ = 0;        // byte and confuse the ROM's ADB manager
            }
            adbMaybeClock();   // the rule inside decides if this clocks
        }
    };
    via_->srArmed = [this](bool input) {
        adbArmed_ = true;
        adbArmedInput_ = input;
        if (onAdbEvent) onAdbEvent("arm", adb_->state(), input ? 1u : 0u);
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
    scsi_->reset();
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

bool Machine::romDiskComboHeld() const {
    return adb_->keyHeld(adbkey::kCommand) && adb_->keyHeld(adbkey::kOption) &&
           adb_->keyHeld(adbkey::kX)       && adb_->keyHeld(adbkey::kO);
}

u8 Machine::adbLastCommand() const {
    return adb_->lastCommand();
}

Machine::AdbStats Machine::adbStats() const {
    return {adb_->mousePolls(), adb_->kbdPolls(), adb_->mouseReports(),
            adb_->kbdReg2(), adb_->kbdReg3(), adb_->mouseReg3()};
}

Machine::ScsiStats Machine::scsiStats() const {
    ScsiStats s{};
    s.reads = scsi_->diagReads;
    s.writes = scsi_->diagWrites;
    s.selects = scsi_->diagSelects;
    s.commands = scsi_->diagCommands;
    s.dataInBytes = scsi_->diagDataInBytes;
    s.dataOutBytes = scsi_->diagDataOutBytes;
    for (int i = 0; i < 12; ++i) s.lastCdb[i] = scsi_->diagLastCdb[i];
    s.lastCdbLen = scsi_->diagLastCdbLen;
    return s;
}

int Machine::scsiWriteTrace(u16* out, int maxN) const {
    int n = scsi_->diagWriteTraceLen;
    if (n > maxN) n = maxN;
    for (int i = 0; i < n; ++i) out[i] = scsi_->diagWriteTrace[i];
    return n;
}

int Machine::scsiCdbHist(u8* out, int maxCdbs) const {
    int n = scsi_->diagCdbHistLen;
    if (n > maxCdbs) n = maxCdbs;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < 12; ++j) out[i * 12 + j] = scsi_->diagCdbHist[i][j];
    return n;
}

const std::vector<u8>& Machine::adbCmdTrace() const {
    return adb_->cmdTrace();
}

const std::vector<u8>& Machine::adbRespTrace() const {
    return adb_->respTrace();
}

Machine::ViaRegs Machine::viaRegs() const {
    return {via_->ora(),  via_->orb(),  via_->ddra(), via_->ddrb(),
            via_->acr(),  via_->pcr(),  via_->ifr(),  via_->ier(),
            via_->shiftValue(), via_->t1Counter(), via_->t2Counter(),
            via_->irqAsserted()};
}

u32 Machine::screenBase() const {
    return static_cast<u32>(ram_.size()) - (screenAlt_ ? 0xD900u : 0x5900u);
}

u32 Machine::soundBase() const {
    // Main sound/PWM buffer sits just below the top of RAM (alt buffer is
    // 0x5F00 lower). 370 words, one scanned per horizontal line.
    return static_cast<u32>(ram_.size()) - (screenAlt_ ? 0x5F00u : 0x0300u);
}

void Machine::drainAudio(std::vector<u8>& out) {
    out.swap(audioOut_);
    audioOut_.clear();
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
    if (addr >= 0x174 && addr <= 0x183 && !overlay_) {   // KeyMap region
        const u32 rpc = cpu_.pc;
        if (keyMapReads_ == 0) keyMapReadPc_ = rpc;
        ++keyMapReads_;
        bool seen = false;
        for (int i = 0; i < keyMapPcN_; ++i)
            if (keyMapPcs_[i] == rpc) { seen = true; break; }
        if (!seen && keyMapPcN_ < 12) keyMapPcs_[keyMapPcN_++] = rpc;
    }
    if (addr < 0x400000) {
        // The overlay maps only the ROM-sized window at zero, reads only;
        // RAM above it (and all writes) behave normally.
        if (overlay_ && addr <= romMask_) return rom_[addr];
        return ram_[addr & ramMask_];
    }
    if (addr < 0x580000) return rom_[addr & romMask_];
    if (addr < 0x600000) {          // NCR 5380 SCSI: read bank (even address)
        const u8 v = scsi_->read((addr >> 4) & 7);
        logAccess("SCSI", addr, false, v);
        return v;
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
    if (addr < 0xE00000) {          // IWM
        return iwmAccess((addr >> 9) & 0xF, false, 0);
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
    if (addr < 0x600000) {          // NCR 5380 SCSI: write bank (odd address)
        scsi_->write((addr >> 4) & 7, value);
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
    if (addr < 0xE00000) {          // IWM
        iwmAccess((addr >> 9) & 0xF, true, value);
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
    // Any non-idle state clocks when the ROM has armed the SR. Earlier this also
    // required transactionOpen() for the data states, but that stalled the ADB
    // manager: after the idle wake, the ROM sets state 1 to read pending data
    // WITHOUT re-issuing a command (so open_ is still false), and gating the
    // shift there left it spinning on an SR interrupt that never came. Mini
    // vMac's ADB_DoNewState services states 0/1/2 unconditionally; match that.
    // Stale shifts are still cancelled separately when the bus reaches idle.
    const bool canClock = st != 3;
    if (canClock && adbArmed_ && adbPending_ == 0) {
        adbArmed_ = false;
        // The ROM's ADB manager expects ~260 us between state change and the
        // shift completion (~5440 CPU cycles at 7.83 MHz). Too short and it
        // mishandles the transaction (mouse stutters, polling stalls).
        adbPending_ = 5440;
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
                if (onAdbEvent) onAdbEvent("shiftIn", adb_->state(), v);
                via_->completeShift(true, v);
            } else {
                const u8 v = via_->shiftValue();
                logAccess("ADBo", static_cast<u32>(adb_->state()), true, v);
                if (onAdbEvent) onAdbEvent("shiftOut", adb_->state(), v);
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
    if (!floppy_.empty() && trySonyTrap()) {
        tickDevices(40);
        return 40;
    }
    cpu_.setIrqLevel(via_->irqAsserted() ? 1 : 0);
    const int c = cpu_.step();
    tickDevices(c);
    return c;
}

// ---- High-level .Sony floppy driver -------------------------------------
//
// The ROM's real .Sony driver reads the physical drive through the IWM. We
// intercept its Open/Prime/Control/Status routines at their entry points and
// service a raw disk image directly, which sidesteps the IWM and the GCR/MFM
// encoding entirely (the same approach mature Mac emulators take).

void Machine::insertFloppy(std::vector<u8> image, bool readOnly) {
    if (drvStatusAddr_ != 0 && !image.empty()) {
        // Post-boot swap. The installer has already ejected the old disk (its own
        // csCode-7 eject cleared dsDiskInPlace). Drop the new image in, flag the
        // disk present, and post a disk-inserted event next frame so the disk-
        // switch re-reads it through our .Sony Prime.
        floppy_ = std::move(image);
        floppyRO_ = readOnly;
        write8(drvStatusAddr_ + dsDiskInPlace, 1);      // flag: disk present
        floppyInsertPending_ = true;
        if (onDiag) {
            char b[128];
            std::snprintf(b, sizeof b, "floppy: inserted %zu bytes (drvStatus=%06X), disk present",
                          floppy_.size(), drvStatusAddr_);
            onDiag(b);
        }
        return;
    }
    // Pre-boot (or an empty image): take effect immediately. The .Sony driver's
    // Open adds the drive later (drvStatusAddr_ == 0 until then).
    floppy_ = std::move(image);
    floppyRO_ = readOnly;
    if (onDiag) {
        char b[128];
        std::snprintf(b, sizeof b, "floppy: inserted %zu bytes pre-boot (driver Open adds the drive)",
                      floppy_.size());
        onDiag(b);
    }
}

void Machine::ejectFloppy() {
    floppy_.clear();
    if (drvStatusAddr_) write8(drvStatusAddr_ + dsDiskInPlace, 0);
}

void Machine::insertHardDisk(std::vector<u8> image, bool readOnly) {
    hd_ = std::move(image);
    hdRO_ = readOnly;
    hdStatusAddr_ = 0;   // re-added to the drive queue on the next driver Open
    // Present the volume on the SCSI bus (target ID 0) wrapped in an Apple partition
    // structure -- Driver Descriptor Map + Apple Partition Map + a driver partition --
    // so the ROM's boot scan can read a real map and driver from it. The .Sony shim
    // still does the actual mounting during the transition.
    auto driver = scsi::buildScsiDriver();
    scsiImage_ = scsi::buildAppleScsiDisk(hd_, driver);
    // The HFS volume sits after block 0 (DDM) + 3 partition-map blocks + the driver
    // blocks. Remember its byte offset so hardDiskImage() can sync SCSI writes back out.
    const std::size_t drvBlocks = driver.empty() ? 1u : (driver.size() + 511) / 512;
    hfsImageOffset_ = static_cast<u32>((4 + drvBlocks) * 512);
    scsi_->disk.attach(&scsiImage_, 0);
    scsi_->disk.readOnly = readOnly;
}

// The 16 IWM addresses each toggle one line; the odd address sets, the even
// clears. A q7-off read returns the register selected by q6/q7.
u8 Machine::iwmAccess(int reg, bool write, u8 data) {
    u8 ret = data;
    switch (reg) {
        case 0x0: iwmLines_ &= ~0x01u; break;   // ca0 off
        case 0x1: iwmLines_ |=  0x01u; break;   // ca0 on
        case 0x2: iwmLines_ &= ~0x02u; break;   // ca1 off
        case 0x3: iwmLines_ |=  0x02u; break;   // ca1 on
        case 0x4: iwmLines_ &= ~0x04u; break;   // ca2 off
        case 0x5: iwmLines_ |=  0x04u; break;   // ca2 on
        case 0x6: iwmLines_ &= ~0x08u; break;   // ca3/LSTRB off
        case 0x7: iwmLines_ |=  0x08u; break;   // ca3/LSTRB on
        case 0x8: iwmLines_ &= ~0x10u; break;   // motor off
        case 0x9: iwmLines_ |=  0x10u; break;   // motor on
        case 0xA: iwmLines_ &= ~0x20u; break;   // internal drive
        case 0xB: iwmLines_ |=  0x20u; break;   // external drive
        case 0xC: iwmLines_ &= ~0x40u; break;   // q6 off
        case 0xD: iwmLines_ |=  0x40u; break;   // q6 on
        case 0xE:                               // q7 off (read register)
            if (!write) ret = iwmReadReg();
            iwmLines_ &= ~0x80u;
            break;
        case 0xF:                               // q7 on (write mode/data)
            if (write && (iwmLines_ & 0x10) == 0) iwmMode_ = data;  // motor off: mode reg
            iwmLines_ |= 0x80u;
            break;
    }
    return ret;
}

u8 Machine::iwmReadReg() {
    switch ((iwmLines_ >> 6) & 3) {   // q6 = bit6, q7 = bit7
        case 1:  return iwmStatus();  // q6 only: Status (holds the sense bit)
        default: return 0;            // Data / Handshake: no encoded stream
    }
}

u8 Machine::iwmStatus() {
    const bool sel = ((via_->ora() >> 5) & 1) != 0;   // VIA PA5 selects the line
    const int idx = ((iwmLines_ & 0x04) ? 8 : 0) |    // ca2
                    ((iwmLines_ & 0x02) ? 4 : 0) |    // ca1
                    ((iwmLines_ & 0x01) ? 2 : 0) |    // ca0
                    (sel ? 1 : 0);
    bool high = false;   // active-low lines: false (0) = asserted
    switch (idx) {
        case 0x1:
            high = floppy_.empty();   // CSTIN: 0 = disk in place
            if (cstinLogBudget_ > 0 && onDiag) {
                --cstinLogBudget_;
                char b[64];
                std::snprintf(b, sizeof b, "CSTIN poll: %s", high ? "no-disk" : "disk-in");
                onDiag(b);
            }
            break;
        case 0x3: high = !floppyRO_;      break;   // WRPROT: 0 = protected
        case 0x5: high = false;           break;   // TK0: 0 = on track 0
        case 0xE: high = false;           break;   // INSTALLED: 0 = drive present
        default:  high = false;           break;
    }
    u8 s = high ? 0x80 : 0x00;
    if (iwmLines_ & 0x10) s |= 0x20;   // motor on
    s |= (iwmMode_ & 0x1F);            // Mode register reads back in bits 0-4
    return s;
}

u32 Machine::findSonyDriver() {
    // The .Sony DRVR resource: the Pascal name ".Sony" whose 18-bytes-earlier
    // header carries small, in-range routine offsets (distinguishing it from
    // the plain name references elsewhere in the ROM).
    static const u8 name[6] = {0x05, '.', 'S', 'o', 'n', 'y'};
    for (u32 i = 20; i + 6 < rom_.size(); ++i) {
        bool match = true;
        for (int j = 0; j < 6; ++j)
            if (rom_[i + j] != name[j]) { match = false; break; }
        if (!match) continue;
        const u32 h = i - 18;   // drvrFlags, 18 bytes before the name length byte
        auto off = [&](u32 a) { return (static_cast<u32>(rom_[a]) << 8) | rom_[a + 1]; };
        const u32 o = off(h + 8), p = off(h + 10), c = off(h + 12), s = off(h + 14);
        if (o && p && c && s && o < 0x2000 && p < 0x2000 && c < 0x2000 && s < 0x2000)
            return 0x400000u + h;
    }
    return 0;
}

bool Machine::trySonyTrap() {
    if (inSony_ || sonyPrimePc_ == 0) return false;
    const u32 pc = cpu_.pc;
    int (Machine::*fn)(u32, u32) = nullptr;
    if (pc == sonyOpenPc_)         fn = &Machine::sonyOpen;
    else if (pc == sonyPrimePc_)   fn = &Machine::sonyPrime;
    else if (pc == sonyControlPc_) fn = &Machine::sonyControl;
    else if (pc == sonyStatusPc_)  fn = &Machine::sonyStatus;
    else return false;

    inSony_ = true;
    const u32 pb = cpu_.a[0], dce = cpu_.a[1];
    if (sonyLogBudget_ > 0 && onDiag) {
        --sonyLogBudget_;
        const char* nm = fn == &Machine::sonyOpen ? "Open" :
                         fn == &Machine::sonyPrime ? "Prime" :
                         fn == &Machine::sonyControl ? "Ctrl" : "Status";
        const s16 drive = pb ? static_cast<s16>(read16(pb + ioVRefNum)) : 0;
        const u16 code = pb ? read16(pb + csCode) : 0;       // csCode for Ctrl/Status
        const u8 dip = drvStatusAddr_ ? read8(drvStatusAddr_ + dsDiskInPlace) : 0xFF;
        char b[128];
        std::snprintf(b, sizeof b, "sony %-6s drive=%d csCode=%u dsDiskInPlace=%u",
                      nm, static_cast<int>(drive), static_cast<unsigned>(code),
                      static_cast<unsigned>(dip));
        onDiag(b);
    }
    const int result = (this->*fn)(pb, dce);
    cpu_.d[0] = static_cast<u32>(static_cast<s32>(result));


    // Driver return convention (per the Mac Device Manager's IOReturn): an
    // immediate (noQueue) call sets ioResult and returns with RTS; a queued
    // call completes through IODone, which dequeues it and runs the completion
    // routine. Open is not queued I/O and always returns to its caller.
    auto doRts = [&] {
        const u32 sp = cpu_.a[7];
        cpu_.pc = read32(sp);
        cpu_.a[7] = sp + 4;
    };
    const bool immediate = pc != sonyPrimePc_ && pc != sonyControlPc_ &&
                           pc != sonyStatusPc_;                       // Open
    const bool noQueue = pb && (read16(pb + ioTrap) & 0x0200);        // noQueueBit
    const u32 ioDone = read32(0x08FC);
    if (immediate || noQueue || result > 0 || !ioDone) {
        if (pb) write16(pb + ioResult, static_cast<u16>(result > 0 ? result : 0));
        doRts();
    } else {
        cpu_.pc = ioDone;   // JMP IODone (dequeue + completion for a queued call)
    }
    inSony_ = false;
    return true;
}

void Machine::execute68kTrap(u16 trap) {
    // Place the A-line word in a scratch cell above the sound buffer, point the
    // PC at it, and step until control returns to the following word. The trap
    // dispatcher runs the routine and adjusts the return PC past the A-line.
    const u32 scratch = (static_cast<u32>(ram_.size()) - 8) & ramMask_;
    ram_[scratch]     = static_cast<u8>(trap >> 8);
    ram_[scratch + 1] = static_cast<u8>(trap & 0xFF);
    const u32 savedPc = cpu_.pc;
    const u16 savedSr = cpu_.getSR();
    cpu_.pc = scratch;
    for (int guard = 0; guard < 4000000 && cpu_.pc != scratch + 2 && !cpu_.halted; ++guard) {
        // Consult the .Sony driver intercept here too. A trap we execute (e.g.
        // _MountVol for the hard disk) reads volumes through the .Sony driver's
        // Prime entry, which must reach our C handler rather than the ROM's real
        // driver code (which delegates any drive it doesn't own -> address error).
        if ((!floppy_.empty() || !hd_.empty()) && trySonyTrap()) {
            tickDevices(40);
            continue;
        }
        cpu_.setIrqLevel(via_->irqAsserted() ? 1 : 0);
        tickDevices(cpu_.step());
    }
    cpu_.pc = savedPc;
    cpu_.setSR(savedSr);
}

int Machine::sonyOpen(u32 /*pb*/, u32 dce) {
    write32(dce + dCtlPosition, 0);
    // Queue version must be >= 3 or System 8 replaces the driver.
    write16(dce + dCtlQHdr, static_cast<u16>((read16(dce + dCtlQHdr) & 0xFF00) | 3));
    write32(0x134, 0xDEADBEEF);   // fake SonyVars pointer
    installSonyDrives();
    return kNoErr;
}

// Register the floppy (drive 2) and, if present, the hard disk (drive 3) in the
// drive queue. Reached through sonyOpen when the System opens the .Sony driver;
// also called directly for boot paths that never open it (ROM-disk boot with no
// floppy), where the ROM driver is pre-marked open so its Open routine never runs.
void Machine::installSonyDrives() {
    if (drvStatusAddr_ != 0) return;   // already installed

    // Allocate the drive-status record from the system heap.
    cpu_.d[0] = SIZEOF_DrvSts;
    execute68kTrap(kTrapNewPtrSysClear);
    if (cpu_.a[0] == 0) return;
    drvStatusAddr_ = cpu_.a[0];

    write16(drvStatusAddr_ + dsQType, static_cast<u16>(kSonyType));
    write8(drvStatusAddr_ + dsInstalled, 1);
    write8(drvStatusAddr_ + dsSides, 0xFF);       // double-sided
    write8(drvStatusAddr_ + dsTwoSideFmt, 0xFF);
    write8(drvStatusAddr_ + dsNewIntf, 0xFF);
    write8(drvStatusAddr_ + dsMFMDrive, 0xFF);    // SuperDrive
    write8(drvStatusAddr_ + dsMFMDisk, 0xFF);     // MFM disk
    write8(drvStatusAddr_ + dsTwoMegFmt, 0xFF);   // 1.44MB
    write8(drvStatusAddr_ + dsDiskInPlace, floppy_.empty() ? 0 : 1);
    write8(drvStatusAddr_ + dsWriteProt, floppyRO_ ? 0xFF : 0);

    // Add to the drive queue: D0 = (driveNum << 16) | refNum, A0 = &dsQLink.
    floppyDriveNum_ = 2;   // internal floppy
    cpu_.d[0] = (static_cast<u32>(floppyDriveNum_) << 16) |
                (static_cast<u32>(kSonyRefNum) & 0xFFFF);
    cpu_.a[0] = drvStatusAddr_ + dsQLink;
    execute68kTrap(kTrapAddDrive);

    // Install a Time Manager task, as the real .Sony Open does (ROM $434778):
    // the driver's disk-motor spin-down timer. System 6's extended Time Manager
    // patch walks tm_var+8 and re-installs every existing timer, so an empty
    // queue there address-errors it. A standalone zeroed TMTask keeps it valid;
    // the task is never Primed, so it never fires.
    cpu_.d[0] = 32;                          // >= extended TMTask size (22)
    execute68kTrap(kTrapNewPtrSysClear);     // A0 = zeroed system-heap block
    if (cpu_.a[0] != 0) {
        const u32 tmTask = cpu_.a[0];
        write32(tmTask + 6, 0x43469A);       // tmAddr -> a harmless ROM RTS
        cpu_.a[0] = tmTask;
        execute68kTrap(kTrapInsTime);        // _InsTime -> enqueues into tm_var+8
    }

    // A second, fixed drive for the hard disk, if one is mounted. Following Mini
    // vMac's SONYEMDV model: the ROM's .Sony driver only owns its floppy drives,
    // so the HD gets its OWN driver reference number (-2) and a separate unit-table
    // slot aliased to the .Sony DCE -- the ROM then dispatches the HD's I/O to the
    // same driver code we hook, which serves it from the image by drive number.
    if (!hd_.empty() && !scsiHandlesHd_) {
        cpu_.d[0] = SIZEOF_DrvSts;
        execute68kTrap(kTrapNewPtrSysClear);
        if (cpu_.a[0] != 0) {
            hdStatusAddr_ = cpu_.a[0];
            write16(hdStatusAddr_ + dsQType, static_cast<u16>(kSonyType));
            write8(hdStatusAddr_ + dsInstalled, 1);
            write8(hdStatusAddr_ + dsSides, 0xFF);
            write8(hdStatusAddr_ + dsDiskInPlace, 8);   // 8 = non-ejectable disk
            write8(hdStatusAddr_ + dsWriteProt, hdRO_ ? 0xFF : 0);
            hdDriveNum_ = floppyDriveNum_ + 1;          // drive 3
            cpu_.d[0] = (static_cast<u32>(hdDriveNum_) << 16) |
                        (static_cast<u32>(kHdRefNum) & 0xFFFF);
            cpu_.a[0] = hdStatusAddr_ + dsQLink;
            execute68kTrap(kTrapAddDrive);

            // Alias unit-table slot 1 (refNum -2, the HD) to slot 4 (refNum -5,
            // .Sony) so the HD's I/O dispatches to the .Sony driver code we hook.
            const u32 utb = read32(0x011C);             // UTableBase
            if (utb != 0) write32(utb + 4 * 1, read32(utb + 4 * 4));

            cpu_.d[0] = 80;                      // a param block for _MountVol
            execute68kTrap(kTrapNewPtrSysClear);
            hdMountPb_ = cpu_.a[0];
            hdAutoMount_ = true;   // HD configured; allow the auto-mount trigger
        }
    } else if (!hd_.empty() && scsiHandlesHd_) {
        // SCSI owns the HD: the disk's own driver (loaded by the ROM from its
        // Apple_Driver43 partition) runs _DrvrInstall + _AddDrive for the drive. We
        // only set up the deferred _MountVol trigger for that drive (number 4, matching
        // the installer), so the System mounts it through the disk driver's own Prime.
        hdDriveNum_ = floppyDriveNum_ + 2;   // drive 4
        cpu_.d[0] = 80;
        execute68kTrap(kTrapNewPtrSysClear);
        hdMountPb_ = cpu_.a[0];
        hdAutoMount_ = true;
    }
}

int Machine::sonyPrime(u32 pb, u32 dce) {
    write32(pb + ioActCount, 0);

    // Route to the drive named in the parameter block: the fixed hard disk if
    // its number matches, otherwise the floppy.
    const s16 drive = static_cast<s16>(read16(pb + ioVRefNum));
    const bool toHd = hdDriveNum_ != 0 && drive == hdDriveNum_;
    std::vector<u8>* img = toHd ? &hd_ : &floppy_;
    const u32 statusAddr = toHd ? hdStatusAddr_ : drvStatusAddr_;
    const bool ro = toHd ? hdRO_ : floppyRO_;

    if (statusAddr == 0 || read8(statusAddr + dsDiskInPlace) == 0)
        return kOffLinErr;
    if (!toHd) write8(statusAddr + dsDiskInPlace, 2);   // floppy: disk accessed

    const u32 buffer = read32(pb + ioBuffer);
    const u32 length = read32(pb + ioReqCount);
    const u32 position = read32(dce + dCtlPosition);
    if ((length & 0x1FF) || (position & 0x1FF)) return kParamErr;

    const bool isRead = (read16(pb + ioTrap) & 0xFF) == kARdCmd;
    if (isRead) {
        if (toHd) ++hdReads_;
        for (u32 i = 0; i < length; ++i) {
            const u32 src = position + i;
            write8(buffer + i, src < img->size() ? (*img)[src] : 0);
        }
        write32(0x2FC, 0);   // clear TagBuf
        write32(0x300, 0);
        write32(0x304, 0);
    } else {
        if (ro) return kWPrErr;
        if (toHd) ++hdWrites_;
        for (u32 i = 0; i < length; ++i) {
            const u32 dst = position + i;
            if (dst < img->size()) (*img)[dst] = read8(buffer + i);
        }
    }
    write32(pb + ioActCount, length);
    write32(dce + dCtlPosition, position + length);
    return kNoErr;
}

int Machine::sonyControl(u32 pb, u32 /*dce*/) {
    const u16 code = read16(pb + csCode);
    switch (code) {
        case 1:    // KillIO
            return kControlErr;
        case 7:    // eject
            if (drvStatusAddr_) write8(drvStatusAddr_ + dsDiskInPlace, 0);
            sonyLogBudget_ = 600;   // trace the disk-switch wait that follows an eject
            cstinLogBudget_ = 200;
            if (onDiag) onDiag("sony: eject (csCode 7) -> disk-switch wait begins");
            return kNoErr;
        default:   // verify / format / tag buffer / track cache: accept
            return kNoErr;
    }
}

int Machine::sonyStatus(u32 pb, u32 /*dce*/) {
    const u16 code = read16(pb + csCode);
    const s16 drive = static_cast<s16>(read16(pb + ioVRefNum));
    const bool toHd = hdDriveNum_ != 0 && drive == hdDriveNum_;
    const u32 statusAddr = toHd ? hdStatusAddr_ : drvStatusAddr_;
    switch (code) {
        case 8:    // return the drive status record
            if (statusAddr)
                for (int i = 0; i < 22; ++i)
                    write8(pb + csParam + i, read8(statusAddr + dsWriteProt + i));
            return kNoErr;
        case 6: {  // format list: one format spanning the whole medium
            const u32 blocks = toHd ? static_cast<u32>(hd_.size() / 512) : 2880u;
            write16(pb + csParam, 1);
            write32(pb + csParam + 2, blocks);
            return kNoErr;
        }
        default:
            return kNoErr;
    }
}

void Machine::runFrame() {
    ++frameCounter_;

    // Force the built-in ROM disk (EDisk) to boot. The ROM boots System 6 from
    // ROM only when TWO conditions hold at its startup boot-device check ($43F8E6):
    //   1. low-mem $0CB3 == $0B  -- gated on VIA PA3 reading low (see wireVia); and
    //   2. the KeyMap at $0174 exactly matches the Cmd-Opt-X-O bit pattern.
    // wireVia drives PA3 low when the ROM disk is forced/the combo is held; here we
    // also hold the Cmd-Opt-X-O pattern in the KeyMap through the check. We start
    // only once $0CB3 has latched to $0B (which happens right after the RAM test, so
    // the RAM test's use of low memory is never disturbed) and stop as soon as a
    // boot device is chosen (BootDrive $0210 leaves $FFFF), so the combo is gone
    // before the Finder loads and is never mistaken for a "rebuild desktop" request.
    if ((forceRomDisk_ || romDiskComboHeld()) && read8(0x0CB3) == 0x0B) {
        if (read16(0x0210) == 0xFFFF) {   // no boot device chosen yet: hold the combo
            static const u8 kCmdOptXO[16] = {
                0x80, 0x00, 0x00, 0x80,  0x00, 0x00, 0x80, 0x04,
                0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00};
            for (int i = 0; i < 16; ++i)
                write8(0x0174 + static_cast<u32>(i), kCmdOptXO[i]);
            romDiskKeymapHeld_ = true;
        } else if (romDiskKeymapHeld_) {
            // The boot device is chosen and the ROM has latched the combo. Release
            // it (clear the KeyMap) once, as a real key-up would -- otherwise the
            // still-"held" Cmd-Opt is read by the loading Finder as "rebuild the
            // desktop". (The real-key path clears $0174 itself on key-up.)
            for (int i = 0; i < 16; ++i) write8(0x0174 + static_cast<u32>(i), 0);
            romDiskKeymapHeld_ = false;
        }
    }

    // A floppy swapped in after boot. The ROM notices a disk change through the
    // drive's disk-in-place sense line (IWM CSTIN) + the DrvSts flag, which it
    // polls -- it has to see the disk leave and return. insertFloppy reports the
    // drive empty for a short window (floppyEjectSense_); here we count it down,
    // then drop the new image in and flag the disk present so the ROM runs its OWN
    // disk-inserted path (offline the old volume, mount the new one via our .Sony
    // Prime -- the same path the boot floppy took). Mounting it ourselves or
    // posting a bare diskEvt did not satisfy the "please insert the disk" modal.
    if (floppyInsertPending_ && !inSony_ && drvStatusAddr_ != 0) {
        // A bare diskEvt does not mount anything. The ROM's disk-insert interrupt does
        // two things: it _MountVols the drive, then posts a disk-inserted event carrying
        // that result. Do the same -- _MountVol reads the newly inserted volume through
        // our .Sony Prime and adds its VCB; the event tells the Finder to show the icon.
        u32 sd[8], sa[8];
        for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
        if (floppyMountPb_ == 0) {
            cpu_.d[0] = 80;                              // a param block for _MountVol
            execute68kTrap(kTrapNewPtrSysClear);
            floppyMountPb_ = cpu_.a[0];
        }
        u16 mountRes = 0;
        if (floppyMountPb_ != 0) {
            write16(floppyMountPb_ + ioVRefNum, static_cast<u16>(floppyDriveNum_));
            cpu_.a[0] = floppyMountPb_;
            execute68kTrap(kTrapMountVol);              // _MountVol the internal drive
            mountRes = static_cast<u16>(cpu_.d[0] & 0xFFFF);
        }
        cpu_.d[0] = 7;                                   // diskEvt
        cpu_.a[0] = (static_cast<u32>(mountRes) << 16) | static_cast<u16>(floppyDriveNum_);
        execute68kTrap(kTrapPostEvent);                  // hi word = mount result, lo = drive
        for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
        floppyInsertPending_ = false;
        if (onDiag) {
            char b[96];
            std::snprintf(b, sizeof b, "floppy: _MountVol(drive %d) -> %04X, diskEvt posted",
                          floppyDriveNum_, mountRes);
            onDiag(b);
        }
    }

    // Under ROM-disk boot with no floppy, the System never opens the .Sony driver
    // itself, so sonyOpen (which registers the hard disk) never runs and an attached
    // HD won't appear. Force .Sony open once the Device Manager is up, so the HD is
    // added exactly as a floppy insertion would have. Only fires while .Sony is NOT
    // already open (drvStatusAddr_ == 0), so it never runs under a floppy boot.
    if (!hd_.empty() && drvStatusAddr_ == 0 && sonyOpenPc_ != 0 && !inSony_ &&
        frameCounter_ > 1600 && (frameCounter_ % 60) == 0 &&
        read32(0x011C) != 0 && read32(0x011C) < 0x800000) {
        u32 sd[8], sa[8];
        for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
        if (sonyForceOpenPb_ == 0) {
            cpu_.d[0] = 96;
            execute68kTrap(kTrapNewPtrSysClear);
            sonyForceOpenPb_ = cpu_.a[0];
            if (sonyForceOpenPb_ != 0) {
                static const u8 nm[6] = {0x05, '.', 'S', 'o', 'n', 'y'};
                for (int i = 0; i < 6; ++i) write8(sonyForceOpenPb_ + 64 + i, nm[i]);
            }
        }
        if (sonyForceOpenPb_ != 0) {
            write32(sonyForceOpenPb_ + ioNamePtr, sonyForceOpenPb_ + 64);
            write8(sonyForceOpenPb_ + ioPermssn, 0);
            cpu_.a[0] = sonyForceOpenPb_;
            execute68kTrap(kTrapOpen);   // ensure the .Sony DCE exists (the alias target)
        }
        // The ROM driver is pre-open so _Open never ran our sonyOpen; register the
        // drives directly against the now-valid .Sony unit-table slot.
        inSony_ = true;
        installSonyDrives();
        inSony_ = false;
        for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
        if (onDiag) {
            char b[96];
            std::snprintf(b, sizeof b, "hd: ROM-boot drive install -> drvStatus=%06X hdDrive=%d",
                          drvStatusAddr_, hdDriveNum_);
            onDiag(b);
        }
    }

    // Mount the hard-disk volume once the System's file system is actually ready.
    // _MountVol enqueues the new VCB into a low-memory volume queue at $360 that
    // the System only builds when it mounts the boot floppy's volume (~cyc 250M /
    // frame ~1920). The old frame>1200 guess fired ~100M cycles before that, into
    // a still-0xFFFFFFFF queue header, and address-errored. Gate on the queue
    // being initialized instead of guessing a frame; retry every 90 frames.
    if (hdAutoMount_ && hdDriveNum_ != 0 && hdMountPb_ != 0 && !hdMounted_ &&
        diskEvtPosts_ < 15 && (frameCounter_ % 90) == 0 && !inSony_ &&
        (((static_cast<u32>(read16(0x360)) << 16) | read16(0x362)) != 0xFFFFFFFFu)) {
        // Mount the hard-disk volume once the System is up (the boot path only
        // mounts the startup floppy). Preserve the interrupted System's
        // registers -- execute68kTrap only saves PC/SR, so _MountVol would
        // otherwise clobber D0-D7/A0-A6 and the System would fault on resume.
        u32 sd[8], sa[8];
        for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
        write16(hdMountPb_ + ioVRefNum, static_cast<u16>(hdDriveNum_));
        cpu_.a[0] = hdMountPb_;
        execute68kTrap(kTrapMountVol);                 // _MountVol drive hdDriveNum_
        diskEvtResult_ = cpu_.d[0] & 0xFFFF;           // OSErr from _MountVol
        ++diskEvtPosts_;
        // 0 = mounted; 0xFFC9 = volOnLinErr (-55) = the volume is already
        // on-line (a prior async attempt mounted it). Either way we are done.
        if (diskEvtResult_ == 0 || diskEvtResult_ == 0xFFC9) hdMounted_ = true;
        for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
    }

    u64 target = totalCycles_;
    for (int line = 0; line < kLinesPerFrame; ++line) {
        // /VBL is active-low: high while the beam draws, low during blanking.
        if (line == 0) via_->setCA1(true);
        if (line == kScreenH) {
            // Once-per-frame ADB wake (like Mini vMac's 60 Hz ADB_Update): if
            // the bus is idle and a device has input pending, fire a shift
            // completion so the ROM's ADB manager resumes its poll round-robin.
            // The ROM stops ADB after startup and needs this nudge to resume.
            //
            // But the ROM only consumes a wake while it is actually autopolling.
            // During the boot-time "spin until the ADB bus is idle" loop
            // ($00BB0A) it is not, so a wake there just keeps the bus busy and
            // deadlocks the spin (a mouse nudge during boot wedged the whole
            // boot this way). So track whether our wakes produce polls: if they
            // do not for several frames, back off and flush the stale input,
            // letting the bus idle. Real desktop input is never dropped -- there
            // the ROM polls, which bumps the count and resets the streak.
            const u32 adbPolls = adb_->mousePolls() + adb_->kbdPolls();
            if (adbPolls != adbLastPollTotal_) { adbLastPollTotal_ = adbPolls; adbWakeStreak_ = 0; }
            if (adbPending_ == 0 && adb_->state() == 3 && adb_->hasPendingEvent()) {
                if (adbWakeStreak_ < 8) {
                    adb_->reStageLastTalk();
                    via_->completeShift(true, 0xFF);
                    ++adbWakeStreak_;
                } else {
                    adb_->flushStaleInput();   // ROM isn't polling; let it idle
                    adbWakeStreak_ = 0;
                }
            } else {
                adbWakeStreak_ = 0;
            }
            via_->setCA1(false);   // /VBL pulse
        }
        if (ca2PulseLines_ > 0 && --ca2PulseLines_ == 0) via_->setCA2(false);
        target += kCyclesPerLine;
        while (totalCycles_ < target) {
            stepInstruction();
            if (cpu_.halted) return;
        }

        // Sound: one 8-bit sample per scanline. The high byte of each word in
        // the buffer is the PWM level; PA0-2 scale the volume, and PB7 high
        // disables the output.
        const u8 raw = ram_[(soundBase() + static_cast<u32>(line) * 2) & ramMask_];
        const int vol = via_->ora() & 0x07;
        const bool enabled = (via_->orb() & 0x80) == 0;
        int s = 0x80;
        if (enabled && vol != 0) s = 0x80 + ((static_cast<int>(raw) - 0x80) * vol) / 7;
        if (audioOut_.size() < 8192) audioOut_.push_back(static_cast<u8>(s));
    }
}

} // namespace openmac
