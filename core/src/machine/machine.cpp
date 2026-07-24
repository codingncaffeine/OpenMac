#include "openmac/machine.hpp"

#include "adb.hpp"
#include "iwm.hpp"
#include "rtc.hpp"
#include "gcr.hpp"
#include "mfm.hpp"
#include "scsi.hpp"
#include "sony.hpp"
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
constexpr u16 kTrapUnmountVol = 0xA00E;
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
      iwm_(std::make_unique<Iwm>()),
      drive0_(std::make_unique<SonyDrive>()),
      drive1_(std::make_unique<SonyDrive>()),
      driveBay2_(std::make_unique<SonyDrive>()),
      cpu_(*this) {
    // The Classic has one internal 1.4 MB SuperDrive; the external port is empty.
    // The internal mechanism always reads its medium out of floppy_, so an empty
    // floppy_ simply means no disk is in the drive.
    drive0_->installed = true;
    drive0_->image = &floppy_;
    drive1_->installed = false;
    driveBay2_->installed = false;   // the Classic has one internal drive
    // Report an 800K double-sided mechanism, not a SuperDrive, until the SWIM's
    // ISM/MFM path exists. Drive status line $A is what the ROM asks: answering
    // "SuperDrive" sends its .Sony driver down the MFM path and it stalls before
    // it ever spins the motor. Answering "plain GCR drive" makes it proceed.
    drive0_->superDrive = false;
    ramMask_ = cfg.ramSize - 1;
    // ROM sizes are powers of two (Classic: 512K); mirror across its window.
    u32 rs = 1;
    while (rs < rom_.size()) rs <<= 1;
    rom_.resize(rs, 0xFF);
    romMask_ = rs - 1;
    wireVia();
    findGcrErrorSites(findSonyDriver());
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
        if (ddr & 0x40) screenAlt_ = (value & 0x40) == 0;   // PA6 (vPage2): 0 = alt screen buffer
        if (ddr & 0x08) soundAlt_  = (value & 0x08) == 0;   // PA3 (vSndPg2): 0 = alt sound buffer (independent of video)
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
    iwm_->reset();
    drive0_->reset();
    drive1_->reset();
    driveBay2_->reset();
    lstrbPrev_ = false;
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

void Machine::postMouseButton(bool down) {
    // _PostEvent: A0 = event code (mouseDown=1 / mouseUp=2), D0 = message (0 for
    // mouse). PostEvent fills evtQWhere from the low-mem mouse location and
    // evtQWhen from the tick count, so position the cursor first (mouseMove).
    cpu_.a[0] = down ? 1u : 2u;
    cpu_.d[0] = 0;
    execute68kTrap(0xA02F);
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
    // 0x5F00 lower), selected by PA3 (vSndPg2) -- INDEPENDENT of the video page.
    // Tying this to screenAlt_ made screen double-buffering (e.g. Dark Castle
    // flipping PA6 every frame) drag the sound read to the unwritten alt buffer,
    // producing the garbage/chopping heard in-game. 370 words, one per scanline.
    return static_cast<u32>(ram_.size()) - (soundAlt_ ? 0x5F00u : 0x0300u);
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
    // One unsigned compare to see whether the ROM's GCR reader is about to give
    // up on a sector, and if so which check failed. Always on: it costs nothing
    // and it is the only direct read on whether our nibble stream is right.
    if (gcrErrSpan_ && (cpu_.pc - gcrErrLo_) <= gcrErrSpan_) noteGcrError(cpu_.pc);
    if (!sonyShim_ && sonyPrimePc_ && cpu_.pc == sonyPrimePc_) watchSonyPrime();
    if (!sonyShim_ && sonyResultPc_ && cpu_.pc == sonyResultPc_) watchSonyResult();
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
    flushFloppyTrack();   // whatever the driver wrote to the outgoing disk
    if (drvStatusAddr_ != 0 && !image.empty()) {
        // Post-boot swap: stage the new image but keep the outgoing disk in place. The
        // mount trigger (runFrame) unmounts the outgoing volume first -- while its image
        // is still current, so its data flushes -- then seats and mounts the new disk.
        // Skipping the unmount leaves the old volume on the drive, so _MountVol returns
        // volOnLinErr (or the stale volume masks the new one) and nothing appears.
        floppyPending_ = std::move(image);
        floppyPendingRO_ = readOnly;
        floppyInsertPending_ = true;
        if (onDiag) {
            char b[128];
            std::snprintf(b, sizeof b, "floppy: swap staged (%zu bytes) -- unmount old, then mount new",
                          floppyPending_.size());
            onDiag(b);
        }
        return;
    }
    // Pre-boot (or an empty image): take effect immediately. The .Sony driver's
    // Open adds the drive later (drvStatusAddr_ == 0 until then).
    floppy_ = std::move(image);
    ++floppyGen_;
    floppyRO_ = readOnly;
    if (onDiag) {
        char b[128];
        std::snprintf(b, sizeof b, "floppy: inserted %zu bytes pre-boot (driver Open adds the drive)",
                      floppy_.size());
        onDiag(b);
    }
}

void Machine::ejectFloppy() {
    flushFloppyTrack();
    // A floppy that mounted after boot must be UNMOUNTED (its VCB dropped) to leave the
    // desktop -- merely reporting the disk out keeps the volume on-line and the Finder
    // asking for it back. Defer to a clean frame boundary; keep the image until then so
    // the unmount's flush lands on the right disk. Pre-boot / empty drive: clear now.
    if (drvStatusAddr_ != 0 && !floppy_.empty()) {
        floppyEjectPending_ = true;
        return;
    }
    floppy_.clear();
    ++floppyGen_;
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

void Machine::setSwimEnabled(bool on) { iwm_->swimEnabled = on; }
bool Machine::iwmInIsmMode() const { return iwm_->ismSelected(); }

void Machine::setSonyLineOverride(u16 mask, u16 value) {
    drive0_->overrideMask = mask;
    drive0_->overrideValue = value;
}

u32 Machine::sonyCommandCount(int drive, int reg) const {
    return sonyCmds_[drive & 1][reg & 7];
}

// Which mechanism the controller currently addresses.
//
// The IWM's drive-select latch only chooses between the internal port and the
// external one. The internal port is the SE's two-bay design, and which bay
// answers is picked by VIA PA4: the driver's drive-select routine drives it
// high for the first internal drive and low for the second ($43F806, reached
// through $436C20 from $435534). The Classic ships one internal mechanism, so
// the other bay has to read as empty -- otherwise the ROM's drive scan finds
// the same drive twice and registers a floppy drive that is not there, and the
// System then chases the same disk through two drive-queue entries.
//
// The mechanism sits on the bay PA4 selects when it is low, which is also where
// PA4 rests: the same pin is the boot overlay latch, and the driver deliberately
// leaves it alone while the ROM disk is in use ($43F806 returns early when
// $0CB3 says so), so a drive that needed PA4 driven high would be unreachable
// under a ROM-disk boot.
SonyDrive& Machine::selectedDrive() {
    if (iwm_->externalDrive()) return *drive1_;
    return ((via_->ora() >> 4) & 1) ? *driveBay2_ : *drive0_;
}

// Keep the nibble stream under the head in step with the head position, the
// side the SEL line selects and the medium in the drive. Encoding a track costs
// real work, so it is only redone when one of those actually changes.
void Machine::iwmUpdateTrack() {
    SonyDrive& d = selectedDrive();
    // SEL is multiplexed: it addresses the status registers AND selects the head.
    // The drive latches the head when the phase lines address the read-data line
    // (RDDATA0 = lower, RDDATA1 = upper -- Inside Macintosh III-35), so follow it
    // only then rather than on every SEL change, which would thrash the surface.
    const int addr = iwmDriveReg();
    if ((addr & 0xE) == 0x8) d.headUpper = (addr & 1) != 0;
    if (&d != drive0_.get() || !d.hasDisk()) return;
    // High-density media is MFM, and MFM is framed by the SWIM in hardware, not
    // by this byte-level GCR surface. A real SuperDrive in IWM/GCR mode gets
    // nothing off such a disk, so neither does the driver: laying a GCR track
    // over an MFM image would let it read a plausible-looking prefix and, once
    // there is a write path, lay GCR sectors over a 1.4 MB volume. Those disks
    // stay with the high-level driver until the ISM path exists.
    if (d.hdMedia) {
        // High-density media is MFM: framed by the SWIM in hardware, not by the
        // byte-level GCR surface. Without the ISM path there is nothing to hand
        // the driver -- laying a GCR track over an MFM image would let it read a
        // plausible prefix and, with a write path, ruin a 1.4 MB volume.
        if (!iwm_->swimEnabled) {
            d.invalidateTrack();
            if (hdMediaLog_ > 0 && onDiag) {
                --hdMediaLog_;
                onDiag("sony: 1.4MB media needs the SWIM's MFM path; the GCR surface "
                       "stays blank");
            }
            return;
        }
        const int mside = d.headUpper ? 1 : 0;
        if (d.trackLoaded() && trackCacheTrack_ == d.track &&
            trackCacheSide_ == mside && trackCacheGen_ == floppyGen_)
            return;
        flushFloppyTrack();
        if (d.track < 0 || d.track >= mfm::kTracks) { d.invalidateTrack(); return; }
        std::vector<u8> bytes, marks;
        mfm::buildTrack(floppy_, d.track, mside, bytes, marks);
        d.setTrackData(std::move(bytes), std::move(marks));
        trackCacheTrack_ = d.track;
        trackCacheSide_  = mside;
        trackCacheGen_   = floppyGen_;
        if (trackLogBudget_ > 0 && onDiag) {
            --trackLogBudget_;
            char b[96];
            std::snprintf(b, sizeof b, "sony: MFM track %d side %d framed (%zu bytes)",
                          d.track, mside, d.trackData().size());
            onDiag(b);
        }
        return;
    }
    const int side = d.headUpper ? 1 : 0;
    const int sides = floppy_.size() > 440u * 1024u ? 2 : 1;
    const u8 format = sides == 2 ? 0x22 : 0x02;
    // Keyed on the medium's generation, not its length: swapping in a different
    // image of the same size would otherwise leave the old track under the head.
    if (d.trackLoaded() && trackCacheTrack_ == d.track &&
        trackCacheSide_ == side && trackCacheGen_ == floppyGen_)
        return;
    // The head is about to move off a track the driver wrote to, so put what it
    // wrote back into the image first.
    flushFloppyTrack();
    if (d.track < 0 || d.track > 79 || side >= sides) { d.invalidateTrack(); return; }
    d.setTrackData(gcr::buildTrack(floppy_, d.track, side, sides, format));
    trackCacheTrack_ = d.track;
    trackCacheSide_  = side;
    trackCacheGen_   = floppyGen_;
    if (trackLogBudget_ > 0 && onDiag) {
        --trackLogBudget_;
        char b[96];
        std::snprintf(b, sizeof b, "sony: track %d side %d encoded (%zu nibbles)",
                      d.track, side, d.trackData().size());
        onDiag(b);
    }
}

// Decode the track under the head back into the disk image. The driver writes
// nibbles, not sectors: it finds an address field by reading, switches the head
// to write and lays down the data field that follows, and a format pass writes
// whole tracks including their address fields. So the image is reconstructed by
// parsing the stream the same way a drive would, which makes both cases the same
// code path and keeps a formatted-from-inside-the-OS disk readable.
void Machine::flushFloppyTrack() {
    SonyDrive& d = *drive0_;
    if (!d.trackDirty() || floppy_.empty() || floppyRO_) { d.clearTrackDirty(); return; }
    const int sides = floppy_.size() > 440u * 1024u ? 2 : 1;
    const int wrote = gcr::decodeTrack(d.trackData(), floppy_, trackCacheTrack_, sides);
    d.clearTrackDirty();
    ++floppyWrites_;
    if (writeLogBudget_ > 0 && onDiag) {
        --writeLogBudget_;
        char b[96];
        std::snprintf(b, sizeof b, "sony: track %d side %d written back (%d sectors)",
                      trackCacheTrack_, trackCacheSide_, wrote);
        onDiag(b);
    }
}

// Move bytes between the ISM's one-deep FIFO and the rotating surface.
//
// The chip does the framing in MFM mode, so this is where "the search for the
// mark byte is invisible to the software" (SWIM ref p.23) lives: setting ACTION
// on a read parks the head on the next mark byte, and from there the FIFO simply
// follows the surface. On a write, a byte the CPU hands over goes down at the
// head, and the Mark and CRC registers put a mark byte or the two CRC bytes on
// the disk instead.
void Machine::ismService(SonyDrive& d) {
    const bool action = iwm_->ismAction();
    if (action && !ismActionPrev_) {
        if (!iwm_->ismWriting()) d.syncToMark(totalCycles_);
        ismCrcOut_ = 0xFFFF;
    }
    ismActionPrev_ = action;
    if (!action) return;

    if (!iwm_->ismWriting()) {
        if (!iwm_->ismDataReady) {
            u8 b = 0;
            bool mark = false;
            if (d.nextByte(totalCycles_, &b, &mark)) {
                iwm_->ismData = b;
                iwm_->ismMarkNext = mark;
                iwm_->ismDataReady = true;
                iwm_->ismCrcAdd(b);
                ++iwmDataBytes_;
            }
        }
        return;
    }

    // Write side: the handshake reports room for a byte, and whatever the CPU
    // pushed goes down at the head.
    iwm_->ismDataReady = d.writeReady(totalCycles_);
    if (iwm_->ismWroteData || iwm_->ismWroteMark) {
        const bool mark = iwm_->ismWroteMark;
        d.writeByte(totalCycles_, iwm_->ismWritten, mark);
        ismCrcOut_ = mfm::crc16Update(ismCrcOut_, iwm_->ismWritten);
        iwm_->ismWroteData = iwm_->ismWroteMark = false;
        ++iwmDataWrites_;
    }
    if (iwm_->ismWroteCrc) {
        iwm_->ismWroteCrc = false;
        d.writeByte(totalCycles_, static_cast<u8>(ismCrcOut_ >> 8), false);
        d.writeByte(totalCycles_, static_cast<u8>(ismCrcOut_), false);
        iwmDataWrites_ += 2;
    }
}

// One IWM soft-switch access. The chip decodes the address and registers; the
// machine supplies the drive status line the phase lines currently select,
// because only it knows the drives.
u8 Machine::iwmAccess(int reg, bool write, u8 data) {
    const u32 accessPc = cpu_.recentPc(0);
    // The medium lives in floppy_, which the .Sony shim and the host swap under
    // us; keep the mechanism's view of it current.
    drive0_->readOnly = floppyRO_;
    drive0_->hdMedia  = floppy_.size() >= 1440u * 1024u;
    iwm_->senseHigh = iwmSenseLine();
    iwmUpdateTrack();
    SonyDrive& d = selectedDrive();
    if (iwm_->ismSelected()) ismService(d);
    else {
        // Q7 high means the chip is driving the write head. Reading the surface
        // then would consume the bytes the driver is in the middle of laying
        // down, so the read path stands aside while a write is in progress; the
        // surface still advances with time, because writing advances it too.
        // A drive reading high-density media in GCR mode gets nothing off it:
        // MFM is framed by the chip, and the track under the head is legible
        // only through the ISM path. Handing those bytes to the software GCR
        // reader lets it find marks in MFM noise, which takes the driver's own
        // media probe down the wrong branch.
        if (!(iwm_->lines() & Iwm::kQ7) && !d.hdMedia) {
            iwm_->dataByte = d.readNibble(totalCycles_);
            if (iwm_->dataByte) ++iwmDataBytes_;
        }
        iwm_->writeReady = d.writeReady(totalCycles_);
    }
    const u8 ret = iwm_->access(reg, write, data);
    if (iwm_->ismSelected()) ismService(d);
    else if (iwm_->writeLatched) {
        iwm_->writeLatched = false;
        d.writeNibble(totalCycles_, iwm_->writeData);
        ++iwmDataWrites_;
    }
    // Count the register the access actually landed on: every access toggles its
    // latch first, so the register a read returns is the one the NEW line state
    // selects, and sampling before the access counts the previous one.
    if (!write && iwm_->selected() == Iwm::Reg::Data) ++iwmDataReads_;
    iwmStrobe();
    if (onIwmAccess)
        onIwmAccess(reg, write, ret, iwm_->lines(), accessPc, iwmDriveReg());
    return ret;
}

int Machine::iwmDriveReg() const {
    return iwm_->driveRegister(((via_->ora() >> 5) & 1) != 0);   // SEL = VIA PA5
}

// Level of the addressed Sony drive status line, from the mechanism itself.
bool Machine::iwmSenseLine() {
    const int addr = iwmDriveReg();
    SonyDrive& d = selectedDrive();
    const bool level = d.sense(addr, totalCycles_);
    if (addr == 0x1 && cstinLogBudget_ > 0 && onDiag) {
        --cstinLogBudget_;
        char b[64];
        std::snprintf(b, sizeof b, "CSTIN poll: %s", level ? "no-disk" : "disk-in");
        onDiag(b);
    }
    if (addr == 0x6 && !level) d.clearSwitched();   // reading the line clears it
    return level;
}

// A drive command is latched on the rising edge of LSTRB, with the register in
// CA1:CA0:SEL and the data bit on CA2. While the strobe stays high a held eject
// can mature (it needs ~750 ms, which is what stops the brief pulse some ROMs
// emit at boot from ejecting the disk).
void Machine::iwmStrobe() {
    const bool lstrb = iwm_->lstrb();
    if (lstrb) {
        // addr is CA2:CA1:CA0:SEL. The command register is CA1:CA0:SEL -- the
        // low three bits -- and CA2 (the top bit) carries the data written to it.
        const int addr = iwmDriveReg();
        const int cmd  = addr & 7;
        const bool bit = (addr & 8) != 0;
        SonyDrive& d = selectedDrive();
        if (!lstrbPrev_) {
            d.command(cmd, bit, totalCycles_);
            ++sonyCmds_[iwm_->externalDrive() ? 1 : 0][cmd & 7];
            if (sonyCmdLog_ > 0 && onDiag) {
                --sonyCmdLog_;
                static const char* kCmd[8] = {"DIRTN", "?1", "STEP", "?3",
                                              "MOTORON", "?5", "EJECT", "?7"};
                char b[96];
                std::snprintf(b, sizeof b, "sony cmd: drive%d %s=%d (track=%d)",
                              iwm_->externalDrive() ? 2 : 1, kCmd[cmd & 7],
                              bit ? 1 : 0, d.track);
                onDiag(b);
            }
        }
    }
    {
        // The eject is a mechanism, not a strobe: it runs on its own clock once
        // commanded, so it is advanced on every access rather than only while
        // LSTRB is asserted.
        SonyDrive& d = selectedDrive();
        d.tickEject(totalCycles_);
        if (d.takeEjectRequest()) {
            if (&d == drive0_.get()) flushFloppyTrack();   // before the medium goes
            d.removeDisk();
            if (&d == drive0_.get()) {
                // Ejecting a disk does not destroy it. Keep the medium's final
                // contents so the host can still write them back to the file it
                // came from -- everything the guest wrote up to this moment has
                // just been flushed into it.
                floppyEjected_ = std::move(floppy_);
                floppy_.clear();
                ++floppyGen_;
                floppyRO_ = false;
                ++floppyEjects_;
                if (onDiag) onDiag("sony: drive ejected the disk");
            }
        }
    }
    lstrbPrev_ = lstrb;
}

// The ROM does GCR decoding in software, which means it also reports its own
// failures: every bail-out in the sector reader loads a Sony driver error code
// ($B8-$BE, i.e. -72..-66) into D0 and branches to a common tail. Finding those
// exits turns an opaque "this disk is unreadable" into a specific verdict on the
// nibble stream we synthesise. Anchor on the reader's own mark table -- the six
// bytes D5 AA 96 DE AA FF, the address-field prologue and epilogue it matches
// against -- then scan the code that follows for MOVEQ #$Bx,D0 immediately
// followed by a short branch. Anchoring on data rather than on fixed addresses
// keeps this working if the ROM is ever swapped.
void Machine::findGcrErrorSites(u32 drvrBase) {
    if (!drvrBase) return;
    // The driver and its GCR routines occupy roughly 40 KB from the DRVR header
    // (in this ROM, header $434680, sector reader $43D0xx).
    const u32 begin = drvrBase & romMask_;
    const u32 limit = static_cast<u32>(rom_.size()) - 4;
    const u32 end = (begin + 0xA000 < limit) ? begin + 0xA000 : limit;
    for (u32 a = begin & ~1u; a < end; a += 2) {
        if (rom_[a] != 0x70) continue;                            // MOVEQ #imm,D0
        const u8 code = rom_[a + 1];
        // $AC-$BF is the whole Sony driver error range, -84 (verErr) through -65
        // (offLinErr): not only the nibble-level failures but the ones the sector
        // search itself reports, seekErr and sectNFErr.
        if (code < 0xAC || code > 0xBF) continue;
        if (rom_[a + 2] != 0x60) continue;                        // followed by BRA.s
        gcrErrSites_.push_back({0x400000u + a, code});
        // Every one of these paths branches to its routine's common exit, and
        // the offLinErr site lives in Prime -- so its branch target is where
        // Prime hands its result back. Watching that reports the driver's own
        // verdict on a request instead of leaving us to infer one, which is the
        // only way to see a read that gives up without setting an error code.
        if (code == 0xBF && !sonyResultPc_) {
            const auto disp = static_cast<signed char>(rom_[a + 3]);
            sonyResultPc_ = 0x400000u + a + 2 + 2 + static_cast<u32>(disp);
        }
    }
    if (gcrErrSites_.empty()) return;
    gcrErrLo_ = gcrErrSites_.front().pc;
    gcrErrSpan_ = gcrErrSites_.back().pc - gcrErrLo_;
}

// The .Sony driver is about to return: D0 holds the result code.
void Machine::watchSonyResult() {
    const u16 res = static_cast<u16>(cpu_.d[0] & 0xFFFF);
    ++sonyResults_;
    if (res) ++sonyResultErrs_;
    if (sonyResultLog_ <= 0 || !onDiag) return;
    --sonyResultLog_;
    char b[128];
    std::snprintf(b, sizeof b, "sony result: %04X %s (track %d side %d)", res,
                  res == 0 ? "ok" : Machine::gcrErrorName(static_cast<u8>(res & 0xFF)),
                  drive0_->track, drive0_->headUpper ? 1 : 0);
    onDiag(b);
}

const char* Machine::gcrErrorName(u8 code) {
    switch (code) {
        case 0xAC: return "verErr(-84) track failed to verify";
        case 0xAD: return "fmt2Err(-83) can't get enough sync";
        case 0xAE: return "fmt1Err(-82) can't find sector 0 after format";
        case 0xAF: return "sectNFErr(-81) sector never found on this track";
        case 0xB0: return "seekErr(-80) wrong track number in the address mark";
        case 0xB1: return "spdAdjErr(-79) can't adjust disk speed";
        case 0xB2: return "twoSideErr(-78) second side on a one-sided drive";
        case 0xB3: return "initIWMErr(-77) unable to initialize the IWM";
        case 0xB4: return "tk0BadErr(-76) track 0 detect doesn't change";
        case 0xB5: return "cantStepErr(-75) step handshake failed";
        case 0xB6: return "wrUnderrun(-74) write underrun";
        case 0xB7: return "badDBtSlp(-73) data field bit-slip nibbles";
        case 0xBF: return "offLinErr(-65) drive off-line";
        case 0xBE: return "noNybErr(-66) no disk bytes under the head";
        case 0xBD: return "noAdrMkErr(-67) address mark D5 AA 96 not found";
        case 0xBC: return "dataVerErr(-68) read verify compare failed";
        case 0xBB: return "badCksmErr(-69) address field checksum";
        case 0xBA: return "badBtSlpErr(-70) address field epilogue DE AA";
        case 0xB9: return "noDtaMkErr(-71) data mark D5 AA AD not found";
        case 0xB8: return "badDCksum(-72) data field checksum";
        default:   return "(not a disk error)";
    }
}

void Machine::noteGcrError(u32 pc) {
    for (const auto& s : gcrErrSites_) {
        if (s.pc != pc) continue;
        ++gcrErrors_[s.code - 0xB8];
        if (onGcrError) onGcrError(s.code, pc);
        if (gcrErrLog_ > 0 && onDiag) {
            --gcrErrLog_;
            char b[128];
            std::snprintf(b, sizeof b, "gcr: read failed, ROM says %02X %s (at %06X)",
                          s.code, gcrErrorName(s.code), pc);
            onDiag(b);
        }
        return;
    }
}

// With the shim disabled the ROM's own driver services disk I/O, so the only
// way to see what the File Manager actually asked the disk for is to watch its
// Prime entry: A0 is the parameter block, A1 the device control entry.
void Machine::watchSonyPrime() {
    if (sonyPrimeLog_ <= 0 || !onDiag) return;
    --sonyPrimeLog_;
    const u32 pb = cpu_.a[0], dce = cpu_.a[1];
    const int drive = static_cast<s16>(read16(pb + ioVRefNum));
    const u32 length = read32(pb + ioReqCount);
    const u32 position = read32(dce + dCtlPosition);
    const bool isRead = (read16(pb + ioTrap) & 0xFF) == kARdCmd;
    char b[128];
    std::snprintf(b, sizeof b, "sony Prime(rom): drive=%d %s block %u len=%u",
                  drive, isRead ? "read " : "write", position / 512u, length);
    onDiag(b);
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
    if (!sonyShim_ || inSony_ || sonyPrimePc_ == 0) return false;
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

    // An eject requested from the UI: unmount the volume so it leaves the desktop,
    // rather than just reporting the disk out (which leaves the Finder wanting it back).
    // A busy volume (files open, fBsyErr) can't be put away -- leave it, as a real Mac does.
    if (floppyEjectPending_ && !inSony_ && drvStatusAddr_ != 0) {
        u32 sd[8], sa[8];
        for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
        if (floppyMountPb_ == 0) {
            cpu_.d[0] = 80;
            execute68kTrap(kTrapNewPtrSysClear);
            floppyMountPb_ = cpu_.a[0];
        }
        const u32 vcbTailBefore = (static_cast<u32>(read16(0x035C)) << 16) | read16(0x035E);
        u16 unmountRes = 0;
        if (floppyMountPb_ != 0) {
            write16(floppyMountPb_ + ioVRefNum, static_cast<u16>(floppyDriveNum_));
            cpu_.a[0] = floppyMountPb_;
            execute68kTrap(kTrapUnmountVol);
            unmountRes = static_cast<u16>(cpu_.d[0] & 0xFFFF);
        }
        const bool ejected = (unmountRes != 0xFFD1);   // fBsyErr (-47) => busy, keep it
        if (ejected) {
            floppy_.clear();
            ++floppyGen_;
            write8(drvStatusAddr_ + dsDiskInPlace, 0);
        }
        for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
        floppyEjectPending_ = false;
        if (onDiag) {
            const u32 vcbTailAfter = (static_cast<u32>(read16(0x035C)) << 16) | read16(0x035E);
            char b[160];
            std::snprintf(b, sizeof b, "floppy: eject unmount=%04X ejected=%d VCBtail %06X->%06X%s",
                          unmountRes, ejected ? 1 : 0, vcbTailBefore & 0xFFFFFF,
                          vcbTailAfter & 0xFFFFFF, ejected ? "" : " [disk busy -- close its windows first]");
            onDiag(b);
        }
    }

    // A floppy inserted or swapped after boot. Complete it here at a clean frame
    // boundary the way the ROM's disk-insert interrupt does: unmount any outgoing
    // volume, seat the new disk, _MountVol it, then post the disk-inserted event with
    // that result. (A bare diskEvt never mounts; and skipping the unmount leaves the
    // old volume on the drive, so _MountVol returns volOnLinErr, or the stale volume
    // masks the new one, and nothing appears in the Finder.)
    if (floppyInsertPending_ && !inSony_ && drvStatusAddr_ != 0) {
        u32 sd[8], sa[8];
        for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
        if (floppyMountPb_ == 0) {
            cpu_.d[0] = 80;                              // param block for _UnmountVol/_MountVol
            execute68kTrap(kTrapNewPtrSysClear);
            floppyMountPb_ = cpu_.a[0];
        }
        const bool hadOld = !floppy_.empty();           // a disk to eject before the new one
        u16 unmountRes = 0;
        bool seated = false;
        if (floppyMountPb_ != 0 && !floppyPending_.empty()) {
            if (hadOld) {
                // Eject the outgoing volume first -- its image is still current, so a
                // flush lands on the right disk. If it is busy (files open) this fails
                // and we must NOT seat the new disk over a mismatched volume.
                write16(floppyMountPb_ + ioVRefNum, static_cast<u16>(floppyDriveNum_));
                cpu_.a[0] = floppyMountPb_;
                execute68kTrap(kTrapUnmountVol);
                unmountRes = static_cast<u16>(cpu_.d[0] & 0xFFFF);
            }
            if (!hadOld || unmountRes != 0xFFD1) {
                // Empty drive, or the outgoing volume no longer holds the drive
                // (unmounted cleanly, or already offline/gone). Only a *busy* volume
                // (fBsyErr = -47 = 0xFFD1, files still open) blocks the swap. Seat the
                // disk and post the disk-inserted event; the SYSTEM mounts it itself
                // (IM Files: WaitNextEvent -> SystemEvent -> PBMountVol, result into the
                // event's high word). We must NOT pre-mount -- doing so left a VCB the
                // System's own mount then tripped over, ejecting the disk as
                // "unreadable" during an installer disk-switch.
                floppy_ = std::move(floppyPending_);
                ++floppyGen_;
                floppyRO_ = floppyPendingRO_;
                write8(drvStatusAddr_ + dsDiskInPlace, 1);
                seated = true;
                // _PostEvent (Inside Macintosh I-257): A0 = event code, D0 = message.
                cpu_.a[0] = 7;                          // diskEvt
                cpu_.d[0] = static_cast<u16>(floppyDriveNum_);  // message: drive #, result word 0
                execute68kTrap(kTrapPostEvent);
            }
            floppyPending_.clear();                     // drop the staged disk either way
        }
        for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
        floppyInsertPending_ = false;
        if (onDiag) {
            char b[160];
            std::snprintf(b, sizeof b,
                "floppy: unmount=%04X seated=%d diskEvt posted (drv %d) -- System mounts%s",
                unmountRes, seated ? 1 : 0, floppyDriveNum_,
                (hadOld && !seated) ? " [outgoing disk busy -- eject it first]" : "");
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
        // Gate the Sound-Manager buffer on the CPU-written vSndEnb (ORB bit 7,
        // low = on) only. The T1->PB7 square wave (Via6522::pb7) is a separate,
        // rarely-used direct-tone path; a title that sets ACR bit 7 to run the
        // timer must not silence or chop the buffer, so the buffer gate stays
        // insensitive to the timer-driven PB7 toggle.
        const bool enabled = (via_->orb() & 0x80) == 0;   // vSndEnb: 0 = sound on
        int s = 0x80;
        // Reciprocal volume law (matches Mini vMac SNDEMDEV): attenuate by
        // 1/(8-vol) so volume 0 is 1/8 (quiet but audible) and volume 7 is full
        // scale, instead of a linear curve that muted volume 0 outright.
        if (enabled) s = 0x80 + (static_cast<int>(raw) - 0x80) / (8 - vol);
        if (audioOut_.size() < 8192) audioOut_.push_back(static_cast<u8>(s));
    }

    // Boot-trace: while the OS boots, emit a periodic state line to the log (onDiag ->
    // openmac.log) so a stuck or failing boot is visible instead of a black box. PC/SR
    // show where the CPU is, hdAcc shows disk progress, and scr is a framebuffer hash
    // that changes as the screen draws (Happy Mac -> Welcome -> desktop). ~50 lines over
    // the first 3000 frames, then it goes quiet.
    if (bootTraceFrame_ < 3000) {
        if (onDiag && (bootTraceFrame_ % 60) == 0) {
            const u32 sb = screenBase();
            u32 scr = 2166136261u;
            for (u32 i = 0; i < 21888u; i += 64) scr = (scr ^ ram_[(sb + i) & ramMask_]) * 16777619u;
            char b[112];
            std::snprintf(b, sizeof b, "boot f=%u pc=%06X sr=%04X hdAcc=%u scr=%08X%s",
                          bootTraceFrame_, cpu_.pc, cpu_.getSR(), hdAccessCount(), scr,
                          cpu_.halted ? " HALTED" : "");
            onDiag(b);
        }
        ++bootTraceFrame_;
    }
}

} // namespace openmac
