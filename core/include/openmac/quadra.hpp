#pragma once

// Macintosh Quadra 650 machine: 68040 @ 33 MHz on the djMEMC/IOSB board.
// RAM + ROM with the read-triggered boot overlay, VIA1 (real 6522) + VIA2
// (IOSB's pseudo-VIA facade), the PIC ADB modem behind VIA1's shift register,
// the classic 343-0042 RTC/PRAM chip on VIA1 port B, DAFB video with 1MB
// VRAM, EASC+DFAC audio, NCR 53C96 SCSI over the shared target bus, and
// probe-safe SCC/SWIM2/SONIC stubs. Empty NuBus slot space bus-errors, which
// is what the Slot Manager's probes expect.
//
// Reference: Quadra 650 hardware dossier (register maps and boot sequence
// facts drawn from Apple Developer Notes for this board family and MAME as
// a behavioral reference). Clean-room.

#include "openmac/bus040.hpp"
#include "openmac/cpu040.hpp"
#include "openmac/types.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace openmac {

class Via6522;
class PseudoVia;
class Rtc;
class AdbTransceiver;
class Dafb;
class Easc;
class Ncr53c96;
class Scc8530;
class ScsiDisk;
class ScsiCdRom;
class ScsiEthernet;
class SonyDrive;

class QuadraMachine final : public IBus040 {
public:
    struct Config {
        u32 ramSize = 8u * 1024 * 1024;   // 8/16/32/64/128 MB
    };

    QuadraMachine(std::vector<u8> rom, const Config& cfg);
    explicit QuadraMachine(std::vector<u8> rom);
    ~QuadraMachine() override;

    void reset();

    // Run one 60.15 Hz tick frame (370 slices of CPU time; audio and the
    // DAFB vertical blank ride the slice cadence).
    void runFrame();
    int stepInstruction();

    M68040& cpu() { return cpu_; }
    u64 totalCycles() const { return totalCycles_; }
    u64 frameCount() const { return frameCounter_; }
    bool overlayActive() const { return overlay_; }

    // Video: geometry follows what the ROM programs into DAFB.
    int screenWidth() const;
    int screenHeight() const;
    void renderScreen(u32* argbOut) const;

    // Audio: 8-bit unsigned mono at the 22.25 kHz slice rate.
    void drainAudio(std::vector<u8>& out);

    // Input through the ADB modem.
    void mouseMove(int dx, int dy, bool button);
    void keyEvent(u8 adbCode, bool down);
    u32 adbMousePolls() const;
    u32 adbKbdPolls() const;
    u32 adbMouseReports() const;   // polls that actually carried motion
    u32 hdReads() const { return hdReadCount_; }
    u32 hdWrites() const { return hdWriteCount_; }
    u32 fdReads() const { return fdReadCount_; }
    u32 fdWrites() const { return fdWriteCount_; }
    // A text snapshot of CPU, low memory and every device -- what to capture
    // when the guest stops responding. Reads model state only, so taking it
    // cannot perturb the machine.
    std::string diagnosticReport() const;
    // Flush and unmount every mounted volume, as Shut Down would. Call before
    // persisting an image when the host application is closing: without it the
    // System's cached blocks are lost and the volume stays flagged unclean,
    // which this ROM refuses at the next boot.
    bool shutdownVolumes();
    u32 adbMouseBytesRead() const; // mouse bytes the guest actually clocked in
    std::vector<u8> adbMouseBytesLog() const;
    void adbClearCmdTrace();
    std::vector<u8> adbCmdTrace() const;   // CPU-issued ADB commands since the clear
    void armAdbSrTrace(int n) { adbSrTraceBudget_ = n; }   // log VIA1 SR reads w/ PCs
    void watchMem(u32 addr, u32 len, int budget) {         // log RAM writes w/ PCs
        watchAddr_ = addr; watchLen_ = len; watchBudget_ = budget;
    }
    void armDataInTrace(int n) { dataInPcBudget_ = n; }
    void traceHdRequests(int n) { hdTraceBudget_ = n; }
    // Log every device access made between two frames by code inside a PC
    // range: which registers a routine actually touches, and what they
    // answered. Answers "what is this driver waiting on" directly.
    void traceIoWindow(u32 fromFrame, u32 toFrame, u32 pcLo, u32 pcHi,
                       int budget) {
        ioTraceFromFrame_ = fromFrame; ioTraceToFrame_ = toFrame;
        ioTracePcLo_ = pcLo; ioTracePcHi_ = pcHi; ioTraceBudget_ = budget;
    }
    void countPc(u32 pc) { countPc_ = pc; countPcHits_ = 0; }
    u32 countPcHits() const { return countPcHits_; }
    bool dafbVblEnabled() const;
    u32 dafbSwatchReg(int i) const;
    // Which display is plugged into the video port. The monitor identifies
    // itself on the connector's sense lines and the ROM picks the timing from
    // that, so this is how the machine is told to run at another resolution.
    void setMonitorSense(u32 grounded, u32 pairs);

    // SCSI media, mirroring the Classic's surface. Images are wrapped in an
    // Apple partition map with a driver the ROM's boot scan can load.
    void insertHardDisk(std::vector<u8> image, bool readOnly = false);
    bool hardDiskPresent() const { return !hd_.empty(); }
    const std::vector<u8>& hardDiskImage() const;
    void insertHardDisk2(std::vector<u8> image, bool readOnly = false);
    void detachHardDisk2();
    bool hardDisk2Present() const { return !hd2_.empty(); }
    const std::vector<u8>& hardDisk2Image() const;
    void attachCdRom(bool attached, int busId = 3);
    bool cdRomAttached() const;
    // Returns 1 when the drive took the disc, 0 when the file was refused.
    int insertCd(std::vector<u8> image);
    void ejectCd();
    bool cdPresent() const;
    void attachNet(bool attached, int busId = 4);
    bool netAttached() const;
    bool netInject(const u8* frame, u32 len);
    bool netDrain(std::vector<u8>& out);

    // Floppy: the internal SuperDrive, served at driver level (the ROM's own
    // .Sony sees a present mechanism through the SWIM2 sense lines; Prime is
    // answered from the image). Raw 1.44 MB dumps only for now.
    // Returns 1 when the drive took the disk, 0 when the file was refused.
    int insertFloppy(std::vector<u8> image, bool readOnly = false);
    void ejectFloppy();
    bool floppyPresent() const;
    const std::vector<u8>& floppyImage() const { return floppy_; }
    // The medium a guest-commanded eject pushed out, with every byte the
    // guest wrote still in it; empty when nothing has been ejected.
    std::vector<u8> takeEjectedFloppy() { return std::move(floppyEjected_); }
    // The medium wearing the containers its file arrived in, ready to write
    // back: the disk in the drive, or the one last ejected.
    std::vector<u8> floppyForWriteBack() const;

    // Diagnostics.
    struct ScsiDiag {
        u32 writes, selects, commands;
        u8 lastCdb[12];
        int lastCdbLen;
    };
    ScsiDiag scsiDiag() const;
    // Ask the guest's Gestalt a question by running the trap on the guest
    // CPU. Headless diagnosis only: the answer is whatever the System's own
    // table holds. Returns the OSErr; the response lands in `response`.
    s32 gestaltQuery(u32 selector, u32& response);
    // Diagnosis switch: skip the injected mount/announce of hard disks, to
    // separate "our injection perturbs the boot" from everything else.
    bool suppressHdAnnounce = false;
    const std::vector<std::string>& accessLog() const { return accessLog_; }
    void clearAccessLog() { accessLog_.clear(); }
    std::function<void(const char* msg)> onDiag;
    // Fires on VRAM writes with the offset and the guest PC. Bring-up only.
    std::function<void(u32 off, u32 pc)> onVramWrite;

    // IBus040 (the CPU's view of the machine).
    u8   read8(u32 addr) override;
    u16  read16(u32 addr) override;
    u32  read32(u32 addr) override;
    void write8(u32 addr, u8 value) override;
    void write16(u32 addr, u16 value) override;
    void write32(u32 addr, u32 value) override;

private:
    void wireDevices();
    void tickDevices(int cpuCycles);
    void adbMaybeClock();
    void adbIdleWake();
    void logAccess(const char* what, u32 addr, bool write, u32 value);

    // I/O window dispatch ($50000000 block, mirrors folded). The public
    // entry points trace; the Impl bodies do the decode.
    u8   ioRead8(u32 off);
    void ioWrite8(u32 off, u8 v);
    u8   ioRead8Impl(u32 off);
    void ioWrite8Impl(u32 off, u8 v);
    void traceIo(u32 off, bool write, u8 v);
    bool markVolumeClean(u16 drive);
    void completeFloppyEject();
    void clearDriveInPlace(u16 drive);
    std::vector<u8> floppyFileImage(const std::vector<u8>& sectors) const;
    u32  ioRead32(u32 off);
    void ioWrite32(u32 off, u32 v);
    void sonicWrite(u32 reg, u16 v);

    std::vector<u8> ram_;
    std::vector<u8> rom_;
    u32 romMask_ = 0;
    bool overlay_ = true;

    // djMEMC registers echo what was written (A/UX checks the readback);
    // IOSB's block does the same except the SCSI wait-state select.
    u32 djmemcRegs_[16]{};
    u16 iosbRegs_[32]{};

    // Z8530 SCC: interrupt-capable model (System 7.5's LocalTalk driver is
    // interrupt-driven; a chip that never interrupts stalls its node
    // acquisition at the Welcome screen). Lives in machine/quadra/scc.hpp.

    // SWIM2 stub: mode set/clear pair, a phases latch the ROM's presence
    // probe writes patterns into and reads back, and a no-disk handshake.
    u8 swimMode_ = 0x40;
    u8 swimSetup_ = 0;
    u8 swimPhases_ = 0;
    u8 swimParams_[16]{};
    int swimParamPtr_ = 0;

    // SONIC stub: register echo, silicon revision 6, reset-state CR.
    u16 sonicRegs_[0x40]{};

    u8 macProm_[8]{};

    void servePrime();
    void execute68kTrap(u16 trap);
    void announceHardDisks();
    // The on-disk SCSI driver's Prime, served in C++ (see the definition).
    // A pointer handed over by the guest can carry Memory Manager flag bits in
    // its high byte -- the 24-bit era's locked/purgeable marks ride there, and
    // the guest's own translation drops them. Strip them ONLY when the address
    // does not already point into RAM: with more than 16 MB fitted, a real
    // address has significant bits above bit 23, and truncating it aims the
    // transfer at the wrong memory. (That is a System Error a few instructions
    // later, and it is why 136 MB behaved differently from 8 MB.)
    u32 guestPtr(u32 p) const {
        if (p < ram_.size()) return p;
        const u32 stripped = p & 0x00FFFFFFu;
        return stripped < ram_.size() ? stripped : p;
    }
    void serveDiskPrime(int unit);
    void serveDiskCtlStatus(int unit, bool status);
    void findDiskDriverPrime();
    bool volumeMountedFor(u16 drive);
    bool inDriver_ = false;      // serving a driver request: no nested traps
    u32 diskPrimePc_[2] = {0, 0};   // unit 1 (drive 4) and unit 33 (drive 5)
    u32 diskCtlPc_[2] = {0, 0}, diskStatusPc_[2] = {0, 0};

    std::unique_ptr<SonyDrive> fd_;
    std::vector<u8> floppy_;
    std::vector<u8> floppyEjected_;
    bool floppyEjectPending_ = false;   // front-end eject, run at a frame edge
    // Containers peeled on insertion -- MacBinary around DiskCopy 4.2 around
    // the sectors, either or both -- kept as plain bytes so this header need
    // not know the container formats, and so the medium goes back to the host
    // in the file format it arrived in, reassembled around the guest's writes.
    std::vector<u8> fdMbHeader_, fdMbResource_;
    std::vector<u8> fdDcHeader_, fdDcTags_;
    // VIA1 PA5 = the drive's SEL line. Undriven pins read HIGH (the outA
    // handler models the pull-up with value|~ddr), so the power-on state is
    // high too: at reset the ROM walks the SWIM2 phase lines before touching
    // the VIA, and with SEL low that walk's one LSTRB rise addresses control
    // register 6 -- EJECT -- and throws out whatever disk was in the drive.
    // With SEL high it addresses register 7, which no drive implements.
    bool floppySel_ = true;
    u8 swimPhasePrev_ = 0;        // LSTRB edge detection

    std::unique_ptr<Via6522> via1_;
    std::unique_ptr<PseudoVia> via2_;
    std::unique_ptr<Rtc> rtc_;
    std::unique_ptr<AdbTransceiver> adb_;
    std::unique_ptr<Dafb> dafb_;
    std::unique_ptr<Easc> easc_;
    std::unique_ptr<Scc8530> scc_;
    std::unique_ptr<Ncr53c96> scsi_;
    std::unique_ptr<ScsiDisk> disk_;
    std::unique_ptr<ScsiDisk> disk2_;
    std::unique_ptr<ScsiCdRom> cdrom_;
    std::unique_ptr<ScsiEthernet> netdev_;
    M68040 cpu_;

    mutable std::vector<u8> hd_;
    std::vector<u8> scsiImage_;
    u32 hfsImageOffset_ = 0;
    bool hdRO_ = false;
    mutable std::vector<u8> hd2_;
    std::vector<u8> scsiImage2_;
    u32 hfsImageOffset2_ = 0;

    // ADB shift-register bridge (same push model as the Classic).
    bool adbArmed_ = false;
    bool adbArmedInput_ = false;
    int adbPending_ = 0;
    bool adbPendingInput_ = false;
    int adbWakeStreak_ = 0;
    u32 adbLastPollTotal_ = 0;

    void updateIpl();

    u64 totalCycles_ = 0;
    u64 frameCounter_ = 0;
    int via1Remainder_ = 0;
    u64 secondAcc_ = 0;
    int audioAcc_ = 0;
    int sliceInFrame_ = 0;
    u32 vblAcc_ = 0;
    int ca2PulseSlices_ = 0;
    u32 bootTraceFrame_ = 0;

    int eascDiagBudget_ = 24;   // trace the first EASC control reads
    int swimDiagBudget_ = 400;  // trace the first SWIM2 accesses (with PCs)
    int cmdDiagBudget_ = 120;   // trace notable drive command strobes
    int primeDiagBudget_ = 48;  // trace the first .Sony Prime requests we serve
    int adbSrTraceBudget_ = 0;  // armed by the input test: log SR reads with PCs
    int dataInPcBudget_ = 0;    // log which routine drains SCSI data-in bytes
    u32 watchAddr_ = 0, watchLen_ = 0;   // RAM write-watch window
    int watchBudget_ = 0;

    // Device-access trace, gated by frame window and by the PC that issued
    // the access. A stalled driver's whole conversation with its chip is a
    // few dozen lines once the ROM's own interrupt traffic is filtered out
    // by PC -- unfiltered it is millions.
    u32 ioTraceFromFrame_ = 0, ioTraceToFrame_ = 0;
    u32 ioTracePcLo_ = 0, ioTracePcHi_ = 0xFFFFFFFFu;
    int ioTraceBudget_ = 0;
    u64 ioRecent_[8]{};        // last few printed accesses (pc+reg+value)
    int ioRecentLen_ = 0;
    u32 ioRepeatCount_ = 0;
    void flushIoCycle();
    u32 countPc_ = 0, countPcHits_ = 0;  // execution counter for one address
    bool dafbIrq_ = false;               // internal video interrupt line (VIA2 PA6)
    // A hard disk present at power-on entered the drive queue during the ROM
    // scan, whose announcement startup flushes; once the System settles, a
    // disk-inserted event makes it the same event a user's insertion is.
    bool hdAnnouncePending_ = false, hd2AnnouncePending_ = false;
    int  hdAnnounceDelay_ = 0;
    bool announceInFlight_ = false;
    u32  hdMountPb_ = 0;        // System-heap param block for the _MountVol retries
    int  hdMountTries_ = 0;
    u32  hdReadCount_ = 0, hdWriteCount_ = 0;
    u32  fdReadCount_ = 0, fdWriteCount_ = 0;
    int  hdTraceBudget_ = 0;    // traceHdRequests: log every disk request
    int  hdErrBudget_ = 20;     // report the first failed disk requests
    int scsiDiagBudget_ = 60;   // trace the first 53C96 register accesses
    int iplDiagBudget_ = 12;    // trace the first level-2 interrupt assertions
    int via2DiagBudget_ = 24;   // trace the first VIA2 IFR/IER writes
    int cdbDiagBudget_ = 2000;    // trace the first SCSI register writes
    int cdbListBudget_ = 600;     // CDB completions: own budget, storm-proof
    int cdbSeen_ = 0;             // completions so far (re-arms the reg trace)
    std::vector<std::string> accessLog_;
    std::vector<u8> audioOut_;
};

} // namespace openmac
