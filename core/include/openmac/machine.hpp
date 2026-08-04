#pragma once

// Macintosh Classic machine: 68000 + RAM/ROM with boot overlay + VIA 6522 +
// RTC/PRAM + video, on the SE/Classic address map. SCC/SCSI/IWM are safe
// logged stubs until their phases.

#include "openmac/bus.hpp"
#include "openmac/cpu.hpp"
#include "openmac/types.hpp"

#include <memory>
#include <string>
#include <vector>

namespace openmac {

class Via6522;
class Rtc;
class AdbTransceiver;
class Ncr5380;
class ScsiDisk;
class ScsiCdRom;
class ScsiEthernet;
class Iwm;
class SonyDrive;

class Machine final : public IBus {
public:
    struct Config {
        u32 ramSize = 4u * 1024 * 1024;   // 1, 2, 2.5 or 4 MB
    };

    static constexpr int kScreenW = 512;
    static constexpr int kScreenH = 342;
    static constexpr int kLinesPerFrame = 370;
    static constexpr int kCyclesPerLine = 352;

    Machine(std::vector<u8> rom, const Config& cfg);
    explicit Machine(std::vector<u8> rom);   // default Config, defined in .cpp
    ~Machine() override;

    void reset();

    // Run one 60.15 Hz frame (370 lines x 352 CPU cycles).
    void runFrame();

    // Debugger: execute a single instruction with device time advancing.
    int stepInstruction();

    M68000& cpu() { return cpu_; }
    Via6522& via() { return *via_; }
    Rtc& rtc() { return *rtc_; }
    u64 totalCycles() const { return totalCycles_; }
    // Frames since power-on. Everything the machine says about itself is
    // stamped with this, so anything keeping its own record needs to read it
    // to line the two up.
    u64 frameCount() const { return frameCounter_; }

    // Re-open the disk diagnostics for another `budget` lines each. They are
    // budgeted because a running System reads constantly and would bury the
    // log; but the interesting request is usually the last one, not the first,
    // so it has to be possible to open them again at a chosen moment.
    void openDiskLog(int budget) {
        gcrErrLog_ = sonyPrimeLog_ = sonyResultLog_ = sonyCmdLog_ = budget;
    }

    // Check every completed read against the medium it came from. A read that
    // returns the wrong bytes and reports success is invisible from every other
    // vantage -- the driver is satisfied, the System is satisfied, and it
    // surfaces much later as a file that will not open.
    void setVerifyReads(bool on) { verifyReads_ = on; }
    // Log every SCSI block read/write that touches this byte of the HFS volume
    // (offset within the volume, not the partitioned image; -1 = off). The bytes
    // at the watched offset and the guest PC go to onDiag -- the tool for asking
    // "who wrote THIS word, and what did it write".
    void setHfsWatch(s64 volumeByteOff) { hfsWatch_ = volumeByteOff; }
    u32 readsVerified() const { return readsVerified_; }
    u32 readMismatches() const { return readMismatches_; }
    bool overlayActive() const { return overlay_; }

    u32 screenBase() const;
    u32 soundBase() const;
    u32 ramSize() const { return static_cast<u32>(ram_.size()); }
    // Expand the 1-bit framebuffer to ARGB8888 (kScreenW * kScreenH).
    void renderScreen(u32* argbOut) const;

    // What became of a disk handed to a drive. kAccepted means the mechanism has
    // it and the ROM's driver will find sectors on it; kRefused means the file is
    // not something a floppy drive can hold -- the drive is left exactly as it
    // was, and mediumText() says what the file actually is, in words meant for
    // the person who chose it.
    enum class InsertVerdict { kAccepted, kRefused };
    // The last per-drive medium description (0 = internal, 1 = external):
    // geometry and container for an accepted disk, the reason for a refusal.
    const char* mediumText(int drive) const { return mediumText_[drive & 1]; }

    // Floppy: put a disk image in the internal drive. Raw 400K/800K/1.4MB
    // sector dumps mount as-is; DiskCopy 4.2 and MacBinary wrappers (and the
    // two nested) are recognised and stripped, and are faithfully reassembled
    // around the guest's writes when the image is read back out. Anything that
    // is not floppy media -- an NDIF image, an application, an archive -- is
    // refused with its nature named rather than framed as noise.
    InsertVerdict insertFloppy(std::vector<u8> image, bool readOnly = false);
    void ejectFloppy();
    bool floppyInserted() const { return !floppy_.empty() || !floppyStaged_.empty(); }
    // Persist writes by reading this back out. When the ROM's own driver is
    // running the hardware, whatever it has written to the track under the head
    // is still a nibble stream; flushing decodes it back into the image first.
    const std::vector<u8>& floppyImage();
    // How many times the drive has thrown a disk out on its own.
    u32 floppyEjectCount() const { return floppyEjects_; }

    // The external drive port. A Classic has one, and the ROM probes it on every
    // boot -- an empty port floats every status line high, which is what tells
    // the drive scan to move on. Attach a mechanism and the scan registers a
    // second floppy drive; its disk then behaves exactly like the internal one,
    // because it is the same mechanism model over the same chip.
    void setExternalDriveAttached(bool on);
    bool externalDriveAttached() const;
    InsertVerdict insertExternalFloppy(std::vector<u8> image, bool readOnly = false);
    void ejectExternalFloppy();
    bool externalFloppyInserted() const { return !floppy2_.empty(); }
    const std::vector<u8>& externalFloppyImage();

    // Hard disk: a second, fixed (non-removable) image mounted through the same
    // high-level .Sony interception as a hard-disk volume. Empty = no hard disk.
    // Persist writes by reading hardDiskImage() back out after running.
    void insertHardDisk(std::vector<u8> image, bool readOnly = false);
    bool hardDiskPresent() const { return !hd_.empty(); }
    const std::vector<u8>& hardDiskImage() const {
        // When SCSI owns the disk, runtime writes land in the partitioned image; copy the
        // HFS volume back out of it so callers persist the current contents, not the stale
        // original. (The .Sony path writes hd_ directly, so this is a no-op there.)
        if (scsiHandlesHd_ && !scsiImage_.empty() &&
            static_cast<std::size_t>(hfsImageOffset_) + hd_.size() <= scsiImage_.size())
            hd_.assign(scsiImage_.begin() + hfsImageOffset_,
                       scsiImage_.begin() + hfsImageOffset_ + hd_.size());
        return hd_;
    }
    u32 hdAccessCount() const { return hdReads_ + hdWrites_; }

    // A second, independent SCSI disk (ID 1, drive 5, unit 33) — the folder
    // disk rides here so the user's main hard disk keeps its slot. Attach it
    // before boot (the ROM's bus scan runs its driver and mounts it); read the
    // image back out to persist the guest's writes.
    void insertHardDisk2(std::vector<u8> image, bool readOnly = false);
    void detachHardDisk2();
    bool hardDisk2Present() const { return !hd2_.empty(); }
    const std::vector<u8>& hardDisk2Image() const;
    // Was a second disk on the bus when the ROM's boot scan ran? Only then is
    // its on-disk driver resident, and only a resident driver lets a disk
    // attached MID-SESSION mount without a restart. The front end asks this to
    // decide between "it appears in a few seconds" and offering a restart.
    bool hardDisk2DriverResident() const { return bootHadHd2_; }
    // Flush and unmount ONLY the second disk's volume (drive 5), leaving the
    // boot volume alone. This is how a drop box republishes: the guest must let
    // go of the volume -- and write its cached blocks through our driver into
    // the image -- before the host may rebuild it and put it back. Returns true
    // once the volume is off line, which includes it never having been mounted.
    bool unmountSecondDisk();
    // Push out every block the File Manager is still holding, before the host
    // writes an image to a file the user keeps. Our driver serves writes
    // synchronously, so the image is current for everything the System has
    // ISSUED -- but not for what it is still caching, and persisting without
    // this saves a volume that was consistent at no point in time.
    bool flushVolumes();

    // CD-ROM: an AppleCD SC-class SCSI drive. The drive is attached to the bus
    // (a device, persisting across discs); a disc image is inserted into it
    // (media, always read-only -- nothing is ever copied back out). The guest
    // ejects discs on its own -- the Finder's drag to the Trash -- so a front
    // end polls cdPresent() rather than trusting its last action, exactly like
    // the floppies.
    void attachCdRom(bool attached, int busId = 3);
    bool cdRomAttached() const;
    InsertVerdict insertCd(std::vector<u8> image);
    void ejectCd();
    bool cdPresent() const;
    const char* cdMediumText() const { return cdMediumText_; }

    // Parameter RAM: the one part of a Macintosh that survives being switched
    // off. Held only in memory it comes back blank every launch and the machine
    // forgets every setting a user made. The blob is opaque -- 256 bytes of
    // XPRAM as the guest wrote them, plus the clock -- so whatever a control
    // panel stores is what comes back. `addSeconds` is how long the machine was
    // off; a real one keeps counting on its battery.
    static constexpr u32 kPramBlobBytes = 268;
    u32 savePram(u8* out, u32 cap) const;
    bool loadPram(const u8* data, u32 len, u32 addSeconds);

    // Networking: a DaynaPORT SCSI/Link Ethernet adapter (SCSI ID 4). The
    // device moves raw Ethernet frames; the front end owns the backend (its
    // user-mode NAT). Inject queues a host frame for the guest; drain hands
    // out one guest frame, empty result = nothing waiting.
    void attachNet(bool attached, int busId = 4);
    bool netAttached() const;
    bool netInject(const u8* frame, u32 len);
    bool netDrain(std::vector<u8>& out);
    // Bytes each mechanism has actually handed the controller. A drive the ROM
    // addresses but never reads from is the signature of a phantom bay.
    u32 surfaceReads(int which) const;
    int hardDiskDriveNum() const { return hdDriveNum_; }   // 0 until Open adds it
    u32 diskEvtPosts() const { return diskEvtPosts_; }
    u32 diskEvtResult() const { return diskEvtResult_; }

    // Move the audio produced since the last call (unsigned 8-bit mono at the
    // ~22.25 kHz scanline rate) into `out`; the internal buffer is emptied.
    void drainAudio(std::vector<u8>& out);

    // Host input, delivered through the ADB devices.
    void mouseMove(int dx, int dy, bool button);
    void keyEvent(u8 adbCode, bool down);
    bool keyHeld(u8 adbCode) const;

    // Post a mouseDown/mouseUp event into the OS event queue at the current cursor
    // location (via _PostEvent). On real hardware these events are queued at
    // interrupt time, so both GetNextEvent and the low-level GetOSEvent see them;
    // our ADB path only updates the button low-mem, which the ROM's GetNextEvent
    // re-derives but GetOSEvent (used by e.g. the Installer) does not. Test/driver
    // helper for headless UI automation.
    void postMouseButton(bool down);

    // Force the built-in ROM disk to boot (System 6) by holding the
    // Cmd-Opt-X-O keys down in the KeyMap through the boot-device search,
    // which is what the physical key combo does.
    void setForceRomDisk(bool on) { forceRomDisk_ = on; }
    u32 keyMapReads() const { return keyMapReads_; }
    u32 keyMapReadPc() const { return keyMapReadPc_; }
    int keyMapPcCount() const { return keyMapPcN_; }
    u32 keyMapPc(int i) const { return keyMapPcs_[i]; }
    u8 adbLastCommand() const;

    struct AdbStats {
        u32 mousePolls, kbdPolls, mouseReports;
        u32 kbdReg2, kbdReg3, mouseReg3;
    };
    AdbStats adbStats() const;

    // NCR 5380 activity, for the SCSI bring-up trace.
    struct ScsiStats {
        u32 reads, writes, selects, commands, dataInBytes, dataOutBytes;
        u8 lastCdb[12];
        int lastCdbLen;
        // Live bus state at sample time -- a run that ends wedged mid-transaction
        // shows WHERE the bus stuck (phase != 0/BusFree with xferPos < xferLen).
        int phase;               // Ncr5380::Phase
        u32 xferPos, xferLen;    // progress through the current data/status transfer
        int cdbPos, cdbLen;      // progress through the current CDB
    };
    ScsiStats scsiStats() const;
    int scsiWriteTrace(u16* out, int maxN) const;   // first N register writes, (reg<<8)|val
    int scsiCdbHist(u8* out, int maxCdbs) const;     // first N CDBs, 12 bytes each

    const std::vector<u8>& adbCmdTrace() const;
    const std::vector<u8>& adbRespTrace() const;

    // VIA register snapshot for the monitor (the Via6522 type is internal, so
    // the debugger and trace tool read state through this instead).
    struct ViaRegs {
        u8 ora, orb, ddra, ddrb, acr, pcr, ifr, ier, sr;
        u16 t1c, t2c;
        bool irq;
    };
    ViaRegs viaRegs() const;

    // ADB bus event trace for the monitor: fires on state changes and shift
    // in/out with the current ADB state (0=cmd 1/2=data 3=idle) and the byte.
    // ev is one of "state", "arm", "shiftOut", "shiftIn". Diagnostics only.
    std::function<void(const char* ev, int state, u32 value)> onAdbEvent;

    // Rare, always-on diagnostics (disk insert/mount results, etc.) routed to the
    // GUI log so a "swap didn't work" report has hard numbers behind it.
    std::function<void(const char* msg)> onDiag;

    // Disk-chip (IWM/SWIM) bus trace: every soft-switch access, with the register
    // index 0-15, direction, the byte, the line state after the access, the PC and
    // the Sony drive-register address the phase lines currently select (CA2 CA1 CA0
    // SEL, 0-15). Diagnostics only -- used to survey what the ROM's real .Sony
    // driver demands of the hardware before the shim is replaced by real emulation.
    std::function<void(int reg, bool write, u8 data, u8 lines, u32 pc, int driveReg)> onIwmAccess;

    // Let the disk chip answer the ROM's SWIM probe and unlock its ISM register
    // set. The probe's outcome decides whether the driver works the chip as an
    // IWM (software GCR) or as a SWIM (hardware MFM), so this switches the whole
    // disk path over; it stays off until the ISM data path is finished.
    void setSwimEnabled(bool on);
    bool iwmInIsmMode() const;

    // Force individual Sony drive status lines on the internal drive (bit n of
    // mask selects line n, bit n of value is the level). Investigation aid for
    // the drive-register assignments the documentation never pinned down.
    void setSonyLineOverride(u16 mask, u16 value);
    // How many commands the ROM has issued to each mechanism, by register.
    u32 sonyCommandCount(int drive, int reg) const;
    // Data-register reads the ROM made, and disk bytes the surface handed over.
    u32 iwmDataReads() const { return iwmDataReads_; }
    u32 iwmDataBytes() const { return iwmDataBytes_; }
    // Bytes pushed at the write head, and tracks decoded back into the image.
    u32 iwmDataWrites() const { return iwmDataWrites_; }
    u32 floppyTrackWrites() const { return floppyWrites_; }

    // The ROM's own GCR sector reader says why a read failed: each bail-out
    // loads a Sony driver error code into D0 and branches to a common exit.
    // Watching those exits turns "the disk is unreadable" into "the address
    // mark was never found", which is the difference between guessing at the
    // nibble stream and measuring it. Codes are $B8-$BE (-72..-66).
    std::function<void(u8 code, u32 pc)> onGcrError;
    u32 gcrErrorCount(u8 code) const {
        const int i = static_cast<int>(code) - 0xB8;
        return (i >= 0 && i < 8) ? gcrErrors_[i] : 0;
    }
    static const char* gcrErrorName(u8 code);
    // When a read error first and last occurred, and how many came from the
    // external drive. Errors confined to the boot-time media probe and errors
    // arriving steadily under a running System mean very different things.
    u32 gcrErrorFirstFrame(u8 code) const {
        const int i = static_cast<int>(code) - 0xB8;
        return (i >= 0 && i < 8) ? gcrErrFirstFrame_[i] : 0;
    }
    u32 gcrErrorLastFrame(u8 code) const {
        const int i = static_cast<int>(code) - 0xB8;
        return (i >= 0 && i < 8) ? gcrErrLastFrame_[i] : 0;
    }
    u32 gcrErrorExternal(u8 code) const {
        const int i = static_cast<int>(code) - 0xB8;
        return (i >= 0 && i < 8) ? gcrErrExternal_[i] : 0;
    }

    // Unmapped/stub access log (instrument first): capped, newest last.
    const std::vector<std::string>& accessLog() const { return accessLog_; }
    void clearAccessLog() { accessLog_.clear(); }

    // IBus (the CPU's view of the machine)
    u8   read8(u32 addr) override;
    u16  read16(u32 addr) override;
    void write8(u32 addr, u8 value) override;
    void write16(u32 addr, u16 value) override;

    // Run a Mac A-line trap synchronously on the guest, nested inside whatever
    // it was doing. Only PC/SR are preserved -- the caller saves and restores
    // any registers it needs, checks the owning manager's busy flag first
    // (File Manager: bit 0 of $0360), and takes the same care a driver would.
    // Public for the test harness's guest-side exercises.
    void execute68kTrap(u16 trap);

private:
    void wireVia();
    void logAccess(const char* what, u32 addr, bool write, u32 value);
    void logScsiAccess(u32 addr, bool write, u32 value);
    void tickDevices(int cpuCycles);

    // 16/32-bit big-endian views over the bus, for reading Mac parameter
    // blocks and buffers from driver handlers.
    u32 read32(u32 addr) { return (static_cast<u32>(read16(addr)) << 16) | read16(addr + 2); }
    void write32(u32 addr, u32 v) {
        write16(addr, static_cast<u16>(v >> 16));
        write16(addr + 2, static_cast<u16>(v));
    }

    // High-level .Sony driver replacement. We intercept the ROM driver's
    // Open/Prime/Control/Status routines and service disk I/O from floppy_
    // directly. Locating the driver yields its four routine entry points.
    u32 findSonyDriver();          // ROM address of the .Sony DRVR, or 0
    bool trySonyTrap();            // dispatch if the PC is a driver routine
    void findGcrErrorSites(u32 drvrBase);   // locate the driver's error exits in the ROM
    void noteGcrError(u32 pc);     // one of them was reached
    void watchSonyPrime();         // log a read/write the ROM's own driver is about to do
    void watchSonyResult();        // log the result the ROM's own driver hands back
    void describeMedium(int drive, const char* which, const char* refusal);
    void flushTrack(SonyDrive& d);  // decode a written track back into that drive's image
    void flushFloppyTrack();       // ...the internal drive's, which the host persists
    void ismService(SonyDrive& d); // move bytes between the ISM FIFO and the surface
    void installHardDisk();       // give the hard disk its deferred mount
    int sonyPrime(u32 pb, u32 dce);
    int sonyControl(u32 pb, u32 dce);
    int sonyStatus(u32 pb, u32 dce);

    // IWM/SWIM disk controller. The chip itself lives in Iwm; the machine owns
    // the drives and feeds it the selected drive status line before each access.
    u8 iwmAccess(int reg, bool write, u8 data);
    int iwmDriveReg() const;   // Sony drive-register address CA2:CA1:CA0:SEL (0-15)
    bool iwmSenseLine();       // level of the drive status line that address selects

    std::vector<u8> floppy_;
    std::vector<u8> floppy2_;      // the external drive's medium
    // Containers stripped on insertion -- MacBinary around DiskCopy 4.2 around
    // the sectors, either or both -- kept so the image is handed back out in
    // the file format it arrived in, reassembled around the guest's writes.
    struct MediumWrapper {
        std::vector<u8> mbHeader, mbResource;   // MacBinary, when the file wore it
        std::vector<u8> dcHeader, dcTags;       // DiskCopy 4.2, likewise
        void clear() { *this = MediumWrapper{}; }
        // Put the wrappers back around the sector data, innermost first.
        std::vector<u8> reassemble(const std::vector<u8>& data) const;
    };
    MediumWrapper floppyWrap_, floppy2Wrap_;
    // A floppy put in before the ROM-boot System is running. The ROM's early
    // drive poll announces a power-on disk while the System is still coming up,
    // and startup flushes the event queue -- the announcement is discarded and
    // nothing ever repeats it, so the disk sits in the drive unmounted. Staging
    // the disk until the System can act makes its insertion the same event a
    // user's insertion is, which is the path that demonstrably mounts.
    std::vector<u8> floppyStaged_;
    MediumWrapper floppyStagedWrap_;
    bool floppyStagedRO_ = false;
    void seatFloppy(std::vector<u8> image, MediumWrapper wrap, bool readOnly);
    std::vector<u8> mediumOut_;
    char mediumText_[2][224] = {{0}, {0}};
    // Peel MacBinary and DiskCopy wrappers off `image` into `wrap` and judge
    // what is left. Returns nullptr for mountable floppy media, else the
    // refusal text (which is also stored for mediumText()).
    const char* classifyMedium(int drive, std::vector<u8>& image, MediumWrapper& wrap);
    bool floppyRO_ = false;
    u32 sonyOpenPc_ = 0, sonyPrimePc_ = 0, sonyControlPc_ = 0, sonyStatusPc_ = 0;
    // Error exits of the ROM's GCR reader, kept as a span plus a short list so
    // the per-instruction test is a single unsigned compare.
    struct GcrErrSite { u32 pc; u8 code; };
    std::vector<GcrErrSite> gcrErrSites_;
    u32 gcrErrLo_ = 0, gcrErrSpan_ = 0;
    u32 gcrErrors_[8] = {};
    u32 gcrErrFirstFrame_[8] = {}, gcrErrLastFrame_[8] = {}, gcrErrExternal_[8] = {};
    int gcrErrLog_ = 40;           // diag budget: report the first few, then count only
    int sonyPrimeLog_ = 40;        // diag budget for the ROM driver's own I/O requests
    u32 sonyResultPc_ = 0;         // where the ROM's own driver hands its result back
    int sonyResultLog_ = 40;
    u32 sonyResults_ = 0, sonyResultErrs_ = 0;
    // Read verification: what the last Prime was asked for, so what it delivers
    // can be compared against the medium once it reports success.
    bool verifyReads_ = false, verifyPending_ = false;
    int verifyDrive_ = 1;          // which drive the pending verification read hit
    s64 hfsWatch_ = -1;            // watched HFS-volume byte offset (-1 = off)
    u32 verifyPos_ = 0, verifyLen_ = 0, verifyBuf_ = 0;
    u32 readsVerified_ = 0, readMismatches_ = 0;
    int verifyLog_ = 20;
    void verifyLastRead();
    u32 drvStatusAddr_ = 0;        // Mac address of our DrvSts record
    bool inSony_ = false;          // re-entrancy guard during trap execution

    mutable std::vector<u8> hd_;   // hard-disk image (empty = none); synced from scsiImage_ on read-out
    std::vector<u8> scsiImage_;    // hd_ wrapped in an Apple partition structure for the SCSI bus
    u32 hfsImageOffset_ = 0;       // byte offset of the HFS volume within scsiImage_
    mutable std::vector<u8> hd2_;  // second SCSI disk (the folder disk), same lifecycle
    std::vector<u8> scsiImage2_;
    u32 hfsImageOffset2_ = 0;
    // The ROM only mounts the BOOT volume; every other disk on a real Mac
    // appears because something posts its mount. Drive 5 gets the same
    // machine-side _MountVol injection that drive 4 grew for mid-session
    // attaches, staggered against it and under the same File-Manager-idle
    // gates.
    bool hd2Installed_ = false;    // mount param block allocated
    u32 hd2MountPb_ = 0;
    bool hd2Mounted_ = false;
    u32 hd2MountTries_ = 0;
    bool bootHadHd2_ = false;      // disk 2 present when the boot scan ran
    bool scsiHandlesHd_ = true;    // true: SCSI driver owns the HD (skip .Sony HD reg); false: .Sony still mounts
    bool hdRO_ = false;
    bool hdInstalled_ = false;     // the hard disk's deferred mount is set up
    int hdDriveNum_ = 0;
    u32 hdReads_ = 0, hdWrites_ = 0;   // block-I/O counters (feasibility probe)
    u32 hdMountPb_ = 0;                // system-heap param block for _MountVol
    u32 diskEvtPosts_ = 0;             // mount attempts
    u32 diskEvtResult_ = 0xFFFFFFFFu;  // last _MountVol OSErr
    bool hdMounted_ = false;           // volume mounted OK; stop retrying
    // Enabled by sonyOpen once a hard disk is configured; the mount trigger then
    // runs _MountVol with the .Sony intercept consulted (see execute68kTrap).
    bool hdAutoMount_ = false;
    int sonyLogBudget_ = 0;            // trace the next N .Sony driver calls after a swap
    int floppyEjectSense_ = 0;         // frames to report "no disk" so the ROM sees a swap edge
    int cstinLogBudget_ = 0;           // trace the next N disk-in-place sense reads after a swap

    // The last few distinct status lines the driver read, so a drive command can
    // be explained by the poll sequence that led to it.
    struct SenseSample { u32 frame; u32 pc; u8 addr; u8 level; };
    static constexpr unsigned kSenseRing = 16;
    SenseSample senseRing_[kSenseRing] = {};
    unsigned senseRingN_ = 0;
    static const char* senseLineName(int addr);
    void dumpSenseRing(const char* why);

    std::vector<u8> ram_;
    std::vector<u8> rom_;
    u32 ramMask_ = 0;
    u32 romMask_ = 0;
    bool overlay_ = true;
    bool screenAlt_ = false;
    bool soundAlt_  = false;   // PA3 (vSndPg2) sound-page select, independent of screenAlt_
    u32  bootTraceFrame_ = 0;  // boot-trace heartbeat frame counter (restarts each reset())

    std::unique_ptr<Via6522> via_;
    std::unique_ptr<Rtc> rtc_;
    std::unique_ptr<AdbTransceiver> adb_;
    std::unique_ptr<Ncr5380> scsi_;
    std::unique_ptr<ScsiDisk> disk2_;   // second disk target (SCSI ID 1)
    std::unique_ptr<ScsiCdRom> cdrom_;
    std::unique_ptr<ScsiEthernet> netdev_;
    char cdMediumText_[224] = {0};
    std::unique_ptr<Iwm> iwm_;
    // Internal (drive 1) and external (drive 2) Sony mechanisms. The Classic ships
    // one internal SuperDrive; the external port is empty unless a drive is added.
    // Internal mechanism, external mechanism, and the Classic's empty second
    // internal bay (see selectedDrive).
    std::unique_ptr<SonyDrive> drive0_, drive1_, driveBay2_;
    SonyDrive& selectedDrive();
    void iwmStrobe();          // service an LSTRB-latched drive command
    void iwmUpdateTrack();     // keep the nibble stream matched to the head
    int  trackCacheTrack_ = -1, trackCacheSide_ = -1;
    u32  trackCacheGen_ = 0;
    u32  floppyGen_ = 1;           // bumped whenever the medium is swapped or removed
    int  hdMediaLog_ = 1;          // one-shot note that HD media needs the MFM path
    bool ismActionPrev_ = false;   // edge detect on the ISM ACTION bit
    u16  ismCrcOut_ = 0xFFFF;      // CRC accumulated over bytes written in MFM mode
    bool ismWriteMarkPrev_ = false;  // last written byte was a mark (a field opener)
    bool ismReadMarkPrev_ = false;   // last byte read off the surface was a mark
    int  writeLogBudget_ = 12;     // diag budget for tracks written back to the image
    u32  floppyWrites_ = 0;        // tracks the driver has written through the surface
    u32  floppyEjects_ = 0;        // times the drive threw a disk out on its own
    std::vector<u8> floppyEjected_;// the last medium the drive ejected
    u32  iwmDataWrites_ = 0;       // bytes the driver has pushed at the write head
    int  trackLogBudget_ = 12;
    u32  iwmDataReads_ = 0, iwmDataBytes_ = 0;
    bool lstrbPrev_ = false;   // edge detector for the command strobe
    int  sonyCmdLog_ = 40;     // log the first N drive commands to the diag
    u32  sonyCmds_[2][8] = {}; // per-drive, per-register command counts
    M68000 cpu_;

    // The CPU arms a shift; the transceiver clocks it only in an active
    // state (0/1/2), either when armed there or when the state lines enter
    // one. Idle never clocks — that is what ends a transaction.
    bool adbArmed_ = false;
    bool adbArmedInput_ = false;
    int adbPending_ = 0;        // CPU cycles until delivery (0 = none)
    bool adbPendingInput_ = false;

    // Back-off for the once-per-frame idle wake: if wakes stop producing polls
    // (the ROM is not autopolling, e.g. it is in the boot idle-wait spin), stop
    // nudging and flush stale input so the bus can idle. See runFrame.
    int adbWakeStreak_ = 0;
    u32 adbLastPollTotal_ = 0;

    void adbMaybeClock();

    // True while all four Cmd-Opt-X-O startup keys are physically held: the
    // Mac Classic's request to boot the built-in ROM disk. Used (with the
    // force flag) to drive VIA PA3 low and hold the KeyMap combo through the
    // ROM's boot-device check.
    bool romDiskComboHeld() const;

    u64 totalCycles_ = 0;
    u64 frameCounter_ = 0;
    bool forceRomDisk_ = false;
    bool romDiskKeymapHeld_ = false;   // force path is holding Cmd-Opt-X-O in $0174
    u32 keyMapReads_ = 0;
    u32 keyMapReadPc_ = 0;
    u32 keyMapPcs_[12]{};
    int keyMapPcN_ = 0;
    u64 lineTarget_ = 0;
    int viaRemainder_ = 0;
    u64 secondAcc_ = 0;
    int ca2PulseLines_ = 0;

    // Minimal Z8530 SCC: shared register pointer, enough status for the ROM
    // (RR0 = tx buffer empty, RR1 = all sent). Real serial arrives later.
    int sccPtr_ = 0;
    u8 sccRegs_[16]{};

    std::vector<std::string> accessLog_;
    std::vector<u8> audioOut_;
};

} // namespace openmac
