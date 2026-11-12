#pragma once

// The host's side of a 68040 Macintosh's mass storage, shared by the boards
// that have it: hard-disk images on two seats, the CD-ROM drive, mounting and
// unmounting on the front end's behalf, the shutdown flush, and the guest
// calls those take. A board derives from MacHostServices, attaches the three
// SCSI devices (disk_, disk2_, cdrom_) to its own bus, and calls in from
// three places: serveStorageHooks before each instruction, storageSecondTick
// and storageFrameTick from its device clock, machineStarting when the CPU is
// at the ROM's entry. Nothing here knows an I/O address.

#include "openmac/bus040.hpp"
#include "openmac/cpu040.hpp"
#include "openmac/types.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace openmac {

class ScsiDisk;
class ScsiCdRom;

class MacHostServices {
public:
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
    // Where a disc's HFS volume starts inside its flat image: the Apple
    // partition map's Apple_HFS entry, or 0 for a bare HFS master. Pure data
    // structure, so it is testable on its own -- and the block size the map is
    // addressed in (512, whatever the disc's sectors are) is the whole trick.
    static u32 findHfsPartition(const std::vector<u8>& disc);
    void ejectCd();
    bool cdPresent() const;

    // Flush and unmount every mounted volume, as Shut Down would. Call before
    // persisting an image when the host application is closing: without it the
    // System's cached blocks are lost and the volume stays flagged unclean,
    // which this ROM refuses at the next boot.
    bool shutdownVolumes();
    // Flush and unmount ONLY the second disk's volume (the second seat), leaving the
    // boot volume alone. This is how the drop box republishes: the guest must
    // let go of the volume -- and write its cached blocks through our driver
    // into the image -- before the host may rebuild it. Returns true once the
    // volume is off line, which includes it never having been mounted.
    bool unmountSecondDisk();
    // Read a file off the second disk's volume THROUGH THE GUEST: its own File
    // Manager, _Open / _Read / _Close, served by our driver hook exactly as an
    // application's read is. Reading the image with our own HFS reader answers
    // a different question -- it says the bytes are on the volume, not that the
    // guest can get at them, and an archiver calling a file corrupt is a claim
    // about the second thing. Diagnostic only; never on a shipping path.
    bool readFileThroughGuest(const std::string& name, std::vector<u8>& out,
                              std::string& why);
    // True once the System has installed and we have hooked the second disk's
    // driver. Until then a volume put on that seat cannot mount, because there
    // is no driver to read it: the boot scan is what loads one.
    bool secondDiskDriverResident() const { return diskPrimePc_[1] != 0; }

    // Test entry: make the guest call ShutDwnStart (restart). False = not now.
    bool guestRestart();
    // Test entry: make the guest call ShutDwnPower, what the Finder's Shut
    // Down ends in. False = not now.
    bool guestPowerOff();
    // Test entry: have the guest play its alert sound (SysBeep) by running
    // the trap on the guest CPU. False = not now.
    bool systemBeep();
    // Ask the guest's Gestalt a question by running the trap on the guest
    // CPU. Headless diagnosis only: the answer is whatever the System's own
    // table holds. Returns the OSErr; the response lands in `response`.
    s32 gestaltQuery(u32 selector, u32& response);
    // Every slot resource the guest's Slot Manager knows, in its own order
    // (SNextSRsrc from slot 0 until it reports no more), by running the trap
    // on the guest CPU. Headless diagnosis only, like gestaltQuery. False =
    // the addressing mode did not allow a call just now.
    struct SlotResource { u8 slot, id; u16 category, type, drvrSw, drvrHw; };
    bool slotResources(std::vector<SlotResource>& out);
    u32 hdReads() const { return hdReadCount_; }
    u32 hdWrites() const { return hdWriteCount_; }
    void traceHdRequests(int n) { hdTraceBudget_ = n; }

    // Diagnosis switch: skip the injected mount/announce of hard disks, to
    // separate "our injection perturbs the boot" from everything else.
    bool suppressHdAnnounce = false;
    // Diagnosis switch: attach a volume exactly as the file holds it, with no
    // repair pass. Set this to reproduce what an old, over-large extents tree
    // does to the ROM's rebuild -- the refusal is invisible once the volume has
    // been put back inside the ROM's reach.
    bool suppressVolumeRepair = false;
    std::function<void(const char* msg)> onDiag;

protected:
    // The board: its bus (physical addresses), its CPU, its RAM. All three
    // may still be under construction when this is called; none is touched
    // before the machine runs.
    MacHostServices(IBus040& bus, M68040& cpu, std::vector<u8>& ram);
    ~MacHostServices();

    // Execute one instruction with the board's devices clocked. Guest calls
    // made from here run through it.
    virtual int stepInstruction() = 0;

    // The CPU is about to execute a driver entry this code serves: serve the
    // request and return true. `frame` only dates the diagnostics.
    bool serveStorageHooks(u64 frame);
    void storageSecondTick();   // once per emulated second
    void storageFrameTick();    // once per frame

    // Guest memory through the board's bus, as the moved code always read it.
    u8   read8(u32 addr) { return hbus_.read8(addr); }
    u16  read16(u32 addr) { return hbus_.read16(addr); }
    u32  read32(u32 addr) { return hbus_.read32(addr); }
    void write8(u32 addr, u8 value) { hbus_.write8(addr, value); }
    void write16(u32 addr, u16 value) { hbus_.write16(addr, value); }
    void write32(u32 addr, u32 value) { hbus_.write32(addr, value); }

    // The CPU reached the ROM's entry (power on or a guest restart): forget
    // the last startup. lowMemoryFresh says when its unit table is gone too.
    void machineStarting();
    bool lowMemoryFresh();
    void execute68kTrap(u16 trap);
    // True while the MMU is in the other addressing mode from the one the
    // System started in; no trap is injected then.
    bool addressingModeSwapped();
    bool leaveSwappedAddressingMode();
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
        if (p < hram_.size()) return p;
        const u32 stripped = p & 0x00FFFFFFu;
        return stripped < hram_.size() ? stripped : p;
    }
    void serveDiskPrime(int unit);
    void serveDiskCtlStatus(int unit, bool status);
    void findDiskDriverPrime();
    // The CD needs a driver, and the guest does not always have one. The ROM
    // carries one for floppies and the hard disk brings its own on its
    // partition map, but a CD-ROM's driver comes from the disc itself (a
    // startup disc, loaded by the ROM) or from Apple's CD-ROM extension. Where
    // either is there it runs the drive over the SCSI bus and this machine
    // stays out of it (guestCdDrive). A System with neither would never put
    // the drive in the queue and no disc of any format could mount, so for
    // that case install one once startup has finished: a DRVR named ".AppleCD"
    // whose entry points are addresses this machine watches, exactly as it
    // watches the hard disk's Prime and the ROM's .Sony Prime. See
    // BasiliskII/src/cdrom.cpp, which reaches the same conclusion from the
    // other direction.
    void installCdDriver();
    void mountCdVolume();
    void completeCdEject();
    void serveCdPrime();
    void serveCdCtlStatus(bool status);
    bool volumeMountedFor(u16 drive);
    u16 guestCdDrive();
    bool startupFinished();
    bool markVolumeClean(u16 drive);
    static bool markVolumeCleanIn(std::vector<u8>& backing, u32 base, u32 volumeSize);

    IBus040& hbus_;
    M68040& hcpu_;
    std::vector<u8>& hram_;

    bool unitTableFresh_ = false;   // rebuilt since the machine last started
    bool sawNoApp_ = false;         // CurApName seen unset since then
    bool inDriver_ = false;      // serving a driver request: no nested traps
    u32 diskPrimePc_[2] = {0, 0};   // units 32 and 33: the two seats
    u32 diskCtlPc_[2] = {0, 0}, diskStatusPc_[2] = {0, 0};
    // The installed .AppleCD driver: where its stubs live, and the drive it
    // owns. cdDrive_ is 0 until AddDrive has taken.
    u32 cdDrvr_ = 0, cdPrimePc_ = 0, cdCtlPc_ = 0, cdStatusPc_ = 0;
    u32 cdStatus_ = 0;              // the DrvSts the drive queue points into
    s16 cdRefNum_ = 0;
    u16 cdDrive_ = 0;
    u32 cdHfsOffset_ = 0;           // the disc's HFS partition, in bytes
    bool cdMountPending_ = false;
    int cdMountTries_ = 0;
    int cdInstallTries_ = 0;
    bool cdEjectPending_ = false;   // front-end eject, run at a frame edge
    int cdEjectTries_ = 0;
    std::unique_ptr<ScsiDisk> disk_;
    std::unique_ptr<ScsiDisk> disk2_;
    std::unique_ptr<ScsiCdRom> cdrom_;
    mutable std::vector<u8> hd_;
    std::vector<u8> scsiImage_;
    u32 hfsImageOffset_ = 0;
    bool hdRO_ = false;
    mutable std::vector<u8> hd2_;
    std::vector<u8> scsiImage2_;
    u32 hfsImageOffset2_ = 0;
    bool hd2RO_ = false;
    // A hard disk present at power-on entered the drive queue during the ROM
    // scan, whose announcement startup flushes; once the System settles, a
    // disk-inserted event makes it the same event a user's insertion is.
    bool hdAnnouncePending_ = false, hd2AnnouncePending_ = false;
    int  hdAnnounceDelay_ = 0;
    bool announceInFlight_ = false;
    u32  hdMountPb_ = 0;        // System-heap param block for the _MountVol retries
    int  hdMountTries_ = 0;
    u32  hdReadCount_ = 0, hdWriteCount_ = 0;
    int  hdTraceBudget_ = 0;    // traceHdRequests: log every disk request
    int  hdErrBudget_ = 20;     // report the first failed disk requests
};

} // namespace openmac
