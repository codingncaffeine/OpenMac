#include "openmac/hostservices.hpp"

#include "openmac/hfs.hpp"

#include "cdmedia.hpp"
#include "scsi.hpp"
#include "scsicd.hpp"
#include "scsiimage.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

// The host's side of a 68040 Macintosh's mass storage: hard-disk images on
// two seats, the CD-ROM drive, the volume mount and unmount the front end
// asks for, and the guest calls those need. A board (the Quadra 650; the
// Quadra 950 next) derives from MacHostServices, attaches the three SCSI
// devices to its own bus and calls in from its instruction step and its
// device clock. Nothing here knows an I/O address.

namespace openmac {

namespace {
// The drive numbers the two hard-disk seats add themselves under. A driver
// that is loaded properly takes the first free number from 4 up, and one
// can be loaded BEFORE these: the ROM installs the driver off a startup CD
// at SCSI ID 3 ahead of the disk at ID 0, and it takes 4. The seats' own
// numbers are fixed (the driver image carries them), so they sit above
// anything that early loader can reach; every later driver sees them in
// the drive queue and steps past.
constexpr u16 kHdDrive = 8, kHd2Drive = 9;
} // namespace

// The CPU is at the ROM's entry: power on, or a restart the guest started
// itself (ShutDwnStart ends in a jump to ROM base + $0A). Everything this
// machine learned about the last startup is void from here -- the drivers
// it hooked and the one it installed lived in a System heap that is about to
// be rebuilt, and an address kept from it belongs to whatever the new
// startup loads there. With a CD in the drive that is the disc's own driver,
// loaded before the hard disks': it ran into a stale Prime address, the
// machine served a disk request that was never made, and the restart ended
// at the sad Mac.
//
// The CPU's RESET instruction is not this event: the ROM also executes it
// well into a startup, with its drivers already in place.
void MacHostServices::machineStarting() {
    // The drive gets its reset: tray closed, 2048-byte blocks again.
    cdrom_->busReset();
    cdDrvr_ = cdPrimePc_ = cdCtlPc_ = cdStatusPc_ = 0;
    cdStatus_ = 0;
    cdRefNum_ = 0;
    cdDrive_ = 0;
    cdInstallTries_ = 0;
    cdMountTries_ = 0;
    cdHfsOffset_ = cdrom_->discPresent() ? findHfsPartition(cdrom_->media()) : 0;
    cdMountPending_ = cdrom_->discPresent();
    for (int i = 0; i < 2; ++i)
        diskPrimePc_[i] = diskCtlPc_[i] = diskStatusPc_[i] = 0;
    hdMountPb_ = 0;
    hdAnnouncePending_ = !hd_.empty();
    hd2AnnouncePending_ = !hd2_.empty();
    hdMountTries_ = 0;
    // The volume queue is the last startup's: nothing seen in it counts.
    hd2SeenMounted_ = false;
    hd2OffSeconds_ = 0;
    // Low memory still describes the last startup until the ROM rebuilds
    // it. Nothing is read back out of it before then (see lowMemoryFresh).
    unitTableFresh_ = false;
    sawNoApp_ = false;
    // Whatever waited for a call of the last startup's guest is void too.
    storageDue_ = false;
    hd2UnmountAsked_ = 0;
    beepAsked_ = false;
    hcpu_.holdAtCalls = 0;
}

// Has the ROM rebuilt the unit table since the machine last started? The
// old one still holds the hard disks' drivers; a new one is empty at their
// units until the startup scan installs them again, many frames later.
bool MacHostServices::lowMemoryFresh() {
    if (unitTableFresh_) return true;
    const u32 uTable = guestPtr(read32(0x011C));
    if (!uTable || uTable + 34 * 4u >= hram_.size() ||
        (read32(uTable + 32 * 4u) == 0 && read32(uTable + 33 * 4u) == 0))
        unitTableFresh_ = true;
    return unitTableFresh_;
}

// Is a volume already on line for this drive? Walk the VCB queue ($0356 is the
// header, qHead at $0358); each VCB carries its drive number at vcbDrvNum
// (+$6E). This is how we tell "the System mounted it itself" from "nobody has".
bool MacHostServices::volumeMountedFor(u16 drive) {
    u32 vcb = guestPtr(read32(0x0358));
    for (int n = 0; vcb && vcb + 80 < hram_.size() && n < 16; ++n) {
        if (read16(vcb + 72) == drive) return true;   // vcbDrvNum
        vcb = guestPtr(read32(vcb));                  // qLink
    }
    return false;
}

// The drive a CD-ROM driver of the guest's own answers for, or 0. Apple's
// driver -- loaded off a startup disc by the ROM, or installed by the Apple
// CD-ROM extension -- is named .AppleCD like the one this machine installs,
// so the drive queue is walked for an element whose driver carries that name
// under a reference number that is not ours. While there is one, the disc is
// its business over the SCSI bus and this machine's driver stays out: two
// drivers on one drive mount the same volume twice.
u16 MacHostServices::guestCdDrive() {
    if (!lowMemoryFresh()) return 0;
    const u32 uTable = guestPtr(read32(0x011C));
    const u32 units = read16(0x01D2);                 // UnitNtryCnt
    if (!uTable || uTable + units * 4u >= hram_.size()) return 0;
    u32 dqe = guestPtr(read32(0x030A));               // DrvQHdr.qHead
    for (int n = 0; dqe && dqe + 12 < hram_.size() && n < 32; ++n) {
        const s16 ref = static_cast<s16>(read16(dqe + 8));   // dQRefNum
        const u32 unit = static_cast<u16>(~ref);
        if (ref < 0 && unit < units && !(cdDrvr_ && ref == cdRefNum_)) {
            const u32 h = guestPtr(read32(uTable + unit * 4u));
            const u32 dce = (h && h + 4 < hram_.size()) ? guestPtr(read32(h)) : 0;
            if (dce && dce + 8 < hram_.size()) {
                // dCtlDriver is a handle when dRAMBased is set, a pointer
                // when it is not; the name is a Pascal string at +18.
                u32 drvr = guestPtr(read32(dce));
                if ((read16(dce + 4) & 0x0040) && drvr && drvr + 4 < hram_.size())
                    drvr = guestPtr(read32(drvr));
                static const char kName[] = "\010.AppleCD";
                bool match = drvr && drvr + 18 + sizeof kName < hram_.size();
                for (u32 i = 0; match && i + 1 < sizeof kName; ++i)
                    match = read8(drvr + 18 + i) == static_cast<u8>(kName[i]);
                if (match) return read16(dqe + 6);    // dQDrive
            }
        }
        dqe = guestPtr(read32(dqe));                  // qLink
    }
    return 0;
}

// Has startup run its course? CurApName ($0910) is filler until the first
// application -- the Finder -- is launched, and extensions load before that.
// The machine's CD driver waits for this: installed any earlier, it would
// already own the drive when the Apple CD-ROM extension loads and claims it.
bool MacHostServices::startupFinished() {
    const u8 len = read8(0x0910);
    if (len == 0 || len > 31) { sawNoApp_ = true; return false; }
    for (u8 i = 1; i <= len; ++i)
        if (read8(0x0910 + i) < 0x20) { sawNoApp_ = true; return false; }
    // A name left over from before a restart is not this startup's.
    return sawNoApp_;
}

// Locate the Prime entry of each installed .ScsiHD driver, so the machine can
// serve its requests directly. The unit table ($011C) holds a handle per unit;
// the DCE's dCtlDriver points at the DRVR, whose header carries the Prime
// offset. Cached once the System has installed the driver.
void MacHostServices::findDiskDriverPrime() {
    // Unit 32 + SCSI ID, the numbering every SCSI disk driver follows. It is
    // not cosmetic for the first seat: the ROM's default startup device is
    // the driver at reference number -33 (unit 32, the disk at ID 0), and a
    // disk that answers there starts the machine ahead of a startup CD, as
    // the internal disk of a real one does.
    static const int kUnits[2] = {32, 33};
    const u32 uTable = guestPtr(read32(0x011C));
    if (!uTable || uTable + 256 >= hram_.size()) return;
    for (int i = 0; i < 2; ++i) {
        if (diskPrimePc_[i]) continue;
        const u32 h = guestPtr(read32(uTable + static_cast<u32>(kUnits[i]) * 4u));
        if (!h || h + 4 >= hram_.size()) continue;
        const u32 dce = guestPtr(read32(h));
        if (!dce || dce + 4 >= hram_.size()) continue;
        const u32 drvr = guestPtr(read32(dce));       // dCtlDriver
        if (!drvr || drvr + 0x20 >= hram_.size()) continue;
        // Only OUR driver: its name field is ".ScsiHD".
        if (read8(drvr + 0x12) != 7 || read8(drvr + 0x13) != '.' ||
            read8(drvr + 0x14) != 'S' || read8(drvr + 0x15) != 'c')
            continue;
        const u32 prime = drvr + read16(drvr + 0x0A);
        diskPrimePc_[i] = prime;
        diskCtlPc_[i] = drvr + read16(drvr + 0x0C);      // drvrCtl
        diskStatusPc_[i] = drvr + read16(drvr + 0x0E);   // drvrStatus
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
void MacHostServices::serveDiskPrime(int unit) {
    const u32 pb = guestPtr(hcpu_.a[0]);
    const u32 dce = guestPtr(hcpu_.a[1]);
    const u16 trap = read16(pb + 0x06);
    const u32 buf = guestPtr(read32(pb + 0x20));
    const u32 req = read32(pb + 0x24);
    const u16 posMode = read16(pb + 0x2C);
    const u32 posOff = read32(pb + 0x2E);
    const u32 dctlPos = read32(dce + 0x10);

    std::vector<u8>& img = unit ? hd2_ : hd_;
    const bool ro = unit ? hd2RO_ : hdRO_;
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
    // Every request the volume's own driver makes, in order. Two boots of the
    // same disk that diverge are best compared here: this is what the guest
    // asked the medium for, before any interpretation.
    if (onDiag && hdTraceBudget_ > 0) {
        --hdTraceBudget_;
        char b[128];
        std::snprintf(b, sizeof b, "HDIO %s pos=%u(blk %u) req=%u -> %u err=%d",
                      (trap & 1) ? "write" : "read ", pos, pos / 512, req, done,
                      result);
        onDiag(b);
    }
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
    hcpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const u32 target = read32(0x08FC);                 // jIODone
    hcpu_.a[0] = target;
    hcpu_.pc = target;
}

// Where the disc's HFS volume begins. Apple's own CD masters put an Apple
// partition map at the front and the volume inside an "Apple_HFS" partition;
// a plain HFS master has its MDB two blocks in and starts at zero. The map's
// entries are addressed in 512-byte blocks even on a disc whose sectors are
// 2048, which is why this walks blocks rather than sectors.
//
// Reference: Inside Macintosh: Devices, "The SCSI Manager" (partition map), and
// the same walk in BasiliskII/src/cdrom.cpp.
u32 MacHostServices::findHfsPartition(const std::vector<u8>& disc) {
    return cd::findHfsPartition(disc);   // shared with the IIfx (cdmedia.hpp)
}

// Build the .AppleCD driver in the System heap and hand it to the Device
// Manager. Its Open/Prime/Control/Status entry points are four addresses this
// machine watches; when the guest's Device Manager jumps to one, stepInstruction
// serves the request in C++ and returns through jIODone -- the same shape as the
// hard disk's Prime hook, so the guest's DCE, queue and completion path stay its
// own and only the transfer is ours.
//
// This runs once the System is up, for the same reason the hard disk's mount
// does: a driver installed during the ROM's boot scan is installed into a
// machine that has not built its unit table yet.
void MacHostServices::installCdDriver() {
    if (cdDrvr_ || inDriver_ || announceInFlight_) return;
    if (read32(0x0358) == 0) return;                  // System never came up
    if ((((static_cast<u32>(read16(0x0360)) << 16) | read16(0x0362)) == 0xFFFFFFFFu))
        return;                                       // FS queue not built
    if (read8(0x0360) & 1) return;                    // FSBusy: a file op is running
    if ((hcpu_.getSR() & 0x0700) != 0) return;         // mid-interrupt: not now
    if (addressingModeSwapped()) return;              // not now
    if (!startupFinished()) return;                   // extensions first
    if (guestCdDrive()) return;                       // the guest has its own
    if (++cdInstallTries_ > 30) return;

    // Find the slot BEFORE allocating anything: the unit table is not built
    // until the System is well under way, and an attempt that gives up after
    // the allocation leaks a block of the System heap every time it retries.
    const u32 uTable = guestPtr(read32(0x011C));
    const u32 units = read16(0x01D2);                 // UnitNtryCnt
    u32 unit = 0;
    for (u32 i = 34; i < units && i < 128; ++i) {     // 0-31 are Apple's,
                                                      // 32 and 33 the disks'
        if (uTable + i * 4u + 3 >= hram_.size()) break;
        if (read32(uTable + i * 4u) == 0) { unit = i; break; }
    }
    if (!uTable || !unit) return;                     // not yet; try again later

    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = hcpu_.d[i]; sa[i] = hcpu_.a[i]; }

    // DRVR header: flags, delay, event mask, menu, then the five entry-point
    // offsets, then the name as a Pascal string. Our entry points are stubs the
    // guest never actually executes, but they have to BE somewhere.
    const u32 kHeader = 0x1C;                         // header + ".AppleCD" + pad
    hcpu_.d[0] = kHeader + 0x20;
    execute68kTrap(0xA71E);                           // _NewPtr,Sys,Clear
    const u32 drvr = guestPtr(hcpu_.a[0]);
    if (!drvr) {
        for (int i = 0; i < 8; ++i) { hcpu_.d[i] = sd[i]; hcpu_.a[i] = sa[i]; }
        announceInFlight_ = false;
        return;
    }
    // dReadEnable | dWritEnable | dCtlEnable | dStatEnable | dNeedLock.
    write16(drvr + 0x00, 0x4F00);                     // drvrFlags
    write16(drvr + 0x02, 0);                          // drvrDelay
    write16(drvr + 0x04, 0);                          // drvrEMask
    write16(drvr + 0x06, 0);                          // drvrMenu
    write16(drvr + 0x08, u16(kHeader + 0x00));        // drvrOpen
    write16(drvr + 0x0A, u16(kHeader + 0x08));        // drvrPrime
    write16(drvr + 0x0C, u16(kHeader + 0x10));        // drvrCtl
    write16(drvr + 0x0E, u16(kHeader + 0x18));        // drvrStatus
    write16(drvr + 0x10, u16(kHeader + 0x00));        // drvrClose -> the Open stub
    const char* name = "\010.AppleCD";
    for (int i = 0; i < 9; ++i) write8(drvr + 0x12 + u32(i), u8(name[i]));
    // Prime, Control and Status are QUEUED calls: they finish through IODone,
    // so a stub that jumps there is a correct driver on its own and nothing is
    // stranded if the hook ever misses.
    for (u32 off : {0x08u, 0x10u, 0x18u}) {
        const u32 at = drvr + kHeader + off;
        write16(at + 0, 0x7000);                      // MOVEQ #0,D0
        write16(at + 2, 0x2078); write16(at + 4, 0x08FC);   // MOVEA.L $08FC.W,A0
        write16(at + 6, 0x4ED0);                      // JMP (A0)
    }
    // ⛔ Open and Close are NOT. They are immediate calls and must RTS with the
    // result in D0. Sending them through IODone asks the Device Manager to
    // complete a request nobody queued -- it dequeues from a DCE queue that is
    // empty and takes a completion routine out of a parameter block that is not
    // one. Nothing faults there; the damage lands later, in a trap table that
    // dispatches to zero and a System heap full of a repeating fill.
    {
        const u32 at = drvr + kHeader + 0x00;
        write16(at + 0, 0x7000);                      // MOVEQ #0,D0
        write16(at + 2, 0x4E75);                      // RTS
        write16(at + 4, 0x4E71); write16(at + 6, 0x4E71);   // NOP NOP (padding)
    }
    cdDrvr_ = drvr;
    cdPrimePc_ = drvr + kHeader + 0x08;
    cdCtlPc_ = drvr + kHeader + 0x10;
    cdStatusPc_ = drvr + kHeader + 0x18;

    // Into the unit table by hand rather than through _DrvrInstall. That trap
    // takes a refNum the caller has to have picked already, and picking one by
    // trying them in turn means writing over whatever is there when the guess
    // is wrong -- which is exactly what it did: low memory came back as
    // $6DB6DB6D and the machine was gone. Finding an empty slot first is the
    // same work without the guessing.
    const s16 refNum = static_cast<s16>(~unit);
    cdRefNum_ = refNum;
    // The unit table holds HANDLES to device control entries, so the DCE is a
    // relocatable block -- and it must not move under the Device Manager, hence
    // the lock.
    hcpu_.d[0] = 50;                                   // AuxDCE
    execute68kTrap(0xA722);                           // _NewHandle,Sys,Clear
    const u32 dceH = hcpu_.a[0];
    if (dceH) { hcpu_.a[0] = dceH; execute68kTrap(0xA029); }   // _HLock
    const u32 dce = dceH ? guestPtr(read32(guestPtr(dceH))) : 0;
    if (!dce) {
        cdDrvr_ = cdPrimePc_ = cdCtlPc_ = cdStatusPc_ = 0;
        for (int i = 0; i < 8; ++i) { hcpu_.d[i] = sd[i]; hcpu_.a[i] = sa[i]; }
        announceInFlight_ = false;
        if (onDiag) onDiag("cd: no memory for the driver's control entry");
        return;
    }
    write32(dce + 0x00, drvr);                        // dCtlDriver (a pointer)
    write16(dce + 0x04, u16(0x4F00 | 0x0020));        // dCtlFlags | dOpened
    write16(dce + 0x18, static_cast<u16>(refNum));    // dCtlRefNum
    write32(uTable + unit * 4u, dceH);

    // The drive itself: a DrvSts the drive queue links through, then AddDrive.
    hcpu_.d[0] = 22;                                   // SIZEOF DrvSts
    execute68kTrap(0xA71E);                           // _NewPtr,Sys,Clear
    cdStatus_ = guestPtr(hcpu_.a[0]);
    if (cdStatus_) {
        write8(cdStatus_ + 2, 0x80);                  // dsWriteProt: locked
        write8(cdStatus_ + 3, cdrom_->discPresent() ? 1 : 0);   // dsDiskInPlace
        write8(cdStatus_ + 4, 1);                     // dsInstalled
        write8(cdStatus_ + 5, 1);                     // dsSides
        cdDrive_ = 6;                                 // past the two SCSI seats
        hcpu_.d[0] = (u32(cdDrive_) << 16) | u16(refNum);
        hcpu_.a[0] = cdStatus_ + 6;                    // the DrvQEl itself
        execute68kTrap(0xA04E);                       // _AddDrive
        cdMountPending_ = cdrom_->discPresent();
        cdMountTries_ = 0;
    }
    for (int i = 0; i < 8; ++i) { hcpu_.d[i] = sd[i]; hcpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
    if (onDiag) {
        char b[144];
        std::snprintf(b, sizeof b,
                      "cd: .AppleCD installed at unit %u (refNum %d), drive %u, "
                      "Prime at %08X", unit, refNum, cdDrive_, cdPrimePc_);
        onDiag(b);
    }
}

// Ask the File Manager to mount the disc, the same way the hard disks are
// mounted: _MountVol on the drive, gated on the System being settled. A
// disk-inserted event would be the other way, and the hard disk's history says
// that event makes 7.5's startup re-run a mount it has already failed.
void MacHostServices::mountCdVolume() {
    if (!cdMountPending_ || !cdDrive_) return;
    if (inDriver_ || announceInFlight_) return;
    if (read32(0x0358) == 0) return;                  // System never came up
    if (read8(0x0360) & 1) return;                    // FSBusy
    if ((hcpu_.getSR() & 0x0700) != 0) return;         // mid-interrupt
    if (addressingModeSwapped()) return;              // not now
    if (volumeMountedFor(cdDrive_)) { cdMountPending_ = false; return; }
    if (++cdMountTries_ > 15) { cdMountPending_ = false; return; }
    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = hcpu_.d[i]; sa[i] = hcpu_.a[i]; }
    if (!hdMountPb_) {
        hcpu_.d[0] = 80;
        execute68kTrap(0xA71E);                       // _NewPtr,Sys,Clear
        hdMountPb_ = guestPtr(hcpu_.a[0]);
    }
    s16 res = -108;
    if (hdMountPb_) {
        for (u32 i = 0; i < 80; ++i) write8(hdMountPb_ + i, 0);
        write16(hdMountPb_ + 22, cdDrive_);           // ioVRefNum = the drive
        hcpu_.a[0] = hdMountPb_;
        execute68kTrap(0xA00F);                       // _MountVol
        res = static_cast<s16>(hcpu_.d[0] & 0xFFFF);
    }
    for (int i = 0; i < 8; ++i) { hcpu_.d[i] = sd[i]; hcpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
    if (res == 0 || res == -55) cdMountPending_ = false;   // mounted, or already
    if (onDiag) {
        char b[80];
        std::snprintf(b, sizeof b, "cd: mount drive %u -> %d", cdDrive_, res);
        onDiag(b);
    }
}

// The CD driver's Prime: read-only, served straight out of the disc image at
// the HFS partition's offset. A CD is addressed by the File Manager in 512-byte
// logical blocks whatever the drive's own sector size is, and the image is a
// flat byte run, so the translation is one addition.
void MacHostServices::serveCdPrime() {
    const u32 pb = guestPtr(hcpu_.a[0]);
    const u32 dce = guestPtr(hcpu_.a[1]);
    const u16 trap = read16(pb + 0x06);
    const u32 buf = guestPtr(read32(pb + 0x20));
    const u32 req = read32(pb + 0x24);
    const u16 posMode = read16(pb + 0x2C);
    const u32 posOff = read32(pb + 0x2E);
    const u32 dctlPos = read32(dce + 0x10);
    const std::vector<u8>& disc = cdrom_->media();
    const u32 volSize = disc.size() > cdHfsOffset_
                            ? static_cast<u32>(disc.size()) - cdHfsOffset_ : 0;

    u32 pos = dctlPos;
    switch (posMode & 3) {
    case 1: pos = posOff; break;                                   // fsFromStart
    case 2: pos = volSize + posOff; break;                         // fsFromLEOF
    case 3: pos = dctlPos + posOff; break;                         // fsFromMark
    default: break;                                                // fsAtMark
    }
    s16 result = 0;
    u32 done = 0;
    if (!cdrom_->discPresent() || volSize == 0) {
        result = -65;                                  // offLinErr
    } else if (trap & 1) {
        result = -44;                                  // wPrErr: it is a CD
    } else if (pos >= volSize) {
        result = -39;                                  // eofErr
    } else {
        u32 n = req;
        if (pos + n > volSize) { n = volSize - pos; result = -39; }
        const u32 at = cdHfsOffset_ + pos;
        for (u32 i = 0; i < n; ++i) write8(buf + i, disc[at + i]);
        done = n;
    }
    write32(pb + 0x28, done);                          // ioActCount
    write32(dce + 0x10, pos + done);                   // dCtlPosition
    write16(pb + 0x10, static_cast<u16>(result));      // ioResult
    hcpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const u32 target = read32(0x08FC);                 // jIODone
    hcpu_.a[0] = target;
    hcpu_.pc = target;
}

// Control and Status for the CD. The one that matters is csCode 7, Eject: the
// Finder's drag-to-Trash and Special > Eject Disk both come through here, and a
// drive that answers "done" without letting go leaves an icon on a disc the
// host has already taken back.
void MacHostServices::serveCdCtlStatus(bool status) {
    const u32 pb = guestPtr(hcpu_.a[0]);
    const u16 csCode = read16(pb + 0x1A);
    s16 result = 0;
    if (!status) {
        switch (csCode) {
        case 7:                                        // Eject
            // The same tray the drive's own eject command opens: the disc
            // is back when the machine restarts.
            cdrom_->guestEject();
            if (cdStatus_) write8(cdStatus_ + 3, 0);   // dsDiskInPlace
            if (onDiag) onDiag("cd: the guest ejected the disc");
            break;
        case 21: case 22:                              // icon / drive info
            result = -17;                              // controlErr: not ours
            break;
        case 5: case 6: case 8: case 9: case 23: case 24:
            break;                                     // verify, format, tag: fine
        default:
            break;
        }
    } else if (csCode == 8) {                          // DriveStatus
        if (cdStatus_)
            for (u32 i = 0; i < 22; ++i) write8(pb + 0x1C + i, read8(cdStatus_ + i));
    }
    write16(pb + 0x10, static_cast<u16>(result));      // ioResult
    hcpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const u32 target = read32(0x08FC);                 // jIODone
    hcpu_.a[0] = target;
    hcpu_.pc = target;
}

// The driver's Control and Status entries, answered the way the Classic
// answers them for its own hard disk. Our on-disk driver stubs both to "no
// error" without touching the parameter block, so anything that asks the
// drive about itself -- a format list, a drive size -- reads back whatever
// was in the block already.
void MacHostServices::serveDiskCtlStatus(int unit, bool status) {
    const u32 pb = guestPtr(hcpu_.a[0]);
    const u16 csCode = read16(pb + 0x1A);
    std::vector<u8>& img = unit ? hd2_ : hd_;
    s16 result = 0;
    if (status) {
        // csCode 6, the format list: one format covering the whole medium.
        if (csCode == 6) {
            write16(pb + 0x1C, 1);
            write32(pb + 0x1E, static_cast<u32>(img.size() / 512));
        }
    } else {
        // KillIO is refused; a fixed disk cannot be ejected and the rest
        // (verify, format, tag buffer, track cache) are done already. The
        // icon calls (21 = drive icon, 22 = media icon) must be refused too:
        // they answer with a POINTER in csParam, and "no error" over whatever
        // the block happened to hold sends the caller off to draw from a wild
        // address. controlErr makes the caller use its default icon instead.
        if (csCode == 1 || csCode == 21 || csCode == 22)
            result = -17;                           // controlErr
    }
    write16(pb + 0x10, static_cast<u16>(result));   // ioResult
    hcpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const u32 target = read32(0x08FC);              // jIODone
    hcpu_.a[0] = target;
    hcpu_.pc = target;
}

// Run one A-line trap to completion on the guest CPU: the opcode goes in a
// scratch cell at the top of RAM, the PC points at it, and stepping continues
// until control returns to the following word (the dispatcher adjusts the
// return PC past the A-line). The Classic's proven injection shape.
// A System running in 24-bit mode swaps the MMU to 32-bit for short stretches
// and back (QuickDraw does it around every access to the frame buffer); a
// 32-bit System does the reverse for 24-bit code. A trap injected inside such
// a stretch runs the Memory Manager under the wrong translation: in the 24-bit
// case its master pointers carry flags in the top byte and stop being
// addresses. MMU32Bit ($0CB2) is the mode the MMU is in now; MMFlags ($1EFC)
// bit 0 is the mode the System started in. The IIfx has the same guard, and
// the measurement behind it.
bool MacHostServices::addressingModeSwapped() {
    const bool now32 = read8(0x0CB2) != 0;
    const bool system32 = (read8(0x1EFC) & 1) != 0;
    return now32 != system32;
}

// For the injections that cannot be put off to a later frame: let the guest
// run until it has swapped back.
bool MacHostServices::leaveSwappedAddressingMode() {
    for (int guard = 0; guard < 4000000 && addressingModeSwapped() && !hcpu_.halted;
         ++guard) {
        stepInstruction();
    }
    return !addressingModeSwapped();
}

// Have the guest restart itself: ShutDwnStart, the call behind an installer's
// Restart button and the Finder's Restart. The three words go into
// ApplScratch and the CPU is pointed at them; nothing is put back, because
// the call does not return. A test entry: a restart the guest starts is a
// different road from the reset line, and it has to be reachable without a
// mouse.
bool MacHostServices::guestRestart() {
    if (inDriver_ || announceInFlight_) return false;
    if (read32(0x0358) == 0 || !runToCall()) return false;   // no System / no call
    if (read8(0x0360) & 1) return false;               // FSBusy
    if ((hcpu_.getSR() & 0x0700) != 0) return false;   // mid-interrupt
    if (addressingModeSwapped()) return false;
    const u32 scratch = 0x0A78;
    write16(scratch, 0x3F3C);                         // MOVE.W #2,-(SP)
    write16(scratch + 2, 0x0002);                     //   sdRestart
    write16(scratch + 4, 0xA895);                     // _ShutDown
    hcpu_.pc = scratch;
    return true;
}

bool MacHostServices::guestPutAwaySecondDisk() {
    if (hd2_.empty() || inDriver_ || announceInFlight_) return false;
    if (read32(0x0358) == 0 || !volumeMountedFor(kHd2Drive)) return false;
    if (!runToCall()) return false;
    const bool seen = hd2SeenMounted_;
    unmountSecondDiskAtCall();
    const bool off = !volumeMountedFor(kHd2Drive);
    hd2SeenMounted_ = seen;                           // as if the guest had done it
    return off;
}

bool MacHostServices::guestPowerOff() {
    if (inDriver_ || announceInFlight_) return false;
    if (read32(0x0358) == 0 || !runToCall()) return false;   // no System / no call
    if (read8(0x0360) & 1) return false;               // FSBusy
    if ((hcpu_.getSR() & 0x0700) != 0) return false;   // mid-interrupt
    if (addressingModeSwapped()) return false;
    const u32 scratch = 0x0A78;
    write16(scratch, 0x3F3C);                         // MOVE.W #1,-(SP)
    write16(scratch + 2, 0x0001);                     //   sdPowerOff
    write16(scratch + 4, 0xA895);                     // _ShutDown
    hcpu_.pc = scratch;
    return true;
}

// The sound is played when the guest next makes a call it can be run at, in
// the course of an ordinary frame: run to that call from here, the samples
// of the wait would crowd the sound itself out of the machine's audio buffer.
bool MacHostServices::systemBeep() {
    if (inDriver_ || announceInFlight_ || !guestRunning()) return false;
    if (read32(0x0358) == 0 || (read8(0x0360) & 1)) return false;   // no System / FSBusy
    beepAsked_ = true;
    askForCall();
    return true;
}

void MacHostServices::beepAtCall() {
    if (inDriver_ || announceInFlight_ || addressingModeSwapped()) return;   // not this time
    beepAsked_ = false;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = hcpu_.d[i]; sa[i] = hcpu_.a[i]; }
    hcpu_.a[7] -= 2;
    write16(hcpu_.a[7], 30);                          // duration, in ticks
    execute68kTrap(0xA9C8);                           // _SysBeep
    for (int i = 0; i < 8; ++i) { hcpu_.d[i] = sd[i]; hcpu_.a[i] = sa[i]; }
}

void MacHostServices::execute68kTrap(u16 trap) {
    // ApplScratch, the twelve low-memory bytes reserved for applications. The
    // Classic put this word at the top of RAM, which works on a machine with
    // no MMU -- but here the '040 runs the System's own address space, and
    // fetching from the far top of physical RAM lands somewhere the guest has
    // not mapped: the CPU faults, the guard loop spins on the fault, and the
    // machine ends up executing address 10. Low memory is identity-mapped on
    // every Macintosh, because the vector table and the globals live there.
    //
    // ⛔ And PUT BACK what was there. Those twelve bytes are reserved for the
    // running APPLICATION -- the System does not touch them, so an application
    // is entitled to leave something in them across a Toolbox call. Writing the
    // trap word here and walking away corrupts the first two bytes of whatever
    // the Finder was keeping, permanently, on every injected trap: the disk
    // announce does it up to fifteen times a second while the desktop is up,
    // and the eject does it again. The Finder reads its own scratch back, gets
    // half a trap word where a pointer used to be, follows it, and the machine
    // bus-errors seconds later somewhere that has nothing to do with disks
    // (measured: A3 = A6A07A0E at 0002BBB6, every front-end eject).
    const u32 scratch = 0x0A78;
    const u8 was0 = read8(scratch), was1 = read8(scratch + 1);
    write8(scratch, static_cast<u8>(trap >> 8));
    write8(scratch + 1, static_cast<u8>(trap & 0xFF));
    const u32 savedPc = hcpu_.pc;
    const u16 savedSr = hcpu_.getSR();
    hcpu_.pc = scratch;
    // The trap makes calls of its own, and those are not the guest's: the
    // CPU is not to stop at one. A request that was standing, or one a clock
    // tick makes while the trap runs, is put to the CPU again afterwards.
    const bool wasAsked = hcpu_.holdAtCalls != 0;
    hcpu_.holdAtCalls = 0;
    const bool outer = !inGuestCall_;
    inGuestCall_ = true;
    for (int guard = 0; guard < 64000000 && hcpu_.pc != scratch + 2 && !hcpu_.halted;
         ++guard) {
        stepInstruction();
    }
    // A call that never came back is given up, and said: the guest was not
    // in a state to take it, and what the caller reads next is not a result.
    if (hcpu_.pc != scratch + 2 && onDiag) {
        char b[96];
        std::snprintf(b, sizeof b, "guest call %04X did not return (given up at pc %08X)", trap,
                      hcpu_.pc);
        onDiag(b);
    }
    hcpu_.pc = savedPc;
    hcpu_.setSR(savedSr);
    write8(scratch, was0);
    write8(scratch + 1, was1);
    if (outer) {
        inGuestCall_ = false;
        if (wasAsked || askedInCall_) {
            askedInCall_ = false;
            askForCall();
        }
    }
}

s32 MacHostServices::gestaltQuery(u32 selector, u32& response) {
    runToCall();                                  // where the guest stands, failing that
    if (!leaveSwappedAddressingMode()) { response = 0; return -1; }
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = hcpu_.d[i]; sa[i] = hcpu_.a[i]; }
    hcpu_.d[0] = selector;
    execute68kTrap(0xA1AD);                       // _Gestalt
    const s32 err = static_cast<s16>(hcpu_.d[0] & 0xFFFF);
    response = hcpu_.a[0];
    for (int i = 0; i < 8; ++i) { hcpu_.d[i] = sd[i]; hcpu_.a[i] = sa[i]; }
    return err;
}

bool MacHostServices::slotResources(std::vector<SlotResource>& out) {
    out.clear();
    runToCall();                                  // where the guest stands, failing that
    if (!leaveSwappedAddressingMode()) return false;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = hcpu_.d[i]; sa[i] = hcpu_.a[i]; }
    // The Slot Manager's parameter block (56 bytes), on the guest's stack:
    // spCategory +40, spCType +42, spDrvrSW +44, spDrvrHW +46, spSlot +49,
    // spID +50, spExtDev +51. SNextSRsrc (selector $14) starts after the
    // resource named by slot, ID and external device and fills in the next.
    hcpu_.a[7] -= 64;
    const u32 pb = hcpu_.a[7];
    for (u32 i = 0; i < 64; ++i) write8(pb + i, 0);
    for (int guard = 0; guard < 64; ++guard) {
        hcpu_.a[0] = pb;
        hcpu_.d[0] = 0x14;
        execute68kTrap(0xA06E);                   // _SlotManager
        if (static_cast<s16>(hcpu_.d[0] & 0xFFFF) != 0) break;
        out.push_back(SlotResource{read8(pb + 49), read8(pb + 50), read16(pb + 40), read16(pb + 42),
                                   read16(pb + 44), read16(pb + 46)});
    }
    for (int i = 0; i < 8; ++i) { hcpu_.d[i] = sd[i]; hcpu_.a[i] = sa[i]; }
    return true;
}

// A drive that has sat in the queue since the ROM scan never gets mounted:
// its disk-inserted announcement fell in the window startup flushes. Posting
// the event once the System is up makes the insertion the same event a user's
// insertion is -- the System mounts the volume and the Finder shows it.
void MacHostServices::announceHardDisks() {
    if (suppressHdAnnounce) return;
    if (!lowMemoryFresh()) return;                // still the last startup's
    if (announceInFlight_) return;
    if (!hdAnnouncePending_ && !hd2AnnouncePending_) return;
    if (read32(0x0358) == 0) return;              // no live VCB: System not up
    if (read8(0x0360) & 1) return;                // FSBusy: a file op is running
    if ((hcpu_.getSR() & 0x0700) != 0) return;     // mid-interrupt: not now
    if (addressingModeSwapped()) return;              // not now
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
    if (volumeMountedFor(kHdDrive)) hdAnnouncePending_ = false;
    if (volumeMountedFor(kHd2Drive)) hd2AnnouncePending_ = false;
    if (!hdAnnouncePending_ && !hd2AnnouncePending_) return;
    // The File Manager's own queue must be initialised, or _MountVol enqueues
    // into a header the System has not built yet (the Classic address-errored
    // on exactly this).
    if ((((static_cast<u32>(read16(0x0360)) << 16) | read16(0x0362)) == 0xFFFFFFFFu))
        return;
    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = hcpu_.d[i]; sa[i] = hcpu_.a[i]; }
    // A param block from the System heap, not a corner of RAM we picked: the
    // trap keeps it only for the call, but the heap is the one place nothing
    // else claims. Allocated once, reused for every retry.
    if (!hdMountPb_) {
        hcpu_.d[0] = 80;
        execute68kTrap(0xA71E);                   // _NewPtr,Sys,Clear
        // Strip only what is not already a RAM address: above 16 MB the
        // high byte is significant, below it it may carry flag bits.
        hdMountPb_ = guestPtr(hcpu_.a[0]);
    }
    // Mount the volume -- and post NO event. The applications that must see
    // the disk (the Finder, the 7.5 Installer) enumerate the VCB queue, so
    // the mount alone puts the volume in front of them. Posting a diskEvt as
    // well poisons the 7.5 startup: with the event in the queue, the System's
    // event processing re-runs a drive-1 mount mid-startup, the re-mount
    // comes back volOnLinErr, and the recovery FLUSHES AND EJECTS the boot
    // floppy and asks for it back by name (measured -- the please-insert
    // screen at frame ~1700 appears with the event and not without it).
    auto mountDrive = [&](u16 drive) -> bool {
        if (!hdMountPb_) return false;
        for (u32 i = 0; i < 80; ++i) write8(hdMountPb_ + i, 0);
        write16(hdMountPb_ + 22, drive);          // ioVRefNum = drive number
        hcpu_.a[0] = hdMountPb_;
        execute68kTrap(0xA00F);                   // _MountVol
        const s16 res = static_cast<s16>(hcpu_.d[0] & 0xFFFF);
        if (onDiag) {
            char b[72];
            std::snprintf(b, sizeof b, "hd: drive %u mount result %d", drive, res);
            onDiag(b);
        }
        // 0 = mounted, -55 = volOnLinErr = already on line. Either way, done.
        return res == 0 || res == -55;
    };
    ++hdMountTries_;
    if (hdAnnouncePending_ && mountDrive(kHdDrive)) hdAnnouncePending_ = false;
    if (hd2AnnouncePending_ && mountDrive(kHd2Drive)) hd2AnnouncePending_ = false;
    if (hdMountTries_ >= 15) { hdAnnouncePending_ = hd2AnnouncePending_ = false; }
    for (int i = 0; i < 8; ++i) { hcpu_.d[i] = sd[i]; hcpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
}

// Closing the host application is, to the guest, having its power cut. The
// System marks a volume "in use" on its disk the moment it mounts it for
// writing and only clears that mark on unmount, so a machine that is simply
// switched off leaves every volume flagged unclean, and blocks the System is
// still holding in its cache never reach the image at all.
//
// The flag itself is not a problem -- this ROM rebuilds an unclean volume and
// mounts it, the way a real Macintosh does (see hfs::shrinkExtentsTree for the
// one thing that used to stop it). The lost cache blocks are.
//
// So do what Special > Shut Down does, on the way out: flush each mounted
// volume and unmount it. The guest's own File Manager writes its cached
// blocks through our driver -- which we serve synchronously, so everything
// has landed in the image by the time this returns -- and sets the clean
// mark itself. Nothing is faked: the flag is set by the code that owns it.
// Record in the image that a volume is consistent on disk. drAtrb bit 8 says
// "this volume was unmounted cleanly", which is the File Manager's way of
// promising that nothing it was holding is missing from the medium. After a
// _FlushVol that returned noErr, and with the machine about to be destroyed,
// that promise is simply true -- the flush is what makes it true, and this
// only writes it down. The startup volume cannot be unmounted (its System
// file is open), so there is no other way for the guest to state it.
//
// Both copies of the MDB are updated, as a real unmount does: block 2 and the
// alternate in the second-to-last block of the volume.
// Returns true only when a flag actually had to be changed, so a caller can
// tell "this volume was left mounted" from "it was already clean".
bool MacHostServices::markVolumeCleanIn(std::vector<u8>& backing, u32 base,
                                      u32 volumeSize) {
    if (backing.empty() || volumeSize < 2048) return false;
    bool changed = false;
    // Primary MDB at volume block 2, alternate 1024 bytes from the end.
    for (u32 off : {1024u, volumeSize - 1024u}) {
        const u32 at = base + off;
        if (at + 12 > backing.size()) continue;
        if (backing[at] != 0x42 || backing[at + 1] != 0x44) continue;   // 'BD'
        if (backing[at + 0x0A] & 0x01) continue;                        // clean
        backing[at + 0x0A] = static_cast<u8>(backing[at + 0x0A] | 0x01);
        changed = true;
    }
    return changed;
}

bool MacHostServices::markVolumeClean(u16 drive) {
    const int unit = drive == kHdDrive ? 0 : 1;
    if (unit ? hd2RO_ : hdRO_) return false;
    const std::vector<u8>& img = unit ? hd2_ : hd_;
    if (img.empty()) return false;
    return markVolumeCleanIn(unit ? scsiImage2_ : scsiImage_,
                             unit ? hfsImageOffset2_ : hfsImageOffset_,
                             static_cast<u32>(img.size()));
}

bool MacHostServices::shutdownVolumes() {
    if (hd_.empty() && hd2_.empty()) return false;
    if (inDriver_ || announceInFlight_ || !guestRunning()) return false;
    if (!fileSystemUp()) return false;                // no System, or no FS queue yet
    // At a call of the guest's own, like every other trap run for the host.
    // A guest that makes none -- stopped on a system error, say -- still gets
    // its volumes flushed, where it stands: the machine is about to go, and
    // blocks left in its cache are lost for certain.
    runToCall();
    if (!fileSystemUp()) return false;                // it went down meanwhile
    if (!leaveSwappedAddressingMode()) return false;
    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = hcpu_.d[i]; sa[i] = hcpu_.a[i]; }
    if (!hdMountPb_) {
        hcpu_.d[0] = 80;
        execute68kTrap(0xA71E);                       // _NewPtr,Sys,Clear
        hdMountPb_ = guestPtr(hcpu_.a[0]);
    }
    bool any = false;
    if (hdMountPb_) {
        for (u16 drive : {kHdDrive, kHd2Drive}) {
            if (!volumeMountedFor(drive)) continue;
            auto call = [&](u16 trap) -> s16 {
                for (u32 i = 0; i < 80; ++i) write8(hdMountPb_ + i, 0);
                write16(hdMountPb_ + 22, drive);      // ioVRefNum
                hcpu_.a[0] = hdMountPb_;
                execute68kTrap(trap);
                return static_cast<s16>(hcpu_.d[0] & 0xFFFF);
            };
            // Flush first: the File Manager writes every cached block of this
            // volume through our driver, which we serve synchronously, so
            // when this returns the image on the host holds everything the
            // guest ever wrote.
            const s16 fl = call(0xA013);              // _FlushVol
            // Then try the real unmount. It succeeds for a data volume, and
            // fails -47 (fBsyErr) for the startup volume because the System
            // file is open on it -- a real Shut Down cannot unmount that one
            // either.
            const s16 un = call(0xA00E);              // _UnmountVol
            if (drive == kHd2Drive) hd2SeenMounted_ = false;   // our doing
            bool marked = false;
            if (fl == 0 && un != 0) marked = markVolumeClean(drive);
            any = any || un == 0 || marked;
            if (onDiag) {
                char b[112];
                std::snprintf(b, sizeof b,
                              "hd: drive %u flush %d, unmount %d%s", drive, fl,
                              un, marked ? ", marked cleanly unmounted" : "");
                onDiag(b);
            }
        }
    }
    for (int i = 0; i < 8; ++i) { hcpu_.d[i] = sd[i]; hcpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
    return any;
}

// The first half of a drop-box republish: get the guest off the second volume
// so the host may rebuild it. Only the second seat -- the boot volume is not ours to
// disturb, and unlike shutdownVolumes this is not a shutdown: the machine goes
// on running with one fewer disk for the moment it takes to build the next one.
//
// The flush is the part that matters. The File Manager writes every cached
// block of the volume out through our driver, which we serve synchronously, so
// by the time this returns anything the guest wrote is in the image the caller
// is about to read. Unmounting then makes the guest forget its cached catalog
// and bitmap -- which is exactly why a rebuilt volume may be put in its place.
// Editing the image under a mounted volume instead would leave the guest
// reading its old catalog against the new disk's blocks.
//
// Returns true when the volume is off line afterwards, which includes it never
// having been mounted -- the caller's next step is the same either way.
bool MacHostServices::unmountSecondDisk() {
    if (hd2_.empty()) return true;
    if (inDriver_ || announceInFlight_) return false;
    if (!fileSystemUp()) return true;                 // no System, or no FS queue yet
    if (!volumeMountedFor(kHd2Drive)) return true;    // already off line
    if (!guestRunning()) return true;                 // nothing is reading it
    hd2UnmountAsked_ = 30;
    askForCall();
    return false;
}

// The unmount itself, with the CPU in front of one of the guest's calls.
// True = the traps were run (or there was nothing to unmount); false = the
// File Manager was busy, and the request stands.
bool MacHostServices::unmountSecondDiskAtCall() {
    if (hd2_.empty() || !fileSystemUp()) return true;
    if (inDriver_ || announceInFlight_) return false;
    if (read8(0x0360) & 1) return false;              // FSBusy: a file op is running
    if ((hcpu_.getSR() & 0x0700) != 0) return false;   // mid-interrupt: not now
    if (addressingModeSwapped()) return false;        // not now
    if (!volumeMountedFor(kHd2Drive)) return true;            // already off line
    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = hcpu_.d[i]; sa[i] = hcpu_.a[i]; }
    if (!hdMountPb_) {
        hcpu_.d[0] = 80;
        execute68kTrap(0xA71E);                       // _NewPtr,Sys,Clear
        hdMountPb_ = guestPtr(hcpu_.a[0]);
    }
    s16 fl = 0, un = 0;
    if (hdMountPb_) {
        auto call = [&](u16 trap) -> s16 {
            for (u32 i = 0; i < 80; ++i) write8(hdMountPb_ + i, 0);
            write16(hdMountPb_ + 22, kHd2Drive);      // ioVRefNum = drive number
            hcpu_.a[0] = hdMountPb_;
            execute68kTrap(trap);
            return static_cast<s16>(hcpu_.d[0] & 0xFFFF);
        };
        fl = call(0xA013);                            // _FlushVol
        un = call(0xA00E);                            // _UnmountVol
    }
    for (int i = 0; i < 8; ++i) { hcpu_.d[i] = sd[i]; hcpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
    const bool off = !volumeMountedFor(kHd2Drive);
    // This unmount was asked for from outside; it is not the Macintosh
    // putting the disk away.
    if (off) { hd2SeenMounted_ = false; hd2OffSeconds_ = 0; }
    if (onDiag) {
        char b[112];
        std::snprintf(b, sizeof b, "hd2: flush %d, unmount %d -- volume %s", fl, un,
                      off ? "off line" : "STILL MOUNTED");
        onDiag(b);
    }
    return true;
}

// Read a file the way an application does: the guest's File Manager walks the
// catalog, resolves the extents and calls the driver, and we serve the driver.
// Everything an archiver depends on is therefore in the loop -- which is the
// point. Our own reader can say the bytes are on the volume; only this can say
// the guest is able to fetch them.
bool MacHostServices::readFileThroughGuest(const std::string& name,
                                         std::vector<u8>& out, std::string& why) {
    out.clear();
    why.clear();
    if (hd2_.empty()) { why = "no volume on the second seat"; return false; }
    if (inDriver_ || announceInFlight_) { why = "busy"; return false; }
    if (read32(0x0358) == 0) { why = "the System is not up"; return false; }
    if ((((static_cast<u32>(read16(0x0360)) << 16) | read16(0x0362)) == 0xFFFFFFFFu)) {
        why = "the File Manager queue is not built";
        return false;
    }
    if (read8(0x0360) & 1) { why = "the File Manager is busy"; return false; }
    if (name.empty() || name.size() > 31) { why = "bad name"; return false; }
    if (!runToCall()) { why = "the guest made no call this could be run at"; return false; }
    if (read8(0x0360) & 1) { why = "the File Manager is busy"; return false; }
    if (!leaveSwappedAddressingMode()) { why = "busy"; return false; }

    constexpr u32 kChunk = 32768;
    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = hcpu_.d[i]; sa[i] = hcpu_.a[i]; }

    auto newPtr = [&](u32 bytes) -> u32 {
        hcpu_.d[0] = bytes;
        execute68kTrap(0xA71E);                       // _NewPtr,Sys,Clear
        return guestPtr(hcpu_.a[0]);
    };
    const u32 pb = newPtr(80);
    const u32 nameBuf = newPtr(64);
    const u32 dataBuf = newPtr(kChunk);
    bool ok = false;
    if (!pb || !nameBuf || !dataBuf) {
        why = "the guest heap would not give us working space";
    } else {
        // A Pascal string: length byte, then the characters.
        write8(nameBuf, static_cast<u8>(name.size()));
        for (std::size_t i = 0; i < name.size(); ++i)
            write8(nameBuf + 1 + static_cast<u32>(i), static_cast<u8>(name[i]));

        auto call = [&](u16 trap) -> s16 {
            hcpu_.a[0] = pb;
            execute68kTrap(trap);
            return static_cast<s16>(hcpu_.d[0] & 0xFFFF);
        };
        auto clearPb = [&]() { for (u32 i = 0; i < 80; ++i) write8(pb + i, 0); };

        clearPb();
        write32(pb + 18, nameBuf);                    // ioNamePtr
        write16(pb + 22, kHd2Drive);                  // ioVRefNum = drive number
        write8(pb + 27, 1);                           // ioPermssn = fsRdPerm
        const s16 openErr = call(0xA000);             // _Open
        if (openErr != 0) {
            char b[96];
            std::snprintf(b, sizeof b, "the guest could not open it (_Open %d)", openErr);
            why = b;
        } else {
            const u16 refNum = read16(pb + 24);
            s16 readErr = 0;
            for (;;) {
                clearPb();
                write16(pb + 24, refNum);             // ioRefNum
                write32(pb + 32, dataBuf);            // ioBuffer
                write32(pb + 36, kChunk);             // ioReqCount
                write16(pb + 44, 0);                  // ioPosMode = fsAtMark
                readErr = call(0xA002);               // _Read
                const u32 got = read32(pb + 40);      // ioActCount
                for (u32 i = 0; i < got; ++i) out.push_back(read8(dataBuf + i));
                // -39 eofErr with a short count is the normal end of a file.
                if (readErr != 0 || got == 0) break;
            }
            clearPb();
            write16(pb + 24, refNum);
            call(0xA001);                             // _Close
            if (readErr != 0 && readErr != -39) {
                char b[96];
                std::snprintf(b, sizeof b, "the guest's read failed (_Read %d) after %zu bytes",
                              readErr, out.size());
                why = b;
            } else {
                ok = true;
            }
        }
    }
    for (u32 p : {pb, nameBuf, dataBuf}) {
        if (!p) continue;
        hcpu_.a[0] = p;
        execute68kTrap(0xA01F);                       // _DisposPtr
    }
    for (int i = 0; i < 8; ++i) { hcpu_.d[i] = sd[i]; hcpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
    return ok;
}

void MacHostServices::insertHardDisk(std::vector<u8> image, bool readOnly) {
    hd_ = std::move(image);
    hdRO_ = readOnly;
    if (hd_.empty()) {
        disk_->detach();
        scsiImage_.clear();
        return;
    }
    scsiImage_ = scsi::buildAppleScsiDisk(hd_, scsi::buildScsiDriverPortable(0, kHdDrive, 32));
    hfsImageOffset_ = static_cast<u32>(scsiImage_.size() - hd_.size());
    disk_->attach(&scsiImage_, 0);
    disk_->readOnly = readOnly;
    // A volume left flagged "in use" is one whose machine stopped without
    // unmounting it -- a crash, a kill, the host losing power. That is not a
    // damaged volume and a real Macintosh mounts it: this ROM rebuilds it first
    // (recounting the catalog and remaking the bitmap), then puts up System
    // 7.5's "may not have been shut down properly" notice.
    //
    // What it cannot do is rebuild a volume whose extents-overflow B*-tree is
    // bigger than its own budget arithmetic can measure -- see
    // hfs::shrinkExtentsTree. Volumes this formatter made before 2026-08-03 are
    // all over that line, which is why one bad exit used to leave a disk that
    // would never boot again. Put them back inside it, once, in place, giving
    // back only nodes the tree is not using.
    if (!readOnly && !suppressVolumeRepair) {
        std::string why;
        const bool changed =
            hfs::shrinkExtentsTree(scsiImage_, hfsImageOffset_,
                                   static_cast<u32>(hd_.size()),
                                   hfs::kRepairableExtentsBytes, why);
        if (onDiag && !why.empty())
            onDiag((std::string("hd: ") + why).c_str());
        (void)changed;
    }
    hdAnnouncePending_ = true;
    hdAnnounceDelay_ = 0;
    hdMountTries_ = 0;
}

const std::vector<u8>& MacHostServices::hardDiskImage() const {
    if (!scsiImage_.empty() &&
        static_cast<std::size_t>(hfsImageOffset_) + hd_.size() <= scsiImage_.size())
        hd_.assign(scsiImage_.begin() + hfsImageOffset_,
                   scsiImage_.begin() + hfsImageOffset_ + hd_.size());
    return hd_;
}

void MacHostServices::insertHardDisk2(std::vector<u8> image, bool readOnly) {
    hd2_ = std::move(image);
    hd2RO_ = readOnly;
    hd2SeenMounted_ = hd2PutAway_ = false;
    hd2OffSeconds_ = 0;
    if (hd2_.empty()) {
        disk2_->detach();
        scsiImage2_.clear();
        return;
    }
    scsiImage2_ = scsi::buildAppleScsiDisk(hd2_, scsi::buildScsiDriverPortable(1, kHd2Drive, 33));
    hfsImageOffset2_ = static_cast<u32>(scsiImage2_.size() - hd2_.size());
    disk2_->attach(&scsiImage2_, 1);
    disk2_->readOnly = readOnly;
    // The same repair the boot disk gets: a volume put on this seat may be an
    // old image of the user's too, and it will be mounted by the same ROM.
    if (!readOnly && !suppressVolumeRepair) {
        std::string why;
        hfs::shrinkExtentsTree(scsiImage2_, hfsImageOffset2_,
                               static_cast<u32>(hd2_.size()),
                               hfs::kRepairableExtentsBytes, why);
        if (onDiag && !why.empty())
            onDiag((std::string("hd2: ") + why).c_str());
    }
    hd2AnnouncePending_ = true;
    hdAnnounceDelay_ = 0;
    // The try counter is a give-up for one disk's announcement, not a budget
    // for the machine's lifetime. A drop box republishes this seat over and
    // over; without this reset the boot's own tries exhaust the cap and every
    // later volume is dropped without a single _MountVol ever being issued.
    hdMountTries_ = 0;
}

void MacHostServices::detachHardDisk2() {
    hd2SeenMounted_ = hd2PutAway_ = false;
    hd2OffSeconds_ = 0;
    hd2_.clear();
    scsiImage2_.clear();
    disk2_->detach();
}

const std::vector<u8>& MacHostServices::hardDisk2Image() const {
    if (!scsiImage2_.empty() &&
        static_cast<std::size_t>(hfsImageOffset2_) + hd2_.size() <= scsiImage2_.size())
        hd2_.assign(scsiImage2_.begin() + hfsImageOffset2_,
                    scsiImage2_.begin() + hfsImageOffset2_ + hd2_.size());
    return hd2_;
}

void MacHostServices::attachCdRom(bool attached, int busId) {
    cdrom_->setAttached(attached, busId);
    if (onDiag) {
        char b[64];
        std::snprintf(b, sizeof b, "cd: drive %s (SCSI ID %d)",
                      attached ? "attached" : "detached", busId & 7);
        onDiag(b);
    }
}
bool MacHostServices::cdRomAttached() const { return cdrom_->attachedState(); }
// The Classic has peeled CD containers since its drive was built; this seat
// never did, so a raw-sector .bin went in with its 2352-byte framing still on
// it, and a file the drive cannot serve was accepted in silence. Same code and
// same words on both machines now.
int MacHostServices::insertCd(std::vector<u8> image) {
    cd::Medium m = cd::normalize(std::move(image));
    if (onDiag) {
        char b[272];                    // "cd: refused: " + the 240-byte desc
        std::snprintf(b, sizeof b, "cd: %s%s", m.ok ? "inserted -- " : "refused: ",
                      m.desc);
        onDiag(b);
    }
    if (!m.ok) return 0;
    cdrom_->insert(std::move(m.data));
    // Where the volume starts on this disc, and the drive's own view of it.
    cdHfsOffset_ = findHfsPartition(cdrom_->media());
    if (onDiag && cdHfsOffset_) {
        char b[96];
        std::snprintf(b, sizeof b, "cd: Apple_HFS partition at byte %u",
                      cdHfsOffset_);
        onDiag(b);
    }
    if (cdStatus_) write8(cdStatus_ + 3, 1);          // dsDiskInPlace
    cdMountPending_ = true;
    cdMountTries_ = 0;
    return 1;
}
// The front end's Eject only STAGES: the disc has a volume mounted on it, and
// taking the medium out from under one leaves the guest holding a volume with
// nothing behind it -- offLinErr forever, which is exactly what the floppy path
// had to learn. completeCdEject unmounts first, on the emulation thread, when
// the guest next makes a call it can be run at.
void MacHostServices::ejectCd() {
    if (!cdrom_->discPresent()) return;
    cdEjectPending_ = true;
}

// Does the disc carry a volume the guest has to let go of first?
bool MacHostServices::cdEjectNeedsGuest() {
    // Whichever driver has the disc: this machine's, or the guest's own.
    const u16 drive = cdDrive_ ? cdDrive_ : guestCdDrive();
    return drive && guestRunning() && read32(0x0358) != 0 && volumeMountedFor(drive);
}

// With the CPU in front of one of the guest's calls: unmount the disc's
// volume, then take the disc.
void MacHostServices::completeCdEject() {
    if (!cdEjectPending_) return;
    if (inDriver_ || announceInFlight_) return;
    const u16 drive = cdDrive_ ? cdDrive_ : guestCdDrive();
    if (drive && read32(0x0358) != 0 && volumeMountedFor(drive)) {
        if ((read8(0x0360) & 1) || (hcpu_.getSR() & 0x0700) != 0 || addressingModeSwapped())
            return;                                   // not now: next second
        announceInFlight_ = true;
        u32 sd[8], sa[8];
        for (int i = 0; i < 8; ++i) { sd[i] = hcpu_.d[i]; sa[i] = hcpu_.a[i]; }
        if (!hdMountPb_) {
            hcpu_.d[0] = 80;
            execute68kTrap(0xA71E);                   // _NewPtr,Sys,Clear
            hdMountPb_ = guestPtr(hcpu_.a[0]);
        }
        s16 un = -1;
        if (hdMountPb_) {
            for (u32 i = 0; i < 80; ++i) write8(hdMountPb_ + i, 0);
            write16(hdMountPb_ + 22, drive);          // ioVRefNum
            hcpu_.a[0] = hdMountPb_;
            execute68kTrap(0xA00E);                   // _UnmountVol
            un = static_cast<s16>(hcpu_.d[0] & 0xFFFF);
        }
        for (int i = 0; i < 8; ++i) { hcpu_.d[i] = sd[i]; hcpu_.a[i] = sa[i]; }
        announceInFlight_ = false;
        if (un != 0) {
            // The guest is still using it (an open file, an application running
            // off the disc). A real drive refuses too; try again next tick.
            if (onDiag && cdEjectTries_ == 0) {
                char b[80];
                std::snprintf(b, sizeof b, "cd: the guest is still using the disc (%d)", un);
                onDiag(b);
            }
            if (++cdEjectTries_ < 20) return;
        }
    }
    finishCdEject();
}

void MacHostServices::finishCdEject() {
    cdEjectPending_ = false;
    cdEjectTries_ = 0;
    cdrom_->eject();
    cdHfsOffset_ = 0;
    cdMountPending_ = false;
    if (cdStatus_) write8(cdStatus_ + 3, 0);          // dsDiskInPlace
    if (onDiag) onDiag("cd: disc taken out by the host");
}
bool MacHostServices::cdPresent() const { return cdrom_->discPresent(); }

// ---- the board's three entry points -------------------------------------------

MacHostServices::MacHostServices(IBus040& bus, M68040& cpu, std::vector<u8>& ram)
    : hbus_(bus), hcpu_(cpu), hram_(ram),
      disk_(std::make_unique<ScsiDisk>()),
      disk2_(std::make_unique<ScsiDisk>()),
      cdrom_(std::make_unique<ScsiCdRom>()) {}

MacHostServices::~MacHostServices() = default;

// The CPU is about to execute at `pc`: if that is the Prime, Control or Status
// entry of a driver this code serves, serve the request and return true (the
// board then charges the time of a short driver call and executes nothing).
bool MacHostServices::serveStorageHooks(u64 frame) {
    if ((diskPrimePc_[0] && hcpu_.pc == diskPrimePc_[0]) ||
        (diskPrimePc_[1] && hcpu_.pc == diskPrimePc_[1])) {
        const int unit = (diskPrimePc_[0] && hcpu_.pc == diskPrimePc_[0]) ? 0 : 1;
        inDriver_ = true;
        try { serveDiskPrime(unit); } catch (const BusFault& f) {
            if (onDiag) {
                char b[112];
                std::snprintf(b, sizeof b,
                              "hd prime FAULT pb=%08X dce=%08X addr=%08X (f %u)",
                              hcpu_.a[0], hcpu_.a[1], f.addr,
                              static_cast<unsigned>(frame));
                onDiag(b);
            }
            hcpu_.d[0] = static_cast<u32>(-36);
            hcpu_.pc = read32(0x08FC);
            hcpu_.a[0] = hcpu_.pc;
        }
        inDriver_ = false;
        return true;
    }
    if (cdPrimePc_ && hcpu_.pc == cdPrimePc_) {
        inDriver_ = true;
        try { serveCdPrime(); } catch (const BusFault& f) {
            if (onDiag) {
                char b[112];
                std::snprintf(b, sizeof b, "cd prime FAULT pb=%08X addr=%08X (f %u)",
                              hcpu_.a[0], f.addr, static_cast<unsigned>(frame));
                onDiag(b);
            }
            hcpu_.d[0] = static_cast<u32>(-36);
            hcpu_.pc = read32(0x08FC);
            hcpu_.a[0] = hcpu_.pc;
        }
        inDriver_ = false;
        return true;
    }
    if ((cdCtlPc_ && hcpu_.pc == cdCtlPc_) || (cdStatusPc_ && hcpu_.pc == cdStatusPc_)) {
        const bool isStatus = cdStatusPc_ && hcpu_.pc == cdStatusPc_;
        inDriver_ = true;
        try { serveCdCtlStatus(isStatus); } catch (const BusFault&) {
            hcpu_.d[0] = static_cast<u32>(-36);
            hcpu_.pc = read32(0x08FC);
            hcpu_.a[0] = hcpu_.pc;
        }
        inDriver_ = false;
        return true;
    }
    for (int u = 0; u < 2; ++u) {
        if ((diskCtlPc_[u] && hcpu_.pc == diskCtlPc_[u]) ||
            (diskStatusPc_[u] && hcpu_.pc == diskStatusPc_[u])) {
            const bool isStatus = diskStatusPc_[u] && hcpu_.pc == diskStatusPc_[u];
            inDriver_ = true;
            try { serveDiskCtlStatus(u, isStatus); } catch (const BusFault& f) {
                if (onDiag) {
                    char b[112];
                    std::snprintf(b, sizeof b,
                                  "hd ctl/status FAULT pb=%08X addr=%08X (f %u)",
                                  hcpu_.a[0], f.addr,
                                  static_cast<unsigned>(frame));
                    onDiag(b);
                }
                hcpu_.d[0] = static_cast<u32>(-36);
                hcpu_.pc = read32(0x08FC);
                hcpu_.a[0] = hcpu_.pc;
            }
            inDriver_ = false;
            return true;
        }
    }
    return false;
}

// Has the Macintosh put the second disk's volume away? The Finder does it
// with _UnmountVol when the icon is dragged to the Trash; the drive stays in
// the queue, so the only trace is the volume leaving the VCB queue while the
// System runs on. A restart and a shut down unmount it as well, and are told
// apart by what follows: the machine starts again (machineStarting clears the
// watch) or the last volume goes too and the queue is empty. Three seconds
// with the System up and the volume off line is the put-away.
//
// Nothing is read before this startup's scan has loaded the seat's driver:
// until then the queue in low memory may still be the last startup's.
void MacHostServices::watchSecondDiskPutAway() {
    if (hd2_.empty() || hd2PutAway_) return;
    if (!diskPrimePc_[1] || !lowMemoryFresh() || inDriver_ || announceInFlight_) {
        hd2OffSeconds_ = 0;
        return;
    }
    const bool queueBuilt =
        ((static_cast<u32>(read16(0x0360)) << 16) | read16(0x0362)) != 0xFFFFFFFFu;
    if (!queueBuilt || read32(0x0358) == 0) {         // no volume at all
        hd2OffSeconds_ = 0;
        return;
    }
    if (volumeMountedFor(kHd2Drive)) {
        hd2SeenMounted_ = true;
        hd2OffSeconds_ = 0;
        return;
    }
    if (!hd2SeenMounted_) return;
    if (++hd2OffSeconds_ < 3) return;
    hd2PutAway_ = true;
    if (onDiag) onDiag("hd2: the Macintosh put the volume away");
}

bool MacHostServices::fileSystemUp() {
    return read32(0x0358) != 0 &&                     // a volume is on line
           ((static_cast<u32>(read16(0x0360)) << 16) | read16(0x0362)) != 0xFFFFFFFFu;
}

u8 MacHostServices::callClasses() {
    return startupFinished() ? M68040::kEventCalls
                             : static_cast<u8>(M68040::kEventCalls | M68040::kResourceCalls);
}

void MacHostServices::askForCall() {
    if (inGuestCall_) askedInCall_ = true;
    else hcpu_.holdAtCalls = callClasses();
}

// Is there work for the guest's next call? Only what could run there: a
// request that would give up at its first check asks for nothing, so a
// machine with nothing to do is never stopped.
bool MacHostServices::storageWorkWanted() {
    if (!guestRunning() || read32(0x0358) == 0) return false;
    if (!suppressHdAnnounce && (hdAnnouncePending_ || hd2AnnouncePending_) && lowMemoryFresh())
        return true;
    if (cdEjectPending_ || (cdMountPending_ && cdDrive_)) return true;
    return cdrom_->attachedState() && !cdDrvr_ && cdInstallTries_ <= 30 &&
           startupFinished() && !guestCdDrive();
}

// The CPU has stopped in front of one of the guest's calls. This second's
// storage work is run if it is still owed, then the board's, then an unmount
// the front end asked for.
void MacHostServices::storageAtCall() {
    atCall_ = true;
    if (storageDue_) {
        storageDue_ = false;
        announceHardDisks();
        // The CD's driver goes in on the same clock and for the same reason:
        // there is no unit table to install into until the System has built
        // one. Eject before mount, so a disc swapped at the host is unmounted
        // before its replacement is announced.
        if (cdrom_->attachedState() && !cdDrvr_) installCdDriver();
        if (cdEjectPending_) completeCdEject();
        else if (cdMountPending_) mountCdVolume();
    }
    if (boardWorkWanted()) boardWorkAtCall();
    if (hd2UnmountAsked_ > 0 && unmountSecondDiskAtCall()) hd2UnmountAsked_ = 0;
    if (beepAsked_) beepAtCall();
}

// An idle System 7.5 Finder makes its event call once a second; three
// seconds of a 33 MHz machine is some thirty million instructions.
bool MacHostServices::runToCall() {
    if (!guestRunning() || hcpu_.halted || inGuestCall_) return false;
    atCall_ = false;
    hcpu_.holdAtCalls = callClasses();
    for (int guard = 0; guard < 30000000 && !atCall_ && !hcpu_.halted && guestRunning();
         ++guard) {
        stepInstruction();
    }
    return atCall_;
}

// Once per emulated second.
void MacHostServices::storageSecondTick() {
    watchSecondDiskPutAway();
    // Watched every second, whoever asks later: a name left in CurApName by
    // the last startup only counts once it has been seen unset.
    (void)startupFinished();
    // A second's work still owed means that second brought no call.
    const bool noCall = storageDue_;
    // A disc with no volume on line comes out at once. One the guest holds
    // waits for its next call; after twenty seconds without one, or twenty
    // refusals, it comes out anyway.
    if (cdEjectPending_ && (!cdEjectNeedsGuest() || (noCall && ++cdEjectTries_ >= 20)))
        finishCdEject();
    storageDue_ = storageWorkWanted();
    if (storageDue_) askForCall();
}

// Once per frame.
void MacHostServices::storageFrameTick() {
    // Claim the disk driver's Prime as soon as the ROM's boot scan has
    // installed it -- BEFORE the System's startup mount runs. Hooked
    // late, that mount goes through the SCSI Manager's discard drain,
    // fails, and the volume is written off for the rest of the session.
    if (lowMemoryFresh() &&
        ((!hd_.empty() && !diskPrimePc_[0]) || (!hd2_.empty() && !diskPrimePc_[1])))
        findDiskDriverPrime();
    // What does not keep to the second: an unmount the front end is asking
    // for, and the board's own work.
    if (hd2UnmountAsked_ > 0) --hd2UnmountAsked_;
    if (hd2UnmountAsked_ > 0 || beepAsked_ || boardWorkWanted()) askForCall();
}

} // namespace openmac
