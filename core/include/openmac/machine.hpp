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
    bool overlayActive() const { return overlay_; }

    u32 screenBase() const;
    u32 soundBase() const;
    // Expand the 1-bit framebuffer to ARGB8888 (kScreenW * kScreenH).
    void renderScreen(u32* argbOut) const;

    // Floppy: mount a raw sector image (400K/800K/1.44MB). We service it
    // through a replacement .Sony disk driver rather than emulating the IWM,
    // so any image the ROM's HFS can read will boot. Empty image = no disk.
    void insertFloppy(std::vector<u8> image, bool readOnly = false);
    void ejectFloppy();
    bool floppyInserted() const { return !floppy_.empty(); }

    // Hard disk: a second, fixed (non-removable) image mounted through the same
    // high-level .Sony interception as a hard-disk volume. Empty = no hard disk.
    // Persist writes by reading hardDiskImage() back out after running.
    void insertHardDisk(std::vector<u8> image, bool readOnly = false);
    bool hardDiskPresent() const { return !hd_.empty(); }
    const std::vector<u8>& hardDiskImage() const { return hd_; }
    u32 hdAccessCount() const { return hdReads_ + hdWrites_; }
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

    // Unmapped/stub access log (instrument first): capped, newest last.
    const std::vector<std::string>& accessLog() const { return accessLog_; }
    void clearAccessLog() { accessLog_.clear(); }

    // IBus (the CPU's view of the machine)
    u8   read8(u32 addr) override;
    u16  read16(u32 addr) override;
    void write8(u32 addr, u8 value) override;
    void write16(u32 addr, u16 value) override;

private:
    void wireVia();
    void logAccess(const char* what, u32 addr, bool write, u32 value);
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
    int sonyOpen(u32 pb, u32 dce);
    int sonyPrime(u32 pb, u32 dce);
    int sonyControl(u32 pb, u32 dce);
    int sonyStatus(u32 pb, u32 dce);
    // Run a Mac A-line trap synchronously from within a driver handler.
    void execute68kTrap(u16 trap);

    // Minimal IWM: track the phase/mode lines the ROM pokes and answer its
    // disk-sense reads so the startup probe finds the drive and disk. The
    // actual sector I/O is handled by the .Sony interception, not here.
    u8 iwmAccess(int reg, bool write, u8 data);
    u8 iwmReadReg();
    u8 iwmStatus();
    u8 iwmLines_ = 0;   // b0-3 ca0-3, b4 motor, b5 drivesel, b6 q6, b7 q7
    u8 iwmMode_ = 0;    // Mode register, read back in Status bits 0-4

    std::vector<u8> floppy_;
    bool floppyRO_ = false;
    u32 sonyOpenPc_ = 0, sonyPrimePc_ = 0, sonyControlPc_ = 0, sonyStatusPc_ = 0;
    u32 drvStatusAddr_ = 0;        // Mac address of our DrvSts record
    int floppyDriveNum_ = 0;
    bool inSony_ = false;          // re-entrancy guard during trap execution

    std::vector<u8> hd_;           // hard-disk image (empty = none)
    bool hdRO_ = false;
    u32 hdStatusAddr_ = 0;         // DrvSts record for the hard disk
    int hdDriveNum_ = 0;
    u32 hdReads_ = 0, hdWrites_ = 0;   // block-I/O counters (feasibility probe)
    u32 hdMountPb_ = 0;                // system-heap param block for _MountVol
    u32 diskEvtPosts_ = 0;             // mount attempts
    u32 diskEvtResult_ = 0xFFFFFFFFu;  // last _MountVol OSErr
    bool hdMounted_ = false;           // volume mounted OK; stop retrying
    // Enabled by sonyOpen once a hard disk is configured; the mount trigger then
    // runs _MountVol with the .Sony intercept consulted (see execute68kTrap).
    bool hdAutoMount_ = false;
    bool floppyInsertPending_ = false; // a floppy was swapped in after boot

    std::vector<u8> ram_;
    std::vector<u8> rom_;
    u32 ramMask_ = 0;
    u32 romMask_ = 0;
    bool overlay_ = true;
    bool screenAlt_ = false;

    std::unique_ptr<Via6522> via_;
    std::unique_ptr<Rtc> rtc_;
    std::unique_ptr<AdbTransceiver> adb_;
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

    u64 totalCycles_ = 0;
    u64 frameCounter_ = 0;
    bool forceRomDisk_ = false;
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
