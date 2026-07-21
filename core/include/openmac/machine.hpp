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
    // Expand the 1-bit framebuffer to ARGB8888 (kScreenW * kScreenH).
    void renderScreen(u32* argbOut) const;

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
};

} // namespace openmac
