#pragma once

// Macintosh Quadra 950 machine: 68040 at 33 MHz, RAM in four banks under the
// memory controller at $5000 E000, VIA1 and a real VIA2, the clock/parameter
// RAM/power microcontroller on VIA1's shift register, two I/O processors
// (serial; floppy and ADB), two 53C96 SCSI chips, the Enhanced Apple Sound
// Chip, DAFB video. Empty NuBus slot space ends in a bus error.
//
// Reference: the Quadra 950 hardware dossier (the ROM's own tables and
// power-on behavior, Designing Cards and Drivers for the Macintosh Family
// 3rd ed., the Quadra 950 Developer Note, MAME and Linux as behavioral
// references). Clean-room.
//
// Under construction: the board, its interrupt wiring, everything the ROM
// needs to pass its self-tests, and the video chip with every depth the
// developer note lists are here. Keyboard, mouse and floppy (behind the
// second I/O processor) and the second SCSI bus's wiring follow.

#include "openmac/bus040.hpp"
#include "openmac/cpu040.hpp"
#include "openmac/hostservices.hpp"
#include "openmac/types.hpp"

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace openmac {

class Via6522;
class EgretController;
class MemoryController950;
class IoProcessor;
class Scc8530;
class Easc;
class Ncr53c96;
class Dafb;
class AdbTransceiver;
class AdbBus;

class Quadra950Machine final : public IBus040, public MacHostServices {
public:
    struct Config {
        // The four SIMM banks A-D in megabytes: 0 (empty) or 4, 8, 16, 32, 64.
        std::array<u32, 4> bankMb{4, 4, 0, 0};
        // Video RAM in megabytes: 1 (as shipped) or 2 (the sockets filled).
        u32 vramMb = 2;
    };

    Quadra950Machine(std::vector<u8> rom, const Config& cfg);
    explicit Quadra950Machine(std::vector<u8> rom);
    ~Quadra950Machine() override;

    void reset();

    // One 60.15 Hz frame of CPU time, sliced as on the Quadra 650.
    void runFrame();
    int stepInstruction() override;

    M68040& cpu() { return cpu_; }
    u64 totalCycles() const { return totalCycles_; }
    u64 frameCount() const { return frameCounter_; }
    bool overlayActive() const { return overlay_; }
    u32 ramBytes() const { return static_cast<u32>(ram_.size()); }

    // Which display is on the video port, by the names QuadraMachine::displays
    // lists (the 950 drives the same Apple monitors). False = no such name.
    // Takes effect at the next start: the ROM reads the sense lines once.
    bool setDisplay(const char* name);
    void setMonitorSense(u32 grounded, u32 pairs);

    // Keyboard and mouse on the Apple Desktop Bus: a raw ADB key code going
    // down or up; mouse movement in counts since the last call, with the
    // button's state.
    void keyEvent(u8 adbCode, bool down);
    void mouseEvent(int dx, int dy, bool button);

    int screenWidth() const;
    int screenHeight() const;
    void renderScreen(u32* argbOut) const;
    // What the video chip is programmed to: bits per pixel on the pixel bus
    // (32-bit pixels count as 32), the raster of its timing generator, video
    // RAM fitted, and vertical scans a second in thousandths (0 = no raster).
    struct VideoState { int bitsPerPixel, timingWidth, timingHeight; u32 vramBytes, pixelClockHz, scansMilliHz; };
    VideoState videoState() const;
    void drainAudio(std::vector<u8>& out);

    // ---- parameter RAM and clock --------------------------------------------
    // The same 268-byte blob as the Quadra 650's: 'PRAM', a version, the 256
    // bytes, the clock. Here they live in the microcontroller.
    static constexpr u32 kPramBlobBytes = 268;
    u32 savePram(u8* out, u32 cap) const;
    bool loadPram(const u8* data, u32 len, u32 addSeconds);
    const std::array<u8, 256>& pram() const;

    // The guest asked the microcontroller to switch the machine off / restart
    // it. Cleared by the caller.
    bool powerDownRequested = false;

    // What the memory controller's three boundaries currently say, in bytes.
    u32 bankBoundary(int k) const;

    // A text snapshot of the CPU and the devices, for when the guest stops.
    std::string diagnosticReport() const;

    // IBus040 (the CPU's view of the machine).
    u8   read8(u32 addr) override;
    u16  read16(u32 addr) override;
    u32  read32(u32 addr) override;
    void write8(u32 addr, u8 value) override;
    void write16(u32 addr, u16 value) override;
    void write32(u32 addr, u32 value) override;

private:
    void wireDevices();
    void resetPeripherals();
    void tickDevices(int cpuCycles);
    void updateIpl();
    u8   ioRead8(u32 off, u32 addr);
    void ioWrite8(u32 off, u8 v, u32 addr);
    u8   ramRead8(u32 addr) const;
    void ramWrite8(u32 addr, u8 v);

    std::vector<u8> ram_;      // the fitted bytes, banks A-D back to back
    std::vector<u8> rom_;
    u32 romMask_ = 0;
    u32 romEntry_ = 0;         // the reset vector's program counter
    bool overlay_ = true;

    std::unique_ptr<MemoryController950> memctl_;
    std::unique_ptr<Via6522> via1_;
    std::unique_ptr<Via6522> via2_;
    std::unique_ptr<EgretController> egret_;
    std::unique_ptr<IoProcessor> sccIop_;
    std::unique_ptr<IoProcessor> swimIop_;
    std::unique_ptr<Scc8530> scc_;
    std::unique_ptr<Easc> easc_;
    std::unique_ptr<Ncr53c96> scsi0_;    // internal bus
    std::unique_ptr<Ncr53c96> scsi1_;    // external bus
    std::unique_ptr<AdbTransceiver> adb_;
    std::unique_ptr<AdbBus> adbBus_;
    std::unique_ptr<Dafb> dafb_;
    M68040 cpu_;

    u16 sonicRegs_[0x40]{};
    u8 macProm_[8]{};
    u32 nubusCtlRegs_[8]{};

    bool dafbIrq_ = false;
    bool scsiIrq_[2] = {false, false};
    bool via2Pb7_ = true;      // last level of VIA2 timer 1's output
    bool resetPending_ = false;   // the microcontroller asked for a system reset
    // True when [addr, addr+n) lies in fitted RAM that sits at its own address.
    bool directRam(u32 addr, u32 n) const {
        return addr < ram_.size() && ram_.size() - addr >= n;
    }

    u64 totalCycles_ = 0;
    u64 frameCounter_ = 0;
    int viaRemainder_ = 0;
    u64 secondAcc_ = 0;
    int audioAcc_ = 0;
    int sliceInFrame_ = 0;
    u32 vblAcc_ = 0;
    std::vector<u8> audioOut_;
};

} // namespace openmac
