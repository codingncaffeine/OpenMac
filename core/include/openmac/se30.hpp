#pragma once

// Macintosh SE/30 machine: 68030 and 68882 at 15.6672 MHz, RAM in two banks
// placed by two VIA2 lines, two 6522 VIAs, the clock chip and the ADB
// transceiver on VIA1, an 8530 serial chip, a 5380 SCSI chip, the Apple
// Sound Chip, a SWIM floppy chip, and the built-in 512x342 black-and-white
// screen, which the ROM finds as a card in slot $E through its own 8 KB
// declaration ROM. Unanswered addresses end in a bus error.
//
// Reference: the SE/30 hardware dossier (the ROM's own tables and power-on
// behavior, Guide to the Macintosh Family Hardware 2nd ed., Designing Cards
// and Drivers for the Macintosh Family 3rd ed., Apple Technical Notes HW14
// and HW25, MAME and Linux as behavioral references). Clean-room.
//
// Hard disks, the CD-ROM drive and the network adapter sit on the one SCSI
// bus (IDs 0, 1, 3 and 4); MacHostServices runs them for the front end.

#include "openmac/bus040.hpp"
#include "openmac/cpu030.hpp"
#include "openmac/hostservices.hpp"
#include "openmac/types.hpp"

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace openmac {

class Via6522;
class Rtc;
class AdbTransceiver;
class AdbViaPort;
class Scc8530;
class Ncr5380;
class ScsiEthernet;
class Asc;
class DirectFloppy;
class Se30Video;

class Se30Machine final : public IBus040, public MacHostServices {
public:
    struct Config {
        // The two SIMM banks in megabytes: 0 (empty), 1, 4, 16 or 64. The
        // ROM wants the larger SIMMs in bank A and a single set in bank A.
        u32 bankAMb = 4;
        u32 bankBMb = 4;
        // The 8 KB declaration ROM of the built-in screen. Without one slot
        // $E is empty and the machine starts with no screen.
        std::vector<u8> videoRom;
    };

    // The RAM totals this ROM starts with, in megabytes, and the SIMM sets
    // that make them. 64 MB in bank A alone is in the ROM's own table but
    // fails its address test, on the real machine as here.
    static const std::array<u32, 13>& ramSizesMb();
    static bool banksForTotal(u32 totalMb, u32& bankAMb, u32& bankBMb);

    Se30Machine(std::vector<u8> rom, const Config& cfg);
    ~Se30Machine() override;

    void reset();

    // One frame of CPU time: 370 lines of 704 clocks, 60.15 Hz.
    void runFrame();
    int stepInstruction() override;

    M68040& cpu() { return cpu_; }
    u64 totalCycles() const { return totalCycles_; }
    u64 frameCount() const { return frameCounter_; }
    bool overlayActive() const { return overlay_; }
    u32 ramBytes() const { return static_cast<u32>(ram_.size()); }

    // Keyboard and mouse on the Apple Desktop Bus: a raw ADB key code going
    // down or up; mouse movement in counts since the last call, with the
    // button's state.
    void keyEvent(u8 adbCode, bool down);
    void mouseEvent(int dx, int dy, bool button);

    // The internal SuperDrive: 400K, 800K or 1.44 MB of sectors, bare or in
    // a DiskCopy 4.2 or MacBinary file. False = not a floppy image.
    bool insertFloppy(std::vector<u8> image, bool readOnly = false);
    // The host's Eject. A disk whose volume is on line is not pulled out
    // from under the System (the drive has no way to lose a disk but its own
    // eject motor, and the ROM's driver never looks again): the File Manager
    // is asked to flush and eject it at the guest's next event call, and the
    // mechanism throws it out under a second later. A disk with no volume on
    // line comes out at once, and so does one the Macintosh has not ejected
    // four seconds after being asked. floppyPresent() says when it is out;
    // floppyEjects() counts the mechanism's own ejects.
    void ejectFloppy();
    bool floppyPresent() const;
    u32 floppyEjects() const;
    // The disk's sectors as they are now (the ejected disk's, once it is out).
    const std::vector<u8>& floppySectors();
    // The same in the file format the disk came in: what goes back to the
    // host file.
    std::vector<u8> floppyForWriteBack();
    // Bytes moved between the floppy chip and the disk, and tracks put back
    // into the disk's sectors.
    struct FloppyCounts { u64 bytesRead, bytesWritten; u32 tracksWritten; bool secondRegisterSet; int track; };
    FloppyCounts floppyCounts() const;

    // The SCSI Ethernet adapter, on the same bus as the disks. Frames cross
    // to the host through netInject (host to guest) and netDrain.
    void attachNet(bool attached, int busId = 4);
    bool netAttached() const;
    bool netInject(const u8* frame, u32 len);
    bool netDrain(std::vector<u8>& out);

    // What has crossed the SCSI chip since the machine was built: targets
    // selected, commands carried out (in all and by target ID), data bytes
    // each way, accesses through each of the two data addresses, and
    // handshake cycles the board ended in a bus error because the chip asked
    // for no byte.
    struct ScsiCounts {
        u32 selections, commands, bytesIn, bytesOut;
        std::array<u32, 8> commandsById;
        u64 plainDataCycles, handshakeDataCycles;
        u32 handshakeTimeouts;
    };
    ScsiCounts scsiCounts() const;

    int screenWidth() const;
    int screenHeight() const;
    void renderScreen(u32* argbOut) const;
    void drainAudio(std::vector<u8>& out);

    // ---- parameter RAM and clock --------------------------------------------
    // The same 268-byte blob as the Quadras': 'PRAM', a version, the 256
    // bytes, the clock.
    static constexpr u32 kPramBlobBytes = 268;
    u32 savePram(u8* out, u32 cap) const;
    bool loadPram(const u8* data, u32 len, u32 addSeconds);
    const std::array<u8, 256>& pram() const;

    // What the board's lines say now: the RAM-size bits (VIA2 port A bits
    // 7-6), the address where the bank select line switches to bank B, which
    // screen buffer is shown, and the screen interrupt's two states.
    struct BoardState {
        u8 ramSizeBits;
        u32 bankBStart;
        bool mainScreenBuffer, screenInterruptEnabled, screenInterruptRequested;
        u64 screenInterrupts;   // requests raised since reset
    };
    BoardState boardState() const;

    // A text snapshot of the CPU and the devices, for when the guest stops.
    std::string diagnosticReport() const;

    // IBus040 (the CPU's view of the machine).
    u8   read8(u32 addr) override;
    u16  read16(u32 addr) override;
    u32  read32(u32 addr) override;
    void write8(u32 addr, u8 value) override;
    void write16(u32 addr, u16 value) override;
    void write32(u32 addr, u32 value) override;

    static constexpr u32 kNoRam = 0xFFFFFFFFu;
    // Where a RAM-space address lands in the fitted bytes (bank A first,
    // then bank B), or kNoRam for an empty bank.
    u32 ramIndex(u32 addr) const;

protected:
    bool boardWorkWanted() override;
    void boardWorkAtCall() override;

private:
    void wireDevices();
    void askGuestToEjectFloppy();
    void resetPeripherals();
    void tickDevices(int cpuCycles);
    void updateIpl();
    void updateRamDecode();
    void updateScsiLines();
    bool scsiHandshakeCycle();
    bool deviceRead8(u32 addr, u8& out);
    bool deviceWrite8(u32 addr, u8 value);

    std::vector<u8> ram_;      // the fitted bytes, bank A then bank B
    std::vector<u8> rom_;
    std::vector<u8> videoRom_;
    u32 romMask_ = 0;
    u32 videoRomMask_ = 0;
    u32 romEntry_ = 0;         // the reset vector's program counter
    u32 bankA_ = 0, bankB_ = 0;   // bytes
    u8 sizeBits_ = 3;          // VIA2 port A bits 7-6 as the board sees them
    bool overlay_ = true;
    bool identityRam_ = false; // every fitted byte sits at its own address

    std::unique_ptr<Via6522> via1_;
    std::unique_ptr<Via6522> via2_;
    std::unique_ptr<Rtc> rtc_;
    std::unique_ptr<AdbTransceiver> adb_;
    std::unique_ptr<AdbViaPort> adbPort_;
    std::unique_ptr<Scc8530> scc_;
    std::unique_ptr<Ncr5380> scsi_;
    std::unique_ptr<ScsiEthernet> netdev_;
    std::unique_ptr<Asc> asc_;
    std::unique_ptr<DirectFloppy> floppy_;
    std::unique_ptr<Se30Video> video_;
    M68030 cpu_;

    bool via2Pb7_ = true;      // last level of VIA2 timer 1's output
    u64 screenInterrupts_ = 0;
    u64 totalCycles_ = 0;
    u64 frameCounter_ = 0;
    int viaRemainder_ = 0;
    int lineAcc_ = 0;
    int line_ = 0;
    u64 secondAcc_ = 0;
    int secondPulseLines_ = 0;
    int busWaitClocks_ = 0;    // clocks a bus cycle was held beyond the instruction's own
    u64 scsiPlainCycles_ = 0, scsiHandshakeCycles_ = 0;
    u32 scsiHandshakeTimeouts_ = 0;
    bool floppyEjectPending_ = false;   // the host's Eject, until the disk is out
    bool floppyEjectAsked_ = false;     // the File Manager has been asked
    int floppyEjectFrames_ = 0;         // frames since the host's Eject
    std::vector<u8> audioOut_;
};

} // namespace openmac
