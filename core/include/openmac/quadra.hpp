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
    u32 adbMouseBytesRead() const; // mouse bytes the guest actually clocked in
    std::vector<u8> adbMouseBytesLog() const;
    void adbClearCmdTrace();
    std::vector<u8> adbCmdTrace() const;   // CPU-issued ADB commands since the clear
    void armAdbSrTrace(int n) { adbSrTraceBudget_ = n; }   // log VIA1 SR reads w/ PCs
    void watchMem(u32 addr, u32 len, int budget) {         // log RAM writes w/ PCs
        watchAddr_ = addr; watchLen_ = len; watchBudget_ = budget;
    }
    void countPc(u32 pc) { countPc_ = pc; countPcHits_ = 0; }
    u32 countPcHits() const { return countPcHits_; }
    bool dafbVblEnabled() const;
    u32 dafbSwatchReg(int i) const;

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

    // Diagnostics.
    struct ScsiDiag {
        u32 writes, selects, commands;
        u8 lastCdb[12];
        int lastCdbLen;
    };
    ScsiDiag scsiDiag() const;
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

    // I/O window dispatch ($50000000 block, mirrors folded).
    u8   ioRead8(u32 off);
    void ioWrite8(u32 off, u8 v);
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

    // SCC functional stub (Classic-style: pointer + a live RR0/RR1).
    int sccPtr_ = 0;
    u8 sccRegs_[16]{};

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

    std::unique_ptr<SonyDrive> fd_;
    std::vector<u8> floppy_;
    bool floppySel_ = false;      // VIA1 PA5 = the drive's SEL line
    u8 swimPhasePrev_ = 0;        // LSTRB edge detection

    std::unique_ptr<Via6522> via1_;
    std::unique_ptr<PseudoVia> via2_;
    std::unique_ptr<Rtc> rtc_;
    std::unique_ptr<AdbTransceiver> adb_;
    std::unique_ptr<Dafb> dafb_;
    std::unique_ptr<Easc> easc_;
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
    int primeDiagBudget_ = 48;  // trace the first .Sony Prime requests we serve
    int adbSrTraceBudget_ = 0;  // armed by the input test: log SR reads with PCs
    u32 watchAddr_ = 0, watchLen_ = 0;   // RAM write-watch window
    int watchBudget_ = 0;
    u32 countPc_ = 0, countPcHits_ = 0;  // execution counter for one address
    bool dafbIrq_ = false;               // internal video interrupt line (VIA2 PA6)
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
