#pragma once

// Macintosh IIfx (1990): MC68030/MC68882 at 40 MHz, FMC memory controller,
// OSS interrupt controller, two I/O Processors and six NuBus slots.

#include "openmac/bus040.hpp"
#include "openmac/cpu030.hpp"
#include "openmac/hardware_trace.hpp"
#include "openmac/types.hpp"

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace openmac {

class Via6522;
class Rtc;
class Ncr5380;
class IifxScsiDma;
class ScsiDisk;
class Scc8530;
class Asc;
class Oss;
class IifxIop;
class IifxAdbBus;
class AdbTransceiver;
class IifxNuBusVideo;
class Iwm;
class SonyDrive;
class IifxStateCodec;

class IifxMachine final : public IBus040 {
public:
    struct Config {
        u32 ramSize = 8u * 1024u * 1024u;
        bool videoCard = true;
        // Exercise the motherboard's physical NCR 5380 DMA and SWIM/ISM IOP
        // paths. Kept configurable for synthetic component tests which do not
        // download the proprietary IOP firmware; production frontends enable it.
        bool nativeStorage = false;
        // Optional external declaration ROM for an Apple Macintosh Display
        // Card 8*24 GC. Empty selects OpenMac's documented synthetic video
        // card; firmware is caller-owned and is not distributed by OpenMac.
        std::vector<u8> videoDeclarationRom;
    };

    IifxMachine(std::vector<u8> rom, const Config& config);
    explicit IifxMachine(std::vector<u8> rom);
    ~IifxMachine() override;

    void reset();
    int stepInstruction();
    void runFrame();

    M68030& cpu() { return cpu_; }
    const M68030& cpu() const { return cpu_; }
    u64 totalCycles() const { return totalCycles_; }
    u64 frameCount() const { return frameCounter_; }
    bool overlayActive() const { return overlay_; }
    bool poweredOff() const;

    void insertHardDisk(std::vector<u8> image, bool readOnly = false);
    bool hardDiskPresent() const { return !hardDisk_.empty(); }
    const std::vector<u8>& hardDiskImage() const;
    // Flush and unmount drive 4 through the guest File Manager before the
    // host persists or replaces its image.
    bool shutdownHardDisk();

    // Internal FDHD/SuperDrive. Raw 400K/800K/1.44 MB media and DiskCopy 4.2
    // or MacBinary wrappers are accepted. The wrappers are retained so sector
    // writes go back to the same host-file format on eject/exit.
    int insertFloppy(std::vector<u8> image, bool readOnly = false);
    void ejectFloppy();
    bool floppyPresent() const { return !floppy_.empty(); }
    const std::vector<u8>& floppyImage() const { return floppy_; }
    std::vector<u8> floppyForWriteBack();

    // Front-end devices. Audio is unsigned 8-bit PCM, mono, at the ASC clock
    // selected by the guest (normally 22.254 kHz; 44.1 kHz is also supported).
    // Input enters through the IIfx ISM IOP's ADB channel rather than reaching
    // the guest directly.
    void drainAudio(std::vector<u8>& out);
    u32 audioSampleRate() const;
    void mouseMove(int dx, int dy, bool button);
    void keyEvent(u8 adbCode, bool down);

    // The same opaque battery-backed blob used by the Classic and Quadra:
    // 256 XPRAM bytes plus the RTC seconds counter. Passing out=null returns
    // the required size; addSeconds advances the clock across host downtime.
    u32 savePram(u8* out, u32 cap) const;
    bool loadPram(const u8* data, u32 len, u32 addSeconds = 0);

    int screenWidth() const;
    int screenHeight() const;
    void renderScreen(u32* argbOut) const;

    std::string diagnosticReport() const;
    int diagnosticScsiPhase() const;
    u32 diagnosticScsiCommandCount() const;
    u32 diagnosticScsiTransferPosition() const;
    u32 diagnosticScsiTransferLength() const;
    bool diagnosticScsiIrq() const;
    u32 diagnosticScsiDmaControl() const;
    u32 diagnosticScsiDmaCount() const;
    u32 diagnosticScsiDmaAddress() const;
    bool diagnosticScsiDmaActive() const;
    bool diagnosticScsiDrq() const;
    bool diagnosticIsmFirmwareAlive() const;
    bool diagnosticFloppyServiceReady() const { return floppyServiceReady_; }
    bool diagnosticFloppyMounted();
    u64 diagnosticMediaEventId() const { return activeMediaEventId_; }
    u32 diagnosticGcProcessorPc() const;
    u64 diagnosticGcProcessorInstructions() const;
    bool diagnosticGcProcessorRecentlyExecuted(u32 pc) const;
    void diagnosticSetGcProcessorPcWatch(u32 pc);
    u64 diagnosticGcProcessorPcWatchHits() const;
    // Sample-free execution histogram over one GC PC range; pairs of
    // (count, pc) sorted most-executed first.
    void diagnosticSetGcProcessorProfileRange(u32 first, u32 last);
    std::vector<std::pair<u64, u32>> diagnosticGcProcessorProfile(
        std::size_t limit) const;
    // Record every data access issued from GC PCs inside [first, last].
    void diagnosticSetGcPcTapRange(u32 first, u32 last);
    // Record every GC data access touching addresses inside [first, last].
    void diagnosticSetGcAddrTapRange(u32 first, u32 last,
                                     bool writesOnly = false);
    // Point the card-side bus-master watch at one Macintosh RAM long.
    void diagnosticSetGcDoorbellWatch(u32 address);
    // Record card-side writes covering one physical VRAM byte offset.
    void diagnosticSetGcVramWatch(u32 offset);
    // Watch one PHYSICAL Am29000 register (0-255) for writes.
    void diagnosticSetGcRegisterWatch(u16 physicalIndex);
    // Snapshot one ARCHITECTURAL register (0-127 gr, 128-255 lr through the
    // live window) plus gr1 each time the given GC pc executes.
    void diagnosticSetGcPcSnap(u32 pc, u16 architecturalIndex);
    // Record pc/gr1/bounds + one PHYSICAL register for every GC instruction
    // in [start, start+count); write the capture as text lines to a file.
    void diagnosticSetGcFlightWindow(u64 start, u64 count, u16 physicalIndex);
    bool diagnosticWriteGcFlight(const char* path) const;
    // Read the GC card's 2 MiB data DRAM without changing emulated state.
    // The low 64 KiB is shared with the Macintosh IPC aperture.
    std::vector<u8> diagnosticGcDataMemory(u32 offset, u32 size) const;
    // Read the GC card's expansion DRAM without changing emulated state.
    // Addresses are offsets within the card's 8 MiB expansion bank.
    std::vector<u8> diagnosticGcExpansionMemory(u32 offset, u32 size) const;
    // Read the GC's physically separate 64 KiB instruction SRAM.
    std::vector<u8> diagnosticGcInstructionSram(u32 offset, u32 size) const;
    const std::vector<std::string>& accessLog() const { return accessLog_; }
    void clearAccessLog() { accessLog_.clear(); }
    void setLegacyAccessLogEnabled(bool enabled) {
        legacyAccessLogEnabled_ = enabled;
        if (!enabled) accessLog_.clear();
    }
    // Boot-trace aid: capture the shared SRAM after the motherboard ROM has
    // downloaded either IOP firmware image.  This exposes no Apple data unless
    // the caller explicitly supplies and runs its own ROM.
    std::vector<u8> iopRamImage(bool ism) const;

    // Deterministic hardware trace. The core records typed bus and protocol
    // events into a bounded pre/post-trigger ring; front ends only configure
    // and export it, so tracing cannot perturb emulated device reads.
    void configureHardwareTrace(const HardwareTraceConfig& config);
    HardwareTrace& hardwareTrace() { return hardwareTrace_; }
    const HardwareTrace& hardwareTrace() const { return hardwareTrace_; }
    void triggerHardwareTrace(const std::string& reason);
    void recordHardwareMilestone(const std::string& name);
    bool writeHardwareTraceJsonl(const std::string& path) const;
    // If set, the next trace trigger schedules a full-machine checkpoint at
    // the following completed 68030 instruction boundary. The caller drains
    // it explicitly, keeping filesystem policy out of the emulation core.
    void setTraceCheckpointOnTrigger(bool enabled) {
        traceCheckpointOnTrigger_ = enabled;
    }
    bool traceCheckpointPending() const { return traceCheckpointPending_; }
    void scheduleTraceCheckpoint(const std::string& reason) {
        if (!traceCheckpointOnTrigger_ || traceCheckpointPending_) return;
        traceCheckpointPending_ = true;
        traceCheckpointReason_ = reason;
    }
    // Diagnostic replay aid: correlation IDs describe events within one
    // capture and are intentionally restartable without changing hardware.
    void resetMediaCorrelation() {
        mediaEventId_ = 0;
        activeMediaEventId_ = 0;
    }
    bool takeTraceCheckpoint(std::vector<u8>& state,
                             std::string* reason = nullptr);
    u64 deterministicStateHash() const;
    u64 framebufferHash() const;

    // Versioned, checksummed full-machine checkpoints. The ROM is identified
    // by content rather than copied into every checkpoint; all mutable state,
    // including RAM, controller pipelines, IOP firmware RAM, VRAM and mounted
    // writable media, is self-contained in the returned image.
    std::vector<u8> saveState() const;
    bool loadState(const u8* data, std::size_t size,
                   std::string* error = nullptr);
    bool saveStateFile(const std::string& path,
                       std::string* error = nullptr) const;
    bool loadStateFile(const std::string& path,
                       std::string* error = nullptr);

    // Bring-up hooks. Diagnostics never alter machine state.
    std::function<void(const char*)> onDiag;
    std::function<void(u32 pc)> onStep;

    u8 read8(u32 addr) override;
    u16 read16(u32 addr) override;
    u32 read32(u32 addr) override;
    void write8(u32 addr, u8 value) override;
    void write16(u32 addr, u16 value) override;
    void write32(u32 addr, u32 value) override;
    u8 read8CacheInhibited(u32 addr) override;
    u16 read16CacheInhibited(u32 addr) override;
    u32 read32CacheInhibited(u32 addr) override;
    bool cacheable(u32 addr) const override;
    bool readBurst32(u32 firstAddr, u32 out[4]) override;
    int takeCyclePenalty() override;

private:
    friend class IifxStateCodec;

    void wireDevices();
    void tickDevices(int cpuCycles);
    void updateSccDmaRequests();
    void synchronizeViaAccess();
    void updateIpl();
    u8 ioRead8(u32 addr);
    void ioWrite8(u32 addr, u8 value);
    void logAccess(const char* device, bool write, u32 addr, u32 value,
                   u8 width = 1);
    void serveSonyPrime();
    void serveSonyCtlStatus(bool status);
    void findDiskDriverPrime();
    void serveDiskPrime();
    void serveDiskCtlStatus(bool status);
    bool execute68kTrap(u16 trap);
    void announceHardDisk();
    bool volumeMountedFor(u16 drive);
    bool markHardDiskClean();
    u32 guestPtr(u32 address) const;
    static bool floppyGeometry(std::vector<u8>& image);
    std::vector<u8> floppyFileImage(const std::vector<u8>& sectors) const;
    u8 rawRead8(u32 addr);
    u8 rawRead8Traced(u32 addr);
    u32 rawRead32(u32 addr);
    void rawWrite8(u32 addr, u8 value);
    void rawWrite8Traced(u32 addr, u8 value);
    void clearFmcCache();
    bool fmcLookup(u32 addr, u32& word) const;
    void fmcMergeWrite(u32 addr, u8 value);
    u8 swimAccess(u8 offset, bool write, u8 value = 0);
    void updateFloppyTrack();
    void serviceSwimSurface();
    void flushFloppyTrack();
    HardwareTraceEvent makeTraceEvent(HardwareTraceKind kind,
                                      HardwareTraceSource source,
                                      u32 categories) const;
    void traceAccess(HardwareTraceSource source, bool write, u32 address,
                     u64 value, u8 width, const char* detail = nullptr);
    void traceState(HardwareTraceKind kind, HardwareTraceSource source,
                    u32 categories, const char* detail = nullptr);
    void checkTraceStall(u32 pc);
    void checkScsiProtocol(u8 reg, u8 value);
    void checkHardwareProtocols();
    void protocolAssertion(u64 identity, HardwareTraceSource source,
                           u32 categories, const char* reason,
                           u32 address = 0, u64 value = 0);
    void triggerTrace(const char* reason);

    std::vector<u8> ram_;
    std::vector<u8> rom_;
    u32 romMask_ = 0;
    bool overlay_ = true;

    std::unique_ptr<Via6522> via_;
    std::unique_ptr<Rtc> rtc_;
    std::unique_ptr<Oss> oss_;
    std::unique_ptr<IifxIop> sccIop_;
    std::unique_ptr<IifxIop> ismIop_;
    std::unique_ptr<Scc8530> scc_;
    std::unique_ptr<Ncr5380> scsi_;
    std::unique_ptr<IifxScsiDma> scsiDma_;
    std::unique_ptr<ScsiDisk> disk_;
    std::unique_ptr<Asc> sound_;
    std::unique_ptr<AdbTransceiver> adb_;
    std::unique_ptr<IifxAdbBus> adbBus_;
    std::unique_ptr<Iwm> swim_;
    std::unique_ptr<SonyDrive> floppyDrive_;
    std::unique_ptr<IifxNuBusVideo> video_;
    M68030 cpu_;

    mutable std::vector<u8> hardDisk_;
    std::vector<u8> scsiImage_;
    u32 hfsImageOffset_ = 0;
    u32 hfsVolumeBytes_ = 0;
    bool hardDiskIsWholeScsi_ = false;
    bool inDiskDriver_ = false;
    u32 diskPrimePc_ = 0;
    u32 diskCtlPc_ = 0;
    u32 diskStatusPc_ = 0;
    u32 hardDiskReadCount_ = 0;
    u32 hardDiskWriteCount_ = 0;
    int hardDiskTraceBudget_ = 48;
    int hardDiskErrorBudget_ = 20;
    bool hardDiskMountPending_ = false;
    bool hardDiskMounted_ = false;
    bool hardDiskMountInFlight_ = false;
    u32 hardDiskMountPb_ = 0;
    u32 hardDiskMountTries_ = 0;
    u32 hardDiskMountDelay_ = 0;
    s16 hardDiskMountResult_ = static_cast<s16>(-32768);
    u32 injectedTrapTimeouts_ = 0;
    u64 totalCycles_ = 0;
    u64 frameCounter_ = 0;
    u64 viaPhase_ = 0;
    u64 framePhase_ = 0;
    u64 secondPhase_ = 0;
    u64 audioPhase_ = 0;
    u32 biuRegs_[16]{};
    u8 ossExpansionRegs_[3][0x20]{};
    bool soundIrq_ = false;
    bool videoEnabled_ = true;
    bool nativeStorage_ = false;

    // IIfx Fast Memory Controller: 32 KiB unified, direct-mapped physical
    // cache, one valid tag per 16-byte/four-longword block.  Unlike the 030's
    // logical I/D caches, this cache fills only on an acknowledged burst.
    struct FmcLine {
        u32 tag = 0xFFFFFFFFu;
        bool valid = false;
        u32 data[4]{};
    };
    static constexpr u32 kFmcLines = 2048;
    std::unique_ptr<FmcLine[]> fmcCache_;
    int fmcCyclePenalty_ = 0;
    u64 fmcHits_ = 0;
    u64 fmcMisses_ = 0;
    u64 fmcFills_ = 0;
    u64 fmcCacheInhibited_ = 0;

    std::vector<u8> floppy_;
    std::vector<u8> floppyEjected_;
    std::vector<u8> fdMbHeader_, fdMbResource_;
    std::vector<u8> fdDcHeader_, fdDcTags_;
    bool floppyReadOnly_ = false;
    bool floppyServiceReady_ = false;
    s64 floppyEventRetryCycles_ = 0;
    bool swimLstrbPrev_ = false;
    bool swimActionPrev_ = false;
    bool swimWriteMarkPrev_ = false;
    bool swimReadMarkPrev_ = false;
    bool swimReadSynced_ = false;
    u16 swimCrcOut_ = 0xFFFF;
    u64 swimDataBytes_ = 0;
    u64 swimDataWrites_ = 0;
    u32 floppyTrackWrites_ = 0;
    int swimDiagBudget_ = 128;
    int sonyPrimeDiagBudget_ = 48;
    // Low-memory cursor/ADB probes are useful at boot, but the guest rewrites
    // them continuously once the Event Manager is live. Bound this at the
    // source so a GUI which drains its queue every frame cannot grow an
    // unbounded multi-megabyte session log.
    int lowMemoryDiagBudget_ = 96;
    u64 mediaEventId_ = 0;
    u64 activeMediaEventId_ = 0;
    u32 traceFloppyBlock_ = 0;
    u8 traceFloppyTrack_ = 0xFF;
    u8 traceFloppySide_ = 0xFF;
    u8 traceFloppySector_ = 0xFF;
    u64 traceIopOperationHits_ = 0;

    std::vector<u8> audioOut_;

    std::vector<std::string> accessLog_;
    bool legacyAccessLogEnabled_ = true;
    HardwareTrace hardwareTrace_;
    int traceBusDepth_ = 0;
    u32 traceLastPc_ = 0;
    u64 traceSamePcCount_ = 0;
    int traceLastScsiPhase_ = -1;
    u32 traceLastScsiCommands_ = 0;
    u8 traceLastSwimMode_ = 0xFF;
    u8 traceLastSwimHandshake_ = 0xFF;
    u8 traceLastSwimFifo_ = 0xFF;
    int traceProtocolScsiPhase_ = -1;
    u64 traceAssertionMask_ = 0;
    u64 traceLastVideoAssertions_ = 0;
    u64 traceLastVideoAcks_ = 0;
    u64 traceLastVideoVblank_ = 0;
    u64 traceVideoIrqFrames_ = 0;
    u64 traceVideoIrqEligibleSince_ = 0;
    u32 traceLastMouseReports_ = 0;
    u16 traceLastMouseV_ = 0;
    u16 traceLastMouseH_ = 0;
    u64 traceMouseMoveFrame_ = 0;
    u64 traceMouseVramWrites_ = 0;
    u32 traceVideoFrameSamples_ = 0;
    u64 traceLastVideoSerialCommands_ = 0;
    u64 traceLastVideoPriorityFrame_ = 0;
    bool traceCheckpointOnTrigger_ = false;
    bool traceCheckpointPending_ = false;
    std::string traceCheckpointReason_;
};

} // namespace openmac
