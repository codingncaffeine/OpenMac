#pragma once

// Deterministic, bounded hardware tracing shared by headless tools and front
// ends. Events contain only emulated time/state: no host timestamps, pointers,
// or thread identifiers enter the stream, so two identical runs can be diffed
// byte-for-byte after JSONL export.

#include "openmac/types.hpp"

#include <array>
#include <cstddef>
#include <iosfwd>
#include <string>
#include <vector>

namespace openmac {

enum HardwareTraceCategory : u32 {
    TraceCpu       = 1u << 0,
    TraceBus       = 1u << 1,
    TraceScsi      = 1u << 2,
    TraceSwim      = 1u << 3,
    TraceIop       = 1u << 4,
    TraceMilestone = 1u << 5,
    TraceAssertion = 1u << 6,
    // CPU-visible device-register traffic only.  TraceBus includes RAM and
    // ROM as well; this separate bit keeps long boot captures bounded while
    // preserving every access to VIA/OSS/BIU/IOP/SCSI/SWIM/NuBus hardware.
    TraceIo        = 1u << 7,
    TraceVideo     = 1u << 8,
    TraceAll       = 0x1FFu,
};

enum class HardwareTraceKind : u8 {
    Instruction,
    Access,
    State,
    Mailbox,
    Dma,
    Milestone,
    Assertion,
    Trigger,
};

enum class HardwareTraceSource : u8 {
    Cpu,
    Memory,
    Rom,
    Via,
    Rtc,
    Oss,
    SccIop,
    IsmIop,
    Ncr5380,
    ScsiDma,
    Swim,
    Asc,
    NuBus,
    Biu,
    Other,
};

constexpr u32 hardwareTraceSourceBit(HardwareTraceSource source) {
    return 1u << static_cast<u8>(source);
}

struct HardwareTraceScsiState {
    u8 phase = 0;
    u8 flags = 0; // bit0 REQ, 1 ACK, 2 IRQ, 3 DRQ, 4 phase match, 5 DMA active
    u8 cdbLength = 0;
    u8 reserved = 0;
    u32 transferPosition = 0;
    u32 transferLength = 0;
    u32 dmaControl = 0;
    u32 dmaAddress = 0;
    u32 dmaCount = 0;
    u32 commandCount = 0;
    std::array<u8, 12> cdb{};
};

struct HardwareTraceSwimState {
    u8 mode = 0;
    u8 setup = 0;
    u8 handshake = 0;
    u8 error = 0;
    u8 data = 0;
    u8 lines = 0;
    u8 driveAddress = 0;
    u8 flags = 0; // bit0 ACTION, 1 write, 2 FIFO, 3 mark, 4 CRC-ok,
                  // bit5 disk, 6 switched, 7 motor
    u16 crc = 0;
    u8 track = 0;
    u8 side = 0;
    u8 fifoOccupancy = 0;
    u8 fifoHead = 0;
    u8 fifoTail = 0;
    u8 readSynced = 0;
    u16 fifoHeadCrc = 0;
    // bit0 step-in direction, 1 head busy, 2 motor commanded, 3 drive enabled,
    // bit4 MFM mechanism mode.  stepRemaining is in main-CPU cycles.
    u8 mechanismFlags = 0;
    u64 stepRemaining = 0;
    u32 requestedBlock = 0;
    u8 expectedTrack = 0xFF;
    u8 expectedSide = 0xFF;
    u8 expectedSector = 0xFF;
};

struct HardwareTraceMailboxState {
    u8 length = 0;
    u8 operation = 0;
    u8 channel = 0;
    // bit0 host access, 1 send mailbox, 2 new, 3 complete, 4 block geometry.
    u8 flags = 0;
    s16 result = 0;
    u32 ramAddress = 0;
    u32 block = 0;
    u32 blockCount = 0;
    u8 expectedTrack = 0xFF;
    u8 expectedSide = 0xFF;
    u8 expectedSector = 0xFF;
    std::array<u8, 32> payload{};
};

struct HardwareTraceIopState {
    u16 pc = 0;
    u8 opcode = 0;
    u8 a = 0;
    u8 x = 0;
    u8 y = 0;
    u8 s = 0;
    u8 p = 0;
    u8 status = 0;
    u8 interruptMask = 0;
    u8 interruptFlags = 0;
    u8 reserved = 0;
    std::array<u8, 2> dmaControl{};
    std::array<u8, 2> dmaRequest{};
    std::array<u16, 2> dmaMap{};
    std::array<u16, 2> dmaCount{};
    u64 dmaTransfers = 0;
    // ISM firmware media-poll globals, copied from shared SRAM without
    // executing a peripheral read. These addresses are part of the ROM-
    // downloaded image's private ABI and make native-floppy stalls replayable.
    u8 pollEnable = 0;
    u8 currentDrive = 0;
    u8 driveKind = 0;
    u8 driveState = 0;
    u8 driveFormat = 0;
    u8 receiveState = 0;
    u16 continuation = 0;
    u64 instructions = 0;
};

struct HardwareTraceAscState {
    u8 mode = 0;
    u8 control = 0;
    u8 volume = 0;
    u8 clock = 0;
    u8 status = 0;
    // bit0 stereo, bit1 IRQ asserted
    u8 flags = 0;
    u16 fifoLevelA = 0;
    u16 fifoLevelB = 0;
    std::array<u32, 4> phase{};
    std::array<u32, 4> increment{};
    u64 ramWrites = 0;
    u64 producedSamples = 0;
    u64 nonSilentSamples = 0;
    u64 irqTransitions = 0;
};

struct HardwareTraceVideoState {
    u16 mode = 0;
    u16 control = 0;
    u32 base = 0;
    u32 stride = 0;
    u64 vramWrites = 0;
    u64 frameVramWrites = 0;
    u64 vblankCount = 0;
    u64 vblankAssertions = 0;
    u64 vblankAcks = 0;
    // bit0 genuine 8*24 GC, 1 VBL enabled, 2 slot IRQ asserted,
    // bit3 OSS slot pending. Bits 4..6 contain the programmed OSS priority.
    u8 flags = 0;
    u8 bitsPerPixel = 0;
    u8 ossIpl = 0;
    u8 ramdacMode = 0;
    u8 reserved = 0;
    u16 serialCommand = 0;
    u64 serialCommands = 0;
    u64 timingEchoWrites = 0;
};

struct HardwareTraceEvent {
    u64 sequence = 0;
    u64 cycle = 0;
    u64 value = 0;
    u64 auxiliary = 0;
    // Monotonically increasing identifier for the physical media change that
    // caused this event. Zero means that no floppy media event is in scope.
    u64 correlation = 0;
    u32 pc = 0;
    u32 address = 0;
    u32 categories = 0;
    HardwareTraceKind kind = HardwareTraceKind::State;
    HardwareTraceSource source = HardwareTraceSource::Other;
    u8 width = 0;
    u8 flags = 0; // bit0 write
    HardwareTraceScsiState scsi{};
    HardwareTraceSwimState swim{};
    HardwareTraceIopState iop{};
    HardwareTraceAscState asc{};
    HardwareTraceVideoState video{};
    HardwareTraceMailboxState mailbox{};
    std::array<char, 96> detail{};
};

struct HardwareTraceConfig {
    // Zero disables tracing. Space for post-trigger events is reserved while
    // the pre-trigger ring rolls, so triggering never destroys the lead-in.
    std::size_t capacity = 0;
    std::size_t postTriggerEvents = 0;
    u32 categories = TraceAll;

    // Optional event admission filters. Source filtering applies to ordinary
    // records of every kind; the inclusive address interval applies only to
    // bus-access records. Assertions and trigger markers always survive so a
    // narrow diagnostic capture cannot hide the reason it froze.
    u32 sources = 0xFFFFFFFFu;
    u32 addressFirst = 0;
    u32 addressLast = 0xFFFFFFFFu;

    // Optional deterministic automatic triggers. A zero cycle disables the
    // cycle trigger. PC triggering counts instruction entries at triggerPc.
    u64 triggerCycle = 0;
    u32 triggerPc = 0;
    u64 triggerPcHits = 0;

    // A small independent instruction ring avoids making high-volume IOP
    // execution evict the lower-volume protocol events. It is merged into the
    // chronological pre-trigger stream only when a trace freezes/triggers.
    std::size_t iopFlightEvents = 0;

    // Optional request-boundary trigger for IOP mailbox channel 1. 0xFFFFFFFF
    // disables a selector. The block selector applies to block I/O operations.
    u32 triggerIopOperation = 0xFFFFFFFFu;
    u32 triggerIopBlock = 0xFFFFFFFFu;
    u64 triggerIopHits = 1;
};

class HardwareTrace {
public:
    void configure(const HardwareTraceConfig& config);
    void reset();

    bool enabled() const { return config_.capacity != 0; }
    bool accepts(u32 categories) const {
        return enabled() && !frozen_ && (config_.categories & categories) != 0;
    }
    bool accepts(u32 categories, HardwareTraceSource source) const {
        return accepts(categories) &&
            (config_.sources & hardwareTraceSourceBit(source)) != 0;
    }
    bool acceptsAccess(u32 categories, HardwareTraceSource source,
                       u32 address) const {
        return accepts(categories, source) && address >= config_.addressFirst &&
            address <= config_.addressLast;
    }
    bool acceptsIopFlight() const {
        return enabled() && !frozen_ && config_.iopFlightEvents != 0;
    }
    bool triggered() const { return triggered_; }
    bool frozen() const { return frozen_; }
    const std::string& triggerReason() const { return triggerReason_; }
    const HardwareTraceConfig& config() const { return config_; }
    std::size_t size() const { return events_.size(); }

    void record(HardwareTraceEvent event);
    void recordIopFlight(HardwareTraceEvent event);
    void trigger(u64 cycle, u32 pc, const std::string& reason,
                 u64 correlation = 0);
    void assertion(HardwareTraceEvent event, const std::string& reason);
    void freeze();

    // Returns events in chronological order even while the pre-trigger ring is
    // still rolling. Export has a deterministic schema header followed by one
    // JSON object per event.
    std::vector<HardwareTraceEvent> events() const;
    bool writeJsonl(std::ostream& output) const;

private:
    void appendPreTrigger(HardwareTraceEvent event);
    void appendLinear(HardwareTraceEvent event);
    void mergeIopFlight();
    void normalize();
    std::size_t preTriggerCapacity() const;

    HardwareTraceConfig config_{};
    std::vector<HardwareTraceEvent> events_;
    std::size_t ringHead_ = 0;
    std::vector<HardwareTraceEvent> iopFlight_;
    std::size_t iopFlightHead_ = 0;
    std::size_t postRemaining_ = 0;
    u64 nextSequence_ = 0;
    u64 triggerPcHitCount_ = 0;
    u64 acceptedEvents_ = 0;
    bool triggered_ = false;
    bool frozen_ = false;
    std::string triggerReason_;
};

const char* hardwareTraceKindName(HardwareTraceKind kind);
const char* hardwareTraceSourceName(HardwareTraceSource source);

} // namespace openmac
