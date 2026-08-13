#include "openmac/hardware_trace.hpp"

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <ostream>
#include <sstream>

namespace openmac {

namespace {

void jsonString(std::ostream& output, const char* text) {
    output << '"';
    for (const unsigned char ch : std::string(text ? text : "")) {
        switch (ch) {
        case '"': output << "\\\""; break;
        case '\\': output << "\\\\"; break;
        case '\b': output << "\\b"; break;
        case '\f': output << "\\f"; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default:
            if (ch < 0x20) {
                const auto flags = output.flags();
                const char fill = output.fill();
                output << "\\u" << std::hex << std::setw(4)
                       << std::setfill('0') << static_cast<unsigned>(ch);
                output.flags(flags);
                output.fill(fill);
            } else {
                output << static_cast<char>(ch);
            }
            break;
        }
    }
    output << '"';
}

template <std::size_t N>
void jsonByteArray(std::ostream& output, const std::array<u8, N>& bytes,
                   unsigned length) {
    output << '[';
    const unsigned count = std::min<unsigned>(length,
        static_cast<unsigned>(bytes.size()));
    for (unsigned index = 0; index < count; ++index) {
        if (index) output << ',';
        output << static_cast<unsigned>(bytes[index]);
    }
    output << ']';
}

void setDetail(std::array<char, 96>& destination, const std::string& text) {
    const std::size_t count = std::min(text.size(), destination.size() - 1u);
    std::copy_n(text.data(), count, destination.data());
    destination[count] = '\0';
}

} // namespace

const char* hardwareTraceKindName(HardwareTraceKind kind) {
    switch (kind) {
    case HardwareTraceKind::Instruction: return "instruction";
    case HardwareTraceKind::Access: return "access";
    case HardwareTraceKind::State: return "state";
    case HardwareTraceKind::Mailbox: return "mailbox";
    case HardwareTraceKind::Dma: return "dma";
    case HardwareTraceKind::Milestone: return "milestone";
    case HardwareTraceKind::Assertion: return "assertion";
    case HardwareTraceKind::Trigger: return "trigger";
    }
    return "unknown";
}

const char* hardwareTraceSourceName(HardwareTraceSource source) {
    switch (source) {
    case HardwareTraceSource::Cpu: return "cpu";
    case HardwareTraceSource::Memory: return "memory";
    case HardwareTraceSource::Rom: return "rom";
    case HardwareTraceSource::Via: return "via";
    case HardwareTraceSource::Rtc: return "rtc";
    case HardwareTraceSource::Oss: return "oss";
    case HardwareTraceSource::SccIop: return "scc-iop";
    case HardwareTraceSource::IsmIop: return "ism-iop";
    case HardwareTraceSource::Ncr5380: return "ncr5380";
    case HardwareTraceSource::ScsiDma: return "scsi-dma";
    case HardwareTraceSource::Swim: return "swim";
    case HardwareTraceSource::Asc: return "asc";
    case HardwareTraceSource::NuBus: return "nubus";
    case HardwareTraceSource::Biu: return "biu";
    case HardwareTraceSource::Other: return "other";
    }
    return "unknown";
}

void HardwareTrace::configure(const HardwareTraceConfig& config) {
    config_ = config;
    if (config_.capacity != 0) {
        if (config_.capacity < 2) config_.capacity = 2;
        if (config_.postTriggerEvents >= config_.capacity)
            config_.postTriggerEvents = config_.capacity - 1;
        config_.iopFlightEvents = std::min(config_.iopFlightEvents,
                                            config_.capacity);
        if (config_.triggerIopHits == 0) config_.triggerIopHits = 1;
    }
    events_.clear();
    events_.reserve(config_.capacity);
    iopFlight_.clear();
    iopFlight_.reserve(config_.iopFlightEvents);
    reset();
}

void HardwareTrace::reset() {
    events_.clear();
    iopFlight_.clear();
    ringHead_ = 0;
    iopFlightHead_ = 0;
    postRemaining_ = 0;
    nextSequence_ = 0;
    triggerPcHitCount_ = 0;
    acceptedEvents_ = 0;
    triggered_ = false;
    frozen_ = false;
    triggerReason_.clear();
}

std::size_t HardwareTrace::preTriggerCapacity() const {
    if (config_.capacity == 0) return 0;
    const std::size_t reserve = config_.postTriggerEvents + 1;
    return reserve < config_.capacity ? config_.capacity - reserve : 0;
}

void HardwareTrace::appendPreTrigger(HardwareTraceEvent event) {
    const std::size_t capacity = preTriggerCapacity();
    if (capacity == 0) return;
    if (events_.size() < capacity) {
        events_.push_back(std::move(event));
        return;
    }
    events_[ringHead_] = std::move(event);
    ringHead_ = (ringHead_ + 1) % capacity;
}

void HardwareTrace::appendLinear(HardwareTraceEvent event) {
    if (events_.size() < config_.capacity)
        events_.push_back(std::move(event));
}

void HardwareTrace::normalize() {
    if (ringHead_ == 0 || events_.empty()) return;
    std::rotate(events_.begin(), events_.begin() +
                static_cast<std::ptrdiff_t>(ringHead_), events_.end());
    ringHead_ = 0;
}

void HardwareTrace::record(HardwareTraceEvent event) {
    if (!enabled() || frozen_) return;

    // --trace-pc is a 68030 trigger. IOP instruction records also carry a PC,
    // but their independent 16-bit address space must never satisfy it.
    const bool instruction = event.kind == HardwareTraceKind::Instruction &&
                             event.source == HardwareTraceSource::Cpu;
    if (!triggered_) {
        if (config_.triggerCycle != 0 && event.cycle >= config_.triggerCycle)
            trigger(event.cycle, event.pc, "cycle threshold",
                    event.correlation);
        if (!triggered_ && config_.triggerPcHits != 0 && instruction &&
            event.pc == config_.triggerPc &&
            ++triggerPcHitCount_ >= config_.triggerPcHits)
            trigger(event.cycle, event.pc, "PC hit threshold",
                    event.correlation);
    }

    const bool mandatory = event.kind == HardwareTraceKind::Assertion ||
                           event.kind == HardwareTraceKind::Trigger;
    if (!mandatory) {
        if ((event.categories & config_.categories) == 0 ||
            (config_.sources & hardwareTraceSourceBit(event.source)) == 0)
            return;
        if (event.kind == HardwareTraceKind::Access &&
            (event.address < config_.addressFirst ||
             event.address > config_.addressLast))
            return;
    }
    event.sequence = nextSequence_++;
    ++acceptedEvents_;

    if (!triggered_) {
        appendPreTrigger(std::move(event));
        return;
    }
    appendLinear(std::move(event));
    if (postRemaining_ != 0 && --postRemaining_ == 0) frozen_ = true;
}

void HardwareTrace::recordIopFlight(HardwareTraceEvent event) {
    if (!acceptsIopFlight()) return;
    if ((config_.sources & hardwareTraceSourceBit(event.source)) == 0) return;
    // This is a flight recorder: its value is the bounded lead-in. Continuing
    // at instruction rate would consume the post-trigger protocol window.
    if (triggered_) return;
    event.sequence = nextSequence_++;
    ++acceptedEvents_;

    if (iopFlight_.size() < config_.iopFlightEvents) {
        iopFlight_.push_back(std::move(event));
        return;
    }
    iopFlight_[iopFlightHead_] = std::move(event);
    iopFlightHead_ = (iopFlightHead_ + 1) % config_.iopFlightEvents;
}

void HardwareTrace::mergeIopFlight() {
    if (iopFlight_.empty()) return;
    normalize();

    std::vector<HardwareTraceEvent> flight;
    flight.reserve(iopFlight_.size());
    flight.insert(flight.end(), iopFlight_.begin() +
                  static_cast<std::ptrdiff_t>(iopFlightHead_), iopFlight_.end());
    flight.insert(flight.end(), iopFlight_.begin(), iopFlight_.begin() +
                  static_cast<std::ptrdiff_t>(iopFlightHead_));

    events_.insert(events_.end(), flight.begin(), flight.end());
    std::stable_sort(events_.begin(), events_.end(),
        [](const HardwareTraceEvent& left, const HardwareTraceEvent& right) {
            return left.sequence < right.sequence;
        });
    // Keep the ordinary protocol ring's entire lead-in and make room for the
    // independent instruction recorder by trimming only if their union would
    // consume the trigger/tail reserve.
    const std::size_t capacity = config_.capacity > config_.postTriggerEvents + 1
        ? config_.capacity - config_.postTriggerEvents - 1 : 0;
    if (events_.size() > capacity)
        events_.erase(events_.begin(), events_.begin() +
                      static_cast<std::ptrdiff_t>(events_.size() - capacity));
    iopFlight_.clear();
    iopFlightHead_ = 0;
}

void HardwareTrace::trigger(u64 cycle, u32 pc, const std::string& reason,
                            u64 correlation) {
    if (!enabled() || triggered_) return;
    mergeIopFlight();
    normalize();
    triggered_ = true;
    triggerReason_ = reason;
    postRemaining_ = config_.postTriggerEvents;

    HardwareTraceEvent event;
    event.sequence = nextSequence_++;
    event.cycle = cycle;
    event.pc = pc;
    event.correlation = correlation;
    event.categories = TraceAssertion;
    event.kind = HardwareTraceKind::Trigger;
    event.source = HardwareTraceSource::Other;
    setDetail(event.detail, reason);
    appendLinear(std::move(event));
    if (postRemaining_ == 0) frozen_ = true;
}

void HardwareTrace::assertion(HardwareTraceEvent event,
                              const std::string& reason) {
    if (!enabled() || frozen_) return;
    if (!triggered_)
        trigger(event.cycle, event.pc, reason, event.correlation);
    // A zero-post trigger freezes immediately. Preserve the violation itself
    // by replacing the trigger's final slot only when no post window exists.
    if (frozen_ && config_.postTriggerEvents == 0) {
        frozen_ = false;
        if (!events_.empty()) events_.pop_back();
    }
    event.categories |= TraceAssertion;
    event.kind = HardwareTraceKind::Assertion;
    setDetail(event.detail, reason);
    record(std::move(event));
    if (config_.postTriggerEvents == 0) frozen_ = true;
}

void HardwareTrace::freeze() {
    if (!enabled()) return;
    mergeIopFlight();
    normalize();
    frozen_ = true;
}

std::vector<HardwareTraceEvent> HardwareTrace::events() const {
    if (ringHead_ == 0 || events_.empty()) return events_;
    std::vector<HardwareTraceEvent> chronological;
    chronological.reserve(events_.size());
    chronological.insert(chronological.end(), events_.begin() +
                         static_cast<std::ptrdiff_t>(ringHead_), events_.end());
    chronological.insert(chronological.end(), events_.begin(), events_.begin() +
                         static_cast<std::ptrdiff_t>(ringHead_));
    return chronological;
}

bool HardwareTrace::writeJsonl(std::ostream& output) const {
    output << "{\"schema\":\"openmac.hardware-trace\",\"version\":8"
           << ",\"capacity\":" << config_.capacity
           << ",\"post_trigger_events\":" << config_.postTriggerEvents
           << ",\"iop_flight_events\":" << config_.iopFlightEvents
           << ",\"categories\":" << config_.categories
           << ",\"sources\":" << config_.sources
           << ",\"address_first\":" << config_.addressFirst
           << ",\"address_last\":" << config_.addressLast
           << ",\"triggered\":" << (triggered_ ? "true" : "false")
           << ",\"frozen\":" << (frozen_ ? "true" : "false")
           << ",\"reason\":";
    jsonString(output, triggerReason_.c_str());
    output << "}\n";

    for (const HardwareTraceEvent& event : events()) {
        output << "{\"seq\":" << event.sequence
               << ",\"cycle\":" << event.cycle
               << ",\"pc\":" << event.pc
               << ",\"kind\":";
        jsonString(output, hardwareTraceKindName(event.kind));
        output << ",\"source\":";
        jsonString(output, hardwareTraceSourceName(event.source));
        output << ",\"categories\":" << event.categories
               << ",\"width\":" << static_cast<unsigned>(event.width)
               << ",\"write\":" << ((event.flags & 1u) ? "true" : "false")
               << ",\"address\":" << event.address
               << ",\"value\":" << event.value
               << ",\"auxiliary\":" << event.auxiliary
               << ",\"correlation\":" << event.correlation
               << ",\"detail\":";
        jsonString(output, event.detail.data());
        output << ",\"scsi\":{\"phase\":"
               << static_cast<unsigned>(event.scsi.phase)
               << ",\"flags\":" << static_cast<unsigned>(event.scsi.flags)
               << ",\"position\":" << event.scsi.transferPosition
               << ",\"length\":" << event.scsi.transferLength
               << ",\"dma_control\":" << event.scsi.dmaControl
               << ",\"dma_address\":" << event.scsi.dmaAddress
               << ",\"dma_count\":" << event.scsi.dmaCount
               << ",\"commands\":" << event.scsi.commandCount
               << ",\"cdb\":";
        jsonByteArray(output, event.scsi.cdb, event.scsi.cdbLength);
        output << "},\"swim\":{\"mode\":"
               << static_cast<unsigned>(event.swim.mode)
               << ",\"setup\":" << static_cast<unsigned>(event.swim.setup)
               << ",\"handshake\":" << static_cast<unsigned>(event.swim.handshake)
               << ",\"error\":" << static_cast<unsigned>(event.swim.error)
               << ",\"data\":" << static_cast<unsigned>(event.swim.data)
               << ",\"lines\":" << static_cast<unsigned>(event.swim.lines)
               << ",\"drive_address\":"
               << static_cast<unsigned>(event.swim.driveAddress)
               << ",\"flags\":" << static_cast<unsigned>(event.swim.flags)
               << ",\"crc\":" << event.swim.crc
               << ",\"track\":" << static_cast<unsigned>(event.swim.track)
               << ",\"side\":" << static_cast<unsigned>(event.swim.side)
               << ",\"fifo_occupancy\":"
               << static_cast<unsigned>(event.swim.fifoOccupancy)
               << ",\"fifo_head\":"
               << static_cast<unsigned>(event.swim.fifoHead)
               << ",\"fifo_tail\":"
               << static_cast<unsigned>(event.swim.fifoTail)
               << ",\"read_synced\":"
               << static_cast<unsigned>(event.swim.readSynced)
               << ",\"fifo_head_crc\":" << event.swim.fifoHeadCrc
               << ",\"mechanism_flags\":"
               << static_cast<unsigned>(event.swim.mechanismFlags)
               << ",\"step_remaining\":" << event.swim.stepRemaining
               << ",\"requested_block\":" << event.swim.requestedBlock
               << ",\"expected_track\":"
               << static_cast<unsigned>(event.swim.expectedTrack)
               << ",\"expected_side\":"
               << static_cast<unsigned>(event.swim.expectedSide)
               << ",\"expected_sector\":"
               << static_cast<unsigned>(event.swim.expectedSector)
               << "},\"asc\":{\"mode\":"
               << static_cast<unsigned>(event.asc.mode)
               << ",\"control\":" << static_cast<unsigned>(event.asc.control)
               << ",\"volume\":" << static_cast<unsigned>(event.asc.volume)
               << ",\"clock\":" << static_cast<unsigned>(event.asc.clock)
               << ",\"status\":" << static_cast<unsigned>(event.asc.status)
               << ",\"flags\":" << static_cast<unsigned>(event.asc.flags)
               << ",\"fifo_a\":" << event.asc.fifoLevelA
               << ",\"fifo_b\":" << event.asc.fifoLevelB
               << ",\"phase\":[" << event.asc.phase[0] << ','
               << event.asc.phase[1] << ',' << event.asc.phase[2] << ','
               << event.asc.phase[3] << "]"
               << ",\"increment\":[" << event.asc.increment[0] << ','
               << event.asc.increment[1] << ',' << event.asc.increment[2]
               << ',' << event.asc.increment[3] << "]"
               << ",\"ram_writes\":" << event.asc.ramWrites
               << ",\"produced_samples\":" << event.asc.producedSamples
               << ",\"non_silent_samples\":"
               << event.asc.nonSilentSamples
               << ",\"irq_transitions\":" << event.asc.irqTransitions
               << "},\"iop\":{\"pc\":" << event.iop.pc
               << ",\"opcode\":" << static_cast<unsigned>(event.iop.opcode)
               << ",\"a\":" << static_cast<unsigned>(event.iop.a)
               << ",\"x\":" << static_cast<unsigned>(event.iop.x)
               << ",\"y\":" << static_cast<unsigned>(event.iop.y)
               << ",\"s\":" << static_cast<unsigned>(event.iop.s)
               << ",\"p\":" << static_cast<unsigned>(event.iop.p)
               << ",\"status\":" << static_cast<unsigned>(event.iop.status)
               << ",\"interrupt_mask\":"
               << static_cast<unsigned>(event.iop.interruptMask)
               << ",\"interrupt_flags\":"
               << static_cast<unsigned>(event.iop.interruptFlags)
               << ",\"dma_control\":["
               << static_cast<unsigned>(event.iop.dmaControl[0]) << ','
               << static_cast<unsigned>(event.iop.dmaControl[1])
               << "],\"dma_request\":["
               << static_cast<unsigned>(event.iop.dmaRequest[0]) << ','
               << static_cast<unsigned>(event.iop.dmaRequest[1])
               << "],\"dma_map\":[" << event.iop.dmaMap[0] << ','
               << event.iop.dmaMap[1] << "],\"dma_count\":["
               << event.iop.dmaCount[0] << ',' << event.iop.dmaCount[1]
               << "],\"dma_transfers\":" << event.iop.dmaTransfers
               << ",\"poll_enable\":"
               << static_cast<unsigned>(event.iop.pollEnable)
               << ",\"current_drive\":"
               << static_cast<unsigned>(event.iop.currentDrive)
               << ",\"drive_kind\":"
               << static_cast<unsigned>(event.iop.driveKind)
               << ",\"drive_state\":"
               << static_cast<unsigned>(event.iop.driveState)
               << ",\"drive_format\":"
               << static_cast<unsigned>(event.iop.driveFormat)
               << ",\"receive_state\":"
               << static_cast<unsigned>(event.iop.receiveState)
               << ",\"continuation\":" << event.iop.continuation
               << ",\"instructions\":" << event.iop.instructions
               << "},\"video\":{\"mode\":" << event.video.mode
               << ",\"control\":" << event.video.control
               << ",\"base\":" << event.video.base
               << ",\"stride\":" << event.video.stride
               << ",\"vram_writes\":" << event.video.vramWrites
               << ",\"frame_vram_writes\":" << event.video.frameVramWrites
               << ",\"vblank_count\":" << event.video.vblankCount
               << ",\"vblank_assertions\":" << event.video.vblankAssertions
               << ",\"vblank_acks\":" << event.video.vblankAcks
               << ",\"flags\":" << static_cast<unsigned>(event.video.flags)
               << ",\"bits_per_pixel\":"
               << static_cast<unsigned>(event.video.bitsPerPixel)
               << ",\"oss_ipl\":" << static_cast<unsigned>(event.video.ossIpl)
               << ",\"ramdac_mode\":"
               << static_cast<unsigned>(event.video.ramdacMode)
               << ",\"serial_command\":" << event.video.serialCommand
               << ",\"serial_commands\":" << event.video.serialCommands
               << ",\"timing_echo_writes\":"
               << event.video.timingEchoWrites
               << "},\"mailbox\":";
        if (event.kind != HardwareTraceKind::Mailbox) {
            output << "{}";
        } else {
            output << "{\"length\":"
               << static_cast<unsigned>(event.mailbox.length)
               << ",\"operation\":"
               << static_cast<unsigned>(event.mailbox.operation)
               << ",\"channel\":"
               << static_cast<unsigned>(event.mailbox.channel)
               << ",\"flags\":"
               << static_cast<unsigned>(event.mailbox.flags)
               << ",\"result\":" << event.mailbox.result
               << ",\"ram_address\":" << event.mailbox.ramAddress
               << ",\"block\":" << event.mailbox.block
               << ",\"block_count\":" << event.mailbox.blockCount
               << ",\"expected_track\":"
               << static_cast<unsigned>(event.mailbox.expectedTrack)
               << ",\"expected_side\":"
               << static_cast<unsigned>(event.mailbox.expectedSide)
               << ",\"expected_sector\":"
               << static_cast<unsigned>(event.mailbox.expectedSector)
               << ",\"payload\":";
            jsonByteArray(output, event.mailbox.payload, event.mailbox.length);
            output << '}';
        }
        output << "}\n";
    }
    return output.good();
}

} // namespace openmac
