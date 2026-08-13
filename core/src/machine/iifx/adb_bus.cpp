#include "adb_bus.hpp"

#include "../adb.hpp"

namespace openmac {

namespace {

// ADB timing in 2 MHz ticks.  These values sit near the centers of the
// receiver windows in Apple's ADB specification and preserve the timings used
// by the original IIfx PIC firmware: 35/65-us data halves, 150-us SRQ, and a
// response turnaround centered in the permitted stop-to-start window.
constexpr u64 kAttentionMinimum = 1200; // 600 us
constexpr u64 kResetMinimum = 4500;     // 2.25 ms
constexpr u64 kCellHalfMinimum = 90;    // 45 us
constexpr u64 kListenTurnaround = 300;  // 150 us
constexpr u64 kReplyTurnaround = 81;    // 40.5 us
constexpr u64 kShort = 70;              // 35 us
constexpr u64 kLong = 130;              // 65 us
constexpr u64 kServiceRequest = 300;     // 150 us

} // namespace

IifxAdbBus::IifxAdbBus(AdbTransceiver& devices) : devices_(devices) {
    reset();
}

void IifxAdbBus::reset() {
    receiveState_ = ReceiveState::Idle;
    hostReleased_ = deviceReleased_ = wireHigh_ = true;
    deviceSending_ = false;
    now_ = lastEdge_ = 0;
    shift_ = 0;
    bit_ = listenBytes_ = 0;
    lastListen_.fill(0);
    events_.clear();
    nextEvent_ = 0;
    commands_ = replies_ = replyBytes_ = resets_ = 0;
    replySamples_ = replyLowSamples_ = abortedReplies_ = 0;
    listenTransactions_ = 0;
    lastCommand_ = 0;
}

u64 IifxAdbBus::busTicks(u64 iopCpuCycles) {
    // The PIC clock is 15.6672 MHz and the 65C02 receives clock/8 =
    // 1.9584 MHz.  2,000,000 / 1,958,400 reduces exactly to 625 / 612.
    return (iopCpuCycles * 625u) / 612u;
}

void IifxAdbBus::hostDrive(bool released, u64 iopCpuCycles) {
    const u64 tick = busTicks(iopCpuCycles);
    advance(tick);
    if (hostReleased_ == released) return;

    // A new host transaction aborts a reply still on the wire, just as a
    // master pulling an open-collector bus low dominates a device.
    if (!released && deviceSending_) {
        ++abortedReplies_;
        events_.clear();
        nextEvent_ = 0;
        deviceSending_ = false;
        deviceReleased_ = true;
    }

    hostReleased_ = released;
    const bool newWire = hostReleased_ && deviceReleased_;
    if (newWire != wireHigh_) {
        wireHigh_ = newWire;
        hostEdge(wireHigh_, tick);
    }
}

bool IifxAdbBus::readLine(u64 iopCpuCycles) {
    advance(busTicks(iopCpuCycles));
    if (deviceSending_) {
        ++replySamples_;
        if (!wireHigh_) ++replyLowSamples_;
    }
    return wireHigh_;
}

void IifxAdbBus::advance(u64 now) {
    if (now < now_) now = now_;
    while (nextEvent_ < events_.size() && events_[nextEvent_].tick <= now) {
        const LineEvent event = events_[nextEvent_++];
        setDeviceLine(event.released, event.final);
    }
    if (nextEvent_ == events_.size() && !events_.empty()) {
        events_.clear();
        nextEvent_ = 0;
    }
    now_ = now;
}

void IifxAdbBus::setDeviceLine(bool released, bool final) {
    deviceReleased_ = released;
    if (final) deviceSending_ = false;
    updateWire();
}

void IifxAdbBus::updateWire() {
    // Device-generated edges are consumed by the PIC through readLine(); they
    // must not be fed back into the host-command decoder.
    wireHigh_ = hostReleased_ && deviceReleased_;
}

void IifxAdbBus::hostEdge(bool level, u64 now) {
    const u64 duration = now - lastEdge_;
    lastEdge_ = now;

    switch (receiveState_) {
    case ReceiveState::Idle:
        if (!level) return; // beginning of attention/reset low pulse
        if (duration >= kResetMinimum) {
            devices_.reset();
            ++resets_;
        } else if (duration >= kAttentionMinimum) {
            receiveState_ = ReceiveState::Sync;
            shift_ = 0;
            bit_ = 0;
        }
        break;

    case ReceiveState::Sync:
        if (!level && duration >= kCellHalfMinimum) {
            receiveState_ = ReceiveState::Bits;
            shift_ = 0;
            bit_ = 0;
        }
        break;

    case ReceiveState::Bits:
        if (level) break;
        shift_ = static_cast<u8>((shift_ << 1) |
                                 (duration >= kCellHalfMinimum ? 1u : 0u));
        if (++bit_ == 8) receiveState_ = ReceiveState::Stop;
        break;

    case ReceiveState::Stop:
        if (!level) break;
        if (listenBytes_ != 0 || devices_.listening())
            finishListenByte(now);
        else
            finishCommand(now);
        break;

    case ReceiveState::ListenTurnaround:
        if (!level && duration >= kListenTurnaround)
            receiveState_ = ReceiveState::ListenStart;
        break;

    case ReceiveState::ListenStart:
        if (!level && duration >= kCellHalfMinimum) {
            receiveState_ = ReceiveState::Bits;
            shift_ = 0;
            bit_ = 0;
        }
        break;
    }
}

void IifxAdbBus::finishCommand(u64 now) {
    const u8 command = shift_;
    lastCommand_ = command;
    ++commands_;

    if (command == 0) {
        devices_.reset();
        ++resets_;
        receiveState_ = ReceiveState::Idle;
        return;
    }

    devices_.setState(0);
    devices_.cpuShiftOut(command);
    const int operation = (command >> 2) & 3;
    if (operation == 2 && devices_.listening()) {
        devices_.setState(1);
        listenBytes_ = 0;
        receiveState_ = ReceiveState::ListenTurnaround;
        return;
    }

    std::array<u8, 8> reply{};
    int count = 0;
    if (operation == 3) {
        devices_.setState(1);
        while (count < static_cast<int>(reply.size()) &&
               devices_.responsePending())
            reply[static_cast<std::size_t>(count++)] = devices_.cpuShiftIn();
    }
    devices_.setState(3);
    receiveState_ = ReceiveState::Idle;

    if (count != 0) {
        scheduleReply(now, reply.data(), count);
    } else if (devices_.hasPendingEvent()) {
        // Another device may request service during the stop-to-start time of
        // an otherwise empty Talk transaction.
        scheduleServiceRequest(now);
    }
}

void IifxAdbBus::finishListenByte(u64) {
    if (listenBytes_ < static_cast<int>(lastListen_.size()))
        lastListen_[static_cast<std::size_t>(listenBytes_)] = shift_;
    devices_.cpuShiftOut(shift_);
    ++listenBytes_;
    if (listenBytes_ < 2) {
        shift_ = 0;
        bit_ = 0;
        receiveState_ = ReceiveState::Bits;
        return;
    }
    devices_.setState(3);
    ++listenTransactions_;
    listenBytes_ = 0;
    receiveState_ = ReceiveState::Idle;
}

void IifxAdbBus::scheduleReply(u64 now, const u8* bytes, int count) {
    events_.clear();
    nextEvent_ = 0;
    deviceReleased_ = true;
    deviceSending_ = true;

    u64 at = now + kReplyTurnaround;
    events_.push_back({at, true, false});
    at += kShort;
    events_.push_back({at, false, false});
    at += kShort;
    events_.push_back({at, true, false});
    at += kLong;
    events_.push_back({at, false, false});

    for (int byte = 0; byte < count; ++byte) {
        for (int bit = 7; bit >= 0; --bit) {
            const bool one = (bytes[byte] & (1u << bit)) != 0;
            at += one ? kShort : kLong;
            events_.push_back({at, true, false});
            at += one ? kLong : kShort;
            events_.push_back({at, false, false});
        }
    }
    at += kShort * 2;
    events_.push_back({at, true, true});
    ++replies_;
    replyBytes_ += static_cast<u64>(count);
}

void IifxAdbBus::scheduleServiceRequest(u64 now) {
    events_.clear();
    nextEvent_ = 0;
    deviceSending_ = true;
    setDeviceLine(false, false);
    events_.push_back({now + kServiceRequest, true, true});
}

} // namespace openmac
