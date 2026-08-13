#pragma once

// Bit-serial Apple Desktop Bus as connected to GPIO0 of the IIfx ISM PIC.
//
// The PIC drives an open-collector wire through an inverter and samples the
// same wire on GPIN0.  Time is supplied in 65C02 cycles; internally it is
// converted to a 2 MHz half-microsecond timebase so the documented ADB pulse
// windows can be represented without floating point.  The device behavior is
// provided by AdbTransceiver, shared with OpenMac's other Macintosh models;
// this class is solely the physical-layer serializer.

#include "openmac/types.hpp"

#include <array>
#include <vector>

namespace openmac {

class AdbTransceiver;
class IifxStateCodec;

class IifxAdbBus {
public:
    explicit IifxAdbBus(AdbTransceiver& devices);

    void reset();

    // `released` is the electrical state after the IIfx board inverter:
    // true releases the wire high, false pulls it low.
    void hostDrive(bool released, u64 iopCpuCycles);
    bool readLine(u64 iopCpuCycles);

    u64 commands() const { return commands_; }
    u64 replies() const { return replies_; }
    u64 replyBytes() const { return replyBytes_; }
    u64 resets() const { return resets_; }
    u64 replySamples() const { return replySamples_; }
    u64 replyLowSamples() const { return replyLowSamples_; }
    u64 abortedReplies() const { return abortedReplies_; }
    u64 listenTransactions() const { return listenTransactions_; }
    u8 lastCommand() const { return lastCommand_; }
    u8 lastListenByte(int index) const {
        return index >= 0 && index < 2 ? lastListen_[index] : 0;
    }

private:
    friend class IifxStateCodec;

    // Receive states are deliberately expressed as wire protocol states, not
    // PIC firmware states.  ADB encodes a bit in the high portion of a
    // nominal 100-us cell; the falling edge therefore completes each bit.
    enum class ReceiveState {
        Idle,
        Sync,
        Bits,
        Stop,
        ListenTurnaround,
        ListenStart,
    };

    struct LineEvent {
        u64 tick = 0;
        bool released = true;
        bool final = false;
    };

    static u64 busTicks(u64 iopCpuCycles);
    void advance(u64 now);
    void hostEdge(bool level, u64 now);
    void finishCommand(u64 now);
    void finishListenByte(u64 now);
    void scheduleReply(u64 now, const u8* bytes, int count);
    void scheduleServiceRequest(u64 now);
    void setDeviceLine(bool released, bool final);
    void updateWire();

    AdbTransceiver& devices_;
    ReceiveState receiveState_ = ReceiveState::Idle;
    bool hostReleased_ = true;
    bool deviceReleased_ = true;
    bool wireHigh_ = true;
    bool deviceSending_ = false;
    u64 now_ = 0;
    u64 lastEdge_ = 0;
    u8 shift_ = 0;
    int bit_ = 0;
    int listenBytes_ = 0;
    std::array<u8, 2> lastListen_{};
    std::vector<LineEvent> events_;
    std::size_t nextEvent_ = 0;

    u64 commands_ = 0;
    u64 replies_ = 0;
    u64 replyBytes_ = 0;
    u64 resets_ = 0;
    u64 replySamples_ = 0;
    u64 replyLowSamples_ = 0;
    u64 abortedReplies_ = 0;
    u64 listenTransactions_ = 0;
    u8 lastCommand_ = 0;
};

} // namespace openmac
