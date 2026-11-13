#pragma once

// The Quadra 950's system microcontroller: the clock, the 256 bytes of
// parameter RAM, power and reset. The 950 has no 343-0042 clock chip; this
// part answers for all of that over VIA1's shift register, in the protocol
// known from the IIsi's "Egret".
//
// Modeled as the protocol, not as the chip: there is no firmware here. The
// conversation is taken from the 950 ROM's own routines (hardware dossier,
// "The microcontroller") and the public Linux driver (via-cuda.c, the Egret
// variant), which agree.
//
// Lines, all on VIA1 port B:
//   PB5 out  system session        the host is sending, or accepting a reply
//   PB4 out  VIA full              host: "a byte is in my shift register" when
//                                  sending; byte acknowledge when receiving
//   PB3 in   transceiver session   low while the controller has bytes to send
//
// Host to controller: raise system session, load the shift register, raise
// VIA full -> the controller clocks the byte out of the VIA. Lower VIA full,
// repeat per byte, lower system session to end the packet.
//
// Controller to host: it pulls transceiver session low and clocks in one
// attention byte the host throws away. The host raises system session; each
// following byte arrives when system session rises or VIA full falls, and
// transceiver session is released with the last one. The host may stop early
// by lowering system session.
//
// Packets are [type][command][data...] from the host and, after the
// attention byte, [type][flags][command][data...] back. Type 1 is the
// controller's own command set, type 2 an error, type 3 the once-a-second
// tick the host asked for (command $1B); the ROM hands a type 3 packet to its
// one-second interrupt routine without looking further.
//
// Byte transfers complete at once. On the real part a byte takes a fraction
// of a millisecond; the ROM polls or waits on the shift-register interrupt
// either way.

#include "openmac/types.hpp"

#include <array>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <vector>

namespace openmac {

class EgretController {
    friend class QuadraStateCodec;
public:
    // Commands of packet type 1 the model acts on. Names follow the public
    // Linux header (cuda.h) where it has one.
    enum Command : u8 {
        WarmStart = 0x00, AutoPoll = 0x01, ReadMemory = 0x02, GetTime = 0x03,
        ReadPram = 0x07, WriteMemory = 0x08, SetTime = 0x09, PowerDown = 0x0A,
        WritePram = 0x0C, SendDfac = 0x0E, ResetSystem = 0x11,
        OneSecondMode = 0x1B,
    };
    static constexpr u8 kPseudoPacket = 0x01;
    static constexpr u8 kTickPacket = 0x03;
    // Parameter RAM is also the controller's own memory from here up: the ROM
    // writes its defaults through WriteMemory.
    static constexpr u16 kPramInMemory = 0x0100;

    // ---- wiring (set by the board) -----------------------------------------
    // The byte the host has loaded into its shift register.
    std::function<u8()> hostShiftOut;
    // Complete the host's shift: `input` false = its byte was taken, true =
    // `value` is now in its shift register. Either raises the VIA's flag.
    std::function<void(bool input, u8 value)> completeShift;
    std::function<void()> onPowerDown;
    std::function<void()> onResetSystem;
    std::function<void(const char* msg)> onDiag;

    // The machine was switched on or reset: the lines are idle and nothing is
    // in flight. The clock and parameter RAM are the part's own and stay.
    void reset() {
        state_ = State::Idle;
        sysSession_ = viaFull_ = false;
        xcvrSession_ = false;
        command_.clear();
        reply_.clear();
        replyPos_ = 0;
        tickPending_ = false;
        oneSecondMode_ = 0;
        autoPoll_ = false;
    }

    // Port B as the host drives it (PB5, PB4).
    void setHostLines(bool sysSession, bool viaFull) {
        const bool sessionRise = sysSession && !sysSession_;
        const bool sessionFall = !sysSession && sysSession_;
        const bool fullRise = viaFull && !viaFull_;
        const bool fullFall = !viaFull && viaFull_;
        sysSession_ = sysSession;
        viaFull_ = viaFull;

        if (state_ == State::Replying) {
            if (sessionFall) {                       // the host has what it wanted
                xcvrSession_ = false;
                state_ = State::Idle;
                return;
            }
            if (xcvrSession_ && sysSession_ && (sessionRise || fullFall)) shiftNext();
            return;
        }
        if (fullRise && sysSession_) {               // a byte from the host
            state_ = State::Receiving;
            if (command_.size() < kMaxPacket) command_.push_back(hostShiftOut ? hostShiftOut() : 0);
            if (completeShift) completeShift(false, 0);
            return;
        }
        if (sessionFall && state_ == State::Receiving) execute();
    }

    // PB3 as the host reads it: false (low) while a reply or a tick is offered.
    bool xcvrSessionLine() const { return !xcvrSession_; }

    // One second of machine time.
    void tickSecond() {
        ++seconds_;
        if (oneSecondMode_ != 0) tickPending_ = true;
    }

    // Offer a waiting tick once the lines are idle. Call after anything that
    // may have freed them (the board calls it from its device clock).
    void poll() {
        if (!tickPending_ || state_ != State::Idle || sysSession_ || viaFull_) return;
        tickPending_ = false;
        reply_.assign({0x00, kTickPacket, 0x00, 0x00});
        beginReply();
    }

    u32 seconds() const { return seconds_; }
    void setSeconds(u32 s) { seconds_ = s; }
    std::array<u8, 256>& pram() { return pram_; }
    const std::array<u8, 256>& pram() const { return pram_; }
    u8 oneSecondMode() const { return oneSecondMode_; }
    bool idle() const { return state_ == State::Idle; }

private:
    static constexpr std::size_t kMaxPacket = 300;
    enum class State { Idle, Receiving, Replying };

    void beginReply() {
        replyPos_ = 0;
        state_ = State::Replying;
        xcvrSession_ = true;
        shiftNext();                                 // the attention byte
    }

    void shiftNext() {
        const u8 value = replyPos_ < reply_.size() ? reply_[replyPos_] : 0;
        ++replyPos_;
        if (replyPos_ >= reply_.size()) xcvrSession_ = false;   // released with the last byte
        if (completeShift) completeShift(true, value);
    }

    u8 memory(u32 addr) const {
        return (addr >= kPramInMemory && addr < kPramInMemory + 256u) ? pram_[addr - kPramInMemory] : 0;
    }

    void execute() {
        const u8 type = command_.size() > 0 ? command_[0] : 0;
        const u8 op = command_.size() > 1 ? command_[1] : 0;
        const auto arg = [this](std::size_t i) -> u8 { return i < command_.size() ? command_[i] : 0; };
        const u32 addr = static_cast<u32>(arg(2)) << 8 | arg(3);
        reply_.assign({0x00, type, 0x00, op});       // attention, type, flags, command
        bool known = type == kPseudoPacket;
        if (known) switch (op) {
        case WarmStart:
            break;
        case AutoPoll:
            autoPoll_ = arg(2) != 0;
            break;
        case ReadMemory:                             // as many bytes as the host takes
            for (u32 i = 0; i < 256; ++i) reply_.push_back(memory(addr + i));
            break;
        case GetTime:
            for (int shift = 24; shift >= 0; shift -= 8) reply_.push_back(static_cast<u8>(seconds_ >> shift));
            break;
        case ReadPram:
            for (u32 i = addr; i < 256; ++i) reply_.push_back(pram_[i]);
            break;
        case WriteMemory:
            for (std::size_t i = 4; i < command_.size(); ++i) {
                const u32 at = addr + static_cast<u32>(i - 4);
                if (at >= kPramInMemory && at < kPramInMemory + 256u) pram_[at - kPramInMemory] = command_[i];
            }
            break;
        case SetTime:
            if (command_.size() >= 6)
                seconds_ = static_cast<u32>(arg(2)) << 24 | static_cast<u32>(arg(3)) << 16 |
                           static_cast<u32>(arg(4)) << 8 | arg(5);
            break;
        case WritePram:
            for (std::size_t i = 4; i < command_.size(); ++i) {
                const u32 at = addr + static_cast<u32>(i - 4);
                if (at < 256) pram_[at] = command_[i];
            }
            break;
        case SendDfac:
            break;
        case OneSecondMode:
            oneSecondMode_ = arg(2);
            if (oneSecondMode_ == 0) tickPending_ = false;
            break;
        case PowerDown:
            if (onPowerDown) onPowerDown();
            break;
        case ResetSystem:
            if (onResetSystem) onResetSystem();
            break;
        default:
            known = false;
            break;
        }
        if (!known && onDiag) {
            char text[96];
            int n = std::snprintf(text, sizeof text, "microcontroller: unhandled packet");
            for (std::size_t i = 0; i < command_.size() && i < 8 && n > 0 && n < 80; ++i)
                n += std::snprintf(text + n, sizeof text - static_cast<std::size_t>(n), " %02X", command_[i]);
            onDiag(text);
        }
        command_.clear();
        beginReply();
    }

    State state_ = State::Idle;
    bool sysSession_ = false, viaFull_ = false;
    bool xcvrSession_ = false;        // we are holding PB3 low
    std::vector<u8> command_, reply_;
    std::size_t replyPos_ = 0;
    bool tickPending_ = false;
    u8 oneSecondMode_ = 0;
    bool autoPoll_ = false;
    u32 seconds_ = 0;
    std::array<u8, 256> pram_{};
};

} // namespace openmac
