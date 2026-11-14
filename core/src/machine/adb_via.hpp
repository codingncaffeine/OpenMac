#pragma once

// The ADB transceiver's connection to a VIA, for machines that carry the
// transceiver chip on VIA1 (Macintosh SE, Classic, SE/30, II family): two
// state lines on port B (bits 5-4), the interrupt line on port B bit 3, and
// bytes through the VIA's shift register clocked by the transceiver.
//
// The timing is the Macintosh Classic's, where it was worked out against
// that ROM and System 6 and 7 (machine.cpp): a byte the processor writes
// completes a fixed time after it is armed; entering a data state pushes the
// device's next byte the same time later whether or not the processor has
// read the shift register; reaching the idle state with nothing staged
// cancels a pending shift. Once a frame an idle bus with input waiting gets
// a shift completion so the ROM's ADB manager resumes polling, and input the
// ROM never comes for is dropped after 32 such frames.
//
// The owner calls setState from its port B handler, armed/disarmed from the
// VIA's shift-register callbacks, tick with processor clocks and frameWake
// once a frame.

#include "openmac/types.hpp"

#include "adb.hpp"
#include "via.hpp"

namespace openmac {

class AdbViaPort {
public:
    // `shiftClocks`: processor clocks from a state change or an armed write
    // to the completed byte (about 0.69 ms of machine time).
    AdbViaPort(AdbTransceiver& adb, Via6522& via, int shiftClocks)
        : adb_(adb), via_(via), shiftClocks_(shiftClocks) {}

    void reset() {
        armed_ = false;
        pendingInput_ = false;
        pending_ = 0;
        lastPollTotal_ = 0;
        wakeStreak_ = 0;
    }

    // Port B bits 5-4 as driven.
    void setState(int state) {
        const int prev = adb_.state();
        adb_.setState(state & 3);
        const int st = adb_.state();
        if (st == prev) return;
        if ((st == 1 || st == 2) && !adb_.listening()) {
            pending_ = shiftClocks_;
            pendingInput_ = true;
            armed_ = false;
        } else if (st == 3 && adb_.responsePending()) {
            pending_ = shiftClocks_;
            pendingInput_ = true;
            armed_ = false;
        } else if (st == 3) {
            armed_ = false;
            pending_ = 0;
        } else if (pendingInput_) {
            pending_ = 0;
        }
        maybeClock();
    }

    // The VIA's srArmed callback: only a byte the processor writes arms a
    // transfer; input bytes are pushed by the state lines.
    void armed(bool input) {
        if (input) return;
        armed_ = true;
        maybeClock();
    }
    void disarmed() {
        armed_ = false;
        pending_ = 0;
    }

    // Returns true when a shift completed (the VIA's interrupt state changed).
    bool tick(int clocks) {
        if (pending_ <= 0) return false;
        pending_ -= clocks;
        if (pending_ > 0) return false;
        pending_ = 0;
        if (pendingInput_) {
            via_.completeShift(true, adb_.cpuShiftIn());
        } else {
            adb_.cpuShiftOut(via_.shiftValue());
            via_.completeShift(false, 0);
        }
        return true;
    }

    // Once a frame. Returns true when it woke the ROM.
    bool frameWake() {
        const u32 polls = adb_.mousePolls() + adb_.kbdPolls();
        if (polls != lastPollTotal_) {
            lastPollTotal_ = polls;
            wakeStreak_ = 0;
        }
        if (pending_ == 0 && adb_.state() == 3 && adb_.wakeWorthPoking()) {
            if (wakeStreak_ < 32) {
                adb_.reStageLastTalk();
                via_.completeShift(true, 0xFF);
                ++wakeStreak_;
                return true;
            }
            adb_.flushStaleInput();
            wakeStreak_ = 0;
            return false;
        }
        wakeStreak_ = 0;
        return false;
    }

private:
    friend class QuadraStateCodec;
    void maybeClock() {
        const bool canClock = adb_.state() != 3 || adb_.responsePending();
        if (canClock && armed_ && pending_ == 0) {
            armed_ = false;
            pending_ = shiftClocks_;
            pendingInput_ = false;
        }
    }

    AdbTransceiver& adb_;
    Via6522& via_;
    int shiftClocks_;
    bool armed_ = false;
    bool pendingInput_ = false;
    int pending_ = 0;
    u32 lastPollTotal_ = 0;
    int wakeStreak_ = 0;
};

} // namespace openmac
