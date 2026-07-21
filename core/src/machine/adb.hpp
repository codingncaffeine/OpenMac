#pragma once

// ADB transceiver as seen by the SE/Classic ROM: VIA PB4/PB5 select the
// transaction state (0 = command, 1/2 = data bytes, 3 = idle), bytes travel
// through the VIA shift register, and PB3 is the transceiver interrupt line.
//
// Model (after observed ROM behavior and the Mini vMac reference): every
// armed shift completes; a Talk with no data supplies 0xFF and drops the
// interrupt line, which is how the ROM recognizes an empty address. Devices
// (keyboard, mouse) plug into the buffer in the next stage.

#include "openmac/types.hpp"

namespace openmac {

class AdbTransceiver {
public:
    void reset() {
        state_ = 3;
        cmd_ = 0;
        len_ = idx_ = 0;
        int_ = true;
    }

    void setState(int state) {
        state_ = state & 3;
        if (state_ == 0) int_ = true;   // new command window
        if (state_ == 3) {              // idle ends any transaction
            int_ = true;
            open_ = false;
        }
    }
    int state() const { return state_; }
    bool transactionOpen() const { return open_; }

    // CPU shifted a byte out (command in state 0, Listen data in 1/2).
    void cpuShiftOut(u8 value) {
        if (state_ == 0) {
            cmd_ = value;
            runCommand();
        }
        // Listen data is absorbed until devices exist.
    }

    // CPU armed a shift-in (Talk data in states 1/2). Always yields a byte.
    // /INT is held LOW while valid data is being clocked and HIGH once the
    // transaction is over — an empty Talk reads back garbage with /INT high.
    u8 cpuShiftIn() {
        if ((state_ == 1 || state_ == 2) && idx_ < len_) {
            const u8 v = buf_[idx_++];
            int_ = idx_ < len_ ? false : true;
            return v;
        }
        int_ = true;
        return 0xFF;
    }

    bool intLine() const { return int_; }

private:
    void runCommand() {
        // cmd: [addr:4][cmd:2][reg:2]. No devices attached yet, so every
        // Talk produces an empty response: /INT stays high.
        len_ = idx_ = 0;
        int_ = true;
        open_ = true;
    }

    int state_ = 3;
    u8 cmd_ = 0;
    u8 buf_[8]{};
    int len_ = 0, idx_ = 0;
    bool int_ = true;
    bool open_ = false;
};

} // namespace openmac
