#pragma once

// ADB transceiver + devices as seen by the SE/Classic ROM. VIA PB4/PB5 select
// the transaction state (0 = command, 1/2 = data bytes, 3 = idle), bytes travel
// through the VIA shift register, and PB3 is the transceiver interrupt line.
//
// The transceiver timing model (verified against this ROM): every armed shift
// completes; /INT (PB3) is held LOW while a Talk is still clocking valid
// response bytes and HIGH once the response is exhausted, which is how the ROM
// tells "got data" from "empty address". Commands route to devices by address;
// a keyboard (addr 2) and a mouse (addr 3) hang off the bus.

#include "openmac/types.hpp"

namespace openmac {

// ADB keycodes we care about (Apple keyboard layout).
namespace adbkey {
inline constexpr u8 kX = 0x07;
inline constexpr u8 kO = 0x1F;
inline constexpr u8 kCommand = 0x37;
inline constexpr u8 kShift   = 0x38;
inline constexpr u8 kCapsLock = 0x39;
inline constexpr u8 kOption  = 0x3A;
inline constexpr u8 kControl = 0x3B;
} // namespace adbkey

class AdbTransceiver {
public:
    void reset() {
        state_ = 3;
        cmd_ = 0;
        len_ = idx_ = 0;
        int_ = true;
        open_ = false;
        kbdAddr_ = 2;
        mouseAddr_ = 3;
        kbdHead_ = kbdTail_ = 0;
        for (auto& k : keyState_) k = false;
        mouseDx_ = mouseDy_ = 0;
        mouseButton_ = false;
        mousePending_ = false;
        listenReg_ = -1;
        listenAddr_ = 0;
        listenPos_ = 0;
    }

    // ---- host input injection ----
    void injectKey(u8 adbCode, bool down) {
        const u8 code = adbCode & 0x7F;
        if (keyState_[code] == down) return;          // no transition
        keyState_[code] = down;
        const u8 ev = static_cast<u8>(code | (down ? 0x00 : 0x80));
        const int next = (kbdTail_ + 1) % kKbdQ;
        if (next != kbdHead_) { kbdQ_[kbdTail_] = ev; kbdTail_ = next; }
    }
    bool keyHeld(u8 adbCode) const { return keyState_[adbCode & 0x7F]; }

    void injectMouse(int dx, int dy, bool button) {
        mouseDx_ += dx;
        mouseDy_ += dy;
        mouseButton_ = button;
        mousePending_ = true;
    }

    // ---- transceiver state lines ----
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
    u8 lastCommand() const { return cmd_; }

    // Diagnostics: how often the ROM has Talk-0-polled each device, and how
    // many of those polls actually carried movement/key data.
    u32 mousePolls() const { return mousePolls_; }
    u32 kbdPolls() const { return kbdPolls_; }
    u32 mouseReports() const { return mouseReports_; }

    void cpuShiftOut(u8 value) {
        if (state_ == 0) {
            cmd_ = value;
            runCommand();
        } else if ((state_ == 1 || state_ == 2) && listenReg_ == 3) {
            // Listen register 3: two bytes = a new device register 3. The
            // ROM uses this to relocate devices while probing for address
            // collisions, so the addressed device must actually move.
            if (listenPos_ < 2) listenBuf_[listenPos_++] = value;
            if (listenPos_ == 2) {
                const int newAddr = listenBuf_[0] & 0x0F;
                if (listenAddr_ == kbdAddr_) kbdAddr_ = newAddr;
                else if (listenAddr_ == mouseAddr_) mouseAddr_ = newAddr;
                listenReg_ = -1;
            }
        }
    }

    u8 cpuShiftIn() {
        if ((state_ == 1 || state_ == 2) && idx_ < len_) {
            const u8 v = buf_[idx_++];
            int_ = idx_ < len_ ? false : true;
            return v;
        }
        int_ = true;
        return 0xFF;
    }

    bool intLine() const {
        // While idle, a device holding data asserts SRQ (pulls /INT low) so the
        // ROM's poll comes around and Talks register 0 to it. This only fires
        // between transactions, so it can't disturb enumeration.
        if (state_ == 3 && (mousePending_ || kbdHead_ != kbdTail_)) return false;
        return int_;
    }

private:
    static constexpr int kKbdQ = 32;

    void emit(u8 a, u8 b) {
        buf_[0] = a;
        buf_[1] = b;
        len_ = 2;
        // /INT stays high: the CPU reads the two response bytes straight off
        // the shift register. (Pulling it low pushed the ROM's ADB ISR into
        // its "unsolicited device data" branch during enumeration.)
    }

    void runCommand() {
        // cmd: [addr:4][cmd:2][reg:2]. cmd 0=reset 1=flush 2=listen 3=talk.
        len_ = idx_ = 0;
        int_ = true;
        open_ = true;
        listenReg_ = -1;
        const int addr = (cmd_ >> 4) & 0xF;
        const int op   = (cmd_ >> 2) & 0x3;
        const int reg  = cmd_ & 0x3;

        if (op == 2) {          // Listen: capture the data bytes that follow
            listenReg_ = reg;
            listenAddr_ = addr;
            listenPos_ = 0;
            return;
        }
        if (op != 3) return;    // reset/flush produce no response data

        if (addr == kbdAddr_) talkKeyboard(reg);
        else if (addr == mouseAddr_) talkMouse(reg);
        // any other address: empty (no device), /INT stays high
    }

    void talkKeyboard(int reg) {
        if (reg == 0) {
            ++kbdPolls_;
            if (kbdHead_ == kbdTail_) return;          // no transitions pending
            const u8 first = kbdQ_[kbdHead_];
            kbdHead_ = (kbdHead_ + 1) % kKbdQ;
            u8 second = 0xFF;
            if (kbdHead_ != kbdTail_) {
                second = kbdQ_[kbdHead_];
                kbdHead_ = (kbdHead_ + 1) % kKbdQ;
            }
            emit(first, second);
        } else if (reg == 2) {
            // Modifier/LED register: bit = 0 means the key is down.
            u8 hi = 0xFF;
            if (keyState_[adbkey::kCommand])  hi &= ~0x01u;
            if (keyState_[adbkey::kOption])   hi &= ~0x02u;
            if (keyState_[adbkey::kShift])    hi &= ~0x04u;
            if (keyState_[adbkey::kControl])  hi &= ~0x08u;
            if (keyState_[adbkey::kCapsLock]) hi &= ~0x20u;
            emit(hi, 0xFF);
        } else if (reg == 3) {
            emit(0x22, 0x02);   // SRQ-enable | addr 2 | handler 2 (extended kbd)
        }
    }

    void talkMouse(int reg) {
        if (reg == 0) {
            ++mousePolls_;
            if (!mousePending_) return;
            mousePending_ = false;
            ++mouseReports_;
            const int dx = clampDelta(mouseDx_);
            const int dy = clampDelta(mouseDy_);
            mouseDx_ -= dx;
            mouseDy_ -= dy;
            // byte0: bit7 = button (0 = down), bits6-0 = Y delta (7-bit signed)
            // byte1: bit7 = 1,                  bits6-0 = X delta (7-bit signed)
            const u8 b0 = static_cast<u8>((mouseButton_ ? 0x00 : 0x80) | (dy & 0x7F));
            const u8 b1 = static_cast<u8>(0x80 | (dx & 0x7F));
            emit(b0, b1);
        } else if (reg == 3) {
            emit(0x23, 0x01);   // SRQ-enable | addr 3 | handler 1 (100cpi mouse)
        }
    }

    static int clampDelta(int v) {
        if (v > 63) return 63;
        if (v < -63) return -63;
        return v;
    }

    // transceiver
    int state_ = 3;
    u8 cmd_ = 0;
    u8 buf_[8]{};
    int len_ = 0, idx_ = 0;
    bool int_ = true;
    bool open_ = false;

    // devices
    int kbdAddr_ = 2, mouseAddr_ = 3;
    bool keyState_[128]{};
    u8 kbdQ_[kKbdQ]{};
    int kbdHead_ = 0, kbdTail_ = 0;
    int mouseDx_ = 0, mouseDy_ = 0;
    bool mouseButton_ = false;
    bool mousePending_ = false;
    int listenReg_ = -1, listenAddr_ = 0, listenPos_ = 0;
    u8 listenBuf_[2]{};
    u32 mousePolls_ = 0, kbdPolls_ = 0, mouseReports_ = 0;
};

} // namespace openmac
