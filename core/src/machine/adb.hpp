#pragma once

// ADB transceiver + devices as seen by the SE/Classic ROM. VIA PB4/PB5 select
// the transaction state (0 = command, 1/2 = data bytes, 3 = idle), bytes travel
// through the VIA shift register, and PB3 is the transceiver interrupt line.
//
// The transceiver timing model (verified against this ROM): every armed shift
// completes and /INT (PB3) is held HIGH throughout a Talk response. The ROM
// tells "got data" from "empty address" by the byte it clocks in (0xFF = empty),
// NOT by a /INT edge -- pulsing /INT low mid-response makes the ADB ISR treat a
// routine poll as unsolicited device data and wedge the boot. Commands route to
// devices by address; a keyboard (addr 2) and a mouse (addr 3) hang off the bus.

#include "openmac/types.hpp"

#include <vector>

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
        stagedByWake_ = false;
        kbdAddr_ = 2;
        mouseAddr_ = 3;
        kbdHead_ = kbdTail_ = 0;
        // Preserve physical key state across an ADB reset -- on real hardware the keys
        // are still held -- and re-queue the held keys so the ROM re-learns them via
        // Talk R0 after it re-inits the ADB during boot. This is what makes "hold
        // Shift/Option/Cmd while booting" (skip extensions, zap PRAM, etc.) register.
        for (u8 c = 0; c < 0x80; ++c)
            if (keyState_[c]) {
                const int n = (kbdTail_ + 1) % kKbdQ;
                if (n != kbdHead_) { kbdQ_[kbdTail_] = c; kbdTail_ = n; }
            }
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
        int_ = true;   // every state transition re-raises /INT (the low from an
                       // empty byte spans only until the next state change)
        if (state_ == 3) open_ = false;   // idle ends any transaction
    }
    int state() const { return state_; }
    bool transactionOpen() const { return open_; }
    u8 lastCommand() const { return cmd_; }

    // Diagnostics: how often the ROM has Talk-0-polled each device, and how
    // many of those polls actually carried movement/key data.
    u32 mousePolls() const { return mousePolls_; }
    u32 kbdPolls() const { return kbdPolls_; }
    u32 mouseReports() const { return mouseReports_; }
    u32 mouseBytesRead() const { return mouseBytesRead_; }   // mouse bytes the guest actually clocked in
    const std::vector<u8>& mouseBytesLog() const { return mouseBytesLog_; }
    u32 kbdReg2() const { return kbdReg2_; }   // ROM read modifier state
    u32 kbdReg3() const { return kbdReg3_; }   // keyboard enumerated (device info)
    u32 mouseReg3() const { return mouseReg3_; }
    const std::vector<u8>& cmdTrace() const { return cmdTrace_; }
    const std::vector<u8>& respTrace() const { return respTrace_; }
    void clearCmdTrace() { cmdTrace_.clear(); respTrace_.clear(); }

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
            // /INT is held HIGH for the ENTIRE response, not pulsed between the
            // two bytes. A mid-transfer LOW makes the ROM's ADB ISR read the
            // poll as unsolicited device data and wedge the boot (see emit()).
            // The ROM tells a real report from an empty address by the byte
            // value it clocks in (0xFF = empty), never by a /INT edge.
            if (lastEmitMouse_) {
                ++mouseBytesRead_;
                if (mouseBytesLog_.size() < 16) mouseBytesLog_.push_back(buf_[idx_]);
            }
            return buf_[idx_++];
        }
        if (responsePending()) {
            // A read OUTSIDE the data states while a response waits: the ROM's
            // settled-desktop poll arms once at command time and that shift
            // completes at idle -- it is flushing the register, not collecting
            // data. Serve 0xFF and keep the response intact for the state-1/2
            // reads that follow (serving the real bytes here byte-slipped the
            // whole response and every keystroke vanished).
            return 0xFF;
        }
        // No (more) data: the byte is filler and /INT LOW says so -- that is
        // how the transceiver marks "nothing here" (and how the System's
        // patched ADB manager, which trusts /INT rather than byte values,
        // tells an empty poll from data). /INT re-raises on the next state
        // change. This same low is what sends the manager around its poll-all
        // when another device is sitting on input (the old conditional SRQ),
        // so keyboard and mouse each get their turn either way.
        int_ = false;
        return 0xFF;
    }

    bool intLine() const { return int_; }
    bool listening() const { return listenReg_ >= 0; }

    // A device has movement/keys to report. When the bus is idle the machine
    // must fire a shift-register completion interrupt to wake the ROM into
    // polling (merely asserting /INT has no effect on the ROM's ADB manager).
    bool hasPendingEvent() const { return mousePending_ || kbdHead_ != kbdTail_; }

    // A staged response the ROM has not fully read yet. The ROM's command
    // sequence dips through the idle state BETWEEN issuing a Talk and reading
    // its bytes, so "state 3" alone never means the transaction is over --
    // waking (and restaging) in that window used to wipe the response the ROM
    // was about to read, which is how keystrokes vanished or swapped under a
    // busy mouse: the wake destroyed a staged key report, or replaced a DOWN
    // with the NEXT queued transition.
    bool responsePending() const { return idx_ < len_; }
    // ...and one the ROM itself asked for, which nothing may clobber.
    bool romResponsePending() const { return responsePending() && !stagedByWake_; }
    // Poke the wake when input waits, or when a wake-staged response is still
    // sitting unread (the ROM may have missed the previous poke) -- but never
    // while the ROM is mid-read of its own command's response.
    bool wakeWorthPoking() const {
        return !romResponsePending() && (hasPendingEvent() || responsePending());
    }

    // Re-stage the response for the last Talk command, so an idle wake-up
    // presents valid data rather than a bare interrupt (which the ROM treats
    // as an error and answers with a bus reset). An already-staged unread
    // response is left EXACTLY as it is: restaging a keyboard talk would
    // dequeue the next transition over the top of the unread one.
    void reStageLastTalk() {
        if (responsePending()) return;        // re-poke the same bytes, stage nothing
        if (((cmd_ >> 2) & 3) != 3) return;   // last command was not a Talk
        len_ = idx_ = 0;
        const int addr = (cmd_ >> 4) & 0xF;
        const int reg = cmd_ & 3;
        if (addr == kbdAddr_) talkKeyboard(reg);
        else if (addr == mouseAddr_) talkMouse(reg);
        stagedByWake_ = len_ > 0;
    }

    // Discard input the ROM never came back to poll. In normal use the ROM
    // polls constantly so this is never needed, but an event injected while the
    // ROM is NOT polling (e.g. a mouse nudge during the boot-time "wait for ADB
    // idle" spin) would otherwise keep hasPendingEvent() true forever, making
    // the machine's once-per-frame idle wake fire endlessly and deadlock that
    // spin. Dropping the stale event lets the bus reach idle.
    void flushStaleInput() {
        mousePending_ = false;
        mouseDx_ = mouseDy_ = 0;
        kbdHead_ = kbdTail_ = 0;
        if (stagedByWake_) { len_ = idx_ = 0; stagedByWake_ = false; }
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
        stagedByWake_ = false;   // whatever follows is the ROM's own transaction
        listenReg_ = -1;
        const int addr = (cmd_ >> 4) & 0xF;
        const int op   = (cmd_ >> 2) & 0x3;
        const int reg  = cmd_ & 0x3;

        if (op == 2) {          // Listen: capture the data bytes that follow
            listenReg_ = reg;
            listenAddr_ = addr;
            listenPos_ = 0;
        } else if (op == 3) {
            if (addr == kbdAddr_) talkKeyboard(reg);
            else if (addr == mouseAddr_) talkMouse(reg);
        }
        if (cmdTrace_.size() < 512) {
            cmdTrace_.push_back(cmd_);
            respTrace_.push_back(len_ > 0 ? 1 : 0);   // did a device answer?
        }
    }

    void talkKeyboard(int reg) {
        lastEmitMouse_ = false;
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
            ++kbdReg2_;
            // Modifier/LED register: bit = 0 means the key is down.
            u8 hi = 0xFF;
            if (keyState_[adbkey::kCommand])  hi &= ~0x01u;
            if (keyState_[adbkey::kOption])   hi &= ~0x02u;
            if (keyState_[adbkey::kShift])    hi &= ~0x04u;
            if (keyState_[adbkey::kControl])  hi &= ~0x08u;
            if (keyState_[adbkey::kCapsLock]) hi &= ~0x20u;
            emit(hi, 0xFF);
        } else if (reg == 3) {
            ++kbdReg3_;
            emit(0x62, 0x02);   // excep-event | SRQ | addr 2 | handler 2 (ext kbd)
        }
    }

    void talkMouse(int reg) {
        if (reg == 0) {
            ++mousePolls_;
            // Bus fairness: while the keyboard has transitions waiting, answer
            // the mouse poll EMPTY. The empty response with /INT low is the one
            // SRQ shape the ROM's ISR acts on (measured: /INT during a data
            // response is ignored), so this is what makes the ROM poll-all and
            // drain the keyboard when a game moves the mouse every frame --
            // otherwise every poll carries motion and keys starve until the
            // queue overflows. The motion is deferred, not lost: the deltas
            // keep accumulating and go out on the next poll.
            if (kbdHead_ != kbdTail_) return;
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
            lastEmitMouse_ = true;
            emit(b0, b1);
        } else if (reg == 3) {
            ++mouseReg3_;
            emit(0x63, 0x01);   // excep-event | SRQ | addr 3 | handler 1 (100cpi mouse)
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
    bool stagedByWake_ = false;   // current staged response came from reStageLastTalk

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
    u32 mouseBytesRead_ = 0;
    bool lastEmitMouse_ = false;
    std::vector<u8> mouseBytesLog_;
    u32 kbdReg2_ = 0, kbdReg3_ = 0, mouseReg3_ = 0;
    std::vector<u8> cmdTrace_;
    std::vector<u8> respTrace_;
};

} // namespace openmac
