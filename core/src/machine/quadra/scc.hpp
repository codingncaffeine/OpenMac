#pragma once

// Zilog Z8530 SCC for the Quadra, modeled far enough that System 7.5's
// LocalTalk driver ('ltlk' 0) can run its node-acquisition dance against an
// empty wire and hear every step complete.
//
// The previous stub answered RR0/RR1 with "silent wire" constants, which
// satisfied System 7.0's POLLING LocalTalk on the Classic -- but 7.5's driver
// is interrupt-driven: it pops its own return address into a completion hook,
// starts a transmit, and spins on a flag that only its SCC interrupt handler
// clears. A chip that never interrupts leaves that flag set forever, and the
// startup thread stops at ~5% of the Welcome bar with the cursor still alive.
//
// What has to be real for that driver:
//  - the four ports: ctl B +0, ctl A +2, data B +4, data A +6 (the stub's
//    ctl/data split on bit 1 sent channel-B DATA into the register file);
//  - the chip-wide register pointer with WR0's command field (a command such
//    as Reset-Tx-Int carries pointer bits 2-0 only -- taking the low NIBBLE
//    turns command $28 into "select WR8");
//  - a transmitter with byte pacing: one byte in shift for ~35 us (8 bits at
//    LocalTalk's 230.4 kbit/s), a one-deep buffer, and a Tx-buffer-empty
//    interrupt when the buffer can take the next byte -- instant completion
//    inverts driver bookkeeping the same way instant SCSI timeouts did;
//  - the Tx underrun/EOM latch: WR0 $C0 arms it, and when the transmitter
//    then drains the chip appends CRC+flag, sets RR0 bit 6 and raises an
//    External/Status interrupt. That interrupt IS the frame-done signal the
//    driver's completion path runs on;
//  - RR3 (channel A) pending bits and RR2 (channel B) as the MODIFIED vector,
//    because the ROM's level-4 glue dispatches through it.
//
// Empty-wire receive: hunt stays set, nothing ever arrives, RR10 reports
// missing clocks -- LocalTalk reads that as a free line, transmits, and its
// probes go unanswered, which is exactly how a node address is acquired on a
// cable with no one else on it.
//
// Interrupt line: IPL 4 through the machine's encoder (SCC > VIA2 > VIA1,
// the IOSB priority order).
//
// Reference: Z8530/Z85C30 register and bit layout from the register map on
// the reference shelf; interrupt priority from the IOSB notes. Clean-room
// behavioral reimplementation.

#include "openmac/types.hpp"

#include <cstdio>
#include <functional>

namespace openmac {

class Scc8530 {
public:
    std::function<void(const char*)> onDiag;

    void reset() {
        ptr_ = 0;
        wr2_ = 0;
        wr9_ = 0;
        chanReset(ch_[0]);
        chanReset(ch_[1]);
    }

    // off = port offset & 6: 0 = ctl B, 2 = ctl A, 4 = data B, 6 = data A.
    u8 read(u32 off) {
        Chan& c = ch_[(off >> 1) & 1];
        if (off & 4) return 0;             // data: nothing ever arrives
        const int reg = ptr_;
        ptr_ = 0;
        switch (reg) {
        case 0:  return rr0(c);
        case 1:  return 0x07;              // all sent + SDLC residue 011
        case 2:
            // Channel A returns WR2 raw; channel B returns the vector
            // modified by the highest pending source -- the ROM's level-4
            // dispatch reads this to pick a handler.
            return (&c == &ch_[1]) ? wr2_ : modifiedVector();
        case 3:  return (&c == &ch_[1]) ? pendingBits() : 0;
        case 10: return c.rr10;
        case 12: return c.wr[12];
        case 13: return c.wr[13];
        case 15: return c.wr[15];
        default: return c.wr[reg & 15];
        }
    }

    void write(u32 off, u8 v) {
        Chan& c = ch_[(off >> 1) & 1];
        if (off & 4) {                     // data port: transmit a byte
            txByte(c);
            return;
        }
        if (ptr_ == 0) {
            command(c, v);
            return;
        }
        const int reg = ptr_;
        ptr_ = 0;
        switch (reg) {
        case 2: wr2_ = v; break;
        case 8: txByte(c); break;
        case 9:
            // Shared master interrupt control. The reset field acts first.
            switch (v & 0xC0) {
            case 0x40: chanReset(ch_[0]); break;
            case 0x80: chanReset(ch_[1]); break;
            case 0xC0: chanReset(ch_[0]); chanReset(ch_[1]); wr2_ = 0; break;
            default: break;
            }
            wr9_ = v & 0x3F;
            break;
        case 14:
            // WR14's top three bits are DPLL commands. Search-mode entry,
            // reset-missing-clock and disable all clear the two clock-missing
            // latches RR10 reports. LocalTalk issues "reset missing clock"
            // immediately before every transmit and then tests RR10 bit 7:
            // a chip that answers "clock missing" there refuses to send, and
            // AppleTalk's node acquisition retries forever without ever
            // putting a frame on the wire.
            switch (v & 0xE0) {
            case 0x20: case 0x40: case 0x60:
                c.rr10 &= static_cast<u8>(~0xC0);
                break;
            default:
                break;
            }
            c.wr[14] = v;
            break;
        default:
            c.wr[reg & 15] = v;
            if (regBudget_ > 0 && onDiag) {
                --regBudget_;
                char b[64];
                std::snprintf(b, sizeof b, "SCC %c wr%d=%02X",
                              (&c == &ch_[1]) ? 'A' : 'B', reg, v);
                onDiag(b);
            }
            break;
        }
    }

    void tick(s32 cycles) {
        tickChan(ch_[0], cycles);
        tickChan(ch_[1], cycles);
    }

    bool irqAsserted() const {
        if (!(wr9_ & 0x08)) return false;  // master interrupt enable
        for (const Chan& c : ch_)
            if (c.extIp || c.txIp || c.rxIp) return true;
        return false;
    }

private:
    // One byte on the wire at LocalTalk speed: 8 bits at 230.4 kbit/s is
    // ~34.7 us, ~1150 CPU cycles at 33 MHz. The pacing matters more than the
    // exact figure: the driver expects to feed the frame byte by byte.
    static constexpr s32 kByteCycles = 1150;
    static constexpr s32 kFeedDelay = 96;  // buffer-empty interrupt latency

    struct Chan {
        u8 wr[16]{};
        bool txEmpty = true;       // RR0 bit 2: buffer can take a byte
        bool underrun = true;      // RR0 bit 6 latch; reset by WR0 $C0
        bool underrunArmed = false;
        bool bufFull = false;      // one-deep transmit buffer behind shift
        s32 shiftTimer = 0;        // byte currently on the wire
        s32 feedTimer = 0;         // pending Tx-buffer-empty interrupt
        bool extIp = false, txIp = false, rxIp = false;
        // RR10. The clock-missing latches (bits 7/6) report that the DPLL
        // lost transitions while recovering a clock from an incoming
        // signal. Nothing ever arrives on an unconnected port, so there is
        // no signal to lose a clock on and they stay clear once reset --
        // which is what lets a Mac with nothing plugged in still finish
        // AppleTalk startup, exactly as the real machine does.
        u8 rr10 = 0;
        u32 bytesSent = 0;
    };

    void chanReset(Chan& c) {
        for (u8& r : c.wr) r = 0;
        c.wr[15] = 0xF8;           // ext/status sources enabled by default
        c.txEmpty = true;
        c.underrun = true;
        c.underrunArmed = false;
        c.bufFull = false;
        c.shiftTimer = c.feedTimer = 0;
        c.extIp = c.txIp = c.rxIp = false;
        c.rr10 = 0;
    }

    u8 rr0(const Chan& c) const {
        u8 v = 0x10;                       // hunting: nothing to sync on
        if (c.txEmpty) v |= 0x04;
        if (c.underrun) v |= 0x40;
        return v;
    }

    // RR3, read through channel A: B ext/Tx/Rx are bits 0-2, A's are 3-5.
    u8 pendingBits() const {
        u8 v = 0;
        if (ch_[0].extIp) v |= 0x01;
        if (ch_[0].txIp) v |= 0x02;
        if (ch_[0].rxIp) v |= 0x04;
        if (ch_[1].extIp) v |= 0x08;
        if (ch_[1].txIp) v |= 0x10;
        if (ch_[1].rxIp) v |= 0x20;
        return v;
    }

    // Highest-priority pending source, encoded for RR2 read on channel B.
    // Priority: A Rx > A Tx > A ext > B Rx > B Tx > B ext; "none" reads as
    // 011 (channel B special) per the datasheet.
    u8 modifiedVector() const {
        u8 code = 0x3;
        if (ch_[1].rxIp) code = 0x6;
        else if (ch_[1].txIp) code = 0x4;
        else if (ch_[1].extIp) code = 0x5;
        else if (ch_[0].rxIp) code = 0x2;
        else if (ch_[0].txIp) code = 0x0;
        else if (ch_[0].extIp) code = 0x1;
        if (wr9_ & 0x10) {                 // status high: reversed, V6-V4
            const u8 r = static_cast<u8>(((code & 1) << 2) | (code & 2) |
                                         ((code & 4) >> 2));
            return static_cast<u8>((wr2_ & 0x8F) | (r << 4));
        }
        return static_cast<u8>((wr2_ & 0xF1) | (code << 1));
    }

    void command(Chan& c, u8 v) {
        int reg = v & 7;
        const int cmd = (v >> 3) & 7;
        if (cmd == 1) reg += 8;            // point high
        switch (cmd) {
        case 2:                            // reset ext/status interrupts
            c.extIp = false;
            break;
        case 5:                            // reset Tx int pending
            c.txIp = false;
            break;
        case 6:                            // error reset
            c.rxIp = false;
            break;
        default:
            break;
        }
        switch ((v >> 6) & 3) {
        case 3:                            // reset Tx underrun/EOM latch
            // Arms the end-of-frame report: when the transmitter next runs
            // dry, CRC+flag go out, the latch sets, and ext/status fires.
            c.underrun = false;
            c.underrunArmed = true;
            break;
        default:
            break;
        }
        ptr_ = reg;
        if ((cmd > 1 || (v & 0xC0)) && cmdBudget_ > 0 && onDiag) {
            --cmdBudget_;
            char b[64];
            std::snprintf(b, sizeof b, "SCC %c cmd=%02X",
                          (&c == &ch_[1]) ? 'A' : 'B', v);
            onDiag(b);
        }
    }

    void txByte(Chan& c) {
        ++c.bytesSent;
        if (c.shiftTimer > 0) {
            c.bufFull = true;              // queued behind the byte in shift
            c.txEmpty = false;
        } else {
            c.shiftTimer = kByteCycles;    // straight into the shift register
            c.txEmpty = true;
            c.feedTimer = kFeedDelay;      // "buffer free" fires almost at once
        }
        if (dataBudget_ > 0 && onDiag) {
            --dataBudget_;
            char b[64];
            std::snprintf(b, sizeof b, "SCC %c tx #%u",
                          (&c == &ch_[1]) ? 'A' : 'B', c.bytesSent);
            onDiag(b);
        }
    }

    void tickChan(Chan& c, s32 cycles) {
        if (c.feedTimer > 0) {
            c.feedTimer -= cycles;
            if (c.feedTimer <= 0) {
                c.feedTimer = 0;
                if (c.wr[1] & 0x02) c.txIp = true;   // Tx int enable
            }
        }
        if (c.shiftTimer > 0) {
            c.shiftTimer -= cycles;
            if (c.shiftTimer <= 0) {
                c.shiftTimer = 0;
                if (c.bufFull) {
                    c.bufFull = false;
                    c.txEmpty = true;
                    c.shiftTimer = kByteCycles;
                    c.feedTimer = kFeedDelay;
                } else if (c.underrunArmed) {
                    // Drained mid-frame: the chip closes the frame itself.
                    // The latch sets and ext/status reports end-of-message.
                    c.underrunArmed = false;
                    c.underrun = true;
                    if ((c.wr[15] & 0x40) && (c.wr[1] & 0x01)) c.extIp = true;
                    if (irqBudget_ > 0 && onDiag) {
                        --irqBudget_;
                        char b[64];
                        std::snprintf(b, sizeof b, "SCC %c EOM after %u bytes",
                                      (&c == &ch_[1]) ? 'A' : 'B', c.bytesSent);
                        onDiag(b);
                    }
                }
            }
        }
    }

    Chan ch_[2];                           // [0] = B, [1] = A
    int ptr_ = 0;
    u8 wr2_ = 0;
    u8 wr9_ = 0;
    int cmdBudget_ = 400;
    int regBudget_ = 400;
    int dataBudget_ = 200;
    int irqBudget_ = 60;
};

} // namespace openmac
