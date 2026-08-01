#pragma once

// EASC ("Batman") audio as wired through IOSB on the Quadra 610/650/800,
// plus the DFAC output stage it feeds. EASC is the ASC family with the
// wavetable mode deleted: two 1KB sample FIFOs (channel A at window
// $000-$3FF, B at $400-$7FF) and a control block at $800. The boot chime is
// raw 8-bit samples the ROM pushes into the FIFOs.
//
//   $800 version (EASC reads $B0)     $804 FIFO IRQ status
//   $801 mode (1 = FIFO)              $806 volume
//   $802 control                      $807 clock rate (EASC: fixed 3)
//   $803 FIFO mode                    $F00+ EASC extended block (stored)
//
// The FIFO output is NOT the analog output: IOSB bit-bangs a separate DFAC
// chip (latch/data/clock on VIA2 port B bits 0/3/4), and DFAC's settings
// byte bit 1 gates whether anything is audible at all -- it resets to
// silent, so the chime only sounds once the ROM programs DFAC.
//
// Reference: Quadra 650 hardware dossier (EASC/DFAC sections). Clean-room.

#include "openmac/types.hpp"

#include <functional>

namespace openmac {

class Easc {
public:
    void reset() {
        headA_ = tailA_ = headB_ = tailB_ = 0;
        mode_ = 0;
        control_ = 0;
        fifoMode_ = 0;
        irqStatus_ = 0;
        volume_ = 0;
        for (auto& r : ext_) r = 0;
        // FIFO interrupts start gated off (inverted-sense enables); the ROM
        // opens them only for its interrupt-driven player phase.
        ext_[0x0B] = 1;
        ext_[0x2B] = 1;
        // DFAC state survives reset on real hardware only insofar as the
        // chip's own reset line is separate; model the documented power-on
        // default (settings byte 0 = silent).
        dfacSettings_ = 0;
        dfacShift_ = 0;
        dfacPrevClock_ = false;
        dfacPrevLatch_ = false;
    }

    // ---- CPU window ($50014000 + offset, 0x1000 wide) ----
    u8 read(u32 offset) {
        offset &= 0xFFF;
        if (offset < 0x800) return 0;   // FIFO windows read as empty
        switch (offset) {
        case 0x800: return 0xB0;        // EASC version
        case 0x801: return mode_;
        case 0x802: return control_;
        case 0x803: return fifoMode_;
        case 0x804: {
            // The value a reader sees is the live conditions OR'd with any
            // latched events -- the ISR that answers a stale VIA2 edge must
            // still find its cause here. The read clears the latch and
            // drops the interrupt LINE; only a new event (a drain crossing,
            // or a write/mode change that leaves room) raises it again, so
            // an idle empty FIFO never storms the slot interrupt.
            const u8 v = static_cast<u8>(liveStatus() | irqStatus_);
            irqStatus_ = 0;
            if (onIrq) onIrq(false);
            return v;
        }
        case 0x806: return volume_;
        case 0x807: return 3;           // clock rate: hardwired 44.1kHz
        default:
            if (offset >= 0xF00 && offset < 0xF40) return ext_[offset - 0xF00];
            return 0;
        }
    }

    void write(u32 offset, u8 v) {
        offset &= 0xFFF;
        if (offset < 0x400) {
            push(headA_, tailA_, bufA_, v);
            if (fifoLevelA() <= kFifo / 2) latch(0x01);
            return;
        }
        if (offset < 0x800) {
            push(headB_, tailB_, bufB_, v);
            if (fifoLevelB() <= kFifo / 2) latch(0x04);
            return;
        }
        switch (offset) {
        case 0x801:
            mode_ = v;
            if (v == 1) {   // entering FIFO mode: room is announced
                if (fifoLevelA() <= kFifo / 2) latch(0x01);
                if (fifoLevelB() <= kFifo / 2) latch(0x04);
            }
            break;
        case 0x802: control_ = v; break;
        case 0x803:
            fifoMode_ = v;
            if (v & 0x80) {             // clear FIFOs
                headA_ = tailA_ = headB_ = tailB_ = 0;
            }
            break;
        case 0x806: volume_ = v; break;
        default:
            if (offset >= 0xF00 && offset < 0xF40) ext_[offset - 0xF00] = v;
            break;
        }
    }

    // ---- DFAC 3-wire link (VIA2 PB0 = latch, PB3 = data, PB4 = clock) ----
    void dfacLines(bool latch, bool data, bool clock) {
        if (clock && !dfacPrevClock_) {
            dfacShift_ = static_cast<u8>((dfacShift_ << 1) | (data ? 1 : 0));
        }
        if (latch && !dfacPrevLatch_) {
            dfacSettings_ = dfacShift_;
        }
        dfacPrevClock_ = clock;
        dfacPrevLatch_ = latch;
    }

    // ---- output ----
    // One 8-bit unsigned sample per pull, mixing both FIFO channels; the
    // machine pulls at the classic 22.25kHz rate. Silence when the DFAC
    // output mix is off or the EASC isn't in FIFO mode.
    u8 pullSample() {
        const u32 beforeA = fifoLevelA(), beforeB = fifoLevelB();
        u8 a = 0x80, b = 0x80;
        if (mode_ != 1 || !(dfacSettings_ & 0x02)) {
            pop(headA_, tailA_);        // FIFOs drain even while inaudible
            pop(headB_, tailB_);
        } else {
            a = pop(headA_, tailA_) ? lastA_ : 0x80;
            b = pop(headB_, tailB_) ? lastB_ : 0x80;
        }
        crossings(beforeA, beforeB);
        if (mode_ != 1 || !(dfacSettings_ & 0x02)) return 0x80;
        // DFAC volume: 3-bit attenuation from the settings byte's top bits
        // is modeled coarsely through the EASC volume register instead.
        const int mixed = (static_cast<int>(a) + b) / 2;
        const int vol = (volume_ >> 5) & 7;
        return static_cast<u8>(0x80 + ((mixed - 0x80) * (vol + 1)) / 8);
    }

    u32 fifoLevelA() const { return (tailA_ - headA_) & (kFifo - 1); }
    u32 fifoLevelB() const { return (tailB_ - headB_) & (kFifo - 1); }

    std::function<void(bool level)> onIrq;

private:
    static constexpr u32 kFifo = 1024;

    void push(u32& head, u32& tail, u8* buf, u8 v) {
        const u32 next = (tail + 1) & (kFifo - 1);
        if (next != head) {
            buf[tail] = v;
            tail = next;
        }
    }

    bool pop(u32& head, u32& tail) {
        if (head == tail) return false;
        if (&head == &headA_) lastA_ = bufA_[head];
        else                  lastB_ = bufB_[head];
        head = (head + 1) & (kFifo - 1);
        return true;
    }

    // Live conditions: bit0/bit2 = the A/B FIFO has room (at or below
    // half, including empty -- what lets the first fill burst start),
    // bit1/bit3 = the FIFO has run dry (the ROM parks on bit 3 after
    // loading the chime to wait for it to finish sounding).
    u8 liveStatus() const {
        u8 st = 0;
        if (fifoLevelA() <= kFifo / 2) st |= 0x01;
        if (fifoLevelA() == 0)         st |= 0x02;
        if (fifoLevelB() <= kFifo / 2) st |= 0x04;
        if (fifoLevelB() == 0)         st |= 0x08;
        return st;
    }

    void latch(u8 bits) {
        // The extended block's per-channel FIFO IRQ enables gate the LINE
        // (inverted sense: 0 = enabled). The ROM polls with them off and
        // turns them on only for the interrupt-driven player -- without the
        // gate a stale VIA2 edge from the polled phase fires reentrantly
        // the moment the IER opens.
        u8 lineBits = bits;
        if (ext_[0x0B] & 1) lineBits &= static_cast<u8>(~0x03);
        if (ext_[0x2B] & 1) lineBits &= static_cast<u8>(~0x0C);
        if ((irqStatus_ | bits) != irqStatus_) {
            irqStatus_ |= bits;
            if (lineBits && mode_ == 1 && onIrq) onIrq(true);
        }
    }

    // Drain crossings latch and edge the line; a FIFO simply sitting empty
    // never does.
    void crossings(u32 beforeA, u32 beforeB) {
        u8 ev = 0;
        if (beforeA > kFifo / 2 && fifoLevelA() <= kFifo / 2) ev |= 0x01;
        if (beforeA > 0 && fifoLevelA() == 0)                 ev |= 0x02;
        if (beforeB > kFifo / 2 && fifoLevelB() <= kFifo / 2) ev |= 0x04;
        if (beforeB > 0 && fifoLevelB() == 0)                 ev |= 0x08;
        if (ev) latch(ev);
    }

    u8 bufA_[kFifo]{}, bufB_[kFifo]{};
    u32 headA_ = 0, tailA_ = 0, headB_ = 0, tailB_ = 0;
    u8 lastA_ = 0x80, lastB_ = 0x80;
    u8 mode_ = 0, control_ = 0, fifoMode_ = 0, irqStatus_ = 0, volume_ = 0;
    u8 ext_[0x40]{};

    u8 dfacSettings_ = 0, dfacShift_ = 0;
    bool dfacPrevClock_ = false, dfacPrevLatch_ = false;
};

} // namespace openmac
