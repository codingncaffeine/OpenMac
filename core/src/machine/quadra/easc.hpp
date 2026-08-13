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
// IOSB also bit-bangs the separate DFAC chip (latch/data/clock on VIA2
// port B bits 0/3/4, LSB-first). On this board the DFAC handles the
// input/record side; the EASC's own output feeds the speaker directly, so
// the DFAC settings byte is tracked but does not gate playback (the ROM
// chimes with the DFAC output-mix bit clear).
//
// Reference: Quadra 650 hardware dossier (EASC/DFAC sections). Clean-room.

#include "openmac/types.hpp"

#include <cstdio>
#include <functional>

namespace openmac {

class IifxStateCodec;

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
        // FIFO interrupts start gated off (inverted-sense enables at $09 and
        // $29); the ROM leaves them off because it polls, and the Sound
        // Manager opens channel B's when it wants callbacks.
        ext_[0x09] = 1;
        ext_[0x29] = 1;
        linePending_ = false;
        // CD-XA ADPCM coefficient tables power up with the documented
        // defaults (four (K1,K0) filter pairs); the chime relies on them.
        static constexpr u8 kXaDefault[8] = {0x00, 0x00, 0x00, 0x3C,
                                             0xCC, 0x73, 0xC9, 0x62};
        for (int i = 0; i < 8; ++i) {
            ext_[0x10 + i] = kXaDefault[i];
            ext_[0x30 + i] = kXaDefault[i];
        }
        for (int ch = 0; ch < 2; ++ch) {
            xaParam_[ch] = 0;
            xaPos_[ch] = 0;
            xaSubpos_[ch] = 0;
            xaByte_[ch] = 0;
            xaS0_[ch] = 0;
            xaS1_[ch] = 0;
        }
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
            updateLine();
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
            if (++pushesA_ == 1) diag("EASC first A push (mode=%02X dfac=%02X)", mode_, dfacSettings_);
            push(headA_, tailA_, bufA_, v);
            if (fifoLevelA() <= kFifo / 2) latch(0x01);
            return;
        }
        if (offset < 0x800) {
            if (++pushesB_ == 1) diag("EASC first B push (mode=%02X dfac=%02X)", mode_, dfacSettings_);
            push(headB_, tailB_, bufB_, v);
            if (fifoLevelB() <= kFifo / 2) latch(0x04);
            return;
        }
        switch (offset) {
        case 0x801:
            diag("EASC mode<-%02X (pushes A=%u)", v, pushesA_);
            mode_ = v;
            if (v == 1) {   // entering FIFO mode: room is announced
                if (fifoLevelA() <= kFifo / 2) latch(0x01);
                if (fifoLevelB() <= kFifo / 2) latch(0x04);
            }
            updateLine();   // leaving FIFO mode drops the line
            break;
        case 0x802: control_ = v; break;
        case 0x803:
            fifoMode_ = v;
            if (v & 0x80) {             // clear FIFOs
                headA_ = tailA_ = headB_ = tailB_ = 0;
            }
            break;
        case 0x806: diag("EASC vol<-%02X (%u)", v, 0); volume_ = v; break;
        default:
            if (offset >= 0xF00 && offset < 0xF40) {
                const u32 r = offset - 0xF00;
                ext_[r] = v;
                // Enabling an interrupt is itself an event: the condition it
                // asks about may already be latched.
                if (r == 0x09 || r == 0x29) updateLine();
            }
            break;
        }
    }

    // ---- DFAC 3-wire link (VIA2 PB0 = latch, PB3 = data, PB4 = clock) ----
    // The chip caches the last 8 bits seen on ConfigData at each ConfigClk
    // rising edge -- LSB-first, each new bit lands at bit 7 and walks down --
    // and a ConfigLE rising edge commits that byte. Within one port write
    // the latch edge acts before the clock edge shifts the new data bit
    // (IOSB drives latch, data, clock in that order).
    void dfacLines(bool latch, bool data, bool clock) {
        if (latch && !dfacPrevLatch_) {
            dfacSettings_ = dfacShift_;
            diag("DFAC settings<-%02X (pushes A=%u)", dfacSettings_, pushesA_);
        }
        if (clock && !dfacPrevClock_) {
            dfacShift_ = static_cast<u8>((dfacShift_ >> 1) | (data ? 0x80 : 0));
        }
        dfacPrevClock_ = clock;
        dfacPrevLatch_ = latch;
    }

    // ---- output ----
    // One 8-bit unsigned sample per pull, mixing both FIFO channels; the
    // machine pulls at the classic 22.25kHz rate. On this board the EASC
    // output feeds the speaker directly -- the DFAC sits on the input/gain
    // side, so its settings byte does NOT gate playback (the ROM chimes
    // with the DFAC's output mix bit clear).
    u8 pullSample() {
        const u32 beforeA = fifoLevelA(), beforeB = fifoLevelB();
        int a = 0, b = 0;   // signed 16-bit sample space
        if (mode_ != 1) {
            pop(headA_, tailA_);        // FIFOs drain even while inaudible
            pop(headB_, tailB_);
        } else {
            a = channelSample(0);
            b = channelSample(1);
        }
        crossings(beforeA, beforeB);
        if (mode_ != 1) return 0x80;
        const int mixed = (a + b) / 2;
        const int vol = (volume_ >> 5) & 7;
        const int scaled = (mixed * (vol + 1)) / 8;
        int out = 0x80 + (scaled >> 8);
        if (out < 0) out = 0;
        if (out > 255) out = 255;
        return static_cast<u8>(out);
    }

    u32 fifoLevelA() const { return (tailA_ - headA_) & (kFifo - 1); }
    u32 fifoLevelB() const { return (tailB_ - headB_) & (kFifo - 1); }

    std::function<void(bool level)> onIrq;
    std::function<void(const char* msg)> onDiag;

    // Read-only snapshot for a diagnostic capture. Deliberately does NOT go
    // through read(): reading $804 clears the latch and drops the interrupt
    // line, so an observer that used it would change the thing it reports.
    void debugState(char* out, std::size_t cap) const {
        std::snprintf(out, cap,
                      "mode=%02X vol=%02X irqStatus=%02X line=%d "
                      "enA=%02X(%s) enB=%02X(%s) ctrlA=%02X ctrlB=%02X "
                      "fifoA=%u fifoB=%u pushedA=%u pushedB=%u",
                      mode_, volume_, irqStatus_, linePending_ ? 1 : 0,
                      ext_[0x09], (ext_[0x09] & 1) ? "off" : "ON",
                      ext_[0x29], (ext_[0x29] & 1) ? "off" : "ON",
                      ext_[0x08], ext_[0x28],
                      static_cast<unsigned>(fifoLevelA()),
                      static_cast<unsigned>(fifoLevelB()), pushesA_, pushesB_);
    }

private:
    friend class IifxStateCodec;

    static constexpr u32 kFifo = 1024;

    void diag(const char* fmt, u32 a, u32 b) {
        if (!onDiag || diagBudget_ <= 0) return;
        --diagBudget_;
        char line[96];
        std::snprintf(line, sizeof(line), fmt, a, b);
        onDiag(line);
    }

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

    // Pop one byte from a channel's FIFO; repeats the last byte when empty
    // (the decoder keeps its cadence across a momentary underrun).
    u8 popByte(int ch) {
        if (ch == 0) { pop(headA_, tailA_); return lastA_; }
        pop(headB_, tailB_);
        return lastB_;
    }

    // One output sample for a channel, honoring the FIFO control's CD-XA
    // mode bits ($F08/$F28 bits 1-0): 0 = linear signed PCM, 1 = 8-bit
    // ADPCM, 2 = 4-bit ADPCM (the boot chime), 3 = 2-bit ADPCM. Returns a
    // signed 16-bit sample.
    int channelSample(int ch) {
        const u8 ctrl = ext_[ch ? 0x28 : 0x08];
        const int mode = ctrl & 3;
        if (mode == 0) {
            const bool had = ch == 0 ? pop(headA_, tailA_) : pop(headB_, tailB_);
            if (!had) return 0;
            const u8 raw = ch == 0 ? lastA_ : lastB_;
            return static_cast<int>(static_cast<s8>(raw ^ 0x80)) << 8;
        }
        return decodeXa(ch, mode);
    }

    // CD-XA ADPCM: 28-sample blocks led by a parameter byte (filter in
    // bits 5:4, right-shift range in bits 3:0), packed 8/4/2-bit residuals,
    // and a two-tap predictor whose (K1,K0) pairs live in the coefficient
    // registers ($F10/$F30). K0 scales the previous sample, K1 the one
    // before it, in 1/64 steps.
    int decodeXa(int ch, int mode) {
        if (xaPos_[ch] == 0) {
            xaParam_[ch] = popByte(ch);
            xaSubpos_[ch] = 0;
        }
        const int base = ch ? 0x30 : 0x10;
        const int filter = (xaParam_[ch] >> 4) & 3;
        int shift = xaParam_[ch] & 0xF;
        if (shift > 12) shift = 12;
        const int k0 = static_cast<s8>(ext_[base + filter * 2 + 1]);
        const int k1 = static_cast<s8>(ext_[base + filter * 2]);

        int raw = 0;
        switch (mode) {
        case 1:
            raw = static_cast<s16>(static_cast<u16>(popByte(ch)) << 8);
            break;
        case 2:
            if (xaSubpos_[ch] == 0) {
                xaByte_[ch] = popByte(ch);
                raw = static_cast<s16>(static_cast<u16>(xaByte_[ch] & 0xF) << 12);
                xaSubpos_[ch] = 1;
            } else {
                raw = static_cast<s16>(static_cast<u16>((xaByte_[ch] >> 4) & 0xF) << 12);
                xaSubpos_[ch] = 0;
            }
            break;
        default:   // mode 3
            if (xaSubpos_[ch] == 0) xaByte_[ch] = popByte(ch);
            raw = static_cast<s16>(
                static_cast<u16>((xaByte_[ch] >> (xaSubpos_[ch] * 2)) & 0x3) << 14);
            xaSubpos_[ch] = (xaSubpos_[ch] + 1) & 3;
            break;
        }

        int sample = (raw >> shift) + ((k0 * xaS0_[ch] + k1 * xaS1_[ch] + 32) >> 6);
        if (sample > 32767) sample = 32767;
        if (sample < -32768) sample = -32768;
        xaS1_[ch] = xaS0_[ch];
        xaS0_[ch] = sample;
        if (++xaPos_[ch] >= 28) xaPos_[ch] = 0;
        return sample;
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
        irqStatus_ |= bits;
        updateLine();
    }

    // The per-channel FIFO interrupt enables, inverted sense: 0 = enabled.
    // Channel A's is extended register $09, channel B's is $29 -- the ROM
    // writes 1 to both at init because it drives the chime by POLLING $804,
    // and System 7's Sound Manager writes 0 to $29 when it wants to be
    // called back for more samples. Nothing ever writes $0B/$2B.
    u8 enabledMask() const {
        u8 m = 0;
        if (!(ext_[0x09] & 1)) m |= 0x03;
        if (!(ext_[0x29] & 1)) m |= 0x0C;
        return m;
    }

    // The interrupt output is a LEVEL, not an edge: it is asserted for as
    // long as an enabled condition is latched. Enabling an interrupt whose
    // condition is ALREADY pending must therefore raise the line -- that is
    // exactly what the Sound Manager does (it sets FIFO mode on an empty
    // FIFO, so "room available" is true before it enables the interrupt),
    // and treating the raise as an edge on the status bit left it waiting
    // for a callback that could never come.
    void updateLine() {
        const bool pending = mode_ == 1 && (irqStatus_ & enabledMask()) != 0;
        if (pending == linePending_) return;
        linePending_ = pending;
        if (onIrq) onIrq(pending);
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
    u32 pushesA_ = 0, pushesB_ = 0;
    int diagBudget_ = 48;
    bool linePending_ = false;

    // CD-XA decoder state, per channel.
    u8 xaParam_[2]{};
    int xaPos_[2]{};
    int xaSubpos_[2]{};
    u8 xaByte_[2]{};
    int xaS0_[2]{};
    int xaS1_[2]{};
};

} // namespace openmac
