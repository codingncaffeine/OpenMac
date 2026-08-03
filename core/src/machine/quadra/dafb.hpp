#pragma once

// DAFB video as integrated into djMEMC on the Quadra 610/650/800 ("MEMC"
// variant, version 3, Antelope CLUT/DAC, 1MB VRAM). Register window is 1KB at
// $F9800000 in four 256-byte blocks: DAFB control, swatch timing generator,
// RAMDAC, clock generator. VRAM is a 2MB window at $F9000000 with 1MB
// populated (mirrored). No hardware cursor, no convolution (NTSC/PAL modes
// are documented non-functional on this variant).
//
//   +$00  framebuffer base bits 20-9      +$100 swatch mode
//   +$04  framebuffer base bits 8-5       +$104 IRQ enables (bit0 = VBL)
//   +$08  stride in 32-bit words          +$108 IRQ status (read)
//   +$0C  timing control                  +$114 clear VBL interrupt
//   +$10  configuration                   +$148 HPIX   +$14C VHLINE
//   +$1C  monitor sense (read = inverse)  +$158 VBP    +$15C VAL
//   +$2C  test/version (bits 15-9 = 3)
//   +$200 CLUT address  +$210 CLUT data (R,G,B bytes)  +$220 depth control
//   +$300 DP8534 clock generator (serial-shift protocol; stored, not decoded)
//
// Reference: Quadra 650 hardware dossier (DAFB/MEMC sections), Apple
// Developer Notes for the Centris 650/Quadra 800 family. Clean-room.

#include "openmac/types.hpp"

#include <cstdio>
#include <functional>
#include <vector>

namespace openmac {

class Dafb {
public:
    Dafb() : vram_(kVramSize, 0) {
        reset();
        // A neutral ramp so anything drawn before the ROM loads the CLUT is
        // visible rather than black-on-black.
        for (int i = 0; i < 256; ++i) {
            const u8 v = static_cast<u8>(255 - i);
            clut_[i] = 0xFF000000u | (static_cast<u32>(v) << 16) |
                       (static_cast<u32>(v) << 8) | v;
        }
    }

    void reset() {
        for (auto& r : regs_) r = 0;
        for (auto& r : swatch_) r = 0;
        clutAddr_ = 0;
        clutPhase_ = 0;
        depthCtl_ = 0;
        vblEnabled_ = false;
        vblPending_ = false;
        cursorPending_ = false;
        intMask_ = 0;
        clockShift_ = 0;
    }

    // ---- VRAM (the machine routes $F9000000+ here; 1MB mirrored) ----
    static constexpr u32 kVramSize = 0x100000;
    u8   vramRead8(u32 off) const { return vram_[off & (kVramSize - 1)]; }
    void vramWrite8(u32 off, u8 v) { vram_[off & (kVramSize - 1)] = v; }
    u32 vramRead32(u32 off) const {
        off &= kVramSize - 1;
        return (static_cast<u32>(vram_[off]) << 24) |
               (static_cast<u32>(vram_[(off + 1) & (kVramSize - 1)]) << 16) |
               (static_cast<u32>(vram_[(off + 2) & (kVramSize - 1)]) << 8) |
               vram_[(off + 3) & (kVramSize - 1)];
    }
    void vramWrite32(u32 off, u32 v) {
        off &= kVramSize - 1;
        vram_[off] = static_cast<u8>(v >> 24);
        vram_[(off + 1) & (kVramSize - 1)] = static_cast<u8>(v >> 16);
        vram_[(off + 2) & (kVramSize - 1)] = static_cast<u8>(v >> 8);
        vram_[(off + 3) & (kVramSize - 1)] = static_cast<u8>(v);
    }

    // ---- register window ($F9800000 + offset, offset < 0x400) ----
    u32 read(u32 offset) {
        offset &= 0x3FF;
        const u32 block = offset >> 8;
        const u32 reg = offset & 0xFF;
        switch (block) {
        case 0:
            if (reg == 0x1C) {
                const u32 v = senseRead();
                if (senseDiag_ > 0 && onDiag) {
                    --senseDiag_;
                    char b[80];
                    std::snprintf(b, sizeof b, "SENSE read drive=%X -> %X",
                                  regs_[0x1C >> 2] & 7u, v);
                    onDiag(b);
                }
                return v;
            }
            if (reg == 0x2C) return (3u << 9) | (regs_[0x2C >> 2] & 0x1FFu);
            return regs_[reg >> 2];
        case 1:
            if (reg == 0x08) {   // IRQ status: bit0 = VBL, bit2 = cursor line
                return (vblPending_ ? 1u : 0u) | (cursorPending_ ? 4u : 0u);
            }
            // The interrupt CLEARS work on a read as well as a write (MAME
            // dafb.cpp swatch_r) -- the ISR's TST of the register is its ack.
            // Leaving read-clear out sticks the pending bit, and the level-
            // routed slot interrupt then storms the moment the mask enables.
            if (reg == 0x0C) { cursorPending_ = false; updateIrq(); return 0; }
            if (reg == 0x14) { vblPending_ = false; updateIrq(); return 0; }
            return swatch_[reg >> 2] & 0xFFFu;
        case 2:
            if (reg == 0x00) return clutAddr_;
            if (reg == 0x10) {   // CLUT data readback, same 3-byte cycle
                const u32 rgb = clut_[clutAddr_ & 0xFF];
                const u32 v = (rgb >> (16 - clutPhase_ * 8)) & 0xFF;
                if (++clutPhase_ == 3) {
                    clutPhase_ = 0;
                    clutAddr_ = (clutAddr_ + 1) & 0xFF;
                }
                return v;
            }
            if (reg == 0x20) return depthCtl_;
            return 0;
        default:
            return 0;   // clock generator: nothing readable
        }
    }

    void write(u32 offset, u32 v) {
        offset &= 0x3FF;
        const u32 block = offset >> 8;
        const u32 reg = offset & 0xFF;
        switch (block) {
        case 0:
            regs_[reg >> 2] = v;
            if (reg == 0x1C && senseDiag_ > 0 && onDiag) {
                --senseDiag_;
                char b[80];
                std::snprintf(b, sizeof b, "SENSE write %X", v & 7u);
                onDiag(b);
            }
            break;
        case 1:
            swatch_[reg >> 2] = v & 0xFFFu;
            // Interrupt mask (+$104): bit 0 = vertical blank, bit 2 = the
            // beam crossing the cursor scanline. System 7.5's Cursor Device
            // Manager runs its cursor task off the CURSOR interrupt, not the
            // VBL -- it enables mask $004 and nothing else, so a model that
            // only raises the line for VBL leaves the cursor frozen while
            // the mouse deltas pile up unconsumed in the CDM record.
            // Disabling a source drops its pending bit (MAME dafb.cpp), so a
            // latch left over from the mask-off era cannot fire at enable.
            if (reg == 0x04) {
                intMask_ = v & 7u;
                vblEnabled_ = (v & 1) != 0;
                if (!(v & 1)) vblPending_ = false;
                if (!(v & 4)) cursorPending_ = false;
                updateIrq();
            }
            if (reg == 0x0C) { cursorPending_ = false; updateIrq(); }   // clear cursor int
            if (reg == 0x14) { vblPending_ = false; updateIrq(); }      // clear VBL int
            break;
        case 2:
            if (reg == 0x00) {
                clutAddr_ = v & 0xFF;
                clutPhase_ = 0;
            } else if (reg == 0x10) {
                if (clutDiag_ > 0 && onDiag) {
                    --clutDiag_;
                    char b[80];
                    std::snprintf(b, sizeof b, "CLUT[%u] phase %d <- %08X",
                                  clutAddr_, clutPhase_, v);
                    onDiag(b);
                }
                // Three successive byte writes: R, G, B; autoincrement after
                // the blue byte lands.
                const int shift = 16 - clutPhase_ * 8;
                u32& e = clut_[clutAddr_ & 0xFF];
                e = (e & ~(0xFFu << shift)) | ((v & 0xFF) << shift);
                e |= 0xFF000000u;
                if (++clutPhase_ == 3) {
                    clutPhase_ = 0;
                    clutAddr_ = (clutAddr_ + 1) & 0xFF;
                }
            } else if (reg == 0x20) {
                depthCtl_ = v & 0xFF;
            }
            break;
        default:
            // DP8534 serial shift: store the stream for the record; the
            // pixel clock itself is derived from the swatch timing.
            clockShift_ = (clockShift_ << 1) | (v & 1);
            break;
        }
    }

    // ---- scanout ----
    // Depth from the Antelope control register bits [4:2].
    //
    // Do NOT try to derive this from the row length instead: rows are padded,
    // and by more than a factor of two. The 21-inch display sits at 1 bit with
    // 1152 pixels in a 576-byte row -- 144 bytes of picture in four times the
    // space -- so "the largest depth that fits the row" reads it as 4 bits and
    // renders a quarter of the screen stretched across all of it.
    int bpp() const {
        if (forceBpp_) return forceBpp_;
        switch (depthCtl_ & 0x1C) {
        case 0x00: return 1;
        case 0x08: return 2;
        case 0x10: return 4;
        case 0x18: return 8;
        default:   return 24;
        }
    }
    u32 fbBase() const {
        return ((regs_[0] & 0xFFFu) << 9) | ((regs_[1] & 0xFu) << 5);
    }
    u32 strideBytes() const { return (regs_[2] & 0xFFFu) * 4; }

    // Visible geometry: the raster of the display that is plugged in. See
    // MonitorWiring -- an Apple fixed-frequency monitor runs one resolution,
    // and the swatch counters cannot be read as pixels across modes because
    // they count in a video clock that changes with the display.
    int width() const { return forceW_ ? forceW_ : monitor_.width; }
    int height() const { return forceH_ ? forceH_ : monitor_.height; }

    // Expand the framebuffer to ARGB8888. `out` holds width()*height().
    void render(u32* out) const {
        const int w = width(), h = height();
        const u32 base = fbBase();
        const u32 stride = strideBytes() ? strideBytes()
                                         : static_cast<u32>(w) * static_cast<u32>(bpp()) / 8;
        const int depth = bpp();
        for (int y = 0; y < h; ++y) {
            const u32 row = base + static_cast<u32>(y) * stride;
            u32* dst = out + static_cast<size_t>(y) * static_cast<size_t>(w);
            switch (depth) {
            case 1:
                // One bit is black on white, and the System says so by never
                // programming a palette for it -- a 1-bit screen has no
                // colours to choose. Measured: a whole boot on a 1-bit display
                // makes 18 CLUT writes, every one of them the ROM's power-on
                // test, against 2316 on the same machine at 8 bits. Reading
                // the table here hands back whatever that test left behind,
                // which is why a 1152x870 desktop came up entirely blue.
                for (int x = 0; x < w; ++x) {
                    const u8 b = vramRead8(row + static_cast<u32>(x >> 3));
                    dst[x] = ((b >> (7 - (x & 7))) & 1) ? 0xFF000000u
                                                        : 0xFFFFFFFFu;
                }
                break;
            case 2:
                for (int x = 0; x < w; ++x) {
                    const u8 b = vramRead8(row + static_cast<u32>(x >> 2));
                    dst[x] = clut_[(b >> (6 - 2 * (x & 3))) & 3];
                }
                break;
            case 4:
                for (int x = 0; x < w; ++x) {
                    const u8 b = vramRead8(row + static_cast<u32>(x >> 1));
                    dst[x] = clut_[(x & 1) ? (b & 0xF) : (b >> 4)];
                }
                break;
            case 8:
                for (int x = 0; x < w; ++x)
                    dst[x] = clut_[vramRead8(row + static_cast<u32>(x))];
                break;
            default:   // 24bpp: one pixel per 32-bit word, xRGB
                for (int x = 0; x < w; ++x)
                    dst[x] = 0xFF000000u | (vramRead32(row + static_cast<u32>(x) * 4) & 0xFFFFFF);
                break;
            }
        }
    }

    // Vertical blank, raised by the machine once per display frame. The
    // status bits latch whether or not their interrupts are enabled (the
    // ROM's beam-sync loops poll them directly); the enables only gate the
    // interrupt line. The beam crosses the cursor scanline every frame.
    void vblank() {
        vblPending_ = true;
        cursorPending_ = true;
        updateIrq();
    }
    bool irqAsserted() const {
        return (vblPending_ && (intMask_ & 1u)) || (cursorPending_ && (intMask_ & 4u));
    }
    bool vblIntEnabled() const { return vblEnabled_; }
    u32 swatchReg(int i) const { return swatch_[i & 63]; }
    u32 ctlReg(int i) const { return regs_[i & 63]; }
    u32 depthCtlRaw() const { return depthCtl_; }
    u32 clutEntry(int i) const { return clut_[i & 0xFF]; }
    // Diagnosis override: render at a stated geometry regardless of what the
    // registers decode to, to settle what the guest is actually drawing.
    void forceMode(int w, int h, int bpp) { forceW_ = w; forceH_ = h; forceBpp_ = bpp; }
    u64 clockStream() const { return clockShift_; }
    std::function<void(bool level)> onIrq;
    std::function<void(const char* msg)> onDiag;

    // ---- monitor sense ----
    // The video connector's three sense lines are open collector with pull-ups
    // at the host end. A monitor identifies itself by tying some of them to
    // ground, or to each other; the host reads the resulting pattern. Three
    // lines only give eight identities, so later displays are named by an
    // extended protocol: the host pulls one line low at a time and reads the
    // other two, and a pair wired together drags its partner down with it.
    // Three such passes give a 6-bit code.
    //
    // The model is therefore not a number but the wiring: which lines this
    // monitor grounds, and which it ties together. Reads derive from that, so
    // both the passive read and every extended pass fall out of one
    // description.
    struct MonitorWiring {
        u8 grounded = 0;    // bit n: sense line n tied to ground
        u8 pairs = 0;       // bit0: 0-1 tied, bit1: 1-2 tied, bit2: 0-2 tied
        // The raster this display runs. An Apple fixed-frequency monitor has
        // exactly one, which is why naming the monitor and choosing the
        // resolution are the same act. Taking the raster from here rather than
        // from the swatch counters is also the only reading that survives every
        // mode: the counters are in units of the video clock, and that clock
        // changes with the display, so the same arithmetic reads 640x480 right
        // and 1152x870 four times narrow.
        int width = 640, height = 480;
    };
    void setMonitor(const MonitorWiring& m) { monitor_ = m; }
    MonitorWiring monitor() const { return monitor_; }

private:
    // What the host reads on the sense lines right now. A line reads low when
    // the monitor grounds it, when the host is driving it low, or when it is
    // tied to another line that is low; otherwise the pull-up wins. Register
    // +$1C's low three bits are the host's drive latch, active low, and the
    // value read back is the inverse of the line state -- so a monitor that
    // grounds nothing reads back all ones.
    u32 senseRead() const {
        // The drive latch is active LOW. The ROM writes 7 and then reads, and
        // that write is it letting go of all three lines: taken the other way
        // up it would be holding all three down, and every monitor read back
        // the same value (measured -- the whole sense sweep returned one
        // answer). A zero bit pulls its line to ground for an extended pass.
        const u8 drive = static_cast<u8>(~regs_[0x1C >> 2] & 7u);
        u8 low = static_cast<u8>(monitor_.grounded | drive);
        // Settle the ties: a pair conducts in both directions, and a chain of
        // pairs conducts along its length, so iterate until nothing changes.
        for (int pass = 0; pass < 3; ++pass) {
            const u8 before = low;
            if (monitor_.pairs & 1) { if (low & 3) low |= 3; }        // 0-1
            if (monitor_.pairs & 2) { if (low & 6) low |= 6; }        // 1-2
            if (monitor_.pairs & 4) { if (low & 5) low |= 5; }        // 0-2
            if (low == before) break;
        }
        return low & 7u;
    }

    void updateIrq() {
        if (onIrq) onIrq(irqAsserted());
    }
    MonitorWiring monitor_{};
    mutable int senseDiag_ = 60;
    int clutDiag_ = 24;
    int forceW_ = 0, forceH_ = 0, forceBpp_ = 0;

    std::vector<u8> vram_;
    u32 regs_[64]{};
    u32 swatch_[64]{};
    u32 clut_[256]{};
    u32 clutAddr_ = 0;
    int clutPhase_ = 0;
    u32 depthCtl_ = 0;
    bool vblEnabled_ = false;
    bool vblPending_ = false;
    bool cursorPending_ = false;
    u32 intMask_ = 0;           // +$104: bit0 VBL, bit2 cursor scanline
    u64 clockShift_ = 0;
};

} // namespace openmac
