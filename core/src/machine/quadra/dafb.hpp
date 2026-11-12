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
// The same class is also the stand-alone chip of the Quadra 950 (DAFB II,
// Variant::Standalone). What differs there:
//   - video RAM is 1 MB on the board plus 1 MB in sockets; the 2 MB window
//     does not repeat, and an empty socket answers with an open bus;
//   - the DAC is an AC842a: its second control register reads back with the
//     part's version in the low four bits, and with 2 MB there is room for
//     32-bit pixels;
//   - +$24 / +$28 hold the two SCSI chips' bus timing (stored here);
//   - the clock generator is a DP8531 with sixteen 4-bit registers at
//     +$300, +$310 ... +$3F0, from which the pixel clock follows.
//
// Reference: Quadra 650 hardware dossier (DAFB/MEMC sections), Apple
// Developer Notes for the Centris 650/Quadra 800 family and the Quadra 950;
// MAME's dafb.cpp for the DP8531's divider arithmetic and the AC842a's
// version nibble (behavior only). Clean-room.

#include "openmac/types.hpp"

#include <cstdio>
#include <functional>
#include <vector>

namespace openmac {

class Dafb {
public:
    // Memc: the cell inside the Quadra 610/650/800 memory controller, 1 MB.
    // Standalone: the Quadra 950's chip, with 1 MB or 2 MB of video RAM fitted.
    enum class Variant { Memc, Standalone };

    explicit Dafb(Variant variant = Variant::Memc, u32 vramBytes = kVramSize)
        : variant_(variant),
          vramFitted_(variant == Variant::Memc ? kVramSize : vramBytes),
          vramMask_((variant == Variant::Memc ? kVramSize : kVramWindow) - 1),
          vram_(vramFitted_, 0) {
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
        ctl1_ = 0;
        vblEnabled_ = false;
        vblPending_ = false;
        cursorPending_ = false;
        intMask_ = 0;
        clockShift_ = 0;
        for (auto& r : clockRegs_) r = 0;
        // The stand-alone chip comes up with the sense lines released (a 1
        // bit lets a line go). The Quadra 950's ROM reads them before it
        // writes the drive register; the Quadra 650's writes first.
        if (variant_ == Variant::Standalone) regs_[0x1C >> 2] = 7;
    }

    Variant variant() const { return variant_; }
    u32 vramBytes() const { return vramFitted_; }

    // ---- VRAM (the machine routes $F9000000+ here) ----
    // The memory controller's cell repeats its 1 MB through the 2 MB window.
    // The stand-alone chip decodes the whole window; what is not fitted reads
    // as an open bus and takes no writes, which is how the ROM tells 1 MB
    // from 2 MB (it leaves a different word at the top of each half megabyte
    // and looks which of them are still there).
    static constexpr u32 kVramSize = 0x100000;     // the cell's, and the 950's on-board megabyte
    static constexpr u32 kVramWindow = 0x200000;
    static constexpr u8 kOpenBus = 0xFF;
    u8 vramRead8(u32 off) const {
        off &= vramMask_;
        return off < vramFitted_ ? vram_[off] : kOpenBus;
    }
    void vramWrite8(u32 off, u8 v) {
        off &= vramMask_;
        if (off < vramFitted_) vram_[off] = v;
    }
    u32 vramRead32(u32 off) const {
        return (static_cast<u32>(vramRead8(off)) << 24) | (static_cast<u32>(vramRead8(off + 1)) << 16) |
               (static_cast<u32>(vramRead8(off + 2)) << 8) | vramRead8(off + 3);
    }
    void vramWrite32(u32 off, u32 v) {
        vramWrite8(off, static_cast<u8>(v >> 24));
        vramWrite8(off + 1, static_cast<u8>(v >> 16));
        vramWrite8(off + 2, static_cast<u8>(v >> 8));
        vramWrite8(off + 3, static_cast<u8>(v));
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
            // The control port is two registers, picked by the address
            // register: 0 is the pixel-bus control (depth), 1 a second
            // one. The ROM tells this DAC from the older single-register
            // kind by writing 6 to the first, 0 to the second, and
            // reading the first back -- and offers the 16-bit modes only
            // if the 6 is still there.
            if (reg == 0x20) {
                if (regDiag_ > 0 && onDiag && variant_ == Variant::Standalone) {
                    --regDiag_;
                    char b[64];
                    std::snprintf(b, sizeof b, "DAC ctl[%u] read (%02X, %02X)", clutAddr_, depthCtl_, ctl1_);
                    onDiag(b);
                }
                if (clutAddr_ != 1) return depthCtl_;
                // The AC842a answers with its version in the low four bits.
                return variant_ == Variant::Standalone ? ((ctl1_ & 0xF0u) | kAc842aVersion) : ctl1_;
            }
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
            if (reg != 0x1C && regDiag_ > 0 && onDiag && variant_ == Variant::Standalone) {
                --regDiag_;
                char b[64];
                std::snprintf(b, sizeof b, "video: +$%02X <- %X", reg, v);
                onDiag(b);
            }
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
                if (clutAddr_ == 1) ctl1_ = v & 0xFF;
                else depthCtl_ = v & 0xFF;
                if (ctlDiag_ > 0 && onDiag) {
                    --ctlDiag_;
                    char b[64];
                    std::snprintf(b, sizeof b, "DAC ctl[%u] <- %02X", clutAddr_, v & 0xFF);
                    onDiag(b);
                }
            }
            break;
        default:
            if (variant_ == Variant::Standalone) {
                // DP8531: sixteen registers of four bits, 16 bytes apart.
                clockRegs_[reg >> 4] = static_cast<u8>(v & 0xF);
                break;
            }
            // DP8534 serial shift: store the stream for the record; the
            // pixel clock itself is derived from the swatch timing.
            clockShift_ = (clockShift_ << 1) | (v & 1);
            break;
        }
    }

    // ---- scanout ----
    // Depth from the Antelope control register bits [4:2].
    // (Measured against the ROM's driver: $80/$88/$90/$98 for 1/2/4/8 bits and
    // $9E for the 16-bit mode, with the second control register at $C0.)
    //
    // Do NOT try to derive this from the row length instead: rows are padded,
    // and by more than a factor of two. The 21-inch display sits at 1 bit with
    // 1152 pixels in a 576-byte row -- 144 bytes of picture in four times the
    // space -- so "the largest depth that fits the row" reads it as 4 bits and
    // renders a quarter of the screen stretched across all of it.
    //
    // $1C in those bits is the widest setting, 32-bit pixels. The 16-bit mode
    // is that setting with the low two mode bits also set and the second
    // control register's top two bits on -- which is how $9E with $C0 reads.
    int bpp() const {
        if (forceBpp_) return forceBpp_;
        switch (depthCtl_ & 0x1C) {
        case 0x00: return 1;
        case 0x08: return 2;
        case 0x10: return 4;
        case 0x18: return 8;
        case 0x1C:
            return ((depthCtl_ & 0x06) == 0x06 && (ctl1_ & 0xC0) == 0xC0) ? 16 : 24;
        default:   return 24;
        }
    }
    // The pixel bus carries the dot clock divided by 1, 2 or 4 (control
    // register bits 6-5): the timing registers count in that slower clock, so
    // a 1152-pixel line is programmed as 288 counts.
    // Bit 7 switches that divider in: of the ROM's 36 modes the one with
    // bit 7 clear (32-bit pixels at 832x624, $3C) is also the one whose
    // timing is written in whole dot clocks.
    int clockDivider() const { return (depthCtl_ & 0x80u) ? 1 << ((depthCtl_ >> 5) & 3u) : 1; }
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

    // ---- the raster as programmed ----
    // What the timing generator was told, as against width()/height(), which
    // is what the attached monitor shows. On every display Apple lists the
    // two agree, and the mode tests hold them against each other.
    //   +$140 HAL  first active count       +$144 HFP  first count after it
    //   +$148 HPIX counts per line less 2   +$14C VHLINE half-lines per frame
    //   +$15C VAL  first active half-line   +$160 VFP  first half-line after
    bool interlaced() const { return (regs_[0x10 >> 2] & 4u) != 0; }
    bool displayDisabled() const { return (swatch_[0] & 1u) != 0; }
    int timingWidth() const {
        const int n = static_cast<int>(swatch_[0x44 >> 2]) - static_cast<int>(swatch_[0x40 >> 2]);
        return n > 0 ? n * clockDivider() : 0;
    }
    int timingHeight() const {
        const int n = static_cast<int>(swatch_[0x60 >> 2]) - static_cast<int>(swatch_[0x5C >> 2]);
        if (n <= 0) return 0;
        return interlaced() ? n : n / 2;
    }
    int timingLineClocks() const { return (static_cast<int>(swatch_[0x48 >> 2]) + 2) * clockDivider(); }
    int timingHalfLines() const { return static_cast<int>(swatch_[0x4C >> 2]); }

    // ---- DP8531 clock generator (stand-alone chip) ----
    // The sixteen registers hold, four bits each: 0-3 the feedback count, 4-6
    // the reference divider, 9 the output divider's power of two. The
    // feedback is a dual-modulus counter: its low five bits (inverted) are A,
    // the rest is B, and the multiplier is 32(B - A) + 31(1 + A). The
    // reference is the board's 20 MHz crystal. (Arithmetic after MAME's
    // dafb.cpp; checked against the dot clocks of Apple's displays in the
    // unit tests.)
    u8 clockReg(int i) const { return clockRegs_[i & 15]; }
    u32 pixelClockHz() const {
        if (variant_ != Variant::Standalone) return 0;
        const int r = (clockRegs_[6] << 8) | (clockRegs_[5] << 4) | clockRegs_[4];
        if (r == 0) return 0;
        const int p = 1 << clockRegs_[9];
        const int modulus = (clockRegs_[3] << 12) | (clockRegs_[2] << 8) | (clockRegs_[1] << 4) | clockRegs_[0];
        int a = (modulus & 0x1F) ^ 0x1F;
        int b = (modulus & 0xFFE0) >> 5;
        if (a > b) a = b;
        if (b < 2) b = 2;
        const int n = 32 * (b - a) + 31 * (1 + a);
        const u64 vco = static_cast<u64>(kClockReferenceHz) * static_cast<u64>(n) / static_cast<u64>(r);
        return static_cast<u32>(vco / static_cast<u64>(p));
    }
    // Vertical scans a second (fields, when interlaced), in thousandths; 0
    // while the timing is not programmed.
    u32 refreshMilliHz() const {
        const u64 halfLineClocks = static_cast<u64>(timingLineClocks()) * static_cast<u64>(timingHalfLines());
        const u32 pclk = pixelClockHz();
        if (pclk == 0 || timingWidth() == 0 || timingHeight() == 0 || halfLineClocks == 0) return 0;
        return static_cast<u32>(static_cast<u64>(pclk) * 2000u / halfLineClocks);
    }

    // Expand the framebuffer to ARGB8888. `out` holds width()*height().
    void render(u32* out) const {
        const int w = width(), h = height();
        if (variant_ == Variant::Standalone && displayDisabled()) {
            for (int i = 0; i < w * h; ++i) out[i] = 0xFF000000u;
            return;
        }
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
                //
                // The Quadra 950's ROM does load entries 0 and 1 at one bit
                // (white, and the display's own black), so there the table
                // is used like at every other depth.
                for (int x = 0; x < w; ++x) {
                    const u8 b = vramRead8(row + static_cast<u32>(x >> 3));
                    const u32 bit = (b >> (7 - (x & 7))) & 1u;
                    if (variant_ == Variant::Standalone) dst[x] = clut_[bit];
                    else dst[x] = bit ? 0xFF000000u : 0xFFFFFFFFu;
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
            // In the two direct modes the color table is still in the
            // path: each channel's value picks an entry and takes that
            // entry's own channel, which is where the driver puts the gamma
            // ramp (256 entries for 32-bit pixels; for 16-bit ones the 32
            // entries a 5-bit value reaches when its top bits are repeated
            // below it: 0, 8, 16 ... 247, 255).
            case 16:   // one big-endian word per pixel, x RRRRR GGGGG BBBBB
                for (int x = 0; x < w; ++x) {
                    const u32 at = row + static_cast<u32>(x) * 2;
                    const u32 p = (static_cast<u32>(vramRead8(at)) << 8) | vramRead8(at + 1);
                    const u32 r = (p >> 10) & 31, g = (p >> 5) & 31, b = p & 31;
                    dst[x] = throughTable((r << 3) | (r >> 2), (g << 3) | (g >> 2), (b << 3) | (b >> 2));
                }
                break;
            default:   // 24bpp: one pixel per 32-bit word, xRGB
                for (int x = 0; x < w; ++x) {
                    const u32 p = vramRead32(row + static_cast<u32>(x) * 4);
                    dst[x] = throughTable((p >> 16) & 0xFF, (p >> 8) & 0xFF, p & 0xFF);
                }
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
    // A direct-color pixel through the color table, channel by channel.
    u32 throughTable(u32 r, u32 g, u32 b) const {
        return 0xFF000000u | (clut_[r] & 0x00FF0000u) | (clut_[g] & 0x0000FF00u) | (clut_[b] & 0x000000FFu);
    }
    static constexpr u32 kAc842aVersion = 1;
    static constexpr u32 kClockReferenceHz = 20000000;

    MonitorWiring monitor_{};
    mutable int senseDiag_ = 60;
    int clutDiag_ = 24;
    int forceW_ = 0, forceH_ = 0, forceBpp_ = 0;

    Variant variant_;
    u32 vramFitted_;            // bytes of video RAM present
    u32 vramMask_;              // the decoded window, less one
    std::vector<u8> vram_;
    u8 clockRegs_[16]{};        // DP8531 (stand-alone chip)
    u32 regs_[64]{};
    u32 swatch_[64]{};
    u32 clut_[256]{};
    u32 clutAddr_ = 0;
    int clutPhase_ = 0;
    u32 depthCtl_ = 0;
    u32 ctl1_ = 0;              // the DAC's second control register
    int ctlDiag_ = 40;
    mutable int regDiag_ = 120;  // stand-alone chip: register writes and DAC reads, for the log
    bool vblEnabled_ = false;
    bool vblPending_ = false;
    bool cursorPending_ = false;
    u32 intMask_ = 0;           // +$104: bit0 VBL, bit2 cursor scanline
    u64 clockShift_ = 0;
};

} // namespace openmac
