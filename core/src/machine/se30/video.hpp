#pragma once

// The Macintosh SE/30's built-in screen: 64 KB of video RAM that answers
// through all of slot $E's address space, two screen buffers in it, and one
// interrupt. There is no timing controller, no color table and no register
// of its own -- everything the driver in the video ROM does goes through two
// VIA1 lines.
//
//   VIA1 port A bit 6   which buffer is shown: 1 = main, 0 = alternate
//   VIA1 port B bit 6   0 = the vertical-blank interrupt is enabled;
//                       1 = disabled, and a pending request is cleared
//
// The request is a latch: set at the start of vertical blanking while the
// enable line is low, held until the line goes high. The driver's handler
// raises the line and lowers it again (video ROM $0420).
//
// A buffer is 342 rows of 64 bytes, one bit a dot, most significant bit
// first, 0 = white. Main at +$8040, alternate at +$0040 (Apple Technical
// Note HW14; the video ROM's mode block).
//
// Reference: the SE/30 hardware dossier (Guide to the Macintosh Family
// Hardware 2nd ed. ch. 12, the video ROM's own code, the ROM's behavior).
// Clean-room.

#include "openmac/types.hpp"

#include <array>
#include <functional>

namespace openmac {

class Se30Video {
public:
    static constexpr int kWidth = 512;
    static constexpr int kHeight = 342;
    static constexpr u32 kVramBytes = 0x10000;
    static constexpr u32 kRowBytes = 64;
    static constexpr u32 kBufferBytes = kRowBytes * kHeight;   // 21,888
    static constexpr u32 kMainOffset = 0x8040;
    static constexpr u32 kAlternateOffset = 0x0040;

    // The machine's reset: the lines go back to their power-on levels (both
    // VIA1 lines are inputs then and read high). Video RAM keeps its contents.
    void reset() {
        mainBuffer_ = true;
        enabled_ = false;
        setRequest(false);
    }

    u8 read(u32 addr) const { return vram_[addr & (kVramBytes - 1)]; }
    void write(u32 addr, u8 value) { vram_[addr & (kVramBytes - 1)] = value; }

    void selectMainBuffer(bool main) { mainBuffer_ = main; }
    bool mainBufferShown() const { return mainBuffer_; }

    // VIA1 port B bit 6, as the board sees it: true = line low = enabled.
    void setInterruptEnabled(bool enabled) {
        enabled_ = enabled;
        if (!enabled) setRequest(false);
    }
    bool interruptEnabled() const { return enabled_; }

    // The start of vertical blanking, once a frame.
    void verticalBlank() {
        if (enabled_) setRequest(true);
    }
    bool interruptRequested() const { return request_; }

    // Called when the request line changes level (true = asking).
    std::function<void(bool asking)> onInterrupt;

    // The buffer being shown as 512x342 ARGB pixels.
    void render(u32* argbOut) const {
        const u32 base = mainBuffer_ ? kMainOffset : kAlternateOffset;
        for (u32 row = 0; row < static_cast<u32>(kHeight); ++row) {
            const u8* src = &vram_[base + row * kRowBytes];
            u32* dst = argbOut + row * static_cast<u32>(kWidth);
            for (u32 byte = 0; byte < kRowBytes; ++byte) {
                const u8 bits = src[byte];
                for (u32 bit = 0; bit < 8; ++bit)
                    *dst++ = (bits & (0x80u >> bit)) ? 0xFF000000u : 0xFFFFFFFFu;
            }
        }
    }

    const std::array<u8, kVramBytes>& vram() const { return vram_; }
    std::array<u8, kVramBytes>& vram() { return vram_; }

private:
    void setRequest(bool asking) {
        if (request_ == asking) return;
        request_ = asking;
        if (onInterrupt) onInterrupt(asking);
    }

    std::array<u8, kVramBytes> vram_{};
    bool mainBuffer_ = true;
    bool enabled_ = false;
    bool request_ = false;
};

} // namespace openmac
