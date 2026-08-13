#pragma once

// Macintosh IIfx Operating System Support (OSS) interrupt controller.
//
// OSS replaces VIA2 and the Macintosh-II GLUE interrupt encoder.  Sixteen
// inputs each have a software-programmable 68030 priority (0 disables the
// source, 1..7 select IPL).  Level inputs remain pending while asserted;
// pulse inputs, notably the 60.15 Hz clock, latch until acknowledged.

#include "openmac/types.hpp"

#include <array>
#include <functional>

namespace openmac {

class IifxStateCodec;

class Oss {
public:
    enum Source : int {
        Slot9 = 0, SlotA, SlotB, SlotC, SlotD, SlotE,
        IsmIop = 6,
        SccIop = 7,
        Sound = 8,
        Scsi = 9,
        Clock60Hz = 10,
        Via1 = 11,
        Parity = 14,
    };

    std::function<void()> onInterruptChange;

    void reset() {
        priority_.fill(0);
        levelMask_ = 0;
        latchMask_ = 0;
        romControl_ = 0;
        poweredOff_ = false;
        changed();
    }

    u8 read(u32 reg) const {
        reg &= 0x1FFFu;
        if (reg < priority_.size()) return priority_[reg];
        switch (reg) {
        case 0x202: return static_cast<u8>(pending() >> 8);
        case 0x203: return static_cast<u8>(pending());
        case 0x204: return romControl_;
        case 0x207: return 0;                 // 60 Hz acknowledge is write-only
        default: return 0;
        }
    }

    void write(u32 reg, u8 value) {
        reg &= 0x1FFFu;
        if (reg < priority_.size()) {
            priority_[reg] = static_cast<u8>(value & 7u);
            changed();
            return;
        }
        switch (reg) {
        case 0x202:
            // Diagnostic/service software can clear latched high-byte sources
            // by writing ones.  Live level inputs immediately remain pending.
            latchMask_ &= static_cast<u16>(~(static_cast<u16>(value) << 8));
            changed();
            break;
        case 0x203:
            latchMask_ &= static_cast<u16>(~value);
            changed();
            break;
        case 0x204:
            romControl_ = value;
            poweredOff_ = (value & 0x80u) != 0;
            break;
        case 0x207:
            latchMask_ &= static_cast<u16>(~(1u << Clock60Hz));
            changed();
            break;
        default:
            break;
        }
    }

    void setLevel(int source, bool asserted) {
        if (source < 0 || source >= 16) return;
        const u16 bit = static_cast<u16>(1u << source);
        const u16 before = levelMask_;
        if (asserted) levelMask_ |= bit;
        else levelMask_ &= static_cast<u16>(~bit);
        if (before != levelMask_) changed();
    }

    void pulse(int source) {
        if (source < 0 || source >= 16) return;
        latchMask_ |= static_cast<u16>(1u << source);
        changed();
    }

    u16 pending() const { return static_cast<u16>(levelMask_ | latchMask_); }
    u16 levelMask() const { return levelMask_; }
    u16 latchMask() const { return latchMask_; }
    u8 priority(int source) const {
        return source >= 0 && source < 16 ? priority_[source] : 0;
    }

    int ipl() const {
        const u16 active = pending();
        int result = 0;
        for (int source = 0; source < 16; ++source) {
            if (active & (1u << source)) {
                const int p = priority_[source] & 7;
                if (p > result) result = p;
            }
        }
        return result;
    }

    u8 romControl() const { return romControl_; }
    bool poweredOff() const { return poweredOff_; }

private:
    friend class IifxStateCodec;

    void changed() { if (onInterruptChange) onInterruptChange(); }

    std::array<u8, 16> priority_{};
    u16 levelMask_ = 0;
    u16 latchMask_ = 0;
    u8 romControl_ = 0;
    bool poweredOff_ = false;
};

} // namespace openmac
