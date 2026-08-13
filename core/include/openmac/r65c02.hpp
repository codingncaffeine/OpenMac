#pragma once

// Rockwell/WDC 65C02-compatible execution core.  The IIfx 343S1021 PIC uses
// an NCR 65CX02 cell, including the CMOS instructions (BRA, STZ, TRB/TSB,
// zero-page indirect, PHX/PLX, PHY/PLY and Rockwell bit branches).  The core is
// intentionally bus-agnostic so the PIC can put its shared SRAM and ASIC
// registers in the same 64 KiB program space.

#include "openmac/types.hpp"

#include <array>
#include <functional>

namespace openmac {

class IifxStateCodec;

class R65C02 {
public:
    enum Flag : u8 {
        Carry = 0x01,
        Zero = 0x02,
        IrqDisable = 0x04,
        Decimal = 0x08,
        Break = 0x10,
        Reserved = 0x20,
        Overflow = 0x40,
        Negative = 0x80,
    };

    std::function<u8(u16)> read;
    std::function<void(u16, u8)> write;

    void reset();
    int step();
    void setIrq(bool level) { irq_ = level; }
    void setNmi(bool level);
    u16 recentPc(int back) const {
        const unsigned distance = static_cast<unsigned>(back) & 31u;
        return recentPc_[(recentPcHead_ - 1u - distance) & 31u];
    }
    u16 preBrkPc(int back) const {
        return preBrkPc_[static_cast<unsigned>(back) & 31u];
    }

    u8 a = 0;
    u8 x = 0;
    u8 y = 0;
    u8 s = 0xFF;
    u8 p = Reserved | IrqDisable;
    u16 pc = 0;
    bool waiting = false;
    bool stopped = false;
    u64 instructions = 0;
    u64 cycles = 0;
    u64 brkCount = 0;
    u8 lastOpcode = 0;

private:
    friend class IifxStateCodec;

    enum class Mode { Immediate, Zero, ZeroX, ZeroY, Absolute, AbsoluteX,
                      AbsoluteY, IndirectX, IndirectY, ZeroIndirect };

    u8 rb(u16 address) const;
    void wb(u16 address, u8 value) const;
    u8 fetch8();
    u16 fetch16();
    u16 read16(u16 address) const;
    u16 effective(Mode mode, bool& pageCrossed);
    u8 operand(Mode mode, bool& pageCrossed);
    void push(u8 value);
    u8 pop();
    void nz(u8 value);
    void compare(u8 lhs, u8 rhs);
    void adc(u8 value);
    void sbc(u8 value);
    int interrupt(u16 vector, bool software);
    int branch(bool take);
    int bitBranch(u8 opcode);
    int invalidNop(u8 opcode);

    bool irq_ = false;
    bool nmiLine_ = false;
    bool nmiPending_ = false;
    std::array<u16, 32> recentPc_{};
    std::array<u16, 32> preBrkPc_{};
    unsigned recentPcHead_ = 0;
};

} // namespace openmac
