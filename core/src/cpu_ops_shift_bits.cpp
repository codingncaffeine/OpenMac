#include "cpu_ops.hpp"

namespace openmac {

namespace {

// One shift/rotate of `v` (sized), updating C/X/V trackers.
// type: 0 AS, 1 LS, 2 ROX, 3 RO. Returns new value.
u32 shiftOnce(M68000& c, u32 v, int type, bool left, int size, bool& vflag) {
    const u32 sign = CpuOps::signBit(size);
    const u32 mask = CpuOps::maskFor(size);
    bool carry;
    u32 r;
    if (left) {
        carry = (v & sign) != 0;
        r = (v << 1) & mask;
        switch (type) {
        case 0: // ASL: V accumulates if the sign bit ever changes
            if (((v ^ r) & sign) != 0) vflag = true;
            break;
        case 2: // ROXL: X into bit 0
            if (CpuOps::flag(c, kX)) r |= 1;
            break;
        case 3: // ROL: MSB wraps to bit 0
            if (carry) r |= 1;
            break;
        default: break;
        }
    } else {
        carry = (v & 1) != 0;
        r = (v >> 1) & mask;
        switch (type) {
        case 0: // ASR: replicate sign
            r |= (v & sign);
            break;
        case 2: // ROXR: X into MSB
            if (CpuOps::flag(c, kX)) r |= sign;
            break;
        case 3: // ROR: bit 0 wraps to MSB
            if (carry) r |= sign;
            break;
        default: break;
        }
    }
    CpuOps::setFlag(c, kC, carry);
    if (type != 3) CpuOps::setFlag(c, kX, carry);   // RO leaves X alone
    return r;
}

u32 doShift(M68000& c, u32 v, int type, bool left, int size, int count) {
    // ASR: result sign-fills, but the carry stream is the original bits
    // followed by zeros — a count past the width leaves C = X = 0.
    if (type == 0 && !left) {
        const u32 sign = CpuOps::signBit(size);
        const u32 mask = CpuOps::maskFor(size);
        const int width = size == 0 ? 8 : size == 1 ? 16 : 32;
        const bool neg = (v & sign) != 0;
        u32 r;
        bool carry;
        if (count == 0) {
            r = v;
            carry = false;
            CpuOps::setFlag(c, kC, false);
        } else if (count < width) {
            r = (v >> count) & mask;
            if (neg) r |= (mask << (width - count)) & mask;
            carry = ((v >> (count - 1)) & 1) != 0;
        } else {
            r = neg ? mask : 0;
            carry = (count == width) && neg;
        }
        if (count > 0) {
            CpuOps::setFlag(c, kC, carry);
            CpuOps::setFlag(c, kX, carry);
        }
        CpuOps::setFlag(c, kV, false);
        CpuOps::setNZ(c, r, size);
        return r;
    }

    bool vflag = false;
    for (int i = 0; i < count; ++i) v = shiftOnce(c, v, type, left, size, vflag);
    if (count == 0) {
        // C reflects nothing shifted; ROX is special: C = X
        CpuOps::setFlag(c, kC, type == 2 && CpuOps::flag(c, kX));
    }
    CpuOps::setFlag(c, kV, type == 0 && vflag);
    CpuOps::setNZ(c, v, size);
    return v;
}

} // namespace

int CpuOps::opShiftReg(M68000& c, u16 op) {
    const int size = (op >> 6) & 3;
    const bool left = (op & 0x0100) != 0;
    const int type = (op >> 3) & 3;
    const int reg = op & 7;
    int count;
    if (op & 0x0020) {
        count = static_cast<int>(c.d[(op >> 9) & 7] & 63);
    } else {
        count = (op >> 9) & 7;
        if (count == 0) count = 8;
    }
    const u32 v = c.d[reg] & maskFor(size);
    const u32 r = doShift(c, v, type, left, size, count);
    writeSized(c.d[reg], r, size);
    return (size == 2 ? 8 : 6) + 2 * count;
}

int CpuOps::opShiftMem(M68000& c, u16 op) {
    const bool left = (op & 0x0100) != 0;
    const int type = (op >> 9) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 addr = calcEA(c, mode, reg, 1);
    const u32 v = c.rd16(addr);
    const u32 r = doShift(c, v, type, left, 1, 1);
    c.wr16(addr, static_cast<u16>(r));
    return 8 + eaTimeBW(eaIndex(mode, reg));
}

int CpuOps::opBitOp(M68000& c, u16 op) {
    const int kind = (op >> 6) & 3;   // 0 BTST 1 BCHG 2 BCLR 3 BSET
    const bool isStatic = (op & 0x0100) == 0;
    const int mode = (op >> 3) & 7, reg = op & 7;

    u32 bit;
    if (isStatic) bit = c.fetch16() & 0xFF;
    else          bit = c.d[(op >> 9) & 7];

    if (mode == 0) { // long, on a data register
        bit &= 31;
        const u32 m = 1u << bit;
        setFlag(c, kZ, (c.d[reg] & m) == 0);
        const int high = bit >= 16 ? 2 : 0;   // upper-word bits cost 2 more
        int cyc;
        switch (kind) {
        case 0: cyc = 6; high ? (void)0 : (void)0; break;
        case 1: c.d[reg] ^= m; cyc = 6 + high; break;
        case 2: c.d[reg] &= ~m; cyc = 8 + high; break;
        default: c.d[reg] |= m; cyc = 6 + high; break;
        }
        return cyc + (isStatic ? 4 : 0);
    }

    // byte, in memory (BTST also allows immediate/PC-relative sources)
    bit &= 7;
    const u32 m = 1u << bit;
    if (kind == 0) {
        const u32 v = readEA(c, mode, reg, 0);
        setFlag(c, kZ, (v & m) == 0);
        return (isStatic ? 8 : 4) + eaTimeBW(eaIndex(mode, reg));
    }
    const u32 addr = calcEA(c, mode, reg, 0);
    u32 v = c.rd8(addr);
    setFlag(c, kZ, (v & m) == 0);
    switch (kind) {
    case 1: v ^= m; break;
    case 2: v &= ~m; break;
    default: v |= m; break;
    }
    c.wr8(addr, static_cast<u8>(v));
    return (isStatic ? 12 : 8) + eaTimeBW(eaIndex(mode, reg));
}

} // namespace openmac
