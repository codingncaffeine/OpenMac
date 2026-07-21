#include "cpu_ops.hpp"

namespace openmac {

namespace {

// MULS timing: 38 + 2 per 01/10 transition in (src concatenated with a 0 bit).
int mulsCycles(u16 src) {
    const u32 v = (static_cast<u32>(src) << 1);
    int transitions = 0;
    for (int i = 0; i < 16; ++i) {
        if (((v >> i) & 3) == 1 || ((v >> i) & 3) == 2) ++transitions;
    }
    return 38 + 2 * transitions;
}

int muluCycles(u16 src) {
    int ones = 0;
    for (int i = 0; i < 16; ++i) ones += (src >> i) & 1;
    return 38 + 2 * ones;
}

} // namespace

int CpuOps::opMul(M68000& c, u16 op) {
    const bool isSigned = (op & 0x0100) != 0;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    const u16 s = static_cast<u16>(readEA(c, mode, reg, 1));
    const u16 t = static_cast<u16>(c.d[dreg] & 0xFFFF);

    u32 r;
    if (isSigned) {
        r = static_cast<u32>(static_cast<s32>(static_cast<s16>(s)) *
                             static_cast<s32>(static_cast<s16>(t)));
    } else {
        r = static_cast<u32>(s) * t;
    }
    c.d[dreg] = r;
    setNZ(c, r, 2);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV | kC));
    const int base = isSigned ? mulsCycles(s) : muluCycles(s);
    return base + eaTimeBW(eaIndex(mode, reg));
}

namespace {

// Data-dependent division timing, modeling the 68000's restoring divider.
int divuCycles(u32 dividend, u16 divisor) {
    if ((dividend >> 16) >= divisor) return 10;   // overflow, detected early
    int mcycles = 38;
    const u32 hdivisor = static_cast<u32>(divisor) << 16;
    for (int i = 0; i < 15; ++i) {
        const u32 temp = dividend;
        dividend <<= 1;
        if (static_cast<s32>(temp) < 0) {
            dividend -= hdivisor;
        } else {
            mcycles += 2;
            if (dividend >= hdivisor) { dividend -= hdivisor; mcycles -= 1; }
        }
    }
    return mcycles * 2;
}

int divsCycles(s32 dividend, s16 divisor) {
    int mcycles = 6;
    if (dividend < 0) mcycles += 1;
    const u32 aDividend =
        static_cast<u32>(dividend < 0 ? -static_cast<s64>(dividend) : dividend);
    const u16 aDivisor = static_cast<u16>(divisor < 0 ? -divisor : divisor);
    // The divider checks the pre-shifted dividend: any absolute quotient that
    // cannot fit in 15 bits is caught up front.
    if ((aDividend >> 15) >= aDivisor) return (mcycles + 2) * 2;
    u32 aquot = aDividend / aDivisor;
    mcycles += 55;
    if (divisor >= 0) {
        if (dividend >= 0) mcycles -= 1;
        else mcycles += 1;
    }
    for (int i = 0; i < 15; ++i) {
        if (static_cast<s16>(aquot) >= 0) mcycles += 1;
        aquot <<= 1;
    }
    return mcycles * 2;
}

} // namespace

int CpuOps::opDiv(M68000& c, u16 op) {
    const bool isSigned = (op & 0x0100) != 0;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    const u16 s = static_cast<u16>(readEA(c, mode, reg, 1));
    const u32 t = c.d[dreg];
    const int eaT = eaTimeBW(eaIndex(mode, reg));

    if (s == 0) {
        c.sr_ = static_cast<u16>(c.sr_ & ~(kN | kZ | kV | kC));
        return raiseException(c, kVecZeroDivide, 38 + eaT);
    }

    if (!isSigned) {
        const int cycles = divuCycles(t, s) + eaT;
        const u32 q = t / s;
        if (q > 0xFFFF) {
            setFlag(c, kV, true);
            setFlag(c, kC, false);
            return cycles;
        }
        c.d[dreg] = ((t % s) << 16) | (q & 0xFFFF);
        setNZ(c, q, 1);
        c.sr_ = static_cast<u16>(c.sr_ & ~(kV | kC));
        return cycles;
    }

    const s32 st = static_cast<s32>(t);
    const s32 ss = static_cast<s32>(static_cast<s16>(s));
    const int cycles = divsCycles(st, static_cast<s16>(s)) + eaT;
    const s64 q64 = static_cast<s64>(st) / ss;
    if (q64 > 32767 || q64 < -32768) {
        setFlag(c, kV, true);
        setFlag(c, kC, false);
        return cycles;
    }
    const s32 q = static_cast<s32>(q64);
    const s32 rem = st % ss;
    c.d[dreg] = (static_cast<u32>(rem & 0xFFFF) << 16) | static_cast<u32>(q & 0xFFFF);
    setNZ(c, static_cast<u32>(q), 1);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV | kC));
    return cycles;
}

namespace {

// ABCD/SBCD/NBCD byte cores. Flag behavior (incl. "undefined" N/V) follows
// observed 68000 hardware; the test suite arbitrates the details.
u8 abcdByte(M68000& c, u8 s, u8 t) {
    const u32 x = CpuOps::flag(c, kX) ? 1 : 0;
    const u32 lo = (t & 0x0F) + (s & 0x0F) + x;
    const u32 raw = (t & 0xF0) + (s & 0xF0) + lo;   // uncorrected binary sum
    u32 res = raw;
    if (lo > 9) res += 6;
    bool carry = false;
    if (res > 0x9F) { res += 0x60; carry = true; }
    CpuOps::setFlag(c, kC, carry);
    CpuOps::setFlag(c, kX, carry);
    CpuOps::setFlag(c, kN, (res & 0x80) != 0);
    CpuOps::setFlag(c, kV, ((raw & 0x80) == 0) && ((res & 0x80) != 0));
    if ((res & 0xFF) != 0) CpuOps::setFlag(c, kZ, false);
    return static_cast<u8>(res);
}

u8 sbcdByte(M68000& c, u8 s, u8 t) {
    const s32 x = CpuOps::flag(c, kX) ? 1 : 0;
    const s32 lo = static_cast<s32>(t & 0x0F) - (s & 0x0F) - x;
    s32 res = static_cast<s32>(t) - s - x;
    const s32 pre = res;
    if (lo < 0) res -= 6;
    bool borrow = false;
    if (pre < 0) { res -= 0x60; borrow = true; }
    else if (res < 0) { borrow = true; }
    CpuOps::setFlag(c, kC, borrow);
    CpuOps::setFlag(c, kX, borrow);
    CpuOps::setFlag(c, kN, (res & 0x80) != 0);
    CpuOps::setFlag(c, kV, ((pre & 0x80) != 0) && ((res & 0x80) == 0));
    if ((res & 0xFF) != 0) CpuOps::setFlag(c, kZ, false);
    return static_cast<u8>(res & 0xFF);
}

} // namespace

int CpuOps::opAbcdSbcd(M68000& c, u16 op) {
    const bool isAbcd = (op >> 12) == 0xC;
    const int rx = (op >> 9) & 7;   // destination
    const int ry = op & 7;          // source

    if (!(op & 0x0008)) {
        const u8 s = static_cast<u8>(c.d[ry] & 0xFF);
        const u8 t = static_cast<u8>(c.d[rx] & 0xFF);
        const u8 r = isAbcd ? abcdByte(c, s, t) : sbcdByte(c, s, t);
        writeSized(c.d[rx], r, 0);
        return 6;
    }

    const u8 s = c.rd8(calcPredecLowFirst(c, ry, 0));
    const u8 t0 = c.rd8(calcPredecLowFirst(c, rx, 0));
    const u8 r = isAbcd ? abcdByte(c, s, t0) : sbcdByte(c, s, t0);
    c.wr8(c.a[rx], r);
    return 18;
}

int CpuOps::opNbcd(M68000& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    if (mode == 0) {
        const u8 r = sbcdByte(c, static_cast<u8>(c.d[reg] & 0xFF), 0);
        writeSized(c.d[reg], r, 0);
        return 6;
    }
    const u32 addr = calcEA(c, mode, reg, 0);
    const u8 r = sbcdByte(c, c.rd8(addr), 0);
    c.wr8(addr, r);
    return 8 + eaTimeBW(eaIndex(mode, reg));
}

int CpuOps::opTas(M68000& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    if (mode == 0) {
        const u32 v = c.d[reg] & 0xFF;
        setNZ(c, v, 0);
        c.sr_ = static_cast<u16>(c.sr_ & ~(kV | kC));
        c.d[reg] |= 0x80;
        return 4;
    }
    const u32 addr = calcEA(c, mode, reg, 0);
    const u32 v = c.rd8(addr);
    setNZ(c, v, 0);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV | kC));
    c.wr8(addr, static_cast<u8>(v | 0x80));
    return 10 + eaTimeBW(eaIndex(mode, reg));
}

} // namespace openmac
