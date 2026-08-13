#include "cpu040_ops.hpp"

// Multiply, divide (word and the 68020 long forms), BCD, PACK/UNPK and TAS
// for the '040. Word mul/div flag semantics mirror the SST-validated 68000
// core; the '040's fast multiplier makes the timing data-independent.
//
// Reference: M68000PRM 4-138/4-93 (long mul/div extension words); M68040UM 10.
// Clean-room from the manuals.

namespace openmac {

int CpuOps040::opMul(M68040& c, u16 op) {
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
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
    if (c.is68030()) return 28 + eaFetchTime030(eaIndex(mode, reg), 1);
    return (isSigned ? 16 : 14) + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opDiv(M68040& c, u16 op) {
    const bool isSigned = (op & 0x0100) != 0;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    const u16 s = static_cast<u16>(readEA(c, mode, reg, 1));
    const u32 t = c.d[dreg];
    const int eaT = c.is68030() ? eaFetchTime030(eaIndex(mode, reg), 1)
                                : eaTime(eaIndex(mode, reg));

    if (s == 0) {
        c.sr_ = static_cast<u16>(c.sr_ & ~(kN040 | kZ040 | kV040 | kC040));
        return raiseFrame2(c, kVec040ZeroDivide, instrStart(c), 24 + eaT);
    }

    if (!isSigned) {
        const u32 q = t / s;
        if (q > 0xFFFF) {
            setFlag(c, kV040, true);
            setFlag(c, kC040, false);
            return (c.is68030() ? 44 : 27) + eaT;
        }
        c.d[dreg] = ((t % s) << 16) | (q & 0xFFFF);
        setNZ(c, q, 1);
        c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
        return (c.is68030() ? 44 : 27) + eaT;
    }

    const s32 st = static_cast<s32>(t);
    const s32 ss = static_cast<s32>(static_cast<s16>(s));
    const s64 q64 = static_cast<s64>(st) / ss;
    if (q64 > 32767 || q64 < -32768) {
        setFlag(c, kV040, true);
        setFlag(c, kC040, false);
        return (c.is68030() ? 56 : 27) + eaT;
    }
    const s32 q = static_cast<s32>(q64);
    const s32 rem = st % ss;
    c.d[dreg] = (static_cast<u32>(rem & 0xFFFF) << 16) | static_cast<u32>(q & 0xFFFF);
    setNZ(c, static_cast<u32>(q), 1);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
    return (c.is68030() ? 56 : 27) + eaT;
}

// MULU.L/MULS.L and DIVU.L/DIVS.L/DIVUL.L/DIVSL.L, selected by the opcode's
// bit 6 (0 = multiply at $4C00, 1 = divide at $4C40). Extension word: bits
// 14-12 = Dl/Dq, bit 11 = signed, bit 10 = 64-bit, bits 2-0 = Dh/Dr.
int CpuOps040::opMulDivL(M68040& c, u16 op) {
    const bool isDiv = (op & 0x0040) != 0;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u16 ext = c.fetch16();
    const int rl = (ext >> 12) & 7;         // Dl (mul) / Dq (div)
    const int rh = ext & 7;                 // Dh (mul) / Dr (div)
    const bool isSigned = (ext & 0x0800) != 0;
    const bool is64 = (ext & 0x0400) != 0;
    const u32 s = readEA(c, mode, reg, 2);
    const int eaT = c.is68030()
        ? eaFetchImmediateTime030(c, eaIndex(mode, reg), 1)
        : eaTime(eaIndex(mode, reg));

    if (!isDiv) {
        if (isSigned) {
            const s64 p = static_cast<s64>(static_cast<s32>(s)) *
                          static_cast<s64>(static_cast<s32>(c.d[rl]));
            if (is64) {
                c.d[rh] = static_cast<u32>(static_cast<u64>(p) >> 32);
                c.d[rl] = static_cast<u32>(static_cast<u64>(p));
                setFlag(c, kN040, p < 0);
                setFlag(c, kZ040, p == 0);
                c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
            } else {
                const u32 lo = static_cast<u32>(static_cast<u64>(p));
                c.d[rl] = lo;
                setNZ(c, lo, 2);
                setFlag(c, kV040, p != static_cast<s64>(static_cast<s32>(lo)));
                setFlag(c, kC040, false);
            }
        } else {
            const u64 p = static_cast<u64>(s) * static_cast<u64>(c.d[rl]);
            if (is64) {
                c.d[rh] = static_cast<u32>(p >> 32);
                c.d[rl] = static_cast<u32>(p);
                setFlag(c, kN040, (p & 0x8000000000000000ull) != 0);
                setFlag(c, kZ040, p == 0);
                c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
            } else {
                const u32 lo = static_cast<u32>(p);
                c.d[rl] = lo;
                setNZ(c, lo, 2);
                setFlag(c, kV040, (p >> 32) != 0);
                setFlag(c, kC040, false);
            }
        }
        return (c.is68030() ? 44 : 20) + eaT;
    }

    // Divide. rl = Dq (dividend low / quotient), rh = Dr (remainder; also
    // the dividend high for the 64-bit form).
    if (s == 0) {
        c.sr_ = static_cast<u16>(c.sr_ & ~(kN040 | kZ040 | kV040 | kC040));
        return raiseFrame2(c, kVec040ZeroDivide, instrStart(c), 24 + eaT);
    }

    if (isSigned) {
        const s64 dividend = is64
            ? static_cast<s64>((static_cast<u64>(c.d[rh]) << 32) | c.d[rl])
            : static_cast<s64>(static_cast<s32>(c.d[rl]));
        const s64 ds = static_cast<s64>(static_cast<s32>(s));
        const s64 q = dividend / ds;
        const s64 rem = dividend % ds;
        if (q > 0x7FFFFFFFll || q < -0x80000000ll) {
            setFlag(c, kV040, true);
            setFlag(c, kC040, false);
            return (c.is68030() ? 90 : 44) + eaT;
        }
        if (rh != rl) c.d[rh] = static_cast<u32>(static_cast<u64>(rem));
        c.d[rl] = static_cast<u32>(static_cast<u64>(q));
        setNZ(c, c.d[rl], 2);
        c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
        return (c.is68030() ? 90 : 44) + eaT;
    }

    const u64 dividend = is64 ? ((static_cast<u64>(c.d[rh]) << 32) | c.d[rl])
                              : static_cast<u64>(c.d[rl]);
    const u64 q = dividend / s;
    const u64 rem = dividend % s;
    if (q > 0xFFFFFFFFull) {
        setFlag(c, kV040, true);
        setFlag(c, kC040, false);
        return (c.is68030() ? 78 : 44) + eaT;
    }
    if (rh != rl) c.d[rh] = static_cast<u32>(rem);
    c.d[rl] = static_cast<u32>(q);
    setNZ(c, c.d[rl], 2);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
    return (c.is68030() ? 78 : 44) + eaT;
}

namespace {

u8 abcdByte040(M68040& c, u8 s, u8 t) {
    const u32 x = CpuOps040::flag(c, kX040) ? 1 : 0;
    const u32 lo = (t & 0x0F) + (s & 0x0F) + x;
    const u32 raw = (t & 0xF0) + (s & 0xF0) + lo;
    u32 res = raw;
    if (lo > 9) res += 6;
    bool carry = false;
    if (res > 0x9F) { res += 0x60; carry = true; }
    CpuOps040::setFlag(c, kC040, carry);
    CpuOps040::setFlag(c, kX040, carry);
    CpuOps040::setFlag(c, kN040, (res & 0x80) != 0);
    CpuOps040::setFlag(c, kV040, ((raw & 0x80) == 0) && ((res & 0x80) != 0));
    if ((res & 0xFF) != 0) CpuOps040::setFlag(c, kZ040, false);
    return static_cast<u8>(res);
}

u8 sbcdByte040(M68040& c, u8 s, u8 t) {
    const s32 x = CpuOps040::flag(c, kX040) ? 1 : 0;
    const s32 lo = static_cast<s32>(t & 0x0F) - (s & 0x0F) - x;
    s32 res = static_cast<s32>(t) - s - x;
    const s32 pre = res;
    if (lo < 0) res -= 6;
    bool borrow = false;
    if (pre < 0) { res -= 0x60; borrow = true; }
    else if (res < 0) { borrow = true; }
    CpuOps040::setFlag(c, kC040, borrow);
    CpuOps040::setFlag(c, kX040, borrow);
    CpuOps040::setFlag(c, kN040, (res & 0x80) != 0);
    CpuOps040::setFlag(c, kV040, ((pre & 0x80) != 0) && ((res & 0x80) == 0));
    if ((res & 0xFF) != 0) CpuOps040::setFlag(c, kZ040, false);
    return static_cast<u8>(res & 0xFF);
}

} // namespace

int CpuOps040::opAbcdSbcd(M68040& c, u16 op) {
    const bool isAbcd = (op >> 12) == 0xC;
    const int rx = (op >> 9) & 7;   // destination
    const int ry = op & 7;          // source

    if (!(op & 0x0008)) {
        const u8 s = static_cast<u8>(c.d[ry] & 0xFF);
        const u8 t = static_cast<u8>(c.d[rx] & 0xFF);
        const u8 r = isAbcd ? abcdByte040(c, s, t) : sbcdByte040(c, s, t);
        writeSized(c.d[rx], r, 0);
        return c.is68030() ? 4 : 3;
    }

    const u8 s = c.rd8(calcEA(c, 4, ry, 0));
    const u8 t0 = c.rd8(calcEA(c, 4, rx, 0));
    const u8 r = isAbcd ? abcdByte040(c, s, t0) : sbcdByte040(c, s, t0);
    c.wr8(c.a[rx], r);
    return c.is68030() ? 13 : 4;
}

int CpuOps040::opNbcd(M68040& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    if (mode == 0) {
        const u8 r = sbcdByte040(c, static_cast<u8>(c.d[reg] & 0xFF), 0);
        writeSized(c.d[reg], r, 0);
        return c.is68030() ? 6 : 2;
    }
    const u32 addr = calcEA(c, mode, reg, 0);
    const u8 r = sbcdByte040(c, c.rd8(addr), 0);
    c.wr8(addr, r);
    if (c.is68030()) return 6 + eaFetchTime030(eaIndex(mode, reg), 0);
    return 3 + eaTime(eaIndex(mode, reg));
}

// PACK/UNPK ('020+): unpacked BCD word <-> packed byte, with an adjustment
// added on the unpacked side. No condition codes.
int CpuOps040::opPackUnpk(M68040& c, u16 op) {
    const bool isPack = ((op >> 6) & 3) == 1;
    const bool memForm = (op & 0x0008) != 0;
    const int rx = op & 7;          // source
    const int ry = (op >> 9) & 7;   // destination
    const u16 adj = c.fetch16();

    if (isPack) {
        u16 src;
        if (memForm) src = c.rd16(calcEA(c, 4, rx, 1));
        else         src = static_cast<u16>(c.d[rx] & 0xFFFF);
        src = static_cast<u16>(src + adj);
        const u8 packed = static_cast<u8>(((src >> 4) & 0xF0) | (src & 0x0F));
        if (memForm) c.wr8(calcEA(c, 4, ry, 0), packed);
        else         writeSized(c.d[ry], packed, 0);
    } else {
        u8 src;
        if (memForm) src = c.rd8(calcEA(c, 4, rx, 0));
        else         src = static_cast<u8>(c.d[rx] & 0xFF);
        const u16 unpacked = static_cast<u16>((((src & 0xF0u) << 4) | (src & 0x0Fu)) + adj);
        if (memForm) c.wr16(calcEA(c, 4, ry, 1), unpacked);
        else         writeSized(c.d[ry], unpacked, 1);
    }
    if (c.is68030()) return memForm ? 11 : (isPack ? 6 : 8);
    return memForm ? (isPack ? 5 : 6) : (isPack ? 3 : 4);   // UM 10.5
}

int CpuOps040::opTas(M68040& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    if (mode == 0) {
        const u32 v = c.d[reg] & 0xFF;
        setNZ(c, v, 0);
        c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
        c.d[reg] |= 0x80;
        return c.is68030() ? 4 : 2;
    }
    const u32 addr = calcEA(c, mode, reg, 0);
    const u32 v = c.rd8(addr);
    setNZ(c, v, 0);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
    c.wr8(addr, static_cast<u8>(v | 0x80));
    if (c.is68030()) return 12 + eaCalcTime030(c, eaIndex(mode, reg));
    return 24 + eaTime(eaIndex(mode, reg));
}

} // namespace openmac
