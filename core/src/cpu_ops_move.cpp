#include "cpu_ops.hpp"

namespace openmac {

namespace {

// MOVE destination time: like the EA table, but -(An) costs the same as (An).
int moveDstTime(int idx, bool isLong) {
    const int i = (idx == 4) ? 2 : idx;
    return isLong ? CpuOps::eaTimeL(i) : CpuOps::eaTimeBW(i);
}

} // namespace

int CpuOps::opMove(M68000& c, u16 op) {
    const int size    = (op >> 12) == 1 ? 0 : (op >> 12) == 2 ? 2 : 1;
    const int srcMode = (op >> 3) & 7;
    const int srcReg  = op & 7;
    const int dstMode = (op >> 6) & 7;
    const int dstReg  = (op >> 9) & 7;
    const bool isLong = size == 2;

    const u32 v = readEA(c, srcMode, srcReg, size);
    const int srcIdx = eaIndex(srcMode, srcReg);
    const int srcTime = isLong ? eaTimeL(srcIdx) : eaTimeBW(srcIdx);

    if (dstMode == 1) { // MOVEA: sign-extend word, no flags
        c.a[dstReg] = (size == 1)
            ? static_cast<u32>(static_cast<s32>(static_cast<s16>(v & 0xFFFF)))
            : v;
        return 4 + srcTime;
    }

    setNZ(c, v, size);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV | kC));

    if (dstMode == 0) {
        writeSized(c.d[dstReg], v, size);
        return 4 + srcTime;
    }
    if (dstMode == 4) {
        // MOVE to -(An): long writes the low word first; a fault leaves the
        // register at initial-2 (long) and pushes pc without the usual -2.
        if (isLong) {
            c.a[dstReg] -= 2;
            if (c.a[dstReg] & 1) throw AddressError{c.a[dstReg], false, false, 0};
            c.wr16(c.a[dstReg], static_cast<u16>(v & 0xFFFF));
            c.a[dstReg] -= 2;
            c.wr16(c.a[dstReg], static_cast<u16>(v >> 16));
        } else {
            u32 dec = size == 0 ? 1u : 2u;
            if (size == 0 && dstReg == 7) dec = 2;
            c.a[dstReg] -= dec;
            if (size == 1 && (c.a[dstReg] & 1)) {
                throw AddressError{c.a[dstReg], false, false, 0};
            }
            writeAt(c, c.a[dstReg], v, size);
        }
        return 4 + srcTime + moveDstTime(4, isLong);
    }
    const u32 addr = calcEA(c, dstMode, dstReg, size);
    if ((addr & 1) && size >= 1 && dstMode == 7 && dstReg == 1) {
        // (xxx).l destination: the prefetch lag at the fault depends on the
        // source class — register/immediate sources are one refill ahead.
        const bool srcSimple = srcMode <= 1 || (srcMode == 7 && srcReg == 4);
        throw AddressError{addr, false, false, srcSimple ? -2 : -4};
    }
    writeAt(c, addr, v, size);
    return 4 + srcTime + moveDstTime(eaIndex(dstMode, dstReg), isLong);
}

int CpuOps::opMoveq(M68000& c, u16 op) {
    const u32 v = static_cast<u32>(static_cast<s32>(static_cast<s8>(op & 0xFF)));
    c.d[(op >> 9) & 7] = v;
    setNZ(c, v, 2);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV | kC));
    return 4;
}

int CpuOps::opMoveFromSR(M68000& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    if (mode == 0) {
        writeSized(c.d[reg], c.sr_, 1);
        return 6;
    }
    const u32 addr = calcEA(c, mode, reg, 1);
    (void)c.rd16(addr);          // 68000 reads the destination first
    c.wr16(addr, c.sr_);
    return 8 + eaTimeBW(eaIndex(mode, reg));
}

int CpuOps::opMoveToCCR(M68000& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 v = readEA(c, mode, reg, 1);
    c.setCCR(static_cast<u8>(v));
    return 12 + eaTimeBW(eaIndex(mode, reg));
}

int CpuOps::opMoveToSR(M68000& c, u16 op) {
    if (!flag(c, kS)) return privilegeViolation(c);
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 v = readEA(c, mode, reg, 1);
    c.setSR(static_cast<u16>(v));
    return 12 + eaTimeBW(eaIndex(mode, reg));
}

int CpuOps::opMoveUsp(M68000& c, u16 op) {
    if (!flag(c, kS)) return privilegeViolation(c);
    const int reg = op & 7;
    if (op & 0x0008) c.a[reg] = c.usp;   // USP -> An
    else             c.usp = c.a[reg];   // An -> USP
    return 4;
}

int CpuOps::opLea(M68000& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 addr = calcEA(c, mode, reg, 2);
    c.a[(op >> 9) & 7] = addr;
    static constexpr int cyc[12] = {0, 0, 4, 0, 0, 8, 12, 8, 12, 8, 12, 0};
    return cyc[eaIndex(mode, reg)];
}

int CpuOps::opPea(M68000& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 addr = calcEA(c, mode, reg, 2);
    c.push32(addr);
    static constexpr int cyc[12] = {0, 0, 12, 0, 0, 16, 20, 16, 20, 16, 20, 0};
    return cyc[eaIndex(mode, reg)];
}

int CpuOps::opClr(M68000& c, u16 op) {
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    if (mode != 0) {
        const u32 addr = calcEA(c, mode, reg, size);
        (void)readAt(c, addr, size);  // 68000 reads before clearing
        writeAt(c, addr, 0, size);
    } else {
        writeSized(c.d[reg], 0, size);
    }
    // Flags only once the access has succeeded (fault frames carry old SR).
    setFlag(c, kN, false);
    setFlag(c, kZ, true);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV | kC));
    if (mode == 0) return size == 2 ? 6 : 4;
    const int idx = eaIndex(mode, reg);
    return size == 2 ? 12 + eaTimeL(idx) : 8 + eaTimeBW(idx);
}

int CpuOps::opScc(M68000& c, u16 op) {
    const bool cond = testCond(c, (op >> 8) & 15);
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 v = cond ? 0xFFu : 0x00u;
    if (mode == 0) {
        writeSized(c.d[reg], v, 0);
        return cond ? 6 : 4;
    }
    const u32 addr = calcEA(c, mode, reg, 0);
    (void)c.rd8(addr);
    c.wr8(addr, static_cast<u8>(v));
    return 8 + eaTimeBW(eaIndex(mode, reg));
}

int CpuOps::opTst(M68000& c, u16 op) {
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 v = readEA(c, mode, reg, size);
    setNZ(c, v, size);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV | kC));
    const int idx = eaIndex(mode, reg);
    return 4 + (size == 2 ? eaTimeL(idx) : eaTimeBW(idx));
}

int CpuOps::opExg(M68000& c, u16 op) {
    const int rx = (op >> 9) & 7, ry = op & 7;
    const u16 pat = op & 0x01F8;
    if (pat == 0x0140)      { const u32 t = c.d[rx]; c.d[rx] = c.d[ry]; c.d[ry] = t; }
    else if (pat == 0x0148) { const u32 t = c.a[rx]; c.a[rx] = c.a[ry]; c.a[ry] = t; }
    else                    { const u32 t = c.d[rx]; c.d[rx] = c.a[ry]; c.a[ry] = t; }
    return 6;
}

int CpuOps::opSwap(M68000& c, u16 op) {
    const int reg = op & 7;
    c.d[reg] = (c.d[reg] >> 16) | (c.d[reg] << 16);
    setNZ(c, c.d[reg], 2);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV | kC));
    return 4;
}

int CpuOps::opExt(M68000& c, u16 op) {
    const int reg = op & 7;
    if (((op >> 6) & 3) == 2) { // EXT.w
        const u32 v = static_cast<u32>(static_cast<s32>(static_cast<s8>(c.d[reg] & 0xFF))) & 0xFFFF;
        writeSized(c.d[reg], v, 1);
        setNZ(c, v, 1);
    } else {                    // EXT.l
        const u32 v = static_cast<u32>(static_cast<s32>(static_cast<s16>(c.d[reg] & 0xFFFF)));
        c.d[reg] = v;
        setNZ(c, v, 2);
    }
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV | kC));
    return 4;
}

int CpuOps::opLink(M68000& c, u16 op) {
    const int reg = op & 7;
    const s32 disp = static_cast<s16>(c.fetch16());
    c.a[7] -= 4;
    c.wr32(c.a[7], c.a[reg]);   // for LINK A7 this stores the decremented SP
    c.a[reg] = c.a[7];
    c.a[7] += static_cast<u32>(disp);
    return 16;
}

int CpuOps::opUnlk(M68000& c, u16 op) {
    const int reg = op & 7;
    c.a[7] = c.a[reg];
    c.a[reg] = c.pop32();
    return 12;
}

int CpuOps::opMovem(M68000& c, u16 op) {
    const bool toRegs = (op & 0x0400) != 0;
    const bool isLong = (op & 0x0040) != 0;
    const int size = isLong ? 2 : 1;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u16 mask = c.fetch16();
    const int step = isLong ? 4 : 2;

    int n = 0;
    for (int i = 0; i < 16; ++i) n += (mask >> i) & 1;

    if (!toRegs && mode == 4) { // reg -> mem, predecrement: A7..D0
        // Reversed mask: bit 0 = A7 .. bit 7 = A0, bit 8 = D7 .. bit 15 = D0
        u32 addr = c.a[reg];
        for (int i = 0; i < 16; ++i) {
            if (!((mask >> i) & 1)) continue;
            addr -= static_cast<u32>(step);
            const u32 val = (i < 8) ? c.a[7 - i] : c.d[15 - i];
            if (isLong) {   // low word first, like every predec long write
                c.wr16(addr + 2, static_cast<u16>(val & 0xFFFF));
                c.wr16(addr, static_cast<u16>(val >> 16));
            } else {
                c.wr16(addr, static_cast<u16>(val & 0xFFFF));
            }
        }
        c.a[reg] = addr;
        return 8 + (isLong ? 8 : 4) * n;
    }

    if (!toRegs) { // reg -> mem, control modes: D0..A7 ascending
        u32 addr = calcEA(c, mode, reg, size);
        for (int i = 0; i < 16; ++i) {
            if (!((mask >> i) & 1)) continue;
            const u32 val = (i < 8) ? c.d[i] : c.a[i - 8];
            writeAt(c, addr, isLong ? val : (val & 0xFFFF), size);
            addr += static_cast<u32>(step);
        }
        static constexpr int base[12] = {0, 0, 8, 0, 0, 12, 14, 12, 16, 0, 0, 0};
        return base[eaIndex(mode, reg)] + (isLong ? 8 : 4) * n;
    }

    // mem -> reg: D0..A7 ascending; word loads sign-extend. A postinc base
    // is committed +2 around each read, so a faulting read leaves initial+2.
    u32 addr = (mode == 3) ? c.a[reg] : calcEA(c, mode, reg, size);
    for (int i = 0; i < 16; ++i) {
        if (!((mask >> i) & 1)) continue;
        if (mode == 3) c.a[reg] = addr + 2;
        u32 v = readAt(c, addr, size);
        if (!isLong) v = static_cast<u32>(static_cast<s32>(static_cast<s16>(v & 0xFFFF)));
        if (i < 8) c.d[i] = v; else c.a[i - 8] = v;
        addr += static_cast<u32>(step);
    }
    if (mode == 3) c.a[reg] = addr;
    static constexpr int base[12] = {0, 0, 12, 12, 0, 16, 18, 16, 20, 16, 18, 0};
    return base[eaIndex(mode, reg)] + (isLong ? 8 : 4) * n;
}

int CpuOps::opMovep(M68000& c, u16 op) {
    const int dreg = (op >> 9) & 7;
    const int areg = op & 7;
    const int om = (op >> 6) & 3;   // 0 w m->r, 1 l m->r, 2 w r->m, 3 l r->m
    const s32 disp = static_cast<s16>(c.fetch16());
    u32 addr = c.a[areg] + static_cast<u32>(disp);
    const bool isLong = (om & 1) != 0;
    const bool toReg = om < 2;
    const int bytes = isLong ? 4 : 2;

    if (toReg) {
        u32 v = 0;
        for (int i = 0; i < bytes; ++i) { v = (v << 8) | c.rd8(addr); addr += 2; }
        if (isLong) c.d[dreg] = v;
        else        writeSized(c.d[dreg], v, 1);
    } else {
        for (int i = bytes - 1; i >= 0; --i) {
            c.wr8(addr, static_cast<u8>((c.d[dreg] >> (8 * i)) & 0xFF));
            addr += 2;
        }
    }
    return isLong ? 24 : 16;
}

} // namespace openmac
