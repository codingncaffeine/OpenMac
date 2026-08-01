#include "cpu040_ops.hpp"

// MOVE family for the '040. Differences from the 68000 file: no odd-address
// faults (misaligned data is legal and split by the bus layer), CLR and Scc
// are pure writes (the read-before-write was a 68000 artifact), MOVE from SR
// is privileged ('010+), MOVE from CCR and EXTB.L and LINK.L exist, and MOVEM
// with the base register in a predecrement mask writes the decremented value
// ('020+ rule). Cycle counts are the M68040UM section 10 single-issue values.
//
// Reference: M68000PRM per-instruction pages; M68040UM 10. Clean-room.

namespace openmac {

int CpuOps040::opMove(M68040& c, u16 op) {
    const int size    = (op >> 12) == 1 ? 0 : (op >> 12) == 2 ? 2 : 1;
    const int srcMode = (op >> 3) & 7;
    const int srcReg  = op & 7;
    const int dstMode = (op >> 6) & 7;
    const int dstReg  = (op >> 9) & 7;

    const u32 v = readEA(c, srcMode, srcReg, size);
    const int srcTime = eaTime(eaIndex(srcMode, srcReg));

    if (dstMode == 1) { // MOVEA: sign-extend word, no flags
        c.a[dstReg] = (size == 1)
            ? static_cast<u32>(static_cast<s32>(static_cast<s16>(v & 0xFFFF)))
            : v;
        return 1 + srcTime;
    }

    setNZ(c, v, size);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));

    if (dstMode == 0) {
        writeSized(c.d[dstReg], v, size);
        return 1 + srcTime;
    }
    const u32 addr = calcEA(c, dstMode, dstReg, size);
    writeAt(c, addr, v, size);
    return 1 + srcTime + eaTime(eaIndex(dstMode, dstReg));
}

int CpuOps040::opMoveq(M68040& c, u16 op) {
    const u32 v = static_cast<u32>(static_cast<s32>(static_cast<s8>(op & 0xFF)));
    c.d[(op >> 9) & 7] = v;
    setNZ(c, v, 2);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
    return 1;
}

int CpuOps040::opMoveFromSR(M68040& c, u16 op) {
    if (!flag(c, kS040)) return privilegeViolation(c);   // privileged on '010+
    const int mode = (op >> 3) & 7, reg = op & 7;
    if (mode == 0) {
        writeSized(c.d[reg], c.sr_, 1);
        return 3;
    }
    const u32 addr = calcEA(c, mode, reg, 1);
    c.wr16(addr, c.sr_);
    return 3 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opMoveFromCCR(M68040& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u16 ccr = static_cast<u16>(c.sr_ & 0x1F);
    if (mode == 0) {
        writeSized(c.d[reg], ccr, 1);
        return 2;
    }
    const u32 addr = calcEA(c, mode, reg, 1);
    c.wr16(addr, ccr);
    return 2 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opMoveToCCR(M68040& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 v = readEA(c, mode, reg, 1);
    c.setCCR(static_cast<u8>(v));
    return 2 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opMoveToSR(M68040& c, u16 op) {
    if (!flag(c, kS040)) return privilegeViolation(c);
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 v = readEA(c, mode, reg, 1);
    c.setSR(static_cast<u16>(v));
    return 9 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opMoveUsp(M68040& c, u16 op) {
    if (!flag(c, kS040)) return privilegeViolation(c);
    // Privileged, so a[7] is a supervisor stack and the usp field is the truth.
    const int reg = op & 7;
    if (op & 0x0008) c.a[reg] = c.usp;   // USP -> An
    else             c.usp = c.a[reg];   // An -> USP
    return (op & 0x0008) ? 3 : 7;   // UM 10.5: USP,An 3; An,USP 7
}

int CpuOps040::opLea(M68040& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 addr = calcEA(c, mode, reg, 2);
    c.a[(op >> 9) & 7] = addr;
    return 1 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opPea(M68040& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 addr = calcEA(c, mode, reg, 2);
    c.push32(addr);
    return 2 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opClr(M68040& c, u16 op) {
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    if (mode != 0) {
        const u32 addr = calcEA(c, mode, reg, size);
        writeAt(c, addr, 0, size);   // '010+: a pure write
    } else {
        writeSized(c.d[reg], 0, size);
    }
    setFlag(c, kN040, false);
    setFlag(c, kZ040, true);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
    return mode == 0 ? 1 : 1 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opScc(M68040& c, u16 op) {
    const bool cond = testCond(c, (op >> 8) & 15);
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 v = cond ? 0xFFu : 0x00u;
    if (mode == 0) {
        writeSized(c.d[reg], v, 0);
        return 1;
    }
    const u32 addr = calcEA(c, mode, reg, 0);
    c.wr8(addr, static_cast<u8>(v));   // '010+: no read-before-write
    return 1 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opTst(M68040& c, u16 op) {
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    u32 v = readEA(c, mode, reg, size);
    if (mode == 1 && size == 1) v &= 0xFFFF;   // TST.W An tests the low word
    setNZ(c, v, size);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
    return 1 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opExg(M68040& c, u16 op) {
    const int rx = (op >> 9) & 7, ry = op & 7;
    const u16 pat = op & 0x01F8;
    if (pat == 0x0140)      { const u32 t = c.d[rx]; c.d[rx] = c.d[ry]; c.d[ry] = t; }
    else if (pat == 0x0148) { const u32 t = c.a[rx]; c.a[rx] = c.a[ry]; c.a[ry] = t; }
    else                    { const u32 t = c.d[rx]; c.d[rx] = c.a[ry]; c.a[ry] = t; }
    return 1;
}

int CpuOps040::opSwap(M68040& c, u16 op) {
    const int reg = op & 7;
    c.d[reg] = (c.d[reg] >> 16) | (c.d[reg] << 16);
    setNZ(c, c.d[reg], 2);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
    return 2;
}

int CpuOps040::opExt(M68040& c, u16 op) {
    const int reg = op & 7;
    const int om = (op >> 6) & 7;
    if (om == 2) {          // EXT.W: byte -> word
        const u32 v = static_cast<u32>(static_cast<s32>(static_cast<s8>(c.d[reg] & 0xFF))) & 0xFFFF;
        writeSized(c.d[reg], v, 1);
        setNZ(c, v, 1);
    } else if (om == 3) {   // EXT.L: word -> long
        const u32 v = static_cast<u32>(static_cast<s32>(static_cast<s16>(c.d[reg] & 0xFFFF)));
        c.d[reg] = v;
        setNZ(c, v, 2);
    } else {                // EXTB.L ('020+): byte -> long
        const u32 v = static_cast<u32>(static_cast<s32>(static_cast<s8>(c.d[reg] & 0xFF)));
        c.d[reg] = v;
        setNZ(c, v, 2);
    }
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
    return 1;
}

int CpuOps040::opLink(M68040& c, u16 op) {
    const int reg = op & 7;
    const s32 disp = static_cast<s16>(c.fetch16());
    c.a[7] -= 4;
    c.wr32(c.a[7], c.a[reg]);   // for LINK A7 this stores the decremented SP
    c.a[reg] = c.a[7];
    c.a[7] += static_cast<u32>(disp);
    return 3;
}

int CpuOps040::opLinkL(M68040& c, u16 op) {
    const int reg = op & 7;
    const s32 disp = static_cast<s32>(c.fetch32());
    c.a[7] -= 4;
    c.wr32(c.a[7], c.a[reg]);
    c.a[reg] = c.a[7];
    c.a[7] += static_cast<u32>(disp);
    return 3;
}

int CpuOps040::opUnlk(M68040& c, u16 op) {
    const int reg = op & 7;
    c.a[7] = c.a[reg];
    c.a[reg] = c.pop32();
    return 2;
}

int CpuOps040::opMovem(M68040& c, u16 op) {
    const bool toRegs = (op & 0x0400) != 0;
    const bool isLong = (op & 0x0040) != 0;
    const int size = isLong ? 2 : 1;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u16 mask = c.fetch16();
    const int step = isLong ? 4 : 2;

    int n = 0;
    for (int i = 0; i < 16; ++i) n += (mask >> i) & 1;

    if (!toRegs && mode == 4) { // reg -> mem, predecrement: A7..D0
        // Reversed mask: bit 0 = A7 .. bit 7 = A0, bit 8 = D7 .. bit 15 = D0.
        // '020+ rule: if the base register is in the mask, the value written
        // is the initial value decremented by the operand size.
        const u32 initial = c.a[reg];
        u32 addr = initial;
        for (int i = 0; i < 16; ++i) {
            if (!((mask >> i) & 1)) continue;
            addr -= static_cast<u32>(step);
            u32 val = (i < 8) ? c.a[7 - i] : c.d[15 - i];
            if (i < 8 && (7 - i) == reg) val = initial - static_cast<u32>(step);
            writeAt(c, addr, isLong ? val : (val & 0xFFFF), size);
        }
        c.a[reg] = addr;
        return 2 + n;
    }

    if (!toRegs) { // reg -> mem, control modes: D0..A7 ascending
        u32 addr = calcEA(c, mode, reg, size);
        for (int i = 0; i < 16; ++i) {
            if (!((mask >> i) & 1)) continue;
            const u32 val = (i < 8) ? c.d[i] : c.a[i - 8];
            writeAt(c, addr, isLong ? val : (val & 0xFFFF), size);
            addr += static_cast<u32>(step);
        }
        return 2 + n + eaTime(eaIndex(mode, reg));
    }

    // mem -> reg: D0..A7 ascending; word loads sign-extend.
    u32 addr = (mode == 3) ? c.a[reg] : calcEA(c, mode, reg, size);
    for (int i = 0; i < 16; ++i) {
        if (!((mask >> i) & 1)) continue;
        u32 v = readAt(c, addr, size);
        if (!isLong) v = static_cast<u32>(static_cast<s32>(static_cast<s16>(v & 0xFFFF)));
        if (i < 8) c.d[i] = v; else c.a[i - 8] = v;
        addr += static_cast<u32>(step);
    }
    if (mode == 3) c.a[reg] = addr;   // a loaded base register keeps the load
    return 2 + n + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opMovep(M68040& c, u16 op) {
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
    return isLong ? (toReg ? 10 : 13) : (toReg ? 7 : 11);   // UM 10.5
}

} // namespace openmac
