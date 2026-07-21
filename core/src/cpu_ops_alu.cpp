#include "cpu_ops.hpp"

namespace openmac {

namespace {

// Long ops targeting a register gain 2 cycles when the source is a register
// or immediate (single prefetch overlap on the 68000).
int longRegBonus(int eaIdx) { return (eaIdx <= 1 || eaIdx == 11) ? 2 : 0; }

} // namespace

int CpuOps::opAddSub(M68000& c, u16 op) {
    const bool isAdd = (op >> 12) == 0xD;
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    const int idx = eaIndex(mode, reg);

    if (op & 0x0100) { // Dn op <ea> -> <ea>
        const u32 addr = calcEA(c, mode, reg, size);
        const u32 t = readAt(c, addr, size);
        const u32 s = c.d[dreg];
        const u32 r = isAdd ? addFlags(c, s, t, size, false, false)
                            : subFlags(c, s, t, size, false, false);
        writeAt(c, addr, r, size);
        return size == 2 ? 12 + eaTimeL(idx) : 8 + eaTimeBW(idx);
    }

    // <ea> op Dn -> Dn
    const u32 s = readEA(c, mode, reg, size);
    const u32 t = c.d[dreg];
    const u32 r = isAdd ? addFlags(c, s, t, size, false, false)
                        : subFlags(c, s, t, size, false, false);
    writeSized(c.d[dreg], r, size);
    return size == 2 ? 6 + eaTimeL(idx) + longRegBonus(idx) : 4 + eaTimeBW(idx);
}

int CpuOps::opAdda(M68000& c, u16 op) {
    const bool isAdd = (op >> 12) == 0xD;
    const bool isLong = (op & 0x0100) != 0;
    const int size = isLong ? 2 : 1;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    const int idx = eaIndex(mode, reg);

    u32 s = readEA(c, mode, reg, size);
    if (!isLong) s = static_cast<u32>(static_cast<s32>(static_cast<s16>(s & 0xFFFF)));
    c.a[dreg] = isAdd ? c.a[dreg] + s : c.a[dreg] - s;
    return isLong ? 6 + eaTimeL(idx) + longRegBonus(idx) : 8 + eaTimeBW(idx);
}

int CpuOps::opAddiSubi(M68000& c, u16 op) {
    const bool isAdd = ((op >> 9) & 7) == 3;
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 imm = readEA(c, 7, 4, size);   // immediate fetch

    if (mode == 0) {
        const u32 r = isAdd ? addFlags(c, imm, c.d[reg], size, false, false)
                            : subFlags(c, imm, c.d[reg], size, false, false);
        writeSized(c.d[reg], r, size);
        return size == 2 ? 16 : 8;
    }
    const u32 addr = calcEA(c, mode, reg, size);
    const u32 t = readAt(c, addr, size);
    const u32 r = isAdd ? addFlags(c, imm, t, size, false, false)
                        : subFlags(c, imm, t, size, false, false);
    writeAt(c, addr, r, size);
    const int idx = eaIndex(mode, reg);
    return size == 2 ? 20 + eaTimeL(idx) : 12 + eaTimeBW(idx);
}

int CpuOps::opAddqSubq(M68000& c, u16 op) {
    const bool isSub = (op & 0x0100) != 0;
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    u32 data = (op >> 9) & 7;
    if (data == 0) data = 8;

    if (mode == 1) { // address register: full 32-bit, no flags
        c.a[reg] = isSub ? c.a[reg] - data : c.a[reg] + data;
        return size == 2 ? 6 : 8;   // the word form pays for sign extension
    }
    if (mode == 0) {
        const u32 r = isSub ? subFlags(c, data, c.d[reg], size, false, false)
                            : addFlags(c, data, c.d[reg], size, false, false);
        writeSized(c.d[reg], r, size);
        return size == 2 ? 8 : 4;
    }
    const u32 addr = calcEA(c, mode, reg, size);
    const u32 t = readAt(c, addr, size);
    const u32 r = isSub ? subFlags(c, data, t, size, false, false)
                        : addFlags(c, data, t, size, false, false);
    writeAt(c, addr, r, size);
    const int idx = eaIndex(mode, reg);
    return size == 2 ? 12 + eaTimeL(idx) : 8 + eaTimeBW(idx);
}

int CpuOps::opAddxSubx(M68000& c, u16 op) {
    const bool isAdd = (op >> 12) == 0xD;
    const int size = (op >> 6) & 3;
    const int rx = (op >> 9) & 7;   // destination
    const int ry = op & 7;          // source

    if (!(op & 0x0008)) { // register form
        const u32 r = isAdd ? addFlags(c, c.d[ry], c.d[rx], size, true, true)
                            : subFlags(c, c.d[ry], c.d[rx], size, true, true);
        writeSized(c.d[rx], r, size);
        return size == 2 ? 8 : 4;
    }

    // -(Ay), -(Ax): source read first, long operands low-word-first
    const u32 srcAddr = calcPredecLowFirst(c, ry, size);
    const u32 s = readAt(c, srcAddr, size);
    const u32 dstAddr = calcPredecLowFirst(c, rx, size);
    const u32 t = readAt(c, dstAddr, size);
    const u32 r = isAdd ? addFlags(c, s, t, size, true, true)
                        : subFlags(c, s, t, size, true, true);
    writeAt(c, dstAddr, r, size);
    return size == 2 ? 30 : 18;
}

int CpuOps::opCmp(M68000& c, u16 op) {
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    const u32 s = readEA(c, mode, reg, size);
    cmpFlags(c, s, c.d[dreg], size);
    const int idx = eaIndex(mode, reg);
    return size == 2 ? 6 + eaTimeL(idx) : 4 + eaTimeBW(idx);
}

int CpuOps::opCmpa(M68000& c, u16 op) {
    const bool isLong = (op & 0x0100) != 0;
    const int size = isLong ? 2 : 1;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    u32 s = readEA(c, mode, reg, size);
    if (!isLong) s = static_cast<u32>(static_cast<s32>(static_cast<s16>(s & 0xFFFF)));
    cmpFlags(c, s, c.a[dreg], 2);
    const int idx = eaIndex(mode, reg);
    return 6 + (isLong ? eaTimeL(idx) : eaTimeBW(idx));
}

int CpuOps::opCmpi(M68000& c, u16 op) {
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 imm = readEA(c, 7, 4, size);
    u32 t;
    if (mode == 0) t = c.d[reg] & maskFor(size);
    else           t = readAt(c, calcEA(c, mode, reg, size), size);
    cmpFlags(c, imm, t, size);
    const int idx = eaIndex(mode, reg);
    if (mode == 0) return size == 2 ? 14 : 8;
    return size == 2 ? 12 + eaTimeL(idx) : 8 + eaTimeBW(idx);
}

int CpuOps::opCmpm(M68000& c, u16 op) {
    const int size = (op >> 6) & 3;
    const int rx = (op >> 9) & 7;   // destination
    const int ry = op & 7;          // source
    const u32 s = readAt(c, calcEA(c, 3, ry, size), size);
    const u32 t = readAt(c, calcEA(c, 3, rx, size), size);
    cmpFlags(c, s, t, size);
    return size == 2 ? 20 : 12;
}

int CpuOps::opNeg(M68000& c, u16 op) {
    const bool isNegx = ((op >> 9) & 7) == 0;
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;

    if (mode == 0) {
        const u32 v = c.d[reg] & maskFor(size);
        const u32 r = isNegx ? subFlags(c, v, 0, size, true, true)
                             : subFlags(c, v, 0, size, false, false);
        writeSized(c.d[reg], r, size);
        return size == 2 ? 6 : 4;
    }
    const u32 addr = calcEA(c, mode, reg, size);
    const u32 v = readAt(c, addr, size);
    const u32 r = isNegx ? subFlags(c, v, 0, size, true, true)
                         : subFlags(c, v, 0, size, false, false);
    writeAt(c, addr, r, size);
    const int idx = eaIndex(mode, reg);
    return size == 2 ? 12 + eaTimeL(idx) : 8 + eaTimeBW(idx);
}

int CpuOps::opNot(M68000& c, u16 op) {
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    if (mode == 0) {
        const u32 r = ~c.d[reg] & maskFor(size);
        writeSized(c.d[reg], r, size);
        setLogicFlags(c, r, size);
        return size == 2 ? 6 : 4;
    }
    const u32 addr = calcEA(c, mode, reg, size);
    const u32 r = ~readAt(c, addr, size) & maskFor(size);
    writeAt(c, addr, r, size);
    setLogicFlags(c, r, size);
    const int idx = eaIndex(mode, reg);
    return size == 2 ? 12 + eaTimeL(idx) : 8 + eaTimeBW(idx);
}

int CpuOps::opLogic(M68000& c, u16 op) {
    const int top = op >> 12;   // 0x8 OR, 0xC AND, 0xB EOR
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    const int idx = eaIndex(mode, reg);

    const auto apply = [top](u32 x, u32 y) {
        return top == 0x8 ? (x | y) : top == 0xC ? (x & y) : (x ^ y);
    };

    if (top == 0xB) { // EOR Dn,<ea>
        if (mode == 0) {
            const u32 r = apply(c.d[dreg], c.d[reg]) & maskFor(size);
            writeSized(c.d[reg], r, size);
            setLogicFlags(c, r, size);
            return size == 2 ? 8 : 4;
        }
        const u32 addr = calcEA(c, mode, reg, size);
        const u32 r = apply(c.d[dreg], readAt(c, addr, size)) & maskFor(size);
        writeAt(c, addr, r, size);
        setLogicFlags(c, r, size);
        return size == 2 ? 12 + eaTimeL(idx) : 8 + eaTimeBW(idx);
    }

    if (op & 0x0100) { // Dn op <ea> -> <ea>
        const u32 addr = calcEA(c, mode, reg, size);
        const u32 r = apply(c.d[dreg], readAt(c, addr, size)) & maskFor(size);
        writeAt(c, addr, r, size);
        setLogicFlags(c, r, size);
        return size == 2 ? 12 + eaTimeL(idx) : 8 + eaTimeBW(idx);
    }

    // <ea> op Dn -> Dn
    const u32 s = readEA(c, mode, reg, size);
    const u32 r = apply(s, c.d[dreg]) & maskFor(size);
    writeSized(c.d[dreg], r, size);
    setLogicFlags(c, r, size);
    return size == 2 ? 6 + eaTimeL(idx) + longRegBonus(idx) : 4 + eaTimeBW(idx);
}

int CpuOps::opLogicImm(M68000& c, u16 op) {
    const int kind = (op >> 9) & 7;   // 0 ORI, 1 ANDI, 5 EORI
    const auto apply = [kind](u32 x, u32 y) {
        return kind == 0 ? (x | y) : kind == 1 ? (x & y) : (x ^ y);
    };

    if ((op & 0x00FF) == 0x3C) { // to CCR
        const u32 imm = c.fetch16() & 0xFF;
        c.setCCR(static_cast<u8>(apply(imm, c.getSR() & 0x1F)));
        return 20;
    }
    if ((op & 0x00FF) == 0x7C) { // to SR (privileged)
        if (!flag(c, kS)) return privilegeViolation(c);
        const u32 imm = c.fetch16();
        c.setSR(static_cast<u16>(apply(imm, c.getSR())));
        return 20;
    }

    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 imm = readEA(c, 7, 4, size);
    if (mode == 0) {
        const u32 r = apply(imm, c.d[reg]) & maskFor(size);
        writeSized(c.d[reg], r, size);
        setLogicFlags(c, r, size);
        return size == 2 ? 16 : 8;
    }
    const u32 addr = calcEA(c, mode, reg, size);
    const u32 r = apply(imm, readAt(c, addr, size)) & maskFor(size);
    writeAt(c, addr, r, size);
    setLogicFlags(c, r, size);
    const int idx = eaIndex(mode, reg);
    return size == 2 ? 20 + eaTimeL(idx) : 12 + eaTimeBW(idx);
}

} // namespace openmac
