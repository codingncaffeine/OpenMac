#include "cpu040_ops.hpp"

// ALU family for the '040, plus the 68020-era CAS/CAS2 and CHK2/CMP2.
// Flag semantics are identical to the 68000 core (architecturally unchanged);
// cycle counts are the '040's.
//
// Reference: M68000PRM per-instruction pages; M68040UM 10. Clean-room.

namespace openmac {

int CpuOps040::opAddSub(M68040& c, u16 op) {
    const bool isAdd = (op >> 12) == 0xD;
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    const int idx = eaIndex(mode, reg);

    if (op & 0x0100) { // Dn op <ea> -> <ea>
        const u32 addr = calcEA(c, mode, reg, size);
        const u32 t = readAt(c, addr, size);
        const u32 s = c.d[dreg];
        const u32 r = isAdd ? addFlags(c, s, t, size, false)
                            : subFlags(c, s, t, size, false);
        writeAt(c, addr, r, size);
        return 2 + eaTime(idx);
    }

    // <ea> op Dn -> Dn
    const u32 s = readEA(c, mode, reg, size);
    const u32 t = c.d[dreg];
    const u32 r = isAdd ? addFlags(c, s, t, size, false)
                        : subFlags(c, s, t, size, false);
    writeSized(c.d[dreg], r, size);
    return 1 + eaTime(idx);
}

int CpuOps040::opAdda(M68040& c, u16 op) {
    const bool isAdd = (op >> 12) == 0xD;
    const bool isLong = (op & 0x0100) != 0;
    const int size = isLong ? 2 : 1;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;

    u32 s = readEA(c, mode, reg, size);
    if (!isLong) s = static_cast<u32>(static_cast<s32>(static_cast<s16>(s & 0xFFFF)));
    c.a[dreg] = isAdd ? c.a[dreg] + s : c.a[dreg] - s;
    return 2 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opAddiSubi(M68040& c, u16 op) {
    const bool isAdd = ((op >> 9) & 7) == 3;
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 imm = fetchImm(c, size);

    if (mode == 0) {
        const u32 r = isAdd ? addFlags(c, imm, c.d[reg], size, false)
                            : subFlags(c, imm, c.d[reg], size, false);
        writeSized(c.d[reg], r, size);
        return 1;
    }
    const u32 addr = calcEA(c, mode, reg, size);
    const u32 t = readAt(c, addr, size);
    const u32 r = isAdd ? addFlags(c, imm, t, size, false)
                        : subFlags(c, imm, t, size, false);
    writeAt(c, addr, r, size);
    return 2 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opAddqSubq(M68040& c, u16 op) {
    const bool isSub = (op & 0x0100) != 0;
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    u32 data = (op >> 9) & 7;
    if (data == 0) data = 8;

    if (mode == 1) { // address register: full 32-bit, no flags
        c.a[reg] = isSub ? c.a[reg] - data : c.a[reg] + data;
        return 1;
    }
    if (mode == 0) {
        const u32 r = isSub ? subFlags(c, data, c.d[reg], size, false)
                            : addFlags(c, data, c.d[reg], size, false);
        writeSized(c.d[reg], r, size);
        return 1;
    }
    const u32 addr = calcEA(c, mode, reg, size);
    const u32 t = readAt(c, addr, size);
    const u32 r = isSub ? subFlags(c, data, t, size, false)
                        : addFlags(c, data, t, size, false);
    writeAt(c, addr, r, size);
    return 2 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opAddxSubx(M68040& c, u16 op) {
    const bool isAdd = (op >> 12) == 0xD;
    const int size = (op >> 6) & 3;
    const int rx = (op >> 9) & 7;   // destination
    const int ry = op & 7;          // source

    if (!(op & 0x0008)) { // register form
        const u32 r = isAdd ? addFlags(c, c.d[ry], c.d[rx], size, true, true)
                            : subFlags(c, c.d[ry], c.d[rx], size, true, true);
        writeSized(c.d[rx], r, size);
        return 1;
    }

    // -(Ay), -(Ax): source read first
    const u32 srcAddr = calcEA(c, 4, ry, size);
    const u32 s = readAt(c, srcAddr, size);
    const u32 dstAddr = calcEA(c, 4, rx, size);
    const u32 t = readAt(c, dstAddr, size);
    const u32 r = isAdd ? addFlags(c, s, t, size, true, true)
                        : subFlags(c, s, t, size, true, true);
    writeAt(c, dstAddr, r, size);
    return 3;
}

int CpuOps040::opCmp(M68040& c, u16 op) {
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    const u32 s = readEA(c, mode, reg, size);
    cmpFlags(c, s, c.d[dreg], size);
    return 1 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opCmpa(M68040& c, u16 op) {
    const bool isLong = (op & 0x0100) != 0;
    const int size = isLong ? 2 : 1;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    u32 s = readEA(c, mode, reg, size);
    if (!isLong) s = static_cast<u32>(static_cast<s32>(static_cast<s16>(s & 0xFFFF)));
    cmpFlags(c, s, c.a[dreg], 2);
    return 1 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opCmpi(M68040& c, u16 op) {
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 imm = fetchImm(c, size);
    u32 t;
    if (mode == 0) t = c.d[reg] & maskFor(size);
    else           t = readAt(c, calcEA(c, mode, reg, size), size);
    cmpFlags(c, imm, t, size);
    return 1 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opCmpm(M68040& c, u16 op) {
    const int size = (op >> 6) & 3;
    const int rx = (op >> 9) & 7;   // destination
    const int ry = op & 7;          // source
    const u32 s = readAt(c, calcEA(c, 3, ry, size), size);
    const u32 t = readAt(c, calcEA(c, 3, rx, size), size);
    cmpFlags(c, s, t, size);
    return 3;
}

int CpuOps040::opNeg(M68040& c, u16 op) {
    const bool isNegx = ((op >> 9) & 7) == 0;
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;

    if (mode == 0) {
        const u32 v = c.d[reg] & maskFor(size);
        const u32 r = isNegx ? subFlags(c, v, 0, size, true, true)
                             : subFlags(c, v, 0, size, false);
        writeSized(c.d[reg], r, size);
        return 1;
    }
    const u32 addr = calcEA(c, mode, reg, size);
    const u32 v = readAt(c, addr, size);
    const u32 r = isNegx ? subFlags(c, v, 0, size, true, true)
                         : subFlags(c, v, 0, size, false);
    writeAt(c, addr, r, size);
    return 2 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opNot(M68040& c, u16 op) {
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    if (mode == 0) {
        const u32 r = ~c.d[reg] & maskFor(size);
        writeSized(c.d[reg], r, size);
        setLogicFlags(c, r, size);
        return 1;
    }
    const u32 addr = calcEA(c, mode, reg, size);
    const u32 r = ~readAt(c, addr, size) & maskFor(size);
    writeAt(c, addr, r, size);
    setLogicFlags(c, r, size);
    return 2 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opLogic(M68040& c, u16 op) {
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
            return 1;
        }
        const u32 addr = calcEA(c, mode, reg, size);
        const u32 r = apply(c.d[dreg], readAt(c, addr, size)) & maskFor(size);
        writeAt(c, addr, r, size);
        setLogicFlags(c, r, size);
        return 2 + eaTime(idx);
    }

    if (op & 0x0100) { // Dn op <ea> -> <ea>
        const u32 addr = calcEA(c, mode, reg, size);
        const u32 r = apply(c.d[dreg], readAt(c, addr, size)) & maskFor(size);
        writeAt(c, addr, r, size);
        setLogicFlags(c, r, size);
        return 2 + eaTime(idx);
    }

    // <ea> op Dn -> Dn
    const u32 s = readEA(c, mode, reg, size);
    const u32 r = apply(s, c.d[dreg]) & maskFor(size);
    writeSized(c.d[dreg], r, size);
    setLogicFlags(c, r, size);
    return 1 + eaTime(idx);
}

int CpuOps040::opLogicImm(M68040& c, u16 op) {
    const int kind = (op >> 9) & 7;   // 0 ORI, 1 ANDI, 5 EORI
    const auto apply = [kind](u32 x, u32 y) {
        return kind == 0 ? (x | y) : kind == 1 ? (x & y) : (x ^ y);
    };

    if ((op & 0x00FF) == 0x3C) { // to CCR
        const u32 imm = c.fetch16() & 0xFF;
        c.setCCR(static_cast<u8>(apply(imm, c.getSR() & 0x1F)));
        return 4;
    }
    if ((op & 0x00FF) == 0x7C) { // to SR (privileged)
        if (!flag(c, kS040)) return privilegeViolation(c);
        const u32 imm = c.fetch16();
        c.setSR(static_cast<u16>(apply(imm, c.getSR())));
        return 9;
    }

    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 imm = fetchImm(c, size);
    if (mode == 0) {
        const u32 r = apply(imm, c.d[reg]) & maskFor(size);
        writeSized(c.d[reg], r, size);
        setLogicFlags(c, r, size);
        return 1;
    }
    const u32 addr = calcEA(c, mode, reg, size);
    const u32 r = apply(imm, readAt(c, addr, size)) & maskFor(size);
    writeAt(c, addr, r, size);
    setLogicFlags(c, r, size);
    return 2 + eaTime(eaIndex(mode, reg));
}

// CAS Dc,Du,<ea>: compare the operand with Dc; equal -> write Du, else load
// the operand into Dc. The bus lock is meaningless single-threaded.
int CpuOps040::opCas(M68040& c, u16 op) {
    const int size = ((op >> 9) & 3) - 1;   // 01 b, 10 w, 11 l
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u16 ext = c.fetch16();
    const int dc = ext & 7;
    const int du = (ext >> 6) & 7;

    const u32 addr = calcEA(c, mode, reg, size);
    const u32 t = readAt(c, addr, size);
    cmpFlags(c, c.d[dc], t, size);
    if (flag(c, kZ040)) {
        writeAt(c, addr, c.d[du], size);
    } else {
        writeSized(c.d[dc], t, size);
    }
    return 31 + eaTime(eaIndex(mode, reg));
}

// CAS2 Dc1:Dc2,Du1:Du2,(Rn1):(Rn2): the two-address form.
int CpuOps040::opCas2(M68040& c, u16 op) {
    const int size = ((op >> 9) & 3) - 1;   // 10 w, 11 l
    const u16 e1 = c.fetch16();
    const u16 e2 = c.fetch16();
    const u32 a1 = (e1 & 0x8000) ? c.a[(e1 >> 12) & 7] : c.d[(e1 >> 12) & 7];
    const u32 a2 = (e2 & 0x8000) ? c.a[(e2 >> 12) & 7] : c.d[(e2 >> 12) & 7];
    const int dc1 = e1 & 7, du1 = (e1 >> 6) & 7;
    const int dc2 = e2 & 7, du2 = (e2 >> 6) & 7;

    const u32 t1 = readAt(c, a1, size);
    const u32 t2 = readAt(c, a2, size);
    cmpFlags(c, c.d[dc1], t1, size);
    if (flag(c, kZ040)) {
        cmpFlags(c, c.d[dc2], t2, size);
        if (flag(c, kZ040)) {
            writeAt(c, a1, c.d[du1], size);
            writeAt(c, a2, c.d[du2], size);
            return 55;
        }
    }
    writeSized(c.d[dc1], t1, size);
    writeSized(c.d[dc2], t2, size);
    return 50;
}

// CHK2/CMP2 <ea>,Rn: bounds pair at the EA; the subtraction formulation
// handles signed and unsigned bounds uniformly (PRM's definition).
int CpuOps040::opChk2Cmp2(M68040& c, u16 op) {
    const int size = (op >> 9) & 3;         // 00 b, 01 w, 10 l
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u16 ext = c.fetch16();
    const bool isChk = (ext & 0x0800) != 0;
    const int rn = (ext >> 12) & 7;
    const bool isAddr = (ext & 0x8000) != 0;

    const u32 addr = calcEA(c, mode, reg, size);
    u32 lb = readAt(c, addr, size);
    u32 ub = readAt(c, addr + (size == 0 ? 1u : size == 1 ? 2u : 4u), size);
    u32 val;
    if (isAddr) {
        // Address registers compare full-width; bounds sign-extend.
        val = c.a[rn];
        lb = static_cast<u32>(signExtend(lb, size));
        ub = static_cast<u32>(signExtend(ub, size));
    } else {
        val = c.d[rn] & maskFor(size);
    }
    const u32 m = isAddr ? 0xFFFFFFFFu : maskFor(size);
    const u32 rel = (val - lb) & m;
    const u32 span = (ub - lb) & m;
    const bool out = rel > span;
    setFlag(c, kZ040, val == lb || val == ub);
    setFlag(c, kC040, out);
    if (isChk && out) {
        return raiseFrame2(c, kVec040Chk, instrStart(c), 20);
    }
    return 11 + eaTime(eaIndex(mode, reg));
}

} // namespace openmac
