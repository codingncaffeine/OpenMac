#include "cpu040_ops.hpp"

// Shifts, rotates, single-bit ops, and the 68020 bitfield family for the '040.
// Shift semantics (including the ASR carry-stream and ROX rules) mirror the
// 68000 core, which is SST-validated; the '040 has a barrel shifter so counts
// no longer scale the cycle cost. Bitfields follow M68000PRM 4-31..4-44:
// register fields rotate within the register (offset mod 32); memory fields
// address bytes with a SIGNED offset, touching only the bytes the field spans.
//
// Reference: M68000PRM; M68040UM 10. Clean-room from the manuals.

namespace openmac {

namespace {

u32 shiftOnce040(M68040& c, u32 v, int type, bool left, int size, bool& vflag) {
    const u32 sign = CpuOps040::signBit(size);
    const u32 mask = CpuOps040::maskFor(size);
    bool carry;
    u32 r;
    if (left) {
        carry = (v & sign) != 0;
        r = (v << 1) & mask;
        switch (type) {
        case 0: if (((v ^ r) & sign) != 0) vflag = true; break;   // ASL
        case 2: if (CpuOps040::flag(c, kX040)) r |= 1; break;     // ROXL
        case 3: if (carry) r |= 1; break;                         // ROL
        default: break;
        }
    } else {
        carry = (v & 1) != 0;
        r = (v >> 1) & mask;
        switch (type) {
        case 0: r |= (v & sign); break;                             // ASR
        case 2: if (CpuOps040::flag(c, kX040)) r |= sign; break;    // ROXR
        case 3: if (carry) r |= sign; break;                        // ROR
        default: break;
        }
    }
    CpuOps040::setFlag(c, kC040, carry);
    if (type != 3) CpuOps040::setFlag(c, kX040, carry);
    return r;
}

u32 doShift040(M68040& c, u32 v, int type, bool left, int size, int count) {
    if (type == 0 && !left) {   // ASR: sign-fill result, zero-fed carry stream
        const u32 sign = CpuOps040::signBit(size);
        const u32 mask = CpuOps040::maskFor(size);
        const int width = size == 0 ? 8 : size == 1 ? 16 : 32;
        const bool neg = (v & sign) != 0;
        u32 r;
        bool carry;
        if (count == 0) {
            r = v;
            CpuOps040::setFlag(c, kC040, false);
        } else if (count < width) {
            r = (v >> count) & mask;
            if (neg) r |= (mask << (width - count)) & mask;
            carry = ((v >> (count - 1)) & 1) != 0;
            CpuOps040::setFlag(c, kC040, carry);
            CpuOps040::setFlag(c, kX040, carry);
        } else {
            r = neg ? mask : 0;
            carry = (count == width) && neg;
            CpuOps040::setFlag(c, kC040, carry);
            CpuOps040::setFlag(c, kX040, carry);
        }
        CpuOps040::setFlag(c, kV040, false);
        CpuOps040::setNZ(c, r, size);
        return r;
    }

    bool vflag = false;
    for (int i = 0; i < count; ++i) v = shiftOnce040(c, v, type, left, size, vflag);
    if (count == 0) {
        CpuOps040::setFlag(c, kC040, type == 2 && CpuOps040::flag(c, kX040));
    }
    CpuOps040::setFlag(c, kV040, type == 0 && vflag);
    CpuOps040::setNZ(c, v, size);
    return v;
}

} // namespace

int CpuOps040::opShiftReg(M68040& c, u16 op) {
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
    const u32 r = doShift040(c, v, type, left, size, count);
    writeSized(c.d[reg], r, size);
    if (c.is68030()) {
        const bool dynamic = (op & 0x0020) != 0;
        const int width = size == 0 ? 8 : size == 1 ? 16 : 32;
        if (type == 2) return 12;                 // ROX
        if (type == 0 && left) return dynamic ? 8 : 6;
        if (type == 0 && !left)
            return dynamic ? (count <= width ? 6 : 10) : 4;
        if (type == 1)
            return dynamic ? (count <= width ? 6 : 8) : 4;
        return dynamic ? 8 : 6;                  // RO
    }
    return [&] {   // UM 10.6: ASL 3/4, ASR/LSx/ROL/ROR 2/3, ROX 5/6 (imm/reg count)
        const int dyn = (op & 0x0020) ? 1 : 0;
        if (type == 2) return 5 + dyn;
        if (type == 0 && left) return 3 + dyn;
        return 2 + dyn;
    }();
}

int CpuOps040::opShiftMem(M68040& c, u16 op) {
    const bool left = (op & 0x0100) != 0;
    const int type = (op >> 9) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 addr = calcEA(c, mode, reg, 1);
    const u32 v = c.rd16(addr);
    const u32 r = doShift040(c, v, type, left, 1, 1);
    c.wr16(addr, static_cast<u16>(r));
    if (c.is68030()) {
        const int base = type == 2 ? 4 : type == 1 ? 4
                       : type == 0 ? (left ? 6 : 4) : 6;
        return base + eaFetchTime030(eaIndex(mode, reg), 1);
    }
    return 3 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opBitOp(M68040& c, u16 op) {
    const int kind = (op >> 6) & 3;   // 0 BTST 1 BCHG 2 BCLR 3 BSET
    const bool isStatic = (op & 0x0100) == 0;
    const int mode = (op >> 3) & 7, reg = op & 7;

    u32 bit;
    if (isStatic) bit = c.fetch16() & 0xFF;
    else          bit = c.d[(op >> 9) & 7];

    if (mode == 0) { // long, on a data register
        bit &= 31;
        const u32 m = 1u << bit;
        setFlag(c, kZ040, (c.d[reg] & m) == 0);
        switch (kind) {
        case 0: break;
        case 1: c.d[reg] ^= m; break;
        case 2: c.d[reg] &= ~m; break;
        default: c.d[reg] |= m; break;
        }
        if (c.is68030()) return kind == 0 ? 4 : 6;
        return (kind == 0) ? 2 : 4;
    }

    // byte, in memory
    bit &= 7;
    const u32 m = 1u << bit;
    if (kind == 0) {
        const u32 v = readEA(c, mode, reg, 0);
        setFlag(c, kZ040, (v & m) == 0);
        if (c.is68030()) {
            const int ea = isStatic
                ? eaFetchImmediateTime030(c, eaIndex(mode, reg), 1)
                : eaFetchTime030(eaIndex(mode, reg), 0);
            return 4 + ea;
        }
        return 2 + eaTime(eaIndex(mode, reg));
    }
    const u32 addr = calcEA(c, mode, reg, 0);
    u32 v = c.rd8(addr);
    setFlag(c, kZ040, (v & m) == 0);
    switch (kind) {
    case 1: v ^= m; break;
    case 2: v &= ~m; break;
    default: v |= m; break;
    }
    c.wr8(addr, static_cast<u8>(v));
    if (c.is68030()) {
        const int ea = isStatic
            ? eaFetchImmediateTime030(c, eaIndex(mode, reg), 1)
            : eaFetchTime030(eaIndex(mode, reg), 0);
        return 6 + ea;
    }
    return 4 + eaTime(eaIndex(mode, reg));
}

// The bitfield family. kind (op bits 10-8): 0 BFTST, 1 BFEXTU, 2 BFCHG,
// 3 BFEXTS, 4 BFCLR, 5 BFFFO, 6 BFSET, 7 BFINS.
int CpuOps040::opBitField(M68040& c, u16 op) {
    const int kind = (op >> 8) & 7;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u16 ext = c.fetch16();

    s32 offset;
    if (ext & 0x0800) offset = static_cast<s32>(c.d[(ext >> 6) & 7]);
    else              offset = (ext >> 6) & 0x1F;
    u32 width;
    if (ext & 0x0020) width = c.d[ext & 7] & 31;
    else              width = ext & 0x1F;
    width = ((width - 1) & 31) + 1;   // 1..32, 0 means 32
    const int dn = (ext >> 12) & 7;   // dest (BFEXTx/BFFFO) or source (BFINS)

    u32 field;                          // extracted, right-aligned
    const u32 wmask = (width == 32) ? 0xFFFFFFFFu : ((1u << width) - 1u);

    if (mode == 0) {
        // Register field: offset mod 32, field wraps (a rotate exposes it).
        const u32 off = static_cast<u32>(offset) & 31;
        const u32 rot = (off == 0) ? c.d[reg]
                                   : ((c.d[reg] << off) | (c.d[reg] >> (32 - off)));
        field = (width == 32) ? rot : (rot >> (32 - width));

        setFlag(c, kN040, (field & (1u << (width - 1))) != 0);
        setFlag(c, kZ040, field == 0);
        c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));

        u32 newField = field;
        switch (kind) {
        case 0: return c.is68030() ? 8 : 2;                 // BFTST
        case 1: c.d[dn] = field; return c.is68030() ? 10 : 2; // BFEXTU
        case 3: c.d[dn] = (width == 32) ? field             // BFEXTS
                : ((field & (1u << (width - 1))) ? (field | ~wmask) : field);
            return c.is68030() ? 10 : 2;
        case 5: {                                           // BFFFO
            // Register form: the offset is modulo 32 in all respects,
            // including the returned position.
            u32 i = 0;
            while (i < width && !(field & (1u << (width - 1 - i)))) ++i;
            c.d[dn] = off + i;
            return c.is68030() ? 20 : 4;
        }
        case 2: newField = field ^ wmask; break;            // BFCHG
        case 4: newField = 0; break;                        // BFCLR
        case 6: newField = wmask; break;                    // BFSET
        default:                                            // BFINS
            newField = c.d[dn] & wmask;
            setFlag(c, kN040, (newField & (1u << (width - 1))) != 0);
            setFlag(c, kZ040, newField == 0);
            break;
        }
        // Rotate the new field back into place.
        const u32 shifted = (width == 32) ? newField : (newField << (32 - width));
        const u32 fmaskR  = (width == 32) ? 0xFFFFFFFFu : (wmask << (32 - width));
        const u32 merged  = (rot & ~fmaskR) | (shifted & fmaskR);
        c.d[reg] = (off == 0) ? merged
                              : ((merged >> off) | (merged << (32 - off)));
        if (c.is68030()) return kind == 7 ? 12 : 14;
        return 3;
    }

    // Memory field: signed byte offset, bit offset within the first byte;
    // only the spanned bytes (up to 5) are touched.
    const u32 base = calcEA(c, mode, reg, 0);
    const u32 addr = base + static_cast<u32>(offset >> 3);
    const u32 o = static_cast<u32>(offset & 7);
    const u32 nbytes = (o + width + 7) / 8;

    u64 window = 0;
    for (u32 i = 0; i < nbytes; ++i)
        window = (window << 8) | c.rd8(addr + i);
    const u32 shift = nbytes * 8 - o - width;
    field = static_cast<u32>((window >> shift) & wmask);

    setFlag(c, kN040, (field & (1u << (width - 1))) != 0);
    setFlag(c, kZ040, field == 0);
    c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));

    u32 newField = field;
    bool writeBack = false;
    switch (kind) {
    case 0: break;                                          // BFTST
    case 1: c.d[dn] = field; break;                         // BFEXTU
    case 3: c.d[dn] = (width == 32) ? field                 // BFEXTS
            : ((field & (1u << (width - 1))) ? (field | ~wmask) : field);
        break;
    case 5: {                                               // BFFFO
        u32 i = 0;
        while (i < width && !(field & (1u << (width - 1 - i)))) ++i;
        c.d[dn] = static_cast<u32>(offset) + i;
        break;
    }
    case 2: newField = field ^ wmask; writeBack = true; break;   // BFCHG
    case 4: newField = 0; writeBack = true; break;               // BFCLR
    case 6: newField = wmask; writeBack = true; break;           // BFSET
    default:                                                     // BFINS
        newField = c.d[dn] & wmask;
        setFlag(c, kN040, (newField & (1u << (width - 1))) != 0);
        setFlag(c, kZ040, newField == 0);
        writeBack = true;
        break;
    }

    if (writeBack) {
        const u64 fm = static_cast<u64>(wmask) << shift;
        window = (window & ~fm) | ((static_cast<u64>(newField) << shift) & fm);
        for (u32 i = 0; i < nbytes; ++i) {
            const u32 sh = (nbytes - 1 - i) * 8;
            c.wr8(addr + i, static_cast<u8>((window >> sh) & 0xFF));
        }
    }
    if (c.is68030()) {
        const bool five = nbytes == 5;
        int operationCycles;
        switch (kind) {
        case 0: operationCycles = five ? 14 : 10; break;
        case 1: case 3: operationCycles = five ? 18 : 12; break;
        case 2: case 4: case 6: operationCycles = five ? 22 : 14; break;
        case 5: operationCycles = five ? 28 : 22; break;
        default: operationCycles = five ? 18 : 12; break;
        }
        return operationCycles + eaCalcImmediateTime030(c, eaIndex(mode, reg));
    }
    return 4 + eaTime(eaIndex(mode, reg));
}

} // namespace openmac
