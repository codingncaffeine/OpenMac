#include "cpu040_ops.hpp"

// System control for the '040: MOVEC over the full '040 control register set,
// MOVES through SFC/DFC-selected address spaces, MOVE16 line moves, the cache
// instructions (register-visible; an interpreter reading live memory is
// always coherent, so their functional effect is nil), and PFLUSH/PTEST.
//
// Reference: M68040UM 2 (MOVEC codes), 3 (MMU ops), 4 (caches).
// Clean-room from the manuals.

namespace openmac {

int CpuOps040::opMovec(M68040& c, u16 op) {
    if (!flag(c, kS040)) return privilegeViolation(c);
    const bool toCtrl = (op & 1) != 0;   // 4E7B: general -> control
    const u16 ext = c.fetch16();
    const int reg = (ext >> 12) & 7;
    const bool isAddr = (ext & 0x8000) != 0;
    const u16 code = ext & 0x0FFF;
    u32& gen = isAddr ? c.a[reg] : c.d[reg];

    if (c.is68030()) {
        if (toCtrl) {
            const u32 v = gen;
            bool groupB = false; // SFC, DFC, CACR take the longer write path
            switch (code) {
            case 0x000: c.sfc = v & 7; groupB = true; break;
            case 0x001: c.dfc = v & 7; groupB = true; break;
            case 0x002:
                c.writeCacr030(v);
                groupB = true;
                break;
            case 0x800:
                if (!(c.sr_ & kS040)) c.a[7] = v; else c.usp = v;
                break;
            case 0x801: c.vbr = v; break;
            case 0x802: c.caar = v; break;
            case 0x803:
                if ((c.sr_ & kS040) && (c.sr_ & kM040)) c.a[7] = v;
                else c.msp = v;
                break;
            case 0x804:
                if ((c.sr_ & kS040) && !(c.sr_ & kM040)) c.a[7] = v;
                else c.isp = v;
                break;
            default:
                c.pc = instrStart(c);
                return raiseException(c, kVec040Illegal, 16);
            }
            return groupB ? 12 : 6;
        }

        u32 v;
        switch (code) {
        case 0x000: v = c.sfc; break;
        case 0x001: v = c.dfc; break;
        case 0x002: v = c.cacr; break;
        case 0x800: v = c.uspValue(); break;
        case 0x801: v = c.vbr; break;
        case 0x802: v = c.caar; break;
        case 0x803: v = c.mspValue(); break;
        case 0x804: v = c.ispValue(); break;
        default:
            c.pc = instrStart(c);
            return raiseException(c, kVec040Illegal, 16);
        }
        gen = v;
        return 6;
    }

    if (toCtrl) {
        const u32 v = gen;
        switch (code) {
        case 0x000: c.sfc = v & 7; break;
        case 0x001: c.dfc = v & 7; break;
        case 0x002: c.cacr = v & 0x80008000u; break;
        case 0x003: c.tc = v & 0xC000u; c.tlbFlush(); break;
        case 0x004: c.itt0 = v; break;
        case 0x005: c.itt1 = v; break;
        case 0x006: c.dtt0 = v; break;
        case 0x007: c.dtt1 = v; break;
        case 0x800:
            if (!(c.sr_ & kS040)) c.a[7] = v; else c.usp = v;
            break;
        case 0x801: c.vbr = v; break;
        case 0x803:
            if ((c.sr_ & kS040) && (c.sr_ & kM040)) c.a[7] = v; else c.msp = v;
            break;
        case 0x804:
            if ((c.sr_ & kS040) && !(c.sr_ & kM040)) c.a[7] = v; else c.isp = v;
            break;
        case 0x805: c.mmusr = v; break;
        case 0x806: c.urp = v & 0xFFFFFE00u; c.tlbFlush(); break;
        case 0x807: c.srp = v & 0xFFFFFE00u; c.tlbFlush(); break;
        default:
            c.pc = instrStart(c);
            return raiseException(c, kVec040Illegal, 16);
        }
        return 7;   // UM 10.5: MOVEC Rn,Rc
    }

    u32 v;
    switch (code) {
    case 0x000: v = c.sfc; break;
    case 0x001: v = c.dfc; break;
    case 0x002: v = c.cacr; break;
    case 0x003: v = c.tc; break;
    case 0x004: v = c.itt0; break;
    case 0x005: v = c.itt1; break;
    case 0x006: v = c.dtt0; break;
    case 0x007: v = c.dtt1; break;
    case 0x800: v = c.uspValue(); break;
    case 0x801: v = c.vbr; break;
    case 0x803: v = c.mspValue(); break;
    case 0x804: v = c.ispValue(); break;
    case 0x805: v = c.mmusr; break;
    case 0x806: v = c.urp; break;
    case 0x807: v = c.srp; break;
    default:
        c.pc = instrStart(c);
        return raiseException(c, kVec040Illegal, 16);
    }
    gen = v;
    return 11;   // UM 10.5: MOVEC Rc,Rn
}

// MOVES address-space access: translate under an explicit function code
// (super = FC bit 2, instruction space = FC 2/6), then hit the bus. Only
// aligned pieces reach the bus, as everywhere else.
u32 CpuOps040::fcRead(M68040& c, u32 addr, int size, u32 fc) {
    const bool super = (fc & 4) != 0;
    const bool instr = (fc & 3) == 2;
    if (size == 1 && (addr & 1)) {
        const u32 hi = fcRead(c, addr, 0, fc);
        return (hi << 8) | fcRead(c, addr + 1, 0, fc);
    }
    if (size == 2 && (addr & 3)) {
        const u32 hi = fcRead(c, addr, 1, fc);
        return (hi << 16) | fcRead(c, addr + 2, 1, fc);
    }
    if (c.is68030())
        return c.readCached030(addr, size, fc & 7u, false);
    u32 pa = addr;
    if (c.mmuEnabled()) pa = c.translateFc(addr, false, super, instr);
    try {
        if (size == 0) return c.bus_.read8(pa);
        if (size == 1) return c.bus_.read16(pa);
        return c.bus_.read32(pa);
    } catch (const BusFault&) {
        throw M68040::AccessFault{addr, true, size == 0 ? 1 : size == 1 ? 2 : 4, instr, false};
    }
}

void CpuOps040::fcWrite(M68040& c, u32 addr, u32 v, int size, u32 fc) {
    const bool super = (fc & 4) != 0;
    const bool instr = (fc & 3) == 2;
    if (size == 1 && (addr & 1)) {
        fcWrite(c, addr, v >> 8, 0, fc);
        fcWrite(c, addr + 1, v & 0xFF, 0, fc);
        return;
    }
    if (size == 2 && (addr & 3)) {
        fcWrite(c, addr, v >> 16, 1, fc);
        fcWrite(c, addr + 2, v & 0xFFFF, 1, fc);
        return;
    }
    if (c.is68030()) {
        c.writeCached030(addr, v, size, fc & 7u);
        return;
    }
    u32 pa = addr;
    if (c.mmuEnabled()) pa = c.translateFc(addr, true, super, instr);
    try {
        if (size == 0) c.bus_.write8(pa, static_cast<u8>(v));
        else if (size == 1) c.bus_.write16(pa, static_cast<u16>(v));
        else c.bus_.write32(pa, v);
    } catch (const BusFault&) {
        throw M68040::AccessFault{addr, false, size == 0 ? 1 : size == 1 ? 2 : 4, instr, false};
    }
}

int CpuOps040::opMoves(M68040& c, u16 op) {
    if (!flag(c, kS040)) return privilegeViolation(c);
    const int size = (op >> 6) & 3;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u16 ext = c.fetch16();
    const int gr = (ext >> 12) & 7;
    const bool isAddr = (ext & 0x8000) != 0;
    const bool toMem = (ext & 0x0800) != 0;
    const u32 addr = calcEA(c, mode, reg, size);

    if (toMem) {
        const u32 v = isAddr ? c.a[gr] : c.d[gr];
        fcWrite(c, addr, v & maskFor(size), size, c.dfc);
    } else {
        u32 v = fcRead(c, addr, size, c.sfc);
        if (isAddr) c.a[gr] = static_cast<u32>(signExtend(v, size));
        else        writeSized(c.d[gr], v, size);
    }
    if (c.is68030()) {
        // MC68030UM 11.6.7: the command extension word is part of a
        // calculate-immediate EA stage; the operation proper is 5 clocks for
        // Rn->EA and 7 for EA->Rn.
        return (toMem ? 5 : 7) +
               eaCalcImmediateTime030(c, eaIndex(mode, reg));
    }
    return (toMem ? 13 : 23) + eaTime(eaIndex(mode, reg));
}

// MOVE16: 16-byte aligned line copy. The low four address bits are ignored,
// and the postincrement forms step by 16.
int CpuOps040::opMove16(M68040& c, u16 op) {
    u32 src, dst;
    int srcPost = -1, dstPost = -1;   // An needing +16 afterwards

    if ((op & 0xFFF8) == 0xF620) {    // (Ax)+,(Ay)+
        const u16 ext = c.fetch16();
        const int ax = op & 7;
        const int ay = (ext >> 12) & 7;
        src = c.a[ax] & ~15u;
        dst = c.a[ay] & ~15u;
        srcPost = ax;
        dstPost = ay;
        // Same register: one increment total, as measured on hardware.
        if (ax == ay) dstPost = -1;
    } else {
        const int an = op & 7;
        const u32 abs = c.fetch32();
        switch ((op >> 3) & 3) {
        case 0: src = c.a[an] & ~15u; dst = abs & ~15u; srcPost = an; break;   // (An)+ -> abs
        case 1: src = abs & ~15u; dst = c.a[an] & ~15u; dstPost = an; break;   // abs -> (An)+
        case 2: src = c.a[an] & ~15u; dst = abs & ~15u; break;                 // (An) -> abs
        default: src = abs & ~15u; dst = c.a[an] & ~15u; break;                // abs -> (An)
        }
    }

    u32 line[4];
    for (int i = 0; i < 4; ++i) line[i] = c.rd32(src + static_cast<u32>(i) * 4);
    for (int i = 0; i < 4; ++i) c.wr32(dst + static_cast<u32>(i) * 4, line[i]);
    if (srcPost >= 0) c.a[srcPost] += 16;
    if (dstPost >= 0) c.a[dstPost] += 16;
    return 8;
}

int CpuOps040::opCinvCpush(M68040& c, u16 op) {
    if (!flag(c, kS040)) return privilegeViolation(c);
    // Caches are register-visible only; live-memory reads are always
    // coherent, so invalidate/push has no functional effect to model.
    const int scope = (op >> 3) & 3;   // 1 line, 2 page, 3 all
    return [&] {   // UM 10.3/10.4: page ops walk every set
        const bool push = (op & 0x0020) != 0;
        if (scope == 2) return push ? 267 : 266;
        if (scope == 3) return push ? 267 : 9;
        return push ? 6 : 9;
    }();
}

int CpuOps040::opPflush(M68040& c, u16 op) {
    if (!flag(c, kS040)) return privilegeViolation(c);
    const int variant = (op >> 3) & 3;   // 0 PFLUSHN (An), 1 PFLUSH (An),
                                         // 2 PFLUSHAN, 3 PFLUSHA
    if (variant <= 1) c.tlbFlushPage(c.a[op & 7]);
    else              c.tlbFlush();
    return (((op >> 3) & 3) == 2) ? 27 : 11;   // UM 10.5
}

// PTEST: walk the tables for (An) in the DFC-selected space and report in
// MMUSR. Bits: 31-12 PA, 11 B (walk hit a bus error), 10 G, 9-8 U1/U0,
// 7 S, 6-5 CM, 4 M, 2 W, 1 T (transparent hit), 0 R (resident).
int CpuOps040::opPtest(M68040& c, u16 op) {
    if (!flag(c, kS040)) return privilegeViolation(c);
    const bool isWrite = (op & 0x0020) == 0;   // F548 = PTESTW, F568 = PTESTR
    const u32 la = c.a[op & 7];
    const bool super = (c.dfc & 4) != 0;
    const bool instr = (c.dfc & 3) == 2;

    // Transparent hit reports T|R with the address unchanged.
    const u32 t0 = instr ? c.itt0 : c.dtt0;
    const u32 t1 = instr ? c.itt1 : c.dtt1;
    if (c.ttrMatch(t0, la, super, isWrite) || c.ttrMatch(t1, la, super, isWrite)) {
        c.mmusr = (la & 0xFFFFF000u) | 0x03u;   // T | R
        return 25;
    }

    const bool page8k = (c.tc & 0x4000u) != 0;
    const u32 ri = la >> 25;
    const u32 pi = (la >> 18) & 0x7Fu;
    const u32 pgi = page8k ? ((la >> 13) & 0x1Fu) : ((la >> 12) & 0x3Fu);
    u32 sr = 0;
    try {
        // PTESTR sets the U bits along the search; PTESTW additionally sets
        // the page descriptor's M bit (PRM 6-70).
        const u32 rp = (super ? c.srp : c.urp) & 0xFFFFFE00u;
        const u32 d1a = rp + ri * 4;
        u32 d1 = c.bus_.read32(d1a);
        if ((d1 & 2u) == 0) { c.mmusr = 0; return 25; }
        if (d1 & 4u) sr |= 0x04u;
        if (!(d1 & 8u)) c.bus_.write32(d1a, d1 | 8u);
        const u32 d2a = (d1 & 0xFFFFFE00u) + pi * 4;
        u32 d2 = c.bus_.read32(d2a);
        if ((d2 & 2u) == 0) { c.mmusr = 0; return 25; }
        if (d2 & 4u) sr |= 0x04u;
        if (!(d2 & 8u)) c.bus_.write32(d2a, d2 | 8u);
        u32 d3a = (d2 & (page8k ? 0xFFFFFF80u : 0xFFFFFF00u)) + pgi * 4;
        u32 d3 = c.bus_.read32(d3a);
        if ((d3 & 3u) == 2u) { d3a = d3 & 0xFFFFFFFCu; d3 = c.bus_.read32(d3a); }
        if ((d3 & 3u) == 0u || (d3 & 3u) == 2u) { c.mmusr = 0; return 25; }
        u32 d3New = d3 | 8u;
        if (isWrite && !(sr & 0x04u) && !(d3 & 4u)) d3New |= 0x10u;
        if (d3New != d3) { c.bus_.write32(d3a, d3New); d3 = d3New; }
        // MMUSR: PA | G,U1/U0,S,CM,M,W straight from the page descriptor,
        // W also accumulated from the upper levels, R for resident.
        c.mmusr = (d3 & (page8k ? 0xFFFFE000u : 0xFFFFF000u)) |
                  (d3 & 0x7F4u) | (sr & 0x04u) | 0x01u;
    } catch (const BusFault&) {
        c.mmusr = 0x800u;                    // B: bus error during the walk
    }
    return 25;   // UM 10.5: typical three-level search
}

// MC68030 integrated-PMMU general instruction.  The first word is F000 plus
// an effective address and the second (command) word selects PMOVE, PLOAD,
// PFLUSH, or PTEST.  These are the 68851-compatible encodings documented in
// M68000PRM section 6; 040's one-word F5xx forms must never reach this path.
int CpuOps040::opPmmu030(M68040& c, u16 op) {
    if (!c.is68030()) return opFLine(c, op);
    if (!flag(c, kS040)) return privilegeViolation(c);

    const int mode = (op >> 3) & 7;
    const int reg = op & 7;
    const u16 cmd = c.fetch16();

    auto bad = [&]() {
        c.pc = instrStart(c);
        return raiseException(c, kVec040Illegal, 16);
    };
    auto controlEa = [&]() -> u32 {
        if (!eaValid(mode, reg, kEaControlAlter)) throw 0;
        return calcEA(c, mode, reg, 2);
    };
    auto functionCode = [&](u16 field, u32& fc) -> bool {
        field &= 0x1F;
        if ((field & 0x18) == 0x10) { fc = field & 7; return true; }
        if ((field & 0x18) == 0x08) { fc = c.d[field & 7] & 7; return true; }
        if (field == 0) { fc = c.sfc & 7; return true; }
        if (field == 1) { fc = c.dfc & 7; return true; }
        return false;
    };

    try {
        // PMOVE TC/SRP/CRP: 010 ppp R/W FD 00000000.
        if ((cmd & 0xE000u) == 0x4000u) {
            const int preg = (cmd >> 10) & 7;
            const bool toMemory = (cmd & 0x0200u) != 0;
            const bool noFlush = (cmd & 0x0100u) != 0;
            if ((cmd & 0x00FFu) != 0 || (preg != 0 && preg != 2 && preg != 3))
                return bad();
            const u32 ea = controlEa();
            if (preg == 0) {                       // TC, long
                if (toMemory) c.wr32(ea, c.tc);
                else {
                    c.tc = c.rd32(ea);
                    if (!noFlush) c.tlbFlush();
                }
            } else {                               // SRP/CRP, quad
                u64& rp = preg == 2 ? c.srp030 : c.crp;
                if (toMemory) {
                    c.wr32(ea, static_cast<u32>(rp >> 32));
                    c.wr32(ea + 4, static_cast<u32>(rp));
                } else {
                    rp = (static_cast<u64>(c.rd32(ea)) << 32) | c.rd32(ea + 4);
                    if (!noFlush) c.tlbFlush();
                }
            }
            return 16 + eaTime(eaIndex(mode, reg));
        }

        // PMOVE MMUSR: 011000 R/W 000000000.
        if ((cmd & 0xFC00u) == 0x6000u && (cmd & 0x01FFu) == 0) {
            const bool toMemory = (cmd & 0x0200u) != 0;
            const u32 ea = controlEa();
            if (toMemory) c.wr16(ea, static_cast<u16>(c.mmusr));
            else c.mmusr = c.rd16(ea);
            return 12 + eaTime(eaIndex(mode, reg));
        }

        // PMOVE TT0/TT1: 000 ppp R/W FD 00000000, ppp=010/011.
        if ((cmd & 0xE000u) == 0 &&
            (((cmd >> 10) & 7) == 2 || ((cmd >> 10) & 7) == 3) &&
            (cmd & 0x00FFu) == 0) {
            const int preg = (cmd >> 10) & 7;
            const bool toMemory = (cmd & 0x0200u) != 0;
            const bool noFlush = (cmd & 0x0100u) != 0;
            const u32 ea = controlEa();
            u32& tt = preg == 2 ? c.tt0 : c.tt1;
            if (toMemory) c.wr32(ea, tt);
            else {
                tt = c.rd32(ea);
                if (!noFlush) c.tlbFlush();
            }
            return 16 + eaTime(eaIndex(mode, reg));
        }

        // PLOAD: 001000 R/W 0000 FC.
        if ((cmd & 0xFC00u) == 0x2000u && (cmd & 0x01E0u) == 0) {
            const bool read = (cmd & 0x0200u) != 0;
            u32 fc = 0;
            if (!functionCode(cmd, fc)) return bad();
            const u32 la = controlEa();
            c.tlbFlushPage(la);
            // PLOAD performs a table search even with TC.E clear.  The full
            // 030 walker handles that special case; until a table exists a
            // direct address is the only possible result.
            if (c.mmuEnabled()) (void)c.translateFc030(la, !read, fc);
            return 25 + eaTime(eaIndex(mode, reg));
        }

        // PFLUSHA / PFLUSH FC,MASK[,EA]: mode 001/100/110.
        if ((cmd & 0xE000u) == 0x2000u) {
            const int flushMode = (cmd >> 10) & 7;
            if (cmd & 0x0300u) return bad();
            if (flushMode == 1) {
                if ((cmd & 0x00FFu) != 0) return bad();
                c.tlbFlush();
                return 16;
            }
            if (flushMode != 4 && flushMode != 6) return bad();
            u32 fc = 0;
            if (!functionCode(cmd, fc)) return bad();
            (void)fc; // the first ATC representation does not yet split FCs
            if (flushMode == 6) c.tlbFlushPage(controlEa());
            else c.tlbFlush();
            return 18 + (flushMode == 6 ? eaTime(eaIndex(mode, reg)) : 0);
        }

        // PTEST: 100 level R/W A reg FC.
        if ((cmd & 0xE000u) == 0x8000u) {
            const int level = (cmd >> 10) & 7;
            const bool read = (cmd & 0x0200u) != 0;
            const bool returnDescriptor = (cmd & 0x0100u) != 0;
            const int areg = (cmd >> 5) & 7;
            u32 fc = 0;
            if (!functionCode(cmd, fc)) return bad();
            if (!returnDescriptor && areg != 0) return bad();
            const u32 la = controlEa();
            c.mmusr = 0;
            try {
                const u32 pa = c.translateFc030(la, !read, fc);
                c.mmusr = static_cast<u16>(1u); // resident; walker refines status
                if (returnDescriptor) c.a[areg] = pa & ~3u;
            } catch (const M68040::AccessFault& f) {
                c.mmusr = f.atc ? 0x0400u : 0x0800u;
            }
            return 25 + level * 4 + eaTime(eaIndex(mode, reg));
        }
    } catch (int) {
        return bad();
    }

    return bad();
}

} // namespace openmac
