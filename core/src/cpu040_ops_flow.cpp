#include "cpu040_ops.hpp"

// Flow control for the '040: Bcc with 32-bit displacements, RTD, TRAPcc, the
// long CHK, and the format-aware RTE that understands the '040's own stack
// frames ($0/$1/$2/$3/$7) including the throwaway-frame dance the M bit
// creates around interrupts.
//
// Reference: M68000PRM; M68040UM 8.4 (frames). Clean-room from the manuals.

namespace openmac {

int CpuOps040::opBcc(M68040& c, u16 op) {
    const int cond = (op >> 8) & 15;
    const u32 base = c.pc;
    const u8 displacementCode = static_cast<u8>(op);
    s32 disp = static_cast<s8>(op & 0xFF);
    if (disp == 0) {
        disp = static_cast<s16>(c.fetch16());
    } else if (disp == -1) {   // $FF: 32-bit displacement ('020+)
        disp = static_cast<s32>(c.fetch32());
    }

    if (cond == 1) { // BSR
        c.push32(c.pc);
        jumpTo(c, base + static_cast<u32>(disp));
        return c.is68030() ? 6 : 2;
    }
    if (cond == 0 || testCond(c, cond)) {
        jumpTo(c, base + static_cast<u32>(disp));
        return c.is68030() ? 6 : 2;
    }
    if (c.is68030())
        return displacementCode != 0 && displacementCode != 0xFF ? 4 : 6;
    return 3;   // not-taken costs more than taken on the '040
}

int CpuOps040::opDbcc(M68040& c, u16 op) {
    const int reg = op & 7;
    const u32 base = c.pc;
    const s32 disp = static_cast<s16>(c.fetch16());
    if (testCond(c, (op >> 8) & 15)) return c.is68030() ? 6 : 4;

    const u16 count = static_cast<u16>((c.d[reg] & 0xFFFF) - 1);
    writeSized(c.d[reg], count, 1);
    if (count != 0xFFFF) {
        jumpTo(c, base + static_cast<u32>(disp));
        return c.is68030() ? 6 : 3;
    }
    return c.is68030() ? 10 : 4;
}

int CpuOps040::opJmp(M68040& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 addr = calcEA(c, mode, reg, 2);
    jumpTo(c, addr);
    if (c.is68030()) return 4 + eaJumpTime030(c, eaIndex(mode, reg));
    return 3 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opJsr(M68040& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 addr = calcEA(c, mode, reg, 2);
    const u32 ret = c.pc;
    jumpTo(c, addr);
    c.push32(ret);
    if (c.is68030()) return 4 + eaJumpTime030(c, eaIndex(mode, reg));
    return 3 + eaTime(eaIndex(mode, reg));
}

int CpuOps040::opRts(M68040& c, u16) {
    jumpTo(c, c.pop32());
    return c.is68030() ? 9 : 5;
}

int CpuOps040::opRtr(M68040& c, u16) {
    c.setCCR(static_cast<u8>(c.pop16()));
    jumpTo(c, c.pop32());
    return c.is68030() ? 12 : 7;
}

int CpuOps040::opRtd(M68040& c, u16) {
    const s32 disp = static_cast<s16>(c.fetch16());
    const u32 ret = c.pop32();
    c.a[7] += static_cast<u32>(disp);
    jumpTo(c, ret);
    return c.is68030() ? 10 : 6;
}

// RTE: read the frame in place (a faulting read restarts the RTE), then pop
// by format. A format $1 throwaway frame restores SR -- typically flipping
// back to the master stack -- and re-runs RTE processing there.
int CpuOps040::opRte(M68040& c, u16) {
    if (!flag(c, kS040)) return privilegeViolation(c);
    int throwawayCycles = 0;
    for (;;) {
        const u16 newSR  = c.rd16(c.a[7]);
        const u32 newPC  = c.rd32(c.a[7] + 2);
        const u16 fmtVec = c.rd16(c.a[7] + 6);
        const int fmt = (fmtVec >> 12) & 0xF;
        switch (fmt) {
        case 0x0:
            c.a[7] += 8;
            c.setSR(newSR);
            jumpTo(c, newPC);
            return throwawayCycles + (c.is68030() ? 18 : 13);
        case 0x1:
            c.a[7] += 8;
            c.setSR(newSR);   // back to the stack the real frame lives on
            if (c.is68030()) throwawayCycles += 12;
            continue;
        case 0x2:
        case 0x3:
            c.a[7] += 12;
            c.setSR(newSR);
            jumpTo(c, newPC);
            return throwawayCycles + (c.is68030() ? 18 : 14);
        case 0x7:
            // Access error. Our frames carry no pending writebacks (restart
            // semantics), so the 60-byte frame just pops.
            if (c.is68030()) break;
            c.a[7] += 60;
            c.setSR(newSR);
            jumpTo(c, newPC);
            return 23;
        case 0xA:
            if (!c.is68030()) break;
            c.a[7] += 32;     // 030 short bus-cycle fault frame: 16 words
            c.setSR(newSR);
            jumpTo(c, newPC);
            return throwawayCycles + 36;
        case 0xB:
            if (!c.is68030()) break;
            c.a[7] += 92;     // 030 long bus-cycle fault frame: 46 words
            c.setSR(newSR);
            jumpTo(c, newPC);
            return throwawayCycles + 76;
        default:
            break;
        }
        c.pc = instrStart(c);
        return raiseException(c, kVec040FormatError, c.is68030() ? 18 : 20);
    }
}

int CpuOps040::opTrap(M68040& c, u16 op) {
    return raiseException(c, kVec040TrapBase + (op & 15), c.is68030() ? 18 : 16);
}

int CpuOps040::opTrapcc(M68040& c, u16 op) {
    const int form = op & 7;
    if (form == 2)      (void)c.fetch16();   // optional operand words
    else if (form == 3) (void)c.fetch32();
    if (testCond(c, (op >> 8) & 15)) {
        if (c.is68030()) {
            const int clocks = form == 2 ? 24 : form == 3 ? 26 : 22;
            return raiseFrame2(c, kVec040Trapcc, instrStart(c), clocks);
        }
        return raiseFrame2(c, kVec040Trapcc, instrStart(c), 19);
    }
    if (c.is68030()) return form == 2 ? 6 : form == 3 ? 8 : 4;
    return 5;
}

int CpuOps040::opTrapv(M68040& c, u16) {
    if (flag(c, kV040))
        return raiseFrame2(c, kVec040Trapcc, instrStart(c), c.is68030() ? 22 : 19);
    return c.is68030() ? 4 : 5;
}

int CpuOps040::opChk(M68040& c, u16 op) {
    const bool isLong = ((op >> 6) & 7) == 4;   // '020+ CHK.L
    const int size = isLong ? 2 : 1;
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    const s32 bound = signExtend(readEA(c, mode, reg, size), size);
    const s32 v = signExtend(c.d[dreg], size);
    const int eaT = c.is68030() ? eaFetchTime030(eaIndex(mode, reg), size)
                                : eaTime(eaIndex(mode, reg));

    c.sr_ = static_cast<u16>(c.sr_ & ~(kZ040 | kV040 | kC040));
    if (v < 0) {
        setFlag(c, kN040, true);
        return raiseFrame2(c, kVec040Chk, instrStart(c),
                           (c.is68030() ? 28 : 16) + eaT);
    }
    if (v > bound) {
        setFlag(c, kN040, false);
        return raiseFrame2(c, kVec040Chk, instrStart(c),
                           (c.is68030() ? 28 : 16) + eaT);
    }
    return 8 + eaT;
}

int CpuOps040::opBkpt(M68040& c, u16) {
    // No breakpoint acknowledge cycle here: the '040 takes an illegal
    // instruction exception when nothing answers.
    c.pc = instrStart(c);
    return raiseException(c, kVec040Illegal, c.is68030() ? 18 : 16);
}

int CpuOps040::opStop(M68040& c, u16) {
    if (!flag(c, kS040)) return privilegeViolation(c);
    const u16 imm = c.fetch16();
    c.setSR(imm);
    c.stopped = true;
    return 8;
}

int CpuOps040::opReset(M68040& c, u16) {
    if (!flag(c, kS040)) return privilegeViolation(c);
    if (c.onResetInstruction) c.onResetInstruction();   // /RSTO to peripherals
    return c.is68030() ? 518 : 521;
}

int CpuOps040::opNop(M68040& c, u16) {
    return c.is68030() ? 2 : 8;
}

int CpuOps040::opIllegal(M68040& c, u16) {
    c.pc = instrStart(c);
    return raiseException(c, kVec040Illegal, c.is68030() ? 18 : 16);
}

int CpuOps040::opALine(M68040& c, u16 opcode) {
    c.pc = instrStart(c);
    if (c.onTrap) c.onTrap(opcode, c.pc);
    return raiseException(c, kVec040ALine, c.is68030() ? 18 : 16);
}

int CpuOps040::opFLine(M68040& c, u16) {
    c.pc = instrStart(c);
    return raiseException(c, kVec040FLine, c.is68030() ? 18 : 16);
}

} // namespace openmac
