#include "cpu_ops.hpp"

namespace openmac {

int CpuOps::opBcc(M68000& c, u16 op) {
    const int cond = (op >> 8) & 15;
    const u32 base = c.pc;
    s32 disp = static_cast<s8>(op & 0xFF);
    const bool wordDisp = (disp == 0);
    if (wordDisp) disp = static_cast<s16>(c.fetch16());

    if (cond == 1) { // BSR pushes the return before the target fetch can fault
        c.push32(c.pc);
        jumpTo(c, base + static_cast<u32>(disp));
        return 18;
    }
    if (cond == 0 || testCond(c, cond)) { // BRA / taken Bcc
        jumpTo(c, base + static_cast<u32>(disp));
        return 10;
    }
    return wordDisp ? 12 : 8;
}

int CpuOps::opDbcc(M68000& c, u16 op) {
    const int reg = op & 7;
    const u32 base = c.pc;
    const s32 disp = static_cast<s16>(c.fetch16());
    if (testCond(c, (op >> 8) & 15)) return 12;

    const u16 count = static_cast<u16>((c.d[reg] & 0xFFFF) - 1);
    writeSized(c.d[reg], count, 1);
    if (count != 0xFFFF) {
        jumpTo(c, base + static_cast<u32>(disp));
        return 10;
    }
    return 14;
}

int CpuOps::opJmp(M68000& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 addr = calcEA(c, mode, reg, 2);
    jumpTo(c, addr);
    static constexpr int cyc[12] = {0, 0, 8, 0, 0, 10, 14, 10, 12, 10, 14, 0};
    return cyc[eaIndex(mode, reg)];
}

int CpuOps::opJsr(M68000& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u32 addr = calcEA(c, mode, reg, 2);
    const u32 ret = c.pc;
    jumpTo(c, addr);      // an odd target faults before the return is pushed
    c.push32(ret);
    static constexpr int cyc[12] = {0, 0, 16, 0, 0, 18, 22, 18, 20, 18, 22, 0};
    return cyc[eaIndex(mode, reg)];
}

int CpuOps::opRts(M68000& c, u16) {
    jumpTo(c, c.pop32());
    return 16;
}

int CpuOps::opRtr(M68000& c, u16) {
    c.setCCR(static_cast<u8>(c.pop16()));
    jumpTo(c, c.pop32());
    return 20;
}

int CpuOps::opRte(M68000& c, u16) {
    if (!flag(c, kS)) return privilegeViolation(c);
    const u16 newSR = c.pop16();
    const u32 newPC = c.pop32();
    c.setSR(newSR);
    jumpTo(c, newPC);
    return 20;
}

int CpuOps::opTrap(M68000& c, u16 op) {
    return raiseException(c, kVecTrapBase + (op & 15), 34);
}

int CpuOps::opTrapv(M68000& c, u16) {
    if (flag(c, kV)) return raiseException(c, kVecTrapv, 34);
    return 4;
}

int CpuOps::opChk(M68000& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;
    const s16 bound = static_cast<s16>(readEA(c, mode, reg, 1));
    const s16 v = static_cast<s16>(c.d[dreg] & 0xFFFF);
    const int eaT = eaTimeBW(eaIndex(mode, reg));

    c.sr_ = static_cast<u16>(c.sr_ & ~(kZ | kV | kC));   // hardware clears these
    if (v < 0) {
        setFlag(c, kN, true);
        return raiseException(c, kVecChk, 40 + eaT);
    }
    if (v > bound) {
        setFlag(c, kN, false);
        return raiseException(c, kVecChk, 40 + eaT);
    }
    return 10 + eaT;
}

int CpuOps::opStop(M68000& c, u16) {
    if (!flag(c, kS)) return privilegeViolation(c);
    const u16 imm = c.fetch16();
    c.setSR(imm);
    c.stopped = true;
    return 4;
}

int CpuOps::opReset(M68000& c, u16) {
    if (!flag(c, kS)) return privilegeViolation(c);
    // Asserts /RSTO for 124 clocks; device-reset hook arrives with the machine.
    return 132;
}

int CpuOps::opNop(M68000&, u16) {
    return 4;
}

int CpuOps::opIllegal(M68000& c, u16) {
    c.pc = instrStart(c);
    return raiseException(c, kVecIllegal, 34);
}

int CpuOps::opALine(M68000& c, u16) {
    c.pc = instrStart(c);
    return raiseException(c, kVecALine, 34);
}

int CpuOps::opFLine(M68000& c, u16) {
    c.pc = instrStart(c);
    return raiseException(c, kVecFLine, 34);
}

} // namespace openmac
