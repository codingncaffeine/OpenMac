#pragma once

// Internal to the 68040 CPU implementation. Handler bodies live in the
// cpu040_ops_*.cpp files; everything routes through the 64K dispatch table
// built at startup. Structured after the 68000 core (cpu_ops.hpp), which is
// the style guide; the tables and semantics here are the 68040's own.
//
// Reference: M68040 User's Manual (M68040UM, 1993) sections 2 (integer unit),
// 3 (MMU), 8 (exception processing), 9 (FPU), 10 (instruction timings);
// M68000PRM for encodings. Clean-room from the manuals.

#include "openmac/cpu040.hpp"

#include <array>

namespace openmac {

// SR/CCR bits. The '040 implements T1/T0 (trace all / trace flow), S, M,
// the IPL mask, and X/N/Z/V/C.
inline constexpr u16 kC040  = 0x0001;
inline constexpr u16 kV040  = 0x0002;
inline constexpr u16 kZ040  = 0x0004;
inline constexpr u16 kN040  = 0x0008;
inline constexpr u16 kX040  = 0x0010;
inline constexpr u16 kM040  = 0x1000;
inline constexpr u16 kS040  = 0x2000;
inline constexpr u16 kT0040 = 0x4000;
inline constexpr u16 kT1040 = 0x8000;
inline constexpr u16 kSrMask040 = 0xF71F;   // implemented SR bits on the 68040

// Vector numbers (offsets from VBR, in longs).
inline constexpr int kVec040AccessError   = 2;
inline constexpr int kVec040AddressError  = 3;
inline constexpr int kVec040Illegal       = 4;
inline constexpr int kVec040ZeroDivide    = 5;
inline constexpr int kVec040Chk           = 6;
inline constexpr int kVec040Trapcc        = 7;
inline constexpr int kVec040Privilege     = 8;
inline constexpr int kVec040Trace         = 9;
inline constexpr int kVec040ALine         = 10;
inline constexpr int kVec040FLine         = 11;   // also FP unimplemented instruction
inline constexpr int kVec040FormatError   = 14;
inline constexpr int kVec040Autovector    = 24;   // + level
inline constexpr int kVec040TrapBase      = 32;   // + n
inline constexpr int kVec040FpBsun        = 48;
inline constexpr int kVec040FpInexact     = 49;
inline constexpr int kVec040FpDivZero     = 50;
inline constexpr int kVec040FpUnderflow   = 51;
inline constexpr int kVec040FpOperr       = 52;
inline constexpr int kVec040FpOverflow    = 53;
inline constexpr int kVec040FpSnan        = 54;
inline constexpr int kVec040FpUnsupported = 55;   // packed decimal et al. -> FPSP

struct CpuOps040 {
    using Handler = int (*)(M68040&, u16);

    static std::array<Handler, 65536>& table();

    // ---- size helpers (size: 0=byte 1=word 2=long) ----
    static u32 maskFor(int size) {
        return size == 0 ? 0xFFu : size == 1 ? 0xFFFFu : 0xFFFFFFFFu;
    }
    static u32 signBit(int size) {
        return size == 0 ? 0x80u : size == 1 ? 0x8000u : 0x80000000u;
    }
    static s32 signExtend(u32 v, int size) {
        if (size == 0) return static_cast<s32>(static_cast<s8>(v & 0xFF));
        if (size == 1) return static_cast<s32>(static_cast<s16>(v & 0xFFFF));
        return static_cast<s32>(v);
    }
    static void writeSized(u32& reg, u32 v, int size) {
        if (size == 0)      reg = (reg & 0xFFFFFF00u) | (v & 0xFFu);
        else if (size == 1) reg = (reg & 0xFFFF0000u) | (v & 0xFFFFu);
        else                reg = v;
    }

    // ---- flag helpers ----
    static void setFlag(M68040& c, u16 flag, bool on) {
        if (on) c.sr_ |= flag; else c.sr_ = static_cast<u16>(c.sr_ & ~flag);
    }
    static bool flag(const M68040& c, u16 f) { return (c.sr_ & f) != 0; }

    static void setNZ(M68040& c, u32 v, int size) {
        v &= maskFor(size);
        setFlag(c, kN040, (v & signBit(size)) != 0);
        setFlag(c, kZ040, v == 0);
    }
    static void setLogicFlags(M68040& c, u32 v, int size) {
        setNZ(c, v, size);
        c.sr_ = static_cast<u16>(c.sr_ & ~(kV040 | kC040));
    }
    static u32 addFlags(M68040& c, u32 s, u32 t, int size, bool withX, bool stickyZ = false);
    static u32 subFlags(M68040& c, u32 s, u32 t, int size, bool withX, bool stickyZ = false);
    static void cmpFlags(M68040& c, u32 s, u32 t, int size);

    // ---- effective addresses ----
    // Flattened EA index: 0 Dn, 1 An, 2 (An), 3 (An)+, 4 -(An), 5 d16(An),
    // 6 idx(An), 7 abs.W, 8 abs.L, 9 d16(PC), 10 idx(PC), 11 #imm.
    // Indices 6 and 10 cover the full 68020 extension-word forms (scaled
    // index, base displacement, memory indirect) -- the category masks are
    // unchanged from the 68000 because the new forms live inside existing
    // mode encodings.
    static int  eaIndex(int mode, int reg) { return mode < 7 ? mode : 7 + reg; }
    static int  eaTime(int idx);   // '040 EA fetch clocks (UM section 10)

    enum : u32 {
        EaDn = 1u << 0, EaAn = 1u << 1, EaInd = 1u << 2, EaPostInc = 1u << 3,
        EaPreDec = 1u << 4, EaDisp = 1u << 5, EaIdx8 = 1u << 6, EaAbsW = 1u << 7,
        EaAbsL = 1u << 8, EaPcDisp = 1u << 9, EaPcIdx = 1u << 10, EaImm = 1u << 11,
    };
    static constexpr u32 kEaAll        = 0xFFFu;
    static constexpr u32 kEaDataAddr   = kEaAll & ~(EaAn);
    static constexpr u32 kEaMemAlter   = kEaAll & ~(EaDn | EaAn | EaPcDisp | EaPcIdx | EaImm);
    static constexpr u32 kEaDataAlter  = kEaMemAlter | EaDn;
    static constexpr u32 kEaAlterable  = kEaDataAlter | EaAn;
    static constexpr u32 kEaControl    = EaInd | EaDisp | EaIdx8 | EaAbsW | EaAbsL | EaPcDisp | EaPcIdx;
    static constexpr u32 kEaControlAlter = EaInd | EaDisp | EaIdx8 | EaAbsW | EaAbsL;
    static bool eaValid(int mode, int reg, u32 categories) {
        return (categories & (1u << eaIndex(mode, reg))) != 0;
    }

    // Compute the address for a memory EA (modes 2..7.3), including the full
    // 68020 extension-word forms. Handles An inc/dec with the A7 rule.
    static u32 calcEA(M68040& c, int mode, int reg, int size);

    // Full/brief extension word evaluation with `base` as the base register
    // value (An or PC). Shared by mode 6 and mode 7.3.
    static u32 indexExtension(M68040& c, u32 base);

    static u32 readAt(M68040& c, u32 addr, int size);
    static void writeAt(M68040& c, u32 addr, u32 v, int size);
    static u32 readEA(M68040& c, int mode, int reg, int size);

    // Immediate operand fetch (#imm by size; byte immediates occupy a word).
    static u32 fetchImm(M68040& c, int size);

    static void jumpTo(M68040& c, u32 target);
    static bool testCond(const M68040& c, int cond);

    // ---- exception plumbing ----
    static int raiseException(M68040& c, int vector, int cycles) {
        return c.exceptionFrame0(vector, cycles);
    }
    static int raiseFrame2(M68040& c, int vector, u32 addr, int cycles) {
        return c.exceptionFrame2(vector, addr, cycles);
    }
    static int privilegeViolation(M68040& c) {
        c.pc = c.instrStart_;
        return c.exceptionFrame0(kVec040Privilege, 20);
    }
    static u32 instrStart(const M68040& c) { return c.instrStart_; }

    // Access-error (format $7) entry, including double-fault halt.
    static int enterAccessError(M68040& c, const M68040::AccessFault& f);

    // MOVES-style access through an explicit function code (SFC/DFC space).
    static u32  fcRead(M68040& c, u32 addr, int size, u32 fc);
    static void fcWrite(M68040& c, u32 addr, u32 v, int size, u32 fc);

    // ---- table registration (cpu040_decode.cpp) ----
    static void buildTableInto(std::array<Handler, 65536>& t);

    // ---- handlers: cpu040_ops_move.cpp ----
    static int opMove(M68040&, u16);
    static int opMoveq(M68040&, u16);
    static int opMoveFromSR(M68040&, u16);   // privileged on the '040
    static int opMoveFromCCR(M68040&, u16);
    static int opMoveToCCR(M68040&, u16);
    static int opMoveToSR(M68040&, u16);
    static int opMoveUsp(M68040&, u16);
    static int opLea(M68040&, u16);
    static int opPea(M68040&, u16);
    static int opClr(M68040&, u16);
    static int opScc(M68040&, u16);
    static int opTst(M68040&, u16);          // '020+: An/#imm/PC-relative legal
    static int opExg(M68040&, u16);
    static int opSwap(M68040&, u16);
    static int opExt(M68040&, u16);          // EXT.w/.l + EXTB.L
    static int opLink(M68040&, u16);         // LINK.W
    static int opLinkL(M68040&, u16);        // LINK.L
    static int opUnlk(M68040&, u16);
    static int opMovem(M68040&, u16);
    static int opMovep(M68040&, u16);

    // ---- handlers: cpu040_ops_alu.cpp ----
    static int opAddSub(M68040&, u16);
    static int opAdda(M68040&, u16);
    static int opAddiSubi(M68040&, u16);
    static int opAddqSubq(M68040&, u16);
    static int opAddxSubx(M68040&, u16);
    static int opCmp(M68040&, u16);
    static int opCmpa(M68040&, u16);
    static int opCmpi(M68040&, u16);         // '020+: PC-relative legal
    static int opCmpm(M68040&, u16);
    static int opNeg(M68040&, u16);
    static int opLogic(M68040&, u16);
    static int opLogicImm(M68040&, u16);
    static int opNot(M68040&, u16);
    static int opCas(M68040&, u16);
    static int opCas2(M68040&, u16);
    static int opChk2Cmp2(M68040&, u16);

    // ---- handlers: cpu040_ops_bits.cpp ----
    static int opShiftReg(M68040&, u16);
    static int opShiftMem(M68040&, u16);
    static int opBitOp(M68040&, u16);
    static int opBitField(M68040&, u16);     // BFTST/BFEXTU/BFCHG/BFEXTS/BFCLR/BFFFO/BFSET/BFINS

    // ---- handlers: cpu040_ops_muldiv.cpp ----
    static int opMul(M68040&, u16);          // MULU.W/MULS.W
    static int opMulDivL(M68040&, u16);      // MULU.L/MULS.L/DIVU.L/DIVS.L (0x4C00)
    static int opDiv(M68040&, u16);          // DIVU.W/DIVS.W
    static int opAbcdSbcd(M68040&, u16);
    static int opNbcd(M68040&, u16);
    static int opPackUnpk(M68040&, u16);
    static int opTas(M68040&, u16);

    // ---- handlers: cpu040_ops_flow.cpp ----
    static int opBcc(M68040&, u16);          // 8/16/32-bit displacement
    static int opDbcc(M68040&, u16);
    static int opJmp(M68040&, u16);
    static int opJsr(M68040&, u16);
    static int opRts(M68040&, u16);
    static int opRtr(M68040&, u16);
    static int opRtd(M68040&, u16);
    static int opRte(M68040&, u16);          // format $0/$1/$2/$3/$7 aware
    static int opTrap(M68040&, u16);
    static int opTrapcc(M68040&, u16);
    static int opTrapv(M68040&, u16);
    static int opChk(M68040&, u16);          // CHK.W + CHK.L
    static int opBkpt(M68040&, u16);
    static int opStop(M68040&, u16);
    static int opReset(M68040&, u16);
    static int opNop(M68040&, u16);
    static int opIllegal(M68040&, u16);
    static int opALine(M68040&, u16);
    static int opFLine(M68040&, u16);

    // ---- handlers: cpu040_ops_system.cpp ----
    static int opMovec(M68040&, u16);
    static int opMoves(M68040&, u16);
    static int opMove16(M68040&, u16);
    static int opCinvCpush(M68040&, u16);
    static int opPflush(M68040&, u16);
    static int opPtest(M68040&, u16);

    // ---- handlers: cpu040_fpu.cpp ----
    static int opFpuGen(M68040&, u16);       // F200-F23F general/FMOVE/FMOVEM
    static int opFBcc(M68040&, u16);
    static int opFScc(M68040&, u16);         // + FDBcc + FTRAPcc
    static int opFSave(M68040&, u16);
    static int opFRestore(M68040&, u16);
};

} // namespace openmac
