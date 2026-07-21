#pragma once

// Internal to the CPU implementation. Handler bodies live in the cpu_ops_*.cpp
// files; everything routes through the 64K dispatch table built at startup.

#include "openmac/cpu.hpp"

#include <array>

namespace openmac {

// SR/CCR bits
inline constexpr u16 kC = 0x0001;
inline constexpr u16 kV = 0x0002;
inline constexpr u16 kZ = 0x0004;
inline constexpr u16 kN = 0x0008;
inline constexpr u16 kX = 0x0010;
inline constexpr u16 kS = 0x2000;
inline constexpr u16 kT = 0x8000;
inline constexpr u16 kSrMask = 0xA71F;   // implemented SR bits on the 68000

// Thrown on odd word/long access; step() converts it to a group-0 exception.
struct AddressError {
    u32  addr;
    bool read;
    bool instruction;   // true when raised by an instruction-stream fetch
    int  pcBias = -2;   // data faults push pc-2; a few microcode paths push pc
};

// Vector numbers
inline constexpr int kVecBusError      = 2;
inline constexpr int kVecAddressError  = 3;
inline constexpr int kVecIllegal       = 4;
inline constexpr int kVecZeroDivide    = 5;
inline constexpr int kVecChk           = 6;
inline constexpr int kVecTrapv         = 7;
inline constexpr int kVecPrivilege     = 8;
inline constexpr int kVecTrace         = 9;
inline constexpr int kVecALine         = 10;
inline constexpr int kVecFLine         = 11;
inline constexpr int kVecAutovector    = 24;  // + level
inline constexpr int kVecTrapBase      = 32;  // + n

struct CpuOps {
    using Handler = int (*)(M68000&, u16);

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
    static void setFlag(M68000& c, u16 flag, bool on) {
        if (on) c.sr_ |= flag; else c.sr_ = static_cast<u16>(c.sr_ & ~flag);
    }
    static bool flag(const M68000& c, u16 f) { return (c.sr_ & f) != 0; }

    static void setNZ(M68000& c, u32 v, int size) {
        v &= maskFor(size);
        setFlag(c, kN, (v & signBit(size)) != 0);
        setFlag(c, kZ, v == 0);
    }
    // logic ops: N/Z from result, V=C=0
    static void setLogicFlags(M68000& c, u32 v, int size) {
        setNZ(c, v, size);
        c.sr_ = static_cast<u16>(c.sr_ & ~(kV | kC));
    }
    // Full add flags (X updated). res = s + t (+carryIn)
    static u32 addFlags(M68000& c, u32 s, u32 t, int size, bool withX, bool setXZ = true);
    // Full sub flags (X updated). res = t - s (-borrowIn)  (t = destination)
    static u32 subFlags(M68000& c, u32 s, u32 t, int size, bool withX, bool setXZ = true);
    // CMP: like sub but X untouched, Z set normally
    static void cmpFlags(M68000& c, u32 s, u32 t, int size);

    // ---- effective addresses ----
    // Flattened EA index: 0 Dn, 1 An, 2 (An), 3 (An)+, 4 -(An), 5 d16(An),
    // 6 d8(An,Xn), 7 abs.W, 8 abs.L, 9 d16(PC), 10 d8(PC,Xn), 11 #imm
    static int  eaIndex(int mode, int reg) { return mode < 7 ? mode : 7 + reg; }
    static int  eaTimeBW(int idx);
    static int  eaTimeL(int idx);

    // EA category masks for decode-time validity
    enum : u32 {
        EaDn = 1u << 0, EaAn = 1u << 1, EaInd = 1u << 2, EaPostInc = 1u << 3,
        EaPreDec = 1u << 4, EaDisp = 1u << 5, EaIdx8 = 1u << 6, EaAbsW = 1u << 7,
        EaAbsL = 1u << 8, EaPcDisp = 1u << 9, EaPcIdx = 1u << 10, EaImm = 1u << 11,
    };
    static constexpr u32 kEaAll        = 0xFFFu;
    static constexpr u32 kEaDataAddr   = kEaAll & ~(EaAn);                    // data
    static constexpr u32 kEaMemAlter   = kEaAll & ~(EaDn | EaAn | EaPcDisp | EaPcIdx | EaImm);
    static constexpr u32 kEaDataAlter  = kEaMemAlter | EaDn;
    static constexpr u32 kEaAlterable  = kEaDataAlter | EaAn;
    static constexpr u32 kEaControl    = EaInd | EaDisp | EaIdx8 | EaAbsW | EaAbsL | EaPcDisp | EaPcIdx;
    static constexpr u32 kEaControlAlter = EaInd | EaDisp | EaIdx8 | EaAbsW | EaAbsL;
    static bool eaValid(int mode, int reg, u32 categories) {
        return (categories & (1u << eaIndex(mode, reg))) != 0;
    }

    // Compute the address for a memory EA (modes 2..7.3). Handles An
    // increment/decrement including the A7 byte-keeps-even rule.
    static u32 calcEA(M68000& c, int mode, int reg, int size);

    // Predecrement for the ADDX/SUBX/ABCD/SBCD family: long operands step
    // the register -2, touch the low word first (faulting there with the
    // register left at initial-2), then -2 again.
    static u32 calcPredecLowFirst(M68000& c, int reg, int size);

    // Read a source operand of any EA (including Dn/An/#imm).
    static u32 readEA(M68000& c, int mode, int reg, int size);

    // read + writeback pair for read-modify-write on a computed address
    static u32 readAt(M68000& c, u32 addr, int size);
    static void writeAt(M68000& c, u32 addr, u32 v, int size);

    // Raise an address error if a jump target is odd, else set pc.
    static void jumpTo(M68000& c, u32 target);

    // ---- condition codes (Bcc/Scc/DBcc) ----
    static bool testCond(const M68000& c, int cond);

    // d8(An,Xn)/d8(PC,Xn) brief extension word
    static u32 briefExtension(M68000& c, u32 base);

    // Group-0 (address error) entry, including double-fault halt.
    static int enterAddressError(M68000& c, const AddressError& ae);

    // Exception plumbing for handlers (CpuOps has friend access; the
    // file-local helpers in the ops files do not).
    static int raiseException(M68000& c, int vector, int cycles) {
        return c.exception(vector, cycles);
    }
    static int privilegeViolation(M68000& c) {
        c.pc = c.instrStart_;
        return c.exception(kVecPrivilege, 34);
    }
    static u32 instrStart(const M68000& c) { return c.instrStart_; }

    // ---- table registration (cpu_decode.cpp) ----
    static void buildTableInto(std::array<Handler, 65536>& t);

    // ---- handlers: cpu_ops_move.cpp ----
    static int opMove(M68000&, u16);       // MOVE.b/w/l incl. MOVEA
    static int opMoveq(M68000&, u16);
    static int opMoveFromSR(M68000&, u16);
    static int opMoveToCCR(M68000&, u16);
    static int opMoveToSR(M68000&, u16);
    static int opMoveUsp(M68000&, u16);
    static int opLea(M68000&, u16);
    static int opPea(M68000&, u16);
    static int opClr(M68000&, u16);
    static int opScc(M68000&, u16);
    static int opTst(M68000&, u16);
    static int opExg(M68000&, u16);
    static int opSwap(M68000&, u16);
    static int opExt(M68000&, u16);
    static int opLink(M68000&, u16);
    static int opUnlk(M68000&, u16);
    static int opMovem(M68000&, u16);
    static int opMovep(M68000&, u16);

    // ---- handlers: cpu_ops_alu.cpp ----
    static int opAddSub(M68000&, u16);     // ADD/SUB <ea>,Dn / Dn,<ea>
    static int opAdda(M68000&, u16);       // ADDA/SUBA
    static int opAddiSubi(M68000&, u16);   // ADDI/SUBI
    static int opAddqSubq(M68000&, u16);   // ADDQ/SUBQ
    static int opAddxSubx(M68000&, u16);   // ADDX/SUBX
    static int opCmp(M68000&, u16);        // CMP <ea>,Dn
    static int opCmpa(M68000&, u16);
    static int opCmpi(M68000&, u16);
    static int opCmpm(M68000&, u16);
    static int opNeg(M68000&, u16);        // NEG/NEGX
    static int opLogic(M68000&, u16);      // AND/OR/EOR register forms
    static int opLogicImm(M68000&, u16);   // ANDI/ORI/EORI (incl. to CCR/SR)
    static int opNot(M68000&, u16);

    // ---- handlers: cpu_ops_shift_bits.cpp ----
    static int opShiftReg(M68000&, u16);   // ASx/LSx/ROx/ROXx Dn or #,Dn
    static int opShiftMem(M68000&, u16);   // word memory shifts
    static int opBitOp(M68000&, u16);      // BTST/BCHG/BCLR/BSET

    // ---- handlers: cpu_ops_muldiv_bcd.cpp ----
    static int opMul(M68000&, u16);        // MULU/MULS
    static int opDiv(M68000&, u16);        // DIVU/DIVS
    static int opAbcdSbcd(M68000&, u16);
    static int opNbcd(M68000&, u16);
    static int opTas(M68000&, u16);

    // ---- handlers: cpu_ops_flow.cpp ----
    static int opBcc(M68000&, u16);        // Bcc/BRA/BSR
    static int opDbcc(M68000&, u16);
    static int opJmp(M68000&, u16);
    static int opJsr(M68000&, u16);
    static int opRts(M68000&, u16);
    static int opRtr(M68000&, u16);
    static int opRte(M68000&, u16);
    static int opTrap(M68000&, u16);
    static int opTrapv(M68000&, u16);
    static int opChk(M68000&, u16);
    static int opStop(M68000&, u16);
    static int opReset(M68000&, u16);
    static int opNop(M68000&, u16);
    static int opIllegal(M68000&, u16);
    static int opALine(M68000&, u16);
    static int opFLine(M68000&, u16);
};

} // namespace openmac
