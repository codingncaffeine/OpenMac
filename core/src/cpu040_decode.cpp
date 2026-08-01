#include "cpu040_ops.hpp"

// Builds the 68040's 64K opcode dispatch table. Every encoding not claimed
// here stays on opIllegal (vector 4); $Axxx/$Fxxx trap through their vectors.
// Differences from the 68000 table are the 68020-era instructions (bitfields,
// CAS, CHK2/CMP2, long MUL/DIV, PACK/UNPK, RTD, TRAPcc, MOVEC/MOVES, LINK.L,
// EXTB, CMPI/TST extended EAs, MOVE from CCR, 32-bit Bcc) and the '040's own
// MOVE16, CINV/CPUSH, PFLUSH/PTEST and FPU blocks. CALLM/RTM (68020-only)
// stay illegal, as on silicon.
//
// Reference: M68000PRM instruction encodings; M68040UM 2/3/9 for the
// '040-specific blocks. Clean-room from the manuals.

namespace openmac {

void CpuOps040::buildTableInto(std::array<Handler, 65536>& t) {
    for (u32 opv = 0; opv < 65536; ++opv) {
        const u16 op    = static_cast<u16>(opv);
        const int mode  = (op >> 3) & 7;
        const int reg   = op & 7;
        const int size2 = (op >> 6) & 3;   // 0=b 1=w 2=l 3=other
        const int bit8  = (op >> 8) & 1;
        Handler h = nullptr;

        switch (op >> 12) {
        case 0x0: {
            // CAS / CAS2 sit on the "size2 == 3" rows of the immediate groups.
            if ((op & 0xF9C0) == 0x00C0 && (op & 0x0600) != 0x0600) {
                // CHK2/CMP2: 0000 0ss0 11 <ea>, ss = 00/01/10
                if (eaValid(mode, reg, kEaControl)) h = &opChk2Cmp2;
                break;
            }
            if ((op & 0xF9C0) == 0x08C0 && ((op >> 9) & 3) != 0) {
                // CAS: 0000 1ss0 11 <ea>, ss = 01 b, 10 w, 11 l. The ss = 00
                // row ($08C0) is static BSET and falls through to it.
                if (op == 0x0CFC || op == 0x0EFC) h = &opCas2;
                else if (eaValid(mode, reg, kEaMemAlter)) h = &opCas;
                break;
            }
            if (bit8) {
                if (mode == 1) {
                    h = &opMovep;
                } else {
                    const int kind = size2;   // 0 BTST 1 BCHG 2 BCLR 3 BSET
                    const u32 cat = (kind == 0) ? kEaDataAddr : kEaDataAlter;
                    if (eaValid(mode, reg, cat)) h = &opBitOp;
                }
                break;
            }
            switch ((op >> 9) & 7) {
            case 0: // ORI
                if (op == 0x003C || op == 0x007C) h = &opLogicImm;
                else if (size2 < 3 && eaValid(mode, reg, kEaDataAlter)) h = &opLogicImm;
                break;
            case 1: // ANDI
                if (op == 0x023C || op == 0x027C) h = &opLogicImm;
                else if (size2 < 3 && eaValid(mode, reg, kEaDataAlter)) h = &opLogicImm;
                break;
            case 2: // SUBI
            case 3: // ADDI
                if (size2 < 3 && eaValid(mode, reg, kEaDataAlter)) h = &opAddiSubi;
                break;
            case 4: { // static bit ops
                const int kind = size2;
                const u32 cat = (kind == 0) ? (kEaDataAddr & ~EaImm) : kEaDataAlter;
                if (eaValid(mode, reg, cat)) h = &opBitOp;
                break;
            }
            case 5: // EORI
                if (op == 0x0A3C || op == 0x0A7C) h = &opLogicImm;
                else if (size2 < 3 && eaValid(mode, reg, kEaDataAlter)) h = &opLogicImm;
                break;
            case 6: // CMPI ('020+: PC-relative destinations become legal)
                if (size2 < 3 &&
                    eaValid(mode, reg, kEaDataAlter | EaPcDisp | EaPcIdx))
                    h = &opCmpi;
                break;
            case 7: // MOVES
                if (size2 < 3 && eaValid(mode, reg, kEaMemAlter)) h = &opMoves;
                break;
            default:
                break;
            }
            break;
        }

        case 0x1: case 0x2: case 0x3: { // MOVE / MOVEA
            const int size = (op >> 12) == 1 ? 0 : (op >> 12) == 2 ? 2 : 1;
            const int dstMode = (op >> 6) & 7;
            const int dstReg  = (op >> 9) & 7;
            const u32 srcCat = (size == 0) ? (kEaAll & ~EaAn) : kEaAll;
            if (!eaValid(mode, reg, srcCat)) break;
            if (dstMode == 1) {
                if (size != 0) h = &opMove;   // MOVEA
            } else if (eaValid(dstMode, dstReg, kEaDataAlter)) {
                h = &opMove;
            }
            break;
        }

        case 0x4: {
            if (bit8) {
                const int om = (op >> 6) & 7;
                if (om == 4) { // CHK.L ('020+)
                    if (eaValid(mode, reg, kEaDataAddr)) h = &opChk;
                } else if (om == 6) { // CHK.W
                    if (eaValid(mode, reg, kEaDataAddr)) h = &opChk;
                } else if (om == 7) { // LEA -- and EXTB.L on Dn ($49C0)
                    if (((op >> 9) & 7) == 4 && mode == 0) h = &opExt;
                    else if (eaValid(mode, reg, kEaControl)) h = &opLea;
                }
                break;
            }
            switch ((op >> 9) & 7) {
            case 0: // NEGX / MOVE from SR (privileged on '010+)
                if (size2 < 3) { if (eaValid(mode, reg, kEaDataAlter)) h = &opNeg; }
                else if (eaValid(mode, reg, kEaDataAlter)) h = &opMoveFromSR;
                break;
            case 1: // CLR / MOVE from CCR ('010+)
                if (size2 < 3) { if (eaValid(mode, reg, kEaDataAlter)) h = &opClr; }
                else if (eaValid(mode, reg, kEaDataAlter)) h = &opMoveFromCCR;
                break;
            case 2: // NEG / MOVE to CCR
                if (size2 < 3) { if (eaValid(mode, reg, kEaDataAlter)) h = &opNeg; }
                else if (eaValid(mode, reg, kEaDataAddr)) h = &opMoveToCCR;
                break;
            case 3: // NOT / MOVE to SR
                if (size2 < 3) { if (eaValid(mode, reg, kEaDataAlter)) h = &opNot; }
                else if (eaValid(mode, reg, kEaDataAddr)) h = &opMoveToSR;
                break;
            case 4: // NBCD+LINK.L / SWAP+BKPT+PEA / EXT+MOVEM
                if (size2 == 0) {
                    if ((op & 0xFFF8) == 0x4808) h = &opLinkL;
                    else if (eaValid(mode, reg, kEaDataAlter)) h = &opNbcd;
                } else if (size2 == 1) {
                    if (mode == 0) h = &opSwap;
                    else if (mode == 1) h = &opBkpt;
                    else if (eaValid(mode, reg, kEaControl)) h = &opPea;
                } else {
                    if (mode == 0) h = &opExt;
                    else if (eaValid(mode, reg, kEaControlAlter | EaPreDec)) h = &opMovem;
                }
                break;
            case 5: // TST ('020+: An/#imm/PC-rel legal) / TAS / ILLEGAL
                if (size2 < 3) {
                    u32 cat = kEaAll;
                    if (size2 == 0) cat &= ~EaAn;   // TST.B An stays illegal
                    if (eaValid(mode, reg, cat)) h = &opTst;
                } else {
                    if (op == 0x4AFC) h = &opIllegal;
                    else if (eaValid(mode, reg, kEaDataAlter)) h = &opTas;
                }
                break;
            case 6: // MULx.L/DIVx.L ('020+) / MOVEM mem->reg
                if ((op & 0xFFC0) == 0x4C00) {
                    if (eaValid(mode, reg, kEaDataAddr)) h = &opMulDivL;
                } else if ((op & 0xFFC0) == 0x4C40) {
                    if (eaValid(mode, reg, kEaDataAddr)) h = &opMulDivL;
                } else if (size2 >= 2 && eaValid(mode, reg, kEaControl | EaPostInc)) {
                    h = &opMovem;
                }
                break;
            case 7: {
                if ((op & 0xFFF0) == 0x4E40) h = &opTrap;
                else if ((op & 0xFFF8) == 0x4E50) h = &opLink;
                else if ((op & 0xFFF8) == 0x4E58) h = &opUnlk;
                else if ((op & 0xFFF0) == 0x4E60) h = &opMoveUsp;
                else if (op == 0x4E70) h = &opReset;
                else if (op == 0x4E71) h = &opNop;
                else if (op == 0x4E72) h = &opStop;
                else if (op == 0x4E73) h = &opRte;
                else if (op == 0x4E74) h = &opRtd;
                else if (op == 0x4E75) h = &opRts;
                else if (op == 0x4E76) h = &opTrapv;
                else if (op == 0x4E77) h = &opRtr;
                else if (op == 0x4E7A || op == 0x4E7B) h = &opMovec;
                else if ((op & 0xFFC0) == 0x4E80) {
                    if (eaValid(mode, reg, kEaControl)) h = &opJsr;
                } else if ((op & 0xFFC0) == 0x4EC0) {
                    if (eaValid(mode, reg, kEaControl)) h = &opJmp;
                }
                break;
            }
            default: break;
            }
            break;
        }

        case 0x5: {
            if (size2 == 3) {
                if (mode == 1) h = &opDbcc;
                else if (mode == 7 && (reg == 2 || reg == 3 || reg == 4)) h = &opTrapcc;
                else if (eaValid(mode, reg, kEaDataAlter)) h = &opScc;
            } else {
                const u32 cat = (size2 == 0) ? kEaDataAlter : kEaAlterable;
                if (eaValid(mode, reg, cat)) h = &opAddqSubq;
            }
            break;
        }

        case 0x6:
            h = &opBcc;
            break;

        case 0x7:
            if (!bit8) h = &opMoveq;
            break;

        case 0x8: { // OR / DIV.W / SBCD / PACK / UNPK
            if (size2 == 3) {
                if (eaValid(mode, reg, kEaDataAddr)) h = &opDiv;
            } else if (!bit8) {
                if (eaValid(mode, reg, kEaDataAddr)) h = &opLogic;
            } else {
                if (size2 == 0 && mode <= 1) h = &opAbcdSbcd;        // SBCD
                else if (size2 == 1 && mode <= 1) h = &opPackUnpk;   // PACK
                else if (size2 == 2 && mode <= 1) h = &opPackUnpk;   // UNPK
                else if (eaValid(mode, reg, kEaMemAlter)) h = &opLogic;
            }
            break;
        }

        case 0x9: case 0xD: { // SUB / ADD families
            if (size2 == 3) { // SUBA/ADDA
                if (eaValid(mode, reg, kEaAll)) h = &opAdda;
            } else if (!bit8) {
                const u32 cat = (size2 == 0) ? (kEaAll & ~EaAn) : kEaAll;
                if (eaValid(mode, reg, cat)) h = &opAddSub;
            } else {
                if (mode <= 1) h = &opAddxSubx;
                else if (eaValid(mode, reg, kEaMemAlter)) h = &opAddSub;
            }
            break;
        }

        case 0xA:
            h = &opALine;
            break;

        case 0xB: { // CMP / CMPA / CMPM / EOR
            if (size2 == 3) {
                if (eaValid(mode, reg, kEaAll)) h = &opCmpa;
            } else if (!bit8) {
                const u32 cat = (size2 == 0) ? (kEaAll & ~EaAn) : kEaAll;
                if (eaValid(mode, reg, cat)) h = &opCmp;
            } else {
                if (mode == 1) h = &opCmpm;
                else if (eaValid(mode, reg, kEaDataAlter)) h = &opLogic;   // EOR
            }
            break;
        }

        case 0xC: { // AND / MUL.W / ABCD / EXG
            if (size2 == 3) {
                if (eaValid(mode, reg, kEaDataAddr)) h = &opMul;
            } else if (!bit8) {
                if (eaValid(mode, reg, kEaDataAddr)) h = &opLogic;
            } else {
                const u16 pat = op & 0x01F8;
                if (size2 == 0 && mode <= 1) h = &opAbcdSbcd;   // ABCD
                else if (pat == 0x0140 || pat == 0x0148 || pat == 0x0188) h = &opExg;
                else if (size2 < 3 && eaValid(mode, reg, kEaMemAlter)) h = &opLogic;
            }
            break;
        }

        case 0xE: { // shifts, rotates and bitfields
            if (size2 == 3) {
                if (((op >> 11) & 1) == 0) {
                    if (eaValid(mode, reg, kEaMemAlter)) h = &opShiftMem;
                } else {
                    // BFTST/BFEXTU/BFCHG/BFEXTS/BFCLR/BFFFO/BFSET/BFINS
                    const int kind = (op >> 8) & 7;
                    const bool readOnly = kind == 0 || kind == 1 || kind == 3 || kind == 5;
                    u32 cat = readOnly ? (EaDn | kEaControl) : (EaDn | kEaControlAlter);
                    if (eaValid(mode, reg, cat)) h = &opBitField;
                }
            } else {
                h = &opShiftReg;
            }
            break;
        }

        case 0xF: {
            // The '040's own F-line blocks; anything else F-line traps.
            if ((op & 0xFFC0) == 0xF200) h = &opFpuGen;
            else if ((op & 0xFFC0) == 0xF240) {
                if (mode == 1) h = &opFScc;                       // FDBcc
                else if (mode == 7 && (reg == 2 || reg == 3 || reg == 4)) h = &opFScc; // FTRAPcc
                else if (eaValid(mode, reg, kEaDataAlter)) h = &opFScc;
            }
            else if ((op & 0xFF80) == 0xF280) h = &opFBcc;        // word + long forms
            else if ((op & 0xFFC0) == 0xF300) {
                if (eaValid(mode, reg, kEaControlAlter | EaPreDec)) h = &opFSave;
            }
            else if ((op & 0xFFC0) == 0xF340) {
                if (eaValid(mode, reg, kEaControl | EaPostInc)) h = &opFRestore;
            }
            else if ((op & 0xFF00) == 0xF400) {
                // CINV/CPUSH: scope 00 raises ILLEGAL (vector 4), per PRM 6-3.
                h = (((op >> 3) & 3) != 0) ? &opCinvCpush : &opIllegal;
            }
            else if ((op & 0xFFE0) == 0xF500) h = &opPflush;
            else if ((op & 0xFFF8) == 0xF548) h = &opPtest;   // PTESTW (An)
            else if ((op & 0xFFF8) == 0xF568) h = &opPtest;   // PTESTR (An)
            else if ((op & 0xFFF8) == 0xF620) h = &opMove16;                  // (Ax)+,(Ay)+
            else if ((op & 0xFFE0) == 0xF600) h = &opMove16;                  // absolute forms
            if (!h) h = &opFLine;
            break;
        }

        default:
            break;
        }

        if (h) t[op] = h;
    }
}

} // namespace openmac
