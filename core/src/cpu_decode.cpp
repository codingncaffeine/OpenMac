#include "cpu_ops.hpp"

// Builds the 64K opcode dispatch table. Every encoding not claimed here stays
// on opIllegal (vector 4); $Axxx/$Fxxx trap through their own vectors.

namespace openmac {

void CpuOps::buildTableInto(std::array<Handler, 65536>& t) {
    for (u32 opv = 0; opv < 65536; ++opv) {
        const u16 op    = static_cast<u16>(opv);
        const int mode  = (op >> 3) & 7;
        const int reg   = op & 7;
        const int size2 = (op >> 6) & 3;   // 0=b 1=w 2=l 3=other
        const int bit8  = (op >> 8) & 1;
        Handler h = nullptr;

        switch (op >> 12) {
        case 0x0: {
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
            case 4: { // static bit ops (bit number in extension word)
                const int kind = size2;
                const u32 cat = (kind == 0) ? (kEaDataAddr & ~EaImm) : kEaDataAlter;
                if (eaValid(mode, reg, cat)) h = &opBitOp;
                break;
            }
            case 5: // EORI
                if (op == 0x0A3C || op == 0x0A7C) h = &opLogicImm;
                else if (size2 < 3 && eaValid(mode, reg, kEaDataAlter)) h = &opLogicImm;
                break;
            case 6: // CMPI (68000: data-alterable only)
                if (size2 < 3 && eaValid(mode, reg, kEaDataAlter)) h = &opCmpi;
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
                if (om == 6) { // CHK.w
                    if (eaValid(mode, reg, kEaDataAddr)) h = &opChk;
                } else if (om == 7) { // LEA
                    if (eaValid(mode, reg, kEaControl)) h = &opLea;
                }
                break;
            }
            switch ((op >> 9) & 7) {
            case 0: // NEGX / MOVE from SR
                if (size2 < 3) { if (eaValid(mode, reg, kEaDataAlter)) h = &opNeg; }
                else if (eaValid(mode, reg, kEaDataAlter)) h = &opMoveFromSR;
                break;
            case 1: // CLR (MOVE from CCR is 68010+)
                if (size2 < 3 && eaValid(mode, reg, kEaDataAlter)) h = &opClr;
                break;
            case 2: // NEG / MOVE to CCR
                if (size2 < 3) { if (eaValid(mode, reg, kEaDataAlter)) h = &opNeg; }
                else if (eaValid(mode, reg, kEaDataAddr)) h = &opMoveToCCR;
                break;
            case 3: // NOT / MOVE to SR
                if (size2 < 3) { if (eaValid(mode, reg, kEaDataAlter)) h = &opNot; }
                else if (eaValid(mode, reg, kEaDataAddr)) h = &opMoveToSR;
                break;
            case 4: // NBCD / SWAP / PEA / EXT / MOVEM reg->mem
                if (size2 == 0) {
                    if (eaValid(mode, reg, kEaDataAlter)) h = &opNbcd;
                } else if (size2 == 1) {
                    if (mode == 0) h = &opSwap;
                    else if (eaValid(mode, reg, kEaControl)) h = &opPea;
                } else {
                    if (mode == 0) h = &opExt;
                    else if (eaValid(mode, reg, kEaControlAlter | EaPreDec)) h = &opMovem;
                }
                break;
            case 5: // TST / TAS / ILLEGAL
                if (size2 < 3) {
                    if (eaValid(mode, reg, kEaDataAlter)) h = &opTst;
                } else {
                    if (op == 0x4AFC) h = &opIllegal;
                    else if (eaValid(mode, reg, kEaDataAlter)) h = &opTas;
                }
                break;
            case 6: // MOVEM mem->reg
                if (size2 >= 2 && eaValid(mode, reg, kEaControl | EaPostInc)) h = &opMovem;
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
                else if (op == 0x4E75) h = &opRts;
                else if (op == 0x4E76) h = &opTrapv;
                else if (op == 0x4E77) h = &opRtr;
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

        case 0x8: { // OR / DIV / SBCD
            if (size2 == 3) {
                if (eaValid(mode, reg, kEaDataAddr)) h = &opDiv;
            } else if (!bit8) {
                if (eaValid(mode, reg, kEaDataAddr)) h = &opLogic;
            } else {
                if (size2 == 0 && mode <= 1) h = &opAbcdSbcd;   // SBCD
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

        case 0xC: { // AND / MUL / ABCD / EXG
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

        case 0xE: { // shifts and rotates
            if (size2 == 3) {
                if (((op >> 11) & 1) == 0 && eaValid(mode, reg, kEaMemAlter)) h = &opShiftMem;
            } else {
                h = &opShiftReg;
            }
            break;
        }

        case 0xF:
            h = &opFLine;
            break;

        default:
            break;
        }

        if (h) t[op] = h;
    }
}

} // namespace openmac
