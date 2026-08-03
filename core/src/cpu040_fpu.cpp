#include "cpu040_ops.hpp"

#include <cmath>
#include <cstring>

// The '040 FPU. Accuracy tier decision (mirrors the project's documented
// approach of naming its approximations): the register file is held in host
// doubles, and the FULL 6888x instruction set -- including the transcendentals
// a real '040 traps to the FPSP -- executes natively. That trades the last
// 11 mantissa bits of extended precision and FPSP trap fidelity for a
// complete, always-working FPU without modeling the FSAVE unimplemented-
// instruction frame internals. The 80-bit extended and packed-decimal MEMORY
// formats convert correctly, so data structures round-trip.
//
// Reference: M68040UM section 9, MC68881/68882 User's Manual for opmodes and
// condition predicates, M68000PRM 1.2 for the memory formats. Clean-room.

namespace openmac {

namespace {

// FPSR condition-code bits.
constexpr u32 kFpN   = 1u << 27;
constexpr u32 kFpZ   = 1u << 26;
constexpr u32 kFpI   = 1u << 25;
constexpr u32 kFpNan = 1u << 24;

void setFpccFrom(M68040& c, double v) {
    u32 cc = 0;
    if (std::isnan(v)) cc |= kFpNan;
    else {
        if (std::isinf(v)) cc |= kFpI;
        if (v == 0.0) cc |= kFpZ;
        if (std::signbit(v)) cc |= kFpN;
    }
    c.fpsr = (c.fpsr & 0x00FFFFFFu) | cc;
}

bool fpTestCond(const M68040& c, int cond) {
    const bool nan = (c.fpsr & kFpNan) != 0;
    const bool z   = (c.fpsr & kFpZ) != 0;
    const bool n   = (c.fpsr & kFpN) != 0;
    switch (cond & 0x0F) {   // 0x10-0x1F are the signaling twins
    case 0x0: return false;                     // F / SF
    case 0x1: return z;                         // EQ
    case 0x2: return !nan && !z && !n;          // OGT
    case 0x3: return z || (!nan && !n);         // OGE
    case 0x4: return n && !nan && !z;           // OLT
    case 0x5: return z || (n && !nan);          // OLE
    case 0x6: return !nan && !z;                // OGL
    case 0x7: return !nan;                      // OR
    case 0x8: return nan;                       // UN
    case 0x9: return nan || z;                  // UEQ
    case 0xA: return nan || (!n && !z);         // UGT
    case 0xB: return nan || z || !n;            // UGE
    case 0xC: return nan || (n && !z);          // ULT
    case 0xD: return nan || n || z;             // ULE
    case 0xE: return !z;                        // NE
    default:  return true;                      // T / ST
    }
}

// ---- memory format conversions ----

double singleToDouble(u32 bits) {
    float f;
    std::memcpy(&f, &bits, 4);
    return static_cast<double>(f);
}

u32 doubleToSingle(double v) {
    const float f = static_cast<float>(v);
    u32 bits;
    std::memcpy(&bits, &f, 4);
    return bits;
}

double bitsToDouble(u64 bits) {
    double d;
    std::memcpy(&d, &bits, 8);
    return d;
}

u64 doubleToBits(double v) {
    u64 bits;
    std::memcpy(&bits, &v, 8);
    return bits;
}

// 96-bit extended: [sign|15-bit exponent|16 zeros] [64-bit mantissa, explicit
// integer bit]. Bias 16383.
double extendedToDouble(u32 se, u64 mant) {
    const bool neg = (se & 0x80000000u) != 0;
    const int exp = static_cast<int>((se >> 16) & 0x7FFF);
    if (exp == 0x7FFF) {
        if (mant << 1) return neg ? -std::numeric_limits<double>::quiet_NaN()
                                  : std::numeric_limits<double>::quiet_NaN();
        return neg ? -std::numeric_limits<double>::infinity()
                   : std::numeric_limits<double>::infinity();
    }
    if (mant == 0) return neg ? -0.0 : 0.0;
    // value = mant * 2^(exp - 16383 - 63); denormals (exp 0) share the formula.
    double v = std::ldexp(static_cast<double>(mant), exp - 16383 - 63);
    return neg ? -v : v;
}

void doubleToExtended(double v, u32& se, u64& mant) {
    u32 sign = std::signbit(v) ? 0x80000000u : 0;
    if (std::isnan(v)) { se = sign | 0x7FFF0000u; mant = 0xC000000000000000ull; return; }
    if (std::isinf(v)) { se = sign | 0x7FFF0000u; mant = 0; return; }
    if (v == 0.0)      { se = sign; mant = 0; return; }
    int exp;
    const double frac = std::frexp(std::fabs(v), &exp);   // frac in [0.5, 1)
    // mantissa with explicit integer bit: frac * 2^64, exponent biased.
    mant = static_cast<u64>(std::ldexp(frac, 64));
    se = sign | (static_cast<u32>(exp - 1 + 16383) << 16);
}

// Packed-decimal real: ±d16 . d15..d0 x 10^±eee (3 BCD exponent digits,
// 17 BCD mantissa digits). SANE feeds Str2Dec conversions through this.
double packedToDouble(u32 w0, u32 w1, u32 w2) {
    const bool sm = (w0 & 0x80000000u) != 0;
    const bool se = (w0 & 0x40000000u) != 0;
    int exp = 0;
    for (int i = 2; i >= 0; --i) exp = exp * 10 + static_cast<int>((w0 >> (16 + 4 * i)) & 0xF);
    double mant = static_cast<double>(w0 & 0xF);   // d16, the integer digit
    u64 fracBits = (static_cast<u64>(w1) << 32) | w2;
    double scale = 0.1;
    for (int i = 15; i >= 0; --i) {
        mant += static_cast<double>((fracBits >> (4 * i)) & 0xF) * scale;
        scale *= 0.1;
    }
    double v = mant * std::pow(10.0, se ? -exp : exp);
    return sm ? -v : v;
}

void doubleToPacked(double v, int k, u32& w0, u32& w1, u32& w2) {
    w0 = w1 = w2 = 0;
    if (std::signbit(v)) w0 |= 0x80000000u;
    v = std::fabs(v);
    if (std::isnan(v) || std::isinf(v)) { w0 |= 0x7FFF0000u; return; }
    if (v == 0.0) return;
    // k-factor: k > 0 = significant digits, k <= 0 = digits right of the
    // point. Clamp to the format's 17 digits.
    int digits = k > 0 ? k : 17;
    if (digits < 1) digits = 1;
    if (digits > 17) digits = 17;
    int exp10 = static_cast<int>(std::floor(std::log10(v)));
    double scaled = v / std::pow(10.0, exp10);
    if (scaled >= 10.0) { scaled /= 10.0; ++exp10; }
    if (scaled < 1.0)   { scaled *= 10.0; --exp10; }
    if (k <= 0) {
        digits = exp10 + 1 - k;
        if (digits < 1) digits = 1;
        if (digits > 17) digits = 17;
    }
    // Round to `digits` significant digits and emit BCD.
    double rounded = scaled * std::pow(10.0, digits - 1);
    u64 digval = static_cast<u64>(rounded + 0.5);
    if (digval >= static_cast<u64>(std::pow(10.0, digits))) { digval /= 10; ++exp10; }
    // Redistribute into d16 (integer digit) + 16 fraction digits.
    u8 bcd[17] = {};
    for (int i = digits - 1; i >= 0; --i) { bcd[i] = static_cast<u8>(digval % 10); digval /= 10; }
    // bcd[0] is the leading digit = d16.
    w0 |= bcd[0] & 0xF;
    u64 frac = 0;
    for (int i = 0; i < 16; ++i) frac = (frac << 4) | (i + 1 < 17 ? bcd[i + 1] : 0);
    w1 = static_cast<u32>(frac >> 32);
    w2 = static_cast<u32>(frac);
    int e = exp10;
    if (e < 0) { w0 |= 0x40000000u; e = -e; }
    if (e > 999) e = 999;
    w0 |= static_cast<u32>(e % 10) << 16;
    w0 |= static_cast<u32>((e / 10) % 10) << 20;
    w0 |= static_cast<u32>(e / 100) << 24;
}

// FMOVECR constants (the 6888x on-chip ROM).
double fpuConstant(int offset) {
    switch (offset) {
    case 0x00: return 3.14159265358979323846;        // pi
    case 0x0B: return 0.30102999566398119521;        // log10(2)
    case 0x0C: return 2.71828182845904523536;        // e
    case 0x0D: return 1.44269504088896340736;        // log2(e)
    case 0x0E: return 0.43429448190325182765;        // log10(e)
    case 0x0F: return 0.0;
    case 0x30: return 0.69314718055994530942;        // ln(2)
    case 0x31: return 2.30258509299404568402;        // ln(10)
    case 0x32: return 1.0;
    case 0x33: return 10.0;
    case 0x34: return 1e2;
    case 0x35: return 1e4;
    case 0x36: return 1e8;
    case 0x37: return 1e16;
    case 0x38: return 1e32;
    case 0x39: return 1e64;
    case 0x3A: return 1e128;
    case 0x3B: return 1e256;
    default:   return 0.0;
    }
}

// Operand byte sizes per source/destination format code.
u32 fpFormatBytes(int fmt) {
    switch (fmt) {
    case 0: return 4;    // L
    case 1: return 4;    // S
    case 2: return 12;   // X
    case 3: return 12;   // P
    case 4: return 2;    // W
    case 5: return 8;    // D
    default: return 1;   // B
    }
}

// The effective address for an FPU operand of this format.
//
// Postincrement and predecrement step the address register by the size of the
// OPERAND, and an FPU operand is up to 12 bytes: extended and packed are 12,
// double is 8. The integer EA calculator only knows byte/word/long, so asking
// it for the address moves the register by at most 4 -- and the transfer then
// reads or writes the remaining 8 bytes past the pointer, over whatever is
// there. On the stack that is the caller's saved registers and its return
// address: `FMOVE.X FP0,-(A7)` inside the ROM's SANE glue wrote the result
// across the A0 it had saved four bytes earlier, so the routine returned a
// garbage pointer and its caller followed it. (Same defect the FMOVEM control
// list had; this is its single-register sibling.)
//
// The stack-pointer byte rule holds here too: -(A7)/(A7)+ with a byte operand
// moves A7 by 2, keeping it even.
u32 fpuOperandEA(M68040& c, int mode, int reg, int fmt) {
    const u32 bytes = fpFormatBytes(fmt);
    const u32 step = (bytes == 1 && reg == 7) ? 2u : bytes;
    if (mode == 3) {              // (An)+
        const u32 addr = c.a[reg];
        c.a[reg] += step;
        return addr;
    }
    if (mode == 4) {              // -(An)
        c.a[reg] -= step;
        return c.a[reg];
    }
    // Every other mode ignores the size; pass a code it understands.
    return CpuOps040::calcEA(c, mode, reg, bytes == 1 ? 0 : bytes == 2 ? 1 : 2);
}

} // namespace

// General FPU instruction: F200-F23F with the coprocessor extension word.
int CpuOps040::opFpuGen(M68040& c, u16 op) {
    const u16 ext = c.fetch16();
    const int cls = (ext >> 13) & 7;
    const int mode = (op >> 3) & 7, reg = op & 7;
    c.fpuUsed_ = true;

    switch (cls) {
    case 0:   // FPm -> FPn arithmetic
    case 2: { // <ea> -> FPn arithmetic (or FMOVECR)
        const int srcSpec = (ext >> 10) & 7;
        const int dst = (ext >> 7) & 7;
        const int opmode = ext & 0x7F;

        double src;
        if (cls == 0) {
            src = c.fp[srcSpec];
        } else if (srcSpec == 7) {   // FMOVECR #ccc,FPn
            c.fpiar = instrStart(c);
            c.fp[dst] = fpuConstant(opmode);
            setFpccFrom(c, c.fp[dst]);
            return 4;
        } else {
            // Fetch by format from the EA.
            const int fmt = srcSpec;
            if (mode == 0) {
                // Register direct: B/W/L/S only
                const u32 raw = c.d[reg];
                if (fmt == 1) src = singleToDouble(raw);
                else if (fmt == 0) src = static_cast<double>(static_cast<s32>(raw));
                else if (fmt == 4) src = static_cast<double>(static_cast<s16>(raw & 0xFFFF));
                else src = static_cast<double>(static_cast<s8>(raw & 0xFF));
            } else if (mode == 7 && reg == 4) {
                // Immediate
                if (fmt == 2 || fmt == 3) {
                    const u32 w0 = c.fetch32(), w1 = c.fetch32(), w2 = c.fetch32();
                    src = (fmt == 2)
                        ? extendedToDouble(w0, (static_cast<u64>(w1) << 32) | w2)
                        : packedToDouble(w0, w1, w2);
                } else if (fmt == 5) {
                    const u64 hi = c.fetch32();
                    src = bitsToDouble((hi << 32) | c.fetch32());
                } else {
                    const u32 raw = (fmt == 4 || fmt == 6) ? c.fetch16() : c.fetch32();
                    if (fmt == 1) src = singleToDouble(raw);
                    else if (fmt == 0) src = static_cast<double>(static_cast<s32>(raw));
                    else if (fmt == 4) src = static_cast<double>(static_cast<s16>(raw & 0xFFFF));
                    else src = static_cast<double>(static_cast<s8>(raw & 0xFF));
                }
            } else {
                const u32 addr = fpuOperandEA(c, mode, reg, fmt);
                if (fmt == 2 || fmt == 3) {
                    const u32 w0 = c.rd32(addr);
                    const u32 w1 = c.rd32(addr + 4), w2 = c.rd32(addr + 8);
                    src = (fmt == 2)
                        ? extendedToDouble(w0, (static_cast<u64>(w1) << 32) | w2)
                        : packedToDouble(w0, w1, w2);
                } else if (fmt == 5) {
                    src = bitsToDouble((static_cast<u64>(c.rd32(addr)) << 32) | c.rd32(addr + 4));
                } else if (fmt == 4) {
                    src = static_cast<double>(static_cast<s16>(c.rd16(addr)));
                } else if (fmt == 6) {
                    src = static_cast<double>(static_cast<s8>(c.rd8(addr)));
                } else if (fmt == 1) {
                    src = singleToDouble(c.rd32(addr));
                } else {
                    src = static_cast<double>(static_cast<s32>(c.rd32(addr)));
                }
            }
        }

        c.fpiar = instrStart(c);
        double& fpd = c.fp[dst];
        double r = fpd;
        const double x = src;
        bool store = true;
        switch (opmode) {
        case 0x00: case 0x40: case 0x44: r = x; break;                 // FMOVE/FSMOVE/FDMOVE
        case 0x01: r = std::nearbyint(x); break;                       // FINT
        case 0x02: r = std::sinh(x); break;
        case 0x03: r = std::trunc(x); break;                           // FINTRZ
        case 0x04: case 0x41: case 0x45:                               // FSQRT
            if (x < 0.0) { r = std::numeric_limits<double>::quiet_NaN(); c.fpsr |= 0x2000u; }
            else r = std::sqrt(x);
            break;
        case 0x06: r = std::log1p(x); break;                           // FLOGNP1
        case 0x08: r = std::expm1(x); break;                           // FETOXM1
        case 0x09: r = std::tanh(x); break;
        case 0x0A: r = std::atan(x); break;
        case 0x0C: r = std::asin(x); break;
        case 0x0D: r = std::atanh(x); break;
        case 0x0E: r = std::sin(x); break;
        case 0x0F: r = std::tan(x); break;
        case 0x10: r = std::exp(x); break;                             // FETOX
        case 0x11: r = std::exp2(x); break;                            // FTWOTOX
        case 0x12: r = std::pow(10.0, x); break;                       // FTENTOX
        case 0x14: r = std::log(x); break;                             // FLOGN
        case 0x15: r = std::log10(x); break;
        case 0x16: r = std::log2(x); break;
        case 0x18: case 0x58: case 0x5C: r = std::fabs(x); break;      // FABS
        case 0x19: r = std::cosh(x); break;
        case 0x1A: case 0x5A: case 0x5E: r = -x; break;                // FNEG
        case 0x1C: r = std::acos(x); break;
        case 0x1D: r = std::cos(x); break;
        case 0x1E: {                                                   // FGETEXP
            if (x == 0.0 || std::isinf(x) || std::isnan(x)) r = x;
            else { int e; std::frexp(x, &e); r = static_cast<double>(e - 1); }
            break;
        }
        case 0x1F: {                                                   // FGETMAN
            if (x == 0.0 || std::isinf(x) || std::isnan(x)) r = x;
            else { int e; r = std::frexp(x, &e) * 2.0; if (x < 0 && r > 0) r = -r; }
            break;
        }
        case 0x20: case 0x24: case 0x60: case 0x64:                    // FDIV/FSGLDIV
            if (x == 0.0) c.fpsr |= 0x400u;                            // DZ status
            r = fpd / x;
            break;
        case 0x21: {                                                   // FMOD
            r = std::fmod(fpd, x);
            const double q = std::trunc(fpd / x);
            c.fpsr = (c.fpsr & ~0x00FF0000u) |
                     ((static_cast<u32>(std::fabs(q)) & 0x7Fu) << 16) |
                     ((std::signbit(q) ? 1u : 0u) << 23);
            break;
        }
        case 0x22: case 0x62: case 0x66: r = fpd + x; break;           // FADD
        case 0x23: case 0x27: case 0x63: case 0x67: r = fpd * x; break;// FMUL/FSGLMUL
        case 0x25: {                                                   // FREM
            r = std::remainder(fpd, x);
            const double q = std::nearbyint(fpd / x);
            c.fpsr = (c.fpsr & ~0x00FF0000u) |
                     ((static_cast<u32>(std::fabs(q)) & 0x7Fu) << 16) |
                     ((std::signbit(q) ? 1u : 0u) << 23);
            break;
        }
        case 0x26: r = fpd * std::pow(2.0, std::trunc(x)); break;      // FSCALE
        case 0x28: case 0x68: case 0x6C: r = fpd - x; break;           // FSUB
        case 0x30: case 0x31: case 0x32: case 0x33:                    // FSINCOS
        case 0x34: case 0x35: case 0x36: case 0x37:
            c.fp[opmode & 7] = std::cos(x);
            r = std::sin(x);
            break;
        case 0x38: {                                                   // FCMP
            const double diff = fpd - x;
            if (std::isnan(fpd) || std::isnan(x)) {
                c.fpsr = (c.fpsr & 0x00FFFFFFu) | kFpNan;
            } else if (fpd == x) {
                // Equal: Z, with N tracking the sign of the operands.
                u32 cc = kFpZ;
                if (std::signbit(fpd)) cc |= kFpN;
                c.fpsr = (c.fpsr & 0x00FFFFFFu) | cc;
            } else {
                setFpccFrom(c, diff);
            }
            store = false;
            break;
        }
        case 0x3A:                                                     // FTST
            setFpccFrom(c, x);
            store = false;
            break;
        default:
            // Genuinely undefined opmode: F-line trap.
            c.pc = instrStart(c);
            return raiseException(c, kVec040FLine, 15);
        }

        // Single/double-rounded variants round the result to that precision.
        if ((opmode & 0x40) != 0) {
            if ((opmode & 0x04) == 0 && opmode != 0x44 && opmode != 0x45 &&
                opmode != 0x4C && opmode != 0x64 && opmode != 0x66 && opmode != 0x67 &&
                opmode != 0x6C) {
                r = static_cast<double>(static_cast<float>(r));   // FSxxx
            }
        }
        if (store) {
            fpd = r;
            setFpccFrom(c, fpd);
        }
        return 6;
    }

    case 3: { // FPn -> <ea>
        const int fmt = (ext >> 10) & 7;
        const int src = (ext >> 7) & 7;
        const int kf = ext & 0x7F;
        const double v = c.fp[src];
        c.fpiar = instrStart(c);

        if (mode == 0) {
            if (fmt == 1) c.d[reg] = doubleToSingle(v);
            else if (fmt == 0) c.d[reg] = static_cast<u32>(static_cast<s32>(v));
            else if (fmt == 4) writeSized(c.d[reg], static_cast<u32>(static_cast<s32>(v)) & 0xFFFF, 1);
            else writeSized(c.d[reg], static_cast<u32>(static_cast<s32>(v)) & 0xFF, 0);
            return 4;
        }
        const u32 addr = fpuOperandEA(c, mode, reg, fmt);
        switch (fmt) {
        case 0: c.wr32(addr, static_cast<u32>(static_cast<s32>(v))); break;
        case 1: c.wr32(addr, doubleToSingle(v)); break;
        case 2: {
            u32 se; u64 mant;
            doubleToExtended(v, se, mant);
            c.wr32(addr, se);
            c.wr32(addr + 4, static_cast<u32>(mant >> 32));
            c.wr32(addr + 8, static_cast<u32>(mant));
            break;
        }
        case 3: {
            u32 w0, w1, w2;
            const int k = (kf & 0x40) ? (kf | ~0x7F) : kf;   // sign-extend
            doubleToPacked(v, k, w0, w1, w2);
            c.wr32(addr, w0);
            c.wr32(addr + 4, w1);
            c.wr32(addr + 8, w2);
            break;
        }
        case 4: c.wr16(addr, static_cast<u16>(static_cast<s32>(v))); break;
        case 5: {
            const u64 bits = doubleToBits(v);
            c.wr32(addr, static_cast<u32>(bits >> 32));
            c.wr32(addr + 4, static_cast<u32>(bits));
            break;
        }
        default: c.wr8(addr, static_cast<u8>(static_cast<s32>(v))); break;
        }
        return 4;
    }

    case 4:   // <ea> -> control register(s)
    case 5: { // control register(s) -> <ea>
        const bool toCtrl = cls == 4;
        u32 list = (ext >> 10) & 7;   // bit2 FPCR, bit1 FPSR, bit0 FPIAR
        int n = 0;
        for (u32 b = 0; b < 3; ++b) n += (list >> b) & 1;
        if (n == 0) return 4;

        if (mode == 0 || mode == 1) {   // single register only
            u32* regp = (list == 4) ? &c.fpcr : (list == 2) ? &c.fpsr : &c.fpiar;
            if (toCtrl) *regp = (mode == 0) ? c.d[reg] : c.a[reg];
            else if (mode == 0) c.d[reg] = *regp;
            else c.a[reg] = *regp;
            return 4;
        }
        // The whole list moves as one block, so predecrement must step back over
        // ALL of it before writing and postincrement must step over all of it
        // after: adjusting by one long regardless of the count leaves the block
        // straddling the address register, overwriting the caller's longword
        // just above it. (The ROM's SANE environment call saves a return address
        // exactly there and then jumps through it.)
        u32 addr;
        if (mode == 3) {
            addr = c.a[reg];
            c.a[reg] += 4u * static_cast<u32>(n);
        } else if (mode == 4) {
            c.a[reg] -= 4u * static_cast<u32>(n);
            addr = c.a[reg];
        } else {
            addr = calcEA(c, mode, reg, 2);
        }
        // Transfers highest to lowest: FPCR, FPSR, FPIAR at ascending addresses.
        for (int b = 2; b >= 0; --b) {
            if (!((list >> b) & 1)) continue;
            u32* regp = (b == 2) ? &c.fpcr : (b == 1) ? &c.fpsr : &c.fpiar;
            if (toCtrl) *regp = c.rd32(addr);
            else        c.wr32(addr, *regp);
            addr += 4;
        }
        return 4 + n;
    }

    case 6:   // FMOVEM.X <ea> -> FPn list
    case 7: { // FMOVEM.X FPn list -> <ea>
        const bool toRegs = cls == 6;
        const int lmode = (ext >> 11) & 3;   // 0/1 predec order, 2/3 postinc order; 1/3 dynamic
        u32 list = ext & 0xFF;
        if (lmode & 1) list = c.d[(ext >> 4) & 7] & 0xFF;

        int n = 0;
        for (u32 b = 0; b < 8; ++b) n += (list >> b) & 1;

        if (!toRegs && mode == 4) {
            // Predecrement carries the REVERSED mask -- list bit 0 is FP0 and
            // bit 7 is FP7, against bit 7 = FP0 for control and postincrement.
            // The reversal is what makes a save and its restore name the same
            // registers: this ROM pushes with $01/$0C and pops with $80/$30,
            // and those are the same two masks read from opposite ends.
            // Reading the predecrement mask the postincrement way pushed FP7
            // where FP0 was asked for -- and the ROM's SANE normalize loop,
            // handed a zero mantissa with a live exponent, shifts it left
            // forever (a hang with the cursor still tracking, because the
            // cursor runs off an interrupt and everything else waits on the
            // task that never returns).
            //
            // Registers still store DESCENDING, so that ascending memory holds
            // FP0..FP7 exactly as the postincrement load will read it back.
            u32 addr = c.a[reg];
            for (int i = 7; i >= 0; --i) {
                if (!((list >> i) & 1)) continue;
                addr -= 12;
                u32 se; u64 mant;
                doubleToExtended(c.fp[i], se, mant);
                c.wr32(addr, se);
                c.wr32(addr + 4, static_cast<u32>(mant >> 32));
                c.wr32(addr + 8, static_cast<u32>(mant));
            }
            c.a[reg] = addr;
            return 3 + 2 * n;
        }

        // Control/postinc order: list bit 7 = FP0 .. bit 0 = FP7, ascending.
        u32 addr = (mode == 3) ? c.a[reg] : calcEA(c, mode, reg, 2);
        for (int i = 7; i >= 0; --i) {
            if (!((list >> i) & 1)) continue;
            const int fpn = 7 - i;
            if (toRegs) {
                const u32 se = c.rd32(addr);
                const u64 mant = (static_cast<u64>(c.rd32(addr + 4)) << 32) | c.rd32(addr + 8);
                c.fp[fpn] = extendedToDouble(se, mant);
            } else {
                u32 se; u64 mant;
                doubleToExtended(c.fp[fpn], se, mant);
                c.wr32(addr, se);
                c.wr32(addr + 4, static_cast<u32>(mant >> 32));
                c.wr32(addr + 8, static_cast<u32>(mant));
            }
            addr += 12;
        }
        if (mode == 3) c.a[reg] = addr;
        return 3 + 2 * n;
    }

    default:
        c.pc = instrStart(c);
        return raiseException(c, kVec040FLine, 15);
    }
}

int CpuOps040::opFBcc(M68040& c, u16 op) {
    const int cond = op & 0x3F;
    const bool isLong = (op & 0x0040) != 0;
    const u32 base = c.pc;
    const s32 disp = isLong ? static_cast<s32>(c.fetch32())
                            : static_cast<s32>(static_cast<s16>(c.fetch16()));
    c.fpuUsed_ = true;
    if (fpTestCond(c, cond)) {
        jumpTo(c, base + static_cast<u32>(disp));
        return 4;
    }
    return 3;
}

int CpuOps040::opFScc(M68040& c, u16 op) {
    const int mode = (op >> 3) & 7, reg = op & 7;
    const u16 ext = c.fetch16();
    const int cond = ext & 0x3F;
    c.fpuUsed_ = true;

    if (mode == 1) {   // FDBcc Dn,disp
        const u32 base = c.pc;
        const s32 disp = static_cast<s16>(c.fetch16());
        if (fpTestCond(c, cond)) return 4;
        const u16 count = static_cast<u16>((c.d[reg] & 0xFFFF) - 1);
        writeSized(c.d[reg], count, 1);
        if (count != 0xFFFF) {
            jumpTo(c, base + static_cast<u32>(disp));
            return 4;
        }
        return 4;
    }
    if (mode == 7 && (reg == 2 || reg == 3 || reg == 4)) {   // FTRAPcc
        if (reg == 2)      (void)c.fetch16();
        else if (reg == 3) (void)c.fetch32();
        if (fpTestCond(c, cond)) {
            return raiseFrame2(c, kVec040Trapcc, instrStart(c), 15);
        }
        return 3;
    }
    // FScc
    const u32 v = fpTestCond(c, cond) ? 0xFFu : 0x00u;
    if (mode == 0) {
        writeSized(c.d[reg], v, 0);
        return 3;
    }
    const u32 addr = calcEA(c, mode, reg, 0);
    c.wr8(addr, static_cast<u8>(v));
    return 3 + eaTime(eaIndex(mode, reg));
}

// FSAVE/FRESTORE: with the whole instruction set executing natively there is
// never a mid-instruction exception state to externalize, so FSAVE produces
// the NULL frame until the FPU has been touched and the 4-byte IDLE frame
// after; FRESTORE accepts NULL (resets the FPU) and skips any sized frame.
int CpuOps040::opFSave(M68040& c, u16 op) {
    if (!flag(c, kS040)) return privilegeViolation(c);
    const int mode = (op >> 3) & 7, reg = op & 7;
    // NULL frame: a single zero long. IDLE: version $41, format $00.
    const u32 frame = c.fpuUsed_ ? 0x41000000u : 0u;
    if (mode == 4) {
        c.a[reg] -= 4;
        c.wr32(c.a[reg], frame);
    } else {
        c.wr32(calcEA(c, mode, reg, 2), frame);
    }
    return 4;
}

int CpuOps040::opFRestore(M68040& c, u16 op) {
    if (!flag(c, kS040)) return privilegeViolation(c);
    const int mode = (op >> 3) & 7, reg = op & 7;
    u32 addr = (mode == 3) ? c.a[reg] : calcEA(c, mode, reg, 2);
    const u32 frame = c.rd32(addr);
    u32 size = 4;
    if ((frame >> 24) == 0) {
        // NULL: reset the FPU state.
        for (double& f : c.fp) f = 0.0;
        c.fpcr = c.fpsr = c.fpiar = 0;
        c.fpuUsed_ = false;
    } else {
        // Sized frame: skip it (frame size lives in bits 23-16).
        size = 4 + ((frame >> 16) & 0xFF);
        c.fpuUsed_ = true;
    }
    if (mode == 3) c.a[reg] += size;
    return 4;
}

} // namespace openmac
