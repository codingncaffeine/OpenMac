#include "cpu_ops.hpp"

namespace openmac {

// ---------------------------------------------------------------- construction

M68000::M68000(IBus& bus) : bus_(bus) {
    (void)CpuOps::table(); // force table construction up front
}

void M68000::reset() {
    sr_      = 0x2700;
    stopped  = false;
    halted   = false;
    ssp      = (u32(bus_.read16(0)) << 16) | bus_.read16(2);
    pc       = (u32(bus_.read16(4)) << 16) | bus_.read16(6);
    a[7]     = ssp;
}

void M68000::setIrqLevel(int level) { irqLevel_ = level; }

u32 M68000::uspValue() const { return (sr_ & kS) ? usp : a[7]; }
u32 M68000::sspValue() const { return (sr_ & kS) ? a[7] : ssp; }

void M68000::setSR(u16 value) {
    value &= kSrMask;
    const bool wasS = (sr_ & kS) != 0;
    const bool nowS = (value & kS) != 0;
    if (wasS != nowS) {
        if (wasS) { ssp = a[7]; a[7] = usp; }
        else      { usp = a[7]; a[7] = ssp; }
    }
    sr_ = value;
}

void M68000::setCCR(u8 value) {
    sr_ = static_cast<u16>((sr_ & 0xFF00) | (value & 0x1F));
}

// ---------------------------------------------------------------- bus access

u8 M68000::rd8(u32 addr) {
    const u8 v = bus_.read8(addr & 0xFFFFFF);
    eaUndoReg_ = -1;
    eaFaultCycles_ = 4;
    return v;
}

u16 M68000::rd16(u32 addr) {
    if (addr & 1) throw AddressError{addr, true, false};
    const u16 v = bus_.read16(addr & 0xFFFFFF);
    eaUndoReg_ = -1;
    eaFaultCycles_ = 4;   // a completed access counts toward a later fault
    return v;
}

u32 M68000::rd32(u32 addr) {
    if (addr & 1) throw AddressError{addr, true, false};
    const u32 hi = bus_.read16(addr & 0xFFFFFF);
    const u32 lo = bus_.read16((addr + 2) & 0xFFFFFF);
    eaUndoReg_ = -1;
    eaFaultCycles_ = 8;
    return (hi << 16) | lo;
}

void M68000::wr8(u32 addr, u8 v) {
    bus_.write8(addr & 0xFFFFFF, v);
    eaUndoReg_ = -1;
    eaFaultCycles_ = 4;
}

void M68000::wr16(u32 addr, u16 v) {
    if (addr & 1) throw AddressError{addr, false, false};
    bus_.write16(addr & 0xFFFFFF, v);
    eaUndoReg_ = -1;
    eaFaultCycles_ = 4;
}

void M68000::wr32(u32 addr, u32 v) {
    if (addr & 1) throw AddressError{addr, false, false};
    bus_.write16(addr & 0xFFFFFF, static_cast<u16>(v >> 16));
    bus_.write16((addr + 2) & 0xFFFFFF, static_cast<u16>(v & 0xFFFF));
    eaUndoReg_ = -1;
    eaFaultCycles_ = 8;
}

u16 M68000::fetch16() {
    if (pc & 1) throw AddressError{pc, true, true};
    const u16 v = bus_.read16(pc & 0xFFFFFF);
    pc += 2;
    return v;
}

u32 M68000::fetch32() {
    const u32 hi = fetch16();
    return (hi << 16) | fetch16();
}

void M68000::push16(u16 v) { a[7] -= 2; wr16(a[7], v); }
void M68000::push32(u32 v) { a[7] -= 4; wr32(a[7], v); }
u16  M68000::pop16() { const u16 v = rd16(a[7]); a[7] += 2; return v; }
u32  M68000::pop32() { const u32 v = rd32(a[7]); a[7] += 4; return v; }

// ---------------------------------------------------------------- exceptions

int M68000::exception(int vector, int cycles) {
    if (onException) onException(vector, instrStart_);
    const u16 oldSR = sr_;
    setSR(static_cast<u16>((sr_ | kS) & ~kT));
    push32(pc);
    push16(oldSR);
    pc = rd32(static_cast<u32>(vector) * 4);
    if (pc & 1) { halted = true; }
    return cycles;
}

int M68000::doInterrupt(int level) {
    stopped = false;
    const u16 oldSR = sr_;
    setSR(static_cast<u16>((sr_ | kS) & ~kT));
    sr_ = static_cast<u16>((sr_ & ~0x0700) | (level << 8));
    push32(pc);
    push16(oldSR);
    pc = rd32(static_cast<u32>(kVecAutovector + level) * 4);
    if (pc & 1) { halted = true; }
    return 44;
}

int M68000::step() {
    if (halted) return 4;

    const int mask = (sr_ >> 8) & 7;
    if (irqLevel_ > 0 && (irqLevel_ == 7 || irqLevel_ > mask)) {
        if (onInterrupt) onInterrupt(irqLevel_, 24u + static_cast<u32>(irqLevel_), pc);
        return doInterrupt(irqLevel_);
    }
    if (stopped) return 4;

    if (onStep) onStep(pc);

    const bool traced = (sr_ & kT) != 0;
    instrStart_ = pc;
    pcRing_[pcRingPos_] = pc;
    pcRingPos_ = (pcRingPos_ + 1) & 127;
    eaUndoReg_ = -1;
    eaFaultCycles_ = 0;
    try {
        const u16 op = fetch16();
        ir_ = op;
        int cycles = CpuOps::table()[op](*this, op);
        eaUndoReg_ = -1;
        if (traced && !stopped && !halted) {
            cycles += exception(kVecTrace, 34);
        }
        return cycles;
    } catch (const AddressError& ae) {
        return CpuOps::enterAddressError(*this, ae);
    }
}

// Group-0 exception (address error). 14-byte frame, low to high address:
//   +0  status word (R/W, I/N, FC)
//   +2  access address (long)
//   +6  instruction register
//   +8  status register at the fault
//   +10 program counter (long)
int CpuOps::enterAddressError(M68000& c, const AddressError& ae) {
    if (c.onException) c.onException(kVecAddressError, c.instrStart_);
    try {
        if (c.eaUndoReg_ >= 0) {
            if (!ae.read) c.a[c.eaUndoReg_] = c.eaUndoVal_;   // (An)+ write fault
            c.eaUndoReg_ = -1;
            c.eaUndoKind_ = 0;
        }
        const u16 oldSR = c.sr_;
        c.setSR(static_cast<u16>((oldSR | kS) & ~kT));

        // Status word: upper 11 bits from the instruction register, then
        // R/W (bit 4), instruction-space flag (bit 3), function code.
        const bool wasS = (oldSR & kS) != 0;
        u16 status = static_cast<u16>(c.ir_ & 0xFFE0);
        status |= ae.instruction ? (wasS ? 6 : 2) : (wasS ? 5 : 1);
        if (ae.instruction) status |= 0x08;
        if (ae.read)        status |= 0x10;

        // Jump-target fetch faults push target-4; data faults push pc-2
        // (or pc, for the few microcode paths that say so).
        const u32 pushPc = ae.instruction ? c.pc - 4
                                          : c.pc + static_cast<u32>(ae.pcBias);

        const int pre = ae.instruction ? ae.fetchExtra : c.eaFaultCycles_;
        c.eaFaultCycles_ = 0;
        c.push32(pushPc);
        c.push16(oldSR);
        c.push16(c.ir_);
        c.push32(ae.addr);
        c.push16(status);
        c.pc = c.rd32(kVecAddressError * 4);
        if (c.pc & 1) { c.halted = true; }
        return 50 + pre;
    } catch (const AddressError&) {
        c.halted = true;   // fault during fault processing: dead until reset
        return 4;
    }
}

// ---------------------------------------------------------------- CpuOps bits

std::array<CpuOps::Handler, 65536>& CpuOps::table() {
    static std::array<Handler, 65536> t = [] {
        std::array<Handler, 65536> tbl{};
        for (auto& h : tbl) h = &CpuOps::opIllegal;
        buildTableInto(tbl);
        return tbl;
    }();
    return t;
}

int CpuOps::eaTimeBW(int idx) {
    static constexpr int t[12] = {0, 0, 4, 4, 6, 8, 10, 8, 12, 8, 10, 4};
    return t[idx];
}

int CpuOps::eaTimeL(int idx) {
    static constexpr int t[12] = {0, 0, 8, 8, 10, 12, 14, 12, 16, 12, 14, 8};
    return t[idx];
}

u32 CpuOps::addFlags(M68000& c, u32 s, u32 t, int size, bool withX, bool stickyZ) {
    const u32 m = maskFor(size);
    s &= m; t &= m;
    const u32 x = (withX && flag(c, kX)) ? 1u : 0u;
    const u64 wide = static_cast<u64>(s) + t + x;
    const u32 r = static_cast<u32>(wide) & m;
    const bool carry = wide > m;
    const bool ovf = ((~(s ^ t)) & (s ^ r) & signBit(size)) != 0;
    setFlag(c, kV, ovf);
    setFlag(c, kC, carry);
    setFlag(c, kX, carry);
    setFlag(c, kN, (r & signBit(size)) != 0);
    if (stickyZ) { if (r != 0) setFlag(c, kZ, false); }
    else         { setFlag(c, kZ, r == 0); }
    return r;
}

u32 CpuOps::subFlags(M68000& c, u32 s, u32 t, int size, bool withX, bool stickyZ) {
    const u32 m = maskFor(size);
    s &= m; t &= m;
    const u32 x = (withX && flag(c, kX)) ? 1u : 0u;
    const u32 r = (t - s - x) & m;
    const bool borrow = static_cast<u64>(s) + x > t;
    const bool ovf = (((s ^ t) & (t ^ r)) & signBit(size)) != 0;
    setFlag(c, kV, ovf);
    setFlag(c, kC, borrow);
    setFlag(c, kX, borrow);
    setFlag(c, kN, (r & signBit(size)) != 0);
    if (stickyZ) { if (r != 0) setFlag(c, kZ, false); }
    else         { setFlag(c, kZ, r == 0); }
    return r;
}

void CpuOps::cmpFlags(M68000& c, u32 s, u32 t, int size) {
    const u32 m = maskFor(size);
    s &= m; t &= m;
    const u32 r = (t - s) & m;
    setFlag(c, kV, (((s ^ t) & (t ^ r)) & signBit(size)) != 0);
    setFlag(c, kC, s > t);
    setFlag(c, kN, (r & signBit(size)) != 0);
    setFlag(c, kZ, r == 0);
}

u32 CpuOps::briefExtension(M68000& c, u32 base) {
    const u16 ext = c.fetch16();
    const int xreg = (ext >> 12) & 7;
    u32 idx = (ext & 0x8000) ? c.a[xreg] : c.d[xreg];
    if (!(ext & 0x0800)) idx = static_cast<u32>(static_cast<s32>(static_cast<s16>(idx & 0xFFFF)));
    const s32 disp = static_cast<s8>(ext & 0xFF);
    return base + static_cast<u32>(disp) + idx;
}

u32 CpuOps::calcEA(M68000& c, int mode, int reg, int size) {
    switch (mode) {
    case 2:
        c.eaFaultCycles_ += 0;
        return c.a[reg];
    case 3: {
        const u32 addr = c.a[reg];
        c.eaUndoReg_ = reg;
        c.eaUndoVal_ = addr;
        c.eaUndoKind_ = 1;
        c.eaFaultCycles_ += 0;
        u32 inc = size == 0 ? 1u : size == 1 ? 2u : 4u;
        if (size == 0 && reg == 7) inc = 2;
        c.a[reg] += inc;
        return addr;
    }
    case 4: {
        u32 dec = size == 0 ? 1u : size == 1 ? 2u : 4u;
        if (size == 0 && reg == 7) dec = 2;
        c.a[reg] -= dec;
        c.eaFaultCycles_ += 2;  // odd address caught during the internal cycle
        return c.a[reg];
    }
    case 5: {
        const s32 disp = static_cast<s16>(c.fetch16());
        c.eaFaultCycles_ +=  4;
        return c.a[reg] + static_cast<u32>(disp);
    }
    case 6: {
        const u32 addr = briefExtension(c, c.a[reg]);
        c.eaFaultCycles_ += 6;  // caught during the index internal cycle
        return addr;
    }
    case 7:
        switch (reg) {
        case 0:
            c.eaFaultCycles_ += 4;
            return static_cast<u32>(static_cast<s32>(static_cast<s16>(c.fetch16())));
        case 1:
            c.eaFaultCycles_ += 8;
            return c.fetch32();
        case 2: {
            const u32 base = c.pc;
            const s32 disp = static_cast<s16>(c.fetch16());
            c.eaFaultCycles_ += 4;
            return base + static_cast<u32>(disp);
        }
        case 3: {
            const u32 addr = briefExtension(c, c.pc);
            c.eaFaultCycles_ += 6;
            return addr;
        }
        default: break;
        }
        break;
    default: break;
    }
    return 0; // unreachable for valid decodes
}

u32 CpuOps::calcPredecLowFirst(M68000& c, int reg, int size) {
    c.eaFaultCycles_ += 2;
    if (size < 2) {
        u32 dec = size == 0 ? 1u : 2u;
        if (size == 0 && reg == 7) dec = 2;
        c.a[reg] -= dec;
        return c.a[reg];
    }
    c.a[reg] -= 2;
    if (c.a[reg] & 1) throw AddressError{c.a[reg], true, false};
    c.a[reg] -= 2;
    return c.a[reg];
}

u32 CpuOps::readAt(M68000& c, u32 addr, int size) {
    if (size == 0) return c.rd8(addr);
    if (size == 1) return c.rd16(addr);
    return c.rd32(addr);
}

void CpuOps::writeAt(M68000& c, u32 addr, u32 v, int size) {
    if (size == 0)      c.wr8(addr, static_cast<u8>(v));
    else if (size == 1) c.wr16(addr, static_cast<u16>(v));
    else                c.wr32(addr, v);
}

u32 CpuOps::readEA(M68000& c, int mode, int reg, int size) {
    if (mode == 0) return c.d[reg] & maskFor(size);
    if (mode == 1) return c.a[reg] & maskFor(size);
    if (mode == 7 && reg == 4) {
        if (size == 2) return c.fetch32();
        const u16 w = c.fetch16();
        return size == 0 ? (w & 0xFFu) : w;
    }
    return readAt(c, calcEA(c, mode, reg, size), size);
}

void CpuOps::jumpTo(M68000& c, u32 target, int fetchExtra) {
    c.pc = target;   // hardware loads PC first, then faults on the fetch
    if (target & 1) throw AddressError{target, true, true, -2, fetchExtra};
}

bool CpuOps::testCond(const M68000& c, int cond) {
    const bool n = flag(c, kN), z = flag(c, kZ), v = flag(c, kV), cf = flag(c, kC);
    switch (cond) {
    case 0:  return true;
    case 1:  return false;
    case 2:  return !cf && !z;
    case 3:  return cf || z;
    case 4:  return !cf;
    case 5:  return cf;
    case 6:  return !z;
    case 7:  return z;
    case 8:  return !v;
    case 9:  return v;
    case 10: return !n;
    case 11: return n;
    case 12: return n == v;
    case 13: return n != v;
    case 14: return (n == v) && !z;
    default: return (n != v) || z;
    }
}

} // namespace openmac
