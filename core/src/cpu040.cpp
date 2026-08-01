#include "cpu040_ops.hpp"

// M68040 core engine: reset, the three-bank stack model, MMU-translated bus
// access with misaligned-access splitting, exception frame construction, and
// the step loop. Instruction handlers live in the cpu040_ops_*.cpp files.
//
// Reference: M68040UM sections 2/3/8; frame layouts from section 8.4.
// Clean-room from the manuals.

namespace openmac {

// ---------------------------------------------------------------- construction

M68040::M68040(IBus040& bus) : bus_(bus) {
    (void)CpuOps040::table(); // force table construction up front
}

void M68040::reset() {
    sr_      = 0x2700;        // supervisor, interrupt stack, mask 7, no trace
    stopped  = false;
    halted   = false;
    vbr      = 0;
    sfc = dfc = 0;
    cacr     = 0;
    tc      &= 0x4000u;       // reset clears only E; the P (page size) bit survives
    itt0 = itt1 = dtt0 = dtt1 = 0;
    urp = srp = 0;
    mmusr    = 0;
    fpuUsed_ = false;
    fpcr = fpsr = fpiar = 0;
    tlbFlush();
    isp      = bus_.read32(0);
    pc       = bus_.read32(4);
    a[7]     = isp;
}

void M68040::setIrqLevel(int level) { irqLevel_ = level; }

// Of usp/isp/msp, the field for an inactive bank holds that bank's true value;
// the active bank's truth lives in a[7].
u32 M68040::uspValue() const {
    return (sr_ & kS040) ? usp : a[7];
}
u32 M68040::ispValue() const {
    if ((sr_ & kS040) && !(sr_ & kM040)) return a[7];
    return isp;
}
u32 M68040::mspValue() const {
    if ((sr_ & kS040) && (sr_ & kM040)) return a[7];
    return msp;
}

void M68040::setSR(u16 value) {
    value &= kSrMask040;
    // Bank a[7] out to wherever the CURRENT mode keeps its stack pointer...
    if (sr_ & kS040) {
        if (sr_ & kM040) msp = a[7]; else isp = a[7];
    } else {
        usp = a[7];
    }
    sr_ = value;
    // ...and bank in the stack the NEW mode selects.
    if (sr_ & kS040) {
        a[7] = (sr_ & kM040) ? msp : isp;
    } else {
        a[7] = usp;
    }
}

void M68040::setCCR(u8 value) {
    sr_ = static_cast<u16>((sr_ & 0xFF00) | (value & 0x1F));
}

// ---------------------------------------------------------------- MMU

// Transparent translation register match. Fields (M68040UM 3.1.2): bits 31-24
// logical address base, 23-16 address mask (set bits = don't care), 15 enable,
// 14-13 S-field (00 user, 01 supervisor, 1x both), 2 write-protect.
bool M68040::ttrMatch(u32 ttr, u32 laddr, bool super, bool /*write*/) const {
    if (!(ttr & 0x8000u)) return false;
    const u32 sField = (ttr >> 13) & 3;
    if (sField == 0 && super) return false;
    if (sField == 1 && !super) return false;
    const u32 mask = (ttr >> 16) & 0xFFu;
    const u32 base = (ttr >> 24) & 0xFFu;
    const u32 la8  = (laddr >> 24) & 0xFFu;
    return ((la8 ^ base) & ~mask) == 0;
}

void M68040::tlbFlush() {
    for (auto& e : tlb_) e = TlbEntry{};
}

void M68040::tlbFlushPage(u32 laddr) {
    const u32 pageShift = (tc & 0x4000u) ? 13u : 12u;
    const u32 vpn = laddr >> pageShift;
    // Both the user and supervisor mappings of the page go.
    for (u32 s = 0; s < 2; ++s) {
        TlbEntry& e = tlb_[((vpn << 1) | s) & (kTlbSize - 1)];
        if (e.tag == ((vpn << 1) | s)) e = TlbEntry{};
    }
}

u32 M68040::translate(u32 laddr, bool write, bool instruction) {
    if (!mmuEnabled()) return laddr;
    return translateFc(laddr, write, (sr_ & kS040) != 0, instruction);
}

u32 M68040::translateFc(u32 laddr, bool write, bool super, bool instruction) {
    if (!mmuEnabled()) return laddr;
    // Transparent translation is checked ahead of the ATC (UM 3.2).
    const u32 t0 = instruction ? itt0 : dtt0;
    const u32 t1 = instruction ? itt1 : dtt1;
    if (ttrMatch(t0, laddr, super, write)) {
        if (write && (t0 & 0x4u)) throw AccessFault{laddr, false, 1, instruction, false};
        return laddr;
    }
    if (ttrMatch(t1, laddr, super, write)) {
        if (write && (t1 & 0x4u)) throw AccessFault{laddr, false, 1, instruction, false};
        return laddr;
    }
    const u32 pageShift = (tc & 0x4000u) ? 13u : 12u;
    const u32 vpn = laddr >> pageShift;
    const u32 tag = (vpn << 1) | (super ? 1u : 0u);
    TlbEntry& e = tlb_[tag & (kTlbSize - 1)];
    if (e.tag == tag) {
        // A write through an entry whose page hasn't had its M bit set yet
        // re-walks so the descriptor in memory is updated, as hardware does.
        if (!write || (e.writable && e.modified))
            return e.phys | (laddr & ((1u << pageShift) - 1u));
        if (!e.writable) throw AccessFault{laddr, false, 1, instruction, false};
    }
    return tableWalk(laddr, write, super, instruction);
}

// Three-level table walk (M68040UM 3.3): 7-bit root index, 7-bit pointer
// index, 6-bit (4K) or 5-bit (8K) page index. Upper-level descriptors:
// UDT bits 1-0 (00/01 invalid, 10/11 resident), U bit 3. Page descriptors:
// PDT 00 invalid, 01/11 resident, 10 indirect; W bit 2, U bit 3, M bit 4,
// S bit 7. U and M are updated in memory as hardware would.
u32 M68040::tableWalk(u32 laddr, bool write, bool super, bool instruction) {
    const bool page8k = (tc & 0x4000u) != 0;
    const u32 pageShift = page8k ? 13u : 12u;
    const u32 ri = laddr >> 25;
    const u32 pi = (laddr >> 18) & 0x7Fu;
    const u32 pgi = page8k ? ((laddr >> 13) & 0x1Fu) : ((laddr >> 12) & 0x3Fu);

    bool wp = false;
    try {
        const u32 rp = (super ? srp : urp) & 0xFFFFFE00u;
        const u32 d1a = rp + ri * 4;
        u32 d1 = bus_.read32(d1a);
        if ((d1 & 2u) == 0) throw AccessFault{laddr, !write, 1, instruction, true};
        if (d1 & 4u) wp = true;
        if (!(d1 & 8u)) bus_.write32(d1a, d1 | 8u);   // U bit

        const u32 ptBase = d1 & 0xFFFFFE00u;
        const u32 d2a = ptBase + pi * 4;
        u32 d2 = bus_.read32(d2a);
        if ((d2 & 2u) == 0) throw AccessFault{laddr, !write, 1, instruction, true};
        if (d2 & 4u) wp = true;
        if (!(d2 & 8u)) bus_.write32(d2a, d2 | 8u);

        const u32 pgBase = d2 & (page8k ? 0xFFFFFF80u : 0xFFFFFF00u);
        u32 d3a = pgBase + pgi * 4;
        u32 d3 = bus_.read32(d3a);
        if ((d3 & 3u) == 2u) {           // indirect: one level only
            d3a = d3 & 0xFFFFFFFCu;
            d3 = bus_.read32(d3a);
            if ((d3 & 3u) == 2u || (d3 & 3u) == 0u)
                throw AccessFault{laddr, !write, 1, instruction, true};
        }
        if ((d3 & 3u) == 0u) throw AccessFault{laddr, !write, 1, instruction, true};
        if (d3 & 4u) wp = true;
        if ((d3 & 0x80u) && !super)      // supervisor-only page
            throw AccessFault{laddr, !write, 1, instruction, true};
        if (write && wp)                 // write to protected: M stays clear
            throw AccessFault{laddr, false, 1, instruction, false};

        u32 newBits = d3 | 8u;                     // U
        if (write) newBits |= 0x10u;               // M
        if (newBits != d3) bus_.write32(d3a, newBits);

        const u32 physPage = d3 & (page8k ? 0xFFFFE000u : 0xFFFFF000u);
        const u32 vpn = laddr >> pageShift;
        const u32 tag = (vpn << 1) | (super ? 1u : 0u);
        TlbEntry& e = tlb_[tag & (kTlbSize - 1)];
        e.tag = tag;
        e.phys = physPage;
        e.writable = !wp;
        e.modified = (newBits & 0x10u) != 0;
        return physPage | (laddr & ((1u << pageShift) - 1u));
    } catch (const BusFault&) {
        // A bus error during the walk itself surfaces as an ATC fault.
        throw AccessFault{laddr, !write, 1, instruction, true};
    }
}

// ---------------------------------------------------------------- bus access

// Every access is translated per naturally-aligned piece; the '040 bus
// controller performs the same splitting for misaligned operands, so a
// misaligned long that crosses a page boundary translates each piece.

u8 M68040::rd8(u32 addr) {
    const u32 pa = translate(addr, false, false);
    try {
        return bus_.read8(pa);
    } catch (const BusFault&) {
        throw AccessFault{addr, true, 1, false, false};
    }
}

u16 M68040::rd16(u32 addr) {
    if (addr & 1) {
        const u16 hi = rd8(addr);
        return static_cast<u16>((hi << 8) | rd8(addr + 1));
    }
    const u32 pa = translate(addr, false, false);
    try {
        return bus_.read16(pa);
    } catch (const BusFault&) {
        throw AccessFault{addr, true, 2, false, false};
    }
}

u32 M68040::rd32(u32 addr) {
    if (addr & 3) {
        if ((addr & 1) == 0) {
            const u32 hi = rd16(addr);
            return (hi << 16) | rd16(addr + 2);
        }
        const u32 b0 = rd8(addr);
        const u32 w  = rd16(addr + 1);
        const u32 b3 = rd8(addr + 3);
        return (b0 << 24) | (w << 8) | b3;
    }
    const u32 pa = translate(addr, false, false);
    try {
        return bus_.read32(pa);
    } catch (const BusFault&) {
        throw AccessFault{addr, true, 4, false, false};
    }
}

void M68040::wr8(u32 addr, u8 v) {
    const u32 pa = translate(addr, true, false);
    try {
        bus_.write8(pa, v);
    } catch (const BusFault&) {
        throw AccessFault{addr, false, 1, false, false};
    }
}

void M68040::wr16(u32 addr, u16 v) {
    if (addr & 1) {
        wr8(addr, static_cast<u8>(v >> 8));
        wr8(addr + 1, static_cast<u8>(v & 0xFF));
        return;
    }
    const u32 pa = translate(addr, true, false);
    try {
        bus_.write16(pa, v);
    } catch (const BusFault&) {
        throw AccessFault{addr, false, 2, false, false};
    }
}

void M68040::wr32(u32 addr, u32 v) {
    if (addr & 3) {
        if ((addr & 1) == 0) {
            wr16(addr, static_cast<u16>(v >> 16));
            wr16(addr + 2, static_cast<u16>(v & 0xFFFF));
        } else {
            wr8(addr, static_cast<u8>(v >> 24));
            wr16(addr + 1, static_cast<u16>((v >> 8) & 0xFFFF));
            wr8(addr + 3, static_cast<u8>(v & 0xFF));
        }
        return;
    }
    const u32 pa = translate(addr, true, false);
    try {
        bus_.write32(pa, v);
    } catch (const BusFault&) {
        throw AccessFault{addr, false, 4, false, false};
    }
}

u16 M68040::fetch16() {
    if (pc & 1) throw AccessFault{pc, true, 2, true, false};
    const u32 pa = translate(pc, false, true);
    u16 v;
    try {
        v = bus_.read16(pa);
    } catch (const BusFault&) {
        throw AccessFault{pc, true, 2, true, false};
    }
    pc += 2;
    return v;
}

u32 M68040::fetch32() {
    const u32 hi = fetch16();
    return (hi << 16) | fetch16();
}

void M68040::push16(u16 v) { a[7] -= 2; wr16(a[7], v); }
void M68040::push32(u32 v) { a[7] -= 4; wr32(a[7], v); }
u16  M68040::pop16() { const u16 v = rd16(a[7]); a[7] += 2; return v; }
u32  M68040::pop32() { const u32 v = rd32(a[7]); a[7] += 4; return v; }

// ---------------------------------------------------------------- exceptions

// Four-word format $0 frame: SR, PC, format|vector.
int M68040::exceptionFrame0(int vector, int cycles) {
    if (onException) onException(vector, instrStart_);
    const u16 oldSR = sr_;
    setSR(static_cast<u16>((sr_ | kS040) & ~(kT1040 | kT0040)));
    push16(static_cast<u16>((vector * 4) & 0xFFF));
    push32(pc);
    push16(oldSR);
    pc = rd32(vbr + static_cast<u32>(vector) * 4);
    if (pc & 1) halted = true;
    return cycles;
}

// Six-word format $2 frame: SR, PC, format|vector, instruction address.
int M68040::exceptionFrame2(int vector, u32 addr, int cycles) {
    if (onException) onException(vector, instrStart_);
    const u16 oldSR = sr_;
    setSR(static_cast<u16>((sr_ | kS040) & ~(kT1040 | kT0040)));
    push32(addr);
    push16(static_cast<u16>(0x2000 | ((vector * 4) & 0xFFF)));
    push32(pc);
    push16(oldSR);
    pc = rd32(vbr + static_cast<u32>(vector) * 4);
    if (pc & 1) halted = true;
    return cycles;
}

int M68040::doInterrupt(int level) {
    stopped = false;
    if (onInterrupt)
        onInterrupt(level, static_cast<u32>(kVec040Autovector + level), pc);
    const u16 oldSR = sr_;
    setSR(static_cast<u16>((sr_ | kS040) & ~(kT1040 | kT0040)));
    sr_ = static_cast<u16>((sr_ & ~0x0700) | (level << 8));
    const int vec = kVec040Autovector + level;
    push16(static_cast<u16>((vec * 4) & 0xFFF));
    push32(pc);
    push16(oldSR);
    if (sr_ & kM040) {
        // The interrupt frame went on the master stack; now switch to the
        // interrupt stack and leave a throwaway format $1 frame there. RTE of
        // the throwaway restores M and resumes RTE processing on the master.
        const u16 midSR = sr_;
        setSR(static_cast<u16>(sr_ & ~kM040));
        push16(static_cast<u16>(0x1000 | ((vec * 4) & 0xFFF)));
        push32(pc);
        push16(midSR);
    }
    pc = rd32(vbr + static_cast<u32>(vec) * 4);
    if (pc & 1) halted = true;
    return 30;
}

// Access error: the 30-word format $7 frame (M68040UM 8.4.3). We fault with
// restart semantics -- no partial state is committed, the stacked PC is the
// faulting instruction, the writeback fields are pushed invalid -- so an RTE
// simply re-executes the instruction. The Mac ROM's probe handlers adjust the
// stacked PC themselves when they want to skip the faulting access.
int CpuOps040::enterAccessError(M68040& c, const M68040::AccessFault& f) {
    if (c.onException) c.onException(kVec040AccessError, c.instrStart_);
    try {
        const u16 oldSR = c.sr_;
        c.setSR(static_cast<u16>((oldSR | kS040) & ~(kT1040 | kT0040)));

        // SSW: RW (bit 8), SIZE (bits 6-5: 01 byte, 10 word, 00 long),
        // TM (bits 2-0: data/code, user/super), ATC (bit 10) for MMU misses.
        u16 ssw = 0;
        if (f.read) ssw |= 0x0100;
        if (f.atc)  ssw |= 0x0400;
        const u16 sizeBits = f.size == 1 ? 1u : f.size == 2 ? 2u : 0u;
        ssw |= static_cast<u16>(sizeBits << 5);
        const bool wasS = (oldSR & kS040) != 0;
        u16 tm = f.instruction ? (wasS ? 6u : 2u) : (wasS ? 5u : 1u);
        ssw |= tm;

        for (int i = 0; i < 4; ++i) c.push32(0);   // PD3..PD0 (push data)
        c.push32(0);                               // WB1A
        c.push32(0);                               // WB2D
        c.push32(0);                               // WB2A
        c.push32(0);                               // WB3D
        c.push32(0);                               // WB3A
        c.push32(f.addr);                          // FA (fault address)
        c.push16(0);                               // WB1S (invalid)
        c.push16(0);                               // WB2S
        c.push16(0);                               // WB3S
        c.push16(ssw);
        c.push32(f.addr);                          // effective address
        c.push16(static_cast<u16>(0x7000 | ((kVec040AccessError * 4) & 0xFFF)));
        c.push32(c.instrStart_);                   // restart: the faulting instruction
        c.push16(oldSR);
        c.pc = c.rd32(c.vbr + kVec040AccessError * 4);
        if (c.pc & 1) c.halted = true;
        return 50;
    } catch (const M68040::AccessFault&) {
        c.halted = true;   // fault during fault processing: dead until reset
        return 4;
    }
}

int M68040::step() {
    if (halted) return 4;

    const int mask = (sr_ >> 8) & 7;
    if (irqLevel_ > 0 && (irqLevel_ == 7 || irqLevel_ > mask)) {
        return doInterrupt(irqLevel_);
    }
    if (stopped) return 4;

    if (onStep) onStep(pc);

    const bool traced = (sr_ & kT1040) != 0;
    instrStart_ = pc;
    pcRing_[pcRingPos_] = pc;
    pcRingPos_ = (pcRingPos_ + 1) & 127;
    for (int i = 0; i < 8; ++i) { snapD_[i] = d[i]; snapA_[i] = a[i]; }
    snapSR_ = sr_;
    eaExtra_ = 0;
    try {
        const u16 op = fetch16();
        ir_ = op;
        int cycles = CpuOps040::table()[op](*this, op) + eaExtra_;
        if (traced && !stopped && !halted) {
            cycles += exceptionFrame2(kVec040Trace, instrStart_, 25);
        }
        return cycles;
    } catch (const AccessFault& f) {
        // Restart semantics: put the register file back the way the
        // instruction found it. Memory side effects stand. The snapshot's
        // a[7] is coherent with the snapshot SR's stack banking.
        for (int i = 0; i < 8; ++i) { d[i] = snapD_[i]; a[i] = snapA_[i]; }
        sr_ = snapSR_;
        if (f.instruction && !f.atc && (f.addr & 1)) {
            // Odd instruction address: address error, format $2 vector 3.
            pc = instrStart_;
            return exceptionFrame2(kVec040AddressError, f.addr, 25);
        }
        pc = instrStart_;
        return CpuOps040::enterAccessError(*this, f);
    }
}

// ---------------------------------------------------------------- CpuOps040 bits

std::array<CpuOps040::Handler, 65536>& CpuOps040::table() {
    static std::array<Handler, 65536> t = [] {
        std::array<Handler, 65536> tbl{};
        for (auto& h : tbl) h = &CpuOps040::opIllegal;
        buildTableInto(tbl);
        return tbl;
    }();
    return t;
}

// '040 effective-address EXTRA clocks beyond an instruction's execute-stage
// base (M68040UM section 10 tables). Simple modes overlap entirely with the
// <ea> fetch stage and add nothing; the indexed forms occupy the interlocked
// <ea> calculate stage. Full-format extension words add more via eaExtra_
// (indexExtension), which step() folds into the instruction's total.
int CpuOps040::eaTime(int idx) {
    static constexpr int t[12] = {0, 0, 0, 0, 0, 0, 2, 0, 0, 1, 3, 0};
    return t[idx];
}

u32 CpuOps040::addFlags(M68040& c, u32 s, u32 t, int size, bool withX, bool stickyZ) {
    const u32 m = maskFor(size);
    s &= m; t &= m;
    const u32 x = (withX && flag(c, kX040)) ? 1u : 0u;
    const u64 wide = static_cast<u64>(s) + t + x;
    const u32 r = static_cast<u32>(wide) & m;
    const bool carry = wide > m;
    const bool ovf = ((~(s ^ t)) & (s ^ r) & signBit(size)) != 0;
    setFlag(c, kV040, ovf);
    setFlag(c, kC040, carry);
    setFlag(c, kX040, carry);
    setFlag(c, kN040, (r & signBit(size)) != 0);
    if (stickyZ) { if (r != 0) setFlag(c, kZ040, false); }
    else         { setFlag(c, kZ040, r == 0); }
    return r;
}

u32 CpuOps040::subFlags(M68040& c, u32 s, u32 t, int size, bool withX, bool stickyZ) {
    const u32 m = maskFor(size);
    s &= m; t &= m;
    const u32 x = (withX && flag(c, kX040)) ? 1u : 0u;
    const u32 r = (t - s - x) & m;
    const bool borrow = static_cast<u64>(s) + x > t;
    const bool ovf = (((s ^ t) & (t ^ r)) & signBit(size)) != 0;
    setFlag(c, kV040, ovf);
    setFlag(c, kC040, borrow);
    setFlag(c, kX040, borrow);
    setFlag(c, kN040, (r & signBit(size)) != 0);
    if (stickyZ) { if (r != 0) setFlag(c, kZ040, false); }
    else         { setFlag(c, kZ040, r == 0); }
    return r;
}

void CpuOps040::cmpFlags(M68040& c, u32 s, u32 t, int size) {
    const u32 m = maskFor(size);
    s &= m; t &= m;
    const u32 r = (t - s) & m;
    setFlag(c, kV040, (((s ^ t) & (t ^ r)) & signBit(size)) != 0);
    setFlag(c, kC040, s > t);
    setFlag(c, kN040, (r & signBit(size)) != 0);
    setFlag(c, kZ040, r == 0);
}

u32 CpuOps040::readAt(M68040& c, u32 addr, int size) {
    if (size == 0) return c.rd8(addr);
    if (size == 1) return c.rd16(addr);
    return c.rd32(addr);
}

void CpuOps040::writeAt(M68040& c, u32 addr, u32 v, int size) {
    if (size == 0)      c.wr8(addr, static_cast<u8>(v));
    else if (size == 1) c.wr16(addr, static_cast<u16>(v));
    else                c.wr32(addr, v);
}

u32 CpuOps040::fetchImm(M68040& c, int size) {
    if (size == 2) return c.fetch32();
    const u16 w = c.fetch16();
    return size == 0 ? (w & 0xFFu) : w;
}

u32 CpuOps040::readEA(M68040& c, int mode, int reg, int size) {
    if (mode == 0) return c.d[reg] & maskFor(size);
    if (mode == 1) return c.a[reg] & maskFor(size);
    if (mode == 7 && reg == 4) return fetchImm(c, size);
    return readAt(c, calcEA(c, mode, reg, size), size);
}

void CpuOps040::jumpTo(M68040& c, u32 target) {
    c.pc = target;
    if (target & 1) throw M68040::AccessFault{target, true, 2, true, false};
}

bool CpuOps040::testCond(const M68040& c, int cond) {
    const bool n = flag(c, kN040), z = flag(c, kZ040);
    const bool v = flag(c, kV040), cf = flag(c, kC040);
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
