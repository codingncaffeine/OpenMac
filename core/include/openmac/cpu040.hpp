#pragma once

// Motorola 68040 interpreter. State-accurate, instruction-level cycle counts
// from the M68040 User's Manual timing tables (cache-hit case once the caches
// are enabled). Full integer ISA including the 68020 additions and the '040's
// MOVE16/CINV/CPUSH, the on-chip MMU (transparent translation + table walk +
// software ATC), and the on-chip FPU's hardware subset with unimplemented
// instructions trapping for the FPSP, as on silicon.
//
// Reference: M68040 User's Manual (M68040UM, 1993), M68000 Family Programmer's
// Reference Manual (M68000PRM). Clean-room from the manuals.

#include "openmac/bus040.hpp"
#include "openmac/types.hpp"

#include <functional>

namespace openmac {

struct CpuOps040;

class M68040 {
public:
    explicit M68040(IBus040& bus);

    // Load ISP and PC from vectors 0/1 (physical addresses 0/4), enter
    // supervisor mode with the interrupt stack, IRQ mask 7, MMU and caches
    // disabled, VBR 0.
    void reset();

    // Execute one instruction (or take a pending exception/interrupt).
    // Returns CPU clocks consumed.
    int step();

    // Level currently asserted by the interrupt controller (0 = none).
    // The Quadra autovectors every level.
    void setIrqLevel(int level);

    // Register file: public for the debugger and test harness. a[7] is always
    // the ACTIVE stack pointer; of usp/isp/msp, the fields for the inactive
    // banks hold those banks' true values. uspValue()/ispValue()/mspValue()
    // resolve any bank correctly.
    u32 d[8]{};
    u32 a[8]{};
    u32 usp = 0;
    u32 isp = 0;
    u32 msp = 0;
    u32 pc  = 0;

    u16  getSR() const { return sr_; }
    void setSR(u16 value);   // masks unimplemented bits, banks stack pointers
    void setCCR(u8 value);
    u32  uspValue() const;
    u32  ispValue() const;
    u32  mspValue() const;

    // Control registers (MOVEC): public for the debugger.
    u32 vbr  = 0;
    u32 sfc  = 0;
    u32 dfc  = 0;
    u32 cacr = 0;      // bit 31 = data cache enable, bit 15 = instruction cache enable
    u32 tc   = 0;      // bit 15 = MMU enable, bit 14 = page size (0 = 4K, 1 = 8K)
    u32 itt0 = 0, itt1 = 0;
    u32 dtt0 = 0, dtt1 = 0;
    u32 urp  = 0, srp = 0;
    u32 mmusr = 0;

    // FPU register file. Values are held in host double precision (documented
    // accuracy tier: the 80-bit extended memory FORMAT converts correctly, the
    // extra mantissa bits beyond a double do not round-trip).
    double fp[8]{};
    u32 fpcr = 0, fpsr = 0, fpiar = 0;

    bool stopped = false;    // STOP: waiting for an interrupt
    bool halted  = false;    // double fault: only reset() recovers

    // Recent instruction addresses, newest first (recentPc(0) = last executed).
    u32 recentPc(int back) const { return pcRing_[(pcRingPos_ - 1 - back) & 127]; }

    // The RESET instruction pulses /RSTO: peripherals reset, CPU continues.
    std::function<void()> onResetInstruction;

    // Fires as any exception is entered (vector, address of the faulting
    // instruction). Diagnostics only.
    std::function<void(int vector, u32 pc)> onException;

    // Fires when an A-line (Toolbox/OS trap) instruction executes, with the
    // trap opcode and its address. Used by the debugger for trap breakpoints.
    std::function<void(u16 opcode, u32 pc)> onTrap;

    // Fires when the CPU takes an interrupt, with the level (1-7), the
    // autovector number, and the PC being interrupted. Diagnostics only.
    std::function<void(int level, u32 autovector, u32 pc)> onInterrupt;

    // Fires before each instruction with the PC about to execute.
    std::function<void(u32 pc)> onStep;

    bool mmuEnabled() const { return (tc & 0x8000u) != 0; }

    // Diagnostics: counts EA evaluations that used '020+ extension-word
    // features (scaled index or full format) -- encodings a 68000 would have
    // treated as don't-care bits. The differential test harness uses this to
    // separate architectural divergence from disagreement.
    u32 ext020Count = 0;

    // Diagnostics: the logical address of the most recent access error.
    u32 lastFaultAddr = 0;

private:
    friend struct CpuOps040;

    // Internal fault, carrying what the format $7 access-error frame needs.
    struct AccessFault {
        u32  addr;         // faulting logical address
        bool read;
        int  size;         // bytes: 1, 2 or 4
        bool instruction;  // instruction-stream access
        bool atc;          // MMU table walk found no valid translation
    };

    // Logical-address data access. Misaligned accesses are split into
    // naturally-aligned pieces before translation, as the '040 bus does.
    u8   rd8(u32 addr);
    u16  rd16(u32 addr);
    u32  rd32(u32 addr);
    void wr8(u32 addr, u8 v);
    void wr16(u32 addr, u16 v);
    void wr32(u32 addr, u32 v);

    // Instruction-stream fetch at pc (advances pc). Odd pc raises an
    // address error (format $2, vector 3) before the bus sees it.
    u16 fetch16();
    u32 fetch32();

    void push16(u16 v);
    void push32(u32 v);
    u16  pop16();
    u32  pop32();

    // MMU: translate a logical address for one naturally-aligned access.
    // Returns the physical address; throws AccessFault when no valid,
    // permitted translation exists.
    u32 translate(u32 laddr, bool write, bool instruction);
    u32 translateFc(u32 laddr, bool write, bool super, bool instruction);
    u32 tableWalk(u32 laddr, bool write, bool super, bool instruction);
    bool ttrMatch(u32 ttr, u32 laddr, bool super, bool write) const;
    void tlbFlush();                 // PFLUSHA / TC / RP writes
    void tlbFlushPage(u32 laddr);    // PFLUSH (An)

    // Exception plumbing (frames built by CpuOps040 helpers).
    int exceptionFrame0(int vector, int cycles);
    int exceptionFrame2(int vector, u32 addr, int cycles);
    int doInterrupt(int level);

    IBus040& bus_;
    u16 sr_ = 0x2700;
    int irqLevel_ = 0;
    u32 instrStart_ = 0;   // address of the currently executing instruction
    u16 ir_ = 0;           // currently executing opcode
    u32 pcRing_[128]{};    // recent instruction addresses (debug)
    int pcRingPos_ = 0;

    // Restart semantics: the '040 access-error model re-executes the faulting
    // instruction, so a fault must leave no committed register state behind.
    // The register file is snapshotted per instruction and restored on fault
    // (memory side effects stand, as they do through the real chip's restart).
    u32 snapD_[8]{}, snapA_[8]{};
    u16 snapSR_ = 0;

    // Extra clocks accumulated during EA evaluation (full-format extension
    // words, memory indirection); step() adds them to the handler's return.
    int eaExtra_ = 0;
    bool fpuUsed_ = false; // FSAVE pushes NULL until the FPU has executed something

    // Software ATC: direct-mapped, tag = vpn | super, per-entry write and
    // modified bits so a write through a read-established entry re-walks to
    // set the descriptor's M bit, as hardware does.
    struct TlbEntry {
        u32 tag = 0xFFFFFFFFu;   // vpn<<1 | super
        u32 phys = 0;            // physical page base
        bool writable = false;
        bool modified = false;
    };
    static constexpr int kTlbSize = 256;   // power of two
    TlbEntry tlb_[kTlbSize];
};

} // namespace openmac
