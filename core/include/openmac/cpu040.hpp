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

#include <array>
#include <functional>

namespace openmac {

struct CpuOps040;
class IifxStateCodec;

// The 68030 and 68040 share the 68020-generation integer instruction set and
// effective-address machinery, but they emphatically do not share MMU
// instructions, cache controls, exception frames, or timings.  Keep the
// common execution engine in one place and make those boundaries explicit.
enum class M68kCpuModel : u8 {
    M68030,
    M68040,
};

class M68040 {
public:
    explicit M68040(IBus040& bus);
    M68040(IBus040& bus, M68kCpuModel model);

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
    M68kCpuModel model() const { return model_; }
    bool is68030() const { return model_ == M68kCpuModel::M68030; }

    // Control registers (MOVEC): public for the debugger.
    u32 vbr  = 0;
    u32 sfc  = 0;
    u32 dfc  = 0;
    // Model-specific layouts: 040 uses CACR E1/E0 at 31/15 and TC E/P at
    // 15/14; 030 uses its low-order I/D cache controls and TC.E at bit 31.
    u32 cacr = 0;
    u32 tc   = 0;
    u32 itt0 = 0, itt1 = 0;
    u32 dtt0 = 0, dtt1 = 0;
    u32 urp  = 0, srp = 0;
    u32 mmusr = 0;

    // MC68030-only control/MMU state.  The integrated 030 MMU is programmed
    // with PMOVE rather than MOVEC: CRP/SRP are 64-bit root descriptors and
    // TC/TT0/TT1/MMUSR use the 68851-compatible layouts.  `tc` and `mmusr`
    // above hold the model-selected register; these additional fields cover
    // registers that have no 040 counterpart.
    u32 caar = 0;
    u32 tt0 = 0, tt1 = 0;
    u64 crp = 0, srp030 = 0;

    // FPU register file. Values are held in host double precision (documented
    // accuracy tier: the 80-bit extended memory FORMAT converts correctly, the
    // extra mantissa bits beyond a double do not round-trip).
    double fp[8]{};
    u32 fpcr = 0, fpsr = 0, fpiar = 0;

    bool stopped = false;    // STOP: waiting for an interrupt
    bool halted  = false;    // double fault: only reset() recovers

    // Recent instruction addresses, newest first (recentPc(0) = last executed).
    u32 recentPc(int back) const { return pcRing_[(pcRingPos_ - 1 - back) & 127]; }
    // Address of the instruction currently issuing bus accesses. Unlike `pc`,
    // this does not advance while extension words and operands are fetched.
    u32 instructionAddress() const { return instrStart_; }

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

    bool mmuEnabled() const {
        return is68030() ? ((tc & 0x80000000u) != 0)
                         : ((tc & 0x00008000u) != 0);
    }

    // Diagnostics: counts EA evaluations that used '020+ extension-word
    // features (scaled index or full format) -- encodings a 68000 would have
    // treated as don't-care bits. The differential test harness uses this to
    // separate architectural divergence from disagreement.
    u32 ext020Count = 0;

    // Diagnostics: the logical address of the most recent access error.
    u32 lastFaultAddr = 0;

    // Resolve an address using the current MMU state without performing the
    // final bus access. This is intentionally diagnostics-only: debuggers use
    // it to inspect logical low memory after the ROM enables the PMMU.
    u32 diagnosticTranslate(u32 logical, bool write = false,
                            bool supervisor = true, bool instruction = false) {
        return translateFc(logical, write, supervisor, instruction);
    }

    struct CacheStats030 {
        u64 instructionHits = 0;
        u64 instructionMisses = 0;
        u64 dataHits = 0;
        u64 dataMisses = 0;
        u64 burstLongwords = 0;
    };
    const CacheStats030& cacheStats030() const { return cacheStats030_; }

    // A device or compatibility bridge changed logical memory without going
    // through a CPU data write.  Real DMA users have to manage the IIfx caches;
    // the ROM-driver bridges use this to model their CPU-copy completion and
    // keep a previously read buffer from returning stale cache data. Both
    // return how many valid entries they dropped -- the count of times a
    // stale line would otherwise have been served.
    u32 invalidateDataCache030(u32 logical, u32 bytes);
    // Code a host bridge wrote into logical memory: a 68030 does not snoop
    // its instruction cache, so a stale line would run the previous bytes.
    u32 invalidateInstructionCache030(u32 logical, u32 bytes);

private:
    friend struct CpuOps040;
    friend class IifxStateCodec;

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
    u32 translateFc030(u32 laddr, bool write, u32 fc,
                       bool* cacheInhibit = nullptr);
    u32 tableWalk(u32 laddr, bool write, bool super, bool instruction);
    u32 tableWalk030(u32 laddr, bool write, u32 fc, bool* cacheInhibit);
    bool ttrMatch(u32 ttr, u32 laddr, bool super, bool write) const;
    bool ttrMatch030(u32 ttr, u32 laddr, u32 fc, bool write) const;
    void tlbFlush();                 // PFLUSHA / TC / RP writes
    void tlbFlushPage(u32 laddr);    // PFLUSH (An)

    // MC68030 logical I/D caches (MC68030UM section 6).  Each cache has 16
    // direct-mapped lines, four independently valid longwords per line.
    struct CacheLine030 {
        u32 tag = 0xFFFFFFFFu;        // logical A31..A8
        u8 fc = 0;                    // I$: FC2; D$: FC2..FC0
        u8 valid = 0;                 // one bit per longword
        std::array<u32, 4> data{};
    };
    u32 readCached030(u32 addr, int size, u32 fc, bool instruction);
    void writeCached030(u32 addr, u32 value, int size, u32 fc);
    void writeCacr030(u32 value);
    void clearCaches030();
    static u32 extractCached030(u32 word, u32 addr, int size);
    static u32 mergeCached030(u32 word, u32 addr, u32 value, int size);

    // Exception plumbing (frames built by CpuOps040 helpers).
    int exceptionFrame0(int vector, int cycles);
    int exceptionFrame2(int vector, u32 addr, int cycles);
    int doInterrupt(int level);

    IBus040& bus_;
    M68kCpuModel model_ = M68kCpuModel::M68040;
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
    bool lastEaFull030_ = false; // last EA used a full extension-word format
    bool fpuUsed_ = false; // FSAVE pushes NULL until the FPU has executed something

    std::array<CacheLine030, 16> instructionCache030_{};
    std::array<CacheLine030, 16> dataCache030_{};
    CacheStats030 cacheStats030_{};

    // Software ATC: direct-mapped, tag = vpn | super, per-entry write and
    // modified bits so a write through a read-established entry re-walks to
    // set the descriptor's M bit, as hardware does.
    struct TlbEntry {
        u32 tag = 0xFFFFFFFFu;   // vpn<<1 | super
        u32 phys = 0;            // physical page base
        bool writable = false;
        bool modified = false;
        bool cacheInhibit = false;
        u8 pageShift = 12;       // 030 supports 256 B through 32 KiB pages
        u8 fc = 0;               // 030 ATC tag includes all three FC bits
    };
    static constexpr int kTlbSize = 256;   // power of two
    TlbEntry tlb_[kTlbSize];
    int tlbNext030_ = 0;                  // 22-entry fully associative ATC
    int tlbLastHit030_ = -1;              // most recent ATC hit (lookup hint)
    TlbEntry& atcEntryFor030(u32 tag, u32 fc);
};

} // namespace openmac
