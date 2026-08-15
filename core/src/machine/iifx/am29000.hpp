#pragma once

// AMD Am29000 execution core used by the Macintosh Display Card 8*24 GC.
//
// The instruction implementations are adapted from MAME's portable Am29000
// core (BSD-3-Clause, copyright Philip Bennett).  This wrapper removes MAME's
// device framework and exposes explicit Harvard instruction/data callbacks so
// Dolphin's MFB can supply the card's real memory map.

#include "openmac/types.hpp"

#include <array>
#include <functional>
#include <string>
#include <vector>

namespace openmac {

class IifxStateCodec;

class Am29000 {
public:
    // The MFB needs to distinguish reset/physical fetches from translated
    // instruction fetches; the latter use its external MMU and SRAM cache.
    std::function<u32(u32, bool)> readInstruction;
    // Direct instruction-fetch windows: the memory device may register
    // physical ranges backed by contiguous host arrays so ordinary fetches
    // skip the readInstruction callback entirely.  Fetches outside every
    // window (or before any is registered) take the callback, which the
    // device uses to (re)register the windows -- so a checkpoint load only
    // has to clear them.  Purely a host-side fast path; no emulated state.
    struct FetchWindow {
        u32 base = 0;
        u32 size = 0;
        const u8* data = nullptr;
    };
    static constexpr std::size_t kFetchWindows = 6;
    void setFetchWindow(std::size_t index, u32 base, u32 size,
                        const u8* data) {
        if (index >= kFetchWindows) return;
        fetchWindows_[index] = FetchWindow{base, size, data};
    }
    void clearFetchWindows() {
        for (auto& window : fetchWindows_) window = FetchWindow{};
        fetchTranslationPage_ = nullptr;
    }
    // Direct data windows: physical ranges of plain memory backed by
    // contiguous big-endian host arrays, so an ordinary LOAD/STORE that
    // lands in one is a bounds check and a byte-swapped access instead of
    // the readData/peekData/writeData callbacks.  A window is only ever
    // registered while the board's model can promise the access has no side
    // effect beyond the bytes and the listed counters: the board clears the
    // table the moment that stops being true (an MFB cache fill armed, a
    // diagnostic tap or watch armed, a checkpoint loaded) and its slow path
    // re-registers when allowed again.  Every word-sized access that resolves
    // in a window adds each counter's `add` -- the counters the slow path
    // would have bumped (the board's serialized access totals), so a state
    // file is byte-identical with or without the fast path.  A sub-word
    // store's read-modify-write bumps the read counters as its slow-path
    // peek would.  Purely a host-side fast path; no emulated state.
    struct DataCounter {
        u64* value = nullptr;
        u32 add = 0;
    };
    struct DataWindow {
        u32 base = 0;
        u32 size = 0;
        u8* data = nullptr;
        bool writable = false;
        std::array<DataCounter, 3> readCounters{};
        std::array<DataCounter, 3> writeCounters{};
    };
    static constexpr std::size_t kDataWindows = 8;
    void setDataWindow(std::size_t index, const DataWindow& window) {
        if (index >= kDataWindows) return;
        dataWindows_[index] = window;
    }
    void clearDataWindows() {
        for (auto& window : dataWindows_) window = DataWindow{};
        lastDataWindow_ = 0;
    }
    bool dataWindowsArmed() const {
        for (const auto& window : dataWindows_)
            if (window.data) return true;
        return false;
    }
    // Vector fetches are physical data-memory cycles with distinct STAT
    // signalling on the Am29000 bus.  Cards such as Dolphin use that signal
    // to select a small MFB vector store rather than ordinary data SRAM.
    std::function<u32(u32)> readVector;
    // The AS bit on every load/store is a physical bus signal, not part of
    // the numeric address.  Keep it alongside the address so a board can map
    // instruction/data memory independently from input/output space.
    std::function<u32(u32, bool inputOutput)> readData;
    // Sub-word stores use a read/modify/write fallback when a board only
    // supplies a word-wide data callback.  Devices with read-sensitive
    // registers can provide a side-effect-free value for that merge.
    std::function<u32(u32, bool inputOutput)> peekData;
    std::function<void(u32, u32, bool inputOutput)> writeData;

    Am29000();

    void reset(u32 resetPc = 0);
    int run(int instructionSlots);
    void setInput(int input, bool asserted);
    // External interrupt/timer recognition hold, driven by MFB register
    // +$088: GCOS raises it around the context-switch register-file
    // restore so the 1 kHz tick cannot preempt the loop whose own pointer
    // registers the tick handler clobbers.  Derived state — the video
    // model reapplies it from its serialized register shadow on load.
    void setInterruptHold(bool held) { interruptHold_ = held; }
    bool interruptHold() const { return interruptHold_; }

    u32 pc() const { return m_pc; }
    u32 instructionPc() const { return m_exec_pc; }
    u32 pc0() const { return m_pc0; }
    u32 pc1() const { return m_pc1; }
    u32 pc2() const { return m_pc2; }
    u32 nextPc() const { return m_next_pc; }
    u32 iretPc() const { return m_iret_pc; }
    u32 executingInstruction() const { return m_exec_ir; }
    u32 prefetchedInstruction() const { return m_next_ir; }
    u32 pipelineFlags() const { return m_pl_flags; }
    u32 nextPipelineFlags() const { return m_next_pl_flags; }
    u32 oldProcessorStatus() const { return m_ops; }
    std::size_t recentInstructionCount() const { return recentCount_; }
    u32 recentInstructionPc(std::size_t back) const {
        if (back >= recentCount_) return 0;
        return recentPc_[(recentHead_ - 1u - back) &
                         (recentPc_.size() - 1u)];
    }
    u32 recentInstruction(std::size_t back) const {
        if (back >= recentCount_) return 0;
        return recentIr_[(recentHead_ - 1u - back) &
                         (recentIr_.size() - 1u)];
    }
    // Watch one PHYSICAL register (0-255) for writes; rolling ring of the
    // writer pcs and values.  0xFFFF disables.  Diagnostic-only.
    void setDiagnosticRegisterWatch(u16 physicalIndex) {
        registerWatchIndex_ = physicalIndex;
        refreshProbeFlag();
    }
    std::size_t diagnosticRegisterWatchCount() const {
        return registerWatchCount_;
    }
    u32 diagnosticRegisterWatchPc(std::size_t back) const {
        if (back >= registerWatchCount_) return 0;
        return registerWatchPc_[(registerWatchHead_ - 1u - back) &
                                (registerWatchPc_.size() - 1u)];
    }
    u32 diagnosticRegisterWatchValue(std::size_t back) const {
        if (back >= registerWatchCount_) return 0;
        return registerWatchValue_[(registerWatchHead_ - 1u - back) &
                                   (registerWatchValue_.size() - 1u)];
    }
    u64 diagnosticRegisterWatchInstruction(std::size_t back) const {
        if (back >= registerWatchCount_) return 0;
        return registerWatchInstruction_[(registerWatchHead_ - 1u - back) &
                                         (registerWatchInstruction_.size() -
                                          1u)];
    }
    // The data bus reports *DERR for the access in flight: the executing
    // load/store raises the Data Access Exception with CHC.TF set instead
    // of completing.  Called by the bus model from inside the access.
    void noteDataBusFault() { dataBusFault_ = true; }
    // Snapshot one ARCHITECTURAL register (0-127 = gr, 128-255 = lr through
    // the live window rotation) plus gr1 each time one pc is about to
    // execute; rolling ring.  pc 0 disables.  Diagnostic-only.
    void setDiagnosticPcSnap(u32 pc, u16 architecturalIndex) {
        pcSnapPc_ = pc;
        pcSnapRegister_ = architecturalIndex;
        refreshProbeFlag();
    }
    // Flight recorder: capture pc/gr1/gr126/gr127 + one PHYSICAL register
    // for every instruction in [start, start+count).  Diagnostic-only.
    struct FlightEntry {
        u64 instr;
        u32 pc, gr1, rfb, rab, watched;
    };
    void setDiagnosticFlightWindow(u64 start, u64 count, u16 physicalIndex) {
        flightStart_ = start;
        flightCount_ = count;
        flightWatchIndex_ = physicalIndex;
        flight_.clear();
        if (count != 0 && count <= (1u << 20)) flight_.reserve(count);
        refreshProbeFlag();
    }
    const std::vector<FlightEntry>& diagnosticFlight() const {
        return flight_;
    }
    std::size_t diagnosticPcSnapCount() const { return pcSnapCount_; }
    u64 diagnosticPcSnapInstruction(std::size_t back) const {
        if (back >= pcSnapCount_) return 0;
        return pcSnapInstruction_[(pcSnapHead_ - 1u - back) &
                                  (pcSnapInstruction_.size() - 1u)];
    }
    u32 diagnosticPcSnapWindowBase(std::size_t back) const {
        if (back >= pcSnapCount_) return 0;
        return pcSnapGr1_[(pcSnapHead_ - 1u - back) &
                          (pcSnapGr1_.size() - 1u)];
    }
    u32 diagnosticPcSnapValue(std::size_t back) const {
        if (back >= pcSnapCount_) return 0;
        return pcSnapValue_[(pcSnapHead_ - 1u - back) &
                            (pcSnapValue_.size() - 1u)];
    }
    u32 diagnosticPcSnapSpillBound(std::size_t back) const {
        if (back >= pcSnapCount_) return 0;
        return pcSnapGr126_[(pcSnapHead_ - 1u - back) &
                            (pcSnapGr126_.size() - 1u)];
    }
    u32 diagnosticPcSnapFillBound(std::size_t back) const {
        if (back >= pcSnapCount_) return 0;
        return pcSnapGr127_[(pcSnapHead_ - 1u - back) &
                            (pcSnapGr127_.size() - 1u)];
    }
    std::size_t diagnosticTimerWriteRingCount() const {
        return timerWriteRingCount_;
    }
    u32 diagnosticTimerWritePc(std::size_t back) const {
        if (back >= timerWriteRingCount_) return 0;
        return timerWritePc_[(timerWriteHead_ - 1u - back) &
                             (timerWritePc_.size() - 1u)];
    }
    u32 diagnosticTimerWriteValue(std::size_t back) const {
        if (back >= timerWriteRingCount_) return 0;
        return timerWriteValue_[(timerWriteHead_ - 1u - back) &
                                (timerWriteValue_.size() - 1u)];
    }
    bool diagnosticTimerWriteIsReload(std::size_t back) const {
        if (back >= timerWriteRingCount_) return false;
        return timerWriteIsReload_[(timerWriteHead_ - 1u - back) &
                                   (timerWriteIsReload_.size() - 1u)] != 0;
    }
    u64 diagnosticTimerWriteInstruction(std::size_t back) const {
        if (back >= timerWriteRingCount_) return 0;
        return timerWriteInstruction_[(timerWriteHead_ - 1u - back) &
                                      (timerWriteInstruction_.size() - 1u)];
    }
    std::size_t diagnosticCallTraceCount() const { return callTraceCount_; }
    u32 diagnosticCallTracePc(std::size_t back) const {
        if (back >= callTraceCount_) return 0;
        return callTracePc_[(callTraceHead_ - 1u - back) &
                            (callTracePc_.size() - 1u)];
    }
    u32 diagnosticCallTraceTarget(std::size_t back) const {
        if (back >= callTraceCount_) return 0;
        return callTraceTarget_[(callTraceHead_ - 1u - back) &
                                (callTraceTarget_.size() - 1u)];
    }
    u64 diagnosticCallTraceInstruction(std::size_t back) const {
        if (back >= callTraceCount_) return 0;
        return callTraceInstruction_[(callTraceHead_ - 1u - back) &
                                     (callTraceInstruction_.size() - 1u)];
    }
    std::size_t diagnosticRegister64ChangeCount() const {
        return register64ChangeCount_;
    }
    u32 diagnosticRegister64ChangePc(std::size_t back) const {
        if (back >= register64ChangeCount_) return 0;
        return register64ChangePc_[(register64ChangeHead_ - 1u - back) &
                                   (register64ChangePc_.size() - 1u)];
    }
    u32 diagnosticRegister64ChangeInstruction(std::size_t back) const {
        if (back >= register64ChangeCount_) return 0;
        return register64ChangeIr_[(register64ChangeHead_ - 1u - back) &
                                   (register64ChangeIr_.size() - 1u)];
    }
    u32 diagnosticRegister64ChangeOldValue(std::size_t back) const {
        if (back >= register64ChangeCount_) return 0;
        return register64ChangeOld_[(register64ChangeHead_ - 1u - back) &
                                    (register64ChangeOld_.size() - 1u)];
    }
    u32 diagnosticRegister64ChangeNewValue(std::size_t back) const {
        if (back >= register64ChangeCount_) return 0;
        return register64ChangeNew_[(register64ChangeHead_ - 1u - back) &
                                    (register64ChangeNew_.size() - 1u)];
    }
    u32 processorStatus() const { return m_cps; }
    u32 configuration() const { return m_cfg; }
    u32 vectorAreaBase() const { return m_vab; }
    u32 mmuConfiguration() const { return m_mmu; }
    u32 timerCounter() const { return m_tmc & 0x00FFFFFFu; }
    u32 timerReload() const { return m_tmr; }
    u32 channelAddress() const { return m_cha; }
    u32 channelData() const { return m_chd; }
    u32 channelControl() const { return m_chc; }
    u32 tlbRegister(std::size_t index) const {
        return index < m_tlb.size() ? m_tlb[index] : 0;
    }
    u32 registerValue(std::size_t index) const {
        if (index >= m_r.size()) return 0;
        if (index < 128u) return m_r[index];
        // Local-register names are relative to the current window pointer in
        // gr1. Diagnostics must apply the same rotation as instruction
        // operands; exposing the backing-array index makes lr0..lr127 appear
        // to change identity at every call.
        u8 physical = static_cast<u8>(
            ((m_r[1] >> 2u) & 0x7Fu) + (index & 0x7Fu));
        physical = static_cast<u8>(physical | 0x80u);
        return m_r[physical];
    }
    u32 registerWindowBase() const { return m_r[1]; }
    u64 instructions() const { return instructions_; }
    u64 diagnosticTimerExpiries() const { return timerExpiries_; }
    u64 diagnosticTimerInterrupts() const { return timerInterrupts_; }
    u64 diagnosticTimerWrites() const { return timerWrites_; }
    u64 diagnosticExternalInterrupts(int input) const {
        return externalInterrupts_[input & 3];
    }
    u64 diagnosticExternalAssertions(int input) const {
        return externalAssertions_[input & 3];
    }
    void setDiagnosticPcWatch(u32 pc) {
        diagnosticWatchPc_ = pc;
        diagnosticWatchHits_ = 0;
        diagnosticWatchHitCount_ = 0;
        refreshProbeFlag();
    }
    // Instruction numbers of the first 32 watched-pc hits.
    std::size_t diagnosticPcWatchHitCount() const {
        return diagnosticWatchHitCount_;
    }
    u64 diagnosticPcWatchHitInstruction(std::size_t index) const {
        return index < diagnosticWatchHitCount_
            ? diagnosticWatchHitInstructions_[index] : 0;
    }
    void setDiagnosticProfileRange(u32 first, u32 last) {
        diagnosticProfileFirst_ = first;
        diagnosticProfileCounts_.assign(
            (static_cast<std::size_t>(last - first) >> 2u) + 1u, 0);
        refreshProbeFlag();
    }
    u32 diagnosticProfileFirst() const { return diagnosticProfileFirst_; }
    const std::vector<u64>& diagnosticProfileCounts() const {
        return diagnosticProfileCounts_;
    }
    u64 diagnosticPcWatchHits() const { return diagnosticWatchHits_; }
    bool faulted() const { return faulted_; }
    const std::string& faultReason() const { return faultReason_; }

private:
    friend class IifxStateCodec;

    enum : u32 {
        SprVab = 0, SprOps = 1, SprCps = 2, SprCfg = 3,
        SprCha = 4, SprChd = 5, SprChc = 6, SprRbp = 7,
        SprTmc = 8, SprTmr = 9, SprPc0 = 10, SprPc1 = 11,
        SprPc2 = 12, SprMmu = 13, SprLru = 14,
        SprIpc = 128, SprIpa = 129, SprIpb = 130, SprQ = 131,
        SprAlu = 132, SprBp = 133, SprFc = 134, SprCr = 135,
        SprFpe = 160, SprInte = 161, SprFps = 162,
    };

    enum : u32 {
        ExceptionIllegalOpcode = 0,
        ExceptionUnalignedAccess = 1,
        ExceptionOutOfRange = 2,
        ExceptionProtectionViolation = 5,
        ExceptionInstructionAccessException = 6,
        ExceptionDataAccessException = 7,
        ExceptionUserInstructionTlbMiss = 8,
        ExceptionUserDataTlbMiss = 9,
        ExceptionSupervisorInstructionTlbMiss = 10,
        ExceptionSupervisorDataTlbMiss = 11,
        ExceptionInstructionTlbProtection = 12,
        ExceptionDataTlbProtection = 13,
        ExceptionTimer = 14,
        ExceptionIntr0 = 16,
        ExceptionDivide = 33,
    };

    struct DataSpace {
        Am29000* owner = nullptr;
        u32 read_dword(u32 address, bool inputOutput) const;
        void write_dword(u32 address, u32 value, bool inputOutput) const;
    };

    void fail(const char* reason);
    void signal_exception(u32 type);
    void external_irq_check();
    void timer_cycle();
    void timer_interrupt_check();
    enum class Access { Instruction, Load, Store };
    bool translateAddress(u32 virtualAddress, Access access,
                          bool userAccess, u32& physicalAddress,
                          int* hitSet = nullptr);
    bool trapsUnalignedDataAccess(u32 address, u32 option,
                                  bool instructionData) const;
    u32 read_program_word(u32 address);
    u32 read_data_value(u32 address, u32 option, bool setBytePointer,
                        bool inputOutput);
    void write_data_value(u32 address, u32 value, u32 option,
                          bool inputOutput);
    u32 get_abs_reg(u8 reg, u32 indirectPointer);
    void fetch_decode();
    u32 read_spr(u32 index);
    void write_spr(u32 index, u32 value);

    void ADD(); void ADDS(); void ADDU(); void ADDC();
    void ADDCS(); void ADDCU(); void SUB(); void SUBS();
    void SUBU(); void SUBC(); void SUBCS(); void SUBCU();
    void SUBR(); void SUBRS(); void SUBRU(); void SUBRC();
    void SUBRCS(); void SUBRCU(); void MULTIPLU(); void MULTIPLY();
    void MUL(); void MULL(); void MULU(); void DIVIDE();
    void DIVIDU(); void DIV0(); void DIV(); void DIVL();
    void DIVREM(); void CPEQ(); void CPNEQ(); void CPLT();
    void CPLTU(); void CPLE(); void CPLEU(); void CPGT();
    void CPGTU(); void CPGE(); void CPGEU(); void CPBYTE();
    void ASEQ(); void ASNEQ(); void ASLT(); void ASLTU();
    void ASLE(); void ASLEU(); void ASGT(); void ASGTU();
    void ASGE(); void ASGEU(); void AND(); void ANDN();
    void NAND(); void OR(); void NOR(); void XOR(); void XNOR();
    void SLL(); void SRL(); void SRA(); void EXTRACT();
    void LOAD(); void LOADL(); void LOADSET(); void LOADM();
    void STORE(); void STOREL(); void STOREM(); void EXBYTE();
    void EXHW(); void EXHWS(); void INBYTE(); void INHW();
    void MFSR(); void MFTLB(); void MTSR(); void MTSRIM();
    void MTTLB(); void CONST(); void CONSTH(); void CONSTN();
    void CALL(); void CALLI(); void JMP(); void JMPI();
    void JMPT(); void JMPTI(); void JMPF(); void JMPFI();
    void JMPFDEC(); void CLZ(); void SETIP(); void EMULATE();
    void INV(); void IRET(); void IRETINV(); void HALT();
    void ILLEGAL(); void CONVERT(); void SQRT(); void CLASS();
    void MULTM(); void MULTMU();

    using OpcodeFunction = void (Am29000::*)();
    struct OpcodeInfo {
        OpcodeFunction opcode;
        u32 flags;
    };
    static const std::array<OpcodeInfo, 256> opTable_;

    DataSpace m_data{this};
    std::array<u32, 256> m_r{};
    std::array<FetchWindow, kFetchWindows> fetchWindows_{};
    std::array<DataWindow, kDataWindows> dataWindows_{};
    std::size_t lastDataWindow_ = 0;
    // The window holding physical word `address` (aligned), or nullptr.
    // The last window hit is tried first: GCOS's loops stay in one memory.
    DataWindow* findDataWindow(u32 address, bool write) {
        {
            DataWindow& window = dataWindows_[lastDataWindow_];
            const u32 offset = address - window.base;
            if (window.data && offset < window.size &&
                (!write || window.writable))
                return &window;
        }
        for (std::size_t index = 0; index < dataWindows_.size(); ++index) {
            DataWindow& window = dataWindows_[index];
            const u32 offset = address - window.base;
            if (window.data && offset < window.size &&
                (!write || window.writable)) {
                lastDataWindow_ = index;
                return &window;
            }
        }
        return nullptr;
    }
    static void bumpDataCounters(const std::array<DataCounter, 3>& counters) {
        for (const DataCounter& counter : counters) {
            if (!counter.value) break;
            *counter.value += counter.add;
        }
    }
    // The window the last fetch hit is tried first: code runs for long
    // stretches inside one memory (GCOS in the D expansion DRAM, the kernel
    // in SRAM), and it is the last-registered window that GCOS lives in.
    std::size_t lastFetchWindow_ = 0;
    std::array<u32, 128> m_tlb{};
    // Instruction-fetch translation hint (see read_program_word).  Not
    // architectural state: derived from m_tlb, dropped on any TLB/MMU write.
    bool fetchTranslationValid_ = false;
    bool fetchTranslationUser_ = false;
    u32 fetchTranslationVpage_ = 0;
    u32 fetchTranslationPhys_ = 0;
    u32 fetchTranslationLine_ = 0;
    int fetchTranslationSet_ = 0;
    // Precomputed for the hint's hit path: the two TLB words whose Usage
    // bit the hit rewrites (same value each time), the mask of in-page
    // address bits, and -- when the whole physical page sits inside one
    // fetch window -- the host bytes of that page, so a hit is two stores
    // and a load with no window scan.  Derived; dropped with the hint or
    // the windows.
    u32 fetchTranslationTlbIndex0_ = 0;
    u32 fetchTranslationTlbIndex1_ = 0;
    u32 fetchTranslationUsage_ = 0;
    u32 fetchTranslationPageMask_ = 0;
    const u8* fetchTranslationPage_ = nullptr;
    void resolveFetchTranslationPage() {
        fetchTranslationPage_ = nullptr;
        for (const FetchWindow& window : fetchWindows_) {
            const u32 offset = fetchTranslationPhys_ - window.base;
            if (window.data && offset < window.size &&
                window.size - offset > fetchTranslationPageMask_) {
                fetchTranslationPage_ = window.data + offset;
                return;
            }
        }
    }

    u32 m_pc = 0;
    u32 m_vab = 0;
    u32 m_ops = 0;
    u32 m_cps = 0;
    u32 m_cfg = 0;
    u32 m_cha = 0;
    u32 m_chd = 0;
    u32 m_chc = 0;
    u32 m_rbp = 0;
    u32 m_tmc = 0;
    u32 m_tmr = 0;
    u32 m_pc0 = 0;
    u32 m_pc1 = 0;
    u32 m_pc2 = 0;
    u32 m_mmu = 0;
    u32 m_lru = 0;
    u32 m_ipc = 0;
    u32 m_ipa = 0;
    u32 m_ipb = 0;
    u32 m_q = 0;
    u32 m_alu = 0;
    u32 m_fpe = 0;
    u32 m_inte = 0;
    u32 m_fps = 0;

    u32 m_exceptions = 0;
    std::array<u32, 4> m_exception_queue{};
    u8 m_irq_active = 0;
    u8 m_irq_lines = 0;
    u32 m_exec_ir = 0;
    u32 m_next_ir = 0;
    u32 m_pl_flags = 0;
    u32 m_next_pl_flags = 0;
    u32 m_iret_pc = 0;
    u32 m_exec_pc = 0;
    u32 m_next_pc = 0;

    u64 instructions_ = 0;
    // Trace-only watch state. This is deliberately excluded from save states
    // so replay checkpoints remain independent of the front end's trigger.
    u32 diagnosticWatchPc_ = 0xFFFFFFFFu;
    // True while any optional per-instruction probe (pc watch, profile,
    // flight recorder, pc snapshot, register watch, gr64 change ring) is
    // armed; the always-on flight rings (recent instructions, call trace)
    // do not depend on it. Recomputed by the probes' setters.
    bool probesArmed_ = false;
    void refreshProbeFlag() {
        probesArmed_ = diagnosticWatchPc_ != 0xFFFFFFFFu ||
                       !diagnosticProfileCounts_.empty() || flightCount_ != 0 ||
                       pcSnapPc_ != 0 || registerWatchIndex_ < 256u;
    }
    u64 diagnosticWatchHits_ = 0;
    std::array<u64, 32> diagnosticWatchHitInstructions_{};
    std::size_t diagnosticWatchHitCount_ = 0;
    // Diagnostic-only interrupt-facility counters; not serialized.
    u64 timerExpiries_ = 0;
    u64 timerInterrupts_ = 0;
    u64 timerWrites_ = 0;
    std::array<u64, 4> externalInterrupts_{};
    std::array<u64, 4> externalAssertions_{};
    // Diagnostic-only execution profile over one PC range; not serialized.
    u32 diagnosticProfileFirst_ = 0;
    std::vector<u64> diagnosticProfileCounts_;
    // Keep enough history to see through an exception prologue and a GCOS
    // completion/error helper.  These rings are diagnostic-only and are not
    // part of the checkpoint format.
    std::array<u32, 512> recentPc_{};
    std::array<u32, 512> recentIr_{};
    std::size_t recentHead_ = 0;
    std::size_t recentCount_ = 0;
    std::array<u32, 64> register64ChangePc_{};
    std::array<u32, 64> register64ChangeIr_{};
    std::array<u32, 64> register64ChangeOld_{};
    std::array<u32, 64> register64ChangeNew_{};
    std::size_t register64ChangeHead_ = 0;
    std::size_t register64ChangeCount_ = 0;
    // Rolling ring of taken CALL/CALLI/JMPI/JMPTI/JMPFI transfers so a wedge
    // can be traced to the call chain that led there.  Diagnostic-only.
    std::array<u32, 4096> callTracePc_{};
    std::array<u32, 4096> callTraceTarget_{};
    std::array<u64, 4096> callTraceInstruction_{};
    std::size_t callTraceHead_ = 0;
    std::size_t callTraceCount_ = 0;
    // Rolling ring of TMC/TMR special-register writes: which code re-arms the
    // timer, and with what values.  Diagnostic-only.
    std::array<u32, 64> timerWritePc_{};
    std::array<u32, 64> timerWriteValue_{};
    std::array<u64, 64> timerWriteInstruction_{};
    std::array<u8, 64> timerWriteIsReload_{};
    std::size_t timerWriteHead_ = 0;
    std::size_t timerWriteRingCount_ = 0;
    void recordTimerWrite(bool reloadRegister, u32 value) {
        timerWritePc_[timerWriteHead_] = m_exec_pc;
        timerWriteValue_[timerWriteHead_] = value;
        timerWriteInstruction_[timerWriteHead_] = instructions_;
        timerWriteIsReload_[timerWriteHead_] = reloadRegister ? 1u : 0u;
        timerWriteHead_ = (timerWriteHead_ + 1u) & (timerWritePc_.size() - 1u);
        timerWriteRingCount_ = std::min(timerWriteRingCount_ + 1u,
                                        timerWritePc_.size());
    }
    bool faulted_ = false;
    std::string faultReason_;
    // Derived from the MFB +$088 latch by the video model; not serialized.
    bool interruptHold_ = false;
    // Physical-register write watch (diagnostic-only, not serialized).
    u16 registerWatchIndex_ = 0xFFFFu;
    std::array<u32, 64> registerWatchPc_{};
    std::array<u32, 64> registerWatchValue_{};
    std::array<u64, 64> registerWatchInstruction_{};
    std::size_t registerWatchHead_ = 0;
    std::size_t registerWatchCount_ = 0;
    bool dataBusFault_ = false;
    bool takeDataBusFault() {
        const bool faulted = dataBusFault_;
        dataBusFault_ = false;
        return faulted;
    }
    // A *DERR'd access RETIRES its instruction (the access lives on only
    // in the channel registers); the dispatcher advances the resume pcs
    // past it when this is set.
    void restartChannelAccess();
    u64 flightStart_ = 0;
    u64 flightCount_ = 0;
    u16 flightWatchIndex_ = 0;
    std::vector<FlightEntry> flight_;
    u32 pcSnapPc_ = 0;
    u16 pcSnapRegister_ = 0;
    std::array<u64, 64> pcSnapInstruction_{};
    std::array<u32, 64> pcSnapGr1_{};
    std::array<u32, 64> pcSnapValue_{};
    std::array<u32, 64> pcSnapGr126_{};
    std::array<u32, 64> pcSnapGr127_{};
    std::size_t pcSnapHead_ = 0;
    std::size_t pcSnapCount_ = 0;
};

} // namespace openmac
