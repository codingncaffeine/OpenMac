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

namespace openmac {

class IifxStateCodec;

class Am29000 {
public:
    // The MFB needs to distinguish reset/physical fetches from translated
    // instruction fetches; the latter use its external MMU and SRAM cache.
    std::function<u32(u32, bool)> readInstruction;
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
    void setDiagnosticPcWatch(u32 pc) {
        diagnosticWatchPc_ = pc;
        diagnosticWatchHits_ = 0;
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
                          bool userAccess, u32& physicalAddress);
    bool trapsUnalignedDataAccess(u32 address, u32 option,
                                  bool instructionData) const;
    u32 read_program_word(u32 address);
    u32 read_data_value(u32 address, u32 option, bool signExtend,
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
    std::array<u32, 128> m_tlb{};

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
    u64 diagnosticWatchHits_ = 0;
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
    bool faulted_ = false;
    std::string faultReason_;
};

} // namespace openmac
