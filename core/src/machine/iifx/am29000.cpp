// license:BSD-3-Clause
// copyright-holders:Philip Bennett, OpenMac contributors

#include "am29000.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <limits>

namespace openmac {

namespace {

constexpr u32 PFLAG_EXECUTE_EN = 1u << 2;
constexpr u32 PFLAG_IRQ = 1u << 4;
constexpr u32 PFLAG_JUMP = 1u << 7;
constexpr u32 PFLAG_IRET = 1u << 9;

constexpr u32 CPS_CA = 1u << 15;
constexpr u32 CPS_IP = 1u << 14;
constexpr u32 CPS_TE = 1u << 13;
constexpr u32 CPS_TP = 1u << 12;
constexpr u32 CPS_TU = 1u << 11;
constexpr u32 CPS_FZ = 1u << 10;
constexpr u32 CPS_LK = 1u << 9;
constexpr u32 CPS_RE = 1u << 8;
constexpr u32 CPS_WM = 1u << 7;
constexpr u32 CPS_PD = 1u << 6;
constexpr u32 CPS_PI = 1u << 5;
constexpr u32 CPS_SM = 1u << 4;
constexpr u32 CPS_IM_SHIFT = 2;
constexpr u32 CPS_IM_MASK = 3;
constexpr u32 CPS_DI = 1u << 1;
constexpr u32 CPS_DA = 1u << 0;

constexpr u32 PROCESSOR_REL_FIELD = 3;
constexpr u32 VAB_MASK = 0xFFFF;
constexpr u32 VAB_SHIFT = 16;
constexpr u32 CFG_PRL_SHIFT = 24;
constexpr u32 CFG_DW = 1u << 5;
constexpr u32 CFG_VF = 1u << 4;
constexpr u32 CFG_RV = 1u << 3;
constexpr u32 CFG_BO = 1u << 2;
constexpr u32 CFG_CP = 1u << 1;
constexpr u32 CFG_CD = 1u << 0;

constexpr u32 CHC_CE_CNTL_MASK = 0xFF;
constexpr u32 CHC_CE_CNTL_SHIFT = 24;
constexpr u32 CHC_CR_MASK = 0xFF;
constexpr u32 CHC_CR_SHIFT = 16;
constexpr u32 CHC_LS = 1u << 15;
constexpr u32 CHC_ML = 1u << 14;
constexpr u32 CHC_ST = 1u << 13;
constexpr u32 CHC_LA = 1u << 12;
constexpr u32 CHC_TF = 1u << 11;
constexpr u32 CHC_TR_MASK = 0xFF;
constexpr u32 CHC_TR_SHIFT = 2;
constexpr u32 CHC_NN = 1u << 1;
constexpr u32 CHC_CV = 1u << 0;

constexpr u32 RBP_MASK = 0xFFFF;
constexpr u32 TCV_MASK = 0x00FFFFFF;
// TCV is architecturally zero for the cycle between reaching zero and being
// reloaded.  Keep that one-cycle state in a reserved bit so existing IIfx
// checkpoints continue to serialize the complete timer without a format bump.
constexpr u32 TCV_RELOAD_PENDING = 1u << 31;
constexpr u32 TMR_OV = 1u << 26;
constexpr u32 TMR_IN = 1u << 25;
constexpr u32 TMR_IE = 1u << 24;
constexpr u32 TMR_TRV_MASK = 0x00FFFFFF;
constexpr u32 PC_MASK = 0xFFFFFFFC;
constexpr u32 MMU_PS_MASK = 3;
constexpr u32 MMU_PS_SHIFT = 8;
constexpr u32 MMU_PID_MASK = 0xFF;
constexpr u32 LRU_MASK = 0x3F;
constexpr u32 LRU_SHIFT = 1;

constexpr u32 ALU_DF = 1u << 11;
constexpr u32 ALU_DF_SHIFT = 11;
constexpr u32 ALU_V_SHIFT = 10;
constexpr u32 ALU_V = 1u << 10;
constexpr u32 ALU_N_SHIFT = 9;
constexpr u32 ALU_N = 1u << 9;
constexpr u32 ALU_Z_SHIFT = 8;
constexpr u32 ALU_Z = 1u << 8;
constexpr u32 ALU_C_SHIFT = 7;
constexpr u32 ALU_C = 1u << 7;
constexpr u32 ALU_BP_MASK = 3;
constexpr u32 ALU_BP_SHIFT = 5;
constexpr u32 ALU_FC_MASK = 0x1F;
constexpr u32 ALU_FC_SHIFT = 0;
constexpr u32 IPX_MASK = 0xFF;
constexpr u32 IPX_SHIFT = 2;

struct ArithmeticResult {
    u32 value = 0;
    bool carry = false;
    bool signedOverflow = false;
    bool unsignedOverflow = false;
};

ArithmeticResult addArithmetic(u32 a, u32 b, u32 carry) {
    const u64 wide = u64(a) + u64(b) + carry;
    const s64 signedWide = s64(static_cast<s32>(a)) +
                           s64(static_cast<s32>(b)) + carry;
    return {
        static_cast<u32>(wide),
        (wide >> 32u) != 0,
        signedWide < std::numeric_limits<s32>::min() ||
            signedWide > std::numeric_limits<s32>::max(),
        (wide >> 32u) != 0,
    };
}

ArithmeticResult subtractArithmetic(u32 a, u32 b, u32 carry) {
    const u32 borrow = carry == 0 ? 1u : 0u;
    const u64 subtrahend = u64(b) + borrow;
    const bool underflow = u64(a) < subtrahend;
    const s64 signedWide = s64(static_cast<s32>(a)) -
                           s64(static_cast<s32>(b)) - borrow;
    return {
        static_cast<u32>(u64(a) - subtrahend),
        !underflow,
        signedWide < std::numeric_limits<s32>::min() ||
            signedWide > std::numeric_limits<s32>::max(),
        underflow,
    };
}

void setArithmeticFlags(u32& alu, const ArithmeticResult& result) {
    alu &= ~(ALU_V | ALU_N | ALU_Z | ALU_C);
    if (result.signedOverflow) alu |= ALU_V;
    if ((result.value & 0x80000000u) != 0) alu |= ALU_N;
    if (result.value == 0) alu |= ALU_Z;
    if (result.carry) alu |= ALU_C;
}

} // namespace

#define FREEZE_MODE              (m_cps & CPS_FZ)
#define SUPERVISOR_MODE          (m_cps & CPS_SM)
#define USER_MODE                (~m_cps & CPS_SM)
#define GET_ALU_FC               ((m_alu >> ALU_FC_SHIFT) & ALU_FC_MASK)
#define GET_ALU_BP               ((m_alu >> ALU_BP_SHIFT) & ALU_BP_MASK)
#define GET_CHC_CR               ((m_chc >> CHC_CR_SHIFT) & CHC_CR_MASK)
#define SET_ALU_FC(x)            do { m_alu &= ~(ALU_FC_MASK << ALU_FC_SHIFT); m_alu |= ((x) & ALU_FC_MASK) << ALU_FC_SHIFT; } while (false)
#define SET_ALU_BP(x)            do { m_alu &= ~(ALU_BP_MASK << ALU_BP_SHIFT); m_alu |= ((x) & ALU_BP_MASK) << ALU_BP_SHIFT; } while (false)
#define SET_CHC_CR(x)            do { m_chc &= ~(CHC_CR_MASK << CHC_CR_SHIFT); m_chc |= ((x) & CHC_CR_MASK) << CHC_CR_SHIFT; } while (false)
#define SIGNAL_EXCEPTION(x)      signal_exception(x)
#define fatalerror(...)          do { fail(__FUNCTION__); return; } while (false)
#define logerror(...)            static_cast<void>(0)

Am29000::Am29000() {
    reset();
}

u32 Am29000::DataSpace::read_dword(u32 address, bool inputOutput) const {
    return owner && owner->readData
        ? owner->readData(address, inputOutput) : 0;
}

void Am29000::DataSpace::write_dword(u32 address, u32 value,
                                     bool inputOutput) const {
    if (owner && owner->writeData)
        owner->writeData(address, value, inputOutput);
}

void Am29000::fail(const char* reason) {
    faulted_ = true;
    faultReason_ = reason ? reason : "unknown Am29000 fault";
}

void Am29000::reset(u32 resetPc) {
    m_r.fill(0);
    m_tlb.fill(0);
    m_exception_queue.fill(0);
    m_pc = resetPc;
    m_vab = m_ops = m_cha = m_chd = m_chc = 0;
    m_rbp = m_tmc = m_tmr = m_pc0 = m_pc1 = m_pc2 = 0;
    m_mmu = m_lru = m_ipc = m_ipa = m_ipb = m_q = m_alu = 0;
    m_fpe = m_inte = m_fps = 0;
    m_cfg = PROCESSOR_REL_FIELD << CFG_PRL_SHIFT;
    m_cps = CPS_FZ | CPS_RE | CPS_PD | CPS_PI | CPS_SM | CPS_DI | CPS_DA;
    m_exceptions = 0;
    m_irq_active = m_irq_lines = 0;
    m_exec_ir = m_next_ir = 0;
    m_pl_flags = m_next_pl_flags = 0;
    m_iret_pc = m_exec_pc = 0;
    m_next_pc = resetPc;
    instructions_ = 0;
    diagnosticWatchHits_ = 0;
    recentPc_.fill(0);
    recentIr_.fill(0);
    recentHead_ = recentCount_ = 0;
    register64ChangePc_.fill(0);
    register64ChangeIr_.fill(0);
    register64ChangeOld_.fill(0);
    register64ChangeNew_.fill(0);
    register64ChangeHead_ = register64ChangeCount_ = 0;
    faulted_ = false;
    faultReason_.clear();
}

void Am29000::signal_exception(u32 type) {
    if (m_exceptions < m_exception_queue.size()) {
        m_exception_queue[m_exceptions++] = type;
    } else {
        fail("exception queue overflow");
    }
}

void Am29000::external_irq_check() {
    const int mask = static_cast<int>((m_cps >> CPS_IM_SHIFT) & CPS_IM_MASK);
    const bool enabled = (m_cps & (CPS_DI | CPS_DA)) == 0;
    m_cps &= ~CPS_IP;
    for (int input = 0; input < 4; ++input) {
        const u8 bit = static_cast<u8>(1u << input);
        if ((m_irq_active & bit) == 0 && (m_irq_lines & bit) != 0) {
            if (enabled && input <= mask) {
                m_irq_active = static_cast<u8>(m_irq_active | bit);
                signal_exception(ExceptionIntr0 + static_cast<u32>(input));
                m_pl_flags |= PFLAG_IRQ;
                return;
            }
            m_cps |= CPS_IP;
        } else {
            m_irq_active = static_cast<u8>(m_irq_active & ~bit);
        }
    }
}

void Am29000::timer_cycle() {
    if ((m_tmc & TCV_RELOAD_PENDING) != 0) {
        m_tmc = m_tmr & TMR_TRV_MASK;
        if ((m_tmr & TMR_IN) != 0) m_tmr |= TMR_OV;
        m_tmr |= TMR_IN;
        return;
    }

    // TCV is a wrapping 24-bit counter.  In particular, a software-loaded
    // zero represents a full 2^24-cycle interval rather than an immediate
    // reload.  Reloading is scheduled only when a decrement produces zero.
    const u32 count = ((m_tmc & TCV_MASK) - 1u) & TCV_MASK;
    m_tmc = count | (count == 0 ? TCV_RELOAD_PENDING : 0u);
}

void Am29000::timer_interrupt_check() {
    // Timer interrupts are independent of CPS.DI and the external interrupt
    // mask.  CPS.DA is the only processor-status mask that applies.
    if ((m_tmr & (TMR_IN | TMR_IE)) == (TMR_IN | TMR_IE) &&
        (m_cps & CPS_DA) == 0)
        signal_exception(ExceptionTimer);
}

u32 Am29000::read_program_word(u32 address) {
    const bool translated = (m_cps & (CPS_PI | CPS_RE)) == 0;
    u32 physicalAddress = address;
    if (translated &&
        !translateAddress(address, Access::Instruction, USER_MODE != 0,
                          physicalAddress))
        return 0;
    return readInstruction ? readInstruction(physicalAddress, translated) : 0;
}

bool Am29000::translateAddress(u32 virtualAddress, Access access,
                               bool userAccess, u32& physicalAddress) {
    const u32 pageShift = 10u + ((m_mmu >> MMU_PS_SHIFT) & MMU_PS_MASK);
    const u32 line = (virtualAddress >> pageShift) & 31u;
    const u32 pid = m_mmu & MMU_PID_MASK;
    const u32 tagMask = 0xFFFFFFFFu << (pageShift + 5u);
    int matchingSet = -1;

    for (int set = 0; set < 2; ++set) {
        const std::size_t word0Index =
            static_cast<std::size_t>(set * 64 + line * 2u);
        const u32 word0 = m_tlb[word0Index];
        if ((word0 & (1u << 14)) == 0 ||
            (word0 & 0xFFu) != pid ||
            (word0 & tagMask) != (virtualAddress & tagMask))
            continue;
        if (matchingSet >= 0) {
            // Multiple matches are architecturally unpredictable. Choosing
            // the lower-numbered set makes the result deterministic.
            break;
        }
        matchingSet = set;
    }

    const auto updateLru = [&](int lruSet) {
        m_lru = static_cast<u32>(lruSet * 64) + line * 2u;
    };
    if (matchingSet < 0) {
        const std::size_t set0Word1 = line * 2u + 1u;
        const bool set1IsLru = (m_tlb[set0Word1] & 2u) != 0;
        updateLru(set1IsLru ? 1 : 0);
        if (access == Access::Instruction)
            signal_exception(userAccess ? ExceptionUserInstructionTlbMiss
                                        : ExceptionSupervisorInstructionTlbMiss);
        else
            signal_exception(userAccess ? ExceptionUserDataTlbMiss
                                        : ExceptionSupervisorDataTlbMiss);
        return false;
    }

    const std::size_t word0Index =
        static_cast<std::size_t>(matchingSet * 64 + line * 2u);
    const u32 word0 = m_tlb[word0Index];
    const u32 permission = access == Access::Instruction
        ? (userAccess ? 8u : 11u)
        : access == Access::Store ? (userAccess ? 9u : 12u)
                                  : (userAccess ? 10u : 13u);
    if ((word0 & (1u << permission)) == 0) {
        updateLru(matchingSet);
        signal_exception(access == Access::Instruction
            ? ExceptionInstructionTlbProtection
            : ExceptionDataTlbProtection);
        return false;
    }

    // Both words in a line carry the same Usage bit.  It names the set that
    // is least recently used, so using set 0 makes set 1 the replacement and
    // vice versa.
    const u32 usage = matchingSet == 0 ? 2u : 0u;
    for (int set = 0; set < 2; ++set) {
        const std::size_t word1Index =
            static_cast<std::size_t>(set * 64 + line * 2u + 1u);
        m_tlb[word1Index] = (m_tlb[word1Index] & ~2u) | usage;
    }

    const u32 pageMask = (1u << pageShift) - 1u;
    const u32 word1 = m_tlb[word0Index + 1u];
    physicalAddress = (word1 & ~pageMask) | (virtualAddress & pageMask);
    return true;
}

bool Am29000::trapsUnalignedDataAccess(u32 address, u32 option,
                                       bool instructionData) const {
    if (!instructionData || (m_cps & CPS_TU) == 0) return false;
    switch (option & 7u) {
    case 0: return (address & 3u) != 0; // word
    case 2: return (address & 1u) != 0; // half-word
    default: return false;
    }
}

u32 Am29000::read_data_value(u32 address, u32 option, bool signExtend,
                             bool inputOutput) {
    const auto readWord = [this, inputOutput](u32 byteAddress) {
        return m_data.read_dword(byteAddress & ~3u, inputOutput);
    };
    const auto readByte = [&](u32 byteAddress) {
        const u32 lane = byteAddress & 3u;
        const u32 shift = (m_cfg & CFG_BO) != 0
            ? lane * 8u : (3u - lane) * 8u;
        return static_cast<u8>(readWord(byteAddress) >> shift);
    };

    switch (option & 7u) {
    case 1: { // externally aligned byte access
        const u8 value = readByte(address);
        return signExtend
            ? static_cast<u32>(static_cast<s32>(static_cast<s8>(value)))
            : value;
    }
    case 2: { // externally aligned half-word access
        const u32 first = address & ~1u;
        const u16 value = (m_cfg & CFG_BO) != 0
            ? static_cast<u16>(u16(readByte(first + 1u)) << 8u |
                               readByte(first))
            : static_cast<u16>(u16(readByte(first)) << 8u |
                               readByte(first + 1u));
        return signExtend
            ? static_cast<u32>(static_cast<s32>(static_cast<s16>(value)))
            : value;
    }
    case 3: { // externally aligned 24-bit access
        const u32 value = (m_cfg & CFG_BO) != 0
            ? u32(readByte(address + 2u)) << 16u |
              u32(readByte(address + 1u)) << 8u | readByte(address)
            : u32(readByte(address)) << 16u |
              u32(readByte(address + 1u)) << 8u | readByte(address + 2u);
        return signExtend && (value & 0x00800000u) != 0
            ? value | 0xFF000000u : value;
    }
    default:
        // The external memory system forces ordinary word accesses to a
        // word boundary when CPS.TU does not request an alignment trap.
        return readWord(address);
    }
}

void Am29000::write_data_value(u32 address, u32 value, u32 option,
                               bool inputOutput) {
    const auto writeByte = [this, inputOutput](u32 byteAddress, u8 byte) {
        const u32 aligned = byteAddress & ~3u;
        const u32 lane = byteAddress & 3u;
        const u32 shift = (m_cfg & CFG_BO) != 0
            ? lane * 8u : (3u - lane) * 8u;
        const u32 oldWord = peekData
            ? peekData(aligned, inputOutput)
            : m_data.read_dword(aligned, inputOutput);
        const u32 newWord = (oldWord & ~(0xFFu << shift)) |
                            (u32(byte) << shift);
        m_data.write_dword(aligned, newWord, inputOutput);
    };

    switch (option & 7u) {
    case 1:
        writeByte(address, static_cast<u8>(value));
        return;
    case 2: {
        const u32 first = address & ~1u;
        if ((m_cfg & CFG_BO) != 0) {
            writeByte(first, static_cast<u8>(value));
            writeByte(first + 1u, static_cast<u8>(value >> 8u));
        } else {
            writeByte(first, static_cast<u8>(value >> 8u));
            writeByte(first + 1u, static_cast<u8>(value));
        }
        return;
    }
    case 3:
        if ((m_cfg & CFG_BO) != 0) {
            writeByte(address, static_cast<u8>(value));
            writeByte(address + 1u, static_cast<u8>(value >> 8u));
            writeByte(address + 2u, static_cast<u8>(value >> 16u));
        } else {
            writeByte(address, static_cast<u8>(value >> 16u));
            writeByte(address + 1u, static_cast<u8>(value >> 8u));
            writeByte(address + 2u, static_cast<u8>(value));
        }
        return;
    default:
        m_data.write_dword(address & ~3u, value, inputOutput);
        return;
    }
}

u32 Am29000::get_abs_reg(u8 reg, u32 indirectPointer) {
    if ((reg & 0x80u) != 0) {
        reg = static_cast<u8>(((m_r[1] >> 2) & 0x7Fu) + (reg & 0x7Fu));
        reg = static_cast<u8>(reg | 0x80u);
    } else if (reg == 0) {
        reg = static_cast<u8>((indirectPointer >> IPX_SHIFT) & 0xFFu);
    } else if (reg > 1 && reg < 64) {
        fail("undefined global register access");
        return 0;
    }
    return reg;
}

void Am29000::fetch_decode() {
    constexpr u32 supervisorOnly = 1u << 1;
    constexpr u32 raPresent = 1u << 2;
    constexpr u32 rbPresent = 1u << 3;
    constexpr u32 rcPresent = 1u << 4;
    constexpr u32 sprAccess = 1u << 6;
    const u32 instruction = read_program_word(m_pc);
    if (faulted_) return;
    m_next_ir = instruction;
    const u32 flags = opTable_[instruction >> 24].flags;

    if (USER_MODE) {
        if ((flags & supervisorOnly) != 0) {
            signal_exception(ExceptionProtectionViolation);
            return;
        }
        if ((flags & sprAccess) != 0 &&
            ((instruction >> 8) & 0xFFu) < 128u) {
            signal_exception(ExceptionProtectionViolation);
            return;
        }
        const auto protectedRegister = [&](u32 reg) {
            return (m_rbp & (1u << (reg >> 4))) != 0;
        };
        if (((flags & raPresent) != 0 &&
             protectedRegister((instruction >> 8) & 0xFFu)) ||
            ((flags & rbPresent) != 0 &&
             protectedRegister(instruction & 0xFFu)) ||
            ((flags & rcPresent) != 0 &&
             protectedRegister((instruction >> 16) & 0xFFu))) {
            signal_exception(ExceptionProtectionViolation);
            return;
        }
    }

    if ((m_pl_flags & PFLAG_IRET) != 0) m_next_pc = m_iret_pc;
    else m_next_pc += 4;
}

void Am29000::setInput(int input, bool asserted) {
    if (input < 0 || input >= 4) return;
    const u8 bit = static_cast<u8>(1u << input);
    if (asserted) m_irq_lines = static_cast<u8>(m_irq_lines | bit);
    else m_irq_lines = static_cast<u8>(m_irq_lines & ~bit);
}

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4018 4244 4267 4293 4701)
#endif

#include "am29000_ops.inc"

#ifdef _MSC_VER
#pragma warning(pop)
#endif

int Am29000::run(int instructionSlots) {
    if (instructionSlots <= 0 || faulted_) return 0;
    int consumed = 0;
    while (consumed < instructionSlots && !faulted_) {
        // The Timer Facility advances on every processor cycle, independently
        // of instruction execution and the register-freeze bit.
        timer_cycle();
        external_irq_check();
        timer_interrupt_check();
        m_next_pl_flags = PFLAG_EXECUTE_EN;
        if (!FREEZE_MODE) {
            m_pc1 = m_pc0;
            m_pc0 = m_pc;
        }

        if (m_exceptions != 0) {
            m_ops = m_cps;
            m_cps &= ~(CPS_TE | CPS_TP | CPS_TU | CPS_FZ | CPS_LK |
                       CPS_WM | CPS_PD | CPS_PI | CPS_SM | CPS_DI | CPS_DA);
            m_cps |= CPS_FZ | CPS_PD | CPS_PI | CPS_SM | CPS_DI | CPS_DA;
            if ((m_pl_flags & PFLAG_IRET) != 0) {
                m_pc0 = m_iret_pc;
                m_pc1 = m_next_pc;
            }
            const u32 vector = m_exception_queue[0];
            if ((m_cfg & CFG_VF) != 0) {
                const u32 vectorAddress =
                    (m_vab & 0xFFFFFC00u) | ((vector & 0xFFu) << 2u);
                const u32 entry = readVector
                    ? readVector(vectorAddress)
                    : m_data.read_dword(vectorAddress, false);
                m_pc = entry & ~3u;
                if ((entry & 2u) != 0) m_cps |= CPS_RE;
            } else {
                m_pc = (m_vab & 0xFFFF0000u) |
                       ((vector & 0xFFu) << 8u);
                if ((m_cfg & CFG_RV) != 0) m_cps |= CPS_RE;
            }
            m_next_pc = m_pc;
            m_exceptions = 0;
            m_pl_flags = 0;
        }

        fetch_decode();
        if (faulted_) break;
        if ((m_pl_flags & PFLAG_EXECUTE_EN) != 0) {
            if (!FREEZE_MODE) m_pc2 = m_pc1;
            recentPc_[recentHead_] = m_exec_pc;
            recentIr_[recentHead_] = m_exec_ir;
            recentHead_ = (recentHead_ + 1u) & (recentPc_.size() - 1u);
            recentCount_ = std::min(recentCount_ + 1u, recentPc_.size());
            if (m_exec_pc == diagnosticWatchPc_) ++diagnosticWatchHits_;
            const u32 oldRegister64 = m_r[64];
            (this->*opTable_[m_exec_ir >> 24].opcode)();
            if (faulted_) break;
            if (m_r[64] != oldRegister64) {
                register64ChangePc_[register64ChangeHead_] = m_exec_pc;
                register64ChangeIr_[register64ChangeHead_] = m_exec_ir;
                register64ChangeOld_[register64ChangeHead_] = oldRegister64;
                register64ChangeNew_[register64ChangeHead_] = m_r[64];
                register64ChangeHead_ = (register64ChangeHead_ + 1u) &
                                        (register64ChangePc_.size() - 1u);
                register64ChangeCount_ = std::min(
                    register64ChangeCount_ + 1u, register64ChangePc_.size());
            }
            ++instructions_;
        }

        m_exec_ir = m_next_ir;
        m_pl_flags = m_next_pl_flags;
        m_exec_pc = m_pc;
        m_pc = m_next_pc;
        ++consumed;
    }
    return consumed;
}

#undef fatalerror
#undef logerror
#undef SIGNAL_EXCEPTION
#undef SET_CHC_CR
#undef SET_ALU_BP
#undef SET_ALU_FC
#undef GET_CHC_CR
#undef GET_ALU_BP
#undef GET_ALU_FC
#undef USER_MODE
#undef SUPERVISOR_MODE
#undef FREEZE_MODE

} // namespace openmac
