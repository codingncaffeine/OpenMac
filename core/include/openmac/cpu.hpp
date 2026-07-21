#pragma once

#include "openmac/bus.hpp"
#include "openmac/types.hpp"

#include <functional>

namespace openmac {

struct CpuOps;

// Motorola 68000 interpreter. State-accurate, instruction-level cycle counts.
class M68000 {
public:
    explicit M68000(IBus& bus);

    // Load SSP and PC from vectors 0/1, enter supervisor mode, IRQ mask 7.
    void reset();

    // Execute one instruction (or take a pending exception/interrupt).
    // Returns cycles consumed.
    int step();

    // Level currently asserted by the interrupt controller (0 = none).
    // Level 7 is treated as non-maskable on a 6->7 transition.
    void setIrqLevel(int level);

    // Register file: public for the debugger and test harness.
    // a[7] is always the ACTIVE stack pointer. Of usp/ssp, the field for the
    // inactive bank holds that bank's true value; the active bank's truth
    // lives in a[7]. uspValue()/sspValue() resolve either bank correctly.
    u32 d[8]{};
    u32 a[8]{};
    u32 usp = 0;
    u32 ssp = 0;
    u32 pc  = 0;

    u16  getSR() const { return sr_; }
    void setSR(u16 value);   // masks unimplemented bits, banks stack pointers
    void setCCR(u8 value);
    u32  uspValue() const;
    u32  sspValue() const;

    bool stopped = false;    // STOP: waiting for an interrupt
    bool halted  = false;    // double fault: only reset() recovers

    // Recent instruction addresses, newest first (recentPc(0) = last executed).
    u32 recentPc(int back) const { return pcRing_[(pcRingPos_ - 1 - back) & 127]; }

    // The RESET instruction pulses /RSTO: peripherals reset, CPU continues.
    std::function<void()> onResetInstruction;

    // Fires as any exception is entered (vector, address of the faulting
    // instruction). Diagnostics only.
    std::function<void(int vector, u32 pc)> onException;

private:
    friend struct CpuOps;

    // Data access. Word/long access to an odd address raises an address
    // error (thrown, caught in step(), turned into a group-0 exception).
    u8   rd8(u32 addr);
    u16  rd16(u32 addr);
    u32  rd32(u32 addr);
    void wr8(u32 addr, u8 v);
    void wr16(u32 addr, u16 v);
    void wr32(u32 addr, u32 v);

    // Instruction-stream fetch at pc (advances pc).
    u16 fetch16();
    u32 fetch32();

    void push16(u16 v);
    void push32(u32 v);
    u16  pop16();
    u32  pop32();

    // Enter a group 1/2 exception: push PC/SR, load vector. Returns cycles.
    int exception(int vector, int cycles);
    int doInterrupt(int level);

    IBus& bus_;
    u16 sr_ = 0x2700;
    int irqLevel_ = 0;
    u32 instrStart_ = 0;   // address of the currently executing instruction
    u16 ir_ = 0;           // currently executing opcode (group-0 frames)
    u32 pcRing_[128]{};     // recent instruction addresses (debug)
    int pcRingPos_ = 0;

    // Address-register bookkeeping around a faulting EA access:
    //  kind 1: (An)+ — rolled back only when the faulting access is a WRITE
    //  kind 2: -(An).l — two -2 steps, low word first; a fault leaves the
    //          register (and the frame's access address) at initial-2
    int eaUndoReg_ = -1;
    u32 eaUndoVal_ = 0;
    int eaUndoKind_ = 0;
    int eaFaultCycles_ = 0;   // pre-fault cycles if the pending access faults
};

} // namespace openmac
