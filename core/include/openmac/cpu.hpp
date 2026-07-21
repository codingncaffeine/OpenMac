#pragma once

#include "openmac/bus.hpp"
#include "openmac/types.hpp"

namespace openmac {

// Motorola 68000 interpreter. State-accurate, instruction-level cycle counts.
class M68000 {
public:
    explicit M68000(IBus& bus);

    // Load SSP and PC from vectors 0/1 and enter supervisor mode.
    void reset();

    // Execute one instruction; returns cycles consumed.
    int step();

    // Register file is public: the debugger and test harness read/write it
    // directly. a[7] is always the ACTIVE stack pointer; the inactive one is
    // parked in usp/ssp according to the S bit.
    u32 d[8]{};
    u32 a[8]{};
    u32 usp = 0;
    u32 ssp = 0;
    u32 pc  = 0;
    u16 sr  = 0x2700;

private:
    IBus& bus_;
};

} // namespace openmac
