#pragma once

#include "openmac/types.hpp"

namespace openmac {

// 68000 bus: 24-bit address space, big-endian. Word accesses are even-aligned;
// the CPU raises an address error before an odd word access reaches the bus.
class IBus {
public:
    virtual ~IBus() = default;

    virtual u8   read8(u32 addr) = 0;
    virtual u16  read16(u32 addr) = 0;
    virtual void write8(u32 addr, u8 value) = 0;
    virtual void write16(u32 addr, u16 value) = 0;
};

} // namespace openmac
