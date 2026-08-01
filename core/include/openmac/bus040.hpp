#pragma once

// 68040 bus: 32-bit address space, big-endian, longword-capable. The CPU core
// splits misaligned data accesses into naturally-aligned pieces itself (the
// '040 does the same in its bus controller), so implementations only ever see
// aligned 8/16/32-bit cycles. A machine signals a failed cycle -- unmapped
// space, a NuBus slot probe, write to protected space -- by throwing BusFault;
// the CPU turns it into a format $7 access-error exception.

#include "openmac/types.hpp"

namespace openmac {

// Thrown by an IBus040 implementation when a cycle terminates in a bus error.
struct BusFault {
    u32  addr;
    bool read;
    int  size;   // bytes: 1, 2 or 4
};

class IBus040 {
public:
    virtual ~IBus040() = default;

    virtual u8   read8(u32 addr) = 0;
    virtual u16  read16(u32 addr) = 0;
    virtual u32  read32(u32 addr) = 0;
    virtual void write8(u32 addr, u8 value) = 0;
    virtual void write16(u32 addr, u16 value) = 0;
    virtual void write32(u32 addr, u32 value) = 0;
};

} // namespace openmac
