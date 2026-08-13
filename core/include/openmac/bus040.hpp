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

    // A translated 68030 page/TT entry may assert CIOUT even though the
    // physical target is ordinary RAM.  These reads perform the bus transfer
    // while bypassing any board-level cache.  The default is appropriate for
    // buses without an external cache.
    virtual u8 read8CacheInhibited(u32 addr) { return read8(addr); }
    virtual u16 read16CacheInhibited(u32 addr) { return read16(addr); }
    virtual u32 read32CacheInhibited(u32 addr) { return read32(addr); }

    // CIIN/board-decode result for an otherwise valid physical cycle.  The
    // 68030's on-chip caches only fill from cacheable 32-bit memory; device
    // registers must retain their read side effects.  Existing test buses and
    // the 68040 machines are ordinary memory by default.
    virtual bool cacheable(u32 /*addr*/) const { return true; }

    // A 68030 cache tag miss may request the four-longword line beginning at
    // firstAddr's current entry and wrapping modulo 16 bytes.  Return true
    // only when board hardware acknowledged the burst and filled all four
    // outputs in request order; false means out[0] alone is valid.
    virtual bool readBurst32(u32 firstAddr, u32 out[4]) {
        out[0] = read32(firstAddr);
        return false;
    }

    // Additional processor clocks caused by board-level retry/wait behavior
    // since the preceding call.  Instruction tables already include their
    // nominal two-clock accesses, so this reports only the excess.
    virtual int takeCyclePenalty() { return 0; }
};

} // namespace openmac
