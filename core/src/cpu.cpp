#include "openmac/cpu.hpp"

namespace openmac {

M68000::M68000(IBus& bus) : bus_(bus) {}

void M68000::reset() {
    sr   = 0x2700;
    ssp  = (u32(bus_.read16(0)) << 16) | bus_.read16(2);
    pc   = (u32(bus_.read16(4)) << 16) | bus_.read16(6);
    a[7] = ssp;
}

int M68000::step() {
    // P1: decode and execute. Until then a step is a 4-cycle no-op so the
    // machine scheduler can be exercised.
    return 4;
}

} // namespace openmac
