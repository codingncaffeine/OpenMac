#pragma once

// MC68030 personality of OpenMac's shared 68020+ execution engine.  Integer
// operations and effective-address decoding are shared with M68040; opcode
// legality, MOVEC/PMMU state, exception frames, caches, and timing are selected
// by the model and remain independently testable.

#include "openmac/cpu040.hpp"

namespace openmac {

class M68030 final : public M68040 {
public:
    explicit M68030(IBus040& bus) : M68040(bus, M68kCpuModel::M68030) {}
};

} // namespace openmac
