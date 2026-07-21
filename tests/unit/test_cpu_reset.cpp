#include <doctest/doctest.h>
#include <openmac/cpu.hpp>

#include <vector>

namespace {

class RamBus final : public openmac::IBus {
public:
    explicit RamBus(std::size_t size) : mem(size, 0) {}

    openmac::u8 read8(openmac::u32 addr) override { return mem[addr]; }
    openmac::u16 read16(openmac::u32 addr) override {
        return static_cast<openmac::u16>((mem[addr] << 8) | mem[addr + 1]);
    }
    void write8(openmac::u32 addr, openmac::u8 value) override { mem[addr] = value; }
    void write16(openmac::u32 addr, openmac::u16 value) override {
        mem[addr]     = static_cast<openmac::u8>(value >> 8);
        mem[addr + 1] = static_cast<openmac::u8>(value & 0xFF);
    }

    std::vector<openmac::u8> mem;
};

} // namespace

TEST_CASE("reset loads SSP and PC from the vector table and enters supervisor mode") {
    RamBus bus(0x1000);
    bus.write16(0, 0x0000);
    bus.write16(2, 0x2000); // vector 0: SSP = $00002000
    bus.write16(4, 0x0000);
    bus.write16(6, 0x0400); // vector 1: PC = $00000400

    openmac::M68000 cpu(bus);
    cpu.reset();

    CHECK(cpu.a[7] == 0x2000);
    CHECK(cpu.pc == 0x400);
    CHECK((cpu.getSR() & 0x2000) != 0);
}
