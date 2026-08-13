#include <openmac/r65c02.hpp>

#include <doctest/doctest.h>

#include <array>

using namespace openmac;

namespace {

struct Cpu65Fixture {
    std::array<u8, 65536> memory{};
    R65C02 cpu;

    Cpu65Fixture() {
        cpu.read = [this](u16 address) { return memory[address]; };
        cpu.write = [this](u16 address, u8 value) { memory[address] = value; };
    }

    void start(u16 address = 0x0200) {
        memory[0xFFFC] = static_cast<u8>(address);
        memory[0xFFFD] = static_cast<u8>(address >> 8);
        cpu.reset();
    }
};

} // namespace

TEST_CASE("R65C02 executes CMOS addressing and reports documented cycles") {
    Cpu65Fixture f;
    const u8 program[] = {
        0xA2, 0x04,             // LDX #4              2
        0xA9, 0x5A,             // LDA #$5A            2
        0x9D, 0xFE, 0x20,       // STA $20FE,X         5
        0xB2, 0x10,             // LDA ($10)           5
        0x74, 0x20,             // STZ $20,X           4
        0x80, 0x02,             // BRA +2              3
        0xA9, 0xFF,
        0xEA,
    };
    std::copy(std::begin(program), std::end(program), f.memory.begin() + 0x0200);
    f.memory[0x10] = 0x02;
    f.memory[0x11] = 0x21;
    f.start();

    CHECK(f.cpu.step() == 2);
    CHECK(f.cpu.step() == 2);
    CHECK(f.cpu.step() == 5);
    CHECK(f.memory[0x2102] == 0x5A);
    CHECK(f.cpu.step() == 5);
    CHECK(f.cpu.a == 0x5A);
    CHECK(f.cpu.step() == 4);
    CHECK(f.memory[0x24] == 0);
    CHECK(f.cpu.step() == 3);
    CHECK(f.cpu.pc == 0x020F);
}

TEST_CASE("R65C02 Rockwell bit operations and branches preserve flags") {
    Cpu65Fixture f;
    const u8 program[] = {
        0x87, 0x40,             // SMB0 $40
        0x0F, 0x40, 0x02,       // BBR0 not taken
        0x8F, 0x40, 0x02,       // BBS0 taken
        0xA9, 0xFF,
        0x17, 0x40,             // RMB1 $40 (bit 0 stays set)
    };
    std::copy(std::begin(program), std::end(program), f.memory.begin() + 0x0200);
    f.start();
    CHECK(f.cpu.step() == 5);
    CHECK(f.memory[0x40] == 1);
    CHECK(f.cpu.step() == 5);
    CHECK(f.cpu.step() == 7);
    CHECK(f.cpu.pc == 0x020A);
    CHECK(f.cpu.step() == 5);
    CHECK(f.memory[0x40] == 1);
}

TEST_CASE("R65C02 decimal arithmetic, BRK and RTI use CMOS state") {
    Cpu65Fixture f;
    const u8 program[] = {
        0xF8,                   // SED
        0x18,                   // CLC
        0xA9, 0x49,
        0x69, 0x51,             // decimal 49+51 = 00 carry
        0x00, 0xAA,             // BRK signature byte
        0xEA,
    };
    std::copy(std::begin(program), std::end(program), f.memory.begin() + 0x0200);
    f.memory[0xFFFE] = 0x00;
    f.memory[0xFFFF] = 0x03;
    f.memory[0x0300] = 0x40;    // RTI
    f.start();
    CHECK(f.cpu.step() == 2);
    CHECK(f.cpu.step() == 2);
    CHECK(f.cpu.step() == 2);
    CHECK(f.cpu.step() == 3);   // decimal mode adds one cycle
    CHECK(f.cpu.a == 0);
    CHECK((f.cpu.p & R65C02::Carry) != 0);
    CHECK((f.cpu.p & R65C02::Zero) != 0);
    CHECK(f.cpu.step() == 7);
    CHECK(f.cpu.pc == 0x0300);
    CHECK((f.cpu.p & R65C02::Decimal) == 0);
    CHECK(f.cpu.step() == 6);
    CHECK(f.cpu.pc == 0x0208);
    CHECK((f.cpu.p & R65C02::Decimal) != 0);
}

TEST_CASE("R65C02 IRQ and NMI vectors are sampled at instruction boundaries") {
    Cpu65Fixture f;
    f.memory[0x0200] = 0x58;    // CLI
    f.memory[0x0201] = 0xEA;
    f.memory[0xFFFE] = 0x00;
    f.memory[0xFFFF] = 0x03;
    f.memory[0xFFFA] = 0x00;
    f.memory[0xFFFB] = 0x04;
    f.start();
    CHECK(f.cpu.step() == 2);
    f.cpu.setIrq(true);
    CHECK(f.cpu.step() == 7);
    CHECK(f.cpu.pc == 0x0300);
    f.cpu.setIrq(false);
    f.cpu.setNmi(true);
    CHECK(f.cpu.step() == 7);
    CHECK(f.cpu.pc == 0x0400);
}
