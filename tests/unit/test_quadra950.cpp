#include <doctest/doctest.h>

#include "../../core/src/machine/quadra950/memctl.hpp"

using namespace openmac;

namespace {

constexpr u32 MB = 1024 * 1024;

// The ROM's way of loading the controller: the value goes to the first
// register, is shifted right one place, goes to the next, and so on.
void load(MemoryController950& mc, u32 firstOffset, u32 value, int count) {
    for (int i = 0; i < count; ++i) {
        mc.write(firstOffset + static_cast<u32>(i) * 4, value);
        value >>= 1;
    }
}

// What the ROM writes once it has sized the banks: one boundary at a time,
// six registers each, the running end of RAM in units of 4 MB.
void setBoundaries(MemoryController950& mc, u32 endA, u32 endB, u32 endC) {
    load(mc, 0x00, endA / (4 * MB), 6);
    load(mc, 0x18, endB / (4 * MB), 6);
    load(mc, 0x30, endC / (4 * MB), 6);
}

} // namespace

TEST_CASE("950 memory controller: registers are one bit wide, loaded by shifting") {
    MemoryController950 mc({4, 4, 0, 0});
    load(mc, 0x00, 0x138B0810u, 32);                  // the ROM's reset value
    CHECK(mc.boundaryBytes(0) == 64 * MB);
    CHECK(mc.boundaryBytes(1) == 128 * MB);
    CHECK(mc.boundaryBytes(2) == 192 * MB);
    CHECK(mc.read(0x10) == 1);                        // bit 4 of the value
    CHECK(mc.read(0x00) == 0);
    CHECK(mc.read(0x48) == 0);                        // bit 18, the one the ROM reads
    CHECK(mc.read(0x4C) == 1);                        // bit 19
}

TEST_CASE("950 memory controller: a bank repeats at its own size inside its range") {
    MemoryController950 mc({4, 16, 0, 0});
    load(mc, 0x00, 0x138B0810u, 32);                  // four 64 MB ranges
    // Bank A, 4 MB, at $0000 0000: the top of its range reads the same cell
    // as the top of its first 4 MB.
    CHECK(mc.translate(0x00000000u) == 0);
    CHECK(mc.translate(64 * MB - 4) == 4 * MB - 4);
    CHECK(mc.translate(4 * MB) == 0);
    // Bank B, 16 MB, at $0400 0000, stored after bank A in the flat array.
    CHECK(mc.translate(64 * MB) == 4 * MB);
    CHECK(mc.translate(64 * MB + 16 * MB) == 4 * MB);
    CHECK(mc.translate(128 * MB - 4) == 4 * MB + 16 * MB - 4);
    // Banks C and D are empty.
    CHECK(mc.translate(128 * MB) == MemoryController950::kNoRam);
    CHECK(mc.translate(192 * MB) == MemoryController950::kNoRam);
    CHECK_FALSE(mc.identityMapped());
}

TEST_CASE("950 memory controller: the settled boundaries put RAM back to back") {
    SUBCASE("two 4 MB banks") {
        MemoryController950 mc({4, 4, 0, 0});
        setBoundaries(mc, 4 * MB, 8 * MB, 8 * MB);
        CHECK(mc.flatBytes() == 8 * MB);
        CHECK(mc.translate(0) == 0);
        CHECK(mc.translate(4 * MB) == 4 * MB);
        CHECK(mc.translate(8 * MB - 1) == 8 * MB - 1);
        CHECK(mc.translate(8 * MB) == MemoryController950::kNoRam);   // bank D, empty
        CHECK(mc.identityMapped());
    }
    SUBCASE("four 64 MB banks") {
        MemoryController950 mc({64, 64, 64, 64});
        setBoundaries(mc, 64 * MB, 128 * MB, 192 * MB);
        CHECK(mc.flatBytes() == 256 * MB);
        CHECK(mc.translate(200 * MB + 12) == 200 * MB + 12);
        CHECK(mc.identityMapped());
    }
    SUBCASE("an empty bank between two fitted ones") {
        MemoryController950 mc({4, 0, 4, 0});
        setBoundaries(mc, 4 * MB, 4 * MB, 8 * MB);
        CHECK(mc.translate(4 * MB + 100) == 4 * MB + 100);   // bank C follows A
        CHECK(mc.identityMapped());
    }
    SUBCASE("a small bank below a large one is rotated, not lost") {
        MemoryController950 mc({4, 16, 0, 0});
        setBoundaries(mc, 4 * MB, 20 * MB, 20 * MB);
        // Bank B covers 4-20 MB and answers at (address mod 16 MB): every
        // one of its bytes has exactly one address, but not in flat order.
        CHECK(mc.translate(4 * MB) == 4 * MB + 4 * MB);
        CHECK(mc.translate(16 * MB) == 4 * MB);
        CHECK(mc.translate(20 * MB - 1) == 4 * MB + 4 * MB - 1);
        CHECK_FALSE(mc.identityMapped());
        bool seen[5] = {false, false, false, false, false};
        for (u32 a = 4 * MB; a < 20 * MB; a += 4 * MB) {
            const u32 unit = (mc.translate(a) >> 22);
            REQUIRE(unit >= 1);
            REQUIRE(unit <= 4);
            CHECK_FALSE(seen[unit]);
            seen[unit] = true;
        }
    }
}

TEST_CASE("950 memory controller: the registers above the boundaries are kept, not acted on") {
    MemoryController950 mc({8, 0, 0, 0});
    setBoundaries(mc, 8 * MB, 8 * MB, 8 * MB);
    mc.write(0xA0, 0xFFFF);
    mc.write(0x84, 1);
    load(mc, 0x48, 0x4E2, 12);
    CHECK(mc.read(0xA0) == 0xFFFF);
    CHECK(mc.read(0x84) == 1);
    CHECK(mc.boundaryBytes(0) == 8 * MB);
    CHECK(mc.translate(8 * MB - 1) == 8 * MB - 1);
    CHECK(mc.identityMapped());
}

TEST_CASE("950 memory controller: only whole SIMM sets are bank sizes") {
    CHECK(MemoryController950::validBankMb(0));
    CHECK(MemoryController950::validBankMb(4));
    CHECK(MemoryController950::validBankMb(64));
    CHECK_FALSE(MemoryController950::validBankMb(12));
    CHECK_FALSE(MemoryController950::validBankMb(128));
    MemoryController950 mc({4, 12, 0, 0});            // 12 MB is not a bank: left empty
    CHECK(mc.flatBytes() == 4 * MB);
}
