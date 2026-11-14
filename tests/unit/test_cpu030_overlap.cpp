// MC68030 overlap timing (M68040::setOverlapTiming030): the worked examples
// of the user's manual, section 11.3.4 to 11.5, run as written, and the
// footnoted MOVEM rules. Each stream runs twice: the first pass brings its
// words into the instruction cache (and its operands into the data cache
// where the example wants them there), the second is the one timed.

#include <openmac/cpu030.hpp>

#include <doctest/doctest.h>

#include <initializer_list>
#include <vector>

using namespace openmac;

namespace {

// RAM whose every bus cycle is `wait` clocks longer than two.
class WaitRam final : public IBus040 {
public:
    WaitRam() : mem(1u << 20, 0) {}

    u8 read8(u32 a) override { charge(); return mem.at(a); }
    u16 read16(u32 a) override { charge(); return get16(a); }
    u32 read32(u32 a) override {
        charge();
        return (static_cast<u32>(get16(a)) << 16) | get16(a + 2);
    }
    void write8(u32 a, u8 v) override { charge(); mem.at(a) = v; }
    void write16(u32 a, u16 v) override { charge(); put16(a, v); }
    void write32(u32 a, u32 v) override {
        charge();
        put16(a, static_cast<u16>(v >> 16));
        put16(a + 2, static_cast<u16>(v));
    }
    int takeCyclePenalty() override { const int p = pending; pending = 0; return p; }
    int pendingCyclePenalty() const override { return pending; }

    u16 get16(u32 a) const { return static_cast<u16>((mem.at(a) << 8) | mem.at(a + 1)); }
    void put16(u32 a, u16 v) { mem.at(a) = static_cast<u8>(v >> 8); mem.at(a + 1) = static_cast<u8>(v); }
    void put32(u32 a, u32 v) { put16(a, static_cast<u16>(v >> 16)); put16(a + 2, static_cast<u16>(v)); }

    std::vector<u8> mem;
    int wait = 0;
    int pending = 0;

private:
    void charge() { pending += wait; }
};

constexpr u32 kCode = 0x0400;
constexpr u32 kInstructionCacheOn = 0x0001;
constexpr u32 kDataCacheOn = 0x0100;

struct Overlap030 {
    WaitRam bus;
    M68030 cpu{bus};

    explicit Overlap030(bool overlap = true) {
        bus.put32(0, 0x00010000);
        bus.put32(4, kCode);
        cpu.reset();
        cpu.setOverlapTiming030(overlap);
        cpu.cacr = kInstructionCacheOn;
    }

    void code(std::initializer_list<u16> words) {
        u32 at = kCode;
        for (u16 w : words) { bus.put16(at, w); at += 2; }
    }

    // The registers the manual's examples use, and the memory behind them.
    void stage() {
        cpu.pc = kCode;
        cpu.a[1] = 0x2014;      // -(A1) reads $2010; (8,A1) then writes $2018
        cpu.a[2] = 0x2020;      // ([A2]): the pointer at $2020, the operand at $2030
        cpu.a[3] = 0x2050;
        cpu.a[6] = 0x2040;
        cpu.d[1] = 0x00000001;
        cpu.d[3] = 0x00000005;
        bus.put32(0x2010, 0x00000007);
        bus.put32(0x2020, 0x00002030);
        bus.put32(0x2030, 0x0000FFFF);
        bus.put32(0x2040, 0x12345678);
        bus.mem[0x2050] = 0;
    }

    int run(int instructions) {
        int clocks = 0;
        for (int i = 0; i < instructions; ++i) clocks += cpu.step();
        return clocks;
    }
};

// ADD.L -(A1),D1 / AND.L D1,([A2]) / MOVE.L (A6),(8,A1) / TAS (A3)+ / NEG.L D3
#define MANUAL_STREAM 0xD2A1, 0xC3B2, 0x0151, 0x2356, 0x0008, 0x4ADB, 0x4483

} // namespace

TEST_CASE("030 overlap timing: the manual's instruction stream, 40 clocks") {
    // 11.3.4: 4 + 2 + 10 + 3 + [3 - 1] + [4 - 1] + 2 + 12 + 2. The fetch of
    // (A6) starts under the tail of the AND's write, and the MOVE's
    // destination under the tail of that fetch.
    Overlap030 f;
    f.code({MANUAL_STREAM});
    f.stage();
    (void)f.run(5);
    f.stage();
    CHECK(f.run(5) == 40);

    SUBCASE("the tables' plain sum without it, as before") {
        Overlap030 plain(false);
        plain.code({MANUAL_STREAM});
        plain.stage();
        (void)plain.run(5);
        plain.stage();
        CHECK(plain.run(5) == 41);
    }
}

TEST_CASE("030 overlap timing: operands in the data cache, 37 clocks") {
    // 11.4: 3 + 2 + 10 + 3 + [2 - 1] + 4 + 2 + 12 (the example's four
    // instructions). A fetch the cache answers is a clock shorter and its
    // tail a clock shorter.
    Overlap030 f;
    f.cpu.cacr = kInstructionCacheOn | kDataCacheOn;
    f.code({MANUAL_STREAM});
    f.stage();
    (void)f.run(5);
    f.stage();
    CHECK(f.run(4) == 37);
}

TEST_CASE("030 overlap timing: data cache and four-clock bus cycles, 45 clocks") {
    // 11.5, the second example: 3 + 2 + 12 + 5 + [2 - 1] + 6 + 2 + 14. The
    // pointer of ([A2]) is read from memory, every other operand from the
    // cache; writes always reach the bus, and a write's two extra clocks are
    // tail for the next instruction's head.
    Overlap030 f;
    f.cpu.cacr = kInstructionCacheOn | kDataCacheOn;
    f.code({MANUAL_STREAM});
    f.stage();
    (void)f.run(5);
    f.stage();
    f.bus.wait = 2;
    (void)f.cpu.invalidateDataCache030(0x2020, 4);
    CHECK(f.run(4) == 45);
}

TEST_CASE("030 overlap timing: a write's wait states run under the next head") {
    // MOVE.W D3,(A2)+ (no head, tail 1, 3 clocks) then BSET #0,D2 (head 6,
    // 6 clocks), memory cycles of four clocks: the write is 5 with a tail of
    // 3, and the BSET's first three clocks run under it.
    Overlap030 f;
    f.code({0x34C3, 0x08C2, 0x0000});
    f.stage();
    (void)f.run(2);
    f.stage();
    f.bus.wait = 2;
    CHECK(f.cpu.step() == 5);
    CHECK(f.cpu.step() == 3);
    CHECK(f.cpu.pendingTail030() == 0);

    SUBCASE("an instruction whose words come over the bus overlaps nothing") {
        Overlap030 cold;
        cold.cpu.cacr = 0;                  // no instruction cache: every word is fetched
        cold.code({0x34C3, 0x08C2, 0x0000});
        cold.stage();
        CHECK(cold.cpu.step() == 3);
        CHECK(cold.cpu.step() == 6);
    }

    SUBCASE("exception processing overlaps nothing") {
        // MOVE.W D3,(A2)+ then DIVU.W D4,D0 with a divisor of zero: the
        // divide's own head is two clocks, the exception's none.
        Overlap030 trap;
        trap.bus.put32(5 * 4, 0x00000800);
        trap.code({0x34C3, 0x80C4});
        trap.stage();
        trap.cpu.d[4] = 0;
        (void)trap.run(2);
        trap.stage();
        trap.bus.wait = 2;
        CHECK(trap.cpu.step() == 5);
        CHECK(trap.cpu.pendingTail030() == 3);
        // The frame is six words at an address that is not a long word's:
        // five write cycles, and one read for the vector.
        CHECK(trap.cpu.step() == 24 + 6 * 2);
        CHECK(trap.cpu.pc == 0x00000800);
        CHECK(trap.cpu.pendingTail030() == 0);
    }
}

TEST_CASE("030 overlap timing: a read's wait states lengthen the fetch's tail") {
    // MOVE.L (A6),(8,A1), memory cycles of four clocks, nothing cached: the
    // fetch is 3 + 2 with a tail of 1 + 2, the store 4 + 2 with a head of 2
    // that runs under it: 5 + 6 - 2.
    Overlap030 f;
    f.code({0x2356, 0x0008});
    f.stage();
    f.cpu.a[1] = 0x2010;
    (void)f.run(1);
    f.stage();
    f.cpu.a[1] = 0x2010;
    f.bus.wait = 2;
    CHECK(f.cpu.step() == 9);
    CHECK(f.cpu.pendingTail030() == 2);
}

TEST_CASE("030 overlap timing: MOVEM with wait states, the table's footnote") {
    // MOVEM.L D0-D3,-(A7), four registers, two wait states: (4 + 2n) +
    // (n - 1)w = 18 and 2 for the register list's word, with (n - 1)w = 6
    // left running; MOVEQ's two clocks (all head) vanish under it.
    Overlap030 f;
    f.code({0x48E7, 0xF000, 0x7000, 0x4CDF, 0x000F});
    f.stage();
    f.cpu.a[7] = 0x3000;
    (void)f.run(3);
    f.stage();
    f.cpu.a[7] = 0x3000;
    f.bus.wait = 2;
    CHECK(f.cpu.step() == 20);
    CHECK(f.cpu.pendingTail030() == 6);
    CHECK(f.cpu.step() == 0);
    // MOVEM.L (A7)+,D0-D3: 8 + 4n and 4 for the list's word; a read of four
    // clocks costs what one of two does.
    CHECK(f.cpu.step() == 28);
    CHECK(f.cpu.pendingTail030() == 0);
}
