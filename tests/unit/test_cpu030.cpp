// MC68030 model boundaries: 030 MOVEC/PMMU state, rejection of 040-only
// F-line operations, and the 030 long bus-fault frame.  Shared integer/EA
// behavior remains covered exhaustively by test_cpu040 and the SST runner.

#include <openmac/cpu030.hpp>

#include <doctest/doctest.h>

#include <cstring>
#include <vector>

using namespace openmac;

namespace {

class Ram030 final : public IBus040 {
public:
    explicit Ram030(u32 size = 1u << 20) : mem(size, 0) {}

    u8 read8(u32 a) override {
        if (a >= mem.size() || a == faultAt) throw BusFault{a, true, 1};
        return mem[a];
    }
    u16 read16(u32 a) override {
        if (a == faultAt) throw BusFault{a, true, 2};
        return static_cast<u16>((read8(a) << 8) | read8(a + 1));
    }
    u32 read32(u32 a) override {
        ++read32Count;
        if (a == faultAt) throw BusFault{a, true, 4};
        return (static_cast<u32>(read16(a)) << 16) | read16(a + 2);
    }
    void write8(u32 a, u8 v) override {
        if (a >= mem.size() || a == faultAt) throw BusFault{a, false, 1};
        mem[a] = v;
    }
    void write16(u32 a, u16 v) override {
        if (a == faultAt) throw BusFault{a, false, 2};
        write8(a, static_cast<u8>(v >> 8));
        write8(a + 1, static_cast<u8>(v));
    }
    void write32(u32 a, u32 v) override {
        if (a == faultAt) throw BusFault{a, false, 4};
        write16(a, static_cast<u16>(v >> 16));
        write16(a + 2, static_cast<u16>(v));
    }

    std::vector<u8> mem;
    u32 faultAt = 0xFFFFFFFFu;
    u32 read32Count = 0;
};

struct Fix030 {
    Ram030 bus;
    M68030 cpu{bus};

    Fix030() {
        bus.write32(0, 0x00010000);
        bus.write32(4, 0x00000400);
        cpu.reset();
    }

    int run(std::initializer_list<u16> words) {
        u32 at = cpu.pc;
        for (u16 w : words) { bus.write16(at, w); at += 2; }
        return cpu.step();
    }
};

} // namespace

TEST_CASE("030 reset selects 030 control-register layout") {
    Fix030 f;
    CHECK(f.cpu.model() == M68kCpuModel::M68030);
    CHECK(f.cpu.mmuEnabled() == false);
    CHECK(f.cpu.caar == 0);
    CHECK(f.cpu.crp == 0);
    CHECK(f.cpu.srp030 == 0);
}

TEST_CASE("030 MOVEC implements CAAR and 030 CACR masks") {
    Fix030 f;
    f.cpu.d[0] = 0x12345678;
    f.run({0x4E7B, 0x0802});               // MOVEC D0,CAAR
    CHECK(f.cpu.caar == 0x12345678);

    f.cpu.d[1] = 0xFFFFFFFF;
    f.run({0x4E7B, 0x1002});               // MOVEC D1,CACR
    CHECK(f.cpu.cacr == 0x00003313);

    f.run({0x4E7A, 0x2802});               // MOVEC CAAR,D2
    CHECK(f.cpu.d[2] == 0x12345678);
}

TEST_CASE("030 timing tables are independent of the 040 execute counts") {
    SUBCASE("pipeline and branch controls") {
        Fix030 f;
        CHECK(f.run({0x4E71}) == 2);             // NOP

        f.cpu.setCCR(0);                         // BNE taken
        CHECK(f.run({0x6602}) == 6);
        f.cpu.setCCR(0x04);                      // BNE.B not taken
        CHECK(f.run({0x6602}) == 4);
        f.cpu.setCCR(0x04);                      // BNE.W not taken
        CHECK(f.run({0x6600, 0x0002}) == 6);

        f.cpu.d[0] = 1;                          // DBF, count not expired
        CHECK(f.run({0x51C8, 0x0002}) == 6);
        f.cpu.d[0] = 0;                          // DBF, count expires
        CHECK(f.run({0x51C8, 0x0002}) == 10);
    }

    SUBCASE("fetch, calculate and jump EA tables") {
        Fix030 f;
        f.cpu.a[0] = 0x2000;
        f.cpu.a[1] = 0x2100;
        f.bus.write32(0x2000, 0x12345678);
        CHECK(f.run({0x2010}) == 5);             // MOVE.L (A0),D0: 2+fea(An)=5
        CHECK(f.run({0x2290}) == 6);             // MOVE.L (A0),(A1), overlapped tails

        f.cpu.a[0] = f.cpu.pc + 2;
        CHECK(f.run({0x4ED0}) == 6);             // JMP (A0): 4+jea(An)=6
    }

    SUBCASE("full-format and memory-indirect EA timing") {
        Fix030 f;
        f.cpu.a[0] = 0x2000;
        f.bus.write32(0x2000, 0x11223344);
        // Full extension, index suppressed, word base displacement, no
        // indirection: MOVE.L (0,A0),D0.
        CHECK(f.run({0x2030, 0x0160, 0x0000}) == 8);
        CHECK(f.cpu.d[0] == 0x11223344);

        f.cpu.a[0] = 0x2000;
        f.bus.write32(0x2000, 0x00003000);
        f.bus.write32(0x3000, 0xAABBCCDD);
        // I/IS=1 selects preindexed memory indirect with null outer disp.
        CHECK(f.run({0x2230, 0x0161, 0x0000}) == 12);
        CHECK(f.cpu.d[1] == 0xAABBCCDD);

        f.cpu.a[0] = f.cpu.pc + 6;
        CHECK(f.run({0x4EF0, 0x0160, 0x0000}) == 10); // JMP full (0,A0)
    }

    SUBCASE("MOVEC control-register classes") {
        Fix030 f;
        f.cpu.d[0] = 0x12345678;
        CHECK(f.run({0x4E7B, 0x0801}) == 6);     // D0,VBR (group A)
        CHECK(f.run({0x4E7B, 0x0002}) == 12);    // D0,CACR (group B)
        CHECK(f.run({0x4E7A, 0x1801}) == 6);     // VBR,D1
    }

    SUBCASE("040 timing remains model selected") {
        Ram030 bus;
        bus.write32(0, 0x00010000);
        bus.write32(4, 0x00000400);
        bus.write16(0x0400, 0x4E71);
        M68040 cpu{bus};
        cpu.reset();
        CHECK(cpu.step() == 8);
    }
}

TEST_CASE("030 exception timing uses the documented frame operations") {
    SUBCASE("illegal instruction and normal RTE") {
        Fix030 f;
        f.bus.write32(4 * 4, 0x00000800);
        CHECK(f.run({0x4AFC}) == 18);

        f.bus.write16(0x0800, 0x4E73);           // RTE
        CHECK(f.cpu.step() == 18);
        CHECK(f.cpu.pc == 0x00000400);
    }

    SUBCASE("interrupt on the interrupt stack") {
        Fix030 f;
        f.bus.write32(25 * 4, 0x00000800);
        f.cpu.setSR(0x2000);                     // supervisor, IPL 0, I-stack
        f.cpu.setIrqLevel(1);
        CHECK(f.cpu.step() == 23);
        CHECK(f.cpu.pc == 0x00000800);
    }
}

TEST_CASE("030 uses external MC68882 operation and operand-format timings") {
    Fix030 f;

    f.cpu.d[0] = 100;
    CHECK(f.run({0xF200, 0x4000}) == 43);         // FMOVE.L D0,FP0
    f.cpu.d[1] = 23;
    CHECK(f.run({0xF201, 0x4022}) == 89);         // FADD.L D1,FP0
    CHECK(f.cpu.fp[0] == doctest::Approx(123.0));

    f.cpu.fp[2] = 81.0;
    CHECK(f.run({0xF200, static_cast<u16>((2u << 10) | (3u << 7) | 0x04u)}) == 110);
    CHECK(f.cpu.fp[3] == doctest::Approx(9.0));    // FSQRT.X FP2,FP3

    CHECK(f.run({0xF200, 0x5C00}) == 32);         // FMOVECR #pi,FP0

    f.cpu.a[0] = 0x2000;
    const float source = 2.0f;
    u32 sourceBits = 0;
    std::memcpy(&sourceBits, &source, sizeof sourceBits);
    f.bus.write32(0x2000, sourceBits);
    CHECK(f.run({0xF210, 0x4422}) == 71);         // FADD.S (A0),FP0

    f.cpu.a[0] = 0x2100;
    CHECK(f.run({0xF210, 0x6800}) == 52);         // FMOVE.X FP0,(A0)
}

TEST_CASE("030 FSAVE answers the Macintosh FPU probe as a 68882") {
    // The System's Gestalt/SysEnvirons code: FNOP, FSAVE -(SP), then the
    // frame's first word decides -- $1F18/$3F18 = 68881, $1F38/$3F38 = 68882,
    // anything else = no FPU (which is what a 68040-shaped frame said here).
    Fix030 f;
    f.cpu.a[7] = 0x8000;
    f.run({0xF317});                       // FSAVE (A7): untouched FPU -> NULL
    CHECK(f.bus.read32(0x8000) == 0);

    f.run({0xF280, 0x0000});               // FNOP: the FPU has been touched
    f.run({0xF327});                       // FSAVE -(A7)
    CHECK(f.cpu.a[7] == 0x8000 - 60);      // 68882 IDLE frame: 60 bytes
    CHECK(f.bus.read16(0x8000 - 60) == 0x1F38);
    CHECK(f.bus.read16(0x8000 - 60 + 2) == 0);

    f.run({0xF35F});                       // FRESTORE (A7)+ skips the frame
    CHECK(f.cpu.a[7] == 0x8000);
    f.run({0xF327});                       // still IDLE afterwards
    CHECK(f.bus.read16(0x8000 - 60) == 0x1F38);

    f.cpu.a[7] = 0x8000;
    f.bus.write32(0x8000, 0);
    f.run({0xF35F});                       // FRESTORE of a NULL frame resets it
    CHECK(f.cpu.a[7] == 0x8004);
    f.run({0xF327});
    CHECK(f.bus.read32(0x8004 - 4) == 0);
}

TEST_CASE("030 instruction cache retains code until CAAR-selected invalidation") {
    Fix030 f;
    f.bus.write32(4 * 4, 0x00000800);       // illegal-instruction vector
    f.bus.write16(0x0400, 0x4E71);          // NOP
    f.cpu.cacr = 0x00000001;                // EI

    f.cpu.step();
    CHECK(f.cpu.pc == 0x0402);
    CHECK(f.cpu.cacheStats030().instructionMisses == 1);

    // External memory changes do not alter a valid logical I-cache entry.
    f.bus.mem[0x0400] = 0x4A;
    f.bus.mem[0x0401] = 0xFC;                // ILLEGAL
    f.cpu.pc = 0x0400;
    f.cpu.step();
    CHECK(f.cpu.pc == 0x0402);
    CHECK(f.cpu.cacheStats030().instructionHits == 1);

    // CEI is a write-only command. CAAR bits 7..2 select the exact entry;
    // the persistent EI control bit remains set after MOVEC.
    f.bus.write16(0x0510, 0x4E7B);           // MOVEC D0,CACR
    f.bus.write16(0x0512, 0x0002);
    f.cpu.d[0] = 0x00000005;                 // EI | CEI
    f.cpu.caar = 0x00000000;                 // line 0, entry 0 ($0400)
    f.cpu.pc = 0x0510;
    f.cpu.step();
    CHECK(f.cpu.cacr == 0x00000001);

    f.cpu.pc = 0x0400;
    f.cpu.step();
    CHECK(f.cpu.pc == 0x00000800);            // refetched ILLEGAL
}

TEST_CASE("030 data cache is write-through and clear-data forces a refill") {
    Fix030 f;
    f.cpu.cacr = 0x00000100;                  // ED
    f.cpu.a[0] = 0x00002000;
    f.bus.write32(0x2000, 0x11223344);
    f.bus.write16(0x0400, 0x2010);            // MOVE.L (A0),D0
    f.bus.write16(0x0402, 0x2210);            // MOVE.L (A0),D1
    f.bus.write16(0x0404, 0x2081);            // MOVE.L D1,(A0)
    f.bus.write16(0x0406, 0x2410);            // MOVE.L (A0),D2
    f.bus.write16(0x0408, 0x4E7B);            // MOVEC D3,CACR
    f.bus.write16(0x040A, 0x3002);
    f.bus.write16(0x040C, 0x2810);            // MOVE.L (A0),D4

    f.cpu.step();
    CHECK(f.cpu.d[0] == 0x11223344);
    f.bus.mem[0x2000] = 0xAA;                 // non-CPU/DMA-style change
    f.bus.mem[0x2001] = 0xBB;
    f.bus.mem[0x2002] = 0xCC;
    f.bus.mem[0x2003] = 0xDD;
    f.cpu.step();
    CHECK(f.cpu.d[1] == 0x11223344);           // cache hit, still old

    f.cpu.d[1] = 0x55667788;
    f.cpu.step();
    CHECK(f.bus.read32(0x2000) == 0x55667788); // write-through memory
    f.cpu.step();
    CHECK(f.cpu.d[2] == 0x55667788);           // and updated cache hit

    f.bus.mem[0x2000] = 0xDE;
    f.bus.mem[0x2001] = 0xAD;
    f.bus.mem[0x2002] = 0xBE;
    f.bus.mem[0x2003] = 0xEF;
    f.cpu.d[3] = 0x00000900;                  // ED | CD
    f.cpu.step();
    CHECK(f.cpu.cacr == 0x00000100);           // CD self-cleared
    f.cpu.step();
    CHECK(f.cpu.d[4] == 0xDEADBEEF);
    CHECK(f.cpu.cacheStats030().dataHits >= 2);
    CHECK(f.cpu.cacheStats030().dataMisses == 2);
}

TEST_CASE("030 logical cache hits precede PMMU translation") {
    Fix030 f;
    f.cpu.cacr = 0x00000101;                  // EI | ED
    f.cpu.a[0] = 0x00002000;
    f.bus.write16(0x0400, 0x2010);            // MOVE.L (A0),D0
    f.bus.write32(0x2000, 0x11223344);
    f.cpu.step();                             // populate both logical caches
    CHECK(f.cpu.d[0] == 0x11223344);

    f.bus.mem[0x2000] = 0xAA;
    f.bus.mem[0x2001] = 0xBB;
    f.bus.mem[0x2002] = 0xCC;
    f.bus.mem[0x2003] = 0xDD;
    f.cpu.tc = 0x80000000u;                   // enabled, deliberately invalid root
    f.cpu.pc = 0x0400;
    f.cpu.d[0] = 0;
    f.cpu.step();

    CHECK(f.cpu.pc == 0x0402);                // no ATC/table-search fault
    CHECK(f.cpu.d[0] == 0x11223344);          // stale logical-cache value wins
}

TEST_CASE("030 TT cache-inhibit prevents both on-chip and external cache fills") {
    Fix030 f;
    f.cpu.cacr = 0x00000100;                  // ED
    // Match every supervisor-data address, both reads and writes: E | CI |
    // RWM, FC base 5. TTx remains active with TC.E clear.
    f.cpu.tt0 = 0x00FF8550u;
    f.cpu.a[0] = 0x00002000;
    f.bus.write32(0x2000, 0x11223344);
    f.run({0x2010});                          // MOVE.L (A0),D0
    CHECK(f.cpu.d[0] == 0x11223344);

    f.bus.write32(0x2000, 0xAABBCCDD);
    f.run({0x2210});                          // MOVE.L (A0),D1
    CHECK(f.cpu.d[1] == 0xAABBCCDD);
    CHECK(f.cpu.cacheStats030().dataHits == 0);
    CHECK(f.cpu.cacheStats030().dataMisses == 0);
}

TEST_CASE("030 PMOVE transfers TC, root pointers, TT and MMUSR") {
    Fix030 f;
    f.cpu.a[0] = 0x2000;

    f.bus.write32(0x2000, 0x00C0AA00);
    f.run({0xF010, 0x4000});                // PMOVE (A0),TC
    CHECK(f.cpu.tc == 0x00C0AA00);

    f.bus.write32(0x2000, 0x80000000);
    f.bus.write32(0x2004, 0x00003002);
    f.run({0xF010, 0x4C00});                // PMOVE (A0),CRP
    CHECK(f.cpu.crp == 0x8000000000003002ull);

    f.bus.write32(0x2000, 0x50FF83E7);
    f.run({0xF010, 0x0800});                // PMOVE (A0),TT0
    CHECK(f.cpu.tt0 == 0x50FF83E7);

    f.bus.write16(0x2000, 0xA55A);
    f.run({0xF010, 0x6000});                // PMOVE (A0),MMUSR
    CHECK(f.cpu.mmusr == 0xA55A);

    f.cpu.a[1] = 0x2100;
    f.run({0xF011, 0x6200});                // PMOVE MMUSR,(A1)
    CHECK(f.bus.read16(0x2100) == 0xA55A);
}

TEST_CASE("030 does not decode 040 MOVE16") {
    Fix030 f;
    f.bus.write32(11 * 4, 0x00000800);      // line-F vector
    f.run({0xF620, 0x0000});
    CHECK(f.cpu.pc == 0x00000800);
}

TEST_CASE("030 data bus fault builds a format-B frame") {
    Fix030 f;
    f.bus.write32(2 * 4, 0x00000800);
    f.cpu.a[0] = 0x00008000;
    f.bus.faultAt = 0x00008000;
    const u32 oldSp = f.cpu.a[7];
    const int clocks = f.run({0x2010});    // MOVE.L (A0),D0
    CHECK(clocks == 62);
    CHECK(f.cpu.pc == 0x00000800);
    CHECK(f.cpu.a[7] == oldSp - 92);
    CHECK((f.bus.read16(f.cpu.a[7] + 6) >> 12) == 0xB);
    CHECK(f.bus.read32(f.cpu.a[7] + 0x10) == 0x00008000);
    CHECK((f.bus.read16(f.cpu.a[7] + 0x0A) & 0x0100) != 0); // DF
}
