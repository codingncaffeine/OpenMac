// Board-level IIfx tests: reset overlay, I/O mirrors, VIA identity, OSS
// priorities/acknowledge and the host-visible IOP shared-memory protocol.

#include <openmac/iifx.hpp>
#include <openmac/hfs.hpp>

#include "../../core/src/machine/adb.hpp"
#include "../../core/src/machine/dc42.hpp"
#include "../../core/src/machine/iifx/adb_bus.hpp"
#include "../../core/src/machine/iifx/am29000.hpp"
#include "../../core/src/machine/iifx/asc.hpp"
#include "../../core/src/machine/iifx/iop.hpp"
#include "../../core/src/machine/iifx/nubus_video.hpp"
#include "../../core/src/machine/quadra/scc.hpp"

#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

using namespace openmac;

namespace {

void put32(std::vector<u8>& bytes, u32 at, u32 value) {
    bytes[at] = static_cast<u8>(value >> 24);
    bytes[at + 1] = static_cast<u8>(value >> 16);
    bytes[at + 2] = static_cast<u8>(value >> 8);
    bytes[at + 3] = static_cast<u8>(value);
}

std::vector<u8> testRom() {
    std::vector<u8> rom(512u * 1024u, 0x4E);
    // The IIfx ROM header doubles as reset vectors: checksum-shaped initial
    // SSP followed by the native-ROM entry point.
    put32(rom, 0, 0x00100000);
    put32(rom, 4, 0x40000100);
    for (u32 at = 0x100; at + 1 < rom.size(); at += 2) {
        rom[at] = 0x4E;
        rom[at + 1] = 0x71;                // NOP
    }
    return rom;
}

std::vector<u8> gcDeclarationRom() {
    std::vector<u8> rom(32u * 1024u, 0);
    const std::size_t format = rom.size() - 20u;
    put32(rom, static_cast<u32>(format), 0x00FFA014u);
    put32(rom, static_cast<u32>(format + 4), 0x00006000u);
    rom[format + 12] = 1;
    rom[format + 13] = 1;
    put32(rom, static_cast<u32>(format + 14), 0x5A932BC7u);
    rom[format + 18] = 0;
    rom[format + 19] = 0xE1;
    u32 crc = 0;
    for (std::size_t index = rom.size() - 0x6000u; index < rom.size(); ++index) {
        crc = (crc << 1) | (crc >> 31);
        if (index < format + 8 || index >= format + 12) crc += rom[index];
    }
    put32(rom, static_cast<u32>(format + 8), crc);
    return rom;
}

// Drive one host-to-device ADB command using the pulse widths emitted by the
// genuine IIfx ISM firmware. IifxAdbBus accepts the PIC's 1.9584 MHz cycle
// counter and converts it to its exact 2 MHz wire timebase.
u64 cyclesForBusTicks(u64 ticks) {
    return (ticks * 612u + 624u) / 625u;
}

void adbCommand(IifxAdbBus& bus, u64& cycles, u8 command) {
    const auto advance = [&](u64 ticks) {
        cycles += cyclesForBusTicks(ticks);
    };

    advance(100);
    bus.hostDrive(false, cycles);
    advance(1300);                              // attention: 650 us low
    bus.hostDrive(true, cycles);
    advance(100);                               // sync: 50 us high
    bus.hostDrive(false, cycles);
    for (int bit = 7; bit >= 0; --bit) {
        const bool one = (command & (1u << bit)) != 0;
        advance(one ? 70 : 130);                // low half
        bus.hostDrive(true, cycles);
        advance(one ? 130 : 70);                // high half
        bus.hostDrive(false, cycles);
    }
    advance(130);                               // stop low
    bus.hostDrive(true, cycles);
}

} // namespace

TEST_CASE("Am29000 DW sub-word loads right-justify and SB sign-extends") {
    // With CFG.DW set the processor aligns sub-word data itself: the
    // addressed byte/half-word lands right-justified in the destination,
    // sign-extended when SB=1 and zero-extended when SB=0, and an SB access
    // parks the Byte Pointer on the low lane (~BO in both bits) so the
    // EXBYTE/EXHW/INBYTE/INHW idiom still addresses the datum (Am29030
    // User's Manual 3.3.7.2 and the SB bit description; the DW-enabled
    // Am29000 revisions GCOS configures follow it).  GCOS is compiled for
    // this: ~1300 half-word loads carry SB=1 for signed shorts and SB=0 for
    // unsigned ones and use the value directly -- the region-row merge at
    // $9D7BEB88 compares a loaded half-word against the $7FFF sentinel.
    // The broadcast model ($7FFF7FFF) never matched and the merge ran off
    // the end of card memory.
    Am29000 cpu;
    const std::array<u32, 12> program{
        0x04010273u, // mtsrim CPS,$0173 (leave freeze mode; BP can latch)
        0x04000330u, // mtsrim CFG,$0030 (DW and fetched vectors)
        0x03014000u, // const  gr64,$0100
        0x16124140u, // load   0,$12,gr65,gr64   (signed half-word at +0)
        0x7D424100u, // exhw   gr66,gr65,0       (extract via latched BP)
        0x03014402u, // const  gr68,$0102
        0x16024344u, // load   0,$02,gr67,gr68   (unsigned half-word at +2)
        0x7D454300u, // exhw   gr69,gr67,0       (extract via latched BP)
        0x03014601u, // const  gr70,$0101
        0x16114746u, // load   0,$11,gr71,gr70   (signed byte at +1)
        0x16014846u, // load   0,$01,gr72,gr70   (unsigned byte at +1)
        0x70400101u, // nop
    };
    cpu.readInstruction = [&](u32 address, bool) {
        const std::size_t index = address / 4u;
        return index < program.size() ? program[index] : 0u;
    };
    cpu.readData = [](u32 address, bool inputOutput) {
        CHECK_FALSE(inputOutput);
        return (address & ~3u) == 0x100u ? 0x80817FFEu : 0u;
    };
    cpu.writeData = [](u32, u32, bool) {};

    REQUIRE(cpu.run(13) == 13);
    CHECK_FALSE(cpu.faulted());
    // SB=1 ($12): half-word $8081 at +0 arrives right-justified and
    // sign-extended; BP now points at the low lane, so EXHW re-extracts
    // the same half-word.
    CHECK(cpu.registerValue(65) == 0xFFFF8081u);
    CHECK(cpu.registerValue(66) == 0x00008081u);
    // SB=0 ($02): half-word $7FFE at +2 arrives zero-extended, and EXHW
    // through the still-parked BP yields it unchanged.
    CHECK(cpu.registerValue(67) == 0x00007FFEu);
    CHECK(cpu.registerValue(69) == 0x00007FFEu);
    // Bytes: $81 at +1 is $FFFFFF81 signed and $00000081 unsigned.
    CHECK(cpu.registerValue(71) == 0xFFFFFF81u);
    CHECK(cpu.registerValue(72) == 0x00000081u);
}

TEST_CASE("Am29000 DW sub-word stores replicate the low lane into the addressed lanes") {
    // With CFG.DW set a byte/half-word store replicates the LOW byte or
    // half-word of the source register into every lane and the write
    // enables strobe only the addressed lane(s) (Am29030 User's Manual
    // 3.3.7.2).  Compiled GCOS stores right-justified values with plain
    // STORE.half at any alignment; taking the register lane matching the
    // address wrote the empty high half to even offsets, and writing the
    // full word sprayed the other lanes over the neighbours.
    Am29000 cpu;
    const std::array<u32, 12> program{
        0x04010273u, // mtsrim CPS,$0173 (leave freeze mode)
        0x04000330u, // mtsrim CFG,$0030 (DW and fetched vectors)
        0x03014000u, // const  gr64,$0100
        0x03CC41DDu, // const  gr65,$CCDD
        0x02AA41BBu, // consth gr65,$AABB      (gr65 = $AABBCCDD)
        0x1E024140u, // store  0,$02,gr65,gr64 (half at +0: lanes 0-1)
        0x03014402u, // const  gr68,$0102
        0x1E024144u, // store  0,$02,gr65,gr68 (half at +2: lanes 2-3)
        0x03014501u, // const  gr69,$0101
        0x0311461Fu, // const  gr70,$111F
        0x1E014645u, // store  0,$01,gr70,gr69 (byte at +1)
        0x70400101u, // nop
    };
    cpu.readInstruction = [&](u32 address, bool) {
        const std::size_t index = address / 4u;
        return index < program.size() ? program[index] : 0u;
    };
    u32 memory = 0x11223344u;
    cpu.readData = [&](u32 address, bool) {
        return (address & ~3u) == 0x100u ? memory : 0u;
    };
    cpu.writeData = [&](u32 address, u32 value, bool) {
        REQUIRE((address & ~3u) == 0x100u);
        memory = value;
    };

    REQUIRE(cpu.run(13) == 13);
    CHECK_FALSE(cpu.faulted());
    // Both half stores wrote gr65's LOW half ($CCDD): +0 took lanes 0-1
    // and +2 took lanes 2-3.  The byte store wrote gr70's LOW byte ($1F)
    // to +1 without touching the neighbours.
    CHECK(memory == 0xCC1FCCDDu);
}

TEST_CASE("Am29000 DW importer idiom copies half-words at either alignment") {
    // GC QuickDraw's rect importer ($9D7F8A5C) copies half-words with
    // LOAD.half(SB) -> EXHW -> LOAD.half(SB) -> INHW -> STORE.half.  Under
    // the DW semantics the SB loads park BP on the low lane, EXHW/INHW
    // work on the right-justified datum, and the store replicates it into
    // the addressed lanes -- so the copy is exact for both the even (+0)
    // and odd (+2) half-word of a word.
    Am29000 cpu;
    const std::array<u32, 20> program{
        0x04010273u, // mtsrim CPS,$0173
        0x04000330u, // mtsrim CFG,$0030
        0x03014000u, // const  gr64,$0100 (source, +0)
        0x03024100u, // const  gr65,$0200 (dest, +0)
        0x16124240u, // load   0,$12,gr66,gr64
        0x7D424200u, // exhw   gr66,gr66,0
        0x16124341u, // load   0,$12,gr67,gr65
        0x78434342u, // inhw   gr67,gr67,gr66
        0x1E024341u, // store  0,$02,gr67,gr65
        0x03014402u, // const  gr68,$0102 (source, +2)
        0x03024502u, // const  gr69,$0202 (dest, +2)
        0x16124644u, // load   0,$12,gr70,gr68
        0x7D464600u, // exhw   gr70,gr70,0
        0x16124745u, // load   0,$12,gr71,gr69
        0x78474746u, // inhw   gr71,gr71,gr70
        0x1E024745u, // store  0,$02,gr71,gr69
        0x70400101u, // nop
        0x70400101u, // nop
        0x70400101u, // nop
        0x70400101u, // nop
    };
    cpu.readInstruction = [&](u32 address, bool) {
        const std::size_t index = address / 4u;
        return index < program.size() ? program[index] : 0u;
    };
    u32 source = 0x1234FEDCu;
    u32 dest = 0x00000000u;
    cpu.readData = [&](u32 address, bool) {
        if ((address & ~3u) == 0x100u) return source;
        if ((address & ~3u) == 0x200u) return dest;
        return 0u;
    };
    cpu.writeData = [&](u32 address, u32 value, bool) {
        REQUIRE((address & ~3u) == 0x200u);
        dest = value;
    };

    REQUIRE(cpu.run(17) == 17);
    CHECK_FALSE(cpu.faulted());
    CHECK(dest == 0x1234FEDCu);
}

TEST_CASE("Am29000 assert trap in a jump delay slot resumes at the target") {
    // GCOS returns with `JMPI lr0` and puts the register-stack FILL assert
    // (ASLEU V=41) in the delay slot.  When the assert traps, the handler
    // IRET must resume at the JUMP TARGET: the assert already completed and
    // the branch was already taken.  Resuming at the fall-through instead
    // silently discards the return jump, which sent the 24-bit GC activation
    // task into the scheduler exit instead of back up its call chain (R1).
    Am29000 cpu;
    const std::array<u32, 24> program{
        0x04010272u, // 00 mtsrim CPS,$0172 (freeze off, traps deliverable)
        0x04000330u, // 04 mtsrim CFG,$0030 (DW and fetched vectors)
        0x03004040u, // 08 const  gr64,$0040 (jump target)
        0x03004102u, // 0C const  gr65,2
        0x03004201u, // 10 const  gr66,1
        0xC0000040u, // 14 jmpi   gr64
        0x56414142u, // 18 asleu  V=$41,gr65,gr66 (delay slot; 2<=1 traps)
        0x030043BBu, // 1C const  gr67,$BB (fall-through: must NOT run)
        0x70400101u, // 20 nop
        0x70400101u, // 24 nop
        0x70400101u, // 28 nop
        0x70400101u, // 2C nop
        0x70400101u, // 30
        0x70400101u, // 34
        0x70400101u, // 38
        0x70400101u, // 3C
        0x030043AAu, // 40 const  gr67,$AA (branch target: must run)
        0x70400101u, // 44 nop
        0x70400101u, // 48
        0x70400101u, // 4C
        0x70400101u, // 50
        0x70400101u, // 54
        0x70400101u, // 58
        0x70400101u, // 5C
    };
    const std::array<u32, 2> handler{
        0x88000000u, // 80 iret
        0x70400101u, // 84 nop
    };
    cpu.readInstruction = [&](u32 address, bool) -> u32 {
        if (address >= 0x80u) {
            const std::size_t index = (address - 0x80u) / 4u;
            return index < handler.size() ? handler[index] : 0x70400101u;
        }
        const std::size_t index = address / 4u;
        return index < program.size() ? program[index] : 0x70400101u;
    };
    cpu.readVector = [](u32 address) -> u32 {
        // Vector 65 ($41) entry points at the IRET stub.
        return address == (0x41u << 2u) ? 0x80u : 0x70400101u;
    };
    cpu.readData = [](u32, bool) { return 0u; };
    cpu.writeData = [](u32, u32, bool) {};

    REQUIRE(cpu.run(40) == 40);
    CHECK_FALSE(cpu.faulted());
    // The fill assert fired and returned; execution must have continued at
    // the branch target, not the fall-through after the delay slot.
    CHECK(cpu.registerValue(67) == 0x000000AAu);
}

TEST_CASE("Am29000 execute-stage trap outranks the same cycle's fetch "
          "exception") {
    // A branch's delay slot holds a failing fill assert (ASLEU V=$41) while
    // the SAME cycle's fetch of the branch target raises its own exception.
    // The execute-stage trap belongs to the older instruction and must be
    // the one dispatched; the fetch simply re-issues after the handler
    // returns and re-derives its exception.  Dispatching the fetch's vector
    // instead silently DROPS the fill assert -- the lost window refill let
    // GCQD's sequencer read a stale physical register slot and CALLI the
    // $80000000 callback sentinel (the 8*24 GC depth-path card fault).
    //
    // Fetch-side exception here: the branch target names an RBP-protected
    // register, tripping fetch_decode's user-mode check.  The V41 handler
    // clears RBP, so with the correct dispatch order the refetched target
    // then executes; with the wrong order the V41 vector never arrives and
    // the RBP violation handler loops instead.
    Am29000 cpu;
    const std::array<u32, 24> program{
        0x04010272u, // 00 mtsrim CPS,$0172 (supervisor, traps deliverable)
        0x04000330u, // 04 mtsrim CFG,$0030 (fetched vectors)
        0x04000720u, // 08 mtsrim RBP,$0020 (protect gr80-95 from user mode)
        0x03004040u, // 0C const  gr64,$0040 (branch target)
        0x03004102u, // 10 const  gr65,2
        0x03004201u, // 14 const  gr66,1
        0x04010262u, // 18 mtsrim CPS,$0162 (drop to user mode)
        0xC0000040u, // 1C jmpi   gr64
        0x56414142u, // 20 asleu  V=$41,gr65,gr66 (delay slot; 2<=1 traps)
        0x030043BBu, // 24 const  gr67,$BB (fall-through: must NOT run)
        0x70400101u, // 28 nop
        0x70400101u, // 2C nop
        0x70400101u, // 30
        0x70400101u, // 34
        0x70400101u, // 38
        0x70400101u, // 3C
        0x15505000u, // 40 add    gr80,gr80,0 (RBP-protected at fetch)
        0x030046AAu, // 44 const  gr70,$AA (proof the target ran)
        0xA0000000u, // 48 jmp    . (park; falling into the stubs below
        0x70400101u, // 4C nop      would run a bare IRET with no context)
        0x70400101u, // 50
        0x70400101u, // 54
        0x70400101u, // 58
        0x70400101u, // 5C
    };
    const std::array<u32, 4> fillHandler{
        0x04000700u, // 80 mtsrim RBP,0 (unprotect, so the refetch passes)
        0x03004441u, // 84 const  gr68,$41 (V41 arrived)
        0x88000000u, // 88 iret
        0x70400101u, // 8C nop
    };
    const std::array<u32, 3> protectionHandler{
        0x03004505u, // A0 const  gr69,$05 (protection vector arrived)
        0x88000000u, // A4 iret
        0x70400101u, // A8 nop
    };
    std::vector<u32> fetched;
    cpu.readInstruction = [&](u32 address, bool) -> u32 {
        fetched.push_back(address);
        if (address >= 0xA0u) {
            const std::size_t index = (address - 0xA0u) / 4u;
            return index < protectionHandler.size() ? protectionHandler[index]
                                                    : 0x70400101u;
        }
        if (address >= 0x80u) {
            const std::size_t index = (address - 0x80u) / 4u;
            return index < fillHandler.size() ? fillHandler[index]
                                              : 0x70400101u;
        }
        const std::size_t index = address / 4u;
        return index < program.size() ? program[index] : 0x70400101u;
    };
    std::vector<u32> dispatched;
    cpu.readVector = [&](u32 address) -> u32 {
        dispatched.push_back(address);
        if (address == (0x41u << 2u)) return 0x80u;  // V41 fill assert
        if (address == (5u << 2u)) return 0xA0u;     // protection violation
        return 0x70400101u;
    };
    cpu.readData = [](u32, bool) { return 0u; };
    cpu.writeData = [](u32, u32, bool) {};

    REQUIRE(cpu.run(60) == 60);
    REQUIRE(dispatched.size() >= 1);
    // The FIRST dispatch must be the fill assert's vector, not the fetch's.
    CHECK(dispatched[0] == (0x41u << 2u));
    {
        std::string order;
        for (u32 address : dispatched) {
            char item[16];
            std::snprintf(item, sizeof item, "%X ", address);
            order += item;
        }
        std::string fetches;
        for (u32 address : fetched) {
            char item[16];
            std::snprintf(item, sizeof item, "%X ", address);
            fetches += item;
        }
        INFO("fetch stream: " << fetches);
        CHECK(order == "104 ");
    }
    CHECK_FALSE(cpu.faulted());
    // The fill assert's vector was dispatched (not dropped), the protection
    // handler was never needed (RBP cleared before the refetch), and the
    // branch target executed.
    CHECK(cpu.registerValue(68) == 0x00000041u);
    CHECK(cpu.registerValue(69) == 0x00000000u);
    CHECK(cpu.registerValue(70) == 0x000000AAu);
    CHECK(cpu.registerValue(67) != 0x000000BBu);
}

TEST_CASE("Am29000 data TLB miss retires the load and IRET restarts it from "
          "the channel registers") {
    // User's Manual Table 3-11: a Data TLB Miss reports PC1 = "next" with
    // the channel registers describing the access ("all").  The load
    // therefore RETIRES, the handler maps the page, and the interrupt
    // return re-issues the access from CHA/CHC (return step 5) -- the
    // register is written before the next instruction runs.  Here the load
    // sits in a jump's delay slot, the exact shape of GCOS's
    // `CALLI lr0,gr96 / LOAD lr2,[lr7]` argument load: leaving the channel
    // registers stale dropped the load, the callee ran with the previous
    // argument (pointer 1), and the 24-bit GC activation died in an
    // unaligned copy every round.
    Am29000 cpu;
    const std::array<u32, 12> program{
        0x04010233u, // 00 mtsrim CPS,$0233 (freeze off, PI physical, PD=0)
        0x04000330u, // 04 mtsrim CFG,$0030 (fetched vectors)
        0x04000D00u, // 08 mtsrim MMU,0 (1 KiB pages, PID 0)
        0x03104000u, // 0C const  gr64,$1000 (virtual data address)
        0xA0000004u, // 10 jmp    $0020
        0x16004240u, // 14 load   0,0,gr66,gr64 (delay slot: TLB miss)
        0x030043BBu, // 18 const  gr67,$BB (skipped by the jump)
        0x70400101u, // 1C nop
        0x030044AAu, // 20 const  gr68,$AA (target)
        0x15454201u, // 24 add    gr69,gr66,1 (needs the loaded value)
        0xA0000000u, // 28 jmp    .
        0x70400101u, // 2C nop
    };
    const std::array<u32, 10> handler{
        0xC6460400u, // 80 mfsr   gr70,CHA
        0xC64B0600u, // 84 mfsr   gr75,CHC
        0x03784700u, // 88 const  gr71,$7800 (word0: valid, supervisor RWX)
        0x03104800u, // 8C const  gr72,$1000 (word1: physical page)
        0x03004908u, // 90 const  gr73,8 (line 4, set 0, word0)
        0x03004A09u, // 94 const  gr74,9 (word1)
        0xBE004947u, // 98 mttlb  gr73,gr71
        0xBE004A48u, // 9C mttlb  gr74,gr72
        0x88000000u, // A0 iret
        0x70400101u, // A4 nop
    };
    cpu.readInstruction = [&](u32 address, bool) -> u32 {
        if (address >= 0x80u) {
            const std::size_t index = (address - 0x80u) / 4u;
            return index < handler.size() ? handler[index] : 0x70400101u;
        }
        const std::size_t index = address / 4u;
        return index < program.size() ? program[index] : 0x70400101u;
    };
    std::vector<u32> dispatched;
    cpu.readVector = [&](u32 address) -> u32 {
        dispatched.push_back(address);
        return address == (11u << 2u) ? 0x80u : 0x70400101u;
    };
    u32 dataReads = 0;
    cpu.readData = [&](u32 address, bool) -> u32 {
        if (address == 0x1000u) {
            ++dataReads;
            return 0x12345678u;
        }
        return 0u;
    };
    cpu.writeData = [](u32, u32, bool) {};

    REQUIRE(cpu.run(40) == 40);
    CHECK_FALSE(cpu.faulted());
    REQUIRE(dispatched.size() == 1);
    CHECK(dispatched[0] == (11u << 2u));
    // The handler saw the access in the channel registers: CHA = the
    // virtual address, CHC.CV set.
    CHECK(cpu.registerValue(70) == 0x00001000u);
    CHECK((cpu.registerValue(75) & 1u) == 1u);
    // The load completed on the interrupt return, before the target ran.
    CHECK(dataReads == 1u);
    CHECK(cpu.registerValue(66) == 0x12345678u);
    CHECK(cpu.registerValue(69) == 0x12345679u);
    CHECK(cpu.registerValue(68) == 0x000000AAu);
    CHECK(cpu.registerValue(67) != 0x000000BBu);
}

TEST_CASE("Am29000 *DERR retires the load; IRET resumes at the next "
          "instruction and restarts the access") {
    // Data Access Exception (Table 3-11): PC1 = "next", channel registers
    // "all", CHC.TF set.  The faulted load retires; the handler here clears
    // TF (as GCOS's does) and returns with CV set, so the return step 5
    // restart re-issues the access -- which now succeeds -- and execution
    // continues with the instruction AFTER the load, exactly once.
    Am29000 cpu;
    const std::array<u32, 12> program{
        0x04010273u, // 00 mtsrim CPS,$0273 (freeze off, physical)
        0x04000330u, // 04 mtsrim CFG,$0030 (fetched vectors)
        0x03104000u, // 08 const  gr64,$1000
        0x16004240u, // 0C load   0,0,gr66,gr64 (first access faults)
        0x03004301u, // 10 const  gr67,1 (must run exactly once)
        0x15454201u, // 14 add    gr69,gr66,1
        0x15434301u, // 18 add    gr67,gr67,1 (gr67 ends at 2 iff 10 ran once)
        0xA0000000u, // 1C jmp    .
        0x70400101u, // 20 nop
        0x70400101u, // 24
        0x70400101u, // 28
        0x70400101u, // 2C
    };
    const std::array<u32, 6> handler{
        0xC6460400u, // 80 mfsr   gr70,CHA
        0xC64B0600u, // 84 mfsr   gr75,CHC
        0x034C4F00u, // 88 const  gr76,$4C00... (unused filler)
        0x9C4B4B4Cu, // 8C andn   gr75,gr75,gr76 -> placeholder, replaced below
        0x88000000u, // 90 iret
        0x70400101u, // 94 nop
    };
    // Handler: read CHC, clear TF (bit 10) keeping CV, write it back, IRET.
    std::array<u32, 8> realHandler{
        0xC6460400u, // 80 mfsr   gr70,CHA
        0xC64B0600u, // 84 mfsr   gr75,CHC
        0x03044C00u, // 88 const  gr76,$0400 (TF)
        0x9C4D4B4Cu, // 8C andn   gr77,gr75,gr76
        0xCE00064Du, // 90 mtsr   CHC,gr77
        0x88000000u, // 94 iret
        0x70400101u, // 98 nop
        0x70400101u, // 9C nop
    };
    (void)handler;
    cpu.readInstruction = [&](u32 address, bool) -> u32 {
        if (address >= 0x80u) {
            const std::size_t index = (address - 0x80u) / 4u;
            return index < realHandler.size() ? realHandler[index]
                                              : 0x70400101u;
        }
        const std::size_t index = address / 4u;
        return index < program.size() ? program[index] : 0x70400101u;
    };
    std::vector<u32> dispatched;
    cpu.readVector = [&](u32 address) -> u32 {
        dispatched.push_back(address);
        return address == (7u << 2u) ? 0x80u : 0x70400101u;
    };
    u32 dataReads = 0;
    cpu.readData = [&](u32 address, bool) -> u32 {
        if (address == 0x1000u) {
            if (++dataReads == 1u) {
                cpu.noteDataBusFault(); // absent memory: *DERR
                return 0u;
            }
            return 0x12345678u;
        }
        return 0u;
    };
    cpu.writeData = [](u32, u32, bool) {};

    REQUIRE(cpu.run(40) == 40);
    CHECK_FALSE(cpu.faulted());
    REQUIRE(dispatched.size() == 1);
    CHECK(dispatched[0] == (7u << 2u));
    CHECK(cpu.registerValue(70) == 0x00001000u);
    CHECK((cpu.registerValue(75) & 0x401u) == 0x401u); // TF|CV in handler
    CHECK(dataReads == 2u);
    CHECK(cpu.registerValue(66) == 0x12345678u);
    CHECK(cpu.registerValue(69) == 0x12345679u);
    // The instruction after the load ran exactly once (not skipped, not
    // repeated).
    CHECK(cpu.registerValue(67) == 0x00000002u);
}

TEST_CASE("Am29000 IRET refill completes before a pending timer interrupt") {
    // IRET's two-instruction refill executes with external/timer
    // recognition held off, as on silicon.  Without the hold, a persistent
    // timer request (IN outlives a deferred delivery) preempts the refill
    // at its first boundary forever: GCOS's deferred tick never lets the
    // replayed instruction retire.
    // The handler is GCOS's defer shape: a bare IRET that leaves TMR.IN
    // set.  The request is therefore still pending the moment the return
    // completes.  Without the refill hold the very first interrupted
    // instruction is preempted again forever and never retires; with it,
    // two instructions retire per delivery and the program advances.
    Am29000 cpu;
    const std::array<u32, 12> program{
        0x04010273u, // 00 mtsrim CPS,$0173 (freeze off, DA holds interrupts)
        0x04000330u, // 04 mtsrim CFG,$0030 (fetched vectors)
        0x03754030u, // 08 const  gr64,$7530
        0x02034000u, // 0C consth gr64,$0300  (gr64 = IN|IE|TRV)
        0xCE000940u, // 10 mtsr   TMR,gr64    (timer request pending)
        0xC6420200u, // 14 mfsr   gr66,CPS
        0x9D424201u, // 18 andn   gr66,gr66,1
        0xCE000242u, // 1C mtsr   CPS,gr66    (DA clear: delivery begins)
        0x03004101u, // 20 const  gr65,1
        0x03004202u, // 24 const  gr66,2
        0x14444142u, // 28 add    gr68,gr65,gr66
        0xA0000000u, // 2C jmp .
    };
    cpu.readInstruction = [&](u32 address, bool) -> u32 {
        if (address == 0x80u) return 0x88000000u; // handler: bare iret
        const std::size_t index = address / 4u;
        return index < program.size() ? program[index] : 0x70400101u;
    };
    cpu.readVector = [](u32 address) -> u32 {
        // Vector 14 (timer) lands on the deferring handler.
        return address == (14u << 2u) ? 0x80u : 0x70400101u;
    };
    cpu.readData = [](u32, bool) { return 0u; };
    cpu.writeData = [](u32, u32, bool) {};

    REQUIRE(cpu.run(120) == 120);
    CHECK_FALSE(cpu.faulted());
    // Positive control: the persistent request delivered repeatedly.
    CHECK(cpu.diagnosticTimerInterrupts() >= 2u);
    // The program made forward progress between deliveries.
    CHECK(cpu.registerValue(65) == 0x00000001u);
    CHECK(cpu.registerValue(66) == 0x00000002u);
    CHECK(cpu.registerValue(68) == 0x00000003u);
}

TEST_CASE("Am29000 preserves the load-store address-space signal") {
    Am29000 cpu;
    const std::array<u32, 5> program{
        0x03014000u, // const  gr64,$0100
        0x16004140u, // load   0,$00,gr65,gr64 (AS=0 memory)
        0x16404240u, // load   0,$40,gr66,gr64 (AS=1 input/output)
        0x1E004140u, // store  0,$00,gr65,gr64 (AS=0 memory)
        0x1E404240u, // store  0,$40,gr66,gr64 (AS=1 input/output)
    };
    cpu.readInstruction = [&](u32 address, bool) {
        const std::size_t index = address / 4u;
        return index < program.size() ? program[index] : 0u;
    };
    std::array<int, 2> reads{};
    std::array<int, 2> writes{};
    cpu.readData = [&](u32 address, bool inputOutput) {
        CHECK(address == 0x100u);
        ++reads[inputOutput ? 1u : 0u];
        return inputOutput ? 0x22222222u : 0x11111111u;
    };
    cpu.writeData = [&](u32 address, u32 value, bool inputOutput) {
        CHECK(address == 0x100u);
        CHECK(value == (inputOutput ? 0x22222222u : 0x11111111u));
        ++writes[inputOutput ? 1u : 0u];
    };

    REQUIRE(cpu.run(6) == 6);
    CHECK_FALSE(cpu.faulted());
    CHECK(cpu.registerValue(65) == 0x11111111u);
    CHECK(cpu.registerValue(66) == 0x22222222u);
    CHECK(reads == std::array<int, 2>{1, 1});
    CHECK(writes == std::array<int, 2>{1, 1});
}

TEST_CASE("IIfx original ASC exposes its RAM, live voices, volume, and clocks") {
    Asc asc;
    CHECK(asc.read(0x800) == 0x00);
    CHECK(asc.read(0xF09) == 0x00);
    CHECK(asc.read(0xF29) == 0x00);
    CHECK(asc.read(0x1000) == 0xFF);

    // RAM is directly addressed whenever the chip is not in FIFO mode. This
    // is the path the IIfx ROM uses to construct its startup-chime tables.
    asc.write(0x123, 0xA5);
    asc.write(0x456, 0x5A);
    CHECK(asc.read(0x123) == 0xA5);
    CHECK(asc.read(0x456) == 0x5A);
    CHECK(asc.ramWrites() == 2);

    asc.write(0x801, 0xFE);
    CHECK(asc.mode() == 2);
    asc.write(0x802, 0x02);
    CHECK(asc.stereo());
    asc.write(0x805, 0xFF);
    CHECK(asc.wavetableControl() == 0x0F);
    asc.write(0x806, 0xE0);
    CHECK(asc.volumeLevel() == 7);

    asc.write(0x807, 0);
    CHECK(asc.sampleRate() == Asc::kClassicRate);
    asc.write(0x807, 2);
    CHECK(asc.sampleRate() == 22050);
    asc.write(0x807, 3);
    CHECK(asc.sampleRate() == 44100);

    // Each exposed value is a big-endian 24-bit register with a dead high
    // byte, and reads reflect the live phase accumulator.
    asc.write(0x810, 0xFF);
    asc.write(0x811, 0x12);
    asc.write(0x812, 0x34);
    asc.write(0x813, 0x56);
    asc.write(0x815, 0x01);
    asc.write(0x816, 0x80);
    asc.write(0x817, 0x00);
    CHECK(asc.phase(0) == 0x123456);
    CHECK(asc.increment(0) == 0x018000);
    CHECK(asc.read(0x810) == 0);
    CHECK(asc.read(0x811) == 0x12);
    CHECK(asc.read(0x812) == 0x34);
    CHECK(asc.read(0x813) == 0x56);

    bool irq = false;
    asc.onIrq = [&](bool level) { irq = level; };
    asc.write(0x804, 0x0F);
    CHECK(irq);
    CHECK(asc.read(0x804) == 0x0F);
    CHECK_FALSE(irq);
    CHECK(asc.read(0x804) == 0);
}

TEST_CASE("IIfx ASC mixes four wavetable voices into mono and stereo paths") {
    Asc asc;
    constexpr u8 tableValues[4] = {0x10, 0x20, 0x30, 0x40};
    for (u32 voice = 0; voice < 4; ++voice)
        asc.write(voice * Asc::kTableBytes, tableValues[voice]);
    asc.write(0x806, 0xE0);                    // full Sony output level
    asc.write(0x801, 2);                       // four-voice synthesis

    const Asc::StereoSample mono = asc.pullStereoSample();
    CHECK(mono.left == 0xA0);                  // raw sum of all four voices
    CHECK(mono.right == mono.left);

    asc.write(0x802, 0x02);                    // two voices per channel
    const Asc::StereoSample stereo = asc.pullStereoSample();
    CHECK(stereo.left == 0x30);                // $10 + $20
    CHECK(stereo.right == 0x70);               // $30 + $40

    // Level zero is a true mute, while the other seven settings increase
    // monotonically toward the unattenuated digital result.
    asc.write(0x802, 0);
    u8 previous = 0x80;
    for (u8 level = 0; level < 8; ++level) {
        asc.write(0x806, static_cast<u8>(level << 5));
        const u8 sample = asc.pullSample();
        CHECK(sample >= previous);
        previous = sample;
    }
    CHECK(previous == 0xA0);

    // The accumulator advances before lookup and uses bits 23..15 as the
    // 512-byte table index.
    asc.reset();
    for (u32 voice = 0; voice < 4; ++voice)
        asc.write(voice * Asc::kTableBytes, 0x20);
    asc.write(1, 0x40);
    asc.write(0x815, 0x00);
    asc.write(0x816, 0x80);
    asc.write(0x817, 0x00);
    asc.write(0x806, 0xE0);
    asc.write(0x801, 2);
    CHECK(asc.pullSample() == 0xA0);
    CHECK(asc.phase(0) == 0x008000);
}

TEST_CASE("IIfx ASC FIFO status and IRQ transitions follow original hardware") {
    Asc asc;
    bool irq = false;
    int transitions = 0;
    asc.onIrq = [&](bool level) {
        irq = level;
        ++transitions;
    };
    asc.write(0x806, 0xE0);
    asc.write(0x801, 1);
    asc.write(0x000, 0x90);
    asc.write(0x400, 0x20);                    // ignored while mono
    CHECK(asc.fifoLevel(0) == 1);
    CHECK(asc.fifoLevel(1) == 0);
    const Asc::StereoSample held = asc.pullStereoSample();
    CHECK(held.left == 0x90);
    CHECK(held.right == 0x90);
    CHECK(irq);
    CHECK((asc.read(0x804) & 0x02u) != 0);     // A became empty
    CHECK_FALSE(irq);
    CHECK(asc.pullSample() == 0x90);           // underrun holds last byte

    asc.write(0x802, 0x02);
    asc.write(0x000, 0xA0);
    asc.write(0x400, 0x60);
    CHECK(asc.fifoLevel(0) == 1);
    CHECK(asc.fifoLevel(1) == 1);
    const Asc::StereoSample stereo = asc.pullStereoSample();
    CHECK(stereo.left == 0xA0);
    CHECK(stereo.right == 0x60);
    CHECK((asc.read(0x804) & 0x0Au) == 0x0A);

    for (u32 index = 0; index < 513; ++index)
        asc.write(index & 0x3FFu, static_cast<u8>(index));
    asc.pullSample();
    asc.pullSample();
    CHECK((asc.read(0x804) & 0x01u) != 0);     // crossed below half full

    asc.write(0x803, 0x80);
    CHECK(asc.fifoLevel(0) == 0);
    CHECK(asc.fifoLevel(1) == 0);
    CHECK(asc.read(0x804) == 0x0A);            // both FIFOs marked empty
    CHECK(transitions >= 4);
}

TEST_CASE("IIfx active ASC audio survives a full-machine checkpoint replay") {
    constexpr u32 ascBase = 0x50010000u;
    IifxMachine original(testRom());
    (void)original.read16(0x40000100u);

    // A changing first voice plus three baseline voices gives a small,
    // deterministic ROM-style synthesized waveform on the internal speaker.
    for (u32 index = 0; index < Asc::kTableBytes; ++index) {
        original.write8(ascBase + index, static_cast<u8>(index & 0x3Fu));
        for (u32 voice = 1; voice < 4; ++voice)
            original.write8(ascBase + voice * Asc::kTableBytes + index, 0x20);
    }
    original.write8(ascBase + 0x815, 0x00);
    original.write8(ascBase + 0x816, 0x80);
    original.write8(ascBase + 0x817, 0x00);
    original.write8(ascBase + 0x806, 0xE0);
    original.write8(ascBase + 0x801, 0x02);
    original.runFrame();
    std::vector<u8> warmup;
    original.drainAudio(warmup);
    REQUIRE_FALSE(warmup.empty());
    CHECK(std::any_of(warmup.begin(), warmup.end(),
                      [](u8 sample) { return sample != 0x80; }));

    const std::vector<u8> checkpoint = original.saveState();
    IifxMachine replay(testRom());
    std::string error;
    REQUIRE_MESSAGE(replay.loadState(checkpoint.data(), checkpoint.size(), &error),
                    error);
    CHECK(replay.saveState() == checkpoint);

    original.runFrame();
    replay.runFrame();
    std::vector<u8> expected;
    std::vector<u8> actual;
    original.drainAudio(expected);
    replay.drainAudio(actual);
    CHECK(actual == expected);
    CHECK(std::any_of(actual.begin(), actual.end(),
                      [](u8 sample) { return sample != 0x80; }));
    CHECK(replay.saveState() == original.saveState());
}

TEST_CASE("IIfx battery-backed PRAM survives reset and advances while off") {
    constexpr std::size_t blobBytes = 268;
    std::array<u8, blobBytes> saved{};
    saved[0] = 'P';
    saved[1] = 'R';
    saved[2] = 'A';
    saved[3] = 'M';
    saved[4] = 1;
    for (u32 index = 0; index < 256; ++index)
        saved[8 + index] = static_cast<u8>((index * 37u + 0x5Au) & 0xFFu);
    saved[264] = 0x12;
    saved[265] = 0x34;
    saved[266] = 0x56;
    saved[267] = 0x78;

    IifxMachine mac(testRom());
    CHECK_FALSE(mac.loadPram(saved.data(), blobBytes - 1u, 0));
    REQUIRE(mac.loadPram(saved.data(), blobBytes, 37));

    // A guest RESET resets the RTC serial state, not its battery-backed data.
    mac.reset();
    CHECK(mac.savePram(nullptr, 0) == blobBytes);
    std::array<u8, blobBytes> restored{};
    REQUIRE(mac.savePram(restored.data(), static_cast<u32>(restored.size())) ==
            blobBytes);
    CHECK(std::equal(saved.begin() + 8, saved.begin() + 264,
                     restored.begin() + 8));
    CHECK(restored[264] == 0x12);
    CHECK(restored[265] == 0x34);
    CHECK(restored[266] == 0x56);
    CHECK(restored[267] == 0x9D);             // $12345678 + 37 seconds

    IifxMachine relaunched(testRom());
    REQUIRE(relaunched.loadPram(restored.data(),
                                static_cast<u32>(restored.size())));
    std::array<u8, blobBytes> secondSave{};
    REQUIRE(relaunched.savePram(secondSave.data(),
                                static_cast<u32>(secondSave.size())) ==
            blobBytes);
    CHECK(secondSave == restored);
}

TEST_CASE("IIfx reset overlay gives way to native ROM and RAM") {
    IifxMachine mac(testRom());
    CHECK(mac.overlayActive());
    CHECK(mac.cpu().pc == 0x40000100);
    CHECK(mac.read32(0) == 0x00100000);

    CHECK(mac.read16(0x40000100) == 0x4E71);
    CHECK_FALSE(mac.overlayActive());
    mac.write32(0, 0x12345678);
    CHECK(mac.read32(0) == 0x12345678);
}

TEST_CASE("IIfx FMC cache fills only on burst, wraps, retries and snoops writes") {
    IifxMachine mac(testRom());
    (void)mac.read16(0x40000100);              // release reset overlay
    (void)mac.takeCyclePenalty();
    mac.write32(0x1000, 0x00112233);
    mac.write32(0x1004, 0x44556677);
    mac.write32(0x1008, 0x8899AABB);
    mac.write32(0x100C, 0xCCDDEEFF);

    u32 words[4]{};
    CHECK(mac.readBurst32(0x1008, words));     // miss: FMC acknowledges/fills
    CHECK(words[0] == 0x8899AABB);
    CHECK(words[1] == 0xCCDDEEFF);
    CHECK(words[2] == 0x00112233);             // modulo-16 wrap
    CHECK(words[3] == 0x44556677);
    CHECK(mac.takeCyclePenalty() == 4);        // documented retry penalty

    CHECK_FALSE(mac.readBurst32(0x1008, words)); // hit: no CBACK/burst
    CHECK(words[0] == 0x8899AABB);
    CHECK(mac.takeCyclePenalty() == 0);

    // Every write master updates a resident FMC block.
    mac.write8(0x1009, 0x12);
    CHECK(mac.read32(0x1008) == 0x8812AABB);
    CHECK(mac.takeCyclePenalty() == 0);

    // A non-burst miss is retried but deliberately does not allocate.
    mac.write32(0x2000, 0xDEADBEEF);
    CHECK(mac.read32(0x2000) == 0xDEADBEEF);
    CHECK(mac.takeCyclePenalty() == 4);
    CHECK(mac.readBurst32(0x2000, words));
    CHECK(words[0] == 0xDEADBEEF);
}

TEST_CASE("IIfx accepts only physical four-SIMM bank totals") {
    IifxMachine::Config config;
    config.ramSize = 20u * 1024u * 1024u;           // 16 MB bank + 4 MB bank
    CHECK_NOTHROW(IifxMachine(testRom(), config));
    config.ramSize = 12u * 1024u * 1024u;           // no valid 64-pin SIMM layout
    CHECK_THROWS_AS(IifxMachine(testRom(), config), std::invalid_argument);
}

TEST_CASE("IIfx VIA identity and 256 KiB I/O mirrors") {
    IifxMachine mac(testRom());
    (void)mac.takeCyclePenalty();             // discard reset-vector traffic
    CHECK(mac.read8(0x50000200) == 0xD3);   // VIA1 ORA, original select
    // The 40 MHz CPU must wait for the 783.36 kHz VIA bus. BOOTBEEP relies on
    // this exact first-access delay to pace the startup-chime decay loop.
    CHECK(mac.takeCyclePenalty() == 77);
    CHECK(mac.read8(0x503C0200) == 0xD3);   // same select in a 256 KiB mirror

    // A stock IIfx has no optional RAM Parity Unit.  When software changes
    // the VIA-B direction to probe the hardware, unused PB3..7 must read low;
    // presenting them as floating-high makes System 7 diagnose a broken RPU.
    CHECK((mac.read8(0x50000000) & 0xF8u) == 0);
    CHECK((mac.read8(0x503C0000) & 0xF8u) == 0);

    // Stock machines have no optional RPU/OSS expansion device. The socket
    // selects are decoded, but open-bus reads stay high and writes do not
    // stick; writable readback falsely passes the ROM's presence probe.
    CHECK(mac.read8(0x5001E000) == 0xFF);
    CHECK_NOTHROW(mac.write8(0x5001E000, 0x00));
    CHECK(mac.read8(0x5001E000) == 0xFF);
}

TEST_CASE("IIfx OSS latches and acknowledges the 60.15 Hz source") {
    IifxMachine mac(testRom());
    mac.write8(0x5001A00A, 4);              // source 10 -> IPL4
    mac.runFrame();
    CHECK((mac.read16(0x5001A202) & 0x0400) != 0);
    mac.write8(0x5001A207, 0);
    CHECK((mac.read16(0x5001A202) & 0x0400) == 0);
}

TEST_CASE("IIfx IOP exposes autoincrement RAM, alive and interrupt ack") {
    IifxMachine mac(testRom());
    constexpr u32 base = 0x50F04000;

    const auto setAddress = [&](u16 address) {
        mac.write8(base + 0, static_cast<u8>(address >> 8));
        mac.write8(base + 2, static_cast<u8>(address));
    };
    const auto upload = [&](u16 address, const u8* bytes, std::size_t count) {
        setAddress(address);
        mac.write8(base + 4, 0x02);         // held in reset, AUTOINC on
        for (std::size_t index = 0; index < count; ++index)
            mac.write8(base + 8, bytes[index]);
    };

    // A tiny downloaded R65C02 program publishes the same life byte and then
    // requests host interrupt zero through the PIC's internal $F035 register.
    // This tests the real low-level core and shared-memory mirrors rather than
    // asking a host-side mailbox surrogate to synthesize the handshake.
    const u8 firmware[] = {
        0xA2, 0xFF, 0x9A,                   // LDX #$FF / TXS
        0xA9, 0x06, 0x8D, 0x00, 0x02,     // send channel max
        0x8D, 0x00, 0x03,                   // receive channel max
        0xA9, 0xFF, 0x8D, 0x1F, 0x03,     // alive = $FF
        0xA9, 0x04, 0x8D, 0x35, 0xF0,     // INTHST0
        0x80, 0xFE,                         // BRA *
    };
    const u8 vectors[] = {0x00, 0x04, 0x00, 0x04, 0x00, 0x04};
    upload(0x0400, firmware, sizeof firmware);
    upload(0x7FFA, vectors, sizeof vectors);

    mac.write8(base + 4, 0x06);             // RUN | AUTOINC
    setAddress(0x1234);
    mac.write8(base + 8, 0xA5);
    mac.write8(base + 8, 0x5A);

    setAddress(0x1234);
    CHECK(mac.read8(base + 8) == 0xA5);
    CHECK(mac.read8(base + 8) == 0x5A);

    // The ROM's presence test uses TST.L on the 8-bit data port. Dynamic bus
    // sizing emits four adjacent byte cycles, all four of which alias the port
    // and advance its 16-bit address.
    setAddress(0x2000);
    (void)mac.read32(base + 8);
    CHECK(mac.read16(base + 1) == 0x2004);

    for (int i = 0; i < 200; ++i) mac.stepInstruction();
    setAddress(0x031F);
    CHECK(mac.read8(base + 8) == 0xFF);

    // Alive is independent of the interrupt. The firmware's $F035 write set
    // INT0, which a one in the host status register acknowledges.
    CHECK((mac.read8(base + 4) & 0x10) != 0);
    mac.write8(base + 4, 0x16);              // acknowledge INT0, stay running
    CHECK((mac.read8(base + 4) & 0x10) == 0);
}

TEST_CASE("IIfx SCC transmit readiness paces PIC byte DMA") {
    Scc8530 scc;
    int requestChanges = 0;
    scc.onDmaRequestChange = [&] { ++requestChanges; };

    CHECK(scc.dmaRequest(2, false));        // channel B holding register empty
    CHECK_FALSE(scc.dmaRequest(2, true));  // no receive data on an empty wire
    CHECK_FALSE(scc.dmaRequest(0, false)); // control ports are not DMA targets

    scc.write(4, 0x55);                    // first byte enters shift register
    CHECK(scc.dmaRequest(2, false));        // holding register still available
    scc.write(4, 0xAA);                    // second byte fills the register
    CHECK_FALSE(scc.dmaRequest(2, false));
    CHECK(requestChanges == 1);

    scc.tick(1150);                        // first byte finishes on the wire
    CHECK(scc.dmaRequest(2, false));
    CHECK(requestChanges == 2);
}

TEST_CASE("IIfx NuBus video card has a slot ROM and framebuffer") {
    IifxMachine mac(testRom());

    CHECK(mac.read8(0xF9FFFFFFu) == 0x0F);       // all four byte lanes
    CHECK(mac.read32(0xF9FFFFFAu) == 0x5A932BC7u);

    // Mode $80 is 1 bpp. Bit 7 is the first pixel and Macintosh indexed
    // color zero is white, one is black.
    mac.write8(0xF9001000u, 0x80);
    std::vector<u32> pixels(640u * 480u);
    mac.renderScreen(pixels.data());
    CHECK(pixels[0] == 0xFF000000u);
    CHECK(pixels[1] == 0xFFFFFFFFu);
}

TEST_CASE("IIfx 8 24 GC maps lane-zero ROM and super-slot hardware") {
    IifxMachine::Config config;
    config.videoDeclarationRom = gcDeclarationRom();
    IifxMachine mac(testRom(), config);

    CHECK(mac.read8(0xF9FFFFFCu) == 0xE1);
    CHECK(mac.read8(0xF9FFFFFFu) == 0xFF);
    CHECK(mac.read8(0xF9FFFFE8u) == 0x5A);
    mac.write8(0x9C010000u, 0x80);
    CHECK(mac.read8(0x9C010000u) == 0x80);
}

TEST_CASE("IIfx 8 24 GC keeps standard-slot aliases in 24-bit addressing") {
    IifxMachine::Config config;
    config.ramSize = 16u * 1024u * 1024u;
    config.videoDeclarationRom = gcDeclarationRom();
    IifxMachine mac(testRom(), config);

    // With the MMU off, this populated low address is ordinary IIfx RAM.
    // Accessing the native ROM first removes the reset overlay.
    (void)mac.read8(0x40000000u);
    mac.write8(0x00901234u, 0x11u);
    CHECK(mac.read8(0x00901234u) == 0x11u);

    // A 68030 TC initial shift of eight discards the logical high byte. Slot
    // $9 must therefore claim the resulting $009xxxxx physical cycle ahead
    // of RAM, and FMC must treat it as an expansion rather than a cache fill.
    mac.cpu().tc = 0x80080000u;              // E | IS=8
    CHECK_FALSE(mac.cacheable(0x00901234u));
    CHECK(mac.read8(0x00901234u) == 0x00u);  // GC shared DRAM, not IIfx RAM
    mac.write8(0x00901234u, 0xA5u);
    CHECK(mac.read8(0x00901234u) == 0xA5u);
    CHECK(mac.read8(0xF9001234u) == 0xA5u);

    // The explicitly 24-bit-compatible Slot Manager form aliases the same
    // standard slot space when its tagged high byte reaches the machine bus.
    mac.write8(0xF9905678u, 0x5Au);
    CHECK(mac.read8(0xF9905678u) == 0x5Au);
    CHECK(mac.read8(0xF9005678u) == 0x5Au);

    // Leaving 24-bit mode exposes the original RAM byte again.
    mac.cpu().tc = 0;
    CHECK(mac.read8(0x00901234u) == 0x11u);
}

TEST_CASE("IIfx 8 24 GC separates instruction SRAM, data DRAM, and VRAM") {
    IifxMachine::Config config;
    config.videoDeclarationRom = gcDeclarationRom();
    IifxMachine mac(testRom(), config);

    mac.write32(0x90000000u, 0x89ABCDEFu);
    mac.write32(0x9000FFFCu, 0x01234567u);
    mac.write32(0x90010000u, 0x76543210u);
    CHECK(mac.read32(0x90000000u) == 0x89ABCDEFu);
    CHECK(mac.read32(0x9000FFFCu) == 0x01234567u);
    CHECK(mac.read32(0x90010000u) == 0u);
    CHECK(mac.read32(0x92000000u) == 0u);
    CHECK(mac.read32(0xF9000000u) == 0u);
    mac.write32(0x92000004u, 0x10203040u);
    CHECK(mac.read32(0x92000004u) == 0x10203040u);
    CHECK(mac.read32(0x9C000004u) == 0x10203040u);
    CHECK(mac.read32(0x90000004u) == 0u);
    CHECK(mac.read32(0x9C010004u) == 0xAAAAAAAAu);
    mac.write32(0xF9000008u, 0x50607080u);
    CHECK(mac.read32(0x90000008u) == 0u);
    CHECK(mac.read32(0x92000008u) == 0x50607080u);
    CHECK(mac.read32(0x9C000008u) == 0x50607080u);
    mac.write32(0x9C01000Cu, 0xA1B2C3D4u);
    CHECK(mac.read32(0x9C01000Cu) == 0xA1B2C3D4u);
    CHECK(mac.read32(0x9200000Cu) == 0u);
    mac.write32(0x9D100000u, 0x11223344u);
    mac.write32(0x9D500000u, 0x55667788u);
    CHECK(mac.read32(0x9D100000u) == 0x11223344u);
    CHECK(mac.read32(0x9D500000u) == 0x55667788u);

    IifxNuBusVideo card(gcDeclarationRom());
    CHECK(card.gcDiagnosticReadProcessorData(0x4400000Cu) == 0u);
    CHECK(card.gcDiagnosticReadProcessorData(0x44000010u) == 0xFFFFFFFFu);
    CHECK(card.gcDiagnosticReadProcessorData(0x44000014u) == 0xFFFFFFFFu);
    CHECK(card.gcDiagnosticReadProcessorData(0x44000018u) == 0u);
    card.gcDiagnosticWriteProcessorData(0x9C008C00u, 0x01010000u);
    CHECK(card.gcDiagnosticDramWord(0x00008C00u) == 0x01010000u);
    CHECK(card.gcDiagnosticReadProcessorData(0x9C008C00u) == 0x01010000u);
    card.gcDiagnosticWriteProcessorData(0x9C010000u, 0xC35AA53Cu);
    CHECK(card.gcDiagnosticVramWord(0x00010000u) == 0xC35AA53Cu);
    CHECK(card.gcDiagnosticReadProcessorData(0x9C010000u) == 0xC35AA53Cu);
    card.gcDiagnosticWriteProcessorData(0x41010004u, 0x5AC33CA5u);
    CHECK(card.gcDiagnosticVramWord(0x00010004u) == 0x5AC33CA5u);
    CHECK(card.gcDiagnosticReadProcessorData(0x41010004u) == 0x5AC33CA5u);
    // AC842's RAMDAC is visible directly on the Am29000 bus.  GCOS writes the
    // palette index in the low byte, streams RGB in the high byte, then reads
    // the same three components back in bits 31..24 during its hardware test.
    const u64 unknownBeforeRamdac = card.gcUnknownDataAccesses();
    card.gcDiagnosticWriteProcessorData(0x46C00000u, 0x000000FEu);
    card.gcDiagnosticWriteProcessorData(0x46C00004u, 0x12000000u);
    card.gcDiagnosticWriteProcessorData(0x46C00004u, 0x34000000u);
    card.gcDiagnosticWriteProcessorData(0x46C00004u, 0x56000000u);
    card.gcDiagnosticWriteProcessorData(0x46C00000u, 0x000000FEu);
    CHECK(card.gcDiagnosticReadProcessorData(0x46C00004u) == 0x12000000u);
    CHECK(card.gcDiagnosticReadProcessorData(0x46C00004u) == 0x34000000u);
    CHECK(card.gcDiagnosticReadProcessorData(0x46C00004u) == 0x56000000u);
    // PBCTRL echoes the programmed control byte in the low lane: the depth
    // re-config derives mode geometry from a read-back, and the 24-bit
    // flow polls it for the depth to take.
    card.gcDiagnosticWriteProcessorData(0x46C00008u, 0x0000009Cu);
    CHECK(card.gcDiagnosticReadProcessorData(0x46C00008u) == 0x9Cu);
    CHECK(card.mode() == 0x84u);
    CHECK(card.gcUnknownDataAccesses() == unknownBeforeRamdac);
    card.gcDiagnosticWriteProcessorData(0x9D100000u, 0xA5C33C5Au);
    CHECK(card.gcDiagnosticExpansionDramWord(0x00100000u) == 0xA5C33C5Au);
    card.gcDiagnosticWriteProcessorData(0xF99092D8u, 0xDEADBEEFu);
    CHECK(card.gcDiagnosticDramWord(0x000092D8u) == 0xDEADBEEFu);
    CHECK(card.gcDiagnosticReadProcessorData(0xF90092D8u) == 0xDEADBEEFu);
    u32 gcBusAddress = 0;
    u32 gcBusWriteAddress = 0;
    u32 gcBusWriteValue = 0;
    card.setGcBusMasterCallbacks([&](u32 address) {
        gcBusAddress = address;
        return 0x90010000u;
    }, [&](u32 address, u32 value) {
        gcBusWriteAddress = address;
        gcBusWriteValue = value;
    });
    CHECK(card.gcDiagnosticReadProcessorData(0xF087DBC0u, true) ==
          0x90010000u);
    CHECK(gcBusAddress == 0xF087DBC0u);
    CHECK(card.gcDiagnosticReadProcessorData(0x00002120u) == 0x90010000u);
    CHECK(gcBusAddress == 0x00002120u);
    // In 32-bit mode GCQD commonly imports temporary BitMaps and regions from
    // the top of a large IIfx RAM configuration.  These are ordinary NuBus
    // system-memory cycles too, not unmapped accelerator-local addresses.
    CHECK(card.gcDiagnosticReadProcessorData(0x03FFF3E8u) == 0x90010000u);
    CHECK(gcBusAddress == 0x03FFF3E8u);
    // Locked 24-bit Macintosh handles keep state flags in the high byte.
    // GCQD imports them with ordinary AS=0 loads; RDNC strips the tag.
    CHECK(card.gcDiagnosticReadProcessorData(0x80005098u) == 0x90010000u);
    CHECK(gcBusAddress == 0x00005098u);
    card.gcDiagnosticWriteProcessorData(0x9C008C20u, 0x00003B30u);
    card.gcDiagnosticWriteProcessorData(0x00003B30u, 0x1234ABCDu);
    CHECK(gcBusWriteAddress == 0x00003B30u);
    CHECK(gcBusWriteValue == 0x1234ABCDu);
    gcBusWriteAddress = 0;
    card.gcDiagnosticWriteProcessorData(0x0000B180u, 0x55AA33CCu);
    CHECK(card.gcDiagnosticVramWord(0x0000B180u) == 0xAAAAAAAAu);
    CHECK(gcBusWriteAddress == 0x0000B180u);
    CHECK(gcBusWriteValue == 0x55AA33CCu);
    card.gcDiagnosticWriteProcessorData(0x03FFF3ECu, 0xA55AC33Cu);
    CHECK(gcBusWriteAddress == 0x03FFF3ECu);
    CHECK(gcBusWriteValue == 0xA55AC33Cu);
    // AS is preserved by the CPU, but Dolphin does not use it as a chip
    // select: GCQD's ordinary AS=0 low addresses are Macintosh NuBus cycles.
    // Local VRAM is selected by the explicit $41000000 processor aperture.
    card.gcDiagnosticWriteProcessorData(0x0000B180u, 0xC33C5AA5u, true);
    CHECK(card.gcDiagnosticVramWord(0x0000B180u) == 0xAAAAAAAAu);
    CHECK(gcBusWriteAddress == 0x0000B180u);
    CHECK(gcBusWriteValue == 0xC33C5AA5u);
    card.gcDiagnosticWriteProcessorData(0x4100B180u, 0x09643204u);
    CHECK(card.gcDiagnosticVramWord(0x0000B180u) == 0x09643204u);

    const std::vector<u8> checkpoint = mac.saveState();
    mac.write32(0x90000000u, 0u);
    std::string error;
    REQUIRE_MESSAGE(mac.loadState(checkpoint.data(), checkpoint.size(), &error),
                    error);
    CHECK(mac.read32(0x90000000u) == 0x89ABCDEFu);
    CHECK(mac.read32(0x92000004u) == 0x10203040u);
    CHECK(mac.read32(0x9C01000Cu) == 0xA1B2C3D4u);
    CHECK(mac.read32(0x9D100000u) == 0x11223344u);
    CHECK(mac.read32(0x9D500000u) == 0x55667788u);

    mac.reset();
    CHECK(mac.read32(0x90000000u) == 0u);
    CHECK(mac.read32(0x92000004u) == 0u);
    CHECK(mac.read32(0x9C01000Cu) == 0xAAAAAAAAu);
    CHECK(mac.read32(0x9D100000u) == 0u);
    CHECK(mac.read32(0x9D500000u) == 0u);
}

TEST_CASE("IIfx 8 24 GC timing port crosses raster edges deterministically") {
    IifxNuBusVideo card(gcDeclarationRom());
    constexpr u32 timing = 0x04C00000u;

    CHECK(card.readSuper(timing) == 0x80);
    std::array<u8, 4> echo{};
    for (u32 lane = 0; lane < 4; ++lane)
        echo[lane] = card.readSuper(timing + lane);
    for (u32 lane = 0; lane < 4; ++lane)
        card.writeSuper(timing + lane, echo[lane]);
    CHECK(card.timingEchoWrites() == 1);
    CHECK(card.serialCommands() == 0);

    const u64 toBlank = (IifxNuBusVideo::kGcTimingActiveUnits +
                         IifxNuBusVideo::kGcTimingUnitsPerCpuCycle - 1u) /
                        IifxNuBusVideo::kGcTimingUnitsPerCpuCycle;
    card.tick(toBlank);
    CHECK(card.readSuper(timing) == 0x00);

    const u64 blankUnits = IifxNuBusVideo::kGcTimingFrameUnits -
                           IifxNuBusVideo::kGcTimingActiveUnits;
    const u64 throughBlank = (blankUnits +
                              IifxNuBusVideo::kGcTimingUnitsPerCpuCycle - 1u) /
                             IifxNuBusVideo::kGcTimingUnitsPerCpuCycle;
    card.tick(throughBlank);
    CHECK(card.readSuper(timing) == 0x80);

    // A write without a preceding timing-port read is genuine serial data.
    for (int bit = 11; bit >= 0; --bit) {
        card.writeSuper(timing, (3u & (1u << bit)) ? 0x80 : 0x00);
        card.writeSuper(timing + 1u, 0);
        card.writeSuper(timing + 2u, 0);
        card.writeSuper(timing + 3u, 0);
    }
    CHECK(card.serialCommands() == 1);
    CHECK(card.lastSerialCommand() == 3);
}

TEST_CASE("IIfx 8 24 GC reports a high resolution color monitor") {
    IifxNuBusVideo card(gcDeclarationRom());
    constexpr std::array<u32, 3> senseRegisters{
        0x04000044u, 0x04000048u, 0x0400004Cu};

    // The genuine declaration ROM uses BFEXTU {0:1} on each byte and builds
    // the direct monitor code from the extracted bit-7 values.
    u8 code = 0;
    for (u32 address : senseRegisters)
        code = static_cast<u8>((code << 1) | (card.readSuper(address) >> 7));
    CHECK(code == 6);                          // 640x480 AppleColor Hi-Res

    // Writes to +$08 are monitor/control traffic, never a display-base
    // program. The default framebuffer remains the $010000 page advertised
    // by the genuine card ROM.
    for (u8 value : {u8{0xA0}, u8{0x40}, u8{0x80}, u8{0x00}}) {
        card.writeSuper(0x04000008u, value);
        card.writeSuper(0x04000009u, 0);
        card.writeSuper(0x0400000Au, 0);
        card.writeSuper(0x0400000Bu, 0);
    }
    CHECK(card.baseRegister() == 0);
    CHECK(card.displayBase() == 0x00010000u);
    CHECK(card.displayStride() == 128u);
}

TEST_CASE("IIfx 8 24 GC selects and renders direct xRGB color") {
    IifxMachine::Config config;
    config.videoDeclarationRom = gcDeclarationRom();
    IifxMachine mac(testRom(), config);

    const auto serialRegister = [&](u32 address, u32 value, int bits) {
        for (int bit = bits - 1; bit >= 0; --bit)
            mac.write32(address,
                (value & (1u << bit)) != 0 ? 0x80000000u : 0u);
    };

    // Apple's authentic 341-0266 Dolphin ROM programs AC842 PBCTRL with $9C
    // for direct color. Its MFB then receives the host-aperture base and
    // stride serially: $2000 * 8 = $10000 and $140 * 8 = 2560 xRGB bytes.
    // The packed side of that aperture stores 3/4 as many physical bytes.
    mac.write32(0x96C00008u, 0x9C000000u);
    CHECK(mac.read32(0x96C00008u) == 0x9C000000u);
    serialRegister(0x94000000u, 0x02000u, 20);
    serialRegister(0x940000A0u, 0x00140u, 12);

    // QuickDraw's direct PixMap is xRGB, so a 640-pixel scanline is 2560
    // bytes. The card ignores X and emits the following three components.
    mac.write32(0x9C010000u, 0x00123456u);
    mac.write32(0x9C010004u, 0x00ABCDEFu);
    CHECK(mac.read32(0x9C010000u) == 0x00123456u);
    CHECK(mac.read32(0x9C010004u) == 0x00ABCDEFu);
    std::vector<u32> pixels(640u * 480u);
    mac.renderScreen(pixels.data());
    CHECK(pixels[0] == 0xFF123456u);
    CHECK(pixels[1] == 0xFFABCDEFu);
}

TEST_CASE("IIfx 8 24 GC uses low CLUT entries for indexed pixels") {
    IifxMachine::Config config;
    config.videoDeclarationRom = gcDeclarationRom();
    IifxMachine mac(testRom(), config);

    // The declaration ROM programs entry 0 as white and entry 1 as black.
    // RAMDAC registers occupy lane zero of each NuBus longword.
    mac.write32(0x96C00000u, 0x00000000u);
    mac.write32(0x96C00004u, 0xFF000000u);
    mac.write32(0x96C00004u, 0xFF000000u);
    mac.write32(0x96C00004u, 0xFF000000u);
    mac.write32(0x96C00004u, 0x00000000u);
    mac.write32(0x96C00004u, 0x00000000u);
    mac.write32(0x96C00004u, 0x00000000u);

    // Mode $80 selects 1 bpp and the first two pixels are indices 1 and 0.
    mac.write32(0x96C00008u, 0x80000000u);
    mac.write8(0x9C010000u, 0x80u);
    std::vector<u32> pixels(640u * 480u);
    mac.renderScreen(pixels.data());
    CHECK(pixels[0] == 0xFF000000u);
    CHECK(pixels[1] == 0xFFFFFFFFu);
}

TEST_CASE("IIfx video VBL is a level interrupt on OSS slot 9") {
    IifxMachine mac(testRom());
    mac.write8(0x5001A000u, 3);              // slot 9 -> IPL3
    mac.write32(0xF909013Cu, 0);             // fallback VBL enable
    mac.runFrame();
    CHECK((mac.read16(0x5001A202u) & 1u) != 0);
    mac.write32(0xF9090148u, 0);             // card interrupt acknowledge
    CHECK((mac.read16(0x5001A202u) & 1u) == 0);
}

TEST_CASE("IIfx queues input for the physical ADB firmware and clocks audio") {
    IifxMachine mac(testRom());
    (void)mac.read16(0x40000100);                 // release the reset overlay

    mac.keyEvent(0x00, true);                     // A key down
    mac.runFrame();
    mac.runFrame();
    mac.runFrame();
    const std::string report = mac.diagnosticReport();
    // This synthetic ROM deliberately does not download ISM firmware. The
    // key therefore remains at the device until a physical Talk transaction;
    // no host-side once-per-frame mailbox poll is allowed to consume it.
    CHECK(report.find("ADB wire commands=0") != std::string::npos);
    CHECK(report.find("pending=1") != std::string::npos);

    std::vector<u8> audio;
    mac.drainAudio(audio);
    // 22,254 Hz / 60.15 Hz is just under 370 samples per video frame.
    CHECK(audio.size() >= 1108);
    CHECK(audio.size() <= 1111);
}

TEST_CASE("IIfx physical ADB serializer answers firmware-width Talk pulses") {
    AdbTransceiver devices;
    devices.reset();
    IifxAdbBus bus(devices);
    u64 cycles = 0;

    adbCommand(bus, cycles, 0x2F);               // keyboard Talk register 3
    CHECK(bus.commands() == 1);
    CHECK(bus.lastCommand() == 0x2F);
    CHECK(bus.replies() == 1);
    CHECK(bus.replyBytes() == 2);
    CHECK(devices.kbdReg3() == 1);

    // The device starts its response after the documented turnaround and
    // drives a start bit on the same open-collector wire sampled by GPIN0.
    cycles += cyclesForBusTicks(100);
    CHECK(bus.readLine(cycles));
    cycles += cyclesForBusTicks(80);
    CHECK_FALSE(bus.readLine(cycles));
    cycles += cyclesForBusTicks(80);
    CHECK(bus.readLine(cycles));

    // A real key transition is removed only when the ISM firmware issues the
    // corresponding physical Talk R0 command.
    devices.injectKey(0x00, true);
    REQUIRE(devices.hasPendingEvent());
    cycles += cyclesForBusTicks(5000);            // finish the previous reply
    adbCommand(bus, cycles, 0x2C);                // keyboard Talk register 0
    CHECK(bus.commands() == 2);
    CHECK(bus.replies() == 2);
    CHECK(bus.replyBytes() == 4);
    CHECK(devices.kbdPolls() == 1);
    CHECK_FALSE(devices.hasPendingEvent());
}

TEST_CASE("ADB mouse drains large signed movement over bounded 7-bit reports") {
    AdbTransceiver devices;
    devices.reset();
    devices.injectMouse(150, -150, true);

    const auto signed7 = [](u8 value) {
        const int bits = value & 0x7F;
        return bits & 0x40 ? bits - 0x80 : bits;
    };
    int totalX = 0;
    int totalY = 0;
    int reports = 0;
    while (devices.hasPendingEvent()) {
        REQUIRE(reports < 4);
        devices.setState(0);
        devices.cpuShiftOut(0x3C);             // mouse Talk register 0
        REQUIRE(devices.responsePending());
        devices.setState(1);
        const u8 y = devices.cpuShiftIn();
        devices.setState(2);
        const u8 x = devices.cpuShiftIn();
        devices.setState(3);
        CHECK((y & 0x80u) == 0);               // active-low button is down
        CHECK((x & 0x80u) != 0);               // fixed mouse byte-1 marker
        totalX += signed7(x);
        totalY += signed7(y);
        ++reports;
    }

    CHECK(reports == 3);
    CHECK(totalX == 150);
    CHECK(totalY == -150);
    CHECK(devices.mouseReports() == 3);
}

TEST_CASE("IIfx wraps a bare HFS volume as a boot-visible SCSI disk") {
    IifxMachine mac(testRom());
    std::vector<u8> volume = hfs::formatVolume(2u * 1024u * 1024u, "IIfx HD");
    REQUIRE_FALSE(volume.empty());
    const std::vector<u8> original = volume;
    mac.insertHardDisk(std::move(volume));
    CHECK(mac.hardDiskPresent());
    CHECK(mac.hardDiskImage() == original);
}

TEST_CASE("IIfx installed SCSI driver transfers its HFS partition read and write data") {
    IifxMachine mac(testRom());
    (void)mac.read16(0x40000100);                 // release the reset overlay

    std::vector<u8> volume = hfs::formatVolume(2u * 1024u * 1024u, "IIfx HD");
    REQUIRE_FALSE(volume.empty());
    const std::vector<u8> original = volume;
    mac.insertHardDisk(std::move(volume));

    // Shape the structures the ROM creates after loading the disk's DRVR:
    // unit table -> handle -> DCE -> canonical .ScsiHD header. Macintosh SCSI
    // disks occupy unit 32 upward (refnum -33 downward).
    constexpr u32 unitTable = 0x1000, handle = 0x1100;
    constexpr u32 dce = 0x1200, driver = 0x1300;
    mac.write32(0x011C, unitTable);
    mac.write32(unitTable + 32 * 4, handle);       // unit 32 / refnum -33
    mac.write32(handle, dce);
    mac.write32(dce, driver);
    mac.write16(driver + 0x0A, 0x20);              // Prime
    mac.write16(driver + 0x0C, 0x30);              // Control
    mac.write16(driver + 0x0E, 0x40);              // Status
    mac.write8(driver + 0x12, 7);
    const char name[] = ".ScsiHD";
    for (u32 i = 0; i < 7; ++i) mac.write8(driver + 0x13 + i, name[i]);
    mac.runFrame();                                // discover the installed DRVR

    constexpr u32 pb = 0x2000, buffer = 0x3000, ioDone = 0x4000;
    mac.write32(0x08FC, ioDone);
    mac.write16(pb + 0x06, 2);                     // Read
    mac.write32(pb + 0x20, buffer);
    mac.write32(pb + 0x24, 512);
    mac.write16(pb + 0x2C, 1);                     // fsFromStart
    mac.write32(pb + 0x2E, 1024);                  // HFS MDB block
    mac.write32(dce + 0x10, 0);
    mac.cpu().a[0] = pb;
    mac.cpu().a[1] = dce;
    mac.cpu().pc = driver + 0x20;
    CHECK(mac.stepInstruction() == 40);
    CHECK(mac.cpu().pc == ioDone);
    CHECK(mac.read32(pb + 0x28) == 512);
    CHECK(mac.read8(buffer) == original[1024]);
    CHECK(mac.read8(buffer + 1) == original[1025]);

    for (u32 i = 0; i < 512; ++i)
        mac.write8(buffer + i, static_cast<u8>(i ^ 0xA5));
    mac.write16(pb + 0x06, 3);                     // Write
    mac.write32(pb + 0x2E, 4096);
    mac.write32(dce + 0x10, 0);
    mac.cpu().a[0] = pb;
    mac.cpu().a[1] = dce;
    mac.cpu().pc = driver + 0x20;
    CHECK(mac.stepInstruction() == 40);
    CHECK(mac.read32(pb + 0x28) == 512);
    const auto& written = mac.hardDiskImage();
    CHECK(written[4096] == 0xA5);
    CHECK(written[4096 + 255] == 0x5A);

    mac.write16(pb + 0x1A, 6);                     // Status: format list
    mac.cpu().a[0] = pb;
    mac.cpu().pc = driver + 0x40;
    CHECK(mac.stepInstruction() == 40);
    CHECK(mac.read16(pb + 0x1C) == 1);
    CHECK(mac.read32(pb + 0x1E) == original.size() / 512u);
}

TEST_CASE("IIfx SuperDrive accepts period media and serves the ROM Sony Prime") {
    IifxMachine mac(testRom());
    (void)mac.read16(0x40000100);                 // release the reset overlay

    std::vector<u8> disk(1474560);
    for (u32 i = 0; i < 1024; ++i) disk[i] = static_cast<u8>(i ^ 0x5A);
    REQUIRE(mac.insertFloppy(disk) == 1);
    CHECK(mac.floppyPresent());

    constexpr u32 pb = 0x1000, dce = 0x1100, buffer = 0x2000;
    mac.write16(pb + 0x06, 2);                    // Read request
    mac.write32(pb + 0x20, buffer);
    mac.write32(pb + 0x24, 512);
    mac.write16(pb + 0x2C, 1);                    // fsFromStart
    mac.write32(pb + 0x2E, 0);
    mac.write32(dce + 0x10, 0);
    mac.write32(0x08FC, 0x3000);                  // jIODone
    mac.cpu().a[0] = pb;
    mac.cpu().a[1] = dce;
    mac.cpu().pc = 0x4086C406;
    CHECK(mac.stepInstruction() == 40);
    CHECK(mac.cpu().pc == 0x3000);
    CHECK(mac.read32(pb + 0x28) == 512);
    for (u32 i = 0; i < 512; ++i)
        CHECK(mac.read8(buffer + i) == static_cast<u8>(i ^ 0x5A));

    // The IIfx ROM patches the hot .Sony path into the low-memory Disk Prime
    // vector at $0226.  Startup I/O calls that handler directly rather than
    // re-entering the tiny DRVR header stub, so both addresses must have the
    // same completion semantics.
    mac.write32(dce + 0x10, 512);
    mac.write32(pb + 0x24, 512);
    mac.write32(pb + 0x2E, 512);
    mac.cpu().a[0] = pb;
    mac.cpu().a[1] = dce;
    mac.cpu().pc = 0x0086C5AC;
    CHECK(mac.stepInstruction() == 40);
    CHECK(mac.cpu().pc == 0x3000);
    CHECK(mac.read8(buffer) == static_cast<u8>(512 ^ 0x5A));

    // The same path writes through to the medium and therefore to host
    // write-back, while a write-protected insertion refuses the write.
    for (u32 i = 0; i < 512; ++i) mac.write8(buffer + i, static_cast<u8>(i));
    mac.write16(pb + 0x06, 3);                    // Write request
    mac.write32(dce + 0x10, 0);
    mac.write32(pb + 0x2E, 0);
    mac.cpu().a[0] = pb;
    mac.cpu().a[1] = dce;
    mac.cpu().pc = 0x4086C406;
    mac.stepInstruction();
    CHECK(mac.floppyImage()[0] == 0);
    CHECK(mac.floppyImage()[255] == 0xFF);
    CHECK(mac.floppyForWriteBack().size() == 1474560);

    mac.ejectFloppy();
    CHECK_FALSE(mac.floppyPresent());
    CHECK(mac.floppyForWriteBack().size() == 1474560);
    CHECK(mac.insertFloppy(std::vector<u8>(12345)) == 0);
}

TEST_CASE("IIfx refused floppy swap preserves the mounted disk and its container") {
    IifxMachine mac(testRom());
    std::vector<u8> sectors(819200, 0x5A);
    dc42::Parts wrapper;
    wrapper.header.assign(dc42::kHeaderSize, 0);
    wrapper.header[0] = 6;
    const char name[] = "System";
    for (std::size_t i = 0; i < 6; ++i)
        wrapper.header[1 + i] = static_cast<u8>(name[i]);
    wrapper.header[82] = 0x01;
    wrapper.header[83] = 0x00;
    const std::vector<u8> diskCopy = dc42::rewrap(wrapper, sectors);

    REQUIRE(mac.insertFloppy(diskCopy) == 1);
    REQUIRE(mac.floppyForWriteBack() == diskCopy);
    CHECK(mac.insertFloppy(std::vector<u8>(12345, 0xCC)) == 0);
    CHECK(mac.floppyPresent());
    CHECK(mac.floppyForWriteBack() == diskCopy);
}

TEST_CASE("IIfx full-machine checkpoints replay cycle-for-cycle") {
    IifxMachine::Config config;
    config.ramSize = 4u * 1024u * 1024u;
    config.nativeStorage = true;
    IifxMachine original(testRom(), config);
    (void)original.read16(0x40000100);             // release reset overlay

    std::vector<u8> hardDisk = hfs::formatVolume(
        2u * 1024u * 1024u, "Replay HD");
    REQUIRE_FALSE(hardDisk.empty());
    original.insertHardDisk(std::move(hardDisk));
    std::vector<u8> floppy(1474560u, 0);
    floppy[0] = 0x4C;
    floppy[511] = 0xA5;
    REQUIRE(original.insertFloppy(std::move(floppy)) == 1);

    original.write32(0x00001000u, 0x12345678u);
    original.write8(0xF9001000u, 0x80);
    original.write8(0x50000000u, 0x7F);            // VIA ORB
    original.keyEvent(0x00, true);
    original.mouseMove(3, -2, false);
    for (int index = 0; index < 500; ++index) original.stepInstruction();

    const std::vector<u8> checkpoint = original.saveState();
    REQUIRE(checkpoint.size() > 64);

    IifxMachine replay(testRom(), config);
    std::string error;
    REQUIRE_MESSAGE(replay.loadState(checkpoint.data(), checkpoint.size(), &error),
                    error);
    CHECK(replay.saveState() == checkpoint);
    CHECK(replay.hardDiskPresent());
    CHECK(replay.floppyPresent());
    CHECK(replay.floppyImage()[0] == 0x4C);
    CHECK(replay.floppyImage()[511] == 0xA5);

    const auto advance = [](IifxMachine& machine) {
        machine.keyEvent(0x00, false);
        machine.keyEvent(0x0B, true);
        machine.mouseMove(-5, 9, true);
        machine.write8(0xF9001001u, 0x5A);
        for (int index = 0; index < 2000; ++index)
            machine.stepInstruction();
    };
    advance(original);
    advance(replay);
    CHECK(replay.saveState() == original.saveState());
    CHECK(replay.deterministicStateHash() == original.deterministicStateHash());
    CHECK(replay.framebufferHash() == original.framebufferHash());
    CHECK(replay.hardDiskImage() == original.hardDiskImage());
    CHECK(replay.floppyForWriteBack() == original.floppyForWriteBack());
}

TEST_CASE("IIfx checkpoint validation is atomic and binds ROM and RAM topology") {
    IifxMachine mac(testRom());
    mac.write32(0x1000, 0xCAFEBABEu);
    for (int index = 0; index < 20; ++index) mac.stepInstruction();
    const std::vector<u8> good = mac.saveState();

    std::vector<u8> corrupt = good;
    corrupt.back() ^= 0x80;
    std::string error;
    CHECK_FALSE(mac.loadState(corrupt.data(), corrupt.size(), &error));
    CHECK(error.find("checksum") != std::string::npos);
    CHECK(mac.saveState() == good);

    std::vector<u8> otherRom = testRom();
    otherRom[0x80] ^= 1;
    IifxMachine wrongRom(std::move(otherRom));
    CHECK_FALSE(wrongRom.loadState(good.data(), good.size(), &error));
    CHECK(error.find("different Macintosh IIfx ROM") != std::string::npos);

    IifxMachine::Config fourMeg;
    fourMeg.ramSize = 4u * 1024u * 1024u;
    IifxMachine wrongRam(testRom(), fourMeg);
    CHECK_FALSE(wrongRam.loadState(good.data(), good.size(), &error));
    CHECK(error.find("RAM configuration") != std::string::npos);
}

TEST_CASE("IIfx protocol trace asserts SCSI ACK without target REQ") {
    IifxMachine mac(testRom());
    HardwareTraceConfig trace;
    trace.capacity = 64;
    trace.postTriggerEvents = 0;
    mac.configureHardwareTrace(trace);

    // Direct NCR5380 register window: ICR is register 1, sixteen bytes from
    // the alternate controller base. The bus is still Bus Free, so ACK cannot
    // legally rise because no target has asserted REQ.
    mac.write8(0x5000E010u, 0x10u);
    CHECK(mac.hardwareTrace().triggered());
    CHECK(mac.hardwareTrace().frozen());
    CHECK(mac.hardwareTrace().triggerReason() ==
          "SCSI ACK asserted without target REQ");
    const auto events = mac.hardwareTrace().events();
    REQUIRE_FALSE(events.empty());
    CHECK(events.back().kind == HardwareTraceKind::Assertion);
    CHECK(events.back().source == HardwareTraceSource::Ncr5380);
}

TEST_CASE("IIfx bus-only trace can trigger on a filtered CPU PC") {
    IifxMachine mac(testRom());
    HardwareTraceConfig trace;
    trace.capacity = 16;
    trace.postTriggerEvents = 0;
    trace.categories = TraceBus;
    trace.triggerPc = mac.cpu().pc;
    trace.triggerPcHits = 1;
    mac.configureHardwareTrace(trace);

    mac.stepInstruction();

    CHECK(mac.hardwareTrace().triggered());
    CHECK(mac.hardwareTrace().frozen());
    CHECK(mac.hardwareTrace().triggerReason() == "PC hit threshold");
    const auto events = mac.hardwareTrace().events();
    const auto trigger = std::find_if(events.begin(), events.end(),
        [](const HardwareTraceEvent& event) {
            return event.kind == HardwareTraceKind::Trigger;
        });
    REQUIRE(trigger != events.end());
    CHECK(trigger->pc == 0x40000100u);
}

TEST_CASE("IIfx device-I/O trace excludes ordinary RAM traffic") {
    IifxMachine mac(testRom());
    HardwareTraceConfig trace;
    trace.capacity = 16;
    trace.categories = TraceIo;
    mac.configureHardwareTrace(trace);

    mac.write8(0x1000u, 0xA5u);
    mac.write8(0x50000400u, 0xE0u);          // VIA1 DDRB

    const auto events = mac.hardwareTrace().events();
    REQUIRE(events.size() == 1);
    CHECK(events.front().kind == HardwareTraceKind::Access);
    CHECK(events.front().source == HardwareTraceSource::Via);
    CHECK(events.front().address == 0x50000400u);
    CHECK((events.front().categories & TraceIo) != 0);
}

TEST_CASE("IIfx protocol trace asserts undefined IOP mailbox state") {
    IifxMachine mac(testRom());
    constexpr u32 base = 0x50F12000u;              // ISM IOP mirror
    const auto setAddress = [&](u16 address) {
        mac.write8(base + 0, static_cast<u8>(address >> 8));
        mac.write8(base + 2, static_cast<u8>(address));
    };
    const auto put = [&](u16 address, u8 value) {
        setAddress(address);
        mac.write8(base + 8, value);
    };
    put(IifxIop::Alive, 0xFF);                     // firmware initialized
    put(IifxIop::SendState, 0x7F);                 // outside Idle..Complete

    HardwareTraceConfig trace;
    trace.capacity = 64;
    trace.postTriggerEvents = 0;
    mac.configureHardwareTrace(trace);
    mac.stepInstruction();

    CHECK(mac.hardwareTrace().triggered());
    CHECK(mac.hardwareTrace().triggerReason() ==
          "IOP mailbox state contains an undefined protocol value");
    const auto events = mac.hardwareTrace().events();
    REQUIRE_FALSE(events.empty());
    CHECK(events.back().source == HardwareTraceSource::IsmIop);
}

TEST_CASE("IIfx protocol trace permits unconfigured FF IOP mailboxes") {
    IifxMachine mac(testRom());
    constexpr u32 base = 0x50F12000u;              // ISM IOP mirror
    const auto setAddress = [&](u16 address) {
        mac.write8(base + 0, static_cast<u8>(address >> 8));
        mac.write8(base + 2, static_cast<u8>(address));
    };
    const auto put = [&](u16 address, u8 value) {
        setAddress(address);
        mac.write8(base + 8, value);
    };
    put(IifxIop::Alive, 0xFF);
    put(IifxIop::SendState, 0xFF);
    put(IifxIop::RecvState, 0xFF);

    HardwareTraceConfig trace;
    trace.capacity = 64;
    trace.postTriggerEvents = 0;
    mac.configureHardwareTrace(trace);
    mac.stepInstruction();

    CHECK_FALSE(mac.hardwareTrace().triggered());
}

TEST_CASE("IIfx protocol trace asserts SWIM ACTION without a drive select") {
    IifxMachine mac(testRom());
    constexpr u32 base = 0x50F12000u;              // ISM IOP mirror
    const auto setAddress = [&](u16 address) {
        mac.write8(base + 0, static_cast<u8>(address >> 8));
        mac.write8(base + 2, static_cast<u8>(address));
    };
    const auto upload = [&](u16 address, const u8* bytes, std::size_t count) {
        setAddress(address);
        mac.write8(base + 4, 0x02);                // reset + autoincrement
        for (std::size_t index = 0; index < count; ++index)
            mac.write8(base + 8, bytes[index]);
    };
    // Set SWIM Mode.ACTION through peripheral port 7 while leaving /ENBL1 and
    // /ENBL2 both clear, then wait. This is malformed firmware behavior and
    // exercises the same IOP-to-SWIM path as Apple's downloaded image.
    const u8 firmware[] = {
        0xA9, 0x08,                   // LDA #ACTION
        0x8D, 0x47, 0xF0,             // STA $F047 (SWIM Mode-set)
        0x80, 0xFE,                    // BRA *
    };
    const u8 vectors[] = {0x00, 0x04, 0x00, 0x04, 0x00, 0x04};
    upload(0x0400, firmware, sizeof firmware);
    upload(0x7FFA, vectors, sizeof vectors);

    HardwareTraceConfig trace;
    trace.capacity = 128;
    trace.postTriggerEvents = 0;
    mac.configureHardwareTrace(trace);
    mac.write8(base + 4, 0x06);                    // RUN | AUTOINC
    for (int instruction = 0;
         instruction < 2000 && !mac.hardwareTrace().triggered(); ++instruction)
        mac.stepInstruction();

    CHECK(mac.hardwareTrace().triggered());
    CHECK(mac.hardwareTrace().triggerReason() ==
          "SWIM ACTION asserted with no selected drive");
    const auto events = mac.hardwareTrace().events();
    REQUIRE_FALSE(events.empty());
    CHECK(events.back().source == HardwareTraceSource::Swim);
}
