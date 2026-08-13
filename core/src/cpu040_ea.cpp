#include "cpu040_ops.hpp"

// Effective-address calculation, including the 68020 full-format extension
// words (scaled index, 16/32-bit base displacements, memory indirect pre- and
// post-indexed). The '040 executes all of these; the Mac ROM and System 7 use
// scaled index and base displacement constantly, memory indirect rarely.
//
// Reference: M68000PRM 2.2 (extension word formats); M68040UM 10 for the
// EA fetch-time model. Clean-room from the manuals.

namespace openmac {

// Shared by d8/idx(An) (mode 6) and d8/idx(PC) (mode 7.3). `base` is the base
// register value -- An, or the PC value at the extension word. Bit 8 of the
// extension word selects brief (0) or full (1) format.
u32 CpuOps040::indexExtension(M68040& c, u32 base) {
    const u16 ext = c.fetch16();
    if ((ext & 0x0100) || ((ext >> 9) & 3)) ++c.ext020Count;
    const int xreg = (ext >> 12) & 7;
    u32 idx = (ext & 0x8000) ? c.a[xreg] : c.d[xreg];
    if (!(ext & 0x0800))
        idx = static_cast<u32>(static_cast<s32>(static_cast<s16>(idx & 0xFFFF)));
    idx <<= (ext >> 9) & 3;   // scale x1/x2/x4/x8

    if (!(ext & 0x0100)) {    // brief format: d8 + base + scaled index
        const s32 disp = static_cast<s8>(ext & 0xFF);
        return base + static_cast<u32>(disp) + idx;
    }

    // Full format. BS (bit 7) suppresses the base register, IS (bit 6) the
    // index; BD size (bits 5-4): 01 null, 10 word, 11 long. I/IS (bits 2-0)
    // selects memory indirection and the outer displacement size.
    c.lastEaFull030_ = c.is68030();
    if (!c.is68030()) c.eaExtra_ += 2; // 040 full-format parse
    if (ext & 0x0080) base = 0;
    u32 bd = 0;
    const int bdSize = (ext >> 4) & 3;
    if (bdSize == 2) bd = static_cast<u32>(static_cast<s32>(static_cast<s16>(c.fetch16())));
    else if (bdSize == 3) bd = c.fetch32();
    const bool indexSuppressed = (ext & 0x0040) != 0;
    if (indexSuppressed) idx = 0;

    const int iis = ext & 7;
    if (iis == 0) {
        if (c.is68030()) {
            // The base 030 indexed timing is the brief-format six-clock
            // fetch. Full format has the same fetch cost for null/word base
            // displacement on the ordinary An/PC forms; a long displacement
            // needs three additional instruction words' worth of work.
            if (bdSize == 3) c.eaExtra_ += 6;
            else if (bdSize == 2 && (ext & 0x0080)) c.eaExtra_ += 2;
        }
        return base + bd + idx;                 // no memory indirection
    }

    // Memory indirect: 1-3 = preindexed (index applies before the fetch),
    // 5-7 = postindexed (index applies after; only legal with IS clear).
    if (c.is68030()) {
        c.eaExtra_ += 4;                         // full-format indirection
        if (bdSize == 3) c.eaExtra_ += 6;
        else if (bdSize == 2 && (ext & 0x0080)) c.eaExtra_ += 2;
    } else {
        c.eaExtra_ += 4;                         // 040 indirection fetch
    }
    const bool post = (iis & 4) != 0;
    const int odSize = iis & 3;                 // 1 null, 2 word, 3 long
    const u32 inner = post ? (base + bd) : (base + bd + idx);
    const u32 fetched = c.rd32(inner);
    u32 od = 0;
    if (odSize == 2) od = static_cast<u32>(static_cast<s32>(static_cast<s16>(c.fetch16())));
    else if (odSize == 3) od = c.fetch32();
    if (c.is68030() && odSize >= 2) c.eaExtra_ += 2;
    return fetched + (post ? idx : 0) + od;
}

u32 CpuOps040::calcEA(M68040& c, int mode, int reg, int size) {
    if (c.is68030()) c.lastEaFull030_ = false;
    switch (mode) {
    case 2:
        return c.a[reg];
    case 3: {
        const u32 addr = c.a[reg];
        u32 inc = size == 0 ? 1u : size == 1 ? 2u : 4u;
        if (size == 0 && reg == 7) inc = 2;
        c.a[reg] += inc;
        return addr;
    }
    case 4: {
        u32 dec = size == 0 ? 1u : size == 1 ? 2u : 4u;
        if (size == 0 && reg == 7) dec = 2;
        c.a[reg] -= dec;
        return c.a[reg];
    }
    case 5: {
        const s32 disp = static_cast<s16>(c.fetch16());
        return c.a[reg] + static_cast<u32>(disp);
    }
    case 6:
        return indexExtension(c, c.a[reg]);
    case 7:
        switch (reg) {
        case 0:
            return static_cast<u32>(static_cast<s32>(static_cast<s16>(c.fetch16())));
        case 1:
            return c.fetch32();
        case 2: {
            const u32 base = c.pc;
            const s32 disp = static_cast<s16>(c.fetch16());
            return base + static_cast<u32>(disp);
        }
        case 3:
            return indexExtension(c, c.pc);
        default: break;
        }
        break;
    default: break;
    }
    return 0; // unreachable for valid decodes
}

} // namespace openmac
