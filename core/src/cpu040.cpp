#include "cpu040_ops.hpp"

// M68040 core engine: reset, the three-bank stack model, MMU-translated bus
// access with misaligned-access splitting, exception frame construction, and
// the step loop. Instruction handlers live in the cpu040_ops_*.cpp files.
//
// Reference: M68040UM sections 2/3/8; frame layouts from section 8.4.
// Clean-room from the manuals.

namespace openmac {

// ---------------------------------------------------------------- construction

M68040::M68040(IBus040& bus) : M68040(bus, M68kCpuModel::M68040) {}

M68040::M68040(IBus040& bus, M68kCpuModel model)
    : bus_(bus), model_(model) {
    (void)CpuOps040::table(model_); // force the model's table up front
}

void M68040::reset() {
    sr_      = 0x2700;        // supervisor, interrupt stack, mask 7, no trace
    stopped  = false;
    halted   = false;
    vbr      = 0;
    sfc = dfc = 0;
    cacr = 0;
    caar = 0;
    if (is68030()) {
        // RESET disables translation and both transparent windows.  Keep the
        // reset state deterministic even for architecturally undefined root
        // descriptor payload bits; software must PMOVE them before enabling.
        tc = 0;
        tt0 = tt1 = 0;
        crp = srp030 = 0;
        itt0 = itt1 = dtt0 = dtt1 = 0;
        urp = srp = 0;
    } else {
        tc &= 0x4000u;       // '040 reset clears only E; P survives
        itt0 = itt1 = dtt0 = dtt1 = 0;
        urp = srp = 0;
        tt0 = tt1 = 0;
        crp = srp030 = 0;
    }
    mmusr    = 0;
    fpuUsed_ = false;
    fpcr = fpsr = fpiar = 0;
    if (is68030()) {
        clearCaches030();
        cacheStats030_ = CacheStats030{};
    }
    tlbFlush();
    isp      = bus_.read32(0);
    pc       = bus_.read32(4);
    a[7]     = isp;
}

void M68040::setIrqLevel(int level) { irqLevel_ = level; }

// Of usp/isp/msp, the field for an inactive bank holds that bank's true value;
// the active bank's truth lives in a[7].
u32 M68040::uspValue() const {
    return (sr_ & kS040) ? usp : a[7];
}
u32 M68040::ispValue() const {
    if ((sr_ & kS040) && !(sr_ & kM040)) return a[7];
    return isp;
}
u32 M68040::mspValue() const {
    if ((sr_ & kS040) && (sr_ & kM040)) return a[7];
    return msp;
}

void M68040::setSR(u16 value) {
    value &= kSrMask040;
    // Bank a[7] out to wherever the CURRENT mode keeps its stack pointer...
    if (sr_ & kS040) {
        if (sr_ & kM040) msp = a[7]; else isp = a[7];
    } else {
        usp = a[7];
    }
    sr_ = value;
    // ...and bank in the stack the NEW mode selects.
    if (sr_ & kS040) {
        a[7] = (sr_ & kM040) ? msp : isp;
    } else {
        a[7] = usp;
    }
}

void M68040::setCCR(u8 value) {
    sr_ = static_cast<u16>((sr_ & 0xFF00) | (value & 0x1F));
}

// ---------------------------------------------------------------- MMU

// Transparent translation register match. Fields (M68040UM 3.1.2): bits 31-24
// logical address base, 23-16 address mask (set bits = don't care), 15 enable,
// 14-13 S-field (00 user, 01 supervisor, 1x both), 2 write-protect.
bool M68040::ttrMatch(u32 ttr, u32 laddr, bool super, bool /*write*/) const {
    if (!(ttr & 0x8000u)) return false;
    const u32 sField = (ttr >> 13) & 3;
    if (sField == 0 && super) return false;
    if (sField == 1 && !super) return false;
    const u32 mask = (ttr >> 16) & 0xFFu;
    const u32 base = (ttr >> 24) & 0xFFu;
    const u32 la8  = (laddr >> 24) & 0xFFu;
    return ((la8 ^ base) & ~mask) == 0;
}

// MC68030 transparent-translation register (MC68030UM 9.7.3): address base
// and mask in the upper half, E/CI/RW/RWM in bits 15/10/9/8, then FC base
// in 6-4 and FC mask in 2-0.  Transparent matches are identity mappings and
// operate independently of TC.E.
bool M68040::ttrMatch030(u32 ttr, u32 laddr, u32 fc, bool write) const {
    if (!(ttr & 0x00008000u)) return false;
    const u32 addrBase = ttr >> 24;
    const u32 addrMask = (ttr >> 16) & 0xFFu;
    if ((((laddr >> 24) ^ addrBase) & ~addrMask) != 0) return false;
    const bool rwm = (ttr & 0x00000100u) != 0;
    const bool wantsRead = (ttr & 0x00000200u) != 0;
    if (!rwm && wantsRead == write) return false;
    const u32 fcBase = (ttr >> 4) & 7;
    const u32 fcMask = ttr & 7;
    return (((fc ^ fcBase) & ~fcMask) & 7) == 0;
}

void M68040::tlbFlush() {
    for (auto& e : tlb_) e = TlbEntry{};
    tlbNext030_ = 0;
}

void M68040::tlbFlushPage(u32 laddr) {
    if (is68030()) {
        for (int i = 0; i < 22; ++i) {
            TlbEntry& e = tlb_[i];
            if (e.tag != 0xFFFFFFFFu && e.tag == (laddr >> e.pageShift))
                e = TlbEntry{};
        }
        return;
    }
    const u32 pageShift = (tc & 0x4000u) ? 13u : 12u;
    const u32 vpn = laddr >> pageShift;
    // Both the user and supervisor mappings of the page go.
    for (u32 s = 0; s < 2; ++s) {
        TlbEntry& e = tlb_[((vpn << 1) | s) & (kTlbSize - 1)];
        if (e.tag == ((vpn << 1) | s)) e = TlbEntry{};
    }
}

u32 M68040::translate(u32 laddr, bool write, bool instruction) {
    const bool super = (sr_ & kS040) != 0;
    if (is68030()) return translateFc(laddr, write, super, instruction);
    if (!mmuEnabled()) return laddr;
    return translateFc(laddr, write, super, instruction);
}

u32 M68040::translateFc(u32 laddr, bool write, bool super, bool instruction) {
    if (is68030()) {
        const u32 fc = super ? (instruction ? 6u : 5u)
                             : (instruction ? 2u : 1u);
        return translateFc030(laddr, write, fc);
    }

    if (!mmuEnabled()) return laddr;
    // Transparent translation is checked ahead of the ATC (UM 3.2).
    const u32 t0 = instruction ? itt0 : dtt0;
    const u32 t1 = instruction ? itt1 : dtt1;
    if (ttrMatch(t0, laddr, super, write)) {
        if (write && (t0 & 0x4u)) throw AccessFault{laddr, false, 1, instruction, false};
        return laddr;
    }
    if (ttrMatch(t1, laddr, super, write)) {
        if (write && (t1 & 0x4u)) throw AccessFault{laddr, false, 1, instruction, false};
        return laddr;
    }
    const u32 pageShift = (tc & 0x4000u) ? 13u : 12u;
    const u32 vpn = laddr >> pageShift;
    const u32 tag = (vpn << 1) | (super ? 1u : 0u);
    TlbEntry& e = tlb_[tag & (kTlbSize - 1)];
    if (e.tag == tag) {
        // A write through an entry whose page hasn't had its M bit set yet
        // re-walks so the descriptor in memory is updated, as hardware does.
        if (!write || (e.writable && e.modified))
            return e.phys | (laddr & ((1u << pageShift) - 1u));
        if (!e.writable) throw AccessFault{laddr, false, 1, instruction, false};
    }
    return tableWalk(laddr, write, super, instruction);
}

u32 M68040::translateFc030(u32 laddr, bool write, u32 fc,
                           bool* cacheInhibit) {
    fc &= 7u;
    if (cacheInhibit) *cacheInhibit = false;
    const bool instruction = (fc & 3u) == 2u;

    if (((tt0 | tt1) & 0x00008000u) != 0) {
        const bool tt0Match = ttrMatch030(tt0, laddr, fc, write);
        const bool tt1Match = ttrMatch030(tt1, laddr, fc, write);
        if (tt0Match || tt1Match) {
            if (cacheInhibit)
                *cacheInhibit = (tt0Match && (tt0 & 0x00000400u)) ||
                                (tt1Match && (tt1 & 0x00000400u));
            return laddr;
        }
    }
    if (!mmuEnabled()) return laddr;

    // The 22-entry ATC is fully associative; sequential fetches and stack
    // traffic hit the same entry over and over, so try the last hit before
    // scanning.  (Replacement and flushes rewrite entries in place; the
    // re-check of tag/fc below keeps a stale index harmless.)
    const auto entryHits = [&](TlbEntry& e) {
        return e.tag != 0xFFFFFFFFu && e.fc == fc &&
               e.tag == (laddr >> e.pageShift);
    };
    const auto resolve = [&](TlbEntry& e, int index, u32& out) {
        if (write && !e.writable)
            throw AccessFault{laddr, false, 1, instruction, false};
        // A first write to a read-established entry must revisit the page
        // descriptor so its M history bit is committed before the access.
        if (!write || e.modified) {
            if (cacheInhibit) *cacheInhibit = e.cacheInhibit;
            tlbLastHit030_ = index;
            out = e.phys | (laddr & ((1u << e.pageShift) - 1u));
            return true;
        }
        return false;
    };
    u32 translated = 0;
    if (tlbLastHit030_ >= 0 && tlbLastHit030_ < 22 &&
        entryHits(tlb_[tlbLastHit030_])) {
        if (resolve(tlb_[tlbLastHit030_], tlbLastHit030_, translated))
            return translated;
        return tableWalk030(laddr, write, fc, cacheInhibit);
    }
    for (int i = 0; i < 22; ++i) {
        TlbEntry& e = tlb_[i];
        if (!entryHits(e)) continue;
        if (resolve(e, i, translated)) return translated;
        break;
    }
    return tableWalk030(laddr, write, fc, cacheInhibit);
}

// The 68030 ATC holds at most one entry per (logical page, function code):
// a table search for a page that is already cached -- typically the first
// write to a read-established page, which must commit the descriptor's M
// bit -- updates that entry rather than adding a duplicate.  Only when the
// page is new does the deterministic round-robin replacement pick a slot.
M68040::TlbEntry& M68040::atcEntryFor030(u32 tag, u32 fc) {
    for (int i = 0; i < 22; ++i) {
        TlbEntry& e = tlb_[i];
        if (e.tag != 0xFFFFFFFFu && e.tag == tag && e.fc == fc) return e;
    }
    return tlb_[tlbNext030_++ % 22];
}

// MC68030 variable-depth table walker.  TC supplies an initial shift, up to
// four logical-address index widths, and one of eight page sizes.  Descriptors
// may be short (4 byte), long (8 byte), early-termination pages, or indirect
// page pointers.  The ATC is the documented 22-entry fully-associative cache;
// replacement is deterministic round-robin (replacement order is not
// architecturally observable except through performance).
u32 M68040::tableWalk030(u32 laddr, bool write, u32 fc,
                         bool* cacheInhibit) {
    const int pageShift = static_cast<int>((tc >> 20) & 0xF);
    const int initialShift = static_cast<int>((tc >> 16) & 0xF);
    if (pageShift < 8 || pageShift > 15)
        throw AccessFault{laddr, !write, 1, (fc & 3) == 2, true};

    int widths[4] = {
        static_cast<int>((tc >> 12) & 0xF),
        static_cast<int>((tc >> 8) & 0xF),
        static_cast<int>((tc >> 4) & 0xF),
        static_cast<int>(tc & 0xF),
    };
    int usedWidths = 0, total = initialShift + pageShift;
    while (usedWidths < 4 && widths[usedWidths] != 0) {
        total += widths[usedWidths++];
    }
    if (total != 32)
        throw AccessFault{laddr, !write, 1, (fc & 3) == 2, true};

    const bool useSrp = (tc & 0x02000000u) && (fc & 4);
    const u64 root = useSrp ? srp030 : crp;
    u32 first = static_cast<u32>(root >> 32);
    u32 second = static_cast<u32>(root);
    int format = first & 3;                 // 2 = short, 3 = long
    if (format == 0)
        throw AccessFault{laddr, !write, 1, (fc & 3) == 2, true};

    u32 tableBase = second & 0xFFFFFFF0u;
    bool wp = false;
    bool supervisorOnly = false;
    int bitPos = 32 - initialShift;

    struct Desc {
        u32 at;
        u32 hi;
        u32 lo;
        int bytes;
    };
    auto fetchDesc = [&](u32 at, int bytes) -> Desc {
        Desc d{at, bus_.read32(at), 0, bytes};
        if (bytes == 8) d.lo = bus_.read32(at + 4);
        return d;
    };
    auto updateStatus = [&](Desc& d, bool page) {
        u32 n = d.hi | 0x08u;               // U
        if (page && write && !wp) n |= 0x10u; // M
        if (n != d.hi) {
            bus_.write32(d.at, n);
            d.hi = n;
        }
    };

    try {
        int nextWidth = 0;
        bool fcLevel = (tc & 0x01000000u) != 0;
        for (;;) {
            u32 index;
            if (fcLevel) {
                index = fc & 7;
                fcLevel = false;
            } else {
                if (nextWidth >= usedWidths)
                    throw AccessFault{laddr, !write, 1, (fc & 3) == 2, true};
                const int width = widths[nextWidth++];
                bitPos -= width;
                index = (laddr >> bitPos) & ((1u << width) - 1u);
            }

            const int bytes = format == 3 ? 8 : 4;
            Desc desc = fetchDesc(tableBase + index * static_cast<u32>(bytes), bytes);
            const int dt = desc.hi & 3;
            if (dt == 0)
                throw AccessFault{laddr, !write, 1, (fc & 3) == 2, true};

            wp |= (desc.hi & 0x04u) != 0;
            supervisorOnly |= (desc.hi & 0x80u) != 0;
            if (supervisorOnly && !(fc & 4))
                throw AccessFault{laddr, !write, 1, (fc & 3) == 2, false};
            if (write && wp)
                throw AccessFault{laddr, false, 1, (fc & 3) == 2, false};

            if (dt == 1) {                  // page / early termination
                updateStatus(desc, true);
                const u32 pageWord = desc.bytes == 8 ? desc.lo : desc.hi;
                const int passthrough = bitPos > pageShift ? bitPos : pageShift;
                const u32 passMask = (1u << passthrough) - 1u;
                const u32 pa = (pageWord & ~passMask) | (laddr & passMask);

                TlbEntry& e = atcEntryFor030(laddr >> pageShift, fc);
                e.tag = laddr >> pageShift;
                e.phys = pa & ~((1u << pageShift) - 1u);
                e.writable = !wp;
                e.modified = write;
                e.cacheInhibit = (desc.hi & 0x40u) != 0;
                e.pageShift = static_cast<u8>(pageShift);
                e.fc = static_cast<u8>(fc);
                if (cacheInhibit) *cacheInhibit = e.cacheInhibit;
                return pa;
            }

            if (nextWidth >= usedWidths && !fcLevel) {
                // At page-table depth, DT=2/3 is an indirect descriptor.
                const u32 indirect = (desc.bytes == 8 ? desc.lo : desc.hi) & 0xFFFFFFFCu;
                Desc p = fetchDesc(indirect, dt == 3 ? 8 : 4);
                if ((p.hi & 3) != 1)
                    throw AccessFault{laddr, !write, 1, (fc & 3) == 2, true};
                wp |= (p.hi & 0x04u) != 0;
                supervisorOnly |= (p.hi & 0x80u) != 0;
                if ((supervisorOnly && !(fc & 4)) || (write && wp))
                    throw AccessFault{laddr, !write, 1, (fc & 3) == 2, false};
                updateStatus(p, true);
                const u32 pageWord = p.bytes == 8 ? p.lo : p.hi;
                const u32 mask = (1u << pageShift) - 1u;
                const u32 pa = (pageWord & ~mask) | (laddr & mask);
                TlbEntry& e = atcEntryFor030(laddr >> pageShift, fc);
                e.tag = laddr >> pageShift;
                e.phys = pa & ~mask;
                e.writable = !wp;
                e.modified = write;
                e.cacheInhibit = (p.hi & 0x40u) != 0;
                e.pageShift = static_cast<u8>(pageShift);
                e.fc = static_cast<u8>(fc);
                if (cacheInhibit) *cacheInhibit = e.cacheInhibit;
                return pa;
            }

            updateStatus(desc, false);
            tableBase = (desc.bytes == 8 ? desc.lo : desc.hi) & 0xFFFFFFF0u;
            format = dt;
        }
    } catch (const BusFault&) {
        throw AccessFault{laddr, !write, 1, (fc & 3) == 2, true};
    }
}

// Three-level table walk (M68040UM 3.3): 7-bit root index, 7-bit pointer
// index, 6-bit (4K) or 5-bit (8K) page index. Upper-level descriptors:
// UDT bits 1-0 (00/01 invalid, 10/11 resident), U bit 3. Page descriptors:
// PDT 00 invalid, 01/11 resident, 10 indirect; W bit 2, U bit 3, M bit 4,
// S bit 7. U and M are updated in memory as hardware would.
u32 M68040::tableWalk(u32 laddr, bool write, bool super, bool instruction) {
    const bool page8k = (tc & 0x4000u) != 0;
    const u32 pageShift = page8k ? 13u : 12u;
    const u32 ri = laddr >> 25;
    const u32 pi = (laddr >> 18) & 0x7Fu;
    const u32 pgi = page8k ? ((laddr >> 13) & 0x1Fu) : ((laddr >> 12) & 0x3Fu);

    bool wp = false;
    try {
        const u32 rp = (super ? srp : urp) & 0xFFFFFE00u;
        const u32 d1a = rp + ri * 4;
        u32 d1 = bus_.read32(d1a);
        if ((d1 & 2u) == 0) throw AccessFault{laddr, !write, 1, instruction, true};
        if (d1 & 4u) wp = true;
        if (!(d1 & 8u)) bus_.write32(d1a, d1 | 8u);   // U bit

        const u32 ptBase = d1 & 0xFFFFFE00u;
        const u32 d2a = ptBase + pi * 4;
        u32 d2 = bus_.read32(d2a);
        if ((d2 & 2u) == 0) throw AccessFault{laddr, !write, 1, instruction, true};
        if (d2 & 4u) wp = true;
        if (!(d2 & 8u)) bus_.write32(d2a, d2 | 8u);

        const u32 pgBase = d2 & (page8k ? 0xFFFFFF80u : 0xFFFFFF00u);
        u32 d3a = pgBase + pgi * 4;
        u32 d3 = bus_.read32(d3a);
        if ((d3 & 3u) == 2u) {           // indirect: one level only
            d3a = d3 & 0xFFFFFFFCu;
            d3 = bus_.read32(d3a);
            if ((d3 & 3u) == 2u || (d3 & 3u) == 0u)
                throw AccessFault{laddr, !write, 1, instruction, true};
        }
        if ((d3 & 3u) == 0u) throw AccessFault{laddr, !write, 1, instruction, true};
        if (d3 & 4u) wp = true;
        if ((d3 & 0x80u) && !super)      // supervisor-only page
            throw AccessFault{laddr, !write, 1, instruction, true};
        if (write && wp)                 // write to protected: M stays clear
            throw AccessFault{laddr, false, 1, instruction, false};

        u32 newBits = d3 | 8u;                     // U
        if (write) newBits |= 0x10u;               // M
        if (newBits != d3) bus_.write32(d3a, newBits);

        const u32 physPage = d3 & (page8k ? 0xFFFFE000u : 0xFFFFF000u);
        const u32 vpn = laddr >> pageShift;
        const u32 tag = (vpn << 1) | (super ? 1u : 0u);
        TlbEntry& e = tlb_[tag & (kTlbSize - 1)];
        e.tag = tag;
        e.phys = physPage;
        e.writable = !wp;
        e.modified = (newBits & 0x10u) != 0;
        return physPage | (laddr & ((1u << pageShift) - 1u));
    } catch (const BusFault&) {
        // A bus error during the walk itself surfaces as an ATC fault.
        throw AccessFault{laddr, !write, 1, instruction, true};
    }
}

// ---------------------------------------------------------------- MC68030 caches

void M68040::clearCaches030() {
    for (auto& line : instructionCache030_) line = CacheLine030{};
    for (auto& line : dataCache030_) line = CacheLine030{};
}

void M68040::writeCacr030(u32 value) {
    // CD/CI clear every valid bit.  CED/CEI select exactly one longword by
    // CAAR A7..A2.  These four command bits self-clear and therefore never
    // appear when CACR is read back.
    if (value & 0x00000800u)
        for (auto& line : dataCache030_) line.valid = 0;
    if (value & 0x00000400u) {
        CacheLine030& line = dataCache030_[(caar >> 4) & 15u];
        line.valid &= static_cast<u8>(~(1u << ((caar >> 2) & 3u)));
    }
    if (value & 0x00000008u)
        for (auto& line : instructionCache030_) line.valid = 0;
    if (value & 0x00000004u) {
        CacheLine030& line = instructionCache030_[(caar >> 4) & 15u];
        line.valid &= static_cast<u8>(~(1u << ((caar >> 2) & 3u)));
    }
    cacr = value & 0x00003313u;
}

u32 M68040::extractCached030(u32 word, u32 addr, int size) {
    if (size == 2) return word;
    if (size == 1) return (word >> ((addr & 2u) ? 0 : 16)) & 0xFFFFu;
    return (word >> ((3u - (addr & 3u)) * 8u)) & 0xFFu;
}

u32 M68040::mergeCached030(u32 word, u32 addr, u32 value, int size) {
    if (size == 2) return value;
    if (size == 1) {
        const u32 shift = (addr & 2u) ? 0u : 16u;
        return (word & ~(0xFFFFu << shift)) | ((value & 0xFFFFu) << shift);
    }
    const u32 shift = (3u - (addr & 3u)) * 8u;
    return (word & ~(0xFFu << shift)) | ((value & 0xFFu) << shift);
}

u32 M68040::readCached030(u32 addr, int size, u32 fc, bool instruction) {
    const bool enabled = instruction ? (cacr & 0x00000001u) != 0
                                     : (cacr & 0x00000100u) != 0;

    auto& cache = instruction ? instructionCache030_ : dataCache030_;
    const u32 lineIndex = (addr >> 4) & 15u;
    const u32 entry = (addr >> 2) & 3u;
    const u32 tag = addr >> 8;
    const u8 tagFc = static_cast<u8>(instruction ? (fc & 4u) : (fc & 7u));
    CacheLine030& line = cache[lineIndex];
    const bool lineHit = line.tag == tag && line.fc == tagFc;

    // The 030 caches are logically addressed.  Tag comparison precedes the
    // ATC/table search, so a valid hit completes without translating the
    // address at all (MC68030UM 6.2). Software changing a mapping must flush
    // both the ATC and any affected cache entry.
    if (enabled && lineHit && (line.valid & (1u << entry))) {
        if (instruction) ++cacheStats030_.instructionHits;
        else ++cacheStats030_.dataHits;
        return extractCached030(line.data[entry], addr, size);
    }

    bool cacheInhibit = false;
    const u32 pa = translateFc030(addr, false, fc, &cacheInhibit);

    auto directRead = [&]() -> u32 {
        try {
            if (cacheInhibit) {
                if (size == 0) return bus_.read8CacheInhibited(pa);
                if (size == 1) return bus_.read16CacheInhibited(pa);
                return bus_.read32CacheInhibited(pa);
            }
            if (size == 0) return bus_.read8(pa);
            if (size == 1) return bus_.read16(pa);
            return bus_.read32(pa);
        } catch (const BusFault&) {
            throw AccessFault{addr, true, size == 0 ? 1 : size == 1 ? 2 : 4,
                              instruction, false};
        }
    };
    if (!enabled || cacheInhibit || !bus_.cacheable(pa)) return directRead();
    if (instruction) ++cacheStats030_.instructionMisses;
    else ++cacheStats030_.dataMisses;

    const u32 logicalLong = addr & ~3u;
    const u32 physicalLong = translateFc030(logicalLong, false, fc);
    const bool frozen = instruction ? (cacr & 0x00000002u) != 0
                                    : (cacr & 0x00000200u) != 0;
    const bool wantsBurst = !frozen && !lineHit &&
        (instruction ? (cacr & 0x00000010u) != 0
                     : (cacr & 0x00001000u) != 0);
    u32 word;
    u32 burstWords[4]{};
    bool burstAcknowledged = false;
    try {
        if (wantsBurst) {
            burstAcknowledged = bus_.readBurst32(physicalLong, burstWords);
            word = burstWords[0];
        } else {
            word = bus_.read32(physicalLong);
        }
    } catch (const BusFault&) {
        throw AccessFault{addr, true, size == 0 ? 1 : size == 1 ? 2 : 4,
                          instruction, false};
    }

    if (!frozen) {
        if (!lineHit) {
            line.tag = tag;
            line.fc = tagFc;
            line.valid = 0;
        }
        line.data[entry] = word;
        line.valid |= static_cast<u8>(1u << entry);

        // CBACK is what authorizes the remaining entries.  The IIfx FMC does
        // not acknowledge a burst when its own cache hits, so in that case
        // only the explicitly requested longword becomes valid on-chip.
        if (wantsBurst && burstAcknowledged) {
            cacheStats030_.burstLongwords += 4;
            for (u32 n = 1; n < 4; ++n) {
                const u32 slot = (entry + n) & 3u;
                line.data[slot] = burstWords[n];
                line.valid |= static_cast<u8>(1u << slot);
            }
        }
    }
    return extractCached030(word, addr, size);
}

void M68040::writeCached030(u32 addr, u32 value, int size, u32 fc) {
    bool cacheInhibit = false;
    const u32 pa = translateFc030(addr, true, fc, &cacheInhibit);
    if ((cacr & 0x00000100u) && !cacheInhibit && bus_.cacheable(pa)) {
        const u32 lineIndex = (addr >> 4) & 15u;
        const u32 entry = (addr >> 2) & 3u;
        const u32 tag = addr >> 8;
        const u8 tagFc = static_cast<u8>(fc & 7u);
        CacheLine030& line = dataCache030_[lineIndex];
        const bool lineHit = line.tag == tag && line.fc == tagFc;
        const bool entryHit = lineHit && (line.valid & (1u << entry));
        if (entryHit) {
            // Write-through: update the cache before the external cycle.  The
            // updated entry remains architecturally visible even if that cycle
            // later bus-errors.
            line.data[entry] = mergeCached030(line.data[entry], addr, value, size);
        } else if ((cacr & 0x00002000u) && !(cacr & 0x00000200u)) {
            if (size == 2 && (addr & 3u) == 0) {
                if (!lineHit) {
                    line.tag = tag;
                    line.fc = tagFc;
                    line.valid = 0;
                }
                line.data[entry] = value;
                line.valid |= static_cast<u8>(1u << entry);
            } else {
                // Partial write-allocation never manufactures the unseen
                // bytes of a longword.  The indexed entry becomes invalid and
                // the existing tag is retained on a tag miss.
                line.valid &= static_cast<u8>(~(1u << entry));
            }
        }
    }

    try {
        if (size == 0) bus_.write8(pa, static_cast<u8>(value));
        else if (size == 1) bus_.write16(pa, static_cast<u16>(value));
        else bus_.write32(pa, value);
    } catch (const BusFault&) {
        throw AccessFault{addr, false, size == 0 ? 1 : size == 1 ? 2 : 4,
                          false, false};
    }
}

void M68040::invalidateDataCache030(u32 logical, u32 bytes) {
    if (!is68030() || bytes == 0) return;
    const u64 end = static_cast<u64>(logical) + bytes;
    for (u64 at = logical & ~3u; at < end; at += 4) {
        const u32 address = static_cast<u32>(at);
        CacheLine030& line = dataCache030_[(address >> 4) & 15u];
        if (line.tag == (address >> 8))
            line.valid &= static_cast<u8>(~(1u << ((address >> 2) & 3u)));
    }
}

// ---------------------------------------------------------------- bus access

// Every access is translated per naturally-aligned piece; the '040 bus
// controller performs the same splitting for misaligned operands, so a
// misaligned long that crosses a page boundary translates each piece.

u8 M68040::rd8(u32 addr) {
    if (is68030()) {
        const u32 fc = (sr_ & kS040) ? 5u : 1u;
        return static_cast<u8>(readCached030(addr, 0, fc, false));
    }
    const u32 pa = translate(addr, false, false);
    try {
        return bus_.read8(pa);
    } catch (const BusFault&) {
        throw AccessFault{addr, true, 1, false, false};
    }
}

u16 M68040::rd16(u32 addr) {
    if (addr & 1) {
        const u16 hi = rd8(addr);
        return static_cast<u16>((hi << 8) | rd8(addr + 1));
    }
    if (is68030()) {
        const u32 fc = (sr_ & kS040) ? 5u : 1u;
        return static_cast<u16>(readCached030(addr, 1, fc, false));
    }
    const u32 pa = translate(addr, false, false);
    try {
        return bus_.read16(pa);
    } catch (const BusFault&) {
        throw AccessFault{addr, true, 2, false, false};
    }
}

u32 M68040::rd32(u32 addr) {
    if (addr & 3) {
        if ((addr & 1) == 0) {
            const u32 hi = rd16(addr);
            return (hi << 16) | rd16(addr + 2);
        }
        const u32 b0 = rd8(addr);
        const u32 w  = rd16(addr + 1);
        const u32 b3 = rd8(addr + 3);
        return (b0 << 24) | (w << 8) | b3;
    }
    if (is68030()) {
        const u32 fc = (sr_ & kS040) ? 5u : 1u;
        return readCached030(addr, 2, fc, false);
    }
    const u32 pa = translate(addr, false, false);
    try {
        return bus_.read32(pa);
    } catch (const BusFault&) {
        throw AccessFault{addr, true, 4, false, false};
    }
}

void M68040::wr8(u32 addr, u8 v) {
    if (is68030()) {
        writeCached030(addr, v, 0, (sr_ & kS040) ? 5u : 1u);
        return;
    }
    const u32 pa = translate(addr, true, false);
    try {
        bus_.write8(pa, v);
    } catch (const BusFault&) {
        throw AccessFault{addr, false, 1, false, false};
    }
}

void M68040::wr16(u32 addr, u16 v) {
    if (addr & 1) {
        wr8(addr, static_cast<u8>(v >> 8));
        wr8(addr + 1, static_cast<u8>(v & 0xFF));
        return;
    }
    if (is68030()) {
        writeCached030(addr, v, 1, (sr_ & kS040) ? 5u : 1u);
        return;
    }
    const u32 pa = translate(addr, true, false);
    try {
        bus_.write16(pa, v);
    } catch (const BusFault&) {
        throw AccessFault{addr, false, 2, false, false};
    }
}

void M68040::wr32(u32 addr, u32 v) {
    if (addr & 3) {
        if ((addr & 1) == 0) {
            wr16(addr, static_cast<u16>(v >> 16));
            wr16(addr + 2, static_cast<u16>(v & 0xFFFF));
        } else {
            wr8(addr, static_cast<u8>(v >> 24));
            wr16(addr + 1, static_cast<u16>((v >> 8) & 0xFFFF));
            wr8(addr + 3, static_cast<u8>(v & 0xFF));
        }
        return;
    }
    if (is68030()) {
        writeCached030(addr, v, 2, (sr_ & kS040) ? 5u : 1u);
        return;
    }
    const u32 pa = translate(addr, true, false);
    try {
        bus_.write32(pa, v);
    } catch (const BusFault&) {
        throw AccessFault{addr, false, 4, false, false};
    }
}

u16 M68040::fetch16() {
    if (pc & 1) throw AccessFault{pc, true, 2, true, false};
    if (is68030()) {
        const u32 at = pc;
        const u32 fc = (sr_ & kS040) ? 6u : 2u;
        const u16 v = static_cast<u16>(readCached030(at, 1, fc, true));
        pc += 2;
        return v;
    }
    const u32 pa = translate(pc, false, true);
    u16 v;
    try {
        v = bus_.read16(pa);
    } catch (const BusFault&) {
        throw AccessFault{pc, true, 2, true, false};
    }
    pc += 2;
    return v;
}

u32 M68040::fetch32() {
    const u32 hi = fetch16();
    return (hi << 16) | fetch16();
}

void M68040::push16(u16 v) { a[7] -= 2; wr16(a[7], v); }
void M68040::push32(u32 v) { a[7] -= 4; wr32(a[7], v); }
u16  M68040::pop16() { const u16 v = rd16(a[7]); a[7] += 2; return v; }
u32  M68040::pop32() { const u32 v = rd32(a[7]); a[7] += 4; return v; }

// ---------------------------------------------------------------- exceptions

// Four-word format $0 frame: SR, PC, format|vector.
int M68040::exceptionFrame0(int vector, int cycles) {
    if (onException) onException(vector, instrStart_);
    const u16 oldSR = sr_;
    setSR(static_cast<u16>((sr_ | kS040) & ~(kT1040 | kT0040)));
    push16(static_cast<u16>((vector * 4) & 0xFFF));
    push32(pc);
    push16(oldSR);
    pc = rd32(vbr + static_cast<u32>(vector) * 4);
    if (pc & 1) halted = true;
    return cycles;
}

// Six-word format $2 frame: SR, PC, format|vector, instruction address.
int M68040::exceptionFrame2(int vector, u32 addr, int cycles) {
    if (onException) onException(vector, instrStart_);
    const u16 oldSR = sr_;
    setSR(static_cast<u16>((sr_ | kS040) & ~(kT1040 | kT0040)));
    push32(addr);
    push16(static_cast<u16>(0x2000 | ((vector * 4) & 0xFFF)));
    push32(pc);
    push16(oldSR);
    pc = rd32(vbr + static_cast<u32>(vector) * 4);
    if (pc & 1) halted = true;
    return cycles;
}

int M68040::doInterrupt(int level) {
    stopped = false;
    if (onInterrupt)
        onInterrupt(level, static_cast<u32>(kVec040Autovector + level), pc);
    const u16 oldSR = sr_;
    setSR(static_cast<u16>((sr_ | kS040) & ~(kT1040 | kT0040)));
    sr_ = static_cast<u16>((sr_ & ~0x0700) | (level << 8));
    const int vec = kVec040Autovector + level;
    push16(static_cast<u16>((vec * 4) & 0xFFF));
    push32(pc);
    push16(oldSR);
    const bool masterStack = (sr_ & kM040) != 0;
    if (masterStack) {
        // The interrupt frame went on the master stack; now switch to the
        // interrupt stack and leave a throwaway format $1 frame there. RTE of
        // the throwaway restores M and resumes RTE processing on the master.
        const u16 midSR = sr_;
        setSR(static_cast<u16>(sr_ & ~kM040));
        push16(static_cast<u16>(0x1000 | ((vec * 4) & 0xFFF)));
        push32(pc);
        push16(midSR);
    }
    pc = rd32(vbr + static_cast<u32>(vec) * 4);
    if (pc & 1) halted = true;
    if (is68030()) return masterStack ? 33 : 23;
    return 30;
}

// Access error: the 30-word format $7 frame (M68040UM 8.4.3). We fault with
// restart semantics -- no partial state is committed, the stacked PC is the
// faulting instruction, the writeback fields are pushed invalid -- so an RTE
// simply re-executes the instruction. The Mac ROM's probe handlers adjust the
// stacked PC themselves when they want to skip the faulting access.
int CpuOps040::enterAccessError(M68040& c, const M68040::AccessFault& f) {
    c.lastFaultAddr = f.addr;
    const int vector = (c.is68030() && f.instruction && !f.atc && (f.addr & 1))
                           ? kVec040AddressError
                           : kVec040AccessError;
    if (c.onException) c.onException(vector, c.instrStart_);
    try {
        const u16 oldSR = c.sr_;
        c.setSR(static_cast<u16>((oldSR | kS040) & ~(kT1040 | kT0040)));

        if (c.is68030()) {
            // MC68030 format-$B long bus-fault frame (MC68030UM 8.4).  An
            // operand transfer faults while an instruction is in progress,
            // so the long frame is the conservative restartable form.  The
            // architecturally meaningful fields are populated; undisclosed
            // internal pipeline words are zero, as handlers are required to
            // leave them untouched.
            u16 ssw = 0;
            if (f.instruction) {
                // Stage C fault + rerun.  Address errors have rerun set but no
                // fault bit because no external cycle was attempted.
                ssw = (vector == kVec040AddressError) ? 0x2000u : 0xA000u;
            } else {
                ssw = 0x0100u;                    // DF: rerun data cycle
                if (f.read) ssw |= 0x0040u;       // R/W: 1 = read
                const u16 sz = f.size == 1 ? 1u : f.size == 2 ? 2u : 0u;
                ssw |= static_cast<u16>(sz << 4);
                ssw |= static_cast<u16>((oldSR & kS040) ? 5u : 1u);
            }

            c.a[7] -= 92;                         // format B: 46 words
            const u32 sp = c.a[7];
            c.wr16(sp + 0x00, oldSR);
            c.wr32(sp + 0x02, c.instrStart_);
            c.wr16(sp + 0x06, static_cast<u16>(
                0xB000u | ((vector * 4) & 0x0FFF)));
            c.wr16(sp + 0x08, 0);                 // internal
            c.wr16(sp + 0x0A, ssw);
            c.wr16(sp + 0x0C, 0);                 // pipe stage C image
            c.wr16(sp + 0x0E, c.ir_);             // pipe stage B image
            c.wr32(sp + 0x10, f.addr);            // data-cycle fault address
            c.wr32(sp + 0x18, 0);                 // data output buffer
            c.wr32(sp + 0x24, c.instrStart_ + 4); // stage-B address
            c.wr32(sp + 0x2C, 0);                 // data input buffer
            // All remaining internal words were zeroed by explicit stores so
            // stack contents do not depend on prior RAM bytes.
            for (u32 off = 0x14; off < 0x5C; off += 2) {
                if (off == 0x18 || off == 0x1A || off == 0x24 || off == 0x26 ||
                    off == 0x2C || off == 0x2E)
                    continue;
                c.wr16(sp + off, 0);
            }
            c.pc = c.rd32(c.vbr + static_cast<u32>(vector) * 4);
            if (c.pc & 1) c.halted = true;
            // The restartable format-B path is the manual's long bus-cycle
            // fault operation (MC68030UM 11.6.18).
            return 62;
        }

        // SSW: RW (bit 8), SIZE (bits 6-5: 01 byte, 10 word, 00 long),
        // TM (bits 2-0: data/code, user/super), ATC (bit 10) for MMU misses.
        u16 ssw = 0;
        if (f.read) ssw |= 0x0100;
        if (f.atc)  ssw |= 0x0400;
        const u16 sizeBits = f.size == 1 ? 1u : f.size == 2 ? 2u : 0u;
        ssw |= static_cast<u16>(sizeBits << 5);
        const bool wasS = (oldSR & kS040) != 0;
        u16 tm = f.instruction ? (wasS ? 6u : 2u) : (wasS ? 5u : 1u);
        ssw |= tm;

        for (int i = 0; i < 4; ++i) c.push32(0);   // PD3..PD0 (push data)
        c.push32(0);                               // WB1A
        c.push32(0);                               // WB2D
        c.push32(0);                               // WB2A
        c.push32(0);                               // WB3D
        c.push32(0);                               // WB3A
        c.push32(f.addr);                          // FA (fault address)
        c.push16(0);                               // WB1S (invalid)
        c.push16(0);                               // WB2S
        c.push16(0);                               // WB3S
        c.push16(ssw);
        c.push32(f.addr);                          // effective address
        c.push16(static_cast<u16>(0x7000 | ((kVec040AccessError * 4) & 0xFFF)));
        c.push32(c.instrStart_);                   // restart: the faulting instruction
        c.push16(oldSR);
        c.pc = c.rd32(c.vbr + kVec040AccessError * 4);
        if (c.pc & 1) c.halted = true;
        return 50;
    } catch (const M68040::AccessFault&) {
        c.halted = true;   // fault during fault processing: dead until reset
        return 4;
    }
}

int M68040::step() {
    // Discard reset/debugger traffic from before this instruction.  The
    // returned instruction time includes only penalties incurred by its own
    // fetch, operands, and exception processing.
    (void)bus_.takeCyclePenalty();
    if (halted) return 4;

    const int mask = (sr_ >> 8) & 7;
    if (irqLevel_ > 0 && (irqLevel_ == 7 || irqLevel_ > mask)) {
        const int cycles = doInterrupt(irqLevel_);
        return cycles + bus_.takeCyclePenalty();
    }
    if (stopped) return 4;

    if (onStep) onStep(pc);

    const bool traced = (sr_ & kT1040) != 0;
    instrStart_ = pc;
    pcRing_[pcRingPos_] = pc;
    pcRingPos_ = (pcRingPos_ + 1) & 127;
    for (int i = 0; i < 8; ++i) { snapD_[i] = d[i]; snapA_[i] = a[i]; }
    snapSR_ = sr_;
    eaExtra_ = 0;
    lastEaFull030_ = false;
    try {
        const u16 op = fetch16();
        ir_ = op;
        int cycles = CpuOps040::table(model_)[op](*this, op) + eaExtra_;
        if (traced && !stopped && !halted) {
            cycles += exceptionFrame2(kVec040Trace, instrStart_,
                                      is68030() ? 22 : 25);
        }
        return cycles + bus_.takeCyclePenalty();
    } catch (const AccessFault& f) {
        // Restart semantics: put the register file back the way the
        // instruction found it. Memory side effects stand. The snapshot's
        // a[7] is coherent with the snapshot SR's stack banking.
        for (int i = 0; i < 8; ++i) { d[i] = snapD_[i]; a[i] = snapA_[i]; }
        sr_ = snapSR_;
        if (!is68030() && f.instruction && !f.atc && (f.addr & 1)) {
            // Odd instruction address: address error, format $2 vector 3.
            pc = instrStart_;
            const int cycles = exceptionFrame2(kVec040AddressError, f.addr, 25);
            return cycles + bus_.takeCyclePenalty();
        }
        pc = instrStart_;
        const int cycles = CpuOps040::enterAccessError(*this, f);
        return cycles + bus_.takeCyclePenalty();
    }
}

// ---------------------------------------------------------------- CpuOps040 bits

std::array<CpuOps040::Handler, 65536>& CpuOps040::table(M68kCpuModel model) {
    static std::array<Handler, 65536> t040 = [] {
        std::array<Handler, 65536> tbl{};
        for (auto& h : tbl) h = &CpuOps040::opIllegal;
        buildTableInto(tbl, M68kCpuModel::M68040);
        return tbl;
    }();
    static std::array<Handler, 65536> t030 = [] {
        std::array<Handler, 65536> tbl{};
        for (auto& h : tbl) h = &CpuOps040::opIllegal;
        buildTableInto(tbl, M68kCpuModel::M68030);
        return tbl;
    }();
    return model == M68kCpuModel::M68030 ? t030 : t040;
}

// '040 effective-address EXTRA clocks beyond an instruction's execute-stage
// base (M68040UM section 10 tables). Simple modes overlap entirely with the
// <ea> fetch stage and add nothing; the indexed forms occupy the interlocked
// <ea> calculate stage. Full-format extension words add more via eaExtra_
// (indexExtension), which step() folds into the instruction's total.
int CpuOps040::eaTime(int idx) {
    static constexpr int t[12] = {0, 0, 0, 0, 0, 0, 2, 0, 0, 1, 3, 0};
    return t[idx];
}

// MC68030UM 11.6.1, instruction-cache case, two-clock bus accesses.  Full
// extension-word and memory-indirect supplements are accumulated by
// indexExtension() in eaExtra_; these entries are the single/brief formats.
int CpuOps040::eaFetchTime030(int idx, int size) {
    static constexpr int t[12] = {
        0, 0, 3, 3, 4, 4, 6, 4, 4, 4, 6, 0
    };
    if (idx == 11) return size == 2 ? 4 : 2;
    return t[idx];
}

// MC68030UM 11.6.2.  This stage fetches an immediate source (byte/word = one
// instruction word, long = two) and the destination operand.  Static bit and
// two-word command forms use the word column as well.
int CpuOps040::eaFetchImmediateTime030(M68040& c, int idx, int size) {
    static constexpr int word[12] = {
        2, 2, 3, 5, 4, 4, 6, 6, 6, 4, 6, 4
    };
    static constexpr int lng[12] = {
        4, 4, 4, 7, 4, 6, 8, 8, 8, 6, 8, 6
    };
    return (size == 2 ? lng[idx] : word[idx]) +
           (c.lastEaFull030_ ? 2 : 0);
}

// MC68030UM 11.6.3.  This is address calculation only: no final operand read.
int CpuOps040::eaCalcTime030(M68040& c, int idx) {
    static constexpr int t[12] = {
        0, 0, 2, 2, 2, 2, 4, 2, 4, 2, 4, 0
    };
    return t[idx] + (c.lastEaFull030_ ? 2 : 0);
}

// MC68030UM 11.6.4.  A second instruction word (MOVEM mask, MOVES extension,
// static bit number, etc.) precedes calculation of the destination EA.  The
// common one-word forms are equivalent to the calculate table plus a two-
// clock word fetch, except that (An)+ cannot overlap that fetch.
int CpuOps040::eaCalcImmediateTime030(M68040& c, int idx, int operandWords) {
    static constexpr int oneWord[12] = {
        2, 2, 2, 4, 2, 4, 6, 4, 6, 4, 6, 2
    };
    return oneWord[idx] + (operandWords > 1 ? (operandWords - 1) * 2 : 0) +
           (c.lastEaFull030_ ? 2 : 0);
}

// MC68030UM 11.6.5, single/brief jump effective addresses.
int CpuOps040::eaJumpTime030(M68040&, int idx) {
    static constexpr int t[12] = {
        0, 0, 2, 0, 0, 4, 6, 2, 2, 4, 6, 0
    };
    return t[idx];
}

u32 CpuOps040::addFlags(M68040& c, u32 s, u32 t, int size, bool withX, bool stickyZ) {
    const u32 m = maskFor(size);
    s &= m; t &= m;
    const u32 x = (withX && flag(c, kX040)) ? 1u : 0u;
    const u64 wide = static_cast<u64>(s) + t + x;
    const u32 r = static_cast<u32>(wide) & m;
    const bool carry = wide > m;
    const bool ovf = ((~(s ^ t)) & (s ^ r) & signBit(size)) != 0;
    setFlag(c, kV040, ovf);
    setFlag(c, kC040, carry);
    setFlag(c, kX040, carry);
    setFlag(c, kN040, (r & signBit(size)) != 0);
    if (stickyZ) { if (r != 0) setFlag(c, kZ040, false); }
    else         { setFlag(c, kZ040, r == 0); }
    return r;
}

u32 CpuOps040::subFlags(M68040& c, u32 s, u32 t, int size, bool withX, bool stickyZ) {
    const u32 m = maskFor(size);
    s &= m; t &= m;
    const u32 x = (withX && flag(c, kX040)) ? 1u : 0u;
    const u32 r = (t - s - x) & m;
    const bool borrow = static_cast<u64>(s) + x > t;
    const bool ovf = (((s ^ t) & (t ^ r)) & signBit(size)) != 0;
    setFlag(c, kV040, ovf);
    setFlag(c, kC040, borrow);
    setFlag(c, kX040, borrow);
    setFlag(c, kN040, (r & signBit(size)) != 0);
    if (stickyZ) { if (r != 0) setFlag(c, kZ040, false); }
    else         { setFlag(c, kZ040, r == 0); }
    return r;
}

void CpuOps040::cmpFlags(M68040& c, u32 s, u32 t, int size) {
    const u32 m = maskFor(size);
    s &= m; t &= m;
    const u32 r = (t - s) & m;
    setFlag(c, kV040, (((s ^ t) & (t ^ r)) & signBit(size)) != 0);
    setFlag(c, kC040, s > t);
    setFlag(c, kN040, (r & signBit(size)) != 0);
    setFlag(c, kZ040, r == 0);
}

u32 CpuOps040::readAt(M68040& c, u32 addr, int size) {
    if (size == 0) return c.rd8(addr);
    if (size == 1) return c.rd16(addr);
    return c.rd32(addr);
}

void CpuOps040::writeAt(M68040& c, u32 addr, u32 v, int size) {
    if (size == 0)      c.wr8(addr, static_cast<u8>(v));
    else if (size == 1) c.wr16(addr, static_cast<u16>(v));
    else                c.wr32(addr, v);
}

u32 CpuOps040::fetchImm(M68040& c, int size) {
    if (size == 2) return c.fetch32();
    const u16 w = c.fetch16();
    return size == 0 ? (w & 0xFFu) : w;
}

u32 CpuOps040::readEA(M68040& c, int mode, int reg, int size) {
    if (mode == 0) return c.d[reg] & maskFor(size);
    if (mode == 1) return c.a[reg] & maskFor(size);
    if (mode == 7 && reg == 4) return fetchImm(c, size);
    return readAt(c, calcEA(c, mode, reg, size), size);
}

void CpuOps040::jumpTo(M68040& c, u32 target) {
    c.pc = target;
    if (target & 1) throw M68040::AccessFault{target, true, 2, true, false};
}

bool CpuOps040::testCond(const M68040& c, int cond) {
    const bool n = flag(c, kN040), z = flag(c, kZ040);
    const bool v = flag(c, kV040), cf = flag(c, kC040);
    switch (cond) {
    case 0:  return true;
    case 1:  return false;
    case 2:  return !cf && !z;
    case 3:  return cf || z;
    case 4:  return !cf;
    case 5:  return cf;
    case 6:  return !z;
    case 7:  return z;
    case 8:  return !v;
    case 9:  return v;
    case 10: return !n;
    case 11: return n;
    case 12: return n == v;
    case 13: return n != v;
    case 14: return (n == v) && !z;
    default: return (n != v) || z;
    }
}

} // namespace openmac
