#include "openmac/debugger.hpp"

#include <cstdint>
#include <cstring>

namespace openmac::dbg {

// A-line trap names. An OS trap word is 1010 0 fff tttttttt: the number is the
// low byte and bits 8-10 are per-trap flags (for the Device Manager, $0400 is
// async and $0200 immediate; for the Memory Manager, sys heap and clear). A
// Toolbox trap word is 1010 1 ff nnnnnnnnnn, so its number is the low ten bits.
// Names are what make a trap trace readable, and an unnamed trap in the middle
// of one is the trace's blind spot -- so the OS table is complete rather than
// only the traps some past session happened to need.
const char* trapName(u16 op) {
    if ((op & 0xF000) != 0xA000) return nullptr;
    if (op & 0x0800) {                       // Toolbox trap: number in bits 0-9
        switch (op & 0x03FF) {
            case 0x050: return "_InitCursor";
            case 0x051: return "_SetCursor";
            case 0x052: return "_HideCursor";
            case 0x053: return "_ShowCursor";
            case 0x060: return "_WaitNextEvent";
            case 0x084: return "_DrawString";
            case 0x112: return "_InitWindows";
            case 0x122: return "_BeginUpdate";
            case 0x123: return "_EndUpdate";
            case 0x128: return "_InvalRect";
            case 0x137: return "_DrawMenuBar";
            case 0x13D: return "_MenuSelect";
            case 0x170: return "_GetNextEvent";
            case 0x171: return "_EventAvail";
            case 0x172: return "_GetMouse";
            case 0x173: return "_StillDown";
            case 0x174: return "_Button";
            case 0x175: return "_TickCount";
            case 0x17C: return "_GetNewDialog";
            case 0x185: return "_Alert";
            case 0x186: return "_StopAlert";
            case 0x187: return "_NoteAlert";
            case 0x188: return "_CautionAlert";
            case 0x18B: return "_ParamText";
            case 0x191: return "_ModalDialog";
            case 0x197: return "_OpenResFile";
            case 0x19A: return "_CloseResFile";
            case 0x1A0: return "_GetResource";
            case 0x1A3: return "_ReleaseResource";
            case 0x1B2: return "_SystemEvent";
            case 0x1B4: return "_SystemTask";
            case 0x1C8: return "_SysBeep";
            case 0x1C9: return "_SysError";
            case 0x1E5: return "_InitPack";
            case 0x1E6: return "_InitAllPacks";
            case 0x1E9: return "_Pack2/DiskInit";
            case 0x1EA: return "_Pack3/StdFile";
            case 0x1F0: return "_LoadSeg";
            case 0x1F1: return "_UnLoadSeg";
            case 0x1F4: return "_ExitToShell";
            default:    return "_Toolbox";
        }
    }
    switch (op & 0x00FF) {                    // OS trap: number in bits 0-7
        case 0x00: return "_Open";
        case 0x01: return "_Close";
        case 0x02: return "_Read";
        case 0x03: return "_Write";
        case 0x04: return "_Control";
        case 0x05: return "_Status";
        case 0x06: return "_KillIO";
        case 0x07: return "_GetVolInfo";
        case 0x08: return "_Create";
        case 0x09: return "_Delete";
        case 0x0A: return "_OpenRF";
        case 0x0B: return "_Rename";
        case 0x0C: return "_GetFileInfo";
        case 0x0D: return "_SetFileInfo";
        case 0x0E: return "_UnmountVol";
        case 0x0F: return "_MountVol";
        case 0x10: return "_Allocate";
        case 0x11: return "_GetEOF";
        case 0x12: return "_SetEOF";
        case 0x13: return "_FlushVol";
        case 0x14: return "_GetVol";
        case 0x15: return "_SetVol";
        case 0x16: return "_FInitQueue";
        case 0x17: return "_Eject";
        case 0x18: return "_GetFPos";
        case 0x19: return "_InitZone";
        case 0x1B: return "_SetZone";
        case 0x1C: return "_FreeMem";
        case 0x1D: return "_MaxMem";
        case 0x1E: return "_NewPtr";
        case 0x1F: return "_DisposPtr";
        case 0x20: return "_SetPtrSize";
        case 0x21: return "_GetPtrSize";
        case 0x22: return "_NewHandle";
        case 0x23: return "_DisposHandle";
        case 0x24: return "_SetHandleSize";
        case 0x25: return "_GetHandleSize";
        case 0x26: return "_HandleZone";
        case 0x27: return "_ReallocHandle";
        case 0x28: return "_RecoverHandle";
        case 0x29: return "_HLock";
        case 0x2A: return "_HUnlock";
        case 0x2B: return "_EmptyHandle";
        case 0x2C: return "_InitApplZone";
        case 0x2D: return "_SetApplLimit";
        case 0x2E: return "_BlockMove";
        case 0x2F: return "_PostEvent";
        case 0x30: return "_OSEventAvail";
        case 0x31: return "_GetOSEvent";
        case 0x32: return "_FlushEvents";
        case 0x33: return "_VInstall";
        case 0x34: return "_VRemove";
        case 0x35: return "_OffLine";
        case 0x36: return "_MoreMasters";
        case 0x38: return "_WriteParam";
        case 0x39: return "_ReadDateTime";
        case 0x3A: return "_SetDateTime";
        case 0x3B: return "_Delay";
        case 0x3C: return "_CmpString";
        case 0x3D: return "_DrvrInstall";
        case 0x3E: return "_DrvrRemove";
        case 0x3F: return "_InitUtil";
        case 0x40: return "_ResrvMem";
        case 0x41: return "_SetFilLock";
        case 0x42: return "_RstFilLock";
        case 0x43: return "_SetFilType";
        case 0x44: return "_SetFPos";
        case 0x45: return "_FlushFile";
        case 0x46: return "_GetTrapAddress";
        case 0x47: return "_SetTrapAddress";
        case 0x48: return "_PtrZone";
        case 0x49: return "_HPurge";
        case 0x4A: return "_HNoPurge";
        case 0x4B: return "_SetGrowZone";
        case 0x4C: return "_CompactMem";
        case 0x4D: return "_PurgeMem";
        case 0x4E: return "_AddDrive";
        case 0x4F: return "_RDrvrInstall";
        case 0x50: return "_RelString";
        case 0x51: return "_ReadXPRam";
        case 0x52: return "_WriteXPRam";
        case 0x54: return "_UprString";
        case 0x55: return "_StripAddress";
        case 0x56: return "_LowerText";
        case 0x57: return "_SetAppBase";
        case 0x58: return "_InsTime";
        case 0x59: return "_RmvTime";
        case 0x5A: return "_PrimeTime";
        case 0x5B: return "_PowerOff";
        case 0x5C: return "_MemoryDispatch";
        case 0x5D: return "_SwapMMUMode";
        case 0x5E: return "_NMInstall";
        case 0x5F: return "_NMRemove";
        case 0x60: return "_HFSDispatch";
        case 0x61: return "_MaxBlock";
        case 0x62: return "_PurgeSpace";
        case 0x63: return "_MaxApplZone";
        case 0x64: return "_MoveHHi";
        case 0x65: return "_StackSpace";
        case 0x66: return "_NewEmptyHandle";
        case 0x67: return "_HSetRBit";
        case 0x68: return "_HClrRBit";
        case 0x69: return "_HGetState";
        case 0x6A: return "_HSetState";
        case 0x6C: return "_InitFS";
        case 0x6D: return "_InitEvents";
        case 0x6E: return "_SlotManager";
        case 0x71: return "_AttachVBL";
        case 0x72: return "_DoVBLTask";
        case 0x77: return "_CountADBs";
        case 0x78: return "_GetIndADB";
        case 0x79: return "_GetADBInfo";
        case 0x7A: return "_SetADBInfo";
        case 0x7B: return "_ADBReInit";
        case 0x7C: return "_ADBOp";
        case 0x7D: return "_GetDefaultStartup";
        case 0x7E: return "_SetDefaultStartup";
        case 0x7F: return "_InternalWait";
        case 0x80: return "_GetVideoDefault";
        case 0x81: return "_SetVideoDefault";
        case 0x82: return "_DTInstall";
        case 0x83: return "_SetOSDefault";
        case 0x84: return "_GetOSDefault";
        case 0x85: return "_PMgrOp";
        case 0x89: return "_SCSIAtomic";
        case 0x8A: return "_Sleep";
        case 0x8D: return "_DebugUtil";
        case 0x90: return "_SysEnvirons";
        default:   return "_OSTrap";
    }
}

// OS traps whose result pointer/handle is returned in A0 — a 0 means the
// allocation or lookup failed, which is exactly what produces a NIL bug.
bool trapReturnsPtrInA0(u16 op) {
    if ((op & 0xF800) != 0xA000) return false;   // OS trap only
    switch (op & 0x00FF) {
        case 0x1E: case 0x22: case 0x27: case 0x28: return true;  // New/Realloc/Recover
        default: return false;
    }
}

namespace {
const LowMem kGlobals[] = {
    {0x0108, "MemTop",     4}, {0x010C, "BufPtr",     4}, {0x0114, "HeapEnd",   4},
    {0x0118, "TheZone",    4}, {0x011C, "UTableBase", 4}, {0x0126, "MinStack",  4},
    {0x0130, "ApplLimit",  4}, {0x0134, "SonyVars",   4}, {0x0144, "SysEvtMask",2},
    {0x014A, "EventQueue",10}, {0x0154, "EvtBufCnt",  2}, {0x015C, "SysVersion",2},
    {0x016A, "Ticks",      4}, {0x0174, "KeyMap",     4}, {0x0210, "BootDrive", 2},
    {0x02AA, "ApplZone",   4}, {0x02AE, "ROMBase",    4}, {0x02B2, "RAMBase",   4},
    {0x02E0, "ToExtFS",    4}, {0x0308, "DrvQHdr",   10}, {0x0312, "EjectNotify",4},
    {0x0316, "IAZNotify",  4}, {0x034E, "FCBSPtr",    4}, {0x0352, "DefVCBPtr", 4},
    {0x0356, "VCBQHdr",   10}, {0x0360, "FSQHdr",    10}, {0x0824, "ScrnBase",  4},
    {0x08FC, "JIODone",    4}, {0x0904, "CurrentA5",  4}, {0x0910, "CurApName", 4},
};
} // namespace

std::string symbolFor(Machine& mac, u32 addr) {
    addr &= 0xFFFFFF;
    for (const auto& g : kGlobals) {          // low-memory globals (data)
        const u32 sz = g.size == 10 ? 10u : static_cast<u32>(g.size);
        if (addr >= g.addr && addr < g.addr + sz) {
            char b[40];
            if (addr == g.addr) std::snprintf(b, sizeof b, "%s", g.name);
            else std::snprintf(b, sizeof b, "%s+%X", g.name, addr - g.addr);
            return b;
        }
    }
    if (addr >= 0x400000) {                   // named OS trap handlers ($0400 table)
        u32 bestBase = 0;
        const char* bestName = nullptr;
        for (int i = 0; i < 256; ++i) {
            const char* nm = trapName(static_cast<u16>(0xA000 | i));
            if (!nm || std::strcmp(nm, "_OSTrap") == 0) continue;   // named only
            const u32 h = ((u32(mac.read16(0x0400 + i * 4)) << 16) |
                           mac.read16(0x0400 + i * 4 + 2)) & 0xFFFFFF;
            if (h >= 0x400000 && h <= addr && addr - h < 0x1000 && h > bestBase) {
                bestBase = h;
                bestName = nm;
            }
        }
        if (bestName) {
            char b[48];
            if (addr == bestBase) std::snprintf(b, sizeof b, "%s", bestName);
            else std::snprintf(b, sizeof b, "%s+%X", bestName, addr - bestBase);
            return b;
        }
    }
    return "";
}

void dumpRegs(const M68000& cpu, std::FILE* out) {
    for (int i = 0; i < 8; ++i)
        std::fprintf(out, "D%d=%08X%s", i, cpu.d[i], (i == 3 || i == 7) ? "\n" : " ");
    for (int i = 0; i < 8; ++i)
        std::fprintf(out, "A%d=%08X%s", i, cpu.a[i], (i == 3 || i == 7) ? "\n" : " ");
    const u16 sr = cpu.getSR();
    std::fprintf(out, "PC=%06X SR=%04X [%c%c%c%c%c] %s\n", cpu.pc, sr,
                 sr & 0x10 ? 'X' : '-', sr & 0x08 ? 'N' : '-', sr & 0x04 ? 'Z' : '-',
                 sr & 0x02 ? 'V' : '-', sr & 0x01 ? 'C' : '-',
                 sr & 0x2000 ? "supervisor" : "user");
}

void dumpLowMem(Machine& mac, std::FILE* out) {
    std::fprintf(out, "-- low-memory globals --\n");
    for (const auto& g : kGlobals) {
        std::fprintf(out, "  %-10s $%04X = ", g.name, g.addr);
        if (g.size == 2) {
            std::fprintf(out, "%04X\n", mac.read16(g.addr));
        } else if (g.size == 10) {   // QHdr: flags, head, tail
            const u32 head = (u32(mac.read16(g.addr + 2)) << 16) | mac.read16(g.addr + 4);
            const u32 tail = (u32(mac.read16(g.addr + 6)) << 16) | mac.read16(g.addr + 8);
            std::fprintf(out, "flags=%04X head=%06X tail=%06X\n", mac.read16(g.addr),
                         head, tail);
        } else {
            std::fprintf(out, "%08X\n",
                         (u32(mac.read16(g.addr)) << 16) | mac.read16(g.addr + 2));
        }
    }
}

// ---- compact disassembler ------------------------------------------------

namespace {

struct Cursor {
    const ReadWord& rd;
    u32 p;
    u16 word() { const u16 v = rd(p); p += 2; return v; }
    u32 lng()  { const u32 v = (u32(rd(p)) << 16) | rd(p + 2); p += 4; return v; }
};

// An address as a listing writes it. The 68000-era machines live below 16 MB
// and read best in six digits; a Quadra's ROM sits at $40800000, and masking
// that to 24 bits turns every branch target into a different address. So the
// width follows the value instead of the era.
std::string hexAddr(u32 a) {
    char b[16];
    std::snprintf(b, sizeof b, a <= 0xFFFFFFu ? "$%06X" : "$%08X", a);
    return b;
}

const char* kSizes[] = {".b", ".w", ".l"};
const char* kCond[] = {"T","F","HI","LS","CC","CS","NE","EQ",
                       "VC","VS","PL","MI","GE","LT","GT","LE"};

// A signed 16-bit displacement, written the way it reads in a listing.
std::string disp16(u16 v) {
    char b[16];
    const int d = static_cast<int16_t>(v);
    std::snprintf(b, sizeof b, "%s$%04X", d < 0 ? "-" : "", d < 0 ? -d : d);
    return b;
}

// An indexed operand: (d8,An,Xn) in the 68000's brief form, or the 68020's
// full format, which the Mac's own system software uses freely (a driver
// dispatching through `JSR ([$0174,A5])` is one word longer than the brief
// form). Getting the LENGTH wrong is worse than getting the text wrong: the
// listing silently loses alignment and every instruction after it is fiction.
//
// Reference: M68000PRM 2.2, "Effective Address Encoding Summary".
std::string extended(Cursor& c, const char* base) {
    // For a PC-relative operand the displacement is counted from the extension
    // word itself, and resolving it here is what makes a ROM listing readable:
    // an address you have to add by hand is a branch you will misread.
    const bool pcRel = base[0] == 'P';
    const u32 pcBase = c.p;
    const u16 ext = c.word();
    // Assembled as a string rather than into a fixed buffer: an operand that
    // silently loses its tail is a listing that reads plausibly and names the
    // wrong address, and the widths here depend on four independent fields.
    char nb[16];
    std::snprintf(nb, sizeof nb, "%c%d%s", (ext & 0x8000) ? 'A' : 'D',
                  (ext >> 12) & 7, (ext & 0x0800) ? ".l" : ".w");
    std::string idx = nb;
    switch ((ext >> 9) & 3) {
        case 1: idx += "*2"; break;
        case 2: idx += "*4"; break;
        case 3: idx += "*8"; break;
        default: break;
    }
    if (!(ext & 0x0100)) {                       // brief format, one word
        const s32 d8 = static_cast<int8_t>(ext & 0xFF);
        if (pcRel)
            return hexAddr(pcBase + static_cast<u32>(d8)) + "(PC," + idx + ")";
        return disp16(static_cast<u16>(d8)) + "(" + base + "," + idx + ")";
    }
    // Full format. The extension words that follow are a base displacement of
    // 0/1/2 words (bits 5-4) and then an outer displacement of 0/1/2 (bits
    // 2-0), and both have to be consumed whether or not they are printed.
    const int bdSize = (ext >> 4) & 3;
    const int iis = ext & 7;
    const bool baseSup = (ext & 0x0080) != 0, indexSup = (ext & 0x0040) != 0;
    s32 bd = 0;
    if (bdSize == 2) bd = static_cast<int16_t>(c.word());
    else if (bdSize == 3) bd = static_cast<s32>(c.lng());
    s32 od = 0;
    const int odSize = indexSup ? iis & 3 : (iis & 3);
    const bool memIndirect = (iis & 7) != 0;
    if (memIndirect) {
        if (odSize == 2) od = static_cast<int16_t>(c.word());
        else if (odSize == 3) od = static_cast<s32>(c.lng());
    }
    // Postindexed puts the index outside the indirection; preindexed inside.
    const bool post = !indexSup && (iis & 4) != 0;
    const bool innerIdx = !indexSup && !post;
    std::string inner;
    if (pcRel && !baseSup)
        inner = hexAddr(pcBase + static_cast<u32>(bd)) + "(PC)";
    else {
        if (bd) inner += disp16(static_cast<u16>(bd));
        if (bd && !baseSup) inner += ",";
        if (!baseSup) inner += base;
    }
    if (innerIdx) inner += "," + idx;
    const std::string odText = od ? "," + disp16(static_cast<u16>(od)) : std::string();
    if (!memIndirect) return "(" + inner + ")";
    if (post) return "([" + inner + "]," + idx + odText + ")";
    return "([" + inner + "]" + odText + ")";
}

// Decode an effective address, appending text and consuming extension words.
// `size` is 0/1/2 for byte/word/long and only matters for an immediate.
//
// PC-relative operands are resolved to the address they name rather than left
// as the raw displacement: a ROM listing is nearly all `JSR $xxxx(PC)`, and a
// displacement you have to add by hand is what makes reading one slow.
std::string ea(Cursor& c, int mode, int reg, int size) {
    char b[48];
    switch (mode) {
        case 0: std::snprintf(b, sizeof b, "D%d", reg); break;
        case 1: std::snprintf(b, sizeof b, "A%d", reg); break;
        case 2: std::snprintf(b, sizeof b, "(A%d)", reg); break;
        case 3: std::snprintf(b, sizeof b, "(A%d)+", reg); break;
        case 4: std::snprintf(b, sizeof b, "-(A%d)", reg); break;
        case 5: { const std::string d = disp16(c.word());
                  std::snprintf(b, sizeof b, "%s(A%d)", d.c_str(), reg); break; }
        case 6: { char an[8];
                  std::snprintf(an, sizeof an, "A%d", reg);
                  return extended(c, an); }
        case 7:
            switch (reg) {
                // Absolute short sign-extends: $8000..$FFFF names the top of
                // the address space, which is where low memory's mirrors and
                // the I/O page live on these machines.
                case 0: std::snprintf(b, sizeof b, "%s.W",
                                      hexAddr(static_cast<u32>(static_cast<int16_t>(c.word()))).c_str());
                        break;
                case 1: std::snprintf(b, sizeof b, "%s", hexAddr(c.lng()).c_str()); break;
                case 2: { const u32 base = c.p;
                          const u32 t = base + static_cast<u32>(static_cast<int16_t>(c.word()));
                          std::snprintf(b, sizeof b, "%s(PC)", hexAddr(t).c_str()); break; }
                case 3: return extended(c, "PC");
                case 4:
                    if (size == 2) std::snprintf(b, sizeof b, "#$%08X", c.lng());
                    else if (size == 0) std::snprintf(b, sizeof b, "#$%02X", c.word() & 0xFF);
                    else std::snprintf(b, sizeof b, "#$%04X", c.word());
                    break;
                default: std::snprintf(b, sizeof b, "?"); break;
            }
            break;
        default: std::snprintf(b, sizeof b, "?"); break;
    }
    return b;
}

// The MOVEM register list, which is a bitmap whose bit order depends on the
// direction: -(An) numbers it A7..A0,D7..D0, everything else D0..D7,A0..A7.
std::string regList(u16 mask, bool predec) {
    std::string out;
    for (int i = 0; i < 16; ++i) {
        const int bit = predec ? 15 - i : i;
        if (!(mask & (1u << bit))) continue;
        int run = i;
        while (run + 1 < 16) {
            const int nb = predec ? 15 - (run + 1) : (run + 1);
            if (!(mask & (1u << nb)) || ((run + 1) == 8)) break;   // don't span D7->A0
            ++run;
        }
        char b[16];
        const char* kind0 = i < 8 ? "D" : "A";
        if (run == i) std::snprintf(b, sizeof b, "%s%d", kind0, i & 7);
        else std::snprintf(b, sizeof b, "%s%d-%s%d", kind0, i & 7, kind0, run & 7);
        if (!out.empty()) out += "/";
        out += b;
        i = run;
    }
    return out.empty() ? std::string("-") : out;
}

} // namespace

// A 68000 disassembler covering the whole instruction set. It exists to read
// this ROM: an unrecognised word printed as DC.W is not a gap in cosmetics, it
// is a line of the routine under investigation that has to be decoded by hand,
// and every one of those is a chance to misread a branch.
int disasm(Machine& mac, u32 pc, std::string& out) {
    const ReadWord rd = [&mac](u32 a) { return mac.read16(a); };
    return disasm(rd, pc, out);
}

int disasm(const ReadWord& rd, u32 pc, std::string& out) {
    Cursor c{rd, pc};
    const u16 op = c.word();
    char buf[128];
    const int mode = (op >> 3) & 7, reg = op & 7;
    const int dreg = (op >> 9) & 7;

    auto emit = [&](const char* s) { out += s; };
    auto emit1 = [&](const char* mnem, const std::string& a) {
        std::snprintf(buf, sizeof buf, "%-8s %s", mnem, a.c_str());
        out += buf;
    };
    auto emit2 = [&](const char* mnem, const std::string& a, const std::string& b) {
        std::snprintf(buf, sizeof buf, "%-8s %s,%s", mnem, a.c_str(), b.c_str());
        out += buf;
    };
    auto sized = [&](const char* stem, int sz) {
        static char nb[16];
        std::snprintf(nb, sizeof nb, "%s%s", stem, kSizes[sz & 3]);
        return nb;
    };

    switch (op >> 12) {
    case 0x0: {                                     // immediate / bit / MOVEP
        if ((op & 0xF138) == 0x0108) {              // MOVEP
            const std::string d = disp16(c.word());
            const bool toMem = (op & 0x0080) != 0;
            char mem[24];
            std::snprintf(mem, sizeof mem, "%s(A%d)", d.c_str(), reg);
            char dn[8];
            std::snprintf(dn, sizeof dn, "D%d", dreg);
            std::snprintf(buf, sizeof buf, "MOVEP%-3s %s,%s", (op & 0x40) ? ".l" : ".w",
                          toMem ? dn : mem, toMem ? mem : dn);
            emit(buf);
            break;
        }
        if ((op & 0xFF00) == 0x0800 || (op & 0xF100) == 0x0100) {   // BTST/BCHG/BCLR/BSET
            static const char* bops[] = {"BTST","BCHG","BCLR","BSET"};
            const bool stat = (op & 0xFF00) == 0x0800;
            std::string src;
            if (stat) { char t[12]; std::snprintf(t, sizeof t, "#%d", c.word() & 0x1F); src = t; }
            else      { char t[12]; std::snprintf(t, sizeof t, "D%d", dreg); src = t; }
            emit2(bops[(op >> 6) & 3], src, ea(c, mode, reg, 0));
            break;
        }
        static const char* iops[] = {"ORI","ANDI","SUBI","ADDI",nullptr,"EORI","CMPI",nullptr};
        const char* im = iops[(op >> 9) & 7];
        const int sz = (op >> 6) & 3;
        if (!im || sz == 3) { std::snprintf(buf, sizeof buf, "DC.W     $%04X", op); emit(buf); break; }
        if (mode == 7 && reg == 4) {                // ORI/ANDI/EORI to CCR or SR
            const bool toSr = (sz == 1);
            std::snprintf(buf, sizeof buf, "%-8s #$%04X,%s", im,
                          static_cast<unsigned>(c.word()), toSr ? "SR" : "CCR");
            emit(buf);
            break;
        }
        const std::string s = ea(c, 7, 4, sz);      // the immediate first
        emit2(sized(im, sz), s, ea(c, mode, reg, sz));
        break;
    }
    case 0x1: case 0x2: case 0x3: {                 // MOVE / MOVEA
        const int sz = (op >> 12) == 1 ? 0 : (op >> 12) == 3 ? 1 : 2;
        const std::string s = ea(c, mode, reg, sz);
        const int dm = (op >> 6) & 7;
        const std::string d = ea(c, dm, dreg, sz);
        emit2(sized(dm == 1 ? "MOVEA" : "MOVE", sz), s, d);
        break;
    }
    case 0x4: {                                     // miscellaneous
        if ((op & 0xFFC0) == 0x40C0) { emit2("MOVE", "SR", ea(c, mode, reg, 1)); break; }
        if ((op & 0xFFC0) == 0x44C0) { emit2("MOVE", ea(c, mode, reg, 1), "CCR"); break; }
        if ((op & 0xFFC0) == 0x46C0) { emit2("MOVE", ea(c, mode, reg, 1), "SR"); break; }
        if (op == 0x4E70) { emit("RESET"); break; }
        if (op == 0x4E71) { emit("NOP"); break; }
        if (op == 0x4E72) { std::snprintf(buf, sizeof buf, "STOP     #$%04X",
                                          static_cast<unsigned>(c.word())); emit(buf); break; }
        if (op == 0x4E73) { emit("RTE"); break; }
        if (op == 0x4E75) { emit("RTS"); break; }
        if (op == 0x4E76) { emit("TRAPV"); break; }
        if (op == 0x4E77) { emit("RTR"); break; }
        if ((op & 0xFFF0) == 0x4E40) { std::snprintf(buf, sizeof buf, "TRAP     #%d", op & 15);
                                       emit(buf); break; }
        if ((op & 0xFFF8) == 0x4E50) { std::snprintf(buf, sizeof buf, "LINK     A%d,#%s", reg,
                                                     disp16(c.word()).c_str()); emit(buf); break; }
        if ((op & 0xFFF8) == 0x4E58) { std::snprintf(buf, sizeof buf, "UNLK     A%d", reg);
                                       emit(buf); break; }
        if ((op & 0xFFF8) == 0x4E60) { std::snprintf(buf, sizeof buf, "MOVE     A%d,USP", reg);
                                       emit(buf); break; }
        if ((op & 0xFFF8) == 0x4E68) { std::snprintf(buf, sizeof buf, "MOVE     USP,A%d", reg);
                                       emit(buf); break; }
        if ((op & 0xFFC0) == 0x4E80) { emit1("JSR", ea(c, mode, reg, 2)); break; }
        if ((op & 0xFFC0) == 0x4EC0) { emit1("JMP", ea(c, mode, reg, 2)); break; }
        if ((op & 0xF1C0) == 0x41C0) { const std::string s = ea(c, mode, reg, 2);
                                       char t[8]; std::snprintf(t, sizeof t, "A%d", dreg);
                                       emit2("LEA", s, t); break; }
        if ((op & 0xF1C0) == 0x4180) { const std::string s = ea(c, mode, reg, 1);
                                       char t[8]; std::snprintf(t, sizeof t, "D%d", dreg);
                                       emit2("CHK", s, t); break; }
        if ((op & 0xFFF8) == 0x4840) { std::snprintf(buf, sizeof buf, "SWAP     D%d", reg);
                                       emit(buf); break; }
        if ((op & 0xFFB8) == 0x4880) { std::snprintf(buf, sizeof buf, "EXT%s     D%d",
                                                     (op & 0x40) ? ".l" : ".w", reg);
                                       emit(buf); break; }
        if ((op & 0xFB80) == 0x4880) {              // MOVEM
            const u16 mask = c.word();
            const bool toReg = (op & 0x0400) != 0;
            const char* nm = (op & 0x40) ? "MOVEM.l" : "MOVEM.w";
            const std::string m = ea(c, mode, reg, 2);
            if (toReg) emit2(nm, m, regList(mask, false));
            else       emit2(nm, regList(mask, mode == 4), m);
            break;
        }
        if ((op & 0xFFC0) == 0x4800) { emit1("NBCD", ea(c, mode, reg, 0)); break; }
        if ((op & 0xFFC0) == 0x4840) { emit1("PEA", ea(c, mode, reg, 2)); break; }
        if (op == 0x4AFC) { emit("ILLEGAL"); break; }
        if ((op & 0xFFC0) == 0x4AC0) { emit1("TAS", ea(c, mode, reg, 0)); break; }
        if ((op & 0xFF00) == 0x4A00) { const int sz = (op >> 6) & 3;
                                       if (sz == 3) { std::snprintf(buf, sizeof buf, "DC.W     $%04X", op); emit(buf); break; }
                                       emit1(sized("TST", sz), ea(c, mode, reg, sz)); break; }
        {   // NEGX / CLR / NEG / NOT
            static const char* uops[] = {"NEGX","CLR","NEG","NOT"};
            const int which = (op >> 9) & 7;
            const int sz = (op >> 6) & 3;
            if (which < 4 && sz != 3) { emit1(sized(uops[which], sz), ea(c, mode, reg, sz)); break; }
        }
        std::snprintf(buf, sizeof buf, "DC.W     $%04X", op);
        emit(buf);
        break;
    }
    case 0x5: {                                     // ADDQ / SUBQ / Scc / DBcc
        if ((op & 0x00C0) == 0x00C0) {
            const int cond = (op >> 8) & 15;
            if (mode == 1) {                        // DBcc
                const u32 base = c.p;
                const u32 t = base + static_cast<u32>(static_cast<int16_t>(c.word()));
                std::snprintf(buf, sizeof buf, "DB%-6s D%d,%s", kCond[cond], reg,
                              hexAddr(t).c_str());
                emit(buf);
            } else {
                char nm[12];
                std::snprintf(nm, sizeof nm, "S%s", kCond[cond]);
                emit1(nm, ea(c, mode, reg, 0));
            }
            break;
        }
        const int sz = (op >> 6) & 3;
        const int q = dreg == 0 ? 8 : dreg;
        char imm[8];
        std::snprintf(imm, sizeof imm, "#%d", q);
        emit2(sized((op & 0x0100) ? "SUBQ" : "ADDQ", sz), imm, ea(c, mode, reg, sz));
        break;
    }
    case 0x6: {                                     // Bcc / BRA / BSR
        static const char* bcc[] = {"BRA","BSR","BHI","BLS","BCC","BCS","BNE","BEQ",
                                    "BVC","BVS","BPL","BMI","BGE","BLT","BGT","BLE"};
        int d = static_cast<int8_t>(op & 0xFF);
        const u32 base = c.p;
        if ((op & 0xFF) == 0x00) d = static_cast<int16_t>(c.word());
        else if ((op & 0xFF) == 0xFF) d = static_cast<int32_t>(c.lng());   // 68020+ long form
        std::snprintf(buf, sizeof buf, "%-8s %s", bcc[(op >> 8) & 0xF],
                      hexAddr(base + static_cast<u32>(d)).c_str());
        emit(buf);
        break;
    }
    case 0x7:
        std::snprintf(buf, sizeof buf, "MOVEQ    #$%02X,D%d",
                      static_cast<unsigned>(op & 0xFF), dreg);
        emit(buf);
        break;
    case 0x8: case 0x9: case 0xB: case 0xC: case 0xD: {
        const int grp = op >> 12;
        const char* base = grp == 0x8 ? "OR" : grp == 0x9 ? "SUB"
                         : grp == 0xB ? "CMP" : grp == 0xC ? "AND" : "ADD";
        const int opmode = (op >> 6) & 7;
        char dn[8]; std::snprintf(dn, sizeof dn, "D%d", dreg);
        char an[8]; std::snprintf(an, sizeof an, "A%d", dreg);
        if (opmode == 3 || opmode == 7) {           // xxxA.w / xxxA.l
            if (grp == 0x8 || grp == 0xC) {         // DIVU/DIVS or MULU/MULS
                const char* nm = grp == 0x8 ? (opmode == 3 ? "DIVU" : "DIVS")
                                            : (opmode == 3 ? "MULU" : "MULS");
                emit2(nm, ea(c, mode, reg, 1), dn);
                break;
            }
            char nm[12];
            std::snprintf(nm, sizeof nm, "%sA%s", base, opmode == 3 ? ".w" : ".l");
            emit2(nm, ea(c, mode, reg, opmode == 3 ? 1 : 2), an);
            break;
        }
        const int sz = opmode & 3;
        if (opmode >= 4) {
            if (grp == 0xB && mode == 1) {          // CMPM
                char sa[12], da[12];
                std::snprintf(sa, sizeof sa, "(A%d)+", reg);
                std::snprintf(da, sizeof da, "(A%d)+", dreg);
                emit2(sized("CMPM", sz), sa, da);
                break;
            }
            if ((grp == 0x9 || grp == 0xD) && (mode == 0 || mode == 1)) {   // SUBX/ADDX
                char sa[12], da[12];
                if (mode == 0) { std::snprintf(sa, sizeof sa, "D%d", reg);
                                 std::snprintf(da, sizeof da, "D%d", dreg); }
                else           { std::snprintf(sa, sizeof sa, "-(A%d)", reg);
                                 std::snprintf(da, sizeof da, "-(A%d)", dreg); }
                emit2(sized(grp == 0x9 ? "SUBX" : "ADDX", sz), sa, da);
                break;
            }
            if ((grp == 0x8 || grp == 0xC) && opmode == 4 && (mode == 0 || mode == 1)) {
                char sa[12], da[12];                // SBCD / ABCD
                if (mode == 0) { std::snprintf(sa, sizeof sa, "D%d", reg);
                                 std::snprintf(da, sizeof da, "D%d", dreg); }
                else           { std::snprintf(sa, sizeof sa, "-(A%d)", reg);
                                 std::snprintf(da, sizeof da, "-(A%d)", dreg); }
                emit2(grp == 0x8 ? "SBCD" : "ABCD", sa, da);
                break;
            }
            if (grp == 0xC && (op & 0x0130) == 0x0100) {                    // EXG
                const int m = (op >> 3) & 0x1F;
                char sa[8], da[8];
                if (m == 0x08)      { std::snprintf(sa, sizeof sa, "D%d", dreg);
                                      std::snprintf(da, sizeof da, "D%d", reg); }
                else if (m == 0x09) { std::snprintf(sa, sizeof sa, "A%d", dreg);
                                      std::snprintf(da, sizeof da, "A%d", reg); }
                else                { std::snprintf(sa, sizeof sa, "D%d", dreg);
                                      std::snprintf(da, sizeof da, "A%d", reg); }
                emit2("EXG", sa, da);
                break;
            }
            const char* nm = grp == 0xB ? "EOR" : base;
            emit2(sized(nm, sz), dn, ea(c, mode, reg, sz));
            break;
        }
        emit2(sized(base, sz), ea(c, mode, reg, sz), dn);
        break;
    }
    case 0xE: {                                     // shifts and rotates
        static const char* sops[] = {"AS","LS","ROX","RO"};
        if ((op & 0x00C0) == 0x00C0) {              // memory, by one
            char nm[12];
            std::snprintf(nm, sizeof nm, "%s%c.w", sops[(op >> 9) & 3], (op & 0x0100) ? 'L' : 'R');
            emit1(nm, ea(c, mode, reg, 1));
            break;
        }
        const int sz = (op >> 6) & 3;
        char nm[12];
        std::snprintf(nm, sizeof nm, "%s%c%s", sops[(op >> 3) & 3], (op & 0x0100) ? 'L' : 'R',
                      kSizes[sz & 3]);
        char cnt[8];
        if (op & 0x0020) std::snprintf(cnt, sizeof cnt, "D%d", dreg);
        else             std::snprintf(cnt, sizeof cnt, "#%d", dreg == 0 ? 8 : dreg);
        char dst[8]; std::snprintf(dst, sizeof dst, "D%d", reg);
        emit2(nm, cnt, dst);
        break;
    }
    case 0xA: {
        const char* n = trapName(op);
        std::snprintf(buf, sizeof buf, "%-8s ; $%04X", n ? n : "_Axxx", op);
        emit(buf);
        break;
    }
    default:
        std::snprintf(buf, sizeof buf, "DC.W     $%04X", op);
        emit(buf);
        break;
    }
    return static_cast<int>(c.p - pc);
}

void dumpMem(Machine& mac, u32 addr, u32 len, std::FILE* out) {
    for (u32 row = 0; row < len; row += 16) {
        std::fprintf(out, "  %06X: ", addr + row);
        char ascii[17] = {0};
        for (u32 i = 0; i < 16; ++i) {
            if (row + i < len) {
                const u8 b = mac.read8(addr + row + i);
                std::fprintf(out, "%02X ", b);
                ascii[i] = (b >= 0x20 && b < 0x7F) ? char(b) : '.';
            } else {
                std::fprintf(out, "   ");
                ascii[i] = ' ';
            }
        }
        std::fprintf(out, " %s\n", ascii);
    }
}

// ---- struct-templated memory display -------------------------------------

namespace {

// One field of a Mac OS data structure: a byte/word/long at a fixed offset.
// `ptr` marks longs worth resolving to a symbol (they hold an address).
enum FieldKind { kByte = 1, kWord = 2, kLong = 4 };
struct StructField { const char* name; u32 off; FieldKind kind; bool ptr; };

// Offsets from Inside Macintosh, cross-checked against the enum in machine.cpp
// (ioResult/dCtlPosition/dsDiskInPlace/tmCount) so the decode matches the code.
const StructField kTMTask[] = {
    {"qLink",      0x00, kLong, true },
    {"qType",      0x04, kWord, false},
    {"tmAddr",     0x06, kLong, true },
    {"tmCount",    0x0A, kLong, false},
    {"tmWakeUp",   0x0E, kLong, false},
    {"tmReserved", 0x12, kLong, false},
};
const StructField kQHdr[] = {
    {"qFlags", 0x00, kWord, false},
    {"qHead",  0x02, kLong, true },
    {"qTail",  0x06, kLong, true },
};
const StructField kIOParam[] = {
    {"qLink",       0x00, kLong, true },
    {"qType",       0x04, kWord, false},
    {"ioTrap",      0x06, kWord, false},
    {"ioCmdAddr",   0x08, kLong, true },
    {"ioResult",    0x10, kWord, false},
    {"ioNamePtr",   0x12, kLong, true },
    {"ioVRefNum",   0x16, kWord, false},
    {"ioRefNum",    0x18, kWord, false},
    {"ioBuffer",    0x20, kLong, true },
    {"ioReqCount",  0x24, kLong, false},
    {"ioActCount",  0x28, kLong, false},
    {"ioPosMode",   0x2C, kWord, false},
    {"ioPosOffset", 0x2E, kLong, false},
};
const StructField kDCE[] = {
    {"dCtlDriver",      0x00, kLong, true },
    {"dCtlFlags",       0x04, kWord, false},
    {"dCtlQHdr.qFlags", 0x06, kWord, false},   // dCtlQHdr is a 10-byte QHdr
    {"dCtlQHdr.qHead",  0x08, kLong, true },
    {"dCtlQHdr.qTail",  0x0C, kLong, true },
    {"dCtlPosition",    0x10, kLong, false},
    {"dCtlStorage",     0x14, kLong, true },
    {"dCtlRefNum",      0x18, kWord, false},
    {"dCtlCurTicks",    0x1A, kLong, false},
    {"dCtlWindow",      0x1E, kLong, true },
    {"dCtlDelay",       0x22, kWord, false},
};
// dsTwoSideFmt is at +$12 (18) per Inside Macintosh and machine.cpp's DrvSts
// enum; the queue fields dQDrive/dQRefNum/dQFSID occupy $0C..$11 before it.
const StructField kDrvSts[] = {
    {"dsTrack",       0x00, kWord, false},
    {"dsWriteProt",   0x02, kByte, false},
    {"dsDiskInPlace", 0x03, kByte, false},
    {"dsInstalled",   0x04, kByte, false},
    {"dsSides",       0x05, kByte, false},
    {"dsQLink",       0x06, kLong, true },
    {"dsQType",       0x0A, kWord, false},
    {"dsTwoSideFmt",  0x12, kByte, false},
};

} // namespace

void dumpStruct(Machine& mac, u32 addr, const char* name, std::FILE* out) {
    const StructField* fields = nullptr;
    size_t count = 0;
    if      (std::strcmp(name, "TMTask")  == 0) { fields = kTMTask;  count = sizeof kTMTask  / sizeof kTMTask[0]; }
    else if (std::strcmp(name, "QHdr")    == 0) { fields = kQHdr;    count = sizeof kQHdr    / sizeof kQHdr[0]; }
    else if (std::strcmp(name, "IOParam") == 0) { fields = kIOParam; count = sizeof kIOParam / sizeof kIOParam[0]; }
    else if (std::strcmp(name, "DCE")     == 0) { fields = kDCE;     count = sizeof kDCE     / sizeof kDCE[0]; }
    else if (std::strcmp(name, "DrvSts")  == 0) { fields = kDrvSts;  count = sizeof kDrvSts  / sizeof kDrvSts[0]; }
    else {
        std::fprintf(out, "-- struct '%s' @%06X: unknown (want TMTask|QHdr|IOParam|DCE|DrvSts) --\n",
                     name, addr & 0xFFFFFF);
        return;
    }
    std::fprintf(out, "-- %s @%06X --\n", name, addr & 0xFFFFFF);
    for (size_t i = 0; i < count; ++i) {
        const StructField& f = fields[i];
        const u32 a = (addr + f.off) & 0xFFFFFF;
        std::fprintf(out, "  +%02X  %-16s ", f.off, f.name);
        if (f.kind == kByte) {
            std::fprintf(out, "B  %02X\n", mac.read8(a));
        } else if (f.kind == kWord) {
            std::fprintf(out, "W  %04X\n", mac.read16(a));
        } else {
            const u32 v = (u32(mac.read16(a)) << 16) | mac.read16(a + 2);
            std::fprintf(out, "L  %08X", v);
            if (f.ptr) {
                const std::string s = symbolFor(mac, v);
                if (!s.empty()) std::fprintf(out, "  %s", s.c_str());
            }
            std::fprintf(out, "\n");
        }
    }
}

void dumpDriveQueue(Machine& mac, std::FILE* out) {
    auto r32 = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
    std::fprintf(out, "-- drive queue ($0308) --\n");
    u32 el = r32(0x030A);   // qHead
    int guard = 0;
    if (el == 0) { std::fprintf(out, "  (empty)\n"); return; }
    while (el && guard++ < 16) {
        // A DrvQEl is preceded by 4 flag bytes; the queue links at el.
        const u16 drive = mac.read16(el + 6);
        const u16 ref   = mac.read16(el + 8);
        const u32 size  = r32(el + 12);
        std::fprintf(out, "  drive=%u refNum=%d (0x%04X) blocks=%u @%06X\n", drive,
                     int16_t(ref), ref, size, el);
        el = r32(el);       // qLink
    }
}

void dumpUnitTable(Machine& mac, std::FILE* out) {
    auto r32 = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
    const u32 base = r32(0x011C);      // UTableBase
    const u16 count = mac.read16(0x01D2);   // UnitNtryCnt
    std::fprintf(out, "-- unit table (%u entries @%06X) --\n", count, base);
    for (u16 i = 0; i < count && i < 48; ++i) {
        const u32 dce = r32(base + i * 4);
        if (dce == 0) continue;
        const u16 flags = mac.read16(dce + 4);      // dCtlFlags
        const u32 qHead = r32(dce + 8);             // dCtlQHdr.qHead
        std::fprintf(out, "  unit %2u refNum %d: DCE@%06X flags=%04X%s qHead=%06X\n",
                     i, ~i, dce, flags, (flags & 0x0080) ? " BUSY" : "", qHead);
    }
}

bool describeIOTrap(Machine& mac, u16 trap, u32 pc, u32 a0, std::string& out) {
    const int n = trap & 0xFF;
    if (n < 0x02 || n > 0x06) return false;   // Read/Write/Control/Status/KillIO
    const char* nm = trapName(trap);
    char buf[160];
    const u16 ioTrap = mac.read16(a0 + 6);
    const int16_t ioResult = int16_t(mac.read16(a0 + 16));
    const int16_t vRef = int16_t(mac.read16(a0 + 22));
    const int16_t refNum = int16_t(mac.read16(a0 + 24));
    // Where in the fork the request lands: IOParam ioBuffer+32, ioReqCount+36,
    // ioPosMode+44, ioPosOffset+46. Meaningful for file refNums (> 0).
    const u32 ioBuffer = (u32(mac.read16(a0 + 32)) << 16) | mac.read16(a0 + 34);
    const u32 ioReqCount = (u32(mac.read16(a0 + 36)) << 16) | mac.read16(a0 + 38);
    const int16_t posMode = int16_t(mac.read16(a0 + 44));
    const int32_t posOffset =
        int32_t((u32(mac.read16(a0 + 46)) << 16) | mac.read16(a0 + 48));
    std::snprintf(buf, sizeof buf,
                  "%-8s pc=%06X pb=%06X refNum=%d drive=%d ioResult=%d%s "
                  "buf=%06X req=%u posMode=%d posOff=%d",
                  nm ? nm : "_IO", pc, a0, refNum, vRef, ioResult,
                  (ioTrap & 0x0400) ? " ASYNC" : "",
                  ioBuffer, ioReqCount, posMode, posOffset);
    out = buf;
    return true;
}

namespace {

// Where the call that would return to `v` starts, or 0 if nothing ends there.
// Filtering a stack scan on "looks like a ROM address" only ever finds ROM
// callers, which is no use when the question is which patch in the system heap
// or which application asked for something. Looking for the call instruction
// itself works anywhere in memory, and rejects the data that merely looks like
// a code address -- which is what made a raw scan unreadable.
u32 callSiteOf(Machine& mac, u32 v) {
    if ((v & 1) || v < 0x000400) return 0;
    const u16 w2 = mac.read16((v - 2) & 0xFFFFFF);
    const u16 w4 = mac.read16((v - 4) & 0xFFFFFF);
    const u16 w6 = mac.read16((v - 6) & 0xFFFFFF);
    const u8 lo2 = static_cast<u8>(w2 & 0xFF);
    if ((w2 & 0xFF00) == 0x6100 && lo2 != 0x00 && lo2 != 0xFF) return v - 2;  // BSR.S
    if ((w2 & 0xFFF8) == 0x4E90) return v - 2;                                // JSR (An)
    if (w4 == 0x6100) return v - 4;                                           // BSR.W
    if ((w4 & 0xFFF8) == 0x4EA8) return v - 4;                                // JSR (d16,An)
    if ((w4 & 0xFFF8) == 0x4EB0) return v - 4;                                // JSR (d8,An,Xn)
    if (w4 == 0x4EB8 || w4 == 0x4EBA || w4 == 0x4EBB) return v - 4;           // JSR abs.w/PC
    if (w6 == 0x4EB9) return v - 6;                                           // JSR abs.l
    return 0;
}

} // namespace

void dumpBacktrace(const M68000& cpu, Machine& mac, std::FILE* out) {
    auto r32 = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
    std::fprintf(out, "-- backtrace (A6 chain) --\n");
    std::fprintf(out, "  pc   %06X  %s\n", cpu.pc & 0xFFFFFF,
                 symbolFor(mac, cpu.pc).c_str());
    u32 fp = cpu.a[6];
    for (int i = 0; i < 24 && fp && (fp & 1) == 0 && fp < 0x400000; ++i) {
        const u32 ret = r32(fp + 4) & 0xFFFFFF;
        std::fprintf(out, "  #%-2d  ret %06X  %-20s (frame %06X)\n", i, ret,
                     symbolFor(mac, ret).c_str(), fp);
        const u32 next = r32(fp);       // link to caller's frame (higher address)
        if (next <= fp || (next & 1)) break;
        fp = next;
    }
    // Frame pointers aren't always set up (or A6 may be clobbered), so also
    // walk the raw stack, keeping every long a call instruction ends at.
    std::fprintf(out, "  -- stack scan (A7=%06X, callers innermost first) --\n",
                 cpu.a[7] & 0xFFFFFF);
    u32 sp = cpu.a[7];
    for (int i = 0, found = 0; i < 512 && sp < 0x400000 && found < 32; ++i, sp += 2) {
        const u32 v = r32(sp) & 0xFFFFFF;
        const u32 site = callSiteOf(mac, v);
        if (!site) continue;
        std::string call;
        disasm(mac, site, call);
        std::fprintf(out, "    %06X: %06X %-18s  %s\n", sp, site,
                     symbolFor(mac, site).c_str(), call.c_str());
        ++found;
    }
}

void checkHeap(Machine& mac, std::FILE* out) {
    auto r32 = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
    auto zoneInfo = [&](const char* nm, u32 zp) {
        std::fprintf(out, "  %-8s zone=%06X", nm, zp & 0xFFFFFF);
        if (zp && (zp & 1) == 0 && zp < 0x400000)
            std::fprintf(out, "  bkLim=%06X  free=%d bytes", r32(zp) & 0xFFFFFF,
                         static_cast<int>(r32(zp + 0x0C)));   // zcbFree
        std::fprintf(out, "\n");
    };
    std::fprintf(out, "-- heap zones --\n");
    zoneInfo("TheZone", r32(0x0118));
    zoneInfo("ApplZone", r32(0x02AA));
    zoneInfo("SysZone", r32(0x02A6));
}

void dumpTimerQueue(Machine& mac, std::FILE* out) {
    auto r32 = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
    const u32 tm = r32(0x0B30) & 0xFFFFFF;   // tm_var
    std::fprintf(out, "-- Time Manager queue (tm_var=[$0B30]=%06X) --\n", tm);
    if (!tm || (tm & 1) || tm >= 0x400000) { std::fprintf(out, "  (no tm_var)\n"); return; }
    // The active-timer QHdr sits at tm_var+8: qFlags (word), qHead (long @+$0A).
    std::fprintf(out, "  qFlags=%04X qHead=%06X qTail=%06X\n", mac.read16(tm + 8),
                 r32(tm + 0x0A) & 0xFFFFFF, r32(tm + 0x0E) & 0xFFFFFF);
    u32 el = r32(tm + 0x0A) & 0xFFFFFF;      // qHead
    int n = 0;
    while (el && (el & 1) == 0 && el < 0x400000 && n < 24) {
        // TMTask: qLink (+0), qType (+4), tmAddr (+6), tmCount (+$0A).
        std::fprintf(out, "  task @%06X  qType=%04X tmAddr=%06X tmCount=%d\n", el,
                     mac.read16(el + 4), r32(el + 6) & 0xFFFFFF,
                     static_cast<int>(r32(el + 0x0A)));
        el = r32(el) & 0xFFFFFF;              // qLink
        ++n;
    }
    std::fprintf(out, "  %d task(s) queued\n", n);
}

void dumpVia(Machine& mac, std::FILE* out) {
    const auto v = mac.viaRegs();
    std::fprintf(out, "-- VIA 6522 --\n");
    std::fprintf(out, "  ORA=%02X ORB=%02X DDRA=%02X DDRB=%02X ACR=%02X PCR=%02X SR=%02X\n",
                 v.ora, v.orb, v.ddra, v.ddrb, v.acr, v.pcr, v.sr);
    std::fprintf(out, "  T1=%04X T2=%04X  IFR=%02X IER=%02X  IRQ=%d\n",
                 v.t1c, v.t2c, v.ifr, v.ier, v.irq ? 1 : 0);
    static const char* kSrc[] = {"CA2", "CA1", "SR", "CB2", "CB1", "T2", "T1"};
    const u8 active = v.ifr & v.ier & 0x7F;
    std::fprintf(out, "  pending&enabled:");
    for (int b = 0; b < 7; ++b)
        if (active & (1u << b)) std::fprintf(out, " %s", kSrc[b]);
    std::fprintf(out, "%s\n", active ? "" : " none");
}

} // namespace openmac::dbg
