#include "openmac/debugger.hpp"

#include <cstdint>

namespace openmac::dbg {

const char* trapName(u16 op) {
    if ((op & 0xF000) != 0xA000) return nullptr;
    if (op & 0x0800) {                       // Toolbox trap: number in bits 0-9
        switch (op & 0x0FFF) {
            case 0x0BC8: return "_SysBeep";
            case 0x0A9F: return "_InitCursor";
            case 0x0B44: return "_DrawMenuBar";
            case 0x0B93: return "_InitWindows";
            case 0x0A86: return "_DrawString";
            case 0x0A88: return "_StillDown";
            default:     return "_Toolbox";
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
        case 0x0F: return "_MountVol";
        case 0x13: return "_FlushVol";
        case 0x1D: return "_MaxMem";
        case 0x1E: return "_NewPtr";
        case 0x1F: return "_DisposPtr";
        case 0x22: return "_NewHandle";
        case 0x23: return "_DisposHandle";
        case 0x24: return "_SetHandleSize";
        case 0x25: return "_GetHandleSize";
        case 0x26: return "_HandleZone";
        case 0x27: return "_ReallocHandle";
        case 0x28: return "_RecoverHandle";
        case 0x29: return "_HLock";
        case 0x2A: return "_HUnlock";
        case 0x2E: return "_BlockMove";
        case 0x3C: return "_CmpString";
        case 0x40: return "_ResrvMem";
        case 0x49: return "_HPurge";
        case 0x4A: return "_HNoPurge";
        case 0x4E: return "_AddDrive";
        case 0x55: return "_StripAddress";
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
    {0x0108, "MemTop",     4}, {0x011C, "UTableBase", 4}, {0x0126, "MinStack", 4},
    {0x0130, "ApplLimit",  4}, {0x0134, "SonyVars",   4}, {0x016A, "Ticks",    4},
    {0x0210, "BootDrive",  2}, {0x0220, "FSQHdr",     4}, {0x02AE, "ROMBase",  4},
    {0x02B2, "RAMBase",    4}, {0x0308, "DrvQHdr",   10}, {0x0824, "ScrnBase", 4},
    {0x08FC, "JIODone",    4}, {0x0904, "CurrentA5",  4}, {0x0910, "CurApName",4},
};
} // namespace

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
    Machine& mac;
    u32 p;
    u16 word() { const u16 v = mac.read16(p); p += 2; return v; }
    u32 lng()  { const u32 v = (u32(mac.read16(p)) << 16) | mac.read16(p + 2); p += 4; return v; }
};

const char* kSizes[] = {".b", ".w", ".l"};

// Decode an effective address, appending text and consuming extension words.
std::string ea(Cursor& c, int mode, int reg, int size) {
    char b[48];
    switch (mode) {
        case 0: std::snprintf(b, sizeof b, "D%d", reg); break;
        case 1: std::snprintf(b, sizeof b, "A%d", reg); break;
        case 2: std::snprintf(b, sizeof b, "(A%d)", reg); break;
        case 3: std::snprintf(b, sizeof b, "(A%d)+", reg); break;
        case 4: std::snprintf(b, sizeof b, "-(A%d)", reg); break;
        case 5: std::snprintf(b, sizeof b, "$%04X(A%d)", c.word(), reg); break;
        case 6: { const u16 ext = c.word();
                  std::snprintf(b, sizeof b, "$%02X(A%d,%c%d)", ext & 0xFF, reg,
                                ext & 0x8000 ? 'A' : 'D', (ext >> 12) & 7); break; }
        case 7:
            switch (reg) {
                case 0: std::snprintf(b, sizeof b, "$%04X", c.word()); break;
                case 1: std::snprintf(b, sizeof b, "$%08X", c.lng()); break;
                case 2: std::snprintf(b, sizeof b, "$%04X(PC)", c.word()); break;
                case 3: { const u16 ext = c.word();
                          std::snprintf(b, sizeof b, "$%02X(PC,%c%d)", ext & 0xFF,
                                        ext & 0x8000 ? 'A' : 'D', (ext >> 12) & 7); break; }
                case 4:
                    if (size == 2) std::snprintf(b, sizeof b, "#$%08X", c.lng());
                    else std::snprintf(b, sizeof b, "#$%04X", c.word());
                    break;
                default: std::snprintf(b, sizeof b, "?"); break;
            }
            break;
        default: std::snprintf(b, sizeof b, "?"); break;
    }
    return b;
}

} // namespace

int disasm(Machine& mac, u32 pc, std::string& out) {
    Cursor c{mac, pc};
    const u16 op = c.word();
    char buf[96];

    auto emit = [&](const char* s) { out += s; };

    if ((op & 0xF000) == 0xA000) {
        const char* n = trapName(op);
        std::snprintf(buf, sizeof buf, "%-8s ; $%04X", n ? n : "_Axxx", op);
        emit(buf);
    } else if ((op & 0xF000) == 0x6000) {          // Bcc / BRA / BSR
        static const char* cc[] = {"BRA","BSR","BHI","BLS","BCC","BCS","BNE","BEQ",
                                   "BVC","BVS","BPL","BMI","BGE","BLT","BGT","BLE"};
        int disp = int8_t(op & 0xFF);
        u32 base = c.p;
        if ((op & 0xFF) == 0) { disp = int16_t(c.word()); }
        std::snprintf(buf, sizeof buf, "%-8s $%06X", cc[(op >> 8) & 0xF], base + disp);
        emit(buf);
    } else if ((op & 0xF0F8) == 0x50C8) {          // DBcc
        const int disp = int16_t(c.word());
        std::snprintf(buf, sizeof buf, "DB%-6s D%d,$%06X", "cc", op & 7, c.p - 2 + disp);
        emit(buf);
    } else if (op == 0x4E75) { emit("RTS");
    } else if (op == 0x4E73) { emit("RTE");
    } else if (op == 0x4E71) { emit("NOP");
    } else if ((op & 0xFFC0) == 0x4E80 || (op & 0xFFC0) == 0x4EC0) {  // JSR / JMP
        const std::string t = ea(c, (op >> 3) & 7, op & 7, 2);
        std::snprintf(buf, sizeof buf, "%-8s %s", (op & 0x40) ? "JMP" : "JSR", t.c_str());
        emit(buf);
    } else if ((op & 0xF1C0) == 0x41C0) {          // LEA
        const std::string s = ea(c, (op >> 3) & 7, op & 7, 2);
        std::snprintf(buf, sizeof buf, "LEA      %s,A%d", s.c_str(), (op >> 9) & 7);
        emit(buf);
    } else if ((op & 0xF000) == 0x7000) {          // MOVEQ
        std::snprintf(buf, sizeof buf, "MOVEQ    #$%02X,D%d", op & 0xFF, (op >> 9) & 7);
        emit(buf);
    } else if ((op & 0xC000) == 0 && (op & 0x3000) != 0) {   // MOVE.b/w/l
        const int sz = (op >> 12) == 1 ? 0 : (op >> 12) == 3 ? 1 : 2;
        const std::string s = ea(c, (op >> 3) & 7, op & 7, sz + 1);
        const std::string d = ea(c, (op >> 6) & 7, (op >> 9) & 7, sz + 1);
        std::snprintf(buf, sizeof buf, "MOVE%-4s %s,%s", kSizes[sz], s.c_str(), d.c_str());
        emit(buf);
    } else if ((op & 0xFF00) == 0x4A00) {          // TST
        const int sz = (op >> 6) & 3;
        const std::string s = ea(c, (op >> 3) & 7, op & 7, (sz == 2 ? 2 : sz) + 1);
        std::snprintf(buf, sizeof buf, "TST%-5s %s", kSizes[sz < 3 ? sz : 1], s.c_str());
        emit(buf);
    } else if ((op & 0xFF00) == 0x0800) {          // static BTST/BCHG/BCLR/BSET
        static const char* bops[] = {"BTST","BCHG","BCLR","BSET"};
        const u16 bit = c.word();
        const std::string s = ea(c, (op >> 3) & 7, op & 7, 1);
        std::snprintf(buf, sizeof buf, "%-8s #%d,%s", bops[(op >> 6) & 3], bit & 0x1F,
                      s.c_str());
        emit(buf);
    } else {
        std::snprintf(buf, sizeof buf, "DC.W     $%04X", op);
        emit(buf);
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
    std::snprintf(buf, sizeof buf,
                  "%-8s pc=%06X pb=%06X refNum=%d drive=%d ioResult=%d%s",
                  nm ? nm : "_IO", pc, a0, refNum, vRef, ioResult,
                  (ioTrap & 0x0400) ? " ASYNC" : "");
    out = buf;
    return true;
}

void dumpBacktrace(const M68000& cpu, Machine& mac, std::FILE* out) {
    auto r32 = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
    std::fprintf(out, "-- backtrace (A6 chain) --\n");
    std::fprintf(out, "  pc   %06X\n", cpu.pc & 0xFFFFFF);
    u32 fp = cpu.a[6];
    for (int i = 0; i < 24 && fp && (fp & 1) == 0 && fp < 0x400000; ++i) {
        std::fprintf(out, "  #%-2d  ret %06X  (frame %06X)\n", i, r32(fp + 4) & 0xFFFFFF, fp);
        const u32 next = r32(fp);       // link to caller's frame (higher address)
        if (next <= fp || (next & 1)) break;
        fp = next;
    }
    // Frame pointers aren't always set up (or A6 may be clobbered), so also
    // scan the raw stack for values that look like ROM return addresses.
    std::fprintf(out, "  -- stack scan (A7=%06X) --\n", cpu.a[7] & 0xFFFFFF);
    u32 sp = cpu.a[7];
    for (int i = 0, found = 0; i < 160 && sp < 0x400000 && found < 24; ++i, sp += 2) {
        const u32 v = r32(sp) & 0xFFFFFF;
        if (v >= 0x400100 && v < 0x440000 && (v & 1) == 0) {
            std::fprintf(out, "    %06X: %06X\n", sp, v);
            ++found;
        }
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

} // namespace openmac::dbg
