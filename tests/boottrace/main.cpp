// Headless bring-up tool: runs a ROM for N frames and reports how far the
// machine gets — per-frame PC samples, screen activity, and the stub/unmapped
// access log. The debugger you can read in a terminal.

#include <openmac/machine.hpp>
#include <openmac/debugger.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

using namespace openmac;

namespace {

// Single-step one frame's worth of cycles, collecting unique PCs, then print
// them collapsed into ranges: the shape of the active code path.
void profileFrame(Machine& mac) {
    std::set<u32> pcs;
    const u64 target = mac.totalCycles() +
                       u64(Machine::kLinesPerFrame) * Machine::kCyclesPerLine;
    while (mac.totalCycles() < target && !mac.cpu().halted) {
        pcs.insert(mac.cpu().pc);
        mac.stepInstruction();
    }
    std::printf("-- profile: %zu unique PCs --\n", pcs.size());
    u32 start = 0, prev = 0;
    bool open = false;
    for (u32 pc : pcs) {
        if (!open) { start = prev = pc; open = true; continue; }
        if (pc - prev <= 8) { prev = pc; continue; }
        std::printf("  %06X-%06X\n", start, prev);
        start = prev = pc;
    }
    if (open) std::printf("  %06X-%06X\n", start, prev);
}

// Single-step until PC first hits `stopPc` — or, when stopPc is 1, until
// execution leaves plausible regions (a runaway) — then dump the trail.
void traceUntil(Machine& mac, u32 stopPc, u64 maxCycles) {
    std::vector<u32> ring(96, 0);
    size_t head = 0;
    while (mac.totalCycles() < maxCycles && !mac.cpu().halted) {
        const u32 pc = mac.cpu().pc;
        const u32 p24 = pc & 0xFFFFFF;
        const bool runaway =
            stopPc == 1 && (pc > 0xFFFFFF || (p24 >= 0x480000 && p24 < 0x800000) ||
                            (p24 >= 0x100000 && p24 < 0x400000));
        if (pc == stopPc || runaway) {
            std::printf("-- stopped at pc=%08X after %llu cycles; trail: --\n", pc,
                        static_cast<unsigned long long>(mac.totalCycles()));
            auto rd32 = [&](u32 a) {
                return (u32(mac.read16(a)) << 16) | mac.read16(a + 2);
            };
            std::printf("sr=%04X a7=%08X vec1=%08X vec2=%08X vec3=%08X\n",
                        mac.cpu().getSR(), mac.cpu().a[7], rd32(0x64), rd32(0x68),
                        rd32(0x6C));
            std::printf("via IFR=%02X IER=%02X\n", mac.read8(0xEFFBFE),
                        mac.read8(0xEFFDFE));
            std::vector<u32> trail;
            for (size_t i = 0; i < ring.size(); ++i) {
                const u32 p = ring[(head + i) % ring.size()];
                if (p && (trail.empty() || trail.back() != p)) trail.push_back(p);
            }
            for (size_t i = 0; i < trail.size(); ++i) {
                std::printf("%06X%s", trail[i], (i % 8 == 7) ? "\n" : " ");
            }
            std::printf("\n");
            return;
        }
        ring[head] = pc;
        head = (head + 1) % ring.size();
        mac.stepInstruction();
    }
    std::printf("-- never reached %06X --\n", stopPc);
}

void dumpBmp(Machine& mac, const std::string& path) {
    const int w = Machine::kScreenW, h = Machine::kScreenH;
    std::vector<u32> pix(static_cast<size_t>(w) * h);
    mac.renderScreen(pix.data());
    const int rowBytes = w * 3;
    const u32 imgSize = static_cast<u32>(rowBytes * h);
    const u32 fileSize = 54 + imgSize;
    u8 hdr[54] = {};
    hdr[0] = 'B'; hdr[1] = 'M';
    hdr[2] = static_cast<u8>(fileSize); hdr[3] = static_cast<u8>(fileSize >> 8);
    hdr[4] = static_cast<u8>(fileSize >> 16); hdr[5] = static_cast<u8>(fileSize >> 24);
    hdr[10] = 54; hdr[14] = 40;
    hdr[18] = static_cast<u8>(w); hdr[19] = static_cast<u8>(w >> 8);
    hdr[22] = static_cast<u8>(h); hdr[23] = static_cast<u8>(h >> 8);
    hdr[26] = 1; hdr[28] = 24;
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(hdr), sizeof hdr);
    std::vector<u8> row(static_cast<size_t>(rowBytes));
    for (int y = h - 1; y >= 0; --y) {
        for (int x = 0; x < w; ++x) {
            const u32 p = pix[static_cast<size_t>(y) * w + x];
            row[x * 3 + 0] = static_cast<u8>(p);
            row[x * 3 + 1] = static_cast<u8>(p >> 8);
            row[x * 3 + 2] = static_cast<u8>(p >> 16);
        }
        f.write(reinterpret_cast<const char*>(row.data()), rowBytes);
    }
    std::printf("screen dumped to %s\n", path.c_str());
}

void writeWav(const std::string& path, const std::vector<u8>& s, int rate) {
    std::ofstream f(path, std::ios::binary);
    auto w32 = [&](u32 v) { for (int i = 0; i < 4; ++i) f.put(static_cast<char>((v >> (8 * i)) & 0xFF)); };
    auto w16 = [&](u16 v) { f.put(static_cast<char>(v & 0xFF)); f.put(static_cast<char>((v >> 8) & 0xFF)); };
    const u32 n = static_cast<u32>(s.size());
    f.write("RIFF", 4); w32(36 + n); f.write("WAVE", 4);
    f.write("fmt ", 4); w32(16); w16(1); w16(1); w32(static_cast<u32>(rate));
    w32(static_cast<u32>(rate)); w16(1); w16(8);
    f.write("data", 4); w32(n);
    f.write(reinterpret_cast<const char*>(s.data()), n);
}

} // namespace

int main(int argc, char** argv) {
    std::string romPath;
    int frames = 60;
    u32 ramMB = 4;
    int profileAt = -1;
    u32 traceToPc = 0;
    u32 watchAddr = 0xFFFFFFFFu;
    bool mouseWalk = false;
    bool bootDisk = false;
    bool forceRom = false;
    std::string dumpPath;
    std::string floppyPath;
    bool traceTraps = false, lowmemDump = false, traceOsTraps = false, checkHeapFlag = false;
    bool traceIrq = false;
    u32 breakPc = 0, watchMem = 0xFFFFFFFFu;
    u32 breakTrap = 0, tracePc = 0, dumpMemAddr = 0, dumpMemLen = 0;
    int traceCount = 48;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--rom" && i + 1 < argc) romPath = argv[++i];
        else if (arg == "--floppy" && i + 1 < argc) floppyPath = argv[++i];
        else if (arg == "--trace-traps") traceTraps = true;
        else if (arg == "--trace-irq") traceIrq = true;
        else if (arg == "--trace-os-traps") traceOsTraps = true;
        else if (arg == "--break-pc" && i + 1 < argc)
            breakPc = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        else if (arg == "--watch-mem" && i + 1 < argc)
            watchMem = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        else if (arg == "--check-heap") checkHeapFlag = true;
        else if (arg == "--lowmem") lowmemDump = true;
        else if (arg == "--break-trap" && i + 1 < argc)
            breakTrap = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        else if (arg == "--trace-pc" && i + 1 < argc)
            tracePc = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        else if (arg == "--trace-count" && i + 1 < argc) traceCount = std::atoi(argv[++i]);
        else if (arg == "--dump-mem" && i + 2 < argc) {
            dumpMemAddr = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
            dumpMemLen = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        }
        else if (arg == "--frames" && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (arg == "--ram-mb" && i + 1 < argc) ramMB = static_cast<u32>(std::atoi(argv[++i]));
        else if (arg == "--profile" && i + 1 < argc) profileAt = std::atoi(argv[++i]);
        else if (arg == "--trace-to" && i + 1 < argc) {
            traceToPc = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        }
        else if (arg == "--dump-screen" && i + 1 < argc) dumpPath = argv[++i];
        else if (arg == "--watch" && i + 1 < argc) {
            watchAddr = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        }
        else if (arg == "--mouse-walk") mouseWalk = true;
        else if (arg == "--boot-disk") bootDisk = true;
        else if (arg == "--force-rom") forceRom = true;
    }
    if (romPath.empty()) {
        std::fprintf(stderr, "usage: openmac_trace --rom <path> [--frames N] [--ram-mb M]\n");
        return 2;
    }
    std::ifstream f(romPath, std::ios::binary);
    std::vector<u8> rom{std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
    if (rom.empty()) {
        std::fprintf(stderr, "cannot read ROM: %s\n", romPath.c_str());
        return 2;
    }
    std::printf("ROM %zu bytes, header checksum %02X%02X%02X%02X, version %02X%02X\n",
                rom.size(), rom[0], rom[1], rom[2], rom[3], rom[8], rom[9]);

    Machine mac(std::move(rom), {ramMB * 1024u * 1024u});
    if (forceRom) mac.setForceRomDisk(true);
    if (!floppyPath.empty()) {
        std::ifstream ff(floppyPath, std::ios::binary);
        std::vector<u8> img{std::istreambuf_iterator<char>(ff),
                            std::istreambuf_iterator<char>()};
        if (img.empty()) {
            std::fprintf(stderr, "cannot read floppy: %s\n", floppyPath.c_str());
            return 2;
        }
        std::printf("FLOPPY %zu bytes inserted\n", img.size());
        mac.insertFloppy(std::move(img), false);
    }

    if (traceTraps || breakTrap) {
        mac.cpu().onTrap = [&](u16 trap, u32 pc) {
            if (traceTraps) {
                std::string s;
                if (openmac::dbg::describeIOTrap(mac, trap, pc, mac.cpu().a[0], s))
                    std::printf("TRAP %s\n", s.c_str());
            }
            if (breakTrap && (trap & 0x0FFFu) == (breakTrap & 0x0FFFu)) {
                std::printf("\n=== BREAK trap %04X at pc=%06X ===\n", trap, pc);
                openmac::dbg::dumpRegs(mac.cpu(), stdout);
                openmac::dbg::dumpDriveQueue(mac, stdout);
                openmac::dbg::dumpUnitTable(mac, stdout);
            }
        };
    }

    if (traceIrq) {
        mac.cpu().onInterrupt = [&](int level, u32 vec, u32 pc) {
            const u8 ifr = level == 1 ? mac.viaRegs().ifr : 0;    // VIA IFR = source
            const char* src = (ifr & 0x40) ? "T1" : (ifr & 0x20) ? "T2"
                            : (ifr & 0x10) ? "CB1" : (ifr & 0x04) ? "SR/ADB"
                            : (ifr & 0x02) ? "CA1/VBL" : (ifr & 0x01) ? "CA2" : "?";
            static int n = 0;
            if (n++ < 2000)
                std::printf("IRQ L%d vec=%u pc=%06X ifr=%02X %-8s cyc=%llu\n", level, vec,
                            pc, ifr, src, static_cast<unsigned long long>(mac.totalCycles()));
        };
    }

    u32 wPrevPc = 0, wLastVal = 0;
    bool wFirst = true;
    int bpHits = 0;
    if (breakPc || watchMem != 0xFFFFFFFFu) {
        mac.cpu().onStep = [&](u32 pc) {
            if (breakPc && pc == breakPc && bpHits < 8) {
                ++bpHits;
                std::printf("\n=== BREAK pc=%06X (hit %d) cyc=%llu ===\n", pc, bpHits,
                            static_cast<unsigned long long>(mac.totalCycles()));
                openmac::dbg::dumpRegs(mac.cpu(), stdout);
                openmac::dbg::dumpBacktrace(mac.cpu(), mac, stdout);
            }
            if (watchMem != 0xFFFFFFFFu) {
                const u32 v = (static_cast<u32>(mac.read16(watchMem)) << 16) |
                              mac.read16(watchMem + 2);
                if (wFirst) { wLastVal = v; wFirst = false; }
                else if (v != wLastVal) {
                    std::printf("WATCH [%06X] %08X -> %08X by pc=%06X cyc=%llu\n", watchMem,
                                wLastVal, v, wPrevPc,
                                static_cast<unsigned long long>(mac.totalCycles()));
                    wLastVal = v;
                }
            }
            wPrevPc = pc;
        };
    }

    int excCount = 0;
    mac.cpu().onException = [&](int vector, u32 pc) {
        const bool crash = vector == 2 || vector == 3 || vector == 4 ||
                           vector == 8 || vector == 11;
        if (crash && excCount < 16) {
            std::printf("EXC vec=%d (%s) at pc=%06X  cyc=%llu\n", vector,
                        vector == 2 ? "bus" : vector == 3 ? "addr" :
                        vector == 4 ? "illegal" : vector == 8 ? "priv" : "F-line",
                        pc, static_cast<unsigned long long>(mac.totalCycles()));
            if (vector == 2 || vector == 3 || vector == 11) {   // how we got here
                std::printf("  trail (oldest first):\n   ");
                for (int b = 119; b >= 0; --b) {
                    std::printf(" %06X", mac.cpu().recentPc(b));
                    if (b % 10 == 0) std::printf("\n   ");
                }
                std::printf("\n");
                openmac::dbg::dumpRegs(mac.cpu(), stdout);
                std::string dis;
                openmac::dbg::disasm(mac, pc, dis);
                std::printf("  faulting: %06X  %s\n", pc, dis.c_str());
                openmac::dbg::dumpBacktrace(mac.cpu(), mac, stdout);
                openmac::dbg::checkHeap(mac, stdout);
            }
            ++excCount;
        }
    };

    if (traceToPc) {
        traceUntil(mac, traceToPc, u64(frames) * Machine::kLinesPerFrame * Machine::kCyclesPerLine);
        return 0;
    }

    if (mouseWalk) {
        for (int i = 0; i < 1900 && !mac.cpu().halted; ++i) mac.runFrame();
        std::string before = dumpPath.empty() ? std::string() : dumpPath + ".before.bmp";
        if (!before.empty()) dumpBmp(mac, before);
        // Drive the mouse down-right for a while, then click.
        for (int i = 0; i < 200 && !mac.cpu().halted; ++i) {
            mac.mouseMove(4, 3, i >= 170);
            mac.runFrame();
        }
        if (!dumpPath.empty()) dumpBmp(mac, dumpPath);
        std::printf("mouse-walk done: halted=%d pc=%06X\n",
                    mac.cpu().halted ? 1 : 0, mac.cpu().pc);
        return 0;
    }

    if (watchAddr != 0xFFFFFFFFu) {
        auto rd32 = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
        const u64 maxCycles = u64(frames) * Machine::kLinesPerFrame * Machine::kCyclesPerLine;
        u32 last = rd32(watchAddr);
        std::printf("watch [%06X] initial=%08X\n", watchAddr, last);
        int hits = 0;
        while (mac.totalCycles() < maxCycles && !mac.cpu().halted && hits < 60) {
            const u32 pc = mac.cpu().pc;
            mac.stepInstruction();
            const u32 now = rd32(watchAddr);
            if (now != last) {
                std::printf("[%06X] %08X -> %08X  by pc=%06X\n", watchAddr, last, now, pc);
                last = now;
                ++hits;
            }
        }
        std::printf("halted=%d final=%08X\n", mac.cpu().halted ? 1 : 0, last);
        return 0;
    }

    if (tracePc) {
        const u64 maxCycles = u64(frames) * Machine::kLinesPerFrame * Machine::kCyclesPerLine;
        bool tracing = false;
        int traced = 0;
        while (mac.totalCycles() < maxCycles && !mac.cpu().halted && traced < traceCount) {
            const u32 pc = mac.cpu().pc;
            if (pc == tracePc) tracing = true;
            if (tracing) {
                std::string dis;
                openmac::dbg::disasm(mac, pc, dis);
                std::printf("%06X  %-30s D0=%08X D1=%08X A0=%06X A1=%06X\n", pc,
                            dis.c_str(), mac.cpu().d[0], mac.cpu().d[1],
                            mac.cpu().a[0], mac.cpu().a[1]);
                ++traced;
            }
            mac.stepInstruction();
        }
        std::printf("\n");
        openmac::dbg::dumpRegs(mac.cpu(), stdout);
        return 0;
    }

    if (traceOsTraps) {
        // Log Memory Manager allocation traps with their results; a NIL result
        // (A0 = 0) is the classic origin of a later NIL-dereference crash.
        struct Pend { u32 ret; u16 trap; };
        std::vector<Pend> pend;
        mac.cpu().onTrap = [&](u16 trap, u32 pc) {
            if ((trap & 0xF800) != 0xA000) return;          // OS traps only
            if (openmac::dbg::trapReturnsPtrInA0(trap)) {
                std::printf("-> %-14s size=%-8d @%06X\n", openmac::dbg::trapName(trap),
                            mac.cpu().d[0], pc);
                pend.push_back({pc + 2, trap});
            } else if ((mac.cpu().a[0] & 0xFFFFFF) == 0) {  // NIL pointer argument
                std::printf("!! %-14s A0=NIL @%06X\n", openmac::dbg::trapName(trap), pc);
            }
        };
        const u64 maxCycles = u64(frames) * Machine::kLinesPerFrame * Machine::kCyclesPerLine;
        while (mac.totalCycles() < maxCycles && !mac.cpu().halted) {
            const u32 pc = mac.cpu().pc;
            for (size_t i = pend.size(); i-- > 0;) {
                if (pend[i].ret == pc) {
                    const u32 a0 = mac.cpu().a[0] & 0xFFFFFF;
                    std::printf("<- %-14s A0=%06X%s\n", openmac::dbg::trapName(pend[i].trap),
                                a0, a0 == 0 ? "   *** NIL (allocation failed) ***" : "");
                    pend.erase(pend.begin() + static_cast<long>(i));
                    break;
                }
            }
            mac.stepInstruction();
        }
        std::printf("\n");
        openmac::dbg::checkHeap(mac, stdout);
        return 0;
    }

    int bootHold = 0;
    if (bootDisk) {
        mac.keyEvent(0x37, true); mac.keyEvent(0x3A, true);   // Command, Option
        mac.keyEvent(0x07, true); mac.keyEvent(0x1F, true);   // X, O
        bootHold = 200;
    }

    std::vector<u8> allAudio;
    std::vector<u8> tmpAudio;
    for (int i = 0; i < frames; ++i) {
        if (i == profileAt) profileFrame(mac);
        mac.runFrame();
        mac.drainAudio(tmpAudio);
        allAudio.insert(allAudio.end(), tmpAudio.begin(), tmpAudio.end());
        if (bootHold > 0 && --bootHold == 0) {
            mac.keyEvent(0x37, false); mac.keyEvent(0x3A, false);
            mac.keyEvent(0x07, false); mac.keyEvent(0x1F, false);
        }
        const auto& cpu = mac.cpu();

        // Screen activity: count black pixels in the visible buffer.
        static std::vector<u32> pix(static_cast<size_t>(Machine::kScreenW) * Machine::kScreenH);
        mac.renderScreen(pix.data());
        long black = 0;
        for (u32 p : pix) black += (p == 0xFF000000u);

        if (i < 10 || i % 10 == 9 || cpu.halted) {
            std::printf("frame %3d  pc=%06X sr=%04X overlay=%d black=%ld%s%s\n",
                        i + 1, cpu.pc, cpu.getSR(), mac.overlayActive() ? 1 : 0, black,
                        cpu.stopped ? " STOPPED" : "", cpu.halted ? " HALTED" : "");
        }
        if (cpu.halted) break;
    }

    auto rd32 = [&](u32 a) {
        return (u32(mac.read16(a)) << 16) | mac.read16(a + 2);
    };
    std::printf("globals: MemTop=%08X BufPtr=%08X ScrnBase=%08X SoundBase=%08X\n",
                rd32(0x108), rd32(0x10C), rd32(0x824), rd32(0x266));

    if (lowmemDump) {
        openmac::dbg::dumpLowMem(mac, stdout);
        openmac::dbg::dumpDriveQueue(mac, stdout);
        openmac::dbg::dumpUnitTable(mac, stdout);
        openmac::dbg::dumpTimerQueue(mac, stdout);
        openmac::dbg::dumpVia(mac, stdout);
    }
    if (dumpMemLen) {
        std::printf("-- memory $%06X (%u bytes) --\n", dumpMemAddr, dumpMemLen);
        openmac::dbg::dumpMem(mac, dumpMemAddr, dumpMemLen, stdout);
    }
    if (checkHeapFlag) openmac::dbg::checkHeap(mac, stdout);

    if (!dumpPath.empty()) dumpBmp(mac, dumpPath);

    {
        int lo = 255, hi = 0; long nonSilent = 0;
        for (u8 v : allAudio) { if (v < lo) lo = v; if (v > hi) hi = v; if (v < 0x7C || v > 0x84) ++nonSilent; }
        std::printf("\n-- audio: %zu samples, min=%d max=%d non-silent=%ld --\n",
                    allAudio.size(), lo, hi, nonSilent);
        if (!allAudio.empty())
            writeWav("I:/Visual Studio Projects/scratch/openmac/shots/boot.wav", allAudio, 22254);
    }

    std::printf("\n-- KeyMap ($174) reads: %u  from PCs:", mac.keyMapReads());
    for (int i = 0; i < mac.keyMapPcCount(); ++i) std::printf(" %06X", mac.keyMapPc(i));
    std::printf(" --\n");
    const auto s = mac.adbStats();
    std::printf("-- ADB: kbd[enum=%u modifiers=%u transitions=%u] "
                "mouse[enum=%u polls=%u reports=%u] --\n",
                s.kbdReg3, s.kbdReg2, s.kbdPolls, s.mouseReg3, s.mousePolls, s.mouseReports);
    std::printf("-- ADB command trace (addr.op.reg), first %zu: --\n",
                mac.adbCmdTrace().size());
    const auto& tr = mac.adbCmdTrace();
    const auto& rs = mac.adbRespTrace();
    for (size_t i = 0; i < tr.size(); ++i) {
        const u8 c = tr[i];
        const char* op = ((c >> 2) & 3) == 0 ? "rst" : ((c >> 2) & 3) == 1 ? "flu"
                       : ((c >> 2) & 3) == 2 ? "LSN" : "TLK";
        std::printf(" %d.%s.%d%s", (c >> 4) & 0xF, op, c & 3,
                    (i < rs.size() && rs[i]) ? "+" : " ");
        if ((i + 1) % 8 == 0) std::printf("\n");
    }
    std::printf("\n");

    std::printf("-- access log (%zu entries) --\n", mac.accessLog().size());
    int shown = 0;
    for (const auto& line : mac.accessLog()) {
        std::printf("%s\n", line.c_str());
        if (++shown >= 60) { std::printf("...\n"); break; }
    }
    return 0;
}
