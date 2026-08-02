// Headless Quadra 650 bring-up tool: run a ROM for N frames and report how
// far the machine gets — the boot heartbeat, PC samples, the stub/unmapped
// access log, and a BMP of the framebuffer. The terminal debugger that
// carries the board bring-up, as boottrace carried the Classic's.

#include <openmac/hfs.hpp>
#include <openmac/quadra.hpp>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <string>
#include <vector>

using namespace openmac;

namespace {

std::vector<u8> loadFile(const char* path) {
    std::ifstream f(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

void writeBmp(const char* path, const std::vector<u32>& px, int w, int h) {
    std::ofstream f(path, std::ios::binary);
    const u32 rowBytes = static_cast<u32>(w) * 4;
    const u32 dataSize = rowBytes * static_cast<u32>(h);
    u8 hdr[54] = {'B', 'M'};
    auto p32 = [&](int off, u32 v) {
        hdr[off] = static_cast<u8>(v);
        hdr[off + 1] = static_cast<u8>(v >> 8);
        hdr[off + 2] = static_cast<u8>(v >> 16);
        hdr[off + 3] = static_cast<u8>(v >> 24);
    };
    p32(2, 54 + dataSize);
    p32(10, 54);
    p32(14, 40);
    p32(18, static_cast<u32>(w));
    p32(22, static_cast<u32>(-h));
    hdr[26] = 1;
    hdr[28] = 32;
    p32(34, dataSize);
    f.write(reinterpret_cast<char*>(hdr), 54);
    f.write(reinterpret_cast<const char*>(px.data()), dataSize);
}

// Build a bootable HD image from a CD/disk image that contains an HFS
// volume with a System Folder: locate the HFS partition by its 'BD'
// signature, extract the System Folder tree, rebuild it into a fresh
// volume, and graft the source volume's boot blocks (the part the
// formatter leaves zeroed) so the ROM's boot scan accepts it.
int makeBootableHd(const char* srcPath, const char* outPath, u32 sizeMb) {
    auto src = loadFile(srcPath);
    if (src.empty()) {
        std::fprintf(stderr, "cannot read %s\n", srcPath);
        return 2;
    }
    std::size_t part = 0;
    bool found = false;
    for (std::size_t p = 0; p + 1536 < src.size(); p += 512) {
        if (src[p + 1024] == 0x42 && src[p + 1025] == 0x44) {   // MDB 'BD'
            part = p;
            found = true;
            break;
        }
    }
    if (!found) {
        std::fprintf(stderr, "no HFS volume found in %s\n", srcPath);
        return 2;
    }
    std::printf("hfs volume at offset %zX\n", part);
    std::vector<u8> vol(src.begin() + static_cast<long>(part), src.end());

    std::vector<hfs::Item> items;
    if (!hfs::listVolume(vol, items)) {
        std::fprintf(stderr, "volume does not list\n");
        return 2;
    }
    std::printf("catalog: %zu items\n", items.size());

    // Find the System Folder at the root (id 2).
    u32 sysId = 0;
    for (const auto& it : items) {
        if (it.isDir && it.parent == 2 && it.name == "System Folder") sysId = it.id;
    }
    if (!sysId) {
        std::fprintf(stderr, "no System Folder at the root\n");
        for (const auto& it : items) {
            if (it.parent == 2)
                std::printf("  root item: %s%s\n", it.name.c_str(), it.isDir ? "/" : "");
        }
        return 2;
    }

    hfs::VolumeBuilder b("Quadra HD");
    std::map<u32, u32> dirMap;   // source dir id -> built dir id
    dirMap[2] = 2;

    // Recursive copy of a directory's children.
    std::function<void(u32, u32)> copyDir = [&](u32 srcDir, u32 dstDir) {
        for (const auto& it : items) {
            if (it.parent != srcDir) continue;
            if (it.isDir) {
                const u32 nd = b.addDir(dstDir, it.name, it.crDate, it.mdDate);
                copyDir(it.id, nd);
            } else {
                std::vector<u8> data, rsrc;
                hfs::readFork(vol, it.id, false, data);
                hfs::readFork(vol, it.id, true, rsrc);
                b.addFile(dstDir, it.name, it.type, it.creator, it.fdFlags,
                          std::move(data), std::move(rsrc), it.crDate, it.mdDate);
            }
        }
    };
    const u32 dstSys = b.addDir(2, "System Folder");
    copyDir(sysId, dstSys);

    auto out = b.build(sizeMb * 1024u * 1024u);
    if (out.empty()) {
        std::fprintf(stderr, "build failed: %s\n", b.why().c_str());
        return 2;
    }
    // Boot blocks from the source volume make it bootable.
    std::memcpy(out.data(), vol.data(), 1024);

    std::ofstream f(outPath, std::ios::binary);
    f.write(reinterpret_cast<const char*>(out.data()),
            static_cast<std::streamsize>(out.size()));
    std::printf("wrote %s (%zu bytes)\n", outPath, out.size());
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    const char* romPath = nullptr;
    const char* hdPath = nullptr;
    const char* cdPath = nullptr;
    const char* shotPath = nullptr;
    const char* wavPath = nullptr;
    int frames = 600;
    int ramMb = 8;
    bool showLog = true;
    bool profile = false;
    int traceTraps = 0;      // log this many A-line traps (with opcodes)
    int trapsAfterCdbs = 0;  // ...once this many SCSI CDBs have run
    const char* findHex = nullptr;  // scan guest RAM for these bytes at exit
    bool inputTest = false;         // move the mouse + press a key at the end
    unsigned long dumpMem = 0;
    unsigned long breakPc = 0;
    int breakSkip = 0;
    unsigned long traceFrom = 0;
    int traceCount = 200;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--make-hd" && i + 3 < argc) {
            const char* srcP = argv[++i];
            const char* outP = argv[++i];
            const int mb = std::atoi(argv[++i]);
            return makeBootableHd(srcP, outP, static_cast<u32>(mb));
        }
        if (a == "--rom" && i + 1 < argc) romPath = argv[++i];
        else if (a == "--harddisk" && i + 1 < argc) hdPath = argv[++i];
        else if (a == "--cd" && i + 1 < argc) cdPath = argv[++i];
        else if (a == "--frames" && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (a == "--ram-mb" && i + 1 < argc) ramMb = std::atoi(argv[++i]);
        else if (a == "--dump-screen" && i + 1 < argc) shotPath = argv[++i];
        else if (a == "--dump-audio" && i + 1 < argc) wavPath = argv[++i];
        else if (a == "--no-log") showLog = false;
        else if (a == "--profile") profile = true;
        else if (a == "--dump-mem" && i + 1 < argc) dumpMem = std::strtoul(argv[++i], nullptr, 16);
        else if (a == "--trace-traps" && i + 2 < argc) {
            trapsAfterCdbs = std::atoi(argv[++i]);
            traceTraps = std::atoi(argv[++i]);
        }
        else if (a == "--find" && i + 1 < argc) findHex = argv[++i];
        else if (a == "--input-test") inputTest = true;
        else if (a == "--break-pc" && i + 1 < argc) breakPc = std::strtoul(argv[++i], nullptr, 16);
        else if (a == "--break-skip" && i + 1 < argc) breakSkip = std::atoi(argv[++i]);
        else if (a == "--trace-from" && i + 1 < argc) traceFrom = std::strtoul(argv[++i], nullptr, 16);
        else if (a == "--trace-count" && i + 1 < argc) traceCount = std::atoi(argv[++i]);
    }
    if (!romPath) {
        std::fprintf(stderr,
                     "usage: openmac_trace040 --rom <file> [--frames N] [--ram-mb N]\n"
                     "       [--harddisk <img>] [--dump-screen out.bmp] [--no-log] [--profile]\n");
        return 2;
    }

    auto rom = loadFile(romPath);
    if (rom.empty()) {
        std::fprintf(stderr, "cannot read ROM: %s\n", romPath);
        return 2;
    }

    QuadraMachine::Config cfg;
    cfg.ramSize = static_cast<u32>(ramMb) * 1024u * 1024u;
    QuadraMachine mac(std::move(rom), cfg);
    int cdbCount = 0;
    mac.onDiag = [&cdbCount](const char* m) {
        if (m[0] == 'C' && m[1] == 'D' && m[2] == 'B') ++cdbCount;
        std::printf("%s\n", m);
    };
    int vramBudget = 10;
    mac.onVramWrite = [&](u32 off, u32 pc) {
        if (vramBudget > 0) {
            --vramBudget;
            std::printf("VRAM W off=%06X pc=%08X\n", off, pc);
        }
    };
    std::map<u32, int> busErrSites;
    int excBudget = 60;
    mac.cpu().onException = [&](int vector, u32 pc) {
        if (vector == 10 && traceTraps > 0 && cdbCount >= trapsAfterCdbs) {
            --traceTraps;
            const u16 op = static_cast<u16>((mac.read8(pc) << 8) | mac.read8(pc + 1));
            std::printf("TRAP %04X pc=%08X a0=%08X d0=%08X\n", op, pc,
                        mac.cpu().a[0], mac.cpu().d[0]);
        }
        // A-line/F-line/TRAP are routine; bus errors are aggregated (the
        // ROM's presence probes take them on purpose).
        if (vector == 10 || vector == 11 || (vector >= 32 && vector < 48)) return;
        if (vector == 2) {
            ++busErrSites[pc];
            if (busErrSites[pc] <= 2)
                std::printf("BUSERR pc=%08X addr=%08X\n", pc, mac.cpu().lastFaultAddr);
            return;
        }
        if (excBudget > 0) {
            --excBudget;
            std::printf("EXC vec=%d pc=%08X\n", vector, pc);
        }
    };

    if (hdPath) {
        auto hd = loadFile(hdPath);
        if (hd.empty()) {
            std::fprintf(stderr, "cannot read disk: %s\n", hdPath);
            return 2;
        }
        mac.insertHardDisk(std::move(hd));
        std::printf("hd: attached\n");
    }
    if (cdPath) {
        auto cd = loadFile(cdPath);
        if (cd.empty()) {
            std::fprintf(stderr, "cannot read cd: %s\n", cdPath);
            return 2;
        }
        mac.attachCdRom(true, 3);
        mac.insertCd(std::move(cd));
        std::printf("cd: attached (id 3)\n");
    }

    if (traceFrom) {
        u64 guard = 3'000'000'000ull;
        while (mac.cpu().pc != static_cast<u32>(traceFrom) && !mac.cpu().halted && guard--) {
            mac.stepInstruction();
        }
        std::printf("-- tracing from %08X --\n", mac.cpu().pc);
        for (int i = 0; i < traceCount && !mac.cpu().halted; ++i) {
            const M68040& cc = mac.cpu();
            std::printf("%08X d0=%08X d1=%08X a0=%08X a1=%08X sr=%04X\n", cc.pc,
                        cc.d[0], cc.d[1], cc.a[0], cc.a[1], cc.getSR());
            mac.stepInstruction();
        }
        frames = 0;
    }

    if (breakPc) {
        // Single-step until the PC first reaches the target. Keep a separate
        // ring of PCs OUTSIDE the target's own module (the sad-mac painter
        // spins long enough to flush the CPU's ring), so the trail shows who
        // jumped in.
        const u32 modLo = static_cast<u32>(breakPc) & 0xFFFFFF80u;
        const u32 modHi = modLo + 0x80;
        u32 outside[48] = {};
        int op = 0;
        u32 prev = 0;
        int entries = 0;
        mac.cpu().onStep = [&](u32 pc) {
            if (pc < modLo || pc >= modHi) {
                outside[op] = pc;
                op = (op + 1) % 48;
            } else if (prev < modLo || prev >= modHi) {
                if (entries++ < 8)
                    std::printf("ENTER module: %08X -> %08X\n", prev, pc);
            }
            prev = pc;
        };
        u64 guard = 3'000'000'000ull;
        while (!mac.cpu().halted && guard--) {
            if (mac.cpu().pc == static_cast<u32>(breakPc)) {
                if (breakSkip <= 0) break;
                --breakSkip;
            }
            mac.stepInstruction();
        }
        mac.cpu().onStep = nullptr;
        std::printf("-- break at %08X after %llu cycles --\ntrail outside module:",
                    mac.cpu().pc, static_cast<unsigned long long>(mac.totalCycles()));
        for (int i = 0; i < 48; ++i) std::printf(" %08X", outside[(op + i) % 48]);
        std::printf("\n");
        frames = 0;
    }

    std::vector<u8> audio;
    for (int i = 0; i < frames; ++i) {
        if (profile && (i % 60) == 30) {
            // One frame of unique-PC ranges: the shape of the active code.
            std::map<u32, u32> ranges;
            u32 start = 0, prev = 0;
            bool open = false;
            const u64 target = mac.totalCycles() + 554260;
            while (mac.totalCycles() < target && !mac.cpu().halted) {
                const u32 pc = mac.cpu().pc;
                if (!open) { start = prev = pc; open = true; }
                else if (pc >= prev && pc - prev <= 16) prev = pc;
                else if (pc < prev && prev - pc <= 16) { /* small loop */ }
                else { ranges[start] = prev; start = prev = pc; }
                mac.stepInstruction();
            }
            if (open) ranges[start] = prev;
            std::printf("-- profile frame %d: %zu ranges --\n", i, ranges.size());
            int shown = 0;
            for (const auto& [s, e] : ranges) {
                std::printf("  %08X-%08X\n", s, e);
                if (++shown >= 24) break;
            }
            continue;
        }
        mac.runFrame();
        std::vector<u8> chunk;
        mac.drainAudio(chunk);
        audio.insert(audio.end(), chunk.begin(), chunk.end());
        if (mac.cpu().halted) {
            std::printf("HALTED at frame %d pc=%08X\n", i, mac.cpu().pc);
            break;
        }
    }
    {
        std::size_t loud = 0;
        for (u8 s : audio)
            if (s > 0x84 || s < 0x7C) ++loud;
        std::printf("audio: %zu samples, %zu non-silent\n", audio.size(), loud);
    }

    if (!busErrSites.empty()) {
        std::printf("\n-- bus-error sites --\n");
        for (const auto& [pc, n] : busErrSites)
            std::printf("  pc %08X x%d\n", pc, n);
    }

    const M68040& c = mac.cpu();
    std::printf("\n-- final state --\n");
    std::printf("pc=%08X sr=%04X a7=%08X overlay=%d frames=%llu\n", c.pc, c.getSR(),
                c.a[7], mac.overlayActive() ? 1 : 0,
                static_cast<unsigned long long>(mac.frameCount()));
    for (int i = 0; i < 8; ++i) std::printf("d%d=%08X ", i, c.d[i]);
    std::printf("\n");
    for (int i = 0; i < 8; ++i) std::printf("a%d=%08X ", i, c.a[i]);
    std::printf("\n");
    std::printf("mmu: tc=%08X itt0=%08X itt1=%08X dtt0=%08X dtt1=%08X srp=%08X urp=%08X\n",
                c.tc, c.itt0, c.itt1, c.dtt0, c.dtt1, c.srp, c.urp);
    std::printf("cacr=%08X vbr=%08X\n", c.cacr, c.vbr);
    std::printf("recent pcs:");
    for (int i = 0; i < 16; ++i) std::printf(" %08X", c.recentPc(i));
    std::printf("\nscreen: %dx%d\n", mac.screenWidth(), mac.screenHeight());
    const auto sd = mac.scsiDiag();
    std::printf("scsi: writes=%u selects=%u commands=%u lastCdb:", sd.writes,
                sd.selects, sd.commands);
    for (int i = 0; i < sd.lastCdbLen; ++i) std::printf(" %02X", sd.lastCdb[i]);
    std::printf("\n");

    if (showLog) {
        std::printf("\n-- access log (%zu) --\n", mac.accessLog().size());
        std::size_t shown = 0;
        for (const auto& line : mac.accessLog()) {
            std::printf("%s\n", line.c_str());
            if (++shown >= 60) {
                std::printf("... (%zu more)\n", mac.accessLog().size() - shown);
                break;
            }
        }
    }

    if (inputTest) {
        // The cursor is drawn at the boot-device wait screen; if the ADB
        // path works, injected motion moves it and the framebuffer changes.
        const int w = mac.screenWidth(), h = mac.screenHeight();
        std::vector<u32> before(static_cast<std::size_t>(w) * h);
        std::vector<u32> after(before.size());
        mac.renderScreen(before.data());
        for (int burst = 0; burst < 12; ++burst) {
            mac.mouseMove(10, 6, false);
            for (int f = 0; f < 4; ++f) mac.runFrame();
        }
        mac.keyEvent(0x00, true);    // 'A' down
        for (int f = 0; f < 6; ++f) mac.runFrame();
        mac.keyEvent(0x00, false);
        for (int f = 0; f < 20; ++f) mac.runFrame();
        mac.renderScreen(after.data());
        std::size_t diff = 0;
        for (std::size_t i = 0; i < before.size(); ++i)
            if (before[i] != after[i]) ++diff;
        std::printf("input test: %zu pixels changed; polls mouse=%u kbd=%u\n",
                    diff, mac.adbMousePolls(), mac.adbKbdPolls());
    }

    if (findHex) {
        std::vector<u8> pat;
        for (const char* p = findHex; p[0] && p[1]; p += 2) {
            auto nyb = [](char c) -> int {
                if (c >= '0' && c <= '9') return c - '0';
                if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                return 0;
            };
            pat.push_back(static_cast<u8>((nyb(p[0]) << 4) | nyb(p[1])));
        }
        const u32 ramTop = static_cast<u32>(ramMb) * 1024u * 1024u;
        int hits = 0;
        for (u32 a = 0; a + pat.size() <= ramTop && hits < 8; ++a) {
            bool ok = true;
            for (std::size_t k = 0; k < pat.size(); ++k)
                if (mac.read8(a + static_cast<u32>(k)) != pat[k]) { ok = false; break; }
            if (ok) {
                ++hits;
                std::printf("FOUND at %08X:", a);
                for (int k = 0; k < 32; ++k) std::printf(" %02X", mac.read8(a + static_cast<u32>(k)));
                std::printf("\n");
            }
        }
        if (!hits) std::printf("pattern not found in RAM\n");
    }

    if (dumpMem) {
        std::printf("\n-- memory at %08lX --\n", dumpMem);
        for (int row = 0; row < 8; ++row) {
            std::printf("%08lX:", dumpMem + static_cast<unsigned long>(row) * 16);
            for (int i = 0; i < 16; ++i)
                std::printf(" %02X", mac.read8(static_cast<u32>(dumpMem) +
                                               static_cast<u32>(row * 16 + i)));
            std::printf("\n");
        }
    }

    if (wavPath && !audio.empty()) {
        // Minimal WAV: PCM u8 mono at the machine's 22.25 kHz sample rate.
        std::ofstream f(wavPath, std::ios::binary);
        const u32 rate = 22254;
        const u32 dataSize = static_cast<u32>(audio.size());
        u8 hdr[44] = {'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E',
                      'f', 'm', 't', ' ', 16, 0, 0, 0, 1, 0, 1, 0};
        auto p32 = [&](int off, u32 v) {
            hdr[off] = static_cast<u8>(v);
            hdr[off + 1] = static_cast<u8>(v >> 8);
            hdr[off + 2] = static_cast<u8>(v >> 16);
            hdr[off + 3] = static_cast<u8>(v >> 24);
        };
        p32(4, 36 + dataSize);
        p32(24, rate);
        p32(28, rate);
        hdr[32] = 1;
        hdr[34] = 8;
        hdr[36] = 'd'; hdr[37] = 'a'; hdr[38] = 't'; hdr[39] = 'a';
        p32(40, dataSize);
        f.write(reinterpret_cast<char*>(hdr), 44);
        f.write(reinterpret_cast<const char*>(audio.data()),
                static_cast<std::streamsize>(audio.size()));
        std::printf("audio dumped: %s\n", wavPath);
    }

    if (shotPath) {
        const int w = mac.screenWidth(), h = mac.screenHeight();
        std::vector<u32> px(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
        mac.renderScreen(px.data());
        writeBmp(shotPath, px, w, h);
        std::printf("screen dumped: %s\n", shotPath);
    }
    return 0;
}
