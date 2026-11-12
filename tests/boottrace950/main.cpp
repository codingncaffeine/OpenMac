// Headless Quadra 950 bring-up harness: starts the machine from a ROM file,
// runs a number of frames and reports what the ROM did -- how it sized
// memory, what its self-tests recorded in parameter RAM, where the CPU is.
//
//   openmac_trace950 --rom FILE [--banks A B C D] [--frames N] [--shot OUT.ppm]
//                    [--pram FILE] [--save-pram FILE] [--log] [--report] [--tests]
//                    [--display "13-inch RGB"] [--sense GROUNDED PAIRS]
//                    [--vram 1|2] [--pram-byte OFFSET VALUE]... [--hashes FROM STEP]
//                    [--peek ADDRESS COUNT]... [--watch-pc ADDRESS]...
//                    [--mouse-at FRAME DX DY BUTTON]... [--key-at FRAME CODE down|up]...
//                    [--floppy FILE] [--floppy-writable] [--save-floppy OUT]
//                    [--floppy-at FRAME FILE]... [--eject-floppy-at FRAME]...
//
// --pram-byte changes one byte of parameter RAM after --pram is loaded (the
// stored video mode is byte $48). --hashes prints the distinct pictures seen
// from frame FROM on, sampled every STEP frames. --peek ADDRESS COUNT prints
// guest memory at the end. --watch-pc ADDRESS prints the registers each time
// the CPU reaches it (the first 60 times).
// --banks gives the four SIMM banks in megabytes (0, 4, 8, 16, 32 or 64).
// Exit code 0 = ran to the end; 1 = the CPU halted; 2 = bad arguments.

#include "openmac/quadra950.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace openmac;

namespace {

std::vector<u8> readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return std::vector<u8>(std::istreambuf_iterator<char>(f), {});
}

} // namespace

int main(int argc, char** argv) {
    std::string romPath, shotPath, pramIn, pramOut, display, floppyPath, floppyOut;
    bool floppyWritable = false;
    std::vector<std::pair<int, std::string>> floppyEvents;   // frame, file ("" = the host's Eject)
    Quadra950Machine::Config cfg;
    int frames = 600;
    bool log = false, report = false, tests = false;
    int senseGrounded = -1, sensePairs = 0;
    int hashFrom = -1, hashStep = 1;
    std::vector<std::pair<u32, u32>> peeks;
    std::vector<u32> watchPcs;
    struct Input { int frame, kind, a, b, c; };   // kind 0 = mouse (dx, dy, button), 1 = key (code, down)
    std::vector<Input> inputs;
    std::vector<std::pair<int, int>> pramBytes;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--rom" && i + 1 < argc) romPath = argv[++i];
        else if (a == "--frames" && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (a == "--banks" && i + 4 < argc) {
            for (std::size_t b = 0; b < 4; ++b) cfg.bankMb[b] = static_cast<u32>(std::atoi(argv[++i]));
        }
        else if (a == "--shot" && i + 1 < argc) shotPath = argv[++i];
        else if (a == "--floppy" && i + 1 < argc) floppyPath = argv[++i];
        else if (a == "--floppy-writable") floppyWritable = true;
        else if (a == "--save-floppy" && i + 1 < argc) floppyOut = argv[++i];
        else if (a == "--pram" && i + 1 < argc) pramIn = argv[++i];
        else if (a == "--save-pram" && i + 1 < argc) pramOut = argv[++i];
        else if (a == "--display" && i + 1 < argc) display = argv[++i];
        else if (a == "--sense" && i + 2 < argc) {
            senseGrounded = static_cast<int>(std::strtoul(argv[++i], nullptr, 0));
            sensePairs = static_cast<int>(std::strtoul(argv[++i], nullptr, 0));
        }
        else if (a == "--vram" && i + 1 < argc) cfg.vramMb = static_cast<u32>(std::atoi(argv[++i]));
        else if (a == "--pram-byte" && i + 2 < argc) {
            const int off = static_cast<int>(std::strtoul(argv[++i], nullptr, 0));
            const int val = static_cast<int>(std::strtoul(argv[++i], nullptr, 0));
            pramBytes.emplace_back(off & 0xFF, val & 0xFF);
        }
        else if (a == "--watch-pc" && i + 1 < argc) watchPcs.push_back(static_cast<u32>(std::strtoul(argv[++i], nullptr, 0)));
        else if (a == "--mouse-at" && i + 4 < argc) {
            Input in{std::atoi(argv[i + 1]), 0, std::atoi(argv[i + 2]), std::atoi(argv[i + 3]), std::atoi(argv[i + 4])};
            i += 4;
            inputs.push_back(in);
        }
        else if (a == "--floppy-at" && i + 2 < argc) {
            const int at = std::atoi(argv[++i]);
            floppyEvents.emplace_back(at, argv[++i]);
        }
        else if (a == "--eject-floppy-at" && i + 1 < argc) floppyEvents.emplace_back(std::atoi(argv[++i]), "");
        else if (a == "--key-at" && i + 3 < argc) {
            Input in{std::atoi(argv[i + 1]), 1, static_cast<int>(std::strtoul(argv[i + 2], nullptr, 0)),
                     std::string(argv[i + 3]) == "down" ? 1 : 0, 0};
            i += 3;
            inputs.push_back(in);
        }
        else if (a == "--peek" && i + 2 < argc) {
            const u32 at = static_cast<u32>(std::strtoul(argv[++i], nullptr, 0));
            peeks.emplace_back(at, static_cast<u32>(std::strtoul(argv[++i], nullptr, 0)));
        }
        else if (a == "--hashes" && i + 2 < argc) {
            hashFrom = std::atoi(argv[++i]);
            hashStep = std::max(1, std::atoi(argv[++i]));
        }
        else if (a == "--log") log = true;
        else if (a == "--report") report = true;
        else if (a == "--tests") tests = true;
        else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
    }
    std::vector<u8> rom = readFile(romPath);
    if (rom.size() != 0x100000) {
        std::fprintf(stderr, "ROM: want a 1 MB image, got %zu bytes from '%s'\n", rom.size(), romPath.c_str());
        return 2;
    }
    if (cfg.vramMb != 1 && cfg.vramMb != 2) {
        std::fprintf(stderr, "--vram: 1 or 2 (megabytes)\n");
        return 2;
    }

    Quadra950Machine mac(std::move(rom), cfg);
    if (!display.empty()) {
        if (!mac.setDisplay(display.c_str())) {
            std::fprintf(stderr, "no display named '%s'\n", display.c_str());
            return 2;
        }
        mac.reset();   // the ROM reads the sense lines at startup
    }
    if (senseGrounded >= 0) {
        mac.setMonitorSense(static_cast<u32>(senseGrounded), static_cast<u32>(sensePairs));
        mac.reset();
    }
    u32 lastFrame = 0;
    if (log) mac.onDiag = [&](const char* m) { std::printf("f=%u pc=%08X %s\n", lastFrame, mac.cpu().pc, m); };
    // The ROM's startup self-test loop calls each test at $46F7A (number in
    // D7, routine in A4) and reads its verdict in D6 at $46F7C; zero is a
    // pass. This is the verdict the tool reports: the ROM also logs failures
    // in parameter RAM, but clears that later in the same startup.
    int testsRun = 0, testsFailed = 0;
    std::string failedList;
    u32 currentTest = 0;
    int watchBudget = 60;
    mac.cpu().onStep = [&](u32 pc) {
        for (u32 w : watchPcs) {
            if (pc != w || watchBudget <= 0) continue;
            --watchBudget;
            const auto& c = mac.cpu();
            std::printf("at %08X: D0=%08X D1=%08X D2=%08X D4=%08X D5=%08X D6=%08X A0=%08X A2=%08X A3=%08X A6=%08X\n",
                        pc, c.d[0], c.d[1], c.d[2], c.d[4], c.d[5], c.d[6], c.a[0], c.a[2], c.a[3], c.a[6]);
        }
        if (pc == 0x40846F7Au) {
            currentTest = mac.cpu().d[7] & 0xFFFFu;
            if (tests) std::printf("self-test $%02X at %08X ...", currentTest, mac.cpu().a[4]);
        } else if (pc == 0x40846F7Cu) {
            ++testsRun;
            const u32 verdict = mac.cpu().d[6];
            if (tests) std::printf(" D6=%08X %s\n", verdict, verdict ? "FAILED" : "ok");
            if (verdict) {
                ++testsFailed;
                char one[40];
                std::snprintf(one, sizeof one, " $%02X(%08X)", currentTest, verdict);
                failedList += one;
            }
        }
    };
    if (!floppyPath.empty()) {
        // The file given is never changed: a writable disk lives in memory
        // and goes to --save-floppy at the end.
        const bool ok = mac.insertFloppy(readFile(floppyPath), !floppyWritable);
        std::printf("floppy %s: %s\n", floppyPath.c_str(), ok ? "inserted" : "NOT a floppy image");
        if (!ok) return 2;
    }
    if (!pramIn.empty()) {
        const std::vector<u8> blob = readFile(pramIn);
        std::printf("PRAM %s: %s\n", pramIn.c_str(),
                    mac.loadPram(blob.data(), static_cast<u32>(blob.size()), 0) ? "loaded" : "NOT loaded");
    }
    if (!pramBytes.empty()) {
        u8 blob[Quadra950Machine::kPramBlobBytes];
        mac.savePram(blob, sizeof blob);
        for (const auto& [off, val] : pramBytes) {
            std::printf("parameter RAM $%02X: %02X -> %02X\n", off, blob[8 + off], val);
            blob[8 + off] = static_cast<u8>(val);
        }
        mac.loadPram(blob, sizeof blob, 0);
    }
    std::printf("Quadra 950: banks %u/%u/%u/%u MB (%u MB fitted), %d frames\n", cfg.bankMb[0], cfg.bankMb[1],
                cfg.bankMb[2], cfg.bankMb[3], mac.ramBytes() >> 20, frames);

    std::vector<u32> px;
    const auto screenHash = [&] {
        const int w = mac.screenWidth(), h = mac.screenHeight();
        px.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
        mac.renderScreen(px.data());
        u32 hash = 2166136261u;
        for (u32 p : px) hash = (hash ^ (p & 0xFFFFFFu)) * 16777619u;
        return hash;
    };
    std::map<u32, int> hashes;

    bool halted = false;
    bool floppyIn = mac.floppyPresent();
    for (int f = 0; f < frames; ++f) {
        lastFrame = static_cast<u32>(f);
        if (mac.floppyPresent() != floppyIn) {
            floppyIn = !floppyIn;
            if (!floppyIn) std::printf("floppy left the drive by frame %d\n", f);
        }
        for (const auto& [at, file] : floppyEvents) {
            if (at != f) continue;
            if (file.empty()) {
                mac.ejectFloppy();
                floppyIn = mac.floppyPresent();
                std::printf("floppy eject asked for at frame %d%s\n", f, floppyIn ? "" : " (out at once)");
            } else {
                const bool ok = mac.insertFloppy(readFile(file), !floppyWritable);
                floppyIn = mac.floppyPresent();
                std::printf("floppy %s at frame %d: %s\n", file.c_str(), f, ok ? "inserted" : "NOT a floppy image");
            }
        }
        for (const Input& in : inputs) {
            if (in.frame != f) continue;
            if (in.kind == 0) mac.mouseEvent(in.a, in.b, in.c != 0);
            else mac.keyEvent(static_cast<u8>(in.a), in.b != 0);
        }
        mac.runFrame();
        // The verdict hook costs a call per instruction; the ROM runs its
        // self-tests once, so it comes off when they are through.
        if (testsRun >= 5 && watchPcs.empty() && mac.cpu().onStep) mac.cpu().onStep = nullptr;
        if (hashFrom >= 0 && f >= hashFrom && (f - hashFrom) % hashStep == 0) ++hashes[screenHash()];
        if (mac.cpu().halted) { halted = true; std::printf("CPU HALTED in frame %d\n", f); break; }
        if (mac.powerDownRequested) { std::printf("the guest switched the machine off in frame %d\n", f); break; }
    }

    std::printf("memory controller boundaries: %u, %u, %u MB\n", mac.bankBoundary(0) >> 20,
                mac.bankBoundary(1) >> 20, mac.bankBoundary(2) >> 20);
    const auto& pram = mac.pram();
    if (testsFailed)
        std::printf("SELF-TESTS: %d run, %d FAILED:%s\n", testsRun, testsFailed, failedList.c_str());
    else
        std::printf("self-tests: %d run, all passed\n", testsRun);
    std::printf("parameter RAM $0C-$0F: %02X %02X %02X %02X (%s)\n", pram[0x0C], pram[0x0D], pram[0x0E], pram[0x0F],
                pram[0x0C] == 'N' && pram[0x0D] == 'u' && pram[0x0E] == 'M' && pram[0x0F] == 'c'
                    ? "valid signature" : "no signature");
    std::printf("end: frame %llu pc=%08X sr=%04X overlay=%d screen %dx%d\n",
                static_cast<unsigned long long>(mac.frameCount()), mac.cpu().pc, mac.cpu().getSR(),
                mac.overlayActive() ? 1 : 0, mac.screenWidth(), mac.screenHeight());
    if (report) {
        std::fputs(mac.diagnosticReport().c_str(), stdout);
        // What the guest believes about its screen: ScrnBase ($0824) and
        // ScreenRow ($0106), next to what the video chip was told.
        std::printf("   guest: ScrnBase=%08X ScreenRow=%u bytes\n", mac.read32(0x0824), mac.read16(0x0106));
        // The pointer (Mouse $0830: vertical, horizontal), the button (MBState
        // $0172, bit 7 clear = down) and the first key map word ($0174).
        std::printf("   guest: pointer at %d,%d button %s, KeyMap %08X %08X\n", static_cast<s16>(mac.read16(0x0832)),
                    static_cast<s16>(mac.read16(0x0830)), (mac.read8(0x0172) & 0x80) ? "up" : "down",
                    mac.read32(0x0174), mac.read32(0x0178));
    }
    // One line a script can read: what the chip is set to, and what the
    // guest's main screen device says of itself (MainDevice $08A4 -> GDevice
    // -> its pixel map: row bytes, bounds, pixel size).
    {
        const Quadra950Machine::VideoState v = mac.videoState();
        std::printf("video: chip %d bits, raster %dx%d, shown %dx%d, %u KB video RAM, pixel clock %u Hz, %u.%02u Hz",
                    v.bitsPerPixel, v.timingWidth, v.timingHeight, mac.screenWidth(), mac.screenHeight(),
                    v.vramBytes >> 10, v.pixelClockHz, v.scansMilliHz / 1000, (v.scansMilliHz % 1000) / 10);
        const auto ramPtr = [&](u32 a) { return a != 0 && (a & 1u) == 0 && a < mac.ramBytes() - 64; };
        // A 24-bit Memory Manager keeps flags in a master pointer's top byte
        // (the ROM's startup heap is one, whatever the MMU is set to).
        const auto strip = [&](u32 a) { return a < mac.ramBytes() ? a : a & 0x00FFFFFFu; };
        try {
            const u32 devHandle = mac.read32(0x08A4);
            const u32 dev = ramPtr(devHandle) ? strip(mac.read32(devHandle)) : 0;
            const u32 mapHandle = ramPtr(dev) ? mac.read32(dev + 22) : 0;
            const u32 map = ramPtr(mapHandle) ? strip(mac.read32(mapHandle)) : 0;
            if (ramPtr(map)) {
                std::printf(" | guest: %d bits, %dx%d, row %u bytes, base %08X, stored mode $%02X\n",
                            mac.read16(map + 32), mac.read16(map + 12) - mac.read16(map + 8),
                            mac.read16(map + 10) - mac.read16(map + 6), mac.read16(map + 4) & 0x3FFFu,
                            mac.read32(map), mac.pram()[0x48]);
            } else {
                std::printf(" | guest: no screen device\n");
            }
        } catch (const BusFault&) {
            std::printf(" | guest: no screen device\n");
        }
    }
    // What the guest has running and mounted: the current application's
    // name (CurApName $0910) and the volume names down the VCB queue ($0358).
    {
        const auto pascal = [&](u32 at, u32 limit) {
            std::string text;
            try {
                const u32 n = mac.read8(at);
                if (n > limit) return text;        // not a name: nothing has set it yet
                for (u32 k = 1; k <= n; ++k) {
                    const u8 c = mac.read8(at + k);
                    text += (c >= 0x20 && c < 0x7F) ? static_cast<char>(c) : '?';
                }
            } catch (const BusFault&) {
            }
            return text;
        };
        std::string volumes;
        try {
            u32 vcb = mac.read32(0x0358);
            for (int n = 0; n < 8 && vcb != 0 && vcb < mac.ramBytes(); ++n) {
                volumes += (volumes.empty() ? "" : ", ") + pascal(vcb + 44, 27);
                vcb = mac.read32(vcb);
            }
        } catch (const BusFault&) {
        }
        std::printf("guest: application '%s', volumes [%s]\n", pascal(0x0910, 31).c_str(), volumes.c_str());
    }
    if (!floppyOut.empty()) {
        const std::vector<u8>& sectors = mac.floppySectors();
        std::ofstream out(floppyOut, std::ios::binary);
        out.write(reinterpret_cast<const char*>(sectors.data()), static_cast<std::streamsize>(sectors.size()));
        std::printf("floppy sectors saved: %s (%zu bytes)\n", floppyOut.c_str(), sectors.size());
    }
    for (const auto& [at, count] : peeks) {
        std::printf("peek %08X:", at);
        try {
            for (u32 k = 0; k < count; ++k) std::printf(" %02X", mac.read8(at + k));
        } catch (const BusFault&) {
            std::printf(" (bus error)");
        }
        std::printf("\n");
    }
    if (hashFrom >= 0) {
        std::printf("pictures:");
        for (const auto& [h, n] : hashes) std::printf(" %08X x%d", h, n);
        std::printf("\n");
    }

    if (!shotPath.empty()) {
        const int w = mac.screenWidth(), h = mac.screenHeight();
        px.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
        mac.renderScreen(px.data());
        std::ofstream out(shotPath, std::ios::binary);
        out << "P6\n" << w << " " << h << "\n255\n";
        for (u32 p : px) {
            const char rgb[3] = {static_cast<char>(p >> 16), static_cast<char>(p >> 8), static_cast<char>(p)};
            out.write(rgb, 3);
        }
        std::printf("framebuffer saved: %s (%dx%d)\n", shotPath.c_str(), w, h);
    }
    if (!pramOut.empty()) {
        u8 blob[Quadra950Machine::kPramBlobBytes];
        if (mac.savePram(blob, sizeof blob)) {
            std::ofstream out(pramOut, std::ios::binary);
            out.write(reinterpret_cast<const char*>(blob), sizeof blob);
        }
    }
    return halted ? 1 : 0;
}
