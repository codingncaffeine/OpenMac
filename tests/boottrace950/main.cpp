// Headless Quadra 950 bring-up harness: starts the machine from a ROM file,
// runs a number of frames and reports what the ROM did -- how it sized
// memory, what its self-tests recorded in parameter RAM, where the CPU is.
//
//   openmac_trace950 --rom FILE [--banks A B C D] [--frames N] [--shot OUT.ppm]
//                    [--pram FILE] [--save-pram FILE] [--log] [--report] [--tests]
//                    [--display "13-inch RGB"]
//
// --banks gives the four SIMM banks in megabytes (0, 4, 8, 16, 32 or 64).
// Exit code 0 = ran to the end; 1 = the CPU halted; 2 = bad arguments.

#include "openmac/quadra950.hpp"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace openmac;

namespace {

std::vector<u8> readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return std::vector<u8>(std::istreambuf_iterator<char>(f), {});
}

} // namespace

int main(int argc, char** argv) {
    std::string romPath, shotPath, pramIn, pramOut, display;
    Quadra950Machine::Config cfg;
    int frames = 600;
    bool log = false, report = false, tests = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--rom" && i + 1 < argc) romPath = argv[++i];
        else if (a == "--frames" && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (a == "--banks" && i + 4 < argc) {
            for (std::size_t b = 0; b < 4; ++b) cfg.bankMb[b] = static_cast<u32>(std::atoi(argv[++i]));
        }
        else if (a == "--shot" && i + 1 < argc) shotPath = argv[++i];
        else if (a == "--pram" && i + 1 < argc) pramIn = argv[++i];
        else if (a == "--save-pram" && i + 1 < argc) pramOut = argv[++i];
        else if (a == "--display" && i + 1 < argc) display = argv[++i];
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

    Quadra950Machine mac(std::move(rom), cfg);
    if (!display.empty()) {
        if (!mac.setDisplay(display.c_str())) {
            std::fprintf(stderr, "no display named '%s'\n", display.c_str());
            return 2;
        }
        mac.reset();   // the ROM reads the sense lines at startup
    }
    u32 lastFrame = 0;
    if (log) mac.onDiag = [&](const char* m) { std::printf("f=%u %s\n", lastFrame, m); };
    // The ROM's startup self-test loop calls each test at $46F7A (number in
    // D7, routine in A4) and reads its verdict in D6 at $46F7C; zero is a
    // pass. This is the verdict the tool reports: the ROM also logs failures
    // in parameter RAM, but clears that later in the same startup.
    int testsRun = 0, testsFailed = 0;
    std::string failedList;
    u32 currentTest = 0;
    mac.cpu().onStep = [&](u32 pc) {
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
    if (!pramIn.empty()) {
        const std::vector<u8> blob = readFile(pramIn);
        std::printf("PRAM %s: %s\n", pramIn.c_str(),
                    mac.loadPram(blob.data(), static_cast<u32>(blob.size()), 0) ? "loaded" : "NOT loaded");
    }
    std::printf("Quadra 950: banks %u/%u/%u/%u MB (%u MB fitted), %d frames\n", cfg.bankMb[0], cfg.bankMb[1],
                cfg.bankMb[2], cfg.bankMb[3], mac.ramBytes() >> 20, frames);

    bool halted = false;
    for (int f = 0; f < frames; ++f) {
        lastFrame = static_cast<u32>(f);
        mac.runFrame();
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
    if (report) std::fputs(mac.diagnosticReport().c_str(), stdout);

    if (!shotPath.empty()) {
        const int w = mac.screenWidth(), h = mac.screenHeight();
        std::vector<u32> px(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
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
