// Headless Macintosh SE/30 bring-up harness: starts the machine from a ROM
// file, runs a number of frames and reports what the ROM did -- how it sized
// memory, which machine it took the board for, whether a start-up test
// failed, what is on the screen, where the CPU is.
//
//   openmac_tracese30 --rom FILE [--vrom FILE] [--ram MB | --banks A B]
//                     [--frames N] [--shot OUT.ppm] [--pram FILE] [--save-pram FILE]
//                     [--log] [--report] [--hashes FROM STEP] [--peek ADDRESS COUNT]...
//                     [--mouse-at FRAME DX DY BUTTON]... [--key-at FRAME CODE down|up]...
//
// --ram takes one of the totals the machine offers (1 2 4 5 8 16 17 20 32 65
// 68 80 128); --banks gives the two SIMM banks in megabytes (0, 1, 4, 16 or
// 64) for sets the machine does not offer. --vrom is the 8 KB video ROM;
// without it slot $E is empty. --hashes prints the distinct pictures seen
// from frame FROM on, sampled every STEP frames. --peek prints guest memory
// at the end.
//
// The start-up tests' verdict is read where the ROM takes it: a failed test
// jumps to ROM $2C3C with the test's number in D7 and the failing bits in
// D6; the start-up chime's routine at $5E4A is reached only when none failed.
// Exit code 0 = ran to the end; 1 = the CPU halted; 2 = bad arguments.

#include "openmac/se30.hpp"

#include "../common/tracetools.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include <vector>

using namespace openmac;

namespace {

u32 pictureHash(const std::vector<u32>& px) {
    u32 h = 2166136261u;
    for (u32 p : px) {
        h ^= p & 0xFFu;
        h *= 16777619u;
    }
    return h;
}

struct MouseEvent { int frame, dx, dy; bool button; };
struct KeyEvent { int frame; u8 code; bool down; };

} // namespace

int main(int argc, char** argv) {
    std::string romPath, vromPath, shotPath, pramIn, pramOut;
    int frames = 600;
    u32 bankA = 4, bankB = 4;
    bool log = false, report = false;
    int hashFrom = -1, hashStep = 1;
    std::vector<std::pair<u32, int>> peeks;
    std::vector<MouseEvent> mouse;
    std::vector<KeyEvent> keys;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--rom" && i + 1 < argc) romPath = argv[++i];
        else if (a == "--vrom" && i + 1 < argc) vromPath = argv[++i];
        else if (a == "--frames" && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (a == "--ram" && i + 1 < argc) {
            const u32 total = static_cast<u32>(std::atoi(argv[++i]));
            if (!Se30Machine::banksForTotal(total, bankA, bankB)) {
                std::fprintf(stderr, "--ram %u: not a size the SE/30 offers\n", total);
                return 2;
            }
        }
        else if (a == "--banks" && i + 2 < argc) {
            bankA = static_cast<u32>(std::atoi(argv[++i]));
            bankB = static_cast<u32>(std::atoi(argv[++i]));
        }
        else if (a == "--shot" && i + 1 < argc) shotPath = argv[++i];
        else if (a == "--pram" && i + 1 < argc) pramIn = argv[++i];
        else if (a == "--save-pram" && i + 1 < argc) pramOut = argv[++i];
        else if (a == "--log") log = true;
        else if (a == "--report") report = true;
        else if (a == "--hashes" && i + 2 < argc) { hashFrom = std::atoi(argv[++i]); hashStep = std::atoi(argv[++i]); }
        else if (a == "--peek" && i + 2 < argc) {
            const u32 at = static_cast<u32>(std::strtoul(argv[++i], nullptr, 0));
            peeks.emplace_back(at, std::atoi(argv[++i]));
        }
        else if (a == "--mouse-at" && i + 4 < argc) {
            MouseEvent e{};
            e.frame = std::atoi(argv[++i]); e.dx = std::atoi(argv[++i]); e.dy = std::atoi(argv[++i]);
            e.button = std::atoi(argv[++i]) != 0;
            mouse.push_back(e);
        }
        else if (a == "--key-at" && i + 3 < argc) {
            KeyEvent e{};
            e.frame = std::atoi(argv[++i]);
            e.code = static_cast<u8>(std::strtoul(argv[++i], nullptr, 0));
            e.down = std::string(argv[++i]) == "down";
            keys.push_back(e);
        }
        else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
    }
    if (romPath.empty()) { std::fprintf(stderr, "usage: openmac_tracese30 --rom FILE [options]\n"); return 2; }
    if (hashStep < 1) hashStep = 1;

    std::vector<u8> rom = tracetools::loadFile(romPath.c_str());
    if (rom.size() != 0x40000) { std::fprintf(stderr, "ROM: want 262144 bytes, got %zu\n", rom.size()); return 2; }
    Se30Machine::Config cfg;
    cfg.bankAMb = bankA;
    cfg.bankBMb = bankB;
    if (!vromPath.empty()) {
        cfg.videoRom = tracetools::loadFile(vromPath.c_str());
        if (cfg.videoRom.empty()) { std::fprintf(stderr, "video ROM: cannot read %s\n", vromPath.c_str()); return 2; }
    }
    Se30Machine mac(std::move(rom), cfg);
    if (!pramIn.empty()) {
        const std::vector<u8> blob = tracetools::loadFile(pramIn.c_str());
        const bool ok = mac.loadPram(blob.data(), static_cast<u32>(blob.size()), 0);
        std::printf("parameter RAM %s: %s\n", pramIn.c_str(), ok ? "loaded" : "NOT a parameter RAM file");
        if (!ok) return 2;
    }
    u32 lastFrame = 0;
    if (log) mac.onDiag = [&](const char* m) { std::printf("f=%u pc=%08X %s\n", lastFrame, mac.cpu().pc, m); };

    // The verdict of the start-up tests, read at the ROM's own two exits.
    bool testFailed = false, chimeReached = false;
    u32 failedTest = 0, failedBits = 0;
    int failedFrame = 0, chimeFrame = 0;
    mac.cpu().onStep = [&](u32 pc) {
        if (pc == 0x40802C3Cu && !testFailed && !chimeReached) {
            testFailed = true;
            failedTest = mac.cpu().d[7] & 0xFFFFu;
            failedBits = mac.cpu().d[6];
            failedFrame = static_cast<int>(lastFrame);
        } else if (pc == 0x40805E4Au && !chimeReached && !testFailed) {
            chimeReached = true;
            chimeFrame = static_cast<int>(lastFrame);
        }
    };

    std::vector<u32> px(static_cast<std::size_t>(mac.screenWidth()) * static_cast<std::size_t>(mac.screenHeight()));
    std::map<u32, int> hashes;   // picture -> first frame seen
    std::vector<u8> audio;
    u64 audioSamples = 0, loudSamples = 0;
    for (int f = 0; f < frames; ++f) {
        lastFrame = static_cast<u32>(f);
        for (const MouseEvent& e : mouse) if (e.frame == f) mac.mouseEvent(e.dx, e.dy, e.button);
        for (const KeyEvent& e : keys) if (e.frame == f) mac.keyEvent(e.code, e.down);
        mac.runFrame();
        if (mac.cpu().halted) break;
        // The hook costs a call per instruction; both exits are passed in
        // the first frames.
        if ((chimeReached || testFailed) && mac.cpu().onStep) mac.cpu().onStep = nullptr;
        mac.drainAudio(audio);
        audioSamples += audio.size();
        for (u8 s : audio) if (s < 0x7C || s > 0x84) ++loudSamples;
        if (hashFrom >= 0 && f >= hashFrom && (f - hashFrom) % hashStep == 0) {
            mac.renderScreen(px.data());
            hashes.emplace(pictureHash(px), f);
        }
    }

    const Se30Machine::BoardState board = mac.boardState();
    const bool up = !mac.overlayActive();
    std::printf("ram: bank A %u MB, bank B %u MB, RAM-size bits %u%u (bank B from $%08X)", bankA, bankB,
                board.ramSizeBits >> 1, board.ramSizeBits & 1, board.bankBStart);
    if (up) std::printf(", MemTop $%08X, machine number %u\n", mac.read32(0x0108), mac.read8(0x0CB3));
    else std::printf(", overlay still on\n");
    if (testFailed)
        std::printf("START-UP TEST $%02X FAILED at frame %d, failing bits %08X\n", failedTest, failedFrame, failedBits);
    else if (chimeReached)
        std::printf("start-up tests: none failed (chime at frame %d)\n", chimeFrame);
    else
        std::printf("START-UP TESTS: neither exit reached\n");
    const auto& pram = mac.pram();
    std::printf("parameter RAM $0C-$0F: %02X %02X %02X %02X (%s)\n", pram[0x0C], pram[0x0D], pram[0x0E], pram[0x0F],
                pram[0x0C] == 'N' && pram[0x0D] == 'u' && pram[0x0E] == 'M' && pram[0x0F] == 'c'
                    ? "valid signature" : "no signature");
    mac.renderScreen(px.data());
    std::printf("screen: %s buffer, picture %08X, interrupt %s, %llu requests in %llu frames\n",
                board.mainScreenBuffer ? "main" : "alternate", pictureHash(px),
                board.screenInterruptEnabled ? "enabled" : "off",
                static_cast<unsigned long long>(board.screenInterrupts),
                static_cast<unsigned long long>(mac.frameCount()));
    std::printf("sound: %llu samples, %llu off the center line\n", static_cast<unsigned long long>(audioSamples),
                static_cast<unsigned long long>(loudSamples));
    if (up) {
        // What the guest believes about its screen (ScrnBase $0824, ScreenRow
        // $0106), its pointer (Mouse $0830: vertical, horizontal; MBState
        // $0172, bit 7 clear = down), its keys ($0174) and its clock ($016A).
        std::printf("guest: ScrnBase=%08X ScreenRow=%u, pointer at %d,%d button %s, KeyMap %08X %08X, Ticks %u\n",
                    mac.read32(0x0824), mac.read16(0x0106), static_cast<s16>(mac.read16(0x0832)),
                    static_cast<s16>(mac.read16(0x0830)), (mac.read8(0x0172) & 0x80) ? "up" : "down",
                    mac.read32(0x0174), mac.read32(0x0178), mac.read32(0x016A));
    }
    std::printf("end: frame %llu pc=%08X sr=%04X overlay=%d%s\n", static_cast<unsigned long long>(mac.frameCount()),
                mac.cpu().pc, mac.cpu().getSR(), mac.overlayActive() ? 1 : 0, mac.cpu().halted ? " HALTED" : "");
    if (report) std::fputs(mac.diagnosticReport().c_str(), stdout);
    if (hashFrom >= 0) {
        std::printf("pictures from frame %d, every %d: %zu distinct:", hashFrom, hashStep, hashes.size());
        for (const auto& [h, f] : hashes) std::printf(" %08X@%d", h, f);
        std::printf("\n");
    }
    for (const auto& [at, count] : peeks) {
        std::printf("peek %08X:", at);
        for (int k = 0; k < count; ++k) {
            try { std::printf(" %08X", mac.read32(at + static_cast<u32>(k) * 4)); }
            catch (const BusFault&) { std::printf(" --------"); }
        }
        std::printf("\n");
    }
    if (!shotPath.empty()) {
        std::ofstream out(shotPath, std::ios::binary);
        out << "P6\n" << mac.screenWidth() << " " << mac.screenHeight() << "\n255\n";
        for (u32 p : px) {
            const char rgb[3] = {static_cast<char>(p >> 16), static_cast<char>(p >> 8), static_cast<char>(p)};
            out.write(rgb, 3);
        }
        std::printf("screen written to %s\n", shotPath.c_str());
    }
    if (!pramOut.empty()) {
        std::vector<u8> blob(Se30Machine::kPramBlobBytes);
        mac.savePram(blob.data(), static_cast<u32>(blob.size()));
        std::ofstream out(pramOut, std::ios::binary);
        out.write(reinterpret_cast<const char*>(blob.data()), static_cast<std::streamsize>(blob.size()));
        std::printf("parameter RAM written to %s\n", pramOut.c_str());
    }
    return mac.cpu().halted ? 1 : 0;
}
