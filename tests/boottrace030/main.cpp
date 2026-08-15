// Headless Macintosh IIfx ROM bring-up. Runs the supplied ROM on the real
// machine model and records exceptions, frame milestones, device traffic and
// a deterministic final state. This is intentionally small enough to remain
// useful while every new hardware block is being brought online.

#include <openmac/iifx.hpp>
#include <openmac/debugger.hpp>
#include <openmac/hfs.hpp>
#include "host_sampler.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

using namespace openmac;

namespace {

std::vector<u8> loadFile(const char* path) {
    std::ifstream file(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

void savePixels(const std::vector<u32>& pixels, int width, int height,
                const char* path) {
    if (!path || !*path || width <= 0 || height <= 0 ||
        pixels.size() != static_cast<std::size_t>(width) * height)
        return;
    std::ofstream file(path, std::ios::binary);
    file << "P6\n" << width << ' ' << height << "\n255\n";
    for (u32 pixel : pixels) {
        const char rgb[3] = {
            static_cast<char>((pixel >> 16) & 0xFF),
            static_cast<char>((pixel >> 8) & 0xFF),
            static_cast<char>(pixel & 0xFF)};
        file.write(rgb, sizeof rgb);
    }
    std::printf("framebuffer saved: %s (%dx%d)\n", path, width, height);
}

void saveFrame(const IifxMachine& mac, const char* path) {
    if (!path || !*path) return;
    const int width = mac.screenWidth();
    const int height = mac.screenHeight();
    std::vector<u32> pixels(static_cast<std::size_t>(width) * height);
    mac.renderScreen(pixels.data());
    savePixels(pixels, width, height, path);
}

u64 pixelHash(const std::vector<u32>& pixels, int width, int height,
              int left, int top, int right, int bottom) {
    left = std::clamp(left, 0, width);
    right = std::clamp(right, left, width);
    top = std::clamp(top, 0, height);
    bottom = std::clamp(bottom, top, height);
    u64 hash = 1469598103934665603ull;
    const auto byte = [&hash](u8 value) {
        hash = (hash ^ value) * 1099511628211ull;
    };
    for (int y = top; y < bottom; ++y) {
        for (int x = left; x < right; ++x) {
            const u32 pixel = pixels[static_cast<std::size_t>(y) * width + x];
            byte(static_cast<u8>(pixel >> 24));
            byte(static_cast<u8>(pixel >> 16));
            byte(static_cast<u8>(pixel >> 8));
            byte(static_cast<u8>(pixel));
        }
    }
    return hash;
}

void saveBinary(const std::vector<u8>& bytes, const std::string& path) {
    std::ofstream file(path, std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
    std::printf("binary saved: %s (%zu bytes)\n", path.c_str(), bytes.size());
}

bool saveWav(const std::vector<u8>& samples, const char* path, u32 sampleRate) {
    if (!path || !*path || samples.size() > 0xFFFFFFD7u) return false;
    std::ofstream file(path, std::ios::binary);
    if (!file) return false;
    const auto write16 = [&file](u16 value) {
        const char bytes[2] = {static_cast<char>(value),
                               static_cast<char>(value >> 8)};
        file.write(bytes, sizeof bytes);
    };
    const auto write32 = [&file](u32 value) {
        const char bytes[4] = {static_cast<char>(value),
                               static_cast<char>(value >> 8),
                               static_cast<char>(value >> 16),
                               static_cast<char>(value >> 24)};
        file.write(bytes, sizeof bytes);
    };
    const u32 dataBytes = static_cast<u32>(samples.size());
    const u32 padding = dataBytes & 1u;
    file.write("RIFF", 4);
    write32(36u + dataBytes + padding);
    file.write("WAVEfmt ", 8);
    write32(16);
    write16(1);                               // PCM
    write16(1);                               // mono internal speaker
    write32(sampleRate);
    write32(sampleRate);                      // one byte per sample
    write16(1);
    write16(8);
    file.write("data", 4);
    write32(dataBytes);
    file.write(reinterpret_cast<const char*>(samples.data()),
               static_cast<std::streamsize>(samples.size()));
    if (padding) file.put('\0');
    return static_cast<bool>(file);
}

u64 parseU64(const char* text) {
    return static_cast<u64>(std::strtoull(text, nullptr, 0));
}

void dumpMemory(IifxMachine& mac, u32 address, u32 size, const char* label) {
    std::printf("-- %s @ %08X --\n", label, address);
    for (u32 offset = 0; offset < size; offset += 16) {
        std::printf("  %08X ", address + offset);
        for (u32 column = 0; column < 16 && offset + column < size; ++column)
            std::printf(" %02X", mac.read8(address + offset + column));
        std::printf("\n");
    }
}

void dumpLogicalMemory(IifxMachine& mac, u32 address, u32 size,
                       const char* label) {
    std::printf("-- %s logical @ %08X --\n", label, address);
    for (u32 offset = 0; offset < size; offset += 16) {
        std::printf("  %08X ", address + offset);
        for (u32 column = 0; column < 16 && offset + column < size; ++column) {
            try {
                const u32 logical = address + offset + column;
                const u32 physical = mac.cpu().diagnosticTranslate(logical);
                std::printf(" %02X", mac.read8(physical));
            } catch (...) {
                std::printf(" ??");
            }
        }
        std::printf("\n");
    }
    try {
        std::printf("  first physical address: %08X\n",
                    mac.cpu().diagnosticTranslate(address));
    } catch (...) {
        std::printf("  first physical address: <unmapped>\n");
    }
}

bool applicationIs(IifxMachine& mac, const std::string& name) {
    if (name.empty() || name.size() > 31u ||
        mac.read8(0x02E0u) != static_cast<u8>(name.size()))
        return false;
    for (std::size_t index = 0; index < name.size(); ++index)
        if (mac.read8(0x02E1u + static_cast<u32>(index)) !=
            static_cast<u8>(name[index]))
            return false;
    return true;
}

bool parseTraceSources(const std::string& text, u32& mask) {
    mask = 0;
    std::size_t begin = 0;
    while (begin < text.size()) {
        const std::size_t comma = text.find(',', begin);
        const std::string name = text.substr(
            begin, comma == std::string::npos ? std::string::npos
                                               : comma - begin);
        HardwareTraceSource source;
        if (name == "cpu") source = HardwareTraceSource::Cpu;
        else if (name == "memory") source = HardwareTraceSource::Memory;
        else if (name == "rom") source = HardwareTraceSource::Rom;
        else if (name == "via") source = HardwareTraceSource::Via;
        else if (name == "rtc") source = HardwareTraceSource::Rtc;
        else if (name == "oss") source = HardwareTraceSource::Oss;
        else if (name == "scc-iop") source = HardwareTraceSource::SccIop;
        else if (name == "ism-iop") source = HardwareTraceSource::IsmIop;
        else if (name == "ncr5380") source = HardwareTraceSource::Ncr5380;
        else if (name == "scsi-dma") source = HardwareTraceSource::ScsiDma;
        else if (name == "swim") source = HardwareTraceSource::Swim;
        else if (name == "asc") source = HardwareTraceSource::Asc;
        else if (name == "nubus") source = HardwareTraceSource::NuBus;
        else if (name == "biu") source = HardwareTraceSource::Biu;
        else if (name == "other") source = HardwareTraceSource::Other;
        else return false;
        mask |= hardwareTraceSourceBit(source);
        if (comma == std::string::npos) break;
        begin = comma + 1;
    }
    return mask != 0;
}

bool parseTraceAddressRange(const std::string& text, u32& first, u32& last) {
    const std::size_t colon = text.find(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 == text.size())
        return false;
    const std::string firstText = text.substr(0, colon);
    const std::string lastText = text.substr(colon + 1);
    char* firstEnd = nullptr;
    char* lastEnd = nullptr;
    const unsigned long firstValue = std::strtoul(firstText.c_str(), &firstEnd, 16);
    const unsigned long lastValue = std::strtoul(lastText.c_str(), &lastEnd, 16);
    if (!firstEnd || *firstEnd != '\0' || !lastEnd || *lastEnd != '\0' ||
        firstValue > 0xFFFFFFFFul || lastValue > 0xFFFFFFFFul ||
        firstValue > lastValue)
        return false;
    first = static_cast<u32>(firstValue);
    last = static_cast<u32>(lastValue);
    return true;
}

bool parseHexPair(const std::string& text, u32& first, u32& second) {
    const std::size_t colon = text.find(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 == text.size())
        return false;
    const std::string firstText = text.substr(0, colon);
    const std::string secondText = text.substr(colon + 1);
    char* firstEnd = nullptr;
    char* secondEnd = nullptr;
    const unsigned long firstValue =
        std::strtoul(firstText.c_str(), &firstEnd, 16);
    const unsigned long secondValue =
        std::strtoul(secondText.c_str(), &secondEnd, 16);
    if (!firstEnd || *firstEnd != '\0' || !secondEnd || *secondEnd != '\0' ||
        firstValue > 0xFFFFFFFFul || secondValue > 0xFFFFFFFFul)
        return false;
    first = static_cast<u32>(firstValue);
    second = static_cast<u32>(secondValue);
    return true;
}

} // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: openmac_trace030 <IIfx.ROM> [frames] [disk.img] [ram-mb] [floppy.img] [no-video|no-frame|frame.ppm] [input-frame dx dy] [--shutdown] [--expect-finder|--expect-app name] [--stop-on-milestone] [--native-storage] [--video-rom card.bin] [--dump-iops prefix]\n"
                     "       structured trace: --hw-trace file.jsonl [--structured-only] [--stop-on-trace] [--trace-capacity n] [--trace-post n] [--trace-mask n] [--trace-sources via,oss,biu] [--trace-address-range hex:hex] [--trace-cycle n] [--trace-pc hex --trace-pc-hits n]\n"
                     "       IOP flight/trigger: --trace-iop-flight n [--trace-iop-operation hex] [--trace-iop-block n] [--trace-iop-hits n]\n"
                     "       replay: --load-state file.iifxstate [--save-state file.iifxstate] [--save-disk file.img] [--disk2 file.img [--disk2-read-only] [--save-disk2 file.img]] [--gc-fast-forward] [--load-pram file [--pram-poke hex-offset:hex-byte]] [--checkpoint-cycle n] [--stop-after-checkpoint] [--verify-replay instructions] [--trace-gc-pc hex] [--trace-gc-pc-hits n] [--profile-pc-range hex:hex] [--profile-pc-limit n] [--key-at-frame n hex-adb-code up|down] [--disasm-live hex count] [--poke16 hex:value] [--invalidate-data-cache hex:size] [--dump-memory hex:size] [--dump-logical-memory hex:size] [--dump-binary hex size file] [--dump-gc-dram hex-offset hex-size file] [--dump-gc-binary hex-offset hex-size file] [--dump-gc-sram hex-offset hex-size file]\n"
                     "       media/input timing: [--floppy-at-frame n file.img]... [--eject-floppy-at-frame n]... [--input-at-frame n dx dy up|down]... [--milestone-timeout name:frames]\n"
                     "       trigger capture: --checkpoint-on-trigger file.iifxstate [--checkpoint-at-pc hex file.iifxstate]\n"
                     "       boot display capture: --frame-timeline prefix first-frame last-frame\n"
                     "       trap probe: --watch-trap hex-opcode [--watch-trap hex-opcode]... [--watch-trap-limit n]\n"
                     "       concise output/artifacts: --quiet [--cpu-state] [--media-summary] [--dump-audio sound.wav] [--expect-startup-chime]\n"
                     "       omit floppy.img and use 'no-video' for a headless machine\n"
                     "       add [input-frame] [mouse-dx] [mouse-dy] to inject motion\n"
                     "       use a negative frame count to run that many instructions\n");
        return 2;
    }

    std::vector<u8> rom = loadFile(argv[1]);
    if (rom.empty()) {
        std::fprintf(stderr, "cannot read ROM: %s\n", argv[1]);
        return 2;
    }
    const bool disassembleOnly = argc >= 3 && std::string(argv[2]) == "--disasm";
    const int frames = disassembleOnly ? 1 : (argc >= 3 ? std::atoi(argv[2]) : 600);
    const int ramMb = !disassembleOnly && argc >= 5 ? std::atoi(argv[4]) : 8;
    const bool legalRam = ramMb == 4 || ramMb == 8 || ramMb == 16 ||
        ramMb == 20 || ramMb == 32 || ramMb == 64 || ramMb == 68 ||
        ramMb == 80 || ramMb == 128;
    if (frames == 0 || !legalRam) {
        std::fprintf(stderr,
                     "frames must be nonzero; IIfx RAM must be 4, 8, 16, 20, 32, 64, 68, 80, or 128 MB\n");
        return 2;
    }

    IifxMachine::Config config;
    config.ramSize = static_cast<u32>(ramMb) * 1024u * 1024u;
    for (int i = 2; i < argc; ++i) {
        config.nativeStorage = config.nativeStorage ||
            std::string(argv[i]) == "--native-storage";
        if (std::string(argv[i]) == "--video-rom" && i + 1 < argc) {
            config.videoDeclarationRom = loadFile(argv[++i]);
            if (config.videoDeclarationRom.empty()) {
                std::fprintf(stderr, "cannot read video declaration ROM: %s\n",
                             argv[i]);
                return 2;
            }
        }
    }
    config.videoCard = !((argc >= 6 && std::string(argv[5]) == "no-video") ||
                         (argc >= 7 && std::string(argv[6]) == "no-video"));
    const std::vector<u8> replayRom = rom;
    IifxMachine mac(std::move(rom), config);

    const char* hardwareTracePath = nullptr;
    HardwareTraceConfig hardwareTraceConfig;
    hardwareTraceConfig.capacity = 131072;
    hardwareTraceConfig.postTriggerEvents = 16384;
    bool structuredOnly = false;
    bool quiet = false;
    bool hostProfile = false;
    bool printCpuState = false;
    bool stopOnTrace = false;
    bool shutdown = false;
    std::string expectedApplication;
    bool stopOnMilestone = false;
    bool mediaSummary = false;
    bool expectStartupChime = false;
    bool stopAfterCheckpoint = false;
    const char* loadStatePath = nullptr;
    const char* loadPramPath = nullptr;
    const char* saveStatePath = nullptr;
    const char* saveDiskPath = nullptr;
    const char* disk2Path = nullptr;
    bool disk2ReadOnly = false;
    bool loopFastForward = false;
    const char* saveDisk2Path = nullptr;
    const char* audioDumpPath = nullptr;
    u64 checkpointCycle = 0;
    u64 verifyReplayInstructions = 0;
    u32 traceGcPc = 0;
    bool traceGcPcEnabled = false;
    u64 traceGcPcHits = 1;
    u32 profilePcFirst = 0;
    u32 profilePcLast = 0;
    bool profilePcRange = false;
    u32 profileGcFirst = 0;
    u32 profileGcLast = 0;
    bool profileGcRange = false;
    u32 tapGcFirst = 0;
    u32 tapGcLast = 0;
    bool tapGcRange = false;
    u32 tapGcAddrFirst = 0;
    u32 tapGcAddrLast = 0;
    bool tapGcAddrRange = false;
    bool tapGcAddrWritesOnly = false;
    u32 watchGcRegister = 0;
    bool watchGcRegisterEnabled = false;
    u32 watchGcVram = 0;
    bool watchGcVramEnabled = false;
    u32 snapGcPc = 0;
    u32 snapGcRegister = 0;
    bool snapGcEnabled = false;
    u64 flightGcStart = 0;
    u64 flightGcCount = 0;
    u32 flightGcRegister = 0;
    const char* flightGcPath = nullptr;
    u32 watchGcDoorbell = 0;
    bool watchGcDoorbellEnabled = false;
    std::size_t profilePcLimit = 64;
    std::vector<u64> profilePcCounts;
    const char* iopDumpPrefix = nullptr;
    struct FloppyEvent {
        int frame = 0;
        bool eject = false;
        std::string path;
        std::vector<u8> image;
    };
    std::vector<FloppyEvent> floppyEvents;
    struct InputEvent {
        int frame = 0;
        int dx = 0;
        int dy = 0;
        bool button = false;
    };
    std::vector<InputEvent> inputEvents;
    struct KeyEvent {
        int frame = 0;
        u8 adbCode = 0;
        bool down = false;
    };
    std::vector<KeyEvent> keyEvents;
    // Closed-loop mouse positioning: from `frame` on, nudge the mouse a few
    // pixels per frame toward (x, y) as reported by the guest's low-memory
    // Mouse global ($830: v, h) until it lands there.  Relative motion alone
    // cannot place the pointer because System 7 accelerates fast moves.
    struct MouseGoto {
        int frame = 0;
        int x = 0;
        int y = 0;
        bool active = false;
        bool done = false;
        bool button = false;
    };
    std::vector<MouseGoto> mouseGotos;
    const char* triggerCheckpointPath = nullptr;
    const char* pcCheckpointPath = nullptr;
    u32 pcCheckpointAddress = 0;
    struct LiveDisassembly { u32 address; int count; };
    std::vector<LiveDisassembly> liveDisassemblies;
    struct MemoryRange { u32 address; u32 size; bool logical; };
    std::vector<MemoryRange> memoryDumps;
    std::vector<MemoryRange> dataCacheInvalidations;
    struct BinaryMemoryDump { u32 address; u32 size; std::string path; };
    std::vector<BinaryMemoryDump> binaryMemoryDumps;
    std::vector<BinaryMemoryDump> gcDataMemoryDumps;
    std::vector<BinaryMemoryDump> gcBinaryMemoryDumps;
    std::vector<BinaryMemoryDump> gcSramBinaryMemoryDumps;
    struct MemoryPoke16 { u32 address; u16 value; };
    std::vector<MemoryPoke16> memoryPokes16;
    struct PramPoke { u8 offset; u8 value; };
    std::vector<PramPoke> pramPokes;
    std::vector<u8> pramOverride;
    struct MilestoneDeadline {
        std::string name;
        u64 frames = 0;
        bool reached = false;
    };
    std::vector<MilestoneDeadline> milestoneDeadlines;
    std::string frameTimelinePrefix;
    int frameTimelineFirst = 0;
    int frameTimelineLast = -1;
    std::vector<u16> watchedTraps;
    bool watchedTrapEnabled = false;
    u64 watchedTrapLimit = 32;
    for (int index = 2; index < argc; ++index) {
        const std::string option = argv[index];
        if (option == "--structured-only")
            structuredOnly = true;
        else if (option == "--quiet")
            quiet = true;
        else if (option == "--host-profile")
            hostProfile = true;
        else if (option == "--cpu-state")
            printCpuState = true;
        else if (option == "--stop-on-trace")
            stopOnTrace = true;
        else if (option == "--shutdown")
            shutdown = true;
        else if (option == "--expect-finder")
            expectedApplication = "Finder";
        else if (option == "--expect-app" && index + 1 < argc)
            expectedApplication = argv[++index];
        else if (option == "--stop-on-milestone")
            stopOnMilestone = true;
        else if (option == "--media-summary")
            mediaSummary = true;
        else if (option == "--expect-startup-chime")
            expectStartupChime = true;
        else if (option == "--stop-after-checkpoint")
            stopAfterCheckpoint = true;
        else if (option == "--load-state" && index + 1 < argc)
            loadStatePath = argv[++index];
        else if (option == "--load-pram" && index + 1 < argc)
            loadPramPath = argv[++index];
        else if (option == "--save-state" && index + 1 < argc)
            saveStatePath = argv[++index];
        else if (option == "--save-disk" && index + 1 < argc)
            saveDiskPath = argv[++index];
        else if (option == "--disk2" && index + 1 < argc)
            disk2Path = argv[++index];
        else if (option == "--disk2-read-only")
            disk2ReadOnly = true;
        else if (option == "--gc-fast-forward")
            loopFastForward = true;
        else if (option == "--save-disk2" && index + 1 < argc)
            saveDisk2Path = argv[++index];
        else if (option == "--dump-audio" && index + 1 < argc)
            audioDumpPath = argv[++index];
        else if (option == "--checkpoint-cycle" && index + 1 < argc)
            checkpointCycle = parseU64(argv[++index]);
        else if (option == "--verify-replay" && index + 1 < argc)
            verifyReplayInstructions = parseU64(argv[++index]);
        else if (option == "--trace-gc-pc" && index + 1 < argc) {
            char* end = nullptr;
            const unsigned long value = std::strtoul(argv[++index], &end, 16);
            if (!end || *end != '\0' || value > 0xFFFFFFFFul) {
                std::fprintf(stderr,
                             "--trace-gc-pc requires a hexadecimal address\n");
                return 2;
            }
            traceGcPc = static_cast<u32>(value);
            traceGcPcEnabled = true;
        }
        else if (option == "--trace-gc-pc-hits" && index + 1 < argc) {
            traceGcPcHits = parseU64(argv[++index]);
            if (traceGcPcHits == 0) {
                std::fprintf(stderr,
                             "--trace-gc-pc-hits requires a positive count\n");
                return 2;
            }
        }
        else if ((option == "--tap-gc-addr-range" ||
                  option == "--tap-gc-addr-writes") && index + 1 < argc) {
            if (!parseTraceAddressRange(argv[++index], tapGcAddrFirst,
                                        tapGcAddrLast) ||
                tapGcAddrLast <= tapGcAddrFirst) {
                std::fprintf(stderr,
                    "%s requires hex-first:hex-last\n", option.c_str());
                return 2;
            }
            tapGcAddrRange = true;
            tapGcAddrWritesOnly = option == "--tap-gc-addr-writes";
        }
        else if (option == "--watch-gc-doorbell" && index + 1 < argc) {
            char* end = nullptr;
            const unsigned long value = std::strtoul(argv[++index], &end, 16);
            if (!end || *end != '\0' || value > 0xFFFFFFFFul) {
                std::fprintf(stderr,
                    "--watch-gc-doorbell requires a hexadecimal address\n");
                return 2;
            }
            watchGcDoorbell = static_cast<u32>(value);
            watchGcDoorbellEnabled = true;
        }
        else if (option == "--watch-gc-vram" && index + 1 < argc) {
            char* end = nullptr;
            const unsigned long value = std::strtoul(argv[++index], &end, 16);
            if (!end || *end != 0 || value > 0xFFFFFFFFul) {
                std::fprintf(stderr,
                    "--watch-gc-vram requires a hexadecimal VRAM offset\n");
                return 2;
            }
            watchGcVram = static_cast<u32>(value);
            watchGcVramEnabled = true;
        }
        else if (option == "--watch-gc-register" && index + 1 < argc) {
            char* end = nullptr;
            const unsigned long value = std::strtoul(argv[++index], &end, 16);
            if (!end || *end != '\0' || value > 0xFFul) {
                std::fprintf(stderr,
                    "--watch-gc-register requires a hex physical register "
                    "0-FF\n");
                return 2;
            }
            watchGcRegister = static_cast<u32>(value);
            watchGcRegisterEnabled = true;
        }
        else if (option == "--snap-gc-pc" && index + 1 < argc) {
            if (!parseHexPair(argv[++index], snapGcPc, snapGcRegister) ||
                snapGcPc == 0 || snapGcRegister > 0xFFu) {
                std::fprintf(stderr,
                    "--snap-gc-pc requires hex-pc:hex-architectural-register "
                    "(0-7F gr, 80-FF lr)\n");
                return 2;
            }
            snapGcEnabled = true;
        }
        else if (option == "--flight-gc" && index + 4 < argc) {
            flightGcStart = parseU64(argv[++index]);
            flightGcCount = parseU64(argv[++index]);
            char* end = nullptr;
            const unsigned long reg = std::strtoul(argv[++index], &end, 16);
            flightGcPath = argv[++index];
            if (!end || *end != '\0' || reg > 0xFFul || flightGcCount == 0 ||
                flightGcCount > (1ul << 20)) {
                std::fprintf(stderr,
                    "--flight-gc requires start-instr count hex-physical-"
                    "register file (count <= 1M)\n");
                return 2;
            }
            flightGcRegister = static_cast<u32>(reg);
        }
        else if (option == "--tap-gc-pc-range" && index + 1 < argc) {
            if (!parseTraceAddressRange(argv[++index], tapGcFirst,
                                        tapGcLast) || tapGcLast <= tapGcFirst) {
                std::fprintf(stderr,
                    "--tap-gc-pc-range requires hex-first:hex-last\n");
                return 2;
            }
            tapGcRange = true;
        }
        else if (option == "--profile-gc-range" && index + 1 < argc) {
            if (!parseTraceAddressRange(argv[++index], profileGcFirst,
                                        profileGcLast) ||
                profileGcLast <= profileGcFirst ||
                profileGcLast - profileGcFirst > 0x00400000u) {
                std::fprintf(stderr,
                    "--profile-gc-range requires hex-first:hex-last spanning at most 4 MiB\n");
                return 2;
            }
            profileGcRange = true;
        }
        else if (option == "--profile-pc-range" && index + 1 < argc) {
            if (!parseTraceAddressRange(argv[++index], profilePcFirst,
                                        profilePcLast) ||
                profilePcLast - profilePcFirst > 0x100000u) {
                std::fprintf(stderr,
                    "--profile-pc-range requires hex-first:hex-last spanning at most 1 MiB\n");
                return 2;
            }
            profilePcRange = true;
            profilePcCounts.assign(
                (static_cast<std::size_t>(profilePcLast - profilePcFirst) >> 1u) + 1u,
                0);
        }
        else if (option == "--profile-pc-limit" && index + 1 < argc) {
            profilePcLimit = static_cast<std::size_t>(parseU64(argv[++index]));
            if (profilePcLimit == 0 || profilePcLimit > 4096u) {
                std::fprintf(stderr,
                    "--profile-pc-limit requires a value from 1 through 4096\n");
                return 2;
            }
        }
        else if (option == "--dump-iops" && index + 1 < argc)
            iopDumpPrefix = argv[++index];
        else if (option == "--floppy-at-frame" && index + 2 < argc) {
            FloppyEvent event;
            event.frame = std::atoi(argv[++index]);
            event.path = argv[++index];
            floppyEvents.push_back(std::move(event));
        }
        else if (option == "--eject-floppy-at-frame" && index + 1 < argc) {
            FloppyEvent event;
            event.frame = std::atoi(argv[++index]);
            event.eject = true;
            floppyEvents.push_back(std::move(event));
        }
        else if (option == "--input-at-frame" && index + 4 < argc) {
            InputEvent event;
            event.frame = std::atoi(argv[++index]);
            event.dx = std::atoi(argv[++index]);
            event.dy = std::atoi(argv[++index]);
            const std::string button = argv[++index];
            if (event.frame < 0 || (button != "up" && button != "down")) {
                std::fprintf(stderr,
                    "--input-at-frame requires non-negative n dx dy up|down\n");
                return 2;
            }
            event.button = button == "down";
            inputEvents.push_back(event);
        }
        else if ((option == "--mouse-goto-at-frame" ||
                  option == "--mouse-drag-to-at-frame") && index + 3 < argc) {
            MouseGoto target;
            target.frame = std::atoi(argv[++index]);
            target.x = std::atoi(argv[++index]);
            target.y = std::atoi(argv[++index]);
            target.button = option == "--mouse-drag-to-at-frame";
            if (target.frame < 0 || target.x < 0 || target.y < 0) {
                std::fprintf(stderr,
                    "%s requires non-negative frame x y\n", option.c_str());
                return 2;
            }
            mouseGotos.push_back(target);
        }
        else if (option == "--key-at-frame" && index + 3 < argc) {
            KeyEvent event;
            event.frame = std::atoi(argv[++index]);
            char* end = nullptr;
            const unsigned long code = std::strtoul(argv[++index], &end, 16);
            const std::string state = argv[++index];
            if (event.frame < 0 || !end || *end != '\0' || code > 0x7Ful ||
                (state != "up" && state != "down")) {
                std::fprintf(stderr,
                    "--key-at-frame requires non-negative n hex-adb-code up|down\n");
                return 2;
            }
            event.adbCode = static_cast<u8>(code);
            event.down = state == "down";
            keyEvents.push_back(event);
        }
        else if (option == "--checkpoint-on-trigger" && index + 1 < argc)
            triggerCheckpointPath = argv[++index];
        else if (option == "--checkpoint-at-pc" && index + 2 < argc) {
            char* end = nullptr;
            const unsigned long value = std::strtoul(argv[++index], &end, 16);
            if (!end || *end != '\0' || value > 0xFFFFFFFFul) {
                std::fprintf(stderr,
                    "--checkpoint-at-pc requires a hexadecimal address and file\n");
                return 2;
            }
            pcCheckpointAddress = static_cast<u32>(value);
            pcCheckpointPath = argv[++index];
        }
        else if (option == "--disasm-live" && index + 2 < argc) {
            const u32 address = static_cast<u32>(
                std::strtoul(argv[++index], nullptr, 16));
            const int count = std::atoi(argv[++index]);
            if (count <= 0) {
                std::fprintf(stderr, "--disasm-live count must be positive\n");
                return 2;
            }
            liveDisassemblies.push_back({address, count});
        }
        else if ((option == "--dump-memory" ||
                  option == "--dump-logical-memory") && index + 1 < argc) {
            u32 address = 0;
            u32 size = 0;
            if (!parseHexPair(argv[++index], address, size) || size == 0) {
                std::fprintf(stderr,
                    "%s requires hexadecimal address:positive-size\n",
                    option.c_str());
                return 2;
            }
            memoryDumps.push_back(
                {address, size, option == "--dump-logical-memory"});
        }
        else if ((option == "--dump-binary" ||
                  option == "--dump-gc-dram" ||
                  option == "--dump-gc-binary" ||
                  option == "--dump-gc-sram") && index + 3 < argc) {
            char* addressEnd = nullptr;
            char* sizeEnd = nullptr;
            const unsigned long addressValue =
                std::strtoul(argv[++index], &addressEnd, 16);
            const unsigned long sizeValue =
                std::strtoul(argv[++index], &sizeEnd, 16);
            const std::string path = argv[++index];
            if (!addressEnd || *addressEnd != '\0' ||
                !sizeEnd || *sizeEnd != '\0' || sizeValue == 0 ||
                addressValue > 0xFFFFFFFFul || sizeValue > 0xFFFFFFFFul) {
                std::fprintf(stderr,
                    "%s requires hexadecimal address size file\n",
                    option.c_str());
                return 2;
            }
            auto& dumps = option == "--dump-gc-dram"
                ? gcDataMemoryDumps
                : option == "--dump-gc-binary"
                    ? gcBinaryMemoryDumps
                    : option == "--dump-gc-sram"
                        ? gcSramBinaryMemoryDumps : binaryMemoryDumps;
            dumps.push_back({static_cast<u32>(addressValue),
                             static_cast<u32>(sizeValue), path});
        }
        else if (option == "--poke16" && index + 1 < argc) {
            u32 address = 0;
            u32 value = 0;
            if (!parseHexPair(argv[++index], address, value) ||
                (address & 1u) != 0 || value > 0xFFFFu) {
                std::fprintf(stderr,
                    "--poke16 requires an even hexadecimal address:word\n");
                return 2;
            }
            memoryPokes16.push_back({address, static_cast<u16>(value)});
        }
        else if (option == "--pram-poke" && index + 1 < argc) {
            u32 offset = 0;
            u32 value = 0;
            if (!parseHexPair(argv[++index], offset, value) ||
                offset > 0xFFu || value > 0xFFu) {
                std::fprintf(stderr,
                    "--pram-poke requires hex-byte-offset:hex-byte\n");
                return 2;
            }
            pramPokes.push_back(
                {static_cast<u8>(offset), static_cast<u8>(value)});
        }
        else if (option == "--invalidate-data-cache" && index + 1 < argc) {
            u32 address = 0;
            u32 size = 0;
            if (!parseHexPair(argv[++index], address, size) || size == 0) {
                std::fprintf(stderr,
                    "--invalidate-data-cache requires hexadecimal "
                    "address:positive-size\n");
                return 2;
            }
            dataCacheInvalidations.push_back({address, size, true});
        }
        else if (option == "--milestone-timeout" && index + 1 < argc) {
            const std::string value = argv[++index];
            const std::size_t colon = value.rfind(':');
            if (colon == std::string::npos || colon == 0 ||
                colon + 1 == value.size()) {
                std::fprintf(stderr,
                    "--milestone-timeout requires name:frames\n");
                return 2;
            }
            const u64 deadline = parseU64(value.c_str() + colon + 1);
            if (deadline == 0) {
                std::fprintf(stderr,
                    "milestone timeout frames must be nonzero\n");
                return 2;
            }
            milestoneDeadlines.push_back(
                {value.substr(0, colon), deadline, false});
        }
        else if (option == "--frame-timeline" && index + 3 < argc) {
            frameTimelinePrefix = argv[++index];
            frameTimelineFirst = std::atoi(argv[++index]);
            frameTimelineLast = std::atoi(argv[++index]);
        }
        else if (option == "--watch-trap" && index + 1 < argc) {
            const u64 value = parseU64(argv[++index]);
            if (value > 0xFFFFu) {
                std::fprintf(stderr,
                             "--watch-trap requires a 16-bit opcode\n");
                return 2;
            }
            watchedTraps.push_back(static_cast<u16>(value));
            watchedTrapEnabled = true;
        }
        else if (option == "--watch-trap-limit" && index + 1 < argc) {
            watchedTrapLimit = parseU64(argv[++index]);
            if (watchedTrapLimit == 0) {
                std::fprintf(stderr,
                             "--watch-trap-limit requires a positive count\n");
                return 2;
            }
        }
        else if (option == "--hw-trace" && index + 1 < argc)
            hardwareTracePath = argv[++index];
        else if (option == "--trace-capacity" && index + 1 < argc)
            hardwareTraceConfig.capacity = static_cast<std::size_t>(
                parseU64(argv[++index]));
        else if (option == "--trace-post" && index + 1 < argc)
            hardwareTraceConfig.postTriggerEvents = static_cast<std::size_t>(
                parseU64(argv[++index]));
        else if (option == "--trace-mask" && index + 1 < argc)
            hardwareTraceConfig.categories = static_cast<u32>(
                parseU64(argv[++index]));
        else if (option == "--trace-sources" && index + 1 < argc) {
            if (!parseTraceSources(argv[++index], hardwareTraceConfig.sources)) {
                std::fprintf(stderr, "invalid --trace-sources list\n");
                return 2;
            }
        }
        else if (option == "--trace-address-range" && index + 1 < argc) {
            if (!parseTraceAddressRange(argv[++index],
                                        hardwareTraceConfig.addressFirst,
                                        hardwareTraceConfig.addressLast)) {
                std::fprintf(stderr,
                    "--trace-address-range requires inclusive hex first:last\n");
                return 2;
            }
        }
        else if (option == "--trace-cycle" && index + 1 < argc)
            hardwareTraceConfig.triggerCycle = parseU64(argv[++index]);
        else if (option == "--trace-pc" && index + 1 < argc)
            hardwareTraceConfig.triggerPc = static_cast<u32>(
                std::strtoul(argv[++index], nullptr, 16));
        else if (option == "--trace-pc-hits" && index + 1 < argc)
            hardwareTraceConfig.triggerPcHits = parseU64(argv[++index]);
        else if (option == "--trace-iop-flight" && index + 1 < argc)
            hardwareTraceConfig.iopFlightEvents = static_cast<std::size_t>(
                parseU64(argv[++index]));
        else if (option == "--trace-iop-operation" && index + 1 < argc)
            hardwareTraceConfig.triggerIopOperation = static_cast<u32>(
                std::strtoul(argv[++index], nullptr, 16));
        else if (option == "--trace-iop-block" && index + 1 < argc)
            hardwareTraceConfig.triggerIopBlock = static_cast<u32>(
                parseU64(argv[++index]));
        else if (option == "--trace-iop-hits" && index + 1 < argc)
            hardwareTraceConfig.triggerIopHits = parseU64(argv[++index]);
    }
    if ((checkpointCycle != 0 || stopAfterCheckpoint) && !saveStatePath) {
        std::fprintf(stderr,
                     "--checkpoint-cycle/--stop-after-checkpoint requires --save-state\n");
        return 2;
    }
    if (!frameTimelinePrefix.empty() &&
        (frames < 0 || frameTimelineFirst < 1 ||
         frameTimelineLast < frameTimelineFirst ||
         frameTimelineLast - frameTimelineFirst + 1 > 10000)) {
        std::fprintf(stderr,
                     "--frame-timeline requires a positive frame run and a "
                     "bounded 1-based range of at most 10000 frames\n");
        return 2;
    }
    for (const FloppyEvent& event : floppyEvents) {
        if (event.frame < 0 || (!event.eject && event.path.empty())) {
            std::fprintf(stderr,
                "scheduled floppy events require a non-negative frame%s\n",
                event.eject ? "" : " and image");
            return 2;
        }
    }
    if (hardwareTracePath) {
        if (hardwareTraceConfig.capacity < 2) {
            std::fprintf(stderr, "trace capacity must be at least 2\n");
            return 2;
        }
        if (hardwareTraceConfig.triggerPc != 0 &&
            hardwareTraceConfig.triggerPcHits == 0)
            hardwareTraceConfig.triggerPcHits = 1;
        mac.configureHardwareTrace(hardwareTraceConfig);
        mac.setTraceCheckpointOnTrigger(triggerCheckpointPath != nullptr);
        std::printf("structured trace armed: %s capacity=%zu post=%zu sources=%08X address=%08X:%08X\n",
                    hardwareTracePath, hardwareTraceConfig.capacity,
                    hardwareTraceConfig.postTriggerEvents,
                    hardwareTraceConfig.sources,
                    hardwareTraceConfig.addressFirst,
                    hardwareTraceConfig.addressLast);
    }
    if (triggerCheckpointPath && !hardwareTracePath) {
        std::fprintf(stderr,
                     "--checkpoint-on-trigger requires --hw-trace\n");
        return 2;
    }
    if (structuredOnly && hardwareTracePath) stopOnTrace = true;

    if (!pramPokes.empty() && !loadPramPath) {
        std::fprintf(stderr, "--pram-poke requires --load-pram\n");
        return 2;
    }
    if (loadPramPath) {
        pramOverride = loadFile(loadPramPath);
        if (pramOverride.size() != mac.savePram(nullptr, 0)) {
            std::fprintf(stderr, "invalid PRAM blob: %s (%zu bytes)\n",
                         loadPramPath, pramOverride.size());
            return 2;
        }
        for (const PramPoke& poke : pramPokes)
            pramOverride[8u + poke.offset] = poke.value;
    }
    const auto applyPramOverride = [&]() {
        if (!loadPramPath) return true;
        if (!mac.loadPram(pramOverride.data(),
                          static_cast<u32>(pramOverride.size()))) {
            std::fprintf(stderr, "PRAM load rejected: %s\n", loadPramPath);
            return false;
        }
        std::printf("PRAM loaded: %s", loadPramPath);
        for (const PramPoke& poke : pramPokes)
            std::printf(" [%02X=%02X]", poke.offset, poke.value);
        std::printf("\n");
        return true;
    };
    // A checkpoint contains the complete RTC/XPRAM state. Defer an explicit
    // command-line override until after that checkpoint is restored so the
    // user's later, more specific option cannot be silently discarded.
    if (!loadStatePath && !applyPramOverride()) return 2;

    if (disassembleOnly) {
        if (argc < 4) {
            std::fprintf(stderr, "--disasm requires a hexadecimal address [count]\n");
            return 2;
        }
        u32 pc = static_cast<u32>(std::strtoul(argv[3], nullptr, 16));
        const int count = argc >= 5 ? std::atoi(argv[4]) : 32;
        for (int i = 0; i < count; ++i) {
            std::string instruction;
            const int bytes = dbg::disasm(
                [&](u32 address) { return mac.read16(address); }, pc, instruction);
            std::printf("%08X  %s\n", pc, instruction.c_str());
            pc += static_cast<u32>(bytes);
        }
        return 0;
    }

    if (argc >= 4) {
        std::vector<u8> disk;
        const std::string diskArg = argv[3];
        if (diskArg == "none") {
            // Explicitly no SCSI target, useful for isolating a floppy boot.
        } else if (diskArg.rfind("zero:", 0) == 0) {
            const int mb = std::atoi(diskArg.c_str() + 5);
            if (mb < 2 || mb > 512) {
                std::fprintf(stderr, "zero disk size must be 2..512 MB\n");
                return 2;
            }
            disk.assign(static_cast<std::size_t>(mb) * 1024u * 1024u, 0);
        } else if (diskArg.rfind("blank:", 0) == 0) {
            const int mb = std::atoi(diskArg.c_str() + 6);
            if (mb < 2 || mb > 512) {
                std::fprintf(stderr, "blank disk size must be 2..512 MB\n");
                return 2;
            }
            disk = hfs::formatVolume(static_cast<u32>(mb) * 1024u * 1024u,
                                     "IIfx HD");
        } else {
            disk = loadFile(argv[3]);
        }
        if (diskArg != "none" && disk.empty()) {
            std::fprintf(stderr, "cannot read disk: %s\n", argv[3]);
            return 2;
        }
        if (!disk.empty()) mac.insertHardDisk(std::move(disk));
    }
    if (loopFastForward) mac.setGcLoopFastForward(true);
    if (disk2Path) {
        std::vector<u8> disk2 = loadFile(disk2Path);
        if (disk2.empty()) {
            std::fprintf(stderr, "cannot read second disk: %s\n", disk2Path);
            return 2;
        }
        mac.insertHardDisk2(std::move(disk2), disk2ReadOnly);
    }

    if (argc >= 6 && std::string(argv[5]) != "no-video" &&
        std::string(argv[5]) != "-" && std::string(argv[5]) != "none") {
        std::vector<u8> floppy = loadFile(argv[5]);
        if (floppy.empty() || !mac.insertFloppy(std::move(floppy))) {
            std::fprintf(stderr, "cannot insert floppy: %s\n", argv[5]);
            return 2;
        }
        std::printf("floppy inserted: %s\n", argv[5]);
    }

    for (FloppyEvent& event : floppyEvents) {
        if (event.eject) {
            std::printf("floppy eject staged at frame %d\n", event.frame);
            continue;
        }
        event.image = loadFile(event.path.c_str());
        if (event.image.empty()) {
            std::fprintf(stderr, "cannot read deferred floppy: %s\n",
                         event.path.c_str());
            return 2;
        }
        std::printf("floppy staged: %s at frame %d\n", event.path.c_str(),
                    event.frame);
    }

    if (loadStatePath) {
        std::string error;
        if (!mac.loadStateFile(loadStatePath, &error)) {
            std::fprintf(stderr, "checkpoint load failed: %s\n", error.c_str());
            return 2;
        }
        std::printf("checkpoint loaded: %s cycle=%llu frame=%llu pc=%08X\n",
                    loadStatePath,
                    static_cast<unsigned long long>(mac.totalCycles()),
                    static_cast<unsigned long long>(mac.frameCount()),
                    mac.cpu().pc);
        std::printf("checkpoint media: floppy=%d bytes=%zu event=%llu\n",
                    mac.floppyPresent() ? 1 : 0, mac.floppyImage().size(),
                    static_cast<unsigned long long>(
                        mac.diagnosticMediaEventId()));
    }
    if (loadStatePath && !applyPramOverride()) return 2;
    if (traceGcPcEnabled) mac.diagnosticSetGcProcessorPcWatch(traceGcPc);
    if (profileGcRange)
        mac.diagnosticSetGcProcessorProfileRange(profileGcFirst,
                                                 profileGcLast);
    if (tapGcRange) mac.diagnosticSetGcPcTapRange(tapGcFirst, tapGcLast);
    if (tapGcAddrRange)
        mac.diagnosticSetGcAddrTapRange(tapGcAddrFirst, tapGcAddrLast,
                                        tapGcAddrWritesOnly);
    if (watchGcDoorbellEnabled)
        mac.diagnosticSetGcDoorbellWatch(watchGcDoorbell);
    if (watchGcRegisterEnabled)
        mac.diagnosticSetGcRegisterWatch(static_cast<u16>(watchGcRegister));
    if (watchGcVramEnabled) mac.diagnosticSetGcVramWatch(watchGcVram);
    if (snapGcEnabled)
        mac.diagnosticSetGcPcSnap(snapGcPc,
                                  static_cast<u16>(snapGcRegister));
    if (flightGcPath)
        mac.diagnosticSetGcFlightWindow(flightGcStart, flightGcCount,
                                        static_cast<u16>(flightGcRegister));
    // A replay begins a new diagnostic capture. Old checkpoints predate
    // correlation IDs, so start its event namespace cleanly only when a new
    // physical insertion is staged; otherwise the restored medium remains the
    // event whose subsequent protocol traffic is being replayed.
    if (loadStatePath && !floppyEvents.empty())
        mac.resetMediaCorrelation();
    for (const MemoryPoke16& poke : memoryPokes16) {
        mac.write16(poke.address, poke.value);
        std::printf("memory poke16: %08X=%04X\n", poke.address, poke.value);
    }
    for (const MemoryRange& range : dataCacheInvalidations) {
        mac.cpu().invalidateDataCache030(range.address, range.size);
        std::printf("data-cache invalidated: %08X:%X\n",
                    range.address, range.size);
    }
    // Milestone deadlines are durations from this run/replay origin, not
    // absolute guest frame numbers stored in a checkpoint.
    const u64 milestoneOriginFrame = mac.frameCount();

    if (verifyReplayInstructions != 0) {
        const std::vector<u8> origin = mac.saveState();
        IifxMachine replay(replayRom, config);
        if (hardwareTracePath) replay.configureHardwareTrace(hardwareTraceConfig);
        std::string error;
        if (!replay.loadState(origin.data(), origin.size(), &error)) {
            std::fprintf(stderr, "replay branch load failed: %s\n", error.c_str());
            return 2;
        }
        u64 completed = 0;
        for (; completed < verifyReplayInstructions; ++completed) {
            mac.stepInstruction();
            replay.stepInstruction();
            if (mac.deterministicStateHash() != replay.deterministicStateHash()) {
                std::fprintf(stderr,
                    "replay diverged after %llu instructions at pc=%08X/%08X\n",
                    static_cast<unsigned long long>(completed + 1),
                    mac.cpu().pc, replay.cpu().pc);
                return 4;
            }
            if (mac.cpu().halted || replay.cpu().halted ||
                mac.poweredOff() || replay.poweredOff()) {
                ++completed;
                break;
            }
        }
        if (mac.saveState() != replay.saveState()) {
            std::fprintf(stderr,
                         "replay full-state mismatch after %llu instructions\n",
                         static_cast<unsigned long long>(completed));
            return 4;
        }
        if (hardwareTracePath) {
            std::ostringstream left, right;
            mac.hardwareTrace().writeJsonl(left);
            replay.hardwareTrace().writeJsonl(right);
            if (left.str() != right.str()) {
                std::fprintf(stderr,
                             "replay structured-trace mismatch after %llu instructions\n",
                             static_cast<unsigned long long>(completed));
                return 4;
            }
        }
        if (!mac.loadState(origin.data(), origin.size(), &error)) {
            std::fprintf(stderr, "replay origin restore failed: %s\n", error.c_str());
            return 2;
        }
        std::printf("deterministic replay verified: %llu instructions\n",
                    static_cast<unsigned long long>(completed));
    }

    int exceptionBudget = 48;
    int trapBudget = 12;
    u64 watchedTrapHits = 0;
    mac.cpu().onException = [&](int vector, u32 pc) {
        if (vector == 10 && watchedTrapEnabled) {
            u16 opcode = 0;
            try {
                // onException reports the bus address of the trapping
                // instruction, matching the existing disassembly hook below.
                opcode = mac.read16(pc);
            } catch (...) {
                // The CPU will take the architected fetch fault. A diagnostic
                // watch must not replace it with a host-side failure.
            }
            if (std::find(watchedTraps.begin(), watchedTraps.end(), opcode) !=
                watchedTraps.end()) {
                ++watchedTrapHits;
                if (watchedTrapHits <= watchedTrapLimit) {
                    std::printf("WATCH-TRAP hit=%llu frame=%llu cycle=%llu "
                                "op=%04X pc=%08X sp=%08X d0=%08X a0=%08X",
                                static_cast<unsigned long long>(watchedTrapHits),
                                static_cast<unsigned long long>(mac.frameCount()),
                                static_cast<unsigned long long>(mac.totalCycles()),
                                opcode, pc, mac.cpu().a[7], mac.cpu().d[0],
                                mac.cpu().a[0]);
                    // OS traps carry a parameter block in A0: show ioRefNum,
                    // csCode and the first csParam words (Device Manager
                    // layout) so a Control/Status conversation reads at a
                    // glance. An unreadable block prints nothing extra.
                    if ((opcode & 0x0800u) == 0) {
                        try {
                            std::printf(" pb+18:");
                            for (u32 offset = 0x18u; offset < 0x28u; offset += 2u)
                                std::printf(" %04X",
                                            mac.read16(mac.cpu().a[0] + offset));
                        } catch (...) {
                        }
                    }
                    std::printf("\n");
                }
            }
        }
        if (structuredOnly || quiet) return;
        if (vector == 10) {
            if (trapBudget-- <= 0) return;
        } else if (exceptionBudget-- <= 0) {
            return;
        }
        std::string instruction;
        try {
            dbg::disasm([&](u32 address) { return mac.read16(address); }, pc,
                        instruction);
        } catch (const BusFault&) {
            instruction = "<unreadable>";
        }
        std::printf("%12llu EXCEPTION vector=%d pc=%08X fault=%08X sp=%08X "
                    "a0=%08X a1=%08X a2=%08X d0=%08X  %s\n",
                    static_cast<unsigned long long>(mac.totalCycles()), vector, pc,
                    mac.cpu().lastFaultAddr, mac.cpu().a[7], mac.cpu().a[0],
                    mac.cpu().a[1], mac.cpu().a[2], mac.cpu().d[0],
                    instruction.c_str());
        if (pc == 0x40809948u) {
            std::printf("  pre-fault PCs:");
            for (int back = 15; back >= 0; --back)
                std::printf(" %08X", mac.cpu().recentPc(back));
            std::printf("\n");
        }
    };
    mac.cpu().onInterrupt = [&](int level, u32 vector, u32 pc) {
        if (structuredOnly || quiet) return;
        if (exceptionBudget-- <= 0) return;
        std::printf("%12llu INTERRUPT level=%d vector=%u pc=%08X\n",
                    static_cast<unsigned long long>(mac.totalCycles()), level, vector, pc);
    };
    int deviceBudget = 48;
    int iopDeviceBudget = 4096;
    int nubusBudget = 400000;
    int gcRegisterBudget = 400000;
    int lowMemoryBudget = 0;
    int scsiBudget = 120;
    int rtcBudget = 400000;
    mac.onDiag = [&](const char* message) {
        if (structuredOnly || quiet) return;
        const bool iop = std::string(message).find("IOP ") != std::string::npos;
        const bool nubus = std::string(message).find("NuBus9") != std::string::npos;
        const bool gcRegister =
            std::string(message).find("NuBus9 GC") != std::string::npos;
        const bool lowMemory = std::string(message).find("LOWMEM") != std::string::npos;
        const bool scsi = std::string(message).find("SCSI1") != std::string::npos;
        const bool rtc = std::string(message).find("RTC ") != std::string::npos;
        if ((gcRegister && gcRegisterBudget-- > 0) ||
            (nubus && !gcRegister && nubusBudget-- > 0) ||
            (lowMemory && lowMemoryBudget-- > 0) ||
            (scsi && scsiBudget-- > 0) ||
            (rtc && rtcBudget-- > 0) ||
            (iop && iopDeviceBudget-- > 0) ||
            (!iop && !nubus && !lowMemory && !scsi && !rtc && deviceBudget-- > 0))
            std::printf("DEVICE %s\n", message);
    };
    int adbTraceBudget = 96;
    int queueTraceBudget = 0;
    int scsiFlowBudget = 192;
    int osDefaultTrace = 0;
    int sonyEventBudget = 16;
    int startupDriveBudget = 48;
    int generatedDriverBudget = 48;
    int generatedPrimeBudget = 32;
    int gcSenseBudget = 12;
    int videoDriverBudget = 128;
    bool pcCheckpointSaved = false;
    mac.onStep = [&](u32 pc) {
        if (pcCheckpointPath && !pcCheckpointSaved &&
            pc == pcCheckpointAddress) {
            std::string error;
            if (!mac.saveStateFile(pcCheckpointPath, &error)) {
                std::fprintf(stderr, "PC checkpoint save failed: %s\n",
                             error.c_str());
            } else {
                pcCheckpointSaved = true;
                std::printf("PC checkpoint saved: %s cycle=%llu frame=%llu pc=%08X\n",
                            pcCheckpointPath,
                            static_cast<unsigned long long>(mac.totalCycles()),
                            static_cast<unsigned long long>(mac.frameCount()), pc);
                // The register file at the trigger lets a follow-up run dump
                // frame-relative locals without guessing A6/A7.
                std::printf("PC checkpoint registers:");
                for (int index = 0; index < 8; ++index)
                    std::printf(" D%d=%08X", index, mac.cpu().d[index]);
                for (int index = 0; index < 8; ++index)
                    std::printf(" A%d=%08X", index, mac.cpu().a[index]);
                std::printf("\n");
            }
        }
        if (profilePcRange && pc >= profilePcFirst && pc <= profilePcLast &&
            ((pc - profilePcFirst) & 1u) == 0)
            ++profilePcCounts[(pc - profilePcFirst) >> 1u];
        if (structuredOnly || quiet) return;
        try {
            const u32 physical = mac.cpu().diagnosticTranslate(pc);
            if (mac.read16(physical) == 0x4E70u) {
                std::printf("%12llu CPU-RESET pc=%08X physical=%08X sr=%04X\n",
                            static_cast<unsigned long long>(mac.totalCycles()),
                            pc, physical, mac.cpu().getSR());
            }
        } catch (...) {
            // The instruction fetch itself will generate the architected
            // access fault; diagnostic tracing must never escape into host
            // code first.
        }
        if (gcSenseBudget > 0 &&
            (pc == 0x00004B1Cu || pc == 0x00004B22u ||
             pc == 0x00004B2Cu || pc == 0x00004B32u ||
             pc == 0x00004B3Cu || pc == 0x00004B42u)) {
            --gcSenseBudget;
            const u32 logical = mac.cpu().a[4] +
                static_cast<s16>(mac.cpu().d[2]) + 1u;
            u32 physical = 0xFFFFFFFFu;
            try { physical = mac.cpu().diagnosticTranslate(logical); }
            catch (...) {}
            std::printf("%12llu GC-SENSE pc=%08X D0=%08X D1=%08X "
                        "D2=%08X D4=%08X A4=%08X ea=%08X/%08X sr=%04X\n",
                        static_cast<unsigned long long>(mac.totalCycles()), pc,
                        mac.cpu().d[0], mac.cpu().d[1], mac.cpu().d[2],
                        mac.cpu().d[4], mac.cpu().a[4], logical, physical,
                        mac.cpu().getSR());
        }
        // The fallback video driver is copied into the low system heap.  Its
        // exact address is stable for the stock IIfx ROM/System 7 startup,
        // but keep this deliberately bounded and diagnostic-only.  Recording
        // both logical and translated card destinations makes an MMU mapping
        // failure distinguishable from a driver calling-convention failure.
        if (videoDriverBudget > 0 && pc >= 0x00005000u && pc < 0x00005220u) {
            --videoDriverBudget;
            u32 a2Physical = 0xFFFFFFFFu;
            u32 a3Physical = 0xFFFFFFFFu;
            u32 fbPhysical = 0xFFFFFFFFu;
            u32 crtcPhysical = 0xFFFFFFFFu;
            try { a2Physical = mac.cpu().diagnosticTranslate(mac.cpu().a[2], true); }
            catch (...) {}
            try { a3Physical = mac.cpu().diagnosticTranslate(mac.cpu().a[3], true); }
            catch (...) {}
            try { fbPhysical = mac.cpu().diagnosticTranslate(0xF9000000u, true); }
            catch (...) {}
            try { crtcPhysical = mac.cpu().diagnosticTranslate(0xF920013Cu, true); }
            catch (...) {}
            u16 selector = 0xFFFFu;
            u32 csParam = 0xFFFFFFFFu;
            try {
                selector = mac.read16(mac.cpu().a[0] + 0x1Au);
                csParam = mac.read32(mac.cpu().a[0] + 0x1Cu);
            } catch (...) {}
            std::printf("%12llu VIDEO-DRIVER pc=%08X D0=%08X A0=%08X "
                        "A1=%08X A2=%08X/%08X A3=%08X/%08X "
                        "sel=%04X param=%08X map=%08X/%08X\n",
                        static_cast<unsigned long long>(mac.totalCycles()), pc,
                        mac.cpu().d[0], mac.cpu().a[0], mac.cpu().a[1],
                        mac.cpu().a[2], a2Physical, mac.cpu().a[3], a3Physical,
                        selector, csParam, fbPhysical, crtcPhysical);
        }
        if (generatedPrimeBudget > 0 &&
            (pc == 0x00006206u || pc == 0x00006274u ||
             pc == 0x00006286u)) {
            --generatedPrimeBudget;
            const u32 pb = mac.cpu().a[0];
            const u32 dce = mac.cpu().a[1];
            std::printf("%12llu SCSI-PRIME pc=%08X D0=%08X PB=%08X DCE=%08X "
                        "trap=%04X buf=%08X req=%u act=%u mode=%04X pos=%u "
                        "storage=%u\n",
                        static_cast<unsigned long long>(mac.totalCycles()), pc,
                        mac.cpu().d[0], pb, dce, mac.read16(pb + 6),
                        mac.read32(pb + 0x20), mac.read32(pb + 0x24),
                        mac.read32(pb + 0x28), mac.read16(pb + 0x2C),
                        mac.read32(pb + 0x2E), mac.read32(dce + 0x14));
            if (pc == 0x00006274u) {
                const u32 buffer = mac.read32(pb + 0x20);
                std::printf("  SCSI-PRIME buffer:");
                for (u32 i = 0; i < 16; ++i)
                    std::printf(" %02X", mac.read8(buffer + i));
                std::printf("\n");
            }
        }
        if (generatedDriverBudget > 0 && pc >= 0x00006180u &&
            pc < 0x00006380u) {
            --generatedDriverBudget;
            std::string instruction;
            try {
                dbg::disasm([&](u32 address) { return mac.read16(address); }, pc,
                            instruction);
            } catch (...) {
                instruction = "<unreadable>";
            }
            std::printf("%12llu SCSI-DRIVER pc=%08X D0=%08X D3=%08X "
                        "A0=%08X A1=%08X A2=%08X A3=%08X SP=%08X %s\n",
                        static_cast<unsigned long long>(mac.totalCycles()), pc,
                        mac.cpu().d[0], mac.cpu().d[3], mac.cpu().a[0],
                        mac.cpu().a[1], mac.cpu().a[2], mac.cpu().a[3],
                        mac.cpu().a[7], instruction.c_str());
        }
        if (startupDriveBudget > 0 &&
            (pc == 0x40801484u || pc == 0x408014A4u ||
             pc == 0x408014B0u || pc == 0x408014B4u ||
             pc == 0x408014BEu || pc == 0x408014D2u ||
             pc == 0x408015A0u || pc == 0x408015A4u ||
             pc == 0x408015A6u || pc == 0x408015AAu ||
             pc == 0x408015ACu || pc == 0x408015C0u ||
             pc == 0x40801600u || pc == 0x40801616u ||
             pc == 0x408016D0u || pc == 0x40801740u)) {
            --startupDriveBudget;
            std::printf("%12llu STARTUP pc=%08X D0=%08X D1=%08X "
                        "D4=%08X D5=%08X D6=%08X A2=%08X "
                        "ticks=%08X tried=%04X\n",
                        static_cast<unsigned long long>(mac.totalCycles()), pc,
                        mac.cpu().d[0], mac.cpu().d[1], mac.cpu().d[4],
                        mac.cpu().d[5], mac.cpu().d[6], mac.cpu().a[2],
                        mac.read32(0x016Au), mac.read16(0x0B0Eu));
        }
        if (pc == 0x4086C51Cu || pc == 0x4086C522u || pc == 0x4086C54Cu ||
            pc == 0x0086C51Cu || pc == 0x0086C522u || pc == 0x0086C54Cu) {
            std::printf("%12llu SONY-OPEN pc=%08X D1=%08X D3=%08X D4=%08X "
                        "D5=%08X A2=%08X A3=%08X A4=%08X byte=%02X\n",
                        static_cast<unsigned long long>(mac.totalCycles()), pc,
                        mac.cpu().d[1], mac.cpu().d[3], mac.cpu().d[4],
                        mac.cpu().d[5], mac.cpu().a[2], mac.cpu().a[3],
                        mac.cpu().a[4], mac.read8(mac.cpu().a[4]));
        }
        if (sonyEventBudget > 0 &&
            (pc == 0x4086C988u || pc == 0x0086C988u ||
             pc == 0x4086C8ECu || pc == 0x0086C8ECu ||
             pc == 0x4086C9B2u || pc == 0x0086C9B2u ||
             pc == 0x4086C9B6u || pc == 0x0086C9B6u ||
             pc == 0x4086C9B8u || pc == 0x0086C9B8u ||
             pc == 0x4086C9BCu || pc == 0x0086C9BCu)) {
            --sonyEventBudget;
            std::printf("%12llu SONY-EVENT pc=%08X D0=%08X D1=%08X "
                        "A0=%08X A1=%08X A2=%08X A3=%08X A4=%08X SP=%08X\n",
                        static_cast<unsigned long long>(mac.totalCycles()), pc,
                        mac.cpu().d[0], mac.cpu().d[1], mac.cpu().a[0],
                        mac.cpu().a[1], mac.cpu().a[2], mac.cpu().a[3],
                        mac.cpu().a[4], mac.cpu().a[7]);
            const u32 message = (pc == 0x4086C988u || pc == 0x0086C988u)
                ? mac.cpu().a[0] - 0x20u : mac.cpu().a[2];
            dumpLogicalMemory(mac, message, 32, "Sony event message");
            dumpLogicalMemory(mac, mac.cpu().a[3], 32, "Sony drive record");
        }
        if (pc == 0x4080143Eu) osDefaultTrace = 240;
        if (osDefaultTrace > 0) {
            --osDefaultTrace;
            std::string instruction;
            try {
                dbg::disasm([&](u32 address) { return mac.read16(address); }, pc,
                            instruction);
            } catch (...) {
                instruction = "<unreadable>";
            }
            std::printf("%12llu OSDEFAULT pc=%08X D0=%08X D1=%08X "
                        "A0=%08X A1=%08X SP=%08X %s\n",
                        static_cast<unsigned long long>(mac.totalCycles()), pc,
                        mac.cpu().d[0], mac.cpu().d[1], mac.cpu().a[0],
                        mac.cpu().a[1], mac.cpu().a[7], instruction.c_str());
            if (pc == 0x40801440u)
                dumpLogicalMemory(mac, mac.cpu().a[7], 16, "GetOSDefault result");
        }
        if (scsiFlowBudget > 0 && mac.diagnosticScsiCommandCount() >= 7 &&
            (pc == 0x408073F4u || pc == 0x4080740Au || pc == 0x4080740Cu ||
             pc == 0x408074DCu || pc == 0x4080752Cu || pc == 0x40807632u ||
             pc == 0x40807554u || pc == 0x40807574u || pc == 0x40807578u ||
             pc == 0x4080757Cu || pc == 0x40807580u || pc == 0x40807582u ||
             pc == 0x408075A2u || pc == 0x408075ACu || pc == 0x408075B0u ||
             pc == 0x408075B2u || pc == 0x408075B4u || pc == 0x408075EAu ||
             pc == 0x4080760Eu || pc == 0x40807610u || pc == 0x40807624u ||
             pc == 0x40807678u || pc == 0x40807714u || pc == 0x408077CCu ||
             pc == 0x408077F4u || pc == 0x40807826u || pc == 0x40808A54u ||
             pc == 0x40808D24u || pc == 0x40808D28u || pc == 0x40808D2Au)) {
            --scsiFlowBudget;
            u32 stack0 = 0, stack4 = 0;
            try {
                stack0 = mac.read32(mac.cpu().a[7]);
                stack4 = mac.read32(mac.cpu().a[7] + 4u);
            } catch (...) {
                // A diagnostic probe must not affect the architected access.
            }
            std::printf("%12llu SCSI-FLOW pc=%08X D0=%08X D1=%08X D2=%08X "
                        "D4=%08X D7=%08X A0=%08X A1=%08X A2=%08X A3=%08X "
                        "A6=%08X SP=%08X stk=%08X/%08X cmd=%u phase=%d "
                        "xfer=%u/%u irq=%d drq=%d dma=%d ctl=%08X "
                        "cnt=%u addr=%08X\n",
                        static_cast<unsigned long long>(mac.totalCycles()), pc,
                        mac.cpu().d[0], mac.cpu().d[1], mac.cpu().d[2],
                        mac.cpu().d[4], mac.cpu().d[7], mac.cpu().a[0],
                        mac.cpu().a[1], mac.cpu().a[2], mac.cpu().a[3],
                        mac.cpu().a[6], mac.cpu().a[7], stack0, stack4,
                        mac.diagnosticScsiCommandCount(),
                        mac.diagnosticScsiPhase(),
                        mac.diagnosticScsiTransferPosition(),
                        mac.diagnosticScsiTransferLength(),
                        mac.diagnosticScsiIrq() ? 1 : 0,
                        mac.diagnosticScsiDrq() ? 1 : 0,
                        mac.diagnosticScsiDmaActive() ? 1 : 0,
                        mac.diagnosticScsiDmaControl(),
                        mac.diagnosticScsiDmaCount(),
                        mac.diagnosticScsiDmaAddress());
        }
        if ((pc == 0x40800480u || pc == 0x4080B810u ||
             pc == 0x4080B814u ||
             pc == 0x4080EFD4u || pc == 0x4080F000u ||
             pc == 0x40809930u || pc == 0x40809944u) &&
            queueTraceBudget-- > 0) {
            std::printf("%12llu QUEUE-ROM pc=%08X D0=%08X D1=%08X "
                        "A0=%08X A1=%08X SP=%08X\n",
                        static_cast<unsigned long long>(mac.totalCycles()), pc,
                        mac.cpu().d[0], mac.cpu().d[1], mac.cpu().a[0],
                        mac.cpu().a[1], mac.cpu().a[7]);
            if (pc == 0x4080B810u) mac.clearAccessLog();
            try {
                const u32 queuePhysical =
                    mac.cpu().diagnosticTranslate(0x00000360u);
                const u32 pbPhysical =
                    mac.cpu().diagnosticTranslate(mac.cpu().a[0]);
                std::printf("  translated queue=%08X PB=%08X\n",
                            queuePhysical, pbPhysical);
                dumpMemory(mac, queuePhysical, 16, "translated File Manager queue");
                dumpMemory(mac, pbPhysical, 40, "translated parameter block");
                if (pc == 0x4080B810u || pc == 0x4080B814u)
                    dumpMemory(mac, pbPhysical, 56, "Slot Manager parameter block");
                if (pc == 0x4080B814u)
                    dumpMemory(mac, 0xF9FF0080u, 128, "declaration-ROM driver record");
                if (pc == 0x4080B814u) {
                    std::printf("-- sGetDriver I/O (%zu accesses) --\n",
                                mac.accessLog().size());
                    for (const std::string& line : mac.accessLog())
                        std::printf("%s\n", line.c_str());
                    std::printf("-- sGetDriver recent PCs --\n");
                    for (int back = 127; back >= 0; --back)
                        std::printf(" %08X%s", mac.cpu().recentPc(back),
                                    (back % 8) == 0 ? "\n" : "");
                }
                if (pc == 0x4080EFD4u) {
                    const u32 stackPhysical =
                        mac.cpu().diagnosticTranslate(mac.cpu().a[7]);
                    dumpMemory(mac, stackPhysical, 96, "translated call stack");
                    std::printf("  preceding PCs:");
                    for (int back = 63; back >= 0; --back)
                        std::printf(" %08X", mac.cpu().recentPc(back));
                    std::printf("\n");
                }
            } catch (...) {
                std::printf("  translation failed\n");
            }
        }
        if (adbTraceBudget <= 0) return;
        if (pc == 0x4080A42Cu || pc == 0x4080A45Eu || pc == 0x4080A4D8u ||
            pc == 0x4080A566u || pc == 0x4080AA36u || pc == 0x40804DE4u ||
            pc == 0x40804E94u || pc == 0x40805040u) {
            --adbTraceBudget;
            std::printf("%12llu ADB-ROM pc=%08X D0=%08X D1=%08X D2=%08X "
                        "D3=%08X A0=%08X A1=%08X flags=%02X/%02X\n",
                        static_cast<unsigned long long>(mac.totalCycles()), pc,
                        mac.cpu().d[0], mac.cpu().d[1], mac.cpu().d[2],
                        mac.cpu().d[3], mac.cpu().a[0], mac.cpu().a[1],
                        mac.read8(0x00005111u), mac.read8(0x00005112u));
            if (pc == 0x40804DE4u)
                dumpMemory(mac, mac.cpu().a[0], 24, "IOP PBControl block");
        }
    };

    if (structuredOnly || quiet) {
        mac.setLegacyAccessLogEnabled(false);
        mac.onDiag = {};
        // A trap watch is an explicitly requested probe: keep the exception
        // hook (its other prints already honour quiet).
        if (!watchedTrapEnabled) mac.cpu().onException = {};
        mac.cpu().onInterrupt = {};
    }

    std::printf("IIfx trace: ROM=%s bytes=%zu RAM=%d MB frames=%d%s\n",
                argv[1], static_cast<std::size_t>(512u * 1024u), ramMb, frames,
                mac.hardDiskPresent() ? " disk=attached" : "");
    std::printf("reset pc=%08X sp=%08X overlay=%d\n", mac.cpu().pc,
                mac.cpu().a[7], mac.overlayActive() ? 1 : 0);

    bool checkpointSaved = false;
    bool checkpointStop = false;
    bool triggerCheckpointSaved = false;
    std::vector<u8> capturedAudio;
    u32 capturedAudioRate = 0;
    bool capturedAudioRateChanged = false;
    u64 capturedAudioHash = 1469598103934665603ull;
    u64 capturedAudioNonSilent = 0;
    u64 firstNonSilentSample = ~u64{0};
    u64 lastNonSilentSample = 0;
    u8 capturedAudioMin = 0xFF;
    u8 capturedAudioMax = 0;
    std::ofstream frameTimeline;
    std::vector<u32> frameTimelinePixels;
    u64 previousTimelineHash = 0;
    bool previousTimelineHashValid = false;
    std::size_t frameTimelineCaptures = 0;
    constexpr std::size_t kMaxFrameTimelineCaptures = 256;
    if (!frameTimelinePrefix.empty()) {
        frameTimeline.open(frameTimelinePrefix + ".csv", std::ios::binary);
        if (!frameTimeline) {
            std::fprintf(stderr, "cannot create frame timeline: %s.csv\n",
                         frameTimelinePrefix.c_str());
            return 2;
        }
        frameTimeline << "run_frame,machine_frame,cycles,pc,whole,top,welcome,"
                         "icons,colored_pixels,capture\n";
        frameTimelinePixels.resize(
            static_cast<std::size_t>(mac.screenWidth()) * mac.screenHeight());
        std::printf("frame timeline armed: %s frames=%d..%d max-captures=%zu\n",
                    frameTimelinePrefix.c_str(), frameTimelineFirst,
                    frameTimelineLast, kMaxFrameTimelineCaptures);
    }
    const auto captureFrameTimeline = [&](int runFrame) {
        if (!frameTimeline || runFrame < frameTimelineFirst ||
            runFrame > frameTimelineLast)
            return;
        const int width = mac.screenWidth();
        const int height = mac.screenHeight();
        mac.renderScreen(frameTimelinePixels.data());
        const u64 whole = pixelHash(frameTimelinePixels, width, height,
                                    0, 0, width, height);
        const u64 top = pixelHash(frameTimelinePixels, width, height,
                                  0, 0, width, 100);
        const u64 welcome = pixelHash(frameTimelinePixels, width, height,
                                      80, 75, 560, 225);
        const u64 icons = pixelHash(frameTimelinePixels, width, height,
                                    80, 145, 560, 225);
        std::size_t coloredPixels = 0;
        for (u32 pixel : frameTimelinePixels) {
            const u8 red = static_cast<u8>(pixel >> 16);
            const u8 green = static_cast<u8>(pixel >> 8);
            const u8 blue = static_cast<u8>(pixel);
            if (red != green || green != blue) ++coloredPixels;
        }
        const bool changed = !previousTimelineHashValid ||
                             whole != previousTimelineHash;
        std::string capture;
        if (changed && frameTimelineCaptures < kMaxFrameTimelineCaptures) {
            char suffix[32];
            std::snprintf(suffix, sizeof suffix, "-f%04d.ppm", runFrame);
            capture = frameTimelinePrefix + suffix;
            savePixels(frameTimelinePixels, width, height, capture.c_str());
            ++frameTimelineCaptures;
        }
        previousTimelineHash = whole;
        previousTimelineHashValid = true;
        frameTimeline << runFrame << ',' << mac.frameCount() << ','
                      << mac.totalCycles() << ',' << std::hex << std::uppercase
                      << std::setw(8) << std::setfill('0') << mac.cpu().pc << ','
                      << std::setw(16) << whole << ',' << std::setw(16) << top
                      << ',' << std::setw(16) << welcome << ','
                      << std::setw(16) << icons << std::dec << std::nouppercase
                      << std::setfill(' ') << ',' << coloredPixels << ','
                      << capture << '\n';
    };
    const auto captureAudio = [&]() {
        if (!audioDumpPath && !expectStartupChime) return;
        std::vector<u8> samples;
        mac.drainAudio(samples);
        if (samples.empty()) return;
        const u64 captureBase = static_cast<u64>(capturedAudio.size());
        const u32 rate = mac.audioSampleRate();
        if (capturedAudioRate == 0)
            capturedAudioRate = rate;
        else if (capturedAudioRate != rate)
            capturedAudioRateChanged = true;
        for (std::size_t index = 0; index < samples.size(); ++index) {
            const u8 sample = samples[index];
            capturedAudioHash ^= sample;
            capturedAudioHash *= 1099511628211ull;
            capturedAudioMin = std::min(capturedAudioMin, sample);
            capturedAudioMax = std::max(capturedAudioMax, sample);
            if (sample != 0x80) {
                const u64 position = captureBase + static_cast<u64>(index);
                if (firstNonSilentSample == ~u64{0})
                    firstNonSilentSample = position;
                lastNonSilentSample = position;
                ++capturedAudioNonSilent;
            }
        }
        capturedAudio.insert(capturedAudio.end(), samples.begin(), samples.end());
    };
    const auto saveCheckpointIfDue = [&]() {
        if (!saveStatePath || checkpointSaved || checkpointCycle == 0 ||
            mac.totalCycles() < checkpointCycle)
            return true;
        std::string error;
        if (!mac.saveStateFile(saveStatePath, &error)) {
            std::fprintf(stderr, "checkpoint save failed: %s\n", error.c_str());
            return false;
        }
        checkpointSaved = true;
        checkpointStop = stopAfterCheckpoint;
        std::printf("checkpoint saved: %s cycle=%llu frame=%llu pc=%08X\n",
                    saveStatePath,
                    static_cast<unsigned long long>(mac.totalCycles()),
                    static_cast<unsigned long long>(mac.frameCount()),
                    mac.cpu().pc);
        return true;
    };
    const auto saveTriggerCheckpointIfDue = [&]() {
        if (!triggerCheckpointPath || triggerCheckpointSaved ||
            !mac.traceCheckpointPending())
            return true;
        std::vector<u8> checkpoint;
        std::string reason;
        if (!mac.takeTraceCheckpoint(checkpoint, &reason)) return true;
        std::ofstream output(triggerCheckpointPath, std::ios::binary);
        output.write(reinterpret_cast<const char*>(checkpoint.data()),
                     static_cast<std::streamsize>(checkpoint.size()));
        if (!output) {
            std::fprintf(stderr, "trigger checkpoint save failed: %s\n",
                         triggerCheckpointPath);
            return false;
        }
        triggerCheckpointSaved = true;
        std::printf("trigger checkpoint saved: %s cycle=%llu frame=%llu "
                    "pc=%08X reason=\"%s\"\n",
                    triggerCheckpointPath,
                    static_cast<unsigned long long>(mac.totalCycles()),
                    static_cast<unsigned long long>(mac.frameCount()),
                    mac.cpu().pc, reason.c_str());
        return true;
    };
    bool gcPcTriggered = false;
    const auto checkGcPcTrigger = [&]() {
        if (!traceGcPcEnabled || gcPcTriggered ||
            mac.diagnosticGcProcessorPcWatchHits() < traceGcPcHits)
            return;
        gcPcTriggered = true;
        char reason[64]{};
        std::snprintf(reason, sizeof reason, "GC Am29000 PC %08X", traceGcPc);
        mac.triggerHardwareTrace(reason);
        std::printf("GC PC reached: %08X hit=%llu instructions=%llu cycle=%llu\n",
                    traceGcPc,
                    static_cast<unsigned long long>(
                        mac.diagnosticGcProcessorPcWatchHits()),
                    static_cast<unsigned long long>(
                        mac.diagnosticGcProcessorInstructions()),
                    static_cast<unsigned long long>(mac.totalCycles()));
    };
    const auto updateMilestones = [&]() {
        for (MilestoneDeadline& milestone : milestoneDeadlines) {
            if (milestone.reached) continue;
            bool reached = false;
            if (milestone.name == "iop-alive")
                reached = mac.diagnosticIsmFirmwareAlive();
            else if (milestone.name == "floppy-service")
                reached = mac.diagnosticFloppyServiceReady();
            else if (milestone.name == "floppy-mounted")
                reached = mac.diagnosticFloppyMounted();
            else if (milestone.name == "finder") {
                reached = applicationIs(mac, "Finder");
            } else if (milestone.name == "installer") {
                reached = applicationIs(mac, "Installer");
            } else if (milestone.name.rfind("application-", 0) == 0 &&
                       milestone.name.size() > 12u) {
                reached = applicationIs(mac, milestone.name.substr(12u));
            } else {
                std::fprintf(stderr, "unknown milestone name: %s\n",
                             milestone.name.c_str());
                return false;
            }
            if (reached) {
                milestone.reached = true;
                mac.recordHardwareMilestone(milestone.name);
                std::printf("milestone reached: %s frame=%llu cycle=%llu\n",
                            milestone.name.c_str(),
                            static_cast<unsigned long long>(mac.frameCount()),
                            static_cast<unsigned long long>(mac.totalCycles()));
            } else if (mac.frameCount() - milestoneOriginFrame >=
                       milestone.frames) {
                const std::string reason = "milestone timeout: " + milestone.name;
                mac.triggerHardwareTrace(reason);
                std::printf("milestone timeout: %s after=%llu frame=%llu\n",
                            milestone.name.c_str(),
                            static_cast<unsigned long long>(milestone.frames),
                            static_cast<unsigned long long>(mac.frameCount()));
                return false;
            }
        }
        return true;
    };

    if (frames < 0) {
        for (int instruction = 0; instruction < -frames; ++instruction) {
            try {
                mac.stepInstruction();
                checkGcPcTrigger();
            } catch (const std::exception& error) {
                std::fprintf(stderr,
                             "host exception at instruction=%d pc=%08X: %s\n",
                             instruction, mac.cpu().pc, error.what());
                return 3;
            } catch (...) {
                std::fprintf(stderr,
                             "unknown host exception at instruction=%d pc=%08X\n",
                             instruction, mac.cpu().pc);
                return 3;
            }
            if (triggerCheckpointPath && !triggerCheckpointSaved &&
                mac.hardwareTrace().triggered() &&
                !mac.traceCheckpointPending())
                mac.scheduleTraceCheckpoint(mac.hardwareTrace().triggerReason());
            if (!saveCheckpointIfDue()) return 2;
            if (!saveTriggerCheckpointIfDue()) return 2;
            if ((instruction & 1023) == 1023) captureAudio();
            if (mac.cpu().halted || mac.poweredOff()) break;
            if (stopOnMilestone &&
                applicationIs(mac, expectedApplication)) {
                mac.recordHardwareMilestone("application-" +
                                            expectedApplication);
                std::printf("application milestone reached: %s frame=%llu "
                            "cycle=%llu\n", expectedApplication.c_str(),
                            static_cast<unsigned long long>(mac.frameCount()),
                            static_cast<unsigned long long>(mac.totalCycles()));
                break;
            }
            if (checkpointStop ||
                (stopAfterCheckpoint && pcCheckpointSaved)) break;
            if (stopOnTrace && mac.hardwareTrace().frozen()) break;
        }
    } else {
        HostSampler hostSampler;
        if (hostProfile) hostSampler.start();
        const auto frameLoopStart = std::chrono::steady_clock::now();
        struct SamplerStop {
            HostSampler& sampler;
            bool enabled;
            std::chrono::steady_clock::time_point start;
            int framesRequested;
            ~SamplerStop() {
                const double seconds = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count();
                std::printf("frame loop wall: %.3f s for %d frames (%.2f ms/frame)\n",
                            seconds, framesRequested,
                            framesRequested > 0 ? seconds * 1000.0 / framesRequested
                                                : 0.0);
                if (!enabled) return;
                sampler.stop();
                std::printf("%s", sampler.report().c_str());
            }
        } samplerStop{hostSampler, hostProfile, frameLoopStart, frames};
        for (int frame = 0; frame < frames; ++frame) {
            for (FloppyEvent& event : floppyEvents) {
                if (event.frame != frame) continue;
                if (event.eject) {
                    mac.ejectFloppy();
                    std::printf("floppy ejected at frame=%d\n", frame);
                    mac.recordHardwareMilestone("floppy-ejected");
                    continue;
                }
                const int accepted = mac.insertFloppy(std::move(event.image));
                std::printf("floppy inserted at frame=%d: %s (%s)\n", frame,
                            event.path.c_str(),
                            accepted ? "accepted" : "REFUSED");
                if (!accepted) return 2;
                mac.recordHardwareMilestone("floppy-inserted");
            }
            if (argc >= 8 && frame + 1 == std::atoi(argv[7])) {
                const int dx = argc >= 9 ? std::atoi(argv[8]) : 24;
                const int dy = argc >= 10 ? std::atoi(argv[9]) : 12;
                adbTraceBudget = 96;
                iopDeviceBudget = 96;
                mac.mouseMove(dx, dy, false);
                std::printf("input injected: frame=%d mouse=%d,%d\n",
                            frame + 1, dx, dy);
            }
            for (const InputEvent& event : inputEvents) {
                if (event.frame != frame) continue;
                adbTraceBudget = 96;
                iopDeviceBudget = 96;
                mac.mouseMove(event.dx, event.dy, event.button);
                std::printf("input event: frame=%d mouse=%d,%d button=%s\n",
                            frame, event.dx, event.dy,
                            event.button ? "down" : "up");
            }
            for (MouseGoto& target : mouseGotos) {
                if (target.done || frame < target.frame) continue;
                const int guestV = static_cast<int>(
                    static_cast<int16_t>((mac.read8(0x830u) << 8) |
                                         mac.read8(0x831u)));
                const int guestH = static_cast<int>(
                    static_cast<int16_t>((mac.read8(0x832u) << 8) |
                                         mac.read8(0x833u)));
                const int dx = target.x - guestH;
                const int dy = target.y - guestV;
                if (dx == 0 && dy == 0) {
                    target.done = true;
                    std::printf("mouse goto reached: frame=%d at %d,%d%s\n",
                                frame, guestH, guestV,
                                target.button ? " (button held)" : "");
                    continue;
                }
                const auto step = [](int delta) {
                    if (delta > 3) return 2;
                    if (delta < -3) return -2;
                    return delta > 0 ? 1 : delta < 0 ? -1 : 0;
                };
                mac.mouseMove(step(dx), step(dy), target.button);
                if (!target.active) {
                    target.active = true;
                    std::printf("mouse goto started: frame=%d from %d,%d to "
                                "%d,%d\n", frame, guestH, guestV, target.x,
                                target.y);
                }
                break; // one goto drives the pointer at a time
            }
            for (const KeyEvent& event : keyEvents) {
                if (event.frame != frame) continue;
                adbTraceBudget = 96;
                iopDeviceBudget = 96;
                mac.keyEvent(event.adbCode, event.down);
                std::printf("key event: frame=%d adb=%02X state=%s\n",
                            frame, event.adbCode,
                            event.down ? "down" : "up");
            }
            try {
                const u64 targetFrame = mac.frameCount() + 1;
                while (mac.frameCount() < targetFrame && !mac.cpu().halted &&
                       !mac.poweredOff()) {
                    mac.stepInstruction();
                    checkGcPcTrigger();
                    if (triggerCheckpointPath && !triggerCheckpointSaved &&
                        mac.hardwareTrace().triggered() &&
                        !mac.traceCheckpointPending())
                        mac.scheduleTraceCheckpoint(
                            mac.hardwareTrace().triggerReason());
                    if (!saveCheckpointIfDue()) return 2;
                    if (!saveTriggerCheckpointIfDue()) return 2;
                    if (checkpointStop ||
                        (stopAfterCheckpoint && pcCheckpointSaved) ||
                        (stopOnTrace && mac.hardwareTrace().frozen()))
                        break;
                }
            } catch (const std::exception& error) {
                std::fprintf(stderr,
                             "host exception at frame=%d pc=%08X: %s\n",
                             frame + 1, mac.cpu().pc, error.what());
                return 3;
            } catch (...) {
                std::fprintf(stderr,
                             "unknown host exception at frame=%d pc=%08X\n",
                             frame + 1, mac.cpu().pc);
                return 3;
            }
            captureAudio();
            captureFrameTimeline(frame + 1);
            if (!updateMilestones()) {
                if (!saveTriggerCheckpointIfDue()) return 2;
                if (stopOnTrace || mac.hardwareTrace().triggered()) break;
            }
            if (!structuredOnly && !quiet &&
                (frame < 20 || (frame % 60) == 59 || mac.cpu().halted ||
                 mac.poweredOff())) {
                std::printf("frame=%d cycles=%llu pc=%08X sr=%04X ov=%d log=%zu%s%s\n",
                            frame + 1, static_cast<unsigned long long>(mac.totalCycles()),
                            mac.cpu().pc, mac.cpu().getSR(), mac.overlayActive() ? 1 : 0,
                            mac.accessLog().size(), mac.cpu().halted ? " HALTED" : "",
                            mac.poweredOff() ? " POWERED-OFF" : "");
            }
            if (mac.cpu().halted || mac.poweredOff()) break;
            if (stopOnMilestone &&
                applicationIs(mac, expectedApplication)) {
                mac.recordHardwareMilestone("application-" +
                                            expectedApplication);
                std::printf("application milestone reached: %s frame=%llu "
                            "cycle=%llu\n", expectedApplication.c_str(),
                            static_cast<unsigned long long>(mac.frameCount()),
                            static_cast<unsigned long long>(mac.totalCycles()));
                break;
            }
            if (checkpointStop ||
                (stopAfterCheckpoint && pcCheckpointSaved)) break;
            if (stopOnTrace && mac.hardwareTrace().frozen()) break;
        }
    }

    captureAudio();

    if (saveStatePath && !checkpointSaved) {
        std::string error;
        if (!mac.saveStateFile(saveStatePath, &error)) {
            std::fprintf(stderr, "checkpoint save failed: %s\n", error.c_str());
            return 2;
        }
        checkpointSaved = true;
        std::printf("checkpoint saved: %s cycle=%llu frame=%llu pc=%08X\n",
                    saveStatePath,
                    static_cast<unsigned long long>(mac.totalCycles()),
                    static_cast<unsigned long long>(mac.frameCount()),
                    mac.cpu().pc);
    }

    for (const LiveDisassembly& listing : liveDisassemblies) {
        u32 pc = listing.address;
        std::printf("-- live disassembly at %08X --\n", pc);
        for (int index = 0; index < listing.count; ++index) {
            std::string instruction;
            try {
                const int bytes = dbg::disasm(
                    [&](u32 address) { return mac.read16(address); }, pc,
                    instruction);
                std::printf("  %08X ", pc);
                for (int word = 0; word < 5; ++word) {
                    if (word * 2 < bytes)
                        std::printf(" %04X", mac.read16(
                            pc + static_cast<u32>(word * 2)));
                    else
                        std::printf("     ");
                }
                std::printf("  %s\n", instruction.c_str());
                pc += static_cast<u32>(std::max(bytes, 2));
            } catch (const BusFault&) {
                std::printf("  %08X  <unmapped>\n", pc);
                break;
            }
        }
    }

    if (!structuredOnly && !quiet) {
    std::printf("\n%s", mac.diagnosticReport().c_str());
    std::printf("  MMU CRP=%016llX SRP=%016llX TT0=%08X TT1=%08X\n",
                static_cast<unsigned long long>(mac.cpu().crp),
                static_cast<unsigned long long>(mac.cpu().srp030),
                mac.cpu().tt0, mac.cpu().tt1);
    const u32 adbOp = mac.read32(0x000005F0u);
    const u32 adbGlobals = mac.read32(0x00000CF8u);
    std::printf("  low-memory ADBOp=%08X ADBGlobals=%08X PBControl=%08X "
                "ADBBase=%08X\n", adbOp, adbGlobals, mac.read32(0x0000061Cu),
                mac.read32(0x000001D4u));
    std::printf("  low-memory mouse V=%04X H=%04X raw=%02X %02X %02X %02X\n",
                mac.read16(0x00000828u), mac.read16(0x0000082Au),
                mac.read8(0x00000828u), mac.read8(0x00000829u),
                mac.read8(0x0000082Au), mac.read8(0x0000082Bu));
    try {
        const u32 tickPhysical = mac.cpu().diagnosticTranslate(0x0000016Au);
        std::printf("  low-memory TickCount physical=%08X value=%08X "
                    "raw-at-016A=%08X\n",
                    tickPhysical, mac.read32(tickPhysical), mac.read32(0x0000016Au));
    } catch (...) {
        std::printf("  low-memory TickCount=<unmapped>\n");
    }
    if (adbGlobals >= 0x1000u && adbGlobals < config.ramSize - 0x1B8u)
        dumpMemory(mac, adbGlobals + 0x130u, 0x60u, "ADB globals");
    const u32 driveQueue = mac.read32(0x030Au);
    std::printf("  boot globals BootDrive=%04X ROM85=%04X DrvQHead=%08X\n",
                mac.read16(0x0210u), mac.read16(0x028Eu), driveQueue);
    u32 scsiGlobals = mac.read32(0x0C0Cu);
    if (scsiGlobals >= config.ramSize)
        scsiGlobals &= 0x00FFFFFFu;
    if (scsiGlobals && scsiGlobals + 0x80u < config.ramSize) {
        std::printf("  SCSI globals=%08X controller=%08X dmaRead=%08X "
                    "dmaWrite=%08X dataOffset=%08X flags=%08X\n",
                    scsiGlobals, mac.read32(scsiGlobals + 0x40u),
                    mac.read32(scsiGlobals + 0x44u),
                    mac.read32(scsiGlobals + 0x48u),
                    mac.read32(scsiGlobals + 0x4Cu),
                    mac.read32(scsiGlobals + 0x20u));
    }
    if (driveQueue >= 4 && driveQueue < config.ramSize - 32)
        dumpLogicalMemory(mac, driveQueue - 4, 32, "IIfx Sony drive record");
    auto ramPointer = [&](u32 address) {
        if (address < config.ramSize) return address;
        const u32 low24 = address & 0x00FFFFFFu;
        return low24 < config.ramSize ? low24 : address;
    };
    std::printf("  drive queue:");
    u32 driveEntry = driveQueue;
    for (int count = 0; count < 8 && driveEntry && driveEntry != 0xFFFFFFFFu;
         ++count) {
        driveEntry = ramPointer(driveEntry);
        if (driveEntry < 3 || driveEntry + 12 >= config.ramSize) break;
        std::printf(" [drive=%u ref=%d fsid=%04X inPlace=%d entry=%08X]",
                    mac.read16(driveEntry + 6),
                    static_cast<s16>(mac.read16(driveEntry + 8)),
                    mac.read16(driveEntry + 10),
                    static_cast<s8>(mac.read8(driveEntry - 3)), driveEntry);
        driveEntry = mac.read32(driveEntry);
    }
    std::printf("\n  volume queue:");
    u32 volume = mac.read32(0x0358u);
    for (int count = 0; count < 8 && volume && volume != 0xFFFFFFFFu; ++count) {
        volume = ramPointer(volume);
        if (volume + 80 >= config.ramSize) break;
        char name[28]{};
        const int length = std::min<int>(mac.read8(volume + 44), 27);
        for (int i = 0; i < length; ++i)
            name[i] = static_cast<char>(mac.read8(volume + 45 + i));
        std::printf(" [drive=%d ref=%d sig=%04X name=\"%s\"]",
                    static_cast<s16>(mac.read16(volume + 72)),
                    static_cast<s16>(mac.read16(volume + 74)),
                    mac.read16(volume + 8), name);
        volume = mac.read32(volume);
    }
    std::printf("\n");
    const u8 driverSignature[] = {0x45, 0xFA, 0x00, 0x52, 0x26, 0x28, 0x00, 0x08};
    u32 driverAddress = 0;
    for (u32 address = 0; address + sizeof driverSignature <= config.ramSize;
         ++address) {
        bool match = true;
        for (u32 byte = 0; byte < sizeof driverSignature; ++byte)
            match &= mac.read8(address + byte) == driverSignature[byte];
        if (match) { driverAddress = address; break; }
    }
    std::printf("  generated SCSI driver image=%08X\n", driverAddress);
    if (driverAddress)
        dumpMemory(mac, driverAddress, 0x100u, "generated SCSI driver image");
    dumpMemory(mac, 0x000002E0u, 0x60u, "drive and tag globals");
    dumpLogicalMemory(mac, 0x00001FBCu, 0x100u, "candidate SCSI driver load area");
    dumpMemory(mac, 0x00000340u, 0x60u, "low-memory queues");
    dumpLogicalMemory(mac, 0x00000340u, 0x60u, "low-memory queues");
    std::printf("-- last I/O accesses (%zu total retained) --\n", mac.accessLog().size());
    const std::size_t begin = mac.accessLog().size() > 160 ? mac.accessLog().size() - 160 : 0;
    for (std::size_t i = begin; i < mac.accessLog().size(); ++i)
        std::printf("%s\n", mac.accessLog()[i].c_str());
    std::printf("-- recent PCs --\n");
    for (int back = 31; back >= 0; --back) {
        const u32 pc = mac.cpu().recentPc(back);
        std::string instruction;
        try {
            dbg::disasm([&](u32 address) { return mac.read16(address); }, pc,
                        instruction);
        } catch (const BusFault&) {
            instruction = "<unreadable>";
        }
        std::printf("  %08X  %s\n", pc, instruction.c_str());
    }
    }

    for (const MemoryRange& range : memoryDumps) {
        if (range.logical)
            dumpLogicalMemory(mac, range.address, range.size,
                              "requested memory dump");
        else
            dumpMemory(mac, range.address, range.size,
                       "requested memory dump");
    }
    for (const BinaryMemoryDump& dump : binaryMemoryDumps) {
        std::vector<u8> bytes(dump.size);
        for (u32 offset = 0; offset < dump.size; ++offset)
            bytes[offset] = mac.read8(dump.address + offset);
        saveBinary(bytes, dump.path);
    }
    for (const BinaryMemoryDump& dump : gcDataMemoryDumps) {
        const std::vector<u8> bytes =
            mac.diagnosticGcDataMemory(dump.address, dump.size);
        if (bytes.size() != dump.size) {
            std::fprintf(stderr,
                "invalid GC data-memory dump range: %08X:%X\n",
                dump.address, dump.size);
            return 2;
        }
        saveBinary(bytes, dump.path);
    }
    if (flightGcPath) {
        if (mac.diagnosticWriteGcFlight(flightGcPath))
            std::printf("flight recorder saved: %s\n", flightGcPath);
        else
            std::fprintf(stderr, "flight recorder write failed: %s\n",
                         flightGcPath);
    }
    for (const BinaryMemoryDump& dump : gcBinaryMemoryDumps) {
        const std::vector<u8> bytes =
            mac.diagnosticGcExpansionMemory(dump.address, dump.size);
        if (bytes.size() != dump.size) {
            std::fprintf(stderr,
                "invalid GC expansion-memory dump range: %08X:%X\n",
                dump.address, dump.size);
            return 2;
        }
        saveBinary(bytes, dump.path);
    }
    for (const BinaryMemoryDump& dump : gcSramBinaryMemoryDumps) {
        const std::vector<u8> bytes =
            mac.diagnosticGcInstructionSram(dump.address, dump.size);
        if (bytes.size() != dump.size) {
            std::fprintf(stderr,
                "invalid GC instruction-SRAM dump range: %08X:%X\n",
                dump.address, dump.size);
            return 2;
        }
        saveBinary(bytes, dump.path);
    }

    if (printCpuState) {
        std::printf("CPU pc=%08X sr=%04X", mac.cpu().pc, mac.cpu().getSR());
        for (int index = 0; index < 8; ++index)
            std::printf(" D%d=%08X", index, mac.cpu().d[index]);
        for (int index = 0; index < 8; ++index)
            std::printf(" A%d=%08X", index, mac.cpu().a[index]);
        std::printf("\n");
    }
    if (watchedTrapEnabled) {
        std::printf("watched traps");
        for (const u16 opcode : watchedTraps) std::printf(" %04X", opcode);
        std::printf(" hits=%llu\n",
                    static_cast<unsigned long long>(watchedTrapHits));
    }

    // Quiet/structured captures still produce explicitly requested artifacts.
    // Suppressing legacy printf diagnostics must never suppress observability.
    const char* framePath = argc >= 7 && std::string(argv[6]) != "no-video" &&
                            std::string(argv[6]) != "no-frame"
        ? argv[6] : nullptr;
    saveFrame(mac, framePath);

    if (audioDumpPath) {
        if (capturedAudioRate == 0) capturedAudioRate = mac.audioSampleRate();
        if (!saveWav(capturedAudio, audioDumpPath, capturedAudioRate)) {
            std::fprintf(stderr, "cannot save audio WAV: %s\n", audioDumpPath);
            return 2;
        }
        const u8 minimum = capturedAudio.empty() ? 0x80 : capturedAudioMin;
        const u8 maximum = capturedAudio.empty() ? 0x80 : capturedAudioMax;
        std::printf("audio saved: %s rate=%u samples=%zu non-silent=%llu "
                    "min/max=%02X/%02X hash=%016llX%s\n",
                    audioDumpPath, capturedAudioRate, capturedAudio.size(),
                    static_cast<unsigned long long>(capturedAudioNonSilent),
                    minimum, maximum,
                    static_cast<unsigned long long>(capturedAudioHash),
                    capturedAudioRateChanged ? " RATE-CHANGED" : "");
    }
    if (expectStartupChime) {
        if (capturedAudioRate == 0) capturedAudioRate = mac.audioSampleRate();
        const u64 activeSpan = firstNonSilentSample == ~u64{0}
            ? 0 : lastNonSilentSample - firstNonSilentSample + 1u;
        const bool passed = !capturedAudioRateChanged &&
            capturedAudioRate == 22254u && capturedAudioNonSilent >= 12000u &&
            activeSpan >= 12500u && activeSpan <= 14500u &&
            capturedAudioMin <= 0x60u && capturedAudioMax >= 0x98u;
        std::printf("startup chime: %s rate=%u non-silent=%llu span=%llu "
                    "min/max=%02X/%02X hash=%016llX\n",
                    passed ? "PASS" : "FAIL", capturedAudioRate,
                    static_cast<unsigned long long>(capturedAudioNonSilent),
                    static_cast<unsigned long long>(activeSpan),
                    capturedAudio.empty() ? 0x80 : capturedAudioMin,
                    capturedAudio.empty() ? 0x80 : capturedAudioMax,
                    static_cast<unsigned long long>(capturedAudioHash));
        if (!passed) return 4;
    }

    if (iopDumpPrefix) {
        const std::string prefix = iopDumpPrefix;
        saveBinary(mac.iopRamImage(false), prefix + "-scc.bin");
        saveBinary(mac.iopRamImage(true), prefix + "-ism.bin");
    }
    if (saveDiskPath) {
        const std::vector<u8>& disk = mac.hardDiskImage();
        if (disk.empty()) {
            std::fprintf(stderr, "cannot save disk: no SCSI media attached\n");
            return 2;
        }
        saveBinary(disk, saveDiskPath);
    }
    if (saveDisk2Path) {
        const std::vector<u8>& disk2 = mac.hardDisk2Image();
        if (disk2.empty()) {
            std::fprintf(stderr, "cannot save second disk: none attached\n");
            return 2;
        }
        saveBinary(disk2, saveDisk2Path);
    }

    // A ROM-gated smoke test needs a semantic milestone, not merely an
    // unhalted CPU. CurApName is a Pascal string in low memory and identifies
    // the foreground application reached by the selected startup medium.
    const bool applicationReached =
        applicationIs(mac, expectedApplication);
    if (!expectedApplication.empty())
        std::printf("Application milestone %s: %s\n",
                    expectedApplication.c_str(),
                    applicationReached ? "reached" : "NOT reached");

    if (mediaSummary) {
        auto ramPointer = [&](u32 address) {
            if (address < config.ramSize) return address;
            const u32 low24 = address & 0x00FFFFFFu;
            return low24 < config.ramSize ? low24 : address;
        };
        std::printf("media summary: drives");
        u32 drive = mac.read32(0x030Au);
        for (int count = 0; count < 8 && drive && drive != 0xFFFFFFFFu; ++count) {
            drive = ramPointer(drive);
            if (drive < 3 || drive + 12 >= config.ramSize) break;
            std::printf(" [%u:%d]", mac.read16(drive + 6),
                        static_cast<s8>(mac.read8(drive - 3)));
            drive = mac.read32(drive);
        }
        std::printf(" volumes");
        u32 volume = mac.read32(0x0358u);
        for (int count = 0; count < 8 && volume && volume != 0xFFFFFFFFu; ++count) {
            volume = ramPointer(volume);
            if (volume + 80 >= config.ramSize) break;
            char name[28]{};
            const int length = std::min<int>(mac.read8(volume + 44), 27);
            for (int index = 0; index < length; ++index)
                name[index] = static_cast<char>(mac.read8(volume + 45 + index));
            std::printf(" [%d:%s]", static_cast<s16>(mac.read16(volume + 72)), name);
            volume = mac.read32(volume);
        }
        std::printf("\n");
    }

    if (hardwareTracePath) {
        mac.recordHardwareMilestone(applicationReached
                                        ? "application-" + expectedApplication
                                        : "end-before-application");
        if (!mac.hardwareTrace().triggered())
            mac.triggerHardwareTrace(applicationReached
                                         ? "application milestone"
                                         : "run ended before application milestone");
        if (!saveTriggerCheckpointIfDue()) return 2;
        mac.hardwareTrace().freeze();
        const bool saved = mac.writeHardwareTraceJsonl(hardwareTracePath);
        std::printf("structured trace: %s events=%zu triggered=%d frozen=%d "
                    "reason=\"%s\"\n",
                    saved ? hardwareTracePath : "WRITE FAILED",
                    mac.hardwareTrace().size(),
                    mac.hardwareTrace().triggered() ? 1 : 0,
                    mac.hardwareTrace().frozen() ? 1 : 0,
                    mac.hardwareTrace().triggerReason().c_str());
        if (!saved) return 2;
    }
    if (profileGcRange) {
        const auto sites = mac.diagnosticGcProcessorProfile(profilePcLimit);
        u64 total = 0;
        for (const auto& site : sites) total += site.first;
        std::printf("GC profile %08X:%08X top %zu sites (%llu counted):\n",
                    profileGcFirst, profileGcLast, sites.size(),
                    static_cast<unsigned long long>(total));
        for (const auto& site : sites)
            std::printf("  %08X %llu\n", site.second,
                        static_cast<unsigned long long>(site.first));
    }
    if (profilePcRange) {
        std::vector<std::pair<u64, u32>> sites;
        u64 total = 0;
        for (std::size_t index = 0; index < profilePcCounts.size(); ++index) {
            const u64 count = profilePcCounts[index];
            if (count == 0) continue;
            const u32 pc = profilePcFirst + static_cast<u32>(index * 2u);
            sites.emplace_back(count, pc);
            total += count;
        }
        std::sort(sites.begin(), sites.end(),
                  [](const auto& left, const auto& right) {
                      return left.first != right.first
                          ? left.first > right.first : left.second < right.second;
                  });
        std::printf("profile PC range %08X:%08X total=%llu unique=%zu\n",
                    profilePcFirst, profilePcLast,
                    static_cast<unsigned long long>(total), sites.size());
        const std::size_t shown = std::min(sites.size(), profilePcLimit);
        for (std::size_t index = 0; index < shown; ++index) {
            std::string instruction;
            try {
                dbg::disasm([&](u32 address) { return mac.read16(address); },
                            sites[index].second, instruction);
            } catch (...) {
                instruction = "<unreadable>";
            }
            std::printf("  %08X %10llu  %s\n", sites[index].second,
                        static_cast<unsigned long long>(sites[index].first),
                        instruction.c_str());
        }
    }
    std::printf("deterministic hashes: state=%016llX framebuffer=%016llX\n",
                static_cast<unsigned long long>(mac.deterministicStateHash()),
                static_cast<unsigned long long>(mac.framebufferHash()));

    if (shutdown)
        std::printf("guest hard-disk shutdown: %s\n",
                    mac.shutdownHardDisk() ? "settled" : "not settled");
    if (mac.cpu().halted) return 1;
    return !expectedApplication.empty() && !applicationReached ? 3 : 0;
}
