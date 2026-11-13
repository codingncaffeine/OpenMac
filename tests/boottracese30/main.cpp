// Headless Macintosh SE/30 bring-up harness: starts the machine from a ROM
// file, runs a number of frames and reports what the ROM did -- how it sized
// memory, which machine it took the board for, whether a start-up test
// failed, what is on the screen, where the CPU is.
//
//   openmac_tracese30 --rom FILE [--vrom FILE] [--ram MB | --banks A B]
//                     [--frames N] [--shot OUT.ppm] [--pram FILE] [--save-pram FILE]
//                     [--log] [--report] [--hashes FROM STEP] [--peek ADDRESS COUNT]...
//                     [--mouse-at FRAME DX DY BUTTON]... [--key-at FRAME CODE down|up]...
//                     [--floppy FILE] [--floppy-writable] [--save-floppy OUT]
//                     [--floppy-at FRAME FILE]... [--eject-floppy-at FRAME]...
//                     [--harddisk FILE] [--harddisk2 FILE] [--harddisk2-read-only]
//                     [--save-harddisk OUT] [--save-harddisk-at FRAME OUT]
//                     [--cd FILE] [--eject-cd-at FRAME]
//                     [--guest-restart-at FRAME] [--put-away-disk2-at FRAME]
//                     [--dropbox DIR] [--dropbox-guest-read FILE] [--net]
//                     [--gestalt] [--drivers] [--shutdown] [--hd-trace N]
//                     [--profile FROM TO] [--disasm ADDRESS COUNT]...
//                     [--traps FRAME COUNT] [--guest-calls] [--driver-over-scsi]
//                     [--beep-at FRAME]... [--guest-power-off-at FRAME] [--dump-audio OUT.wav]
//
// Disk and disc files are read into memory and never written; --save-harddisk
// writes the first disk as it stands at the end, --save-harddisk-at as it
// stands before a frame. --dropbox builds a volume
// from a host folder and puts it on the second hard-disk seat, as the front
// end's drop box does; --dropbox-guest-read has the guest read one of its
// files back at the end. --net attaches the SCSI Ethernet adapter (ID 4).
// --gestalt asks the System what it is running on; --drivers lists the
// guest's unit table; --shutdown flushes and unmounts as the front end does
// when it closes. --hd-trace logs the first N requests to the disks' driver.
// --profile counts instructions per 16 bytes of code between two frames and
// lists the busiest places: where a machine that looks hung spends its time.
// --disasm lists guest code at the end of the run. --traps lists the first
// COUNT system calls the guest makes from FRAME on, with D0 and A0;
// --guest-calls (with --log) reports each call the machine runs on the guest
// CPU for the host, going in and coming out. --driver-over-scsi leaves the
// hard disks' own driver to move every block over the SCSI bus.
//
// The sound output is always summarized: how many samples came out of the
// speaker and each stretch of anything but silence (stretches end after 0.2 s
// of silence) with its length and loudest sample, and how the stereo jack's
// two channels compare. --dump-audio writes the jack's two channels to a WAV
// file. --beep-at has the guest play its alert sound; --guest-power-off-at has
// it make the call the Finder's Shut Down ends in.
//
// A floppy file is never changed: a writable disk lives in memory and goes to
// --save-floppy at the end, in the file format it came in.
//
// --ram takes one of the totals the machine offers (1 2 4 5 8 16 17 20 32 65
// 68 80 128); --banks gives the two SIMM banks in megabytes (0, 1, 4, 16 or
// 64) for sets the machine does not offer. --vrom is the 8 KB video ROM;
// without it slot $E is empty. --hashes prints the distinct pictures seen
// from frame FROM on, sampled every STEP frames; a picture counts only when
// the frame before showed the same one, so a frame that ends in the middle
// of a redraw is not taken for a picture. --peek prints guest memory at the
// end.
//
// The start-up tests' verdict is read where the ROM takes it: a failed test
// jumps to ROM $2C3C with the test's number in D7 and the failing bits in
// D6; the start-up chime's routine at $5E4A is reached only when none failed.
// Exit code 0 = ran to the end; 1 = the CPU halted; 2 = bad arguments.

#include "openmac/debugger.hpp"
#include "openmac/se30.hpp"

#include "../common/tracetools.hpp"

#include <algorithm>
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
    std::string romPath, vromPath, shotPath, pramIn, pramOut, floppyPath, floppyOut;
    bool floppyWritable = false;
    std::vector<std::pair<int, std::string>> floppyEvents;   // frame, file ("" = eject)
    std::string hdPath, hd2Path, hdOut, cdPath, dropBoxDir, dropBoxGuestRead;
    bool hd2ReadOnly = false, netAttach = false, askGestalt = false, showDrivers = false, doShutdown = false;
    int ejectCdAt = -1, guestRestartAt = -1, putAwayDisk2At = -1, hdTrace = 0;
    int guestPowerOffAt = -1, hdOutAtFrame = -1;
    std::vector<int> beeps;         // frames at which the alert sound is asked for, in order
    std::string hdOutAt;
    std::string wavPath;
    int profileFrom = -1, profileTo = -1;
    std::map<u32, u64> profile;
    std::vector<std::pair<u32, int>> listings;
    int trapsFrom = -1, trapsLeft = 0;
    bool guestCalls = false, driverOverScsi = false;
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
        else if (a == "--floppy" && i + 1 < argc) floppyPath = argv[++i];
        else if (a == "--floppy-writable") floppyWritable = true;
        else if (a == "--save-floppy" && i + 1 < argc) floppyOut = argv[++i];
        else if (a == "--floppy-at" && i + 2 < argc) { const int f = std::atoi(argv[++i]); floppyEvents.emplace_back(f, argv[++i]); }
        else if (a == "--eject-floppy-at" && i + 1 < argc) floppyEvents.emplace_back(std::atoi(argv[++i]), "");
        else if (a == "--harddisk" && i + 1 < argc) hdPath = argv[++i];
        else if (a == "--harddisk2" && i + 1 < argc) hd2Path = argv[++i];
        else if (a == "--harddisk2-read-only") hd2ReadOnly = true;
        else if (a == "--save-harddisk" && i + 1 < argc) hdOut = argv[++i];
        else if (a == "--save-harddisk-at" && i + 2 < argc) { hdOutAtFrame = std::atoi(argv[++i]); hdOutAt = argv[++i]; }
        else if (a == "--cd" && i + 1 < argc) cdPath = argv[++i];
        else if (a == "--eject-cd-at" && i + 1 < argc) ejectCdAt = std::atoi(argv[++i]);
        else if (a == "--guest-restart-at" && i + 1 < argc) guestRestartAt = std::atoi(argv[++i]);
        else if (a == "--put-away-disk2-at" && i + 1 < argc) putAwayDisk2At = std::atoi(argv[++i]);
        else if (a == "--dropbox" && i + 1 < argc) dropBoxDir = argv[++i];
        else if (a == "--dropbox-guest-read" && i + 1 < argc) dropBoxGuestRead = argv[++i];
        else if (a == "--net") netAttach = true;
        else if (a == "--gestalt") askGestalt = true;
        else if (a == "--drivers") showDrivers = true;
        else if (a == "--shutdown") doShutdown = true;
        else if (a == "--hd-trace" && i + 1 < argc) hdTrace = std::atoi(argv[++i]);
        else if (a == "--guest-calls") guestCalls = true;
        else if (a == "--beep-at" && i + 1 < argc) beeps.push_back(std::atoi(argv[++i]));
        else if (a == "--guest-power-off-at" && i + 1 < argc) guestPowerOffAt = std::atoi(argv[++i]);
        else if (a == "--dump-audio" && i + 1 < argc) wavPath = argv[++i];
        else if (a == "--driver-over-scsi") driverOverScsi = true;
        else if (a == "--profile" && i + 2 < argc) { profileFrom = std::atoi(argv[++i]); profileTo = std::atoi(argv[++i]); }
        else if (a == "--traps" && i + 2 < argc) { trapsFrom = std::atoi(argv[++i]); trapsLeft = std::atoi(argv[++i]); }
        else if (a == "--disasm" && i + 2 < argc) {
            const u32 at = static_cast<u32>(std::strtoul(argv[++i], nullptr, 0));
            listings.emplace_back(at, std::atoi(argv[++i]));
        }
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

    if (!floppyPath.empty()) {
        const bool ok = mac.insertFloppy(tracetools::loadFile(floppyPath.c_str()), !floppyWritable);
        std::printf("floppy %s: %s\n", floppyPath.c_str(), ok ? "inserted" : "NOT a floppy image");
        if (!ok) return 2;
    }
    if (hdTrace > 0) mac.traceHdRequests(hdTrace);
    mac.traceGuestCalls = guestCalls;
    mac.diskDriverOverScsi = driverOverScsi;
    if (netAttach) mac.attachNet(true);
    if (!hdPath.empty()) {
        std::vector<u8> hd = tracetools::loadFile(hdPath.c_str());
        if (hd.empty()) { std::fprintf(stderr, "cannot read disk: %s\n", hdPath.c_str()); return 2; }
        std::printf("hard disk %s: attached (%zu KB)\n", hdPath.c_str(), hd.size() >> 10);
        mac.insertHardDisk(std::move(hd));
    }
    if (!hd2Path.empty()) {
        std::vector<u8> hd = tracetools::loadFile(hd2Path.c_str());
        if (hd.empty()) { std::fprintf(stderr, "cannot read disk: %s\n", hd2Path.c_str()); return 2; }
        std::printf("second hard disk %s: attached (%zu KB)%s\n", hd2Path.c_str(), hd.size() >> 10,
                    hd2ReadOnly ? ", read-only" : "");
        mac.insertHardDisk2(std::move(hd), hd2ReadOnly);
    }
    if (!dropBoxDir.empty()) {
        std::string why;
        std::vector<u8> volume = tracetools::buildFolderVolume(dropBoxDir, why);
        if (volume.empty()) { std::fprintf(stderr, "dropbox: build FAILED: %s\n", why.c_str()); return 2; }
        mac.insertHardDisk2(std::move(volume), false);
        std::printf("dropbox: %s built and attached\n", dropBoxDir.c_str());
    }
    if (!cdPath.empty()) {
        std::vector<u8> cd = tracetools::loadFile(cdPath.c_str());
        if (cd.empty()) { std::fprintf(stderr, "cannot read disc: %s\n", cdPath.c_str()); return 2; }
        mac.attachCdRom(true, 3);
        const bool ok = mac.insertCd(std::move(cd)) != 0;
        std::printf("cd %s: %s\n", cdPath.c_str(), ok ? "in the drive" : "REFUSED");
        if (!ok) return 2;
    }

    // The verdict of the start-up tests, read at the ROM's own two exits.
    bool testFailed = false, chimeReached = false;
    u32 failedTest = 0, failedBits = 0;
    int failedFrame = 0, chimeFrame = 0;
    mac.cpu().onStep = [&](u32 pc) {
        if (profileFrom >= 0 && static_cast<int>(lastFrame) >= profileFrom && static_cast<int>(lastFrame) < profileTo)
            ++profile[pc >> 4];
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

    if (trapsFrom >= 0) {
        mac.cpu().onTrap = [&](u16 opcode, u32 pc) {
            if (static_cast<int>(lastFrame) < trapsFrom || trapsLeft <= 0) return;
            --trapsLeft;
            std::printf("trap %04X at %08X: D0=%08X A0=%08X A1=%08X SR=%04X\n", opcode, pc, mac.cpu().d[0], mac.cpu().a[0],
                        mac.cpu().a[1], mac.cpu().getSR());
        };
    }

    std::vector<u32> px(static_cast<std::size_t>(mac.screenWidth()) * static_cast<std::size_t>(mac.screenHeight()));
    std::map<u32, int> hashes;   // picture -> first frame seen
    u32 lastHash = 0;            // the frame before's
    std::vector<u8> audio, audioChunk, stereo, stereoChunk;
    int ejectsSeen = 0;
    bool disk2PutAwaySeen = false;
    for (int f = 0; f < frames; ++f) {
        lastFrame = static_cast<u32>(f);
        if (hdOutAtFrame == f) {
            const std::vector<u8>& image = mac.hardDiskImage();
            std::ofstream out(hdOutAt, std::ios::binary);
            out.write(reinterpret_cast<const char*>(image.data()), static_cast<std::streamsize>(image.size()));
            std::printf("frame %d: hard disk saved: %s (%zu bytes)\n", f, hdOutAt.c_str(), image.size());
        }
        if (ejectCdAt == f) {
            mac.ejectCd();
            std::printf("cd eject asked for at frame %d\n", f);
        }
        if (putAwayDisk2At >= 0 && f >= putAwayDisk2At && mac.guestPutAwaySecondDisk()) {
            std::printf("second disk: the guest unmounted its volume at frame %d\n", f);
            putAwayDisk2At = -1;
        }
        if (!disk2PutAwaySeen && mac.secondDiskPutAway()) {
            disk2PutAwaySeen = true;
            std::printf("second disk: put away by the Macintosh, seen at frame %d\n", f);
        }
        if (guestRestartAt >= 0 && f >= guestRestartAt && mac.guestRestart()) {
            std::printf("guest restart at frame %d\n", f);
            guestRestartAt = -1;
        }
        if (guestPowerOffAt >= 0 && f >= guestPowerOffAt && mac.guestPowerOff()) {
            std::printf("guest asked to switch off at frame %d\n", f);
            guestPowerOffAt = -1;
        }
        if (!beeps.empty() && f >= beeps.front() && mac.systemBeep()) {
            std::printf("alert sound asked for at frame %d\n", f);
            beeps.erase(beeps.begin());
        }
        for (const MouseEvent& e : mouse) if (e.frame == f) mac.mouseEvent(e.dx, e.dy, e.button);
        for (const KeyEvent& e : keys) if (e.frame == f) mac.keyEvent(e.code, e.down);
        for (const auto& [at, file] : floppyEvents) {
            if (at != f) continue;
            if (file.empty()) {
                mac.ejectFloppy();
                std::printf("frame %d: floppy taken out by the host\n", f);
            } else {
                const bool ok = mac.insertFloppy(tracetools::loadFile(file.c_str()), !floppyWritable);
                std::printf("frame %d: floppy %s %s\n", f, file.c_str(), ok ? "inserted" : "NOT a floppy image");
            }
        }
        if (static_cast<int>(mac.floppyEjects()) != ejectsSeen) {
            ejectsSeen = static_cast<int>(mac.floppyEjects());
            std::printf("frame %d: the Macintosh ejected the floppy\n", f);
        }
        mac.runFrame();
        if (mac.cpu().halted) break;
        // The hook costs a call per instruction; both exits are passed in
        // the first frames.
        if ((chimeReached || testFailed) && (profileFrom < 0 || f >= profileTo) && mac.cpu().onStep)
            mac.cpu().onStep = nullptr;
        mac.drainAudio(audioChunk);
        audio.insert(audio.end(), audioChunk.begin(), audioChunk.end());
        mac.drainAudioStereo(stereoChunk);
        stereo.insert(stereo.end(), stereoChunk.begin(), stereoChunk.end());
        if (hashFrom >= 0 && f + 1 >= hashFrom) {
            mac.renderScreen(px.data());
            const u32 h = pictureHash(px);
            if (f >= hashFrom && (f - hashFrom) % hashStep == 0 && h == lastHash) hashes.emplace(h, f);
            lastHash = h;
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
    {
        // The speaker: every stretch in which the signal moves. A level that
        // is merely held (the chip keeps its last sample when its buffers run
        // empty) is silence to a speaker behind a capacitor, whatever it is.
        constexpr double rate = 22254.5;
        constexpr std::size_t gap = 4451;            // 0.2 s without movement ends a stretch
        std::printf("sound: %zu samples in %u frames\n", audio.size(), lastFrame + 1);
        std::size_t start = 0, last = 0;
        int low = 255, high = 0, shown = 0;
        bool in = false;
        const auto close = [&] {
            if (!in) return;
            in = false;
            if (++shown > 12) return;
            std::printf("sound heard: %.2f to %.2f s, %zu samples, swing %d of 255\n",
                        static_cast<double>(start) / rate, static_cast<double>(last + 1) / rate, last - start + 1,
                        high - low);
        };
        for (std::size_t k = 1; k < audio.size(); ++k) {
            if (audio[k] == audio[k - 1]) {
                if (in && k - last > gap) close();
                continue;
            }
            if (!in) { in = true; start = k - 1; low = high = audio[k - 1]; }
            last = k;
            low = std::min<int>(low, audio[k]);
            high = std::max<int>(high, audio[k]);
        }
        close();
        if (shown > 12) std::printf("sound heard: %d stretches in all\n", shown);
        // The jack: does each channel move, and are they one signal or two?
        std::size_t leftMoves = 0, rightMoves = 0, differ = 0;
        for (std::size_t k = 2; k + 1 < stereo.size(); k += 2) {
            if (stereo[k] != stereo[k - 2]) ++leftMoves;
            if (stereo[k + 1] != stereo[k - 1]) ++rightMoves;
            if (stereo[k] != stereo[k + 1]) ++differ;
        }
        std::printf("sound, stereo jack: %zu sample pairs, left moving in %zu, right in %zu, the two differ in %zu\n",
                    stereo.size() / 2, leftMoves, rightMoves, differ);
    }
    if (up) {
        // The clock chip's seconds against the System's own (Time, $020C),
        // and the one-second pulses the System let go by.
        std::printf("clock: the System's Time is %u s, %.1f s of machine time have passed", mac.read32(0x020C),
                    static_cast<double>(mac.totalCycles()) / 15667200.0);
        std::printf("; %u one-second pulses went unanswered", board.secondsUnanswered);
        for (u32 k = 0; k < board.secondsUnanswered && k < board.unansweredAtFrame.size(); ++k)
            std::printf("%s%u", k ? ", " : " (frames ", board.unansweredAtFrame[k]);
        std::printf("%s\n", board.secondsUnanswered ? ")" : "");
    }
    if (!wavPath.empty()) {
        // PCM, 8 bits unsigned, two channels, at the machine's 22.25 kHz.
        std::ofstream wav(wavPath, std::ios::binary);
        const u32 n = static_cast<u32>(stereo.size()), wavRate = 22255;
        const auto le32 = [&](u32 v) { for (int k = 0; k < 4; ++k) wav.put(static_cast<char>(v >> (8 * k))); };
        const auto le16 = [&](u32 v) { for (int k = 0; k < 2; ++k) wav.put(static_cast<char>(v >> (8 * k))); };
        wav.write("RIFF", 4); le32(36 + n); wav.write("WAVEfmt ", 8); le32(16); le16(1); le16(2);
        le32(wavRate); le32(wavRate * 2); le16(2); le16(8); wav.write("data", 4); le32(n);
        wav.write(reinterpret_cast<const char*>(stereo.data()), static_cast<std::streamsize>(n));
        std::printf("sound saved: %s (%u sample pairs)\n", wavPath.c_str(), n / 2);
    }
    if (up) {
        // What the guest believes about its screen (ScrnBase $0824, ScreenRow
        // $0106), its pointer (Mouse $0830: vertical, horizontal; MBState
        // $0172, bit 7 clear = down), its keys ($0174) and its clock ($016A).
        std::printf("guest: ScrnBase=%08X ScreenRow=%u, pointer at %d,%d button %s, KeyMap %08X %08X, Ticks %u\n",
                    mac.read32(0x0824), mac.read16(0x0106), static_cast<s16>(mac.read16(0x0832)),
                    static_cast<s16>(mac.read16(0x0830)), (mac.read8(0x0172) & 0x80) ? "up" : "down",
                    mac.read32(0x0174), mac.read32(0x0178), mac.read32(0x016A));
    }
    {
        const Se30Machine::FloppyCounts fc = mac.floppyCounts();
        std::printf("floppy: %s, %llu bytes read, %llu written, %u tracks put back, head on track %d, chip in %s mode, %u ejects by the Macintosh\n",
                    mac.floppyPresent() ? "disk in the drive" : "drive empty", static_cast<unsigned long long>(fc.bytesRead),
                    static_cast<unsigned long long>(fc.bytesWritten), fc.tracksWritten, fc.track,
                    fc.secondRegisterSet ? "ISM" : "IWM", mac.floppyEjects());
    }
    if (up) {
        // What the guest has running and mounted: the current application's
        // name (CurApName $0910) and the volume names down the VCB queue ($0356).
        const auto pascal = [&](u32 at, u32 limit) {
            std::string text;
            try {
                const u32 n = mac.read8(at);
                if (n > limit) return text;
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
            u32 vcb = mac.read32(0x0358) & 0x00FFFFFFu;
            for (int n = 0; n < 8 && vcb != 0 && vcb < mac.ramBytes(); ++n) {
                volumes += (volumes.empty() ? "" : ", ") + pascal(vcb + 44, 27);
                vcb = mac.read32(vcb) & 0x00FFFFFFu;
            }
        } catch (const BusFault&) {
        }
        std::printf("guest: application '%s', volumes [%s]\n", pascal(0x0910, 31).c_str(), volumes.c_str());
    }
    {
        const Se30Machine::ScsiCounts sc = mac.scsiCounts();
        std::printf("scsi: %u selections, %u commands, %u bytes in, %u out; data cycles: %llu plain, %llu with handshake, "
                    "%u ended by the watchdog; driver served %u reads, %u writes\n",
                    sc.selections, sc.commands, sc.bytesIn, sc.bytesOut,
                    static_cast<unsigned long long>(sc.plainDataCycles),
                    static_cast<unsigned long long>(sc.handshakeDataCycles), sc.handshakeTimeouts, mac.hdReads(),
                    mac.hdWrites());
        std::printf("scsi commands by ID:");
        for (std::size_t id = 0; id < sc.commandsById.size(); ++id) std::printf(" %zu=%u", id, sc.commandsById[id]);
        std::printf("\n");
    }
    std::printf("end: frame %llu pc=%08X sr=%04X overlay=%d%s\n", static_cast<unsigned long long>(mac.frameCount()),
                mac.cpu().pc, mac.cpu().getSR(), mac.overlayActive() ? 1 : 0, mac.cpu().halted ? " HALTED" : "");
    if (report) std::fputs(mac.diagnosticReport().c_str(), stdout);
    if (hashFrom >= 0) {
        std::printf("pictures from frame %d, every %d: %zu distinct:", hashFrom, hashStep, hashes.size());
        for (const auto& [h, f] : hashes) std::printf(" %08X@%d", h, f);
        std::printf("\n");
    }
    if (!profile.empty()) {
        std::vector<std::pair<u64, u32>> places;
        u64 total = 0;
        for (const auto& [place, count] : profile) { places.emplace_back(count, place); total += count; }
        std::sort(places.rbegin(), places.rend());
        std::printf("profile, frames %d to %d, %llu instructions:\n", profileFrom, profileTo,
                    static_cast<unsigned long long>(total));
        for (std::size_t k = 0; k < places.size() && k < 20; ++k)
            std::printf("   %08X  %5.1f%%\n", places[k].second << 4,
                        100.0 * static_cast<double>(places[k].first) / static_cast<double>(total));
    }
    for (const auto& [from, count] : listings) {
        u32 at = from;
        for (int k = 0; k < count; ++k) {
            std::string text;
            int len = 2;
            try {
                len = dbg::disasm([&](u32 a) { return mac.read16(a); }, at, text);
            } catch (const BusFault&) {
                text = "(bus error)";
            }
            std::printf("%08X  %s\n", at, text.c_str());
            at += static_cast<u32>(len > 0 ? len : 2);
        }
    }
    if (askGestalt) {
        // What the System itself answers: machine type, System version, CPU, memory.
        for (const char* code : {"mach", "sysv", "proc", "fpu ", "ram ", "lram", "addr"}) {
            const u32 selector = (static_cast<u32>(static_cast<u8>(code[0])) << 24) |
                                 (static_cast<u32>(static_cast<u8>(code[1])) << 16) |
                                 (static_cast<u32>(static_cast<u8>(code[2])) << 8) | static_cast<u8>(code[3]);
            u32 response = 0;
            const s32 err = mac.gestaltQuery(selector, response);
            std::printf("gestalt '%s' -> err %d resp %08X (%u)\n", code, err, response, response);
        }
    }
    if (showDrivers) tracetools::printDrivers(mac, mac.ramBytes());
    if (!dropBoxGuestRead.empty()) tracetools::reportGuestRead(mac, dropBoxGuestRead);
    if (doShutdown) std::printf("shutdown volumes: %s\n", mac.shutdownVolumes() ? "unmounted" : "nothing to do");
    if (!hdOut.empty()) {
        const std::vector<u8>& image = mac.hardDiskImage();
        std::ofstream out(hdOut, std::ios::binary);
        out.write(reinterpret_cast<const char*>(image.data()), static_cast<std::streamsize>(image.size()));
        std::printf("hard disk saved: %s (%zu bytes)\n", hdOut.c_str(), image.size());
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
    if (!floppyOut.empty()) {
        const std::vector<u8> file = mac.floppyForWriteBack();
        std::ofstream out(floppyOut, std::ios::binary);
        out.write(reinterpret_cast<const char*>(file.data()), static_cast<std::streamsize>(file.size()));
        std::printf("floppy written to %s (%zu bytes)\n", floppyOut.c_str(), file.size());
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
