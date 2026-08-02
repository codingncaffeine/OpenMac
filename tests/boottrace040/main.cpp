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
    // ...and the volume must say WHICH folder holds that System. drFndrInfo[0]
    // (MDB + 92) is the blessed System Folder's directory id: the ROM's boot
    // scan reads it to find the System file, and a volume that leaves it zero
    // is a disk with no startup system as far as the scan is concerned -- the
    // flashing question mark, however complete the copy underneath.
    const std::size_t mdb = 1024;
    out[mdb + 92] = static_cast<u8>(dstSys >> 24);
    out[mdb + 93] = static_cast<u8>(dstSys >> 16);
    out[mdb + 94] = static_cast<u8>(dstSys >> 8);
    out[mdb + 95] = static_cast<u8>(dstSys);
    std::printf("blessed System Folder: dir id %u\n", dstSys);

    std::ofstream f(outPath, std::ios::binary);
    f.write(reinterpret_cast<const char*>(out.data()),
            static_cast<std::streamsize>(out.size()));
    std::printf("wrote %s (%zu bytes)\n", outPath, out.size());
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    const char* romPath = nullptr;
    const char* fdPath = nullptr;
    int fdAfter = -1;               // --floppy-after: insert mid-run at this frame
    int jiggleAt = -1;              // --jiggle-at: click+move the mouse mid-boot
    unsigned long watchMemAt = 0;   // --watch-mem: log writes to this address
    unsigned long countPcAt = 0;    // --count-pc: count executions of this pc
    std::vector<std::pair<int, int>> clicks;   // --click X Y, in order
    int postFrames = 0;             // --post-frames: run on after the clicks
    bool trapRingOn = false, trapRingArmed = false;   // --trap-ring
    int trapRingAfter = 0;          // --trap-ring-after: post-frame to arm at
    int trapRingFrom = -1;          // --trap-ring-from: BOOT frame to arm at
    bool ringFilesOnly = false;     // --ring-files-only: skip the resource spam
    bool ringNull = false;          // --ring-null: fetch+match, record nothing
    bool noAnnounce = false;        // --no-announce: no injected HD mount
    const char* saveHdPath = nullptr;   // --save-hd: dump the disk at exit
    // kind 0 = File Manager (result from the param block's ioResult), 1 =
    // Resource Manager (result from the ResErr global), 2 = Gestalt (selector
    // in d0), 3 = SCSIDispatch (selector from the stack). nm carries the file
    // name / resource type+id the call named, so the ring reads as a story.
    struct TrapRec {
        u16 op; u32 pc, d0, a0; s16 res;
        u8 kind; s16 vref; u32 dirid; s16 idx; char nm[28]; u32 frame;
    };
    std::vector<TrapRec> trapRing(512);
    std::size_t trapRingPos = 0;
    bool askGestalt = false;        // --gestalt: query the guest at exit
    const char* trapLogPath = nullptr;   // --trap-log: every ringed trap, to a file
    std::ofstream trapLog;
    int shotEvery = 0;              // --shot-every: screen strip during post-frames
    // --floppy-next (repeatable): the rest of a disk set. When the guest
    // ejects the current disk, the next one goes in after a settle pause --
    // the hands that feed an installer asking for disk 2 of 7.
    std::vector<const char*> floppyQueue;
    int floppySettle = 0;
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
    int breakFlush = 0;             // break at the Nth FIFO flush after CDB #8
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
        if (a == "--make-blank" && i + 3 < argc) {
            const char* outP = argv[++i];
            const u32 mb = static_cast<u32>(std::atoi(argv[++i]));
            const char* volName = argv[++i];
            auto img = openmac::hfs::formatVolume(mb * 1024u * 1024u, volName);
            if (img.empty()) { std::fprintf(stderr, "format failed\n"); return 2; }
            std::ofstream f(outP, std::ios::binary);
            f.write(reinterpret_cast<const char*>(img.data()),
                    static_cast<std::streamsize>(img.size()));
            std::printf("wrote %s (%zu bytes, blank HFS \"%s\")\n", outP, img.size(), volName);
            return 0;
        }
        if (a == "--rom" && i + 1 < argc) romPath = argv[++i];
        else if (a == "--floppy" && i + 1 < argc) fdPath = argv[++i];
        else if (a == "--floppy-after" && i + 2 < argc) { fdPath = argv[++i]; fdAfter = std::atoi(argv[++i]); }
        else if (a == "--jiggle-at" && i + 1 < argc) jiggleAt = std::atoi(argv[++i]);
        else if (a == "--watch-mem" && i + 1 < argc) watchMemAt = std::strtoul(argv[++i], nullptr, 16);
        else if (a == "--count-pc" && i + 1 < argc) countPcAt = std::strtoul(argv[++i], nullptr, 16);
        else if (a == "--post-frames" && i + 1 < argc) postFrames = std::atoi(argv[++i]);
        else if (a == "--trap-ring") trapRingArmed = true;
        else if (a == "--trap-ring-after" && i + 1 < argc) trapRingAfter = std::atoi(argv[++i]);
        else if (a == "--trap-ring-from" && i + 1 < argc) trapRingFrom = std::atoi(argv[++i]);
        else if (a == "--ring-files-only") ringFilesOnly = true;
        else if (a == "--ring-null") ringNull = true;
        else if (a == "--no-announce") noAnnounce = true;
        else if (a == "--save-hd" && i + 1 < argc) saveHdPath = argv[++i];
        else if (a == "--click" && i + 2 < argc) {
            const int cx = std::atoi(argv[++i]);
            clicks.emplace_back(cx, std::atoi(argv[++i]));
        }
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
        else if (a == "--gestalt") askGestalt = true;
        else if (a == "--trap-log" && i + 1 < argc) trapLogPath = argv[++i];
        else if (a == "--shot-every" && i + 1 < argc) shotEvery = std::atoi(argv[++i]);
        else if (a == "--floppy-next" && i + 1 < argc) floppyQueue.push_back(argv[++i]);
        else if (a == "--input-test") inputTest = true;
        else if (a == "--break-flush" && i + 1 < argc) breakFlush = std::atoi(argv[++i]);
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

    auto trapName = [](u16 op) -> const char* {
        switch (op & 0xFDFF) {   // fold the H variants onto the base names
        case 0xA000: return "Open";        case 0xA001: return "Close";
        case 0xA002: return "Read";        case 0xA003: return "Write";
        case 0xA004: return "Control";     case 0xA005: return "Status";
        case 0xA007: return "GetVolInfo";  case 0xA008: return "Create";
        case 0xA009: return "Delete";      case 0xA00A: return "OpenRF";
        case 0xA00B: return "Rename";      case 0xA00C: return "GetFileInfo";
        case 0xA00D: return "SetFileInfo"; case 0xA00E: return "UnmountVol";
        case 0xA00F: return "MountVol";    case 0xA010: return "Allocate";
        case 0xA011: return "GetEOF";      case 0xA012: return "SetEOF";
        case 0xA013: return "FlushVol";    case 0xA014: return "GetVol";
        case 0xA015: return "SetVol";      case 0xA017: return "Eject";
        case 0xA018: return "GetFPos";     case 0xA035: return "Offline";
        case 0xA044: return "SetFPos";
        default: break;
        }
        switch (op) {
        case 0xA060: return "FSDispatch";  case 0xA260: return "HFSDispatch";
        case 0xA80C: return "RGetResource";
        case 0xA80D: return "Count1Resources";
        case 0xA80E: return "Get1IndResource";
        case 0xA80F: return "Get1IndType"; case 0xA810: return "Unique1ID";
        case 0xA81A: return "HOpenResFile";
        case 0xA81B: return "HCreateResFile";
        case 0xA81C: return "Count1Types";
        case 0xA81F: return "Get1Resource";
        case 0xA9C4: return "OpenRFPerm";
        case 0xA997: return "OpenResFile"; case 0xA998: return "UseResFile";
        case 0xA999: return "UpdateResFile";
        case 0xA99A: return "CloseResFile";
        case 0xA99B: return "SetResLoad";
        case 0xA99C: return "CountResources";
        case 0xA99D: return "GetIndResource";
        case 0xA9A0: return "GetResource";
        case 0xA9A1: return "GetNamedRes"; case 0xA9A2: return "LoadResource";
        case 0xA9A3: return "ReleaseRes";  case 0xA9A4: return "HomeResFile";
        case 0xA9A5: return "SizeRsrc";    case 0xA9A6: return "GetResAttrs";
        case 0xA9A8: return "GetResInfo";  case 0xA9AA: return "ChangedRes";
        case 0xA9AB: return "AddResource"; case 0xA9AD: return "RmveResource";
        case 0xA9AF: return "ResError";    case 0xA9B0: return "WriteResource";
        case 0xA9B1: return "CreateResFile";
        case 0xA1AD: return "Gestalt";     case 0xA815: return "SCSIDispatch";
        default: return "";
        }
    };
    // HFSDispatch routes a family of calls through one trap; the selector in
    // D0 says which one actually ran.
    auto hfsSel = [](u32 d0) -> const char* {
        switch (d0 & 0xFF) {
        case 1: return "OpenWD";      case 2: return "CloseWD";
        case 5: return "CatMove";     case 6: return "DirCreate";
        case 7: return "GetWDInfo";   case 8: return "GetFCBInfo";
        case 9: return "GetCatInfo";  case 10: return "SetCatInfo";
        case 11: return "SetVolInfo"; case 26: return "LockRng";
        case 27: return "UnlockRng";  default: return "?";
        }
    };
    auto fmtTrap = [&](const TrapRec& t, char* out, std::size_t n) {
        char what[40];
        if (t.op == 0xA260 || t.op == 0xA060)
            std::snprintf(what, sizeof what, "%s.%s", trapName(t.op), hfsSel(t.d0));
        else
            std::snprintf(what, sizeof what, "%s", trapName(t.op));
        char detail[64] = "";
        if (t.kind == 0)
            std::snprintf(detail, sizeof detail, "vref=%d dir=%u idx=%d",
                          t.vref, t.dirid, t.idx);
        std::snprintf(out, n,
                      "f=%u %04X %-22s pc=%08X a0=%08X d0=%08X %-26s \"%s\" -> %d%s",
                      t.frame, t.op, what, t.pc, t.a0, t.d0, detail, t.nm, t.res,
                      t.res < 0 ? "   <-- ERROR" : "");
    };
    if (trapLogPath) trapLog.open(trapLogPath);

    auto rom = loadFile(romPath);
    if (rom.empty()) {
        std::fprintf(stderr, "cannot read ROM: %s\n", romPath);
        return 2;
    }

    QuadraMachine::Config cfg;
    cfg.ramSize = static_cast<u32>(ramMb) * 1024u * 1024u;
    QuadraMachine mac(std::move(rom), cfg);
    int cdbCount = 0;
    int flushCount = 0;
    mac.onDiag = [&](const char* m) {
        if (m[0] == 'C' && m[1] == 'D' && m[2] == 'B') ++cdbCount;
        // The System's mount reads the volume's MDB (LBA 7 on our layout).
        // Arm the data-in PC trace exactly there, so the log shows which ROM
        // routine takes the bytes -- the real reader or the discard drain.
        if (std::strncmp(m, "CDB id0: 08 00 00 07", 20) == 0) mac.armDataInTrace(24);
        if (cdbCount >= 8 && std::strncmp(m, "SCMD 01", 7) == 0) ++flushCount;
        // Without --no-log everything prints; with it, the register-level
        // SCSI chatter (millions of lines on a long run) stays quiet and
        // the structural lines still land.
        if (!showLog && (std::strncmp(m, "SCMD", 4) == 0 || std::strncmp(m, "SREG", 4) == 0 ||
                         std::strncmp(m, "SRD", 3) == 0 || std::strncmp(m, "SEL ", 4) == 0 ||
                         std::strncmp(m, "VIFR", 4) == 0 || std::strncmp(m, "DRQ", 3) == 0 ||
                         std::strncmp(m, "SDMA", 4) == 0 || std::strncmp(m, "SWIM", 4) == 0 ||
                         std::strncmp(m, "XFER", 4) == 0 || std::strncmp(m, "PRIME", 5) == 0 ||
                         std::strncmp(m, "EASC", 4) == 0 || std::strncmp(m, "VIA2", 4) == 0))
            return;
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
        // Keep the tail of the trap stream. When an application gives up with
        // a message of its own, the call it gave up on is a few entries from
        // the end -- and the register it passed says which file or volume.
        // Cheap gate first -- only traps issued from RAM (the System and the
        // application, not the ROM) -- then one 16-bit fetch instead of two
        // 8-bit ones. The guest issues these in the millions, so the cost of
        // looking has to stay well under the cost of running.
        if (vector == 10 && trapRingOn) {
            // The machine's read IS the live bus: an address outside RAM/ROM
            // THROWS a bus fault -- from inside this hook that unwinds
            // through the CPU's own exception dispatch and wrecks the guest
            // (measured: a SCSI retry storm where the clean run sailed).
            // Fetch the opcode only from ranges that cannot fault or mutate.
            const u32 ringRamTop = static_cast<u32>(ramMb) * 1024u * 1024u;
            const bool pcSafe = (pc + 1 < ringRamTop) ||
                                (pc >= 0x40000000u && pc < 0x50000000u && !mac.overlayActive());
            const u16 op = pcSafe ? mac.read16(pc) : 0;
            // The File Manager's own calls. $A000-$A0FF also holds the
            // Memory Manager, and its idle traffic (HPurge, GetHandleSize,
            // OSEventAvail) floods the ring, pushing the interesting tail out
            // before anyone can read it.
            // A018 is the top of the File Manager block: A01B-A01F are the
            // Memory Manager, whose A0 is not a parameter block -- reading
            // "ioResult" through it follows a garbage pointer, and a read
            // that lands in device space MUTATES the machine (FIFO pops,
            // read-to-clear latches). The instrument must observe only.
            const bool fileTrap =
                (op >= 0xA000 && op <= 0xA018) || op == 0xA044 || op == 0xA060 ||
                (op >= 0xA200 && op <= 0xA218) || op == 0xA260 || op == 0xA035;
            // The Resource Manager: an installer reads its script and every
            // file it copies through here, and a failure surfaces as ResError,
            // not as an ioResult. The one-deep (Get1*) family and the open
            // calls live in $A80C-$A81F and $A9C4, outside the classic block.
            const bool resTrap = !ringFilesOnly &&
                                 ((op >= 0xA997 && op <= 0xA9B1) ||
                                  (op >= 0xA80C && op <= 0xA810) ||
                                  (op >= 0xA81A && op <= 0xA81C) ||
                                  op == 0xA81F || op == 0xA9C4);
            const bool gesTrap = op == 0xA1AD;    // selector rides in D0
            const bool scsiTrap = op == 0xA815;   // selector rides on the stack
            if (ringNull) {
                // Bisecting the observer: everything below this line is
                // skipped, so a behavior difference that survives --ring-null
                // lives in the fetch above, and one that disappears lives in
                // the recording.
            } else if (fileTrap || resTrap || gesTrap || scsiTrap) {
                // The call before this one has finished by now: a File Manager
                // call's parameter block carries its ioResult (+16), and a
                // Resource Manager call has left its verdict in the ResErr
                // global ($0A60). That is the answer the application acted
                // on -- and the reason it gave up.
                // Reads made by the instrument stay inside guest RAM: a
                // stale or garbage pointer must never turn into a device
                // access with side effects. Flag bits ride the high byte of
                // Memory Manager pointers below 16 MB, so strip them only
                // when the raw address is not already inside RAM (the
                // machine's own guestPtr rule).
                const u32 ramTop = static_cast<u32>(ramMb) * 1024u * 1024u;
                auto guestRam = [&](u32 a) -> u32 {
                    if (a < ramTop) return a;
                    a &= 0x00FFFFFFu;
                    return a < ramTop ? a : 0xFFFFFFFFu;
                };
                auto peek8 = [&](u32 a) -> u8 {
                    a = guestRam(a);
                    return a != 0xFFFFFFFFu ? mac.read8(a) : 0;
                };
                auto peek16 = [&](u32 a) -> u16 {
                    a = guestRam(a);
                    return a != 0xFFFFFFFFu && a + 1 < ramTop ? mac.read16(a) : 0;
                };
                auto peek32 = [&](u32 a) -> u32 {
                    a = guestRam(a);
                    return a != 0xFFFFFFFFu && a + 3 < ramTop ? mac.read32(a) : 0;
                };
                if (trapRingPos > 0) {
                    auto& prev = trapRing[(trapRingPos - 1) % trapRing.size()];
                    if (prev.kind == 0)
                        prev.res = static_cast<s16>(peek16(prev.a0 + 16));
                    else if (prev.kind == 1)
                        prev.res = static_cast<s16>(peek16(0x0A60));
                    // The record is final once its result is known; a file log
                    // gets every one, not just whatever tail survives the ring.
                    if (trapLog.is_open()) {
                        char line[220];
                        fmtTrap(prev, line, sizeof line);
                        trapLog << line << '\n';
                    }
                }
                auto pch = [](u32 c) {
                    c &= 0xFF;
                    return (c >= 32 && c < 127) ? static_cast<char>(c) : '.';
                };
                TrapRec r{};
                r.op = op; r.pc = pc; r.d0 = mac.cpu().d[0]; r.a0 = mac.cpu().a[0];
                r.frame = static_cast<u32>(mac.frameCount());
                if (fileTrap) {
                    r.kind = 0;
                    const u32 pb = r.a0;
                    r.vref = static_cast<s16>(peek16(pb + 22));
                    const u32 np = peek32(pb + 18);   // ioNamePtr
                    if (np && guestRam(np) != 0xFFFFFFFFu) {
                        const int len = peek8(np);
                        for (int k = 0; k < len && k < 27; ++k)
                            r.nm[k] = pch(peek8(np + 1 + static_cast<u32>(k)));
                    }
                    if (op == 0xA260 || op == 0xA060) {
                        r.dirid = peek32(pb + 48);    // ioDirID
                        r.idx = static_cast<s16>(peek16(pb + 28));   // ioFDirIndex
                    } else if (op == 0xA207) {
                        r.idx = static_cast<s16>(peek16(pb + 28));   // ioVolIndex
                    } else if (op == 0xA004 || op == 0xA005 ||
                               op == 0xA204 || op == 0xA205) {
                        r.idx = static_cast<s16>(peek16(pb + 26));   // csCode
                        r.dirid = peek16(pb + 24);                   // ioCRefNum
                    }
                } else if (resTrap) {
                    r.kind = 1;
                    // Stack-based Toolbox call: for the GetResource shape the
                    // ID word sits on top with the type long above it.
                    const u32 sp = mac.cpu().a[7];
                    const u32 ty = peek32(sp + 2);
                    std::snprintf(r.nm, sizeof r.nm, "'%c%c%c%c' %d",
                                  pch(ty >> 24), pch(ty >> 16), pch(ty >> 8),
                                  pch(ty),
                                  static_cast<s16>(peek16(sp)));
                } else if (gesTrap) {
                    r.kind = 2;
                    const u32 ty = r.d0;
                    std::snprintf(r.nm, sizeof r.nm, "'%c%c%c%c'",
                                  pch(ty >> 24), pch(ty >> 16), pch(ty >> 8),
                                  pch(ty));
                } else {
                    r.kind = 3;
                    r.idx = static_cast<s16>(peek16(mac.cpu().a[7]));
                }
                trapRing[trapRingPos % trapRing.size()] = r;
                ++trapRingPos;
            }
        }
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
        mac.suppressHdAnnounce = noAnnounce;
        mac.insertHardDisk(std::move(hd));
        std::printf("hd: attached\n");
    }
    std::vector<u8> fdDeferred;
    if (fdPath) {
        auto fd = loadFile(fdPath);
        if (fd.empty()) {
            std::fprintf(stderr, "cannot read floppy: %s\n", fdPath);
            return 2;
        }
        if (fdAfter >= 0) {
            fdDeferred = std::move(fd);   // insert mid-run at frame fdAfter
        } else {
            std::printf("floppy: %s\n", mac.insertFloppy(std::move(fd)) ? "inserted" : "REFUSED");
        }
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

    if (breakFlush) {
        // Single-step to the Nth FIFO flush after the probe's CDB and show
        // the PC trail into it -- the flush count is chip-event ground truth,
        // so this lands in the exact routine that decided the transfer's fate.
        u32 ring[64] = {};
        int rp = 0;
        mac.cpu().onStep = [&](u32 pc) {
            ring[rp] = pc;
            rp = (rp + 1) % 64;
        };
        u64 guard = 6'000'000'000ull;
        while (flushCount < breakFlush && !mac.cpu().halted && guard--) {
            mac.stepInstruction();
        }
        mac.cpu().onStep = nullptr;
        std::printf("-- flush #%d reached at pc=%08X --\ntrail:", breakFlush, mac.cpu().pc);
        for (int i = 0; i < 64; ++i) std::printf(" %08X", ring[(rp + i) % 64]);
        std::printf("\n");
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
        if (i == trapRingFrom) trapRingOn = trapRingArmed;
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
        if (fdAfter >= 0 && i == fdAfter && !fdDeferred.empty()) {
            std::printf("floppy (mid-run, frame %d): %s\n", i,
                        mac.insertFloppy(std::move(fdDeferred)) ? "inserted" : "REFUSED");
        }
        // Simulate a user clicking to lock and jiggling the mouse over a span of
        // frames, to see whether injecting ADB traffic mid-boot wedges the machine.
        if (jiggleAt >= 0 && i >= jiggleAt && i < jiggleAt + 40) {
            if (i == jiggleAt) mac.mouseMove(0, 0, true);        // click (lock)
            if (i == jiggleAt + 2) mac.mouseMove(0, 0, false);
            mac.mouseMove((i & 1) ? 7 : -7, (i & 2) ? 5 : -5, false);
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
        // Isolate the two devices: move ONLY the mouse and see whether the
        // cursor tracks it, then press ONLY a key and see the keyboard's
        // effect. Reporting them apart tells a dead mouse from a dead
        // keyboard -- the two ride separate ADB delivery paths.
        const int w = mac.screenWidth(), h = mac.screenHeight();
        std::vector<u32> before(static_cast<std::size_t>(w) * h);
        std::vector<u32> afterMouse(before.size());
        std::vector<u32> afterKey(before.size());
        mac.renderScreen(before.data());
        const u32 mp0 = mac.adbMousePolls();
        // Low-mem mouse globals: RawMouse (ADB writes it), Mouse (the cursor
        // follows it). If RawMouse moves but the cursor does not, the ADB path
        // works and the cursor task is the problem; if neither moves, the
        // System never applied our report.
        const u16 mtV0 = mac.read16(0x0828), mtH0 = mac.read16(0x082A);
        const u16 rawV0 = mac.read16(0x082C), rawH0 = mac.read16(0x082E);
        const u16 msV0 = mac.read16(0x0830), msH0 = mac.read16(0x0832);
        mac.adbClearCmdTrace();
        mac.armAdbSrTrace(30);
        if (watchMemAt) mac.watchMem(static_cast<u32>(watchMemAt), 16, 60);
        if (countPcAt) mac.countPc(static_cast<u32>(countPcAt));
        std::printf("VBL plumbing: dafb vblEnabled=%d  via2 IFR=%02X IER=%02X\n",
                    mac.dafbVblEnabled() ? 1 : 0,
                    mac.read8(0x50F02000u + (13u << 9)),
                    mac.read8(0x50F02000u + (14u << 9)));
        {
            // The drive queue: whether the SCSI driver ever installed a drive.
            u32 e = mac.read32(0x030A);   // DrvQHdr.qHead
            std::printf("drive queue:");
            for (int n = 0; e && n < 8; ++n) {
                const u16 drive = mac.read16(e + 6);
                const u16 refNum = mac.read16(e + 8);
                std::printf(" [drive %u ref %d]", drive, static_cast<s16>(refNum));
                e = mac.read32(e);
            }
            std::printf("\n");
        }
        std::printf("DAFB swatch regs 100-13C:");
        for (int i = 0; i < 16; ++i) std::printf(" %03X", mac.dafbSwatchReg(i));
        std::printf("\n");
        {
            const u32 adbBase = mac.read32(0x0CF8);
            std::printf("ADB globals at %08X:\n", adbBase);
            for (int row = 0; row < 28; ++row) {
                std::printf("  +%03X:", row * 16);
                for (int i = 0; i < 16; ++i)
                    std::printf(" %02X", mac.read8(adbBase + static_cast<u32>(row * 16 + i)));
                std::printf("\n");
            }
        }
        for (int burst = 0; burst < 12; ++burst) {
            mac.mouseMove(10, 6, false);
            for (int f = 0; f < 4; ++f) mac.runFrame();
        }
        std::printf("mouse lowmem: MTemp %d,%d->%d,%d  RawMouse %d,%d->%d,%d  Mouse %d,%d->%d,%d\n",
                    static_cast<s16>(mtH0), static_cast<s16>(mtV0),
                    static_cast<s16>(mac.read16(0x082A)), static_cast<s16>(mac.read16(0x0828)),
                    static_cast<s16>(rawH0), static_cast<s16>(rawV0),
                    static_cast<s16>(mac.read16(0x082E)), static_cast<s16>(mac.read16(0x082C)),
                    static_cast<s16>(msH0), static_cast<s16>(msV0),
                    static_cast<s16>(mac.read16(0x0832)), static_cast<s16>(mac.read16(0x0830)));
        std::printf("mouse bytes delivered to guest:");
        for (u8 b : mac.adbMouseBytesLog()) std::printf(" %02X", b);
        std::printf("\nADB commands the CPU issued during the burst:");
        {
            const auto cmds = mac.adbCmdTrace();
            std::size_t shown = 0;
            for (u8 cb : cmds) { std::printf(" %02X", cb); if (++shown >= 48) break; }
            std::printf(" (%zu total)\n", cmds.size());
        }
        mac.renderScreen(afterMouse.data());
        const u32 kp0 = mac.adbKbdPolls();
        mac.keyEvent(0x00, true);    // 'A' down
        for (int f = 0; f < 6; ++f) mac.runFrame();
        mac.keyEvent(0x00, false);
        for (int f = 0; f < 20; ++f) mac.runFrame();
        mac.renderScreen(afterKey.data());
        std::size_t mouseDiff = 0, keyDiff = 0;
        for (std::size_t i = 0; i < before.size(); ++i) {
            if (before[i] != afterMouse[i]) ++mouseDiff;
            if (afterMouse[i] != afterKey[i]) ++keyDiff;
        }
        std::printf("input test: MOUSE moved %zu px (polls %u, reports %u, bytesRead %u), "
                    "KEYBOARD changed %zu px (polls %u)\n",
                    mouseDiff, mac.adbMousePolls() - mp0, mac.adbMouseReports(),
                    mac.adbMouseBytesRead(), keyDiff, mac.adbKbdPolls() - kp0);
        if (countPcAt)
            std::printf("count-pc %08lX: executed %u times during the test\n",
                        countPcAt, mac.countPcHits());
    }

    // Drive the guest's pointer to a screen position and click there. The
    // mouse is relative, and the System accelerates larger deltas, so this
    // closes the loop on the low-memory cursor position ($0830 = v,h) with
    // small steps rather than trying to compute one jump.
    // Put the pointer where we want it by writing the mouse globals the cursor
    // task reads, rather than nudging it there a few pixels per frame: walking
    // the cursor across the screen cost thousands of emulated frames per click
    // and the System's acceleration made it overshoot besides. MTemp, RawMouse
    // and Mouse all name the same position; CrsrNew asks for a redraw.
    auto moveTo = [&](int tx, int ty) {
        const u16 x = static_cast<u16>(tx), y = static_cast<u16>(ty);
        mac.write16(0x0828, y); mac.write16(0x082A, x);   // MTemp
        mac.write16(0x082C, y); mac.write16(0x082E, x);   // RawMouse
        mac.write16(0x0830, y); mac.write16(0x0832, x);   // Mouse
        mac.write8(0x08CE, 0xFF);                         // CrsrNew
        for (int f = 0; f < 3; ++f) mac.runFrame();
    };
    auto clickAt = [&](int x, int y) {
        moveTo(x, y);
        mac.mouseMove(0, 0, true);
        for (int f = 0; f < 5; ++f) mac.runFrame();
        mac.mouseMove(0, 0, false);
        for (int f = 0; f < 30; ++f) mac.runFrame();
        std::printf("clicked at %d,%d (cursor now %d,%d)\n", x, y,
                    static_cast<s16>(mac.read16(0x0832)),
                    static_cast<s16>(mac.read16(0x0830)));
    };
    for (const auto& pt : clicks) clickAt(pt.first, pt.second);
    // Only record from here on: the boot issues millions of traps and logging
    // them all costs more than the run itself. What matters is the tail.
    for (int f = 0; f < postFrames; ++f) {
        if (f == trapRingAfter) trapRingOn = trapRingArmed;
        mac.runFrame();
        // Feed the disk set: when the guest has ejected the current floppy,
        // give the mechanism a moment to settle and put the next disk in.
        if (!floppyQueue.empty()) {
            if (!mac.floppyPresent()) {
                if (++floppySettle >= 90) {
                    floppySettle = 0;
                    auto img = loadFile(floppyQueue.front());
                    std::printf("floppy swap (post-frame %d): %s -> %s\n", f,
                                floppyQueue.front(),
                                img.empty() ? "UNREADABLE"
                                : mac.insertFloppy(std::move(img)) ? "inserted"
                                                                   : "REFUSED");
                    floppyQueue.erase(floppyQueue.begin());
                }
            } else {
                floppySettle = 0;
            }
        }
        // A screen strip: what the guest was showing at each point of the
        // post-click run, so the trap log's frame stamps line up with what
        // the user would have been looking at.
        if (shotEvery > 0 && shotPath && f > 0 && f % shotEvery == 0) {
            char p[512];
            std::snprintf(p, sizeof p, "%s.f%05d.bmp", shotPath, f);
            const int w = mac.screenWidth(), h = mac.screenHeight();
            std::vector<u32> px(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
            mac.renderScreen(px.data());
            writeBmp(p, px, w, h);
        }
        if (mac.cpu().halted) { std::printf("HALTED post-click frame %d\n", f); break; }
    }

    {
        // Which volumes does the System actually have on line? The volume name
        // (Str27) sits at vcbVN +44 and the drive number at vcbDrvNum +72.
        u32 vcb = mac.read32(0x0358) & 0x00FFFFFFu;
        std::printf("hd requests served: %u reads, %u writes\n",
                    mac.hdReads(), mac.hdWrites());
        std::printf("floppy requests served: %u reads, %u writes\n",
                    mac.fdReads(), mac.fdWrites());
        if (trapRingArmed) {
            if (trapLog.is_open() && trapRingPos > 0) {
                // The newest record never saw a follow-up trap to backfill its
                // result; flush it as-is so the file ends where the run did.
                char line[220];
                fmtTrap(trapRing[(trapRingPos - 1) % trapRing.size()], line,
                        sizeof line);
                trapLog << line << '\n';
                trapLog.flush();
            }
            const std::size_t have = trapRingPos < trapRing.size() ? trapRingPos : trapRing.size();
            const std::size_t show = have < 140 ? have : 140;
            std::printf("-- last %zu of %zu traps --\n", show, trapRingPos);
            for (std::size_t k = show; k > 0; --k) {
                char line[220];
                fmtTrap(trapRing[(trapRingPos - k) % trapRing.size()], line,
                        sizeof line);
                std::printf("  %s\n", line);
            }
        }
        std::printf("mounted volumes:");
        for (int n = 0; vcb && n < 8; ++n) {
            char nm[32] = {0};
            const int len = mac.read8(vcb + 44);
            for (int k = 0; k < len && k < 27; ++k)
                nm[k] = static_cast<char>(mac.read8(vcb + 45 + static_cast<u32>(k)));
            std::printf(" [drive %d \"%s\"]", static_cast<s16>(mac.read16(vcb + 72)), nm);
            vcb = mac.read32(vcb) & 0x00FFFFFFu;
        }
        std::printf("\n");
        // The drive queue tells which drives the System believes exist -- a
        // phantom entry here becomes a phantom volume above. dsDiskInPlace
        // sits 3 bytes before the queue element inside the DrvSts record.
        u32 e = mac.read32(0x030A);   // DrvQHdr.qHead
        std::printf("drive queue:");
        for (int n = 0; e && n < 8; ++n) {
            std::printf(" [drive %u ref %d inPlace %d]", mac.read16(e + 6),
                        static_cast<s16>(mac.read16(e + 8)),
                        static_cast<s8>(mac.read8(e - 3)));
            e = mac.read32(e);
        }
        std::printf("\n");
    }

    if (askGestalt) {
        // Ask the guest's own Gestalt what machine it believes it is running
        // on. The installer picks its script rules off these answers.
        auto ask = [&](const char* code) {
            const u32 sel = (static_cast<u32>(static_cast<u8>(code[0])) << 24) |
                            (static_cast<u32>(static_cast<u8>(code[1])) << 16) |
                            (static_cast<u32>(static_cast<u8>(code[2])) << 8) |
                            static_cast<u32>(static_cast<u8>(code[3]));
            u32 resp = 0;
            const s32 err = mac.gestaltQuery(sel, resp);
            std::printf("gestalt '%s' -> err %d resp %08X (%u)\n", code, err,
                        resp, resp);
        };
        ask("mach"); ask("sysv"); ask("proc"); ask("fpu "); ask("mmu ");
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
    if (saveHdPath) {
        // Everything the guest wrote, so the result of an install can be
        // rebooted or inspected.
        const auto& img = mac.hardDiskImage();
        std::ofstream f(saveHdPath, std::ios::binary);
        f.write(reinterpret_cast<const char*>(img.data()),
                static_cast<std::streamsize>(img.size()));
        std::printf("hd saved: %s (%zu bytes)\n", saveHdPath, img.size());
    }
    return 0;
}
