#pragma once

// What the headless trace tools share for the storage tests: the drop box
// volume built from a host folder, the guest-side read check, and the list
// of drivers in the guest's unit table.

#include <openmac/hfs.hpp>
#include <openmac/types.hpp>

#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

namespace tracetools {

using openmac::u8;
using openmac::u16;
using openmac::u32;
using openmac::s16;
namespace hfs = openmac::hfs;

inline std::vector<u8> loadFile(const char* path) {
    std::ifstream f(path, std::ios::binary);
    return std::vector<u8>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

// ---- call census ------------------------------------------------------
// How often the guest makes the calls a machine can hang work of its own on:
// the event calls a program makes from its own level, and the Resource
// Manager's reads, which is most of what a startup with no program yet is
// made of. Fed from the CPU's trap hook; one line per emulated second with
// the counts and the longest stretch of frames without an event call.
struct CallCensus {
    bool on = false;
    u32 events = 0, resources = 0, second = 0;
    u32 lastEventFrame = 0, longestGap = 0;
    static bool eventCall(u16 w) { return w == 0xA970 || w == 0xA860 || w == 0xA971 || w == 0xA9B4; }
    static bool resourceCall(u16 w) {
        return w == 0xA9A0 || w == 0xA81F || w == 0xA9A1 || w == 0xA820 || w == 0xA997 || w == 0xA81A;
    }
    void note(u16 opcode, u16 sr, u32 frame) {
        if (!on || (sr & 0x0700u) != 0) return;
        if (eventCall(opcode)) {
            ++events;
            if (frame - lastEventFrame > longestGap) longestGap = frame - lastEventFrame;
            lastEventFrame = frame;
        } else if (resourceCall(opcode)) {
            ++resources;
        }
    }
    void frameDone(u32 frame) {
        if (!on || (frame + 1) % 60 != 0) return;
        if (frame - lastEventFrame > longestGap) longestGap = frame - lastEventFrame;
        std::printf("calls second %u (to frame %u): event %u, resource %u, longest stretch without an event call %u frames\n",
                    second++, frame, events, resources, longestGap);
        events = resources = longestGap = 0;
    }
};

// ---- drop box ---------------------------------------------------------
// The front end walks a host folder and hands each file to the volume
// builder; this is the same walk, so a headless run exercises the code path
// the GUI takes rather than a convenient approximation of it.

inline std::pair<u32, u32> inferTypeCreator(const std::string& ext) {
    std::string e;
    for (char c : ext) e += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (e == ".txt" || e == ".text" || e == ".md") return {0x54455854u, 0x74747874u};
    if (e == ".sit") return {0x53495444u, 0x53495421u};
    if (e == ".sea") return {0x4150504Cu, 0x61757374u};
    if (e == ".cpt") return {0x50414354u, 0x43504354u};
    if (e == ".hqx") return {0x54455854u, 0x426E4871u};
    if (e == ".zip") return {0x5A495020u, 0x5A495020u};
    if (e == ".lha" || e == ".lzh") return {0x4C484120u, 0x4C415243u};
    return {0x3F3F3F3Fu, 0x3F3F3F3Fu};
}

inline void addFolderTree(hfs::VolumeBuilder& b, u32 parent,
                   const std::filesystem::path& dir) {
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
        const std::string name = e.path().filename().string();
        if (name == "_openmac-removed") continue;
        if (e.is_directory(ec)) {
            const u32 id = b.addDir(parent, name);
            if (id) addFolderTree(b, id, e.path());
        } else if (e.is_regular_file(ec)) {
            std::vector<u8> data = loadFile(e.path().string().c_str());
            const auto [type, creator] = inferTypeCreator(e.path().extension().string());
            b.addFile(parent, name, type, creator, 0, std::move(data), {});
        }
    }
}

inline std::vector<u8> buildFolderVolume(const std::string& folder, std::string& why) {
    std::filesystem::path p(folder);
    std::string vol = p.filename().string();
    if (vol.empty()) vol = "Drop Box";
    hfs::VolumeBuilder b(vol);
    addFolderTree(b, 2, p);
    std::vector<u8> img = b.build(0);
    if (img.empty()) why = b.why();
    return img;
}

// Read FILE (a host file that is also on the second disk's volume) through
// the guest's own File Manager and compare. The bytes being on the volume and
// the guest being able to fetch them are two different claims; this tests the
// second.
template <class Machine>
void reportGuestRead(Machine& mac, const std::string& hostFile) {
    std::vector<u8> want = loadFile(hostFile.c_str());
    std::string base = std::filesystem::path(hostFile).filename().string();
    std::vector<u8> got;
    std::string why;
    if (!mac.readFileThroughGuest(base, got, why)) {
        std::printf("GUEST READ FAILED: '%s' -- %s\n", base.c_str(), why.c_str());
    } else if (got.size() != want.size()) {
        std::printf("GUEST READ MISMATCH: '%s' is %zu bytes to the guest, "
                    "%zu on the host\n", base.c_str(), got.size(), want.size());
    } else {
        std::size_t diff = 0, firstAt = 0;
        for (std::size_t k = 0; k < got.size(); ++k)
            if (got[k] != want[k]) { if (!diff) firstAt = k; ++diff; }
        if (diff)
            std::printf("GUEST READ MISMATCH: '%s' same length but %zu bytes "
                        "differ, first at offset %zu (block %zu)\n",
                        base.c_str(), diff, firstAt, firstAt / 512);
        else
            std::printf("GUEST READ OK: '%s' %zu bytes, byte-for-byte identical "
                        "read through the guest's own File Manager\n",
                        base.c_str(), got.size());
    }
}

// Every driver in the guest's unit table, by name. ramTop = bytes of RAM.
template <class Machine>
void printDrivers(Machine& mac, u32 ramTop) {
    // UTableBase $011C, UnitNtryCnt $01D2. Each entry is a handle to a DCE;
    // dCtlFlags bit 6 says whether dCtlDriver is a handle or a pointer, and
    // the DRVR's name is a Pascal string 18 bytes into its header.
    // A driver lives in RAM or in the ROM; reading anywhere else would be a
    // device access, and an instrument must not touch the machine.
    const auto readable = [&](u32 a, u32 len) {
        return (a && a + len < ramTop) ||
               (a >= 0x40800000u && a + len < 0x40900000u);
    };
    // A master pointer carries the Memory Manager's locked/purgeable marks
    // in its high byte, and every DCE in this table is a locked one. Strip
    // them the way the machine's own guestPtr does -- only when the raw
    // value is not already a real address, because above 16 MB it can be.
    const auto deref = [&](u32 p) {
        if (!p || p + 3 < ramTop) return p;
        const u32 stripped = p & 0x00FFFFFFu;
        return stripped + 3 < ramTop ? stripped : p;
    };
    const u32 utab = mac.read32(0x011C);
    const u32 count = mac.read16(0x01D2);
    std::printf("\n-- drivers (unit table %08X, %u entries) --\n", utab, count);
    for (u32 i = 0; i < count && readable(utab + i * 4, 4); ++i) {
        const u32 h = mac.read32(utab + i * 4);
        if (!h) continue;                       // an empty slot is normal
        const u32 dce = deref(readable(h, 4) ? mac.read32(h) : 0);
        const u16 flags = readable(dce, 26) ? mac.read16(dce + 4) : 0;
        u32 drvr = readable(dce, 26) ? deref(mac.read32(dce + 0)) : 0;
        // dCtlDriver is a handle when dRAMBased is set and a pointer when
        // it is not -- and the flag is the only thing that says which.
        if (drvr && (flags & 0x0040))
            drvr = deref(readable(drvr, 4) ? mac.read32(drvr) : 0);
        std::string name = "?";
        if (readable(drvr, 20)) {
            const u8 len = mac.read8(drvr + 18);
            if (len && len <= 31 && readable(drvr + 19, len)) {
                name.clear();
                for (u8 k = 0; k < len; ++k)
                    name += char(mac.read8(drvr + 19 + k));
            }
        }
        std::printf("  unit %2u  refNum %5d  h %08X  dce %08X  flags %04X  "
                    "drvr %08X  %s\n",
                    i, static_cast<int>(static_cast<s16>(~u16(i))), h, dce,
                    flags, drvr, name.c_str());
    }
}

} // namespace tracetools
