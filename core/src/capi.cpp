// C ABI implementation: a thin wrapper over Machine plus the dbg:: monitor.
// See openmac/capi.h. Built as the openmac_c shared library for the .NET GUI.

#include "openmac/capi.h"

#include "openmac/debugger.hpp"
#include "openmac/hfs.hpp"
#include "openmac/machine.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using openmac::u8;
using openmac::u16;
using openmac::u32;

struct OMac {
    openmac::Machine mac;
    OMacLogFn logFn = nullptr;
    void* logUser = nullptr;
    uint32_t dbgFlags = 0;
    std::vector<std::string> logBuf;   // drained off the hot path by omac_poll_log
    std::vector<u8> audioBuf;          // drained by omac_drain_audio

    OMac(std::vector<u8> rom, uint32_t ramMB)
        : mac(std::move(rom), openmac::Machine::Config{ramMB * 1024u * 1024u}) {
        // Always-on, low-volume diagnostics (disk insert/mount) -> the GUI log.
        mac.onDiag = [this](const char* s) { log(s); };
    }

    void log(const char* s) {
        if (logFn) logFn(logUser, s);                 // legacy direct callback
        if (logBuf.size() < 4000) logBuf.emplace_back(s);
    }
};

namespace {
void formatRegs(const openmac::M68000& c, char* b, size_t cap)
{
    std::snprintf(b, cap,
        "  D0-3 %08X %08X %08X %08X\n"
        "  D4-7 %08X %08X %08X %08X\n"
        "  A0-3 %08X %08X %08X %08X\n"
        "  A4-7 %08X %08X %08X %08X\n"
        "  PC %06X  SR %04X",
        c.d[0], c.d[1], c.d[2], c.d[3], c.d[4], c.d[5], c.d[6], c.d[7],
        c.a[0], c.a[1], c.a[2], c.a[3], c.a[4], c.a[5], c.a[6], c.a[7],
        c.pc, c.getSR());
}
} // namespace

extern "C" {

OMAC_API OMac* omac_create(const uint8_t* rom, size_t rom_len, uint32_t ram_mb)
{
    if (!rom || rom_len == 0) return nullptr;
    try {
        return new OMac(std::vector<u8>(rom, rom + rom_len), ram_mb ? ram_mb : 4u);
    } catch (...) {
        return nullptr;
    }
}

OMAC_API void omac_destroy(OMac* m) { delete m; }
OMAC_API void omac_reset(OMac* m) { if (m) m->mac.reset(); }
OMAC_API void omac_set_force_rom_disk(OMac* m, int on) { if (m) m->mac.setForceRomDisk(on != 0); }
OMAC_API void omac_run_frame(OMac* m) { if (m) m->mac.runFrame(); }
OMAC_API void omac_render(OMac* m, uint32_t* argb) { if (m && argb) m->mac.renderScreen(argb); }

OMAC_API size_t omac_drain_audio(OMac* m, uint8_t* out, size_t cap)
{
    if (!m || !out || cap == 0) return 0;
    m->mac.drainAudio(m->audioBuf);
    const size_t n = m->audioBuf.size() < cap ? m->audioBuf.size() : cap;
    if (n) std::memcpy(out, m->audioBuf.data(), n);
    return n;
}

OMAC_API void omac_insert_floppy(OMac* m, const uint8_t* img, size_t len, int ro)
{
    if (m && img) m->mac.insertFloppy(std::vector<u8>(img, img + len), ro != 0);
}
OMAC_API void omac_eject_floppy(OMac* m) { if (m) m->mac.ejectFloppy(); }
OMAC_API void omac_insert_harddisk(OMac* m, const uint8_t* img, size_t len, int ro)
{
    if (m && img) m->mac.insertHardDisk(std::vector<u8>(img, img + len), ro != 0);
}

OMAC_API size_t omac_harddisk_data(OMac* m, uint8_t* out, size_t cap)
{
    if (!m || !m->mac.hardDiskPresent()) return 0;
    const auto& img = m->mac.hardDiskImage();
    if (!out) return img.size();               // query size
    const size_t n = img.size() < cap ? img.size() : cap;
    if (n) std::memcpy(out, img.data(), n);
    return n;
}

OMAC_API int omac_format_hfs(uint32_t size_bytes, const char* name, uint8_t* out)
{
    if (!out) return -1;
    try {
        auto v = openmac::hfs::formatVolume(size_bytes, name ? name : "Untitled");
        std::memcpy(out, v.data(), v.size());
        return 0;
    } catch (...) {
        return -2;
    }
}

OMAC_API void omac_mouse(OMac* m, int dx, int dy, int button)
{
    if (m) m->mac.mouseMove(dx, dy, button != 0);
}
OMAC_API void omac_key(OMac* m, int adb, int down)
{
    if (m) m->mac.keyEvent(static_cast<u8>(adb), down != 0);
}

OMAC_API void omac_regs(OMac* m, OMacRegs* out)
{
    if (!m || !out) return;
    const auto& c = m->mac.cpu();
    for (int i = 0; i < 8; ++i) { out->d[i] = c.d[i]; out->a[i] = c.a[i]; }
    out->pc = c.pc;
    out->sr = c.getSR();
    out->cycles = m->mac.totalCycles();
}

OMAC_API void omac_set_log(OMac* m, OMacLogFn fn, void* user)
{
    if (!m) return;
    m->logFn = fn;
    m->logUser = user;
}

OMAC_API void omac_debug_enable(OMac* m, uint32_t flags)
{
    if (!m) return;
    m->dbgFlags = flags;
    auto& cpu = m->mac.cpu();
    OMac* self = m;

    if (flags & OMAC_DBG_TRAPS)
        cpu.onTrap = [self](u16 op, u32 pc) {
            char b[160];
            const char* n = openmac::dbg::trapName(op);
            std::snprintf(b, sizeof b, "TRAP %s ($%04X) @ %06X", n ? n : "?", op, pc);
            self->log(b);
        };
    else
        cpu.onTrap = nullptr;

    if (flags & OMAC_DBG_EXCEPT)
        cpu.onException = [self](int vec, u32 pc) {
            // Real faults only. vec 10 (line-1010 / A-line) is the normal Toolbox
            // trap dispatch and fires constantly -- logging it floods the sink and
            // (via the front-end log callback) starves the boot.
            if (vec != 2 && vec != 3 && vec != 4 && vec != 8 && vec != 11) return;
            static const char* kNames[] = {"reset", "", "bus", "addr", "illegal"};
            const char* nm = (vec >= 0 && vec <= 4) ? kNames[vec] : "exc";
            auto& mac = self->mac;
            char regs[400];
            formatRegs(mac.cpu(), regs, sizeof regs);
            // The faulting instruction plus a short A6 frame-chain backtrace, so the log
            // alone locates a crash's cause -- e.g. who called in with the bad pointer --
            // without having to reproduce it.
            std::string ins;
            openmac::dbg::disasm(mac, pc, ins);
            auto r32 = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
            char bt[240];
            int n = std::snprintf(bt, sizeof bt, "  bt");
            u32 fp = mac.cpu().a[6];
            for (int i = 0; i < 10 && fp >= 0x100 && fp < 0x00400000 && !(fp & 1); ++i) {
                n += std::snprintf(bt + n, (n < (int)sizeof bt) ? sizeof bt - n : 0,
                                   " %06X", r32(fp + 4) & 0xFFFFFF);
                const u32 nf = r32(fp);
                if (nf <= fp || nf >= 0x00400000 || (nf & 1)) break;   // must climb + stay even
                fp = nf;
            }
            // The last instructions executed before the fault -- the path into it.
            char tr[280];
            int tn = std::snprintf(tr, sizeof tr, "  trail");
            for (int i = 23; i >= 0 && tn < (int)sizeof tr; --i)
                tn += std::snprintf(tr + tn, sizeof tr - tn, " %06X",
                                    mac.cpu().recentPc(i) & 0xFFFFFF);
            char b[1400];
            std::snprintf(b, sizeof b, "EXC vec=%d (%s) @ %06X cyc=%llu  [%s]\n%s\n%s\n%s",
                          vec, nm, pc,
                          static_cast<unsigned long long>(mac.totalCycles()),
                          ins.c_str(), regs, bt, tr);
            self->log(b);
        };
    else
        cpu.onException = nullptr;

    if (flags & OMAC_DBG_IRQ)
        cpu.onInterrupt = [self](int level, int vec, u32 pc) {
            char b[96];
            std::snprintf(b, sizeof b, "IRQ level %d (vec %d) @ %06X", level, vec, pc);
            self->log(b);
        };
    else
        cpu.onInterrupt = nullptr;

    if (flags & OMAC_DBG_ADB)
        m->mac.onAdbEvent = [self](const char* ev, int st, u32 val) {
            char b[96];
            std::snprintf(b, sizeof b, "ADB %s state=%d val=%02X", ev, st, val);
            self->log(b);
        };
    else
        m->mac.onAdbEvent = nullptr;
}

OMAC_API void omac_debug_dump(OMac* m, const char* name, char* out, size_t cap)
{
    if (!m || !out || cap == 0) return;
    out[0] = '\0';
    if (!name) return;
    if (std::strcmp(name, "regs") == 0)
        formatRegs(m->mac.cpu(), out, cap);
    else
        std::snprintf(out, cap, "(%s view not wired yet; enable Debug mode to stream it to the log)", name);
}

OMAC_API void omac_poll_log(OMac* m, char* out, size_t cap)
{
    if (!m || !out || cap == 0) return;
    size_t pos = 0;
    for (const auto& line : m->logBuf) {
        if (pos + line.size() + 1 >= cap) break;
        std::memcpy(out + pos, line.data(), line.size());
        pos += line.size();
        out[pos++] = '\n';
    }
    out[pos] = '\0';
    m->logBuf.clear();
}

OMAC_API const char* omac_version(void) { return "OpenMac core 0.1"; }

} // extern "C"
