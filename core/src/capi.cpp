// C ABI implementation: a thin wrapper over Machine plus the dbg:: monitor.
// See openmac/capi.h. Built as the openmac_c shared library for the .NET GUI.

#include "openmac/capi.h"

#include "openmac/debugger.hpp"
#include "openmac/hfs.hpp"
#include "openmac/iifx.hpp"
#include "openmac/machine.hpp"

#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

using openmac::u8;
using openmac::u16;
using openmac::u32;
using openmac::u64;
using openmac::s64;

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

OMAC_API int omac_insert_floppy(OMac* m, const uint8_t* img, size_t len, int ro)
{
    if (!m || !img) return 0;
    return m->mac.insertFloppy(std::vector<u8>(img, img + len), ro != 0) ==
                   openmac::Machine::InsertVerdict::kAccepted
               ? 1
               : 0;
}
OMAC_API void omac_eject_floppy(OMac* m) { if (m) m->mac.ejectFloppy(); }

OMAC_API size_t omac_floppy_medium(OMac* m, int drive, char* out, size_t cap)
{
    if (!m) return 0;
    const char* s = m->mac.mediumText(drive);
    const size_t n = std::strlen(s);
    if (out && cap) {
        const size_t c = n < cap - 1 ? n : cap - 1;
        std::memcpy(out, s, c);
        out[c] = 0;
    }
    return n;
}

OMAC_API void omac_set_external_drive(OMac* m, int attached)
{
    if (m) m->mac.setExternalDriveAttached(attached != 0);
}

OMAC_API int omac_insert_floppy2(OMac* m, const uint8_t* img, size_t len, int ro)
{
    if (!m || !img) return 0;
    return m->mac.insertExternalFloppy(std::vector<u8>(img, img + len), ro != 0) ==
                   openmac::Machine::InsertVerdict::kAccepted
               ? 1
               : 0;
}

OMAC_API void omac_eject_floppy2(OMac* m) { if (m) m->mac.ejectExternalFloppy(); }

OMAC_API int omac_floppy_present(OMac* m, int drive)
{
    if (!m) return 0;
    return (drive == 1 ? m->mac.externalFloppyInserted() : m->mac.floppyInserted()) ? 1 : 0;
}

static size_t copyOut(const std::vector<u8>& src, uint8_t* out, size_t cap)
{
    if (out && cap >= src.size()) std::copy(src.begin(), src.end(), out);
    return src.size();
}

OMAC_API size_t omac_floppy_data(OMac* m, uint8_t* out, size_t cap)
{
    return m ? copyOut(m->mac.floppyImage(), out, cap) : 0;
}

OMAC_API size_t omac_floppy2_data(OMac* m, uint8_t* out, size_t cap)
{
    return m ? copyOut(m->mac.externalFloppyImage(), out, cap) : 0;
}
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

OMAC_API void omac_insert_harddisk2(OMac* m, const uint8_t* img, size_t len, int ro)
{
    if (m && img) m->mac.insertHardDisk2(std::vector<u8>(img, img + len), ro != 0);
}

OMAC_API void omac_detach_harddisk2(OMac* m) { if (m) m->mac.detachHardDisk2(); }

OMAC_API int omac_harddisk2_booted(OMac* m)
{
    return m && m->mac.hardDisk2DriverResident() ? 1 : 0;
}

OMAC_API int omac_unmount_harddisk2(OMac* m)
{
    return (m && m->mac.unmountSecondDisk()) ? 1 : 0;
}

OMAC_API int omac_flush_volumes(OMac* m)
{
    return (m && m->mac.flushVolumes()) ? 1 : 0;
}

OMAC_API size_t omac_harddisk2_data(OMac* m, uint8_t* out, size_t cap)
{
    if (!m || !m->mac.hardDisk2Present()) return 0;
    const auto& img = m->mac.hardDisk2Image();
    if (!out) return img.size();
    const size_t n = img.size() < cap ? img.size() : cap;
    if (n) std::memcpy(out, img.data(), n);
    return n;
}

namespace {

struct HfsBuilderHandle {
    openmac::hfs::VolumeBuilder builder;
    std::vector<u8> built;
    bool done = false;
    explicit HfsBuilderHandle(const char* name)
        : builder(name ? name : "Untitled") {}
    const std::vector<u8>& image(uint32_t sizeBytes)
    {
        if (!done) { built = builder.build(sizeBytes); done = true; }
        return built;
    }
};

struct HfsReaderHandle {
    std::vector<u8> img;
    std::vector<openmac::hfs::Item> items;
};

} // namespace

OMAC_API void* omac_hfsb_begin(const char* volume_name)
{
    try { return new HfsBuilderHandle(volume_name); } catch (...) { return nullptr; }
}

OMAC_API uint32_t omac_hfsb_add_dir(void* b, uint32_t parent, const char* name,
                                    uint32_t cr, uint32_t md)
{
    if (!b || !name) return 0;
    return static_cast<HfsBuilderHandle*>(b)->builder.addDir(parent, name, cr, md);
}

OMAC_API void omac_hfsb_add_file(void* b, uint32_t parent, const char* name,
                                 uint32_t type, uint32_t creator, uint16_t fdflags,
                                 const uint8_t* data, size_t data_len,
                                 const uint8_t* rsrc, size_t rsrc_len,
                                 uint32_t cr, uint32_t md)
{
    if (!b || !name) return;
    static_cast<HfsBuilderHandle*>(b)->builder.addFile(
        parent, name, type, creator, fdflags,
        data ? std::vector<u8>(data, data + data_len) : std::vector<u8>{},
        rsrc ? std::vector<u8>(rsrc, rsrc + rsrc_len) : std::vector<u8>{}, cr, md);
}

OMAC_API size_t omac_hfsb_build(void* b, uint8_t* out, size_t cap)
{
    if (!b) return 0;
    const auto& img = static_cast<HfsBuilderHandle*>(b)->image(0);
    if (!out) return img.size();
    const size_t n = img.size() < cap ? img.size() : cap;
    if (n) std::memcpy(out, img.data(), n);
    return n;
}

OMAC_API size_t omac_hfsb_build_sized(void* b, uint32_t size_bytes, uint8_t* out, size_t cap)
{
    if (!b) return 0;
    const auto& img = static_cast<HfsBuilderHandle*>(b)->image(size_bytes);
    if (!out) return img.size();
    const size_t n = img.size() < cap ? img.size() : cap;
    if (n) std::memcpy(out, img.data(), n);
    return n;
}

OMAC_API size_t omac_hfsb_error(void* b, char* out, size_t cap)
{
    if (!b) return 0;
    const std::string& s = static_cast<HfsBuilderHandle*>(b)->builder.why();
    if (out && cap) {
        const size_t c = s.size() < cap - 1 ? s.size() : cap - 1;
        std::memcpy(out, s.data(), c);
        out[c] = 0;
    }
    return s.size();
}

OMAC_API void omac_hfsb_free(void* b) { delete static_cast<HfsBuilderHandle*>(b); }

OMAC_API void* omac_hfsr_open(const uint8_t* img, size_t len)
{
    if (!img || !len) return nullptr;
    try {
        auto* h = new HfsReaderHandle;
        h->img.assign(img, img + len);
        if (!openmac::hfs::listVolume(h->img, h->items)) { delete h; return nullptr; }
        return h;
    } catch (...) {
        return nullptr;
    }
}

OMAC_API int32_t omac_hfsr_count(void* r)
{
    return r ? static_cast<int32_t>(static_cast<HfsReaderHandle*>(r)->items.size()) : 0;
}

OMAC_API int32_t omac_hfsr_item(void* r, int32_t index, OMacHfsItem* out)
{
    if (!r || !out) return 0;
    const auto& items = static_cast<HfsReaderHandle*>(r)->items;
    if (index < 0 || static_cast<size_t>(index) >= items.size()) return 0;
    const auto& it = items[static_cast<size_t>(index)];
    out->id = it.id;
    out->parent = it.parent;
    out->is_dir = it.isDir ? 1 : 0;
    out->type = it.type;
    out->creator = it.creator;
    out->fd_flags = it.fdFlags;
    out->cr_date = it.crDate;
    out->md_date = it.mdDate;
    out->data_len = it.dataLen;
    out->rsrc_len = it.rsrcLen;
    const size_t n = it.name.size() < sizeof out->name - 1 ? it.name.size()
                                                           : sizeof out->name - 1;
    std::memcpy(out->name, it.name.data(), n);
    out->name[n] = 0;
    return 1;
}

OMAC_API size_t omac_hfsr_fork(void* r, uint32_t file_id, int32_t rsrc,
                               uint8_t* out, size_t cap)
{
    if (!r) return 0;
    auto* h = static_cast<HfsReaderHandle*>(r);
    std::vector<u8> fork;
    if (!openmac::hfs::readFork(h->img, file_id, rsrc != 0, fork)) return 0;
    if (!out) return fork.size();
    const size_t n = fork.size() < cap ? fork.size() : cap;
    if (n) std::memcpy(out, fork.data(), n);
    return n;
}

OMAC_API void omac_hfsr_free(void* r) { delete static_cast<HfsReaderHandle*>(r); }

OMAC_API void omac_cd_attach(OMac* m, int attached, int scsi_id)
{
    if (m) m->mac.attachCdRom(attached != 0, scsi_id);
}

OMAC_API int omac_cd_attached(OMac* m) { return m && m->mac.cdRomAttached() ? 1 : 0; }

OMAC_API int omac_cd_insert(OMac* m, const uint8_t* img, size_t len)
{
    if (!m || !img) return 0;
    return m->mac.insertCd(std::vector<u8>(img, img + len)) ==
                   openmac::Machine::InsertVerdict::kAccepted
               ? 1
               : 0;
}

OMAC_API void omac_net_attach(OMac* m, int attached, int scsi_id)
{
    if (m) m->mac.attachNet(attached != 0, scsi_id);
}

OMAC_API int omac_net_attached(OMac* m) { return m && m->mac.netAttached() ? 1 : 0; }

OMAC_API int omac_net_inject(OMac* m, const uint8_t* frame, size_t len)
{
    if (!m || !frame) return 0;
    return m->mac.netInject(frame, static_cast<u32>(len)) ? 1 : 0;
}

OMAC_API size_t omac_net_drain(OMac* m, uint8_t* out, size_t cap)
{
    if (!m || !out) return 0;
    std::vector<u8> f;
    if (!m->mac.netDrain(f)) return 0;
    const size_t n = f.size() < cap ? f.size() : cap;
    if (n) std::memcpy(out, f.data(), n);
    return n;
}

OMAC_API void omac_cd_eject(OMac* m) { if (m) m->mac.ejectCd(); }
OMAC_API int omac_cd_present(OMac* m) { return m && m->mac.cdPresent() ? 1 : 0; }

// Parameter RAM. Call with out = null for the size. `addSeconds` on load is how
// long the machine was switched off, so the clock carries on the way a battery
// would rather than restarting where it stopped.
OMAC_API size_t omac_pram_save(OMac* m, uint8_t* out, size_t cap)
{
    if (!m) return 0;
    return m->mac.savePram(out, static_cast<uint32_t>(cap));
}

OMAC_API int omac_pram_load(OMac* m, const uint8_t* data, size_t len,
                            uint32_t addSeconds)
{
    return m && m->mac.loadPram(data, static_cast<uint32_t>(len), addSeconds) ? 1 : 0;
}

OMAC_API size_t omac_cd_medium(OMac* m, char* out, size_t cap)
{
    if (!m) return 0;
    const char* s = m->mac.cdMediumText();
    const size_t n = std::strlen(s);
    if (out && cap) {
        const size_t c = n < cap - 1 ? n : cap - 1;
        std::memcpy(out, s, c);
        out[c] = 0;
    }
    return n;
}

OMAC_API int omac_format_hfs(uint32_t size_bytes, const char* name, uint8_t* out)
{
    if (!out) return -1;
    try {
        auto v = openmac::hfs::formatVolume(size_bytes, name ? name : "Untitled");
        if (v.size() != size_bytes) return -2;
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

OMAC_API const char* omac_version(void) { return "OpenMac core 0.5.0"; }

} // extern "C"

// ---- Macintosh IIfx surface -----------------------------------------------

struct OMacFx {
    openmac::IifxMachine mac;
    std::vector<u8> audioBuf;
    std::vector<std::string> logBuf;

    OMacFx(std::vector<u8> rom, std::vector<u8> videoRom, uint32_t ramMB)
        : mac(std::move(rom), [&] {
              openmac::IifxMachine::Config config;
              config.ramSize = ramMB * 1024u * 1024u;
              config.videoCard = true;
              config.nativeStorage = true;
              config.videoDeclarationRom = std::move(videoRom);
              return config;
          }()) {
        // The front end keeps the low-volume lines (disk attach/mount, driver
        // install, GC report, guest faults). The per-access diagnostics are a
        // boot-trace tool: the rolling formatted access log costs a snprintf
        // on EVERY I/O access, and the device-traffic lines (one per SCSI
        // register byte, RTC bit, SCC write, IOP mailbox step) were half a
        // million of the 557k lines in a 45 MB session log — burying the
        // twenty that mattered while the machine paid to format them.
        mac.setLegacyAccessLogEnabled(false);
        mac.setDeviceTrafficDiagnostics(false);
        mac.onDiag = [this](const char* s) {
            if (logBuf.size() < 4000) logBuf.emplace_back(s);
        };
        mac.cpu().onException = [this](int vector, u32 pc) {
            // A-line/F-line traps and TRAP #n are the Macintosh ABI. Keep the
            // exceptional cases that can explain a bomb or a stalled boot.
            if (vector == 10 || vector == 11 || (vector >= 32 && vector < 48))
                return;
            if (pc >= 0x40000000u || faultBudget <= 0) return;
            --faultBudget;
            char line[160];
            std::snprintf(line, sizeof line,
                          "[guest] exception %d pc=%08X addr=%08X sr=%04X frame=%llu",
                          vector, pc, mac.cpu().lastFaultAddr,
                          mac.cpu().getSR(),
                          static_cast<unsigned long long>(mac.frameCount()));
            if (logBuf.size() < 4000) logBuf.emplace_back(line);
        };
    }

    int faultBudget = 40;
    u64 hostMouseCalls = 0;
    u64 hostMouseMotionCalls = 0;
    s64 hostMouseDx = 0;
    s64 hostMouseDy = 0;
    u64 hostMouseButtonTransitions = 0;
    bool hostMouseButton = false;
};

extern "C" {

OMAC_API OMacFx* omac_fx_create(const uint8_t* rom, size_t rom_len,
                                 uint32_t ram_mb)
{
    return omac_fx_create_with_video_rom(rom, rom_len, nullptr, 0, ram_mb);
}

OMAC_API OMacFx* omac_fx_create_with_video_rom(
    const uint8_t* rom, size_t rom_len, const uint8_t* video_rom,
    size_t video_rom_len, uint32_t ram_mb)
{
    // This machine has one supported 512 KiB ROM revision. Rejecting a
    // Classic-family image here is important because both files have the same
    // length and a padded/incorrect image otherwise appears to start before it
    // disappears into unrelated vectors.
    if (!rom || rom_len != 512u * 1024u ||
        rom[0] != 0x41 || rom[1] != 0x47 ||
        rom[2] != 0xDD || rom[3] != 0x77)
        return nullptr;
    try {
        std::vector<u8> video;
        if (video_rom && video_rom_len)
            video.assign(video_rom, video_rom + video_rom_len);
        return new OMacFx(std::vector<u8>(rom, rom + rom_len),
                          std::move(video), ram_mb ? ram_mb : 8u);
    } catch (...) {
        return nullptr;
    }
}

OMAC_API void omac_fx_destroy(OMacFx* m) { delete m; }
OMAC_API void omac_fx_reset(OMacFx* m) { if (m) m->mac.reset(); }
OMAC_API void omac_fx_run_frame(OMacFx* m) { if (m) m->mac.runFrame(); }
OMAC_API int omac_fx_screen_w(OMacFx* m) { return m ? m->mac.screenWidth() : 640; }
OMAC_API int omac_fx_screen_h(OMacFx* m) { return m ? m->mac.screenHeight() : 480; }
OMAC_API void omac_fx_render(OMacFx* m, uint32_t* argb)
{
    if (m && argb) m->mac.renderScreen(argb);
}

OMAC_API uint32_t omac_fx_audio_rate(OMacFx* m)
{
    return m ? m->mac.audioSampleRate() : 22254u;
}

OMAC_API size_t omac_fx_drain_audio(OMacFx* m, uint8_t* out, size_t cap)
{
    if (!m || !out || cap == 0) return 0;
    m->mac.drainAudio(m->audioBuf);
    const size_t n = std::min(m->audioBuf.size(), cap);
    if (n) std::memcpy(out, m->audioBuf.data(), n);
    return n;
}

OMAC_API void omac_fx_mouse(OMacFx* m, int dx, int dy, int button)
{
    if (!m) return;
    ++m->hostMouseCalls;
    if (dx != 0 || dy != 0) {
        ++m->hostMouseMotionCalls;
        m->hostMouseDx += dx;
        m->hostMouseDy += dy;
    }
    const bool down = button != 0;
    if (down != m->hostMouseButton) {
        ++m->hostMouseButtonTransitions;
        m->hostMouseButton = down;
    }
    m->mac.mouseMove(dx, dy, down);
}

OMAC_API void omac_fx_key(OMacFx* m, int adb_code, int down)
{
    if (m) m->mac.keyEvent(static_cast<u8>(adb_code), down != 0);
}

OMAC_API int omac_fx_insert_floppy(OMacFx* m, const uint8_t* img,
                                    size_t len, int read_only)
{
    if (!m || !img || !len) return 0;
    return m->mac.insertFloppy(std::vector<u8>(img, img + len), read_only != 0);
}

OMAC_API void omac_fx_eject_floppy(OMacFx* m)
{
    if (m) m->mac.ejectFloppy();
}

OMAC_API int omac_fx_floppy_present(OMacFx* m)
{
    return m && m->mac.floppyPresent() ? 1 : 0;
}

OMAC_API size_t omac_fx_floppy_writeback(OMacFx* m, uint8_t* out, size_t cap)
{
    if (!m) return 0;
    const std::vector<u8> image = m->mac.floppyForWriteBack();
    if (image.empty()) return 0;
    if (!out) return image.size();
    if (cap < image.size()) return 0;
    std::copy(image.begin(), image.end(), out);
    return image.size();
}

OMAC_API void omac_fx_insert_harddisk(OMacFx* m, const uint8_t* img,
                                       size_t len, int read_only)
{
    if (m && img && len)
        m->mac.insertHardDisk(std::vector<u8>(img, img + len), read_only != 0);
}

OMAC_API void omac_fx_detach_harddisk(OMacFx* m)
{
    if (m) m->mac.insertHardDisk({});
}

OMAC_API size_t omac_fx_harddisk_data(OMacFx* m, uint8_t* out, size_t cap)
{
    if (!m || !m->mac.hardDiskPresent()) return 0;
    const auto& image = m->mac.hardDiskImage();
    if (!out) return image.size();
    if (cap < image.size()) return 0;
    std::copy(image.begin(), image.end(), out);
    return image.size();
}

OMAC_API int omac_fx_shutdown_harddisk(OMacFx* m)
{
    return (m && m->mac.shutdownHardDisk()) ? 1 : 0;
}

OMAC_API void omac_fx_insert_harddisk2(OMacFx* m, const uint8_t* img,
                                        size_t len, int read_only)
{
    if (m && img && len)
        m->mac.insertHardDisk2(std::vector<u8>(img, img + len), read_only != 0);
}

OMAC_API void omac_fx_detach_harddisk2(OMacFx* m)
{
    if (m) m->mac.detachHardDisk2();
}

OMAC_API size_t omac_fx_harddisk2_data(OMacFx* m, uint8_t* out, size_t cap)
{
    if (!m || !m->mac.hardDisk2Present()) return 0;
    const auto& image = m->mac.hardDisk2Image();
    if (!out) return image.size();
    if (cap < image.size()) return 0;
    std::copy(image.begin(), image.end(), out);
    return image.size();
}

OMAC_API int omac_fx_harddisk2_booted(OMacFx* m)
{
    return (m && m->mac.secondDiskDriverResident()) ? 1 : 0;
}

OMAC_API int omac_fx_unmount_harddisk2(OMacFx* m)
{
    return (m && m->mac.unmountHardDisk2()) ? 1 : 0;
}

OMAC_API size_t omac_fx_pram_save(OMacFx* m, uint8_t* out, size_t cap)
{
    return m ? m->mac.savePram(out, static_cast<u32>(cap)) : 0;
}

OMAC_API int omac_fx_pram_load(OMacFx* m, const uint8_t* data, size_t len,
                                uint32_t add_seconds)
{
    return m && m->mac.loadPram(data, static_cast<u32>(len), add_seconds) ? 1 : 0;
}

OMAC_API size_t omac_fx_diagnostics(OMacFx* m, char* out, size_t cap)
{
    if (!m) return 0;
    std::string report = m->mac.diagnosticReport();
    char input[256];
    std::snprintf(input, sizeof input,
                  "  host input mouse calls=%llu motion=%llu delta=%lld/%lld "
                  "button-transitions=%llu down=%d\n",
                  static_cast<unsigned long long>(m->hostMouseCalls),
                  static_cast<unsigned long long>(m->hostMouseMotionCalls),
                  static_cast<long long>(m->hostMouseDx),
                  static_cast<long long>(m->hostMouseDy),
                  static_cast<unsigned long long>(m->hostMouseButtonTransitions),
                  m->hostMouseButton ? 1 : 0);
    report += input;
    if (!out || cap == 0) return report.size() + 1;
    const size_t n = std::min(report.size(), cap - 1);
    std::copy(report.begin(), report.begin() + static_cast<std::ptrdiff_t>(n), out);
    out[n] = '\0';
    return n;
}

OMAC_API void omac_fx_poll_log(OMacFx* m, char* out, size_t cap)
{
    if (!m || !out || cap == 0) return;
    size_t pos = 0, taken = 0;
    for (const auto& line : m->logBuf) {
        if (pos + line.size() + 2 > cap) break;
        std::memcpy(out + pos, line.data(), line.size());
        pos += line.size();
        out[pos++] = '\n';
        ++taken;
    }
    out[pos] = '\0';
    m->logBuf.erase(m->logBuf.begin(),
                    m->logBuf.begin() + static_cast<std::ptrdiff_t>(taken));
}

} // extern "C"

// ---- Quadra 650 surface -----------------------------------------------------

#include "openmac/quadra.hpp"

struct OMacQ {
    openmac::QuadraMachine mac;
    std::vector<u8> audioBuf;
    std::vector<std::string> logBuf;   // drained off the hot path by omac_q_poll_log

    OMacQ(std::vector<u8> rom, uint32_t ramMB)
        : mac(std::move(rom), openmac::QuadraMachine::Config{ramMB * 1024u * 1024u}) {
        // The faults behind a bomb box. Without this the front end is deaf to
        // everything the guest does wrong: a "type 1" system error reached the
        // user with not one line in the log to say where it came from.
        // onDiag is deliberately NOT wired here -- this machine's device
        // channels run at frame rate (the SCC alone writes a thousand lines
        // across a boot) and would bury exactly the lines worth reading.
        mac.cpu().onException = [this](int vector, u32 pc) { fault(vector, pc); };
    }

    void log(const char* s) {
        if (logBuf.size() < 4000) logBuf.emplace_back(s);
    }

    // Report a guest exception. Runs INSIDE the CPU's exception dispatch, so it
    // reads register state only -- a bus read from here can fault again, or pop
    // a device FIFO, and an instrument must not change what it measures.
    void fault(int vector, u32 pc) {
        // The routine ones are the ABI: A-line is every Toolbox call, F-line is
        // the FPU's unimplemented-instruction hand-off, TRAP #n is a system
        // call. Only the ones that end a program are worth a line.
        if (vector == 10 || vector == 11 || (vector >= 32 && vector < 48)) return;
        // The ROM bus-errors on purpose while it sizes RAM and probes the
        // slots; that traffic is startup, not a failure. What the System and
        // an application do is in RAM.
        if (pc >= 0x40000000u) return;
        if (faultBudget_ <= 0) return;
        // A bomb repeats -- the handler re-runs the faulting instruction, and a
        // wild jump faults over and over -- so each site gets two lines.
        int& seen = faultSites_[pc & 0xFFFFFFu];
        if (seen >= 2) return;
        ++seen;
        --faultBudget_;
        const openmac::M68040& c = mac.cpu();
        static const char* const kName[] = {
            "reset SP", "reset PC", "access fault", "address error",
            "illegal instruction", "divide by zero", "CHK", "TRAPcc/TRAPV",
            "privilege violation", "trace"};
        // Whose fault it is: CurApName ($0910, a Str31) is the running
        // application's name -- the same one the System puts in its bomb box,
        // and empty is exactly when that box says "unknown". Low memory is
        // identity-mapped and this read goes straight to the RAM array: no
        // translation to fault in, no device to disturb.
        char who[32] = "";
        const u32 nameLen = mac.read8(0x0910u);
        for (u32 k = 0; k < nameLen && k < 31; ++k) {
            const u8 ch = mac.read8(0x0911u + k);
            who[k] = (ch >= 32 && ch < 127) ? static_cast<char>(ch) : '.';
        }
        char b[256];
        std::snprintf(b, sizeof b,
                      "[guest] %s at pc=%08X addr=%08X sr=%04X app='%s' frame=%llu",
                      vector < 10 ? kName[vector] : "exception", pc,
                      c.lastFaultAddr, c.getSR(), who,
                      static_cast<unsigned long long>(mac.frameCount()));
        log(b);
        // The registers name the pointer that was followed, and the ring names
        // the road in: a wild jump runs on through the weeds before it faults,
        // so the caller is at the far end of it, not the near one.
        std::snprintf(b, sizeof b,
                      "   d0-3 %08X %08X %08X %08X  d4-7 %08X %08X %08X %08X",
                      c.d[0], c.d[1], c.d[2], c.d[3], c.d[4], c.d[5], c.d[6],
                      c.d[7]);
        log(b);
        std::snprintf(b, sizeof b,
                      "   a0-3 %08X %08X %08X %08X  a4-7 %08X %08X %08X %08X",
                      c.a[0], c.a[1], c.a[2], c.a[3], c.a[4], c.a[5], c.a[6],
                      c.a[7]);
        log(b);
        for (int row = 0; row < 4; ++row) {
            std::string line = "   pc ring";
            for (int k = 0; k < 12; ++k) {
                char one[16];
                std::snprintf(one, sizeof one, " %08X",
                              c.recentPc(row * 12 + k));
                line += one;
            }
            log(line.c_str());
        }
        // A guest running in the exception vectors did not get there by
        // branching. The A-line dispatcher writes a trap table entry over its
        // own return address and RTSes into it without checking, so a zero
        // entry lands the guest at address 0 -- and by the time it faults, the
        // one fact that explains it is which trap had no handler. Ask while
        // the evidence is still there; it costs a scan of two tables, and only
        // when the PC says something already went badly wrong.
        if (pc < 0x1000u) {
            const std::string health = mac.trapTableHealth();
            std::size_t at = 0;
            while (at < health.size()) {
                const std::size_t nl = health.find('\n', at);
                const std::size_t end = nl == std::string::npos ? health.size() : nl;
                log(health.substr(at, end - at).c_str());
                at = end + 1;
            }
        }
    }

private:
    int faultBudget_ = 40;             // a log, not a trace
    std::map<u32, int> faultSites_;
};

extern "C" {

OMAC_API OMacQ* omac_q_create(const uint8_t* rom, size_t rom_len, uint32_t ram_mb)
{
    if (!rom || rom_len == 0) return nullptr;
    try {
        return new OMacQ(std::vector<u8>(rom, rom + rom_len), ram_mb ? ram_mb : 8u);
    } catch (...) {
        return nullptr;
    }
}

OMAC_API void omac_q_destroy(OMacQ* m) { delete m; }
OMAC_API void omac_q_reset(OMacQ* m) { if (m) m->mac.reset(); }
OMAC_API void omac_q_run_frame(OMacQ* m) { if (m) m->mac.runFrame(); }
OMAC_API int  omac_q_screen_w(OMacQ* m) { return m ? m->mac.screenWidth() : 640; }
OMAC_API int  omac_q_screen_h(OMacQ* m) { return m ? m->mac.screenHeight() : 480; }
OMAC_API void omac_q_render(OMacQ* m, uint32_t* argb) { if (m && argb) m->mac.renderScreen(argb); }

OMAC_API size_t omac_q_drain_audio(OMacQ* m, uint8_t* out, size_t cap)
{
    if (!m || !out || cap == 0) return 0;
    m->mac.drainAudio(m->audioBuf);
    const size_t n = m->audioBuf.size() < cap ? m->audioBuf.size() : cap;
    if (n) std::memcpy(out, m->audioBuf.data(), n);
    return n;
}

OMAC_API void omac_q_mouse(OMacQ* m, int dx, int dy, int button)
{
    if (m) m->mac.mouseMove(dx, dy, button != 0);
}

OMAC_API void omac_q_key(OMacQ* m, int adb_code, int down)
{
    if (m) m->mac.keyEvent(static_cast<u8>(adb_code), down != 0);
}

OMAC_API void omac_q_insert_harddisk(OMacQ* m, const uint8_t* img, size_t len, int ro)
{
    if (m && img) m->mac.insertHardDisk(std::vector<u8>(img, img + len), ro != 0);
}

OMAC_API size_t omac_q_harddisk_data(OMacQ* m, uint8_t* out, size_t cap)
{
    if (!m || !m->mac.hardDiskPresent()) return 0;
    const auto& img = m->mac.hardDiskImage();
    if (!out) return img.size();
    if (cap < img.size()) return 0;
    std::copy(img.begin(), img.end(), out);
    return img.size();
}

OMAC_API void omac_q_insert_harddisk2(OMacQ* m, const uint8_t* img, size_t len, int ro)
{
    if (m && img) m->mac.insertHardDisk2(std::vector<u8>(img, img + len), ro != 0);
}

OMAC_API void omac_q_detach_harddisk2(OMacQ* m) { if (m) m->mac.detachHardDisk2(); }

OMAC_API size_t omac_q_harddisk2_data(OMacQ* m, uint8_t* out, size_t cap)
{
    if (!m || !m->mac.hardDisk2Present()) return 0;
    const auto& img = m->mac.hardDisk2Image();
    if (!out) return img.size();
    if (cap < img.size()) return 0;
    std::copy(img.begin(), img.end(), out);
    return img.size();
}

OMAC_API int omac_q_harddisk2_booted(OMacQ* m)
{
    return (m && m->mac.secondDiskDriverResident()) ? 1 : 0;
}

OMAC_API int omac_q_unmount_harddisk2(OMacQ* m)
{
    return (m && m->mac.unmountSecondDisk()) ? 1 : 0;
}

OMAC_API int omac_q_shutdown_volumes(OMacQ* m)
{
    return (m && m->mac.shutdownVolumes()) ? 1 : 0;
}

OMAC_API size_t omac_q_floppy_writeback(OMacQ* m, uint8_t* out, size_t cap)
{
    if (!m) return 0;
    const std::vector<u8> img = m->mac.floppyForWriteBack();
    if (img.empty()) return 0;
    if (!out) return img.size();
    if (cap < img.size()) return 0;
    std::copy(img.begin(), img.end(), out);
    return img.size();
}

// Parameter RAM. Call with out = null for the size. `addSeconds` on load is how
// long the machine was switched off, so the clock carries on the way a battery
// would rather than restarting where it stopped.
OMAC_API size_t omac_q_pram_save(OMacQ* m, uint8_t* out, size_t cap)
{
    if (!m) return 0;
    return m->mac.savePram(out, static_cast<uint32_t>(cap));
}

OMAC_API int omac_q_pram_load(OMacQ* m, const uint8_t* data, size_t len,
                              uint32_t addSeconds)
{
    return m && m->mac.loadPram(data, static_cast<uint32_t>(len), addSeconds) ? 1 : 0;
}

OMAC_API size_t omac_q_diagnostics(OMacQ* m, char* out, size_t cap)
{
    if (!m) return 0;
    const std::string s = m->mac.diagnosticReport();
    if (!out || cap == 0) return s.size() + 1;
    const size_t n = s.size() < cap - 1 ? s.size() : cap - 1;
    std::copy(s.begin(), s.begin() + static_cast<std::ptrdiff_t>(n), out);
    out[n] = '\0';
    return n;
}

// Hand over as many whole lines as fit and keep the rest: a fault report is
// several lines long, and half of one in the log is worse than none.
OMAC_API void omac_q_poll_log(OMacQ* m, char* out, size_t cap)
{
    if (!m || !out || cap == 0) return;
    size_t pos = 0, taken = 0;
    for (const auto& line : m->logBuf) {
        if (pos + line.size() + 2 > cap) break;
        std::memcpy(out + pos, line.data(), line.size());
        pos += line.size();
        out[pos++] = '\n';
        ++taken;
    }
    out[pos] = '\0';
    m->logBuf.erase(m->logBuf.begin(),
                    m->logBuf.begin() + static_cast<std::ptrdiff_t>(taken));
}

OMAC_API int omac_q_insert_floppy(OMacQ* m, const uint8_t* img, size_t len, int ro)
{
    if (!m || !img) return 0;
    return m->mac.insertFloppy(std::vector<u8>(img, img + len), ro != 0);
}

OMAC_API void omac_q_eject_floppy(OMacQ* m) { if (m) m->mac.ejectFloppy(); }

OMAC_API int omac_q_floppy_present(OMacQ* m) { return m && m->mac.floppyPresent() ? 1 : 0; }

OMAC_API size_t omac_q_floppy_data(OMacQ* m, uint8_t* out, size_t cap)
{
    if (!m || !m->mac.floppyPresent()) return 0;
    const auto& img = m->mac.floppyImage();
    if (!out) return img.size();
    if (cap < img.size()) return 0;
    std::copy(img.begin(), img.end(), out);
    return img.size();
}

// The drive is a device on the bus; a disc is media in it. Nothing mounts until
// the drive is there, and this front end never had a way to put it there --
// which is why a disc chosen in the GUI did nothing at all.
OMAC_API void omac_q_attach_cd(OMacQ* m, int attached, int busId)
{
    if (m) m->mac.attachCdRom(attached != 0, busId);
}

OMAC_API int omac_q_cd_attached(OMacQ* m) { return m && m->mac.cdRomAttached() ? 1 : 0; }

OMAC_API int omac_q_insert_cd(OMacQ* m, const uint8_t* img, size_t len)
{
    if (!m || !img) return 0;
    return m->mac.insertCd(std::vector<u8>(img, img + len));
}

OMAC_API void omac_q_eject_cd(OMacQ* m) { if (m) m->mac.ejectCd(); }

OMAC_API int omac_q_cd_present(OMacQ* m) { return m && m->mac.cdPresent() ? 1 : 0; }

// The displays the built-in video port drives. Walk index upward until this
// returns null. Static, so a front end can build its menu before there is a
// machine to ask.
OMAC_API const char* omac_q_display_name(int index, int* w, int* h)
{
    int n = 0;
    const openmac::QuadraMachine::DisplayInfo* list =
        openmac::QuadraMachine::displays(n);
    if (index < 0 || index >= n) return nullptr;
    if (w) *w = list[index].width;
    if (h) *h = list[index].height;
    return list[index].name;
}

OMAC_API int omac_q_set_display(OMacQ* m, const char* name)
{
    return (m && m->mac.setDisplay(name)) ? 1 : 0;
}

} // extern "C"
