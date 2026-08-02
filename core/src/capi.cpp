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

// ---- Quadra 650 surface -----------------------------------------------------

#include "openmac/quadra.hpp"

struct OMacQ {
    openmac::QuadraMachine mac;
    std::vector<u8> audioBuf;

    OMacQ(std::vector<u8> rom, uint32_t ramMB)
        : mac(std::move(rom), openmac::QuadraMachine::Config{ramMB * 1024u * 1024u}) {}
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

} // extern "C"
