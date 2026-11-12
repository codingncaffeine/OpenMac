#include "openmac/quadra.hpp"

#include "openmac/hfs.hpp"

#include "../adb.hpp"
#include "../dc42.hpp"
#include "../macbinary.hpp"
#include "../rtc.hpp"
#include "../scsi.hpp"
#include "../cdmedia.hpp"
#include "../scsicd.hpp"
#include "../scsiimage.hpp"
#include "../scsinet.hpp"
#include "../sony.hpp"
#include "../via.hpp"
#include "dafb.hpp"
#include "easc.hpp"
#include "ncr53c96.hpp"
#include "pseudovia.hpp"
#include "scc.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <map>

// Machine wiring and address decode per the Quadra 650 hardware dossier:
// djMEMC owns the map (RAM, ROM alias/overlay, DAFB), IOSB's I/O window is
// carved out at $50000000 with the documented sub-decodes and mirrors, empty
// NuBus space bus-errors. VIA1 is the Classic's real 6522; VIA2 is IOSB's
// pseudo-VIA. The ADB modem talks through VIA1's shift register exactly like
// the Classic's transceiver, and the RTC/PRAM chip is the same 343-0042
// protocol the Classic bit-bangs, so both devices are reused as-is.

namespace openmac {

namespace {

constexpr u64 kCpuHz = 33333333;         // 68040 @ 33.33 MHz
constexpr int kSlicesPerFrame = 370;     // audio rides the slice cadence
constexpr int kCyclesPerSlice = 1498;    // ~60.15 Hz tick frame
// ~694 us from ADB shift arm to completion, as measured on the Classic.
constexpr int kAdbShiftCycles = 23150;

// The 4-bit machine-ID strap on VIA1 port A: Quadra 650 = PA6|PA4|PA1.
constexpr u8 kMachineId = 0x52;

// The ROM's .Sony DRVR (header at $4086C3E0): Prime = header + $26. The
// driver executes through the 24-bit alias as well, so both faces hook.
constexpr u32 kSonyPrime = 0x4086C406u;
constexpr u32 kSonyPrimeAlias = 0x0086C406u;

u8 bitswapNibbles(u8 v) {
    // MAC PROM bytes store the low nibble's bits reversed against the high:
    // bit order 0,1,2,3,7,6,5,4 of the source byte.
    u8 r = 0;
    const int order[8] = {0, 1, 2, 3, 7, 6, 5, 4};
    for (int i = 0; i < 8; ++i) {
        if (v & (1u << order[i])) r |= static_cast<u8>(1u << (7 - i));
    }
    return r;
}

} // namespace

QuadraMachine::QuadraMachine(std::vector<u8> rom, const Config& cfg)
    : MacHostServices(*this, cpu_, ram_),
      ram_(cfg.ramSize, 0),
      rom_(std::move(rom)),
      fd_(std::make_unique<SonyDrive>()),
      via1_(std::make_unique<Via6522>()),
      via2_(std::make_unique<PseudoVia>()),
      rtc_(std::make_unique<Rtc>()),
      adb_(std::make_unique<AdbTransceiver>()),
      dafb_(std::make_unique<Dafb>()),
      easc_(std::make_unique<Easc>()),
      scc_(std::make_unique<Scc8530>()),
      scsi_(std::make_unique<Ncr53c96>()),
      netdev_(std::make_unique<ScsiEthernet>()),
      cpu_(*this) {
    u32 rs = 1;
    while (rs < rom_.size()) rs <<= 1;
    rom_.resize(rs, 0xFF);
    romMask_ = rs - 1;
    romEntry_ = (static_cast<u32>(rom_[4]) << 24) | (static_cast<u32>(rom_[5]) << 16) |
                (static_cast<u32>(rom_[6]) << 8) | rom_[7];

    scc_->onDiag = [this](const char* s) { if (onDiag) onDiag(s); };
    // Default display: the 13-inch/14-inch RGB monitor the machine has always
    // reported. Its sense code is 6, which is one grounded line (bit 0).
    setMonitorSense(0x1, 0);
    scsi_->addTarget(disk_.get());
    scsi_->addTarget(disk2_.get());
    scsi_->addTarget(cdrom_.get());
    // This machine starts up from its CD-ROM drive: the ROM loads the driver
    // off the disc, and that driver wants an Apple drive with a tray.
    cdrom_->setStartupDrive(true);
    scsi_->addTarget(netdev_.get());

    fd_->installed = true;      // the internal SuperDrive is always fitted
    fd_->superDrive = true;
    fd_->doubleSided = true;

    // Ethernet MAC PROM: Apple OUI 08:00:07 + a fixed station id, each byte
    // nibble-bit-swizzled, byte 7 = inverted XOR checksum.
    const u8 mac[6] = {0x08, 0x00, 0x07, 0x2A, 0x65, 0x50};
    u8 x = 0;
    for (int i = 0; i < 6; ++i) {
        macProm_[i] = bitswapNibbles(mac[i]);
        x ^= macProm_[i];
    }
    macProm_[6] = 0;
    macProm_[7] = static_cast<u8>(x ^ 0xFF);

    wireDevices();
    reset();
}

QuadraMachine::QuadraMachine(std::vector<u8> rom)
    : QuadraMachine(std::move(rom), Config{}) {}

QuadraMachine::~QuadraMachine() = default;

void QuadraMachine::wireDevices() {
    via1_->inA = [] {
        // The machine-ID strap: bits 1/2/4/6 carry the board code and the
        // rest read low, so the ROM's masked read sees exactly $52.
        return kMachineId;
    };
    via1_->inB = [this] {
        u8 v = 0xFF;
        if (!rtc_->dataOut()) v = static_cast<u8>(v & ~0x01);
        if (!adb_->intLine()) v = static_cast<u8>(v & ~0x08);   // PB3, active low
        return v;
    };
    via1_->outA = [this](u8 value, u8 ddr) {
        // PA5 is the floppy mechanism's SEL line (the fourth sense-address
        // bit); the .Sony driver drives it while walking the drive status.
        const u8 eff = static_cast<u8>(value | ~ddr);
        floppySel_ = (eff & 0x20) != 0;
    };
    via1_->outB = [this](u8 value, u8 ddr) {
        const u8 eff = static_cast<u8>(value | ~ddr);
        rtc_->setLines((eff & 0x01) != 0, (eff & 0x02) != 0, (eff & 0x04) != 0);
        const int prev = adb_->state();
        adb_->setState((eff >> 4) & 3);   // PB4/PB5 = "newaction"
        if (adb_->state() != prev) {
            // Push model, as the real transceiver behaves: entering a data
            // state clocks the device's next byte ~700 us later whether or not
            // the CPU has armed the shift register. The System's PATCHED RAM
            // ADB manager (the one running once the desktop is up) does NOT
            // pre-read the SR, so an arm-driven input shift never completes
            // under it -- which is why the autopolled mouse reported motion
            // (12 reports) that never reached the guest. Listen transfers stay
            // CPU-driven: a pushed byte there would collide with the write.
            const int st = adb_->state();
            if ((st == 1 || st == 2) && !adb_->listening()) {
                adbPending_ = kAdbShiftCycles;
                adbPendingInput_ = true;
                adbArmed_ = false;
            } else if (st == 3 && adb_->responsePending()) {
                // The settled poll collects a flush byte at idle before it
                // reads the data states; serve it the same pushed way.
                adbPending_ = kAdbShiftCycles;
                adbPendingInput_ = true;
                adbArmed_ = false;
            } else if (st == 3) {
                adbArmed_ = false;   // idle, nothing staged: the transaction is over
                adbPending_ = 0;
            } else if (adbPendingInput_) {
                adbPending_ = 0;     // command window: cancel a stale input shift
            }
            adbMaybeClock();         // output shifts armed by SR writes still clock
        }
        updateIpl();
    };
    via1_->srArmed = [this](bool input) {
        // Input bytes are PUSHED by state changes (see outB); an SR read is
        // just the CPU collecting the latched byte. Only OUTPUT shifts -- the
        // CPU writing a command or Listen data -- arm a transfer here.
        if (input) return;
        adbArmed_ = true;
        adbArmedInput_ = false;
        adbMaybeClock();
    };
    via1_->srDisarmed = [this] { adbArmed_ = false; };

    via2_->outB = [this](u8 value, u8 ddr) {
        const u8 eff = static_cast<u8>(value | ~ddr);
        // DFAC 3-wire: PB0 latch, PB3 data, PB4 clock.
        easc_->dfacLines((eff & 0x01) != 0, (eff & 0x08) != 0, (eff & 0x10) != 0);
    };
    via2_->onIrq = [this](bool) { updateIpl(); };
    via2_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    via2_->ina = [this]() {
        // Port A carries the per-slot interrupt lines, ACTIVE LOW (an idle
        // bus reads $FF). The ROM's slot dispatcher reads this to learn which
        // source interrupted; internal video (the DAFB) rides bit 6. Serving
        // zeros here reads as "every slot at once" and the dispatcher can
        // neither identify nor acknowledge anything.
        return static_cast<u8>(0xFFu & ~(dafbIrq_ ? 0x40u : 0u));
    };

    dafb_->onDiag = [this](const char* s) { if (onDiag) onDiag(s); };
    dafb_->onIrq = [this](bool level) {
        dafbIrq_ = level;
        via2_->setSlotIrq(level);
    };
    easc_->onIrq = [this](bool level) { via2_->setAscIrq(level); };
    easc_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    scsi_->onIrq = [this](bool level) { via2_->setScsiIrq(level); };
    scsi_->onDrq = [this](bool level) { via2_->setScsiDrq(level); };
    scsi_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    scsi_->onCmd = [this](u8 cmd, u32 fifoLevel, u8 phase) {
        if (cdbDiagBudget_ <= 0) return;
        char b[80];
        std::snprintf(b, sizeof b, "SCMD %02X fifo=%u phase=%d", cmd, fifoLevel, phase);
        if (onDiag) onDiag(b);
    };
    scsi_->onCdb = [this](int id, const u8* cdb, int len) {
        // The 8th CDB is the probe's boot-block read; re-arm the register
        // trace there so the storm before it cannot exhaust the budget.
        if (cdbListBudget_ <= 0) return;
        --cdbListBudget_;
        char b[112];
        int n = std::snprintf(b, sizeof b, "CDB id%d:", id);
        for (int i = 0; i < len && n < 100; ++i)
            n += std::snprintf(b + n, sizeof b - static_cast<size_t>(n), " %02X", cdb[i]);
        if (onDiag) onDiag(b);
    };

    netdev_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    // With the PC: whether a command came from the ROM's boot scan or from a
    // driver in RAM is the first question about a disc that will not mount,
    // and the target itself cannot know.
    cdrom_->onDiag = [this](const char* m) {
        if (!onDiag) return;
        char b[220];
        std::snprintf(b, sizeof b, "%s (pc %08X)", m, cpu_.pc);
        onDiag(b);
    };

    cpu_.onResetInstruction = [this] {
        via1_->reset();
        via2_->reset();
        rtc_->reset();
        adb_->reset();
        scsi_->reset();
        easc_->reset();
        adbArmed_ = false;
        adbPending_ = 0;
    };
}

void QuadraMachine::reset() {
    overlay_ = true;
    for (auto& r : djmemcRegs_) r = 0;
    for (auto& r : iosbRegs_) r = 0;
    for (auto& r : sonicRegs_) r = 0;
    sonicRegs_[0x29] = 6;                 // silicon revision
    sonicRegs_[0x00] = 0x0094;            // CR: RST | STP | RXDIS
    scc_->reset();
    swimMode_ = 0x40;
    via1_->reset();
    via2_->reset();
    rtc_->reset();
    adb_->reset();
    dafb_->reset();
    easc_->reset();
    scsi_->reset();
    adbArmed_ = false;
    adbPending_ = 0;
    totalCycles_ = 0;
    via1Remainder_ = 0;
    secondAcc_ = 0;
    vblAcc_ = 0;
    bootTraceFrame_ = 0;
    cpu_.reset();
}

// A snapshot of everything that has ever mattered when this machine stopped
// responding. Every value is read from model state, never through a device
// read() -- several of those clear latches or advance a register pointer, and
// an observer that changes the machine invents its own bugs.
//
// The question a hang always comes down to is "what is the stopped thread
// waiting for", so the report leads with the CPU's recent-PC ring: a thread
// spinning on a status bit shows up as a handful of addresses repeating, and
// the device blocks below say what those addresses are polling.
std::string QuadraMachine::diagnosticReport() const {
    std::string s;
    // Wide enough to hold a device's own state line plus this function's
    // prefix without the compiler having to assume truncation.
    char b[1024];
    auto add = [&](const char* fmt, auto... args) {
        // With no arguments there is nothing to format, and passing a
        // non-literal format string to snprintf is a diagnostic in its own
        // right (-Wformat-security). Take the plain-text path instead.
        if constexpr (sizeof...(args) == 0) {
            s += fmt;
        } else {
            std::snprintf(b, sizeof b, fmt, args...);
            s += b;
        }
        s += '\n';
    };

    const M68040& c = cpu_;
    add("OpenMac Quadra 650 diagnostic snapshot");
    add("frame %llu  cycles %llu  ram %u MB",
        static_cast<unsigned long long>(frameCounter_),
        static_cast<unsigned long long>(totalCycles_),
        static_cast<unsigned>(ram_.size() / (1024 * 1024)));
    add("pc=%08X sr=%04X (%s, IPL %d)%s", c.pc, c.getSR(),
        (c.getSR() & 0x2000) ? "supervisor" : "USER",
        (c.getSR() >> 8) & 7, c.halted ? "  *** HALTED ***" : "");
    add("d0-7 %08X %08X %08X %08X %08X %08X %08X %08X", c.d[0], c.d[1], c.d[2],
        c.d[3], c.d[4], c.d[5], c.d[6], c.d[7]);
    add("a0-7 %08X %08X %08X %08X %08X %08X %08X %08X", c.a[0], c.a[1], c.a[2],
        c.a[3], c.a[4], c.a[5], c.a[6], c.a[7]);

    // The PC ring, and then the same addresses counted. A tight wait loop is
    // a few PCs with high counts; a healthy machine is 128 different ones.
    s += "\nrecent PCs (newest first)\n";
    for (int i = 0; i < 128; ++i) {
        std::snprintf(b, sizeof b, "%08X%s", c.recentPc(i),
                      (i % 8 == 7) ? "\n" : " ");
        s += b;
    }
    std::map<u32, int> hot;
    for (int i = 0; i < 128; ++i) ++hot[c.recentPc(i)];
    std::vector<std::pair<int, u32>> byCount;
    for (const auto& [pc, n] : hot) byCount.emplace_back(n, pc);
    std::sort(byCount.rbegin(), byCount.rend());
    add("\n%zu distinct addresses in the last 128 instructions%s", hot.size(),
        hot.size() <= 8 ? "  <-- looks like a wait loop" : "");
    for (std::size_t i = 0; i < byCount.size() && i < 6; ++i)
        add("   %08X  x%d", byCount[i].second, byCount[i].first);

    s += "\nlow memory\n";
    auto peek16 = [&](u32 a) { return const_cast<QuadraMachine*>(this)->read16(a); };
    auto peek32 = [&](u32 a) { return const_cast<QuadraMachine*>(this)->read32(a); };
    add("   Ticks $016A = %u", peek32(0x016A));
    add("   MemErr $0220 = %d   FSBusy $0360 = %04X",
        static_cast<s16>(peek16(0x0220)), peek16(0x0360));
    // The queue HEADS, not the flags word in front of them: DrvQHdr's head is
    // at $030A, and reading the long at $0308 answers the flags plus the top
    // half of a low-memory pointer -- which is zero on a healthy machine and
    // reads exactly like an empty queue.
    add("   VCBQHdr $0358 = %08X   DrvQHdr $030A = %08X",
        peek32(0x0358), peek32(0x030A));
    // What the guest believes it has. An application's world sitting far below
    // the fitted RAM is how "not enough memory" reaches a user on a machine
    // with plenty -- and nothing else in this snapshot would show it.
    add("   MemTop $0108 = %08X   BufPtr $010C = %08X   HeapEnd $0114 = %08X",
        peek32(0x0108), peek32(0x010C), peek32(0x0114));
    add("   ApplZone $02AA = %08X  ApplLimit $0130 = %08X  CurStackBase $0908 = %08X",
        peek32(0x02AA), peek32(0x0130), peek32(0x0908));
    add("   CurApName $0910 = '%.*s'   CurrentA5 $0904 = %08X",
        peek16(0x0910) >> 8 > 31 ? 31 : peek16(0x0910) >> 8,
        reinterpret_cast<const char*>(ram_.data() + 0x0911), peek32(0x0904));
    add("   Mouse $0830 = %d,%d   SCCRd $01D8 = %08X",
        static_cast<s16>(peek16(0x0832)), static_cast<s16>(peek16(0x0830)),
        peek32(0x01D8));

    s += "\ndevices\n";
    add("   VIA1  ifr=%02X ier=%02X irq=%d", via1_->ifr(), via1_->ier(),
        via1_->irqAsserted() ? 1 : 0);
    add("   VIA2  ifr=%02X ier=%02X irq=%d", via2_->peekIfr(), via2_->peekIer(),
        via2_->irqAsserted() ? 1 : 0);
    // What the GUEST believes its screen is. When our decode of the video
    // registers disagrees with what is on screen, this is the arbiter:
    // ScreenRow is the main screen's rowBytes and CrsrPin its bounding box,
    // both written by the System from the display it configured.
    add("   screen ScreenRow=%d ScrnBase=%08X CrsrPin=%d,%d,%d,%d",
        static_cast<s16>(peek16(0x0106)), peek32(0x0824),
        static_cast<s16>(peek16(0x0834)), static_cast<s16>(peek16(0x0836)),
        static_cast<s16>(peek16(0x0838)), static_cast<s16>(peek16(0x083A)));
    add("   DAFB  %dx%d %dbpp base=%05X stride=%u sense=%X",
        dafb_->width(), dafb_->height(), dafb_->bpp(), dafb_->fbBase(),
        dafb_->strideBytes(), dafb_->read(0x1C));
    add("   depthCtl=%02X", dafb_->depthCtlRaw());
    {
        // The first entries and the last: at low depths Apple's DACs index the
        // table by replicating the pixel across all eight bits, so 1-bit black
        // lives at 255 rather than at 1.
        std::string t = "   clut";
        char one[32];
        for (int i : {0, 1, 2, 3, 85, 170, 254, 255}) {
            std::snprintf(one, sizeof one, " %d=%06X", i, dafb_->clutEntry(i) & 0xFFFFFF);
            t += one;
        }
        s += t;
        s += '\n';
    }
    {
        // The swatch's horizontal and vertical timing: what the ROM programmed
        // from the monitor it sensed, and where the geometry above comes from.
        std::string t = "   dafbreg";
        (void)0;
        char one[32];
        for (u32 r : {0x00u, 0x04u, 0x08u, 0x0Cu, 0x10u, 0x14u, 0x18u, 0x24u, 0x28u}) {
            std::snprintf(one, sizeof one, " %02X=%X", r, dafb_->ctlReg(static_cast<int>(r >> 2)));
            t += one;
        }
        s += t;
        s += '\n';
        t = "   swatch";
        for (u32 r : {0x40u, 0x44u, 0x48u, 0x4Cu, 0x54u, 0x58u, 0x5Cu, 0x60u}) {
            std::snprintf(one, sizeof one, " %02X=%03X", r,
                          dafb_->swatchReg(static_cast<int>(r >> 2)));
            t += one;
        }
        s += t;
        s += '\n';
    }
    // A separate buffer: `add` formats into `b`, and formatting a buffer into
    // itself is undefined -- it truncated these lines mid-word.
    char dev[512];
    scc_->debugState(dev, sizeof dev);
    add("   SCC   %s", dev);
    easc_->debugState(dev, sizeof dev);
    add("   EASC  %s", dev);
    add("   disk  %u reads, %u writes served; floppy %u/%u",
        hdReadCount_, hdWriteCount_, fdReadCount_, fdWriteCount_);
    return s;
}

void QuadraMachine::updateIpl() {
    // IOSB priority order: SCC is level 4, VIA2's aggregate level 2, VIA1
    // level 1.
    int ipl = 0;
    if (scc_->irqAsserted()) ipl = 4;
    else if (via2_->irqAsserted()) ipl = 2;
    else if (via1_->irqAsserted()) ipl = 1;
    if (ipl == 2 && iplDiagBudget_ > 0) {
        --iplDiagBudget_;
        char b[96];
        std::snprintf(b, sizeof b, "IPL2 asserted: via2 ifr=%02X ier=%02X pc=%08X",
                      via2_->read(13), via2_->read(14), cpu_.pc);
        if (onDiag) onDiag(b);
    }
    cpu_.setIrqLevel(ipl);
}

// ---------------------------------------------------------------- bus decode

void QuadraMachine::logAccess(const char* what, u32 addr, bool write, u32 value) {
    if (accessLog_.size() >= 4000) return;
    char buf[96];
    std::snprintf(buf, sizeof buf, "%s %c %08X = %X (pc %08X)", what,
                  write ? 'W' : 'R', addr, value, cpu_.pc);
    accessLog_.emplace_back(buf);
}

u8 QuadraMachine::read8(u32 addr) {
    if (addr < 0x40000000u) {
        if (overlay_ && addr < 0x400000u) return rom_[addr & romMask_];
        if (addr < ram_.size()) return ram_[addr];
        return 0xFF;   // unpopulated RAM: the sizing probe reads open bus
    }
    if (addr < 0x50000000u) {
        overlay_ = false;   // first native-ROM read swaps RAM in at zero
        return rom_[addr & romMask_];
    }
    if (addr < 0x60000000u) return ioRead8(addr);
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) return dafb_->vramRead8(off);
        if (off >= 0x800000u && off < 0x800400u) {
            const u32 v = dafb_->read(off & 0x3FFu & ~3u);
            return static_cast<u8>(v >> (8 * (3 - (off & 3))));
        }
    }
    throw BusFault{addr, true, 1};
}

void QuadraMachine::write8(u32 addr, u8 value) {
    if (addr < 0x40000000u) {
        if (watchLen_ && addr - watchAddr_ < watchLen_ && watchBudget_ > 0 &&
            frameCounter_ >= watchFromFrame_ && onDiag) {
            --watchBudget_;
            char b[64];
            std::snprintf(b, sizeof b, "WATCH w8 %08X=%02X pc=%08X", addr, value, cpu_.pc);
            onDiag(b);
        }
        if (addr < ram_.size()) ram_[addr] = value;
        return;   // writes to holes vanish, as on the real bus
    }
    if (addr < 0x50000000u) return;   // ROM
    if (addr < 0x60000000u) { ioWrite8(addr, value); return; }
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) {
            if (onVramWrite) onVramWrite(off, cpu_.pc);
            dafb_->vramWrite8(off, value);
            return;
        }
        if (off >= 0x800000u && off < 0x800400u) {
            const u32 reg = off & 0x3FFu;
            if ((reg >> 8) == 2) {
                // RAMDAC: each byte write is one step of the CLUT's 3-phase
                // R,G,B cycle -- pass it through untouched (a read-modify-
                // write here would advance the read phase and mangle it).
                dafb_->write(reg & ~3u, value);
                return;
            }
            const u32 r32 = reg & ~3u;
            const int shift = 8 * (3 - (off & 3));
            const u32 old = dafb_->read(r32);
            dafb_->write(r32, (old & ~(0xFFu << shift)) |
                                  (static_cast<u32>(value) << shift));
            return;
        }
    }
    throw BusFault{addr, false, 1};
}

u16 QuadraMachine::read16(u32 addr) {
    if (addr < 0x40000000u) {
        if (overlay_ && addr < 0x400000u) {
            const u32 a = addr & romMask_;
            return static_cast<u16>((rom_[a] << 8) | rom_[(a + 1) & romMask_]);
        }
        if (addr + 1 < ram_.size())
            return static_cast<u16>((ram_[addr] << 8) | ram_[addr + 1]);
        return 0xFFFF;
    }
    if (addr < 0x50000000u) {
        overlay_ = false;
        const u32 a = addr & romMask_;
        return static_cast<u16>((rom_[a] << 8) | rom_[(a + 1) & romMask_]);
    }
    if (addr < 0x60000000u) {
        const u32 off = addr & 0x0003FFFFu;
        if (off >= 0x0A000u && off < 0x0B100u) {   // SONIC: 16-bit lanes
            return sonicRegs_[(off >> 2) & 0x3F];
        }
        // The SCSI pseudo-DMA port is a true 16-bit lane: a word read moves
        // two data bytes (the manager bursts MOVE.W/MOVE.L from it), and
        // touching it without DRQ bus-errors (the not-ready handshake).
        if (off >= 0x10100u && off < 0x10104u) {
            if (!scsi_->drq()) throw BusFault{addr, true, 2};
            if (dataInPcBudget_ > 0 && onDiag) {
                --dataInPcBudget_;
                char b[48];
                std::snprintf(b, sizeof b, "PDMArd16 pc=%08X", cpu_.pc);
                onDiag(b);
            }
            return scsi_->dma16Read();
        }
        // 8-bit peripherals answer 16-bit reads with the byte in both lanes
        // (the SCC glue's documented behavior; safe for the VIAs too since
        // it decodes the register only once).
        const u8 v = ioRead8(addr);
        return static_cast<u16>((v << 8) | v);
    }
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) {
            return static_cast<u16>((dafb_->vramRead8(off) << 8) |
                                    dafb_->vramRead8(off + 1));
        }
        if (off >= 0x800000u && off < 0x800400u) {
            const u32 v = dafb_->read(off & 0x3FFu & ~3u);
            return static_cast<u16>(v >> ((off & 2) ? 0 : 16));
        }
    }
    throw BusFault{addr, true, 2};
}

void QuadraMachine::write16(u32 addr, u16 value) {
    if (addr < 0x40000000u) {
        if (watchLen_ && addr - watchAddr_ < watchLen_ + 1 && watchBudget_ > 0 &&
            frameCounter_ >= watchFromFrame_ && onDiag) {
            --watchBudget_;
            char b[64];
            std::snprintf(b, sizeof b, "WATCH w16 %08X=%04X pc=%08X", addr, value, cpu_.pc);
            onDiag(b);
        }
        if (addr + 1 < ram_.size()) {
            ram_[addr] = static_cast<u8>(value >> 8);
            ram_[addr + 1] = static_cast<u8>(value);
        }
        return;
    }
    if (addr < 0x50000000u) return;
    if (addr < 0x60000000u) {
        const u32 off = addr & 0x0003FFFFu;
        if (off >= 0x0A000u && off < 0x0B100u) {
            sonicWrite((off >> 2) & 0x3F, value);
            return;
        }
        if (off >= 0x0C000u && off < 0x0E000u) {
            ioWrite8(addr, static_cast<u8>(value >> 8));   // SCC: high byte
            return;
        }
        // The SCSI pseudo-DMA port takes both bytes of a word write, and
        // faults when the chip isn't requesting (the not-ready handshake).
        if (off >= 0x10100u && off < 0x10104u) {
            if (!scsi_->drq()) throw BusFault{addr, false, 2};
            scsi_->dma16Write(value);
            return;
        }
        // (The EASC is an 8-bit device on IOSB's byte-steered lane: a wide
        // write delivers ONE byte, like every other 8-bit peripheral here.
        // The ROM's chime player only ever byte-writes the FIFOs anyway.)
        ioWrite8(addr, static_cast<u8>(value >> 8));
        return;
    }
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) {
            dafb_->vramWrite8(off, static_cast<u8>(value >> 8));
            dafb_->vramWrite8(off + 1, static_cast<u8>(value));
            return;
        }
        if (off >= 0x800000u && off < 0x800400u) {
            const u32 reg = off & 0x3FFu & ~3u;
            const int shift = (off & 2) ? 0 : 16;
            const u32 old = dafb_->read(reg);
            dafb_->write(reg, (old & ~(0xFFFFu << shift)) |
                                  (static_cast<u32>(value) << shift));
            return;
        }
    }
    throw BusFault{addr, false, 2};
}

u32 QuadraMachine::read32(u32 addr) {
    if (addr < 0x40000000u) {
        if (overlay_ && addr < 0x400000u) {
            const u32 a = addr & romMask_;
            return (static_cast<u32>(rom_[a]) << 24) |
                   (static_cast<u32>(rom_[(a + 1) & romMask_]) << 16) |
                   (static_cast<u32>(rom_[(a + 2) & romMask_]) << 8) |
                   rom_[(a + 3) & romMask_];
        }
        if (addr + 3 < ram_.size()) {
            return (static_cast<u32>(ram_[addr]) << 24) |
                   (static_cast<u32>(ram_[addr + 1]) << 16) |
                   (static_cast<u32>(ram_[addr + 2]) << 8) | ram_[addr + 3];
        }
        return 0xFFFFFFFFu;
    }
    if (addr < 0x50000000u) {
        overlay_ = false;
        const u32 a = addr & romMask_;
        return (static_cast<u32>(rom_[a]) << 24) |
               (static_cast<u32>(rom_[(a + 1) & romMask_]) << 16) |
               (static_cast<u32>(rom_[(a + 2) & romMask_]) << 8) |
               rom_[(a + 3) & romMask_];
    }
    if (addr < 0x60000000u) return ioRead32(addr);
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) return dafb_->vramRead32(off);
        if (off >= 0x800000u && off < 0x800400u) return dafb_->read(off & 0x3FFu);
    }
    throw BusFault{addr, true, 4};
}

void QuadraMachine::write32(u32 addr, u32 value) {
    if (addr < 0x40000000u) {
        if (watchLen_ && addr - watchAddr_ < watchLen_ + 3 && watchBudget_ > 0 &&
            frameCounter_ >= watchFromFrame_ && onDiag) {
            --watchBudget_;
            char b[72];
            std::snprintf(b, sizeof b, "WATCH w32 %08X=%08X pc=%08X", addr, value, cpu_.pc);
            onDiag(b);
        }
        if (addr + 3 < ram_.size()) {
            ram_[addr] = static_cast<u8>(value >> 24);
            ram_[addr + 1] = static_cast<u8>(value >> 16);
            ram_[addr + 2] = static_cast<u8>(value >> 8);
            ram_[addr + 3] = static_cast<u8>(value);
        }
        return;
    }
    if (addr < 0x50000000u) return;
    if (addr < 0x60000000u) { ioWrite32(addr, value); return; }
    if ((addr >> 24) == 0xF9) {
        const u32 off = addr & 0x00FFFFFFu;
        if (off < 0x200000u) {
            if (onVramWrite) onVramWrite(off, cpu_.pc);
            dafb_->vramWrite32(off, value);
            return;
        }
        if (off >= 0x800000u && off < 0x800400u) {
            dafb_->write(off & 0x3FFu, value);
            return;
        }
    }
    throw BusFault{addr, false, 4};
}

// ---- the $50000000 I/O window, mirrors folded to 256KB ----

// Names the device block an offset lands in, so a trace line reads as a
// conversation with a chip rather than as an address.
namespace {
const char* ioBlockName(u32 off) {
    if (off < 0x02000u) return "VIA1";
    if (off < 0x04000u) return "VIA2";
    if (off >= 0x08000u && off < 0x08008u) return "PROM";
    if (off >= 0x0A000u && off < 0x0B100u) return "SONIC";
    if (off >= 0x0C000u && off < 0x0E000u) return "SCC";
    if (off >= 0x0E000u && off < 0x10000u) return "DJMEMC";
    if (off >= 0x10000u && off < 0x10100u) return "SCSI";
    if (off >= 0x10100u && off < 0x10104u) return "SCSIDMA";
    if (off >= 0x14000u && off < 0x16000u) return "EASC";
    if (off >= 0x1E000u && off < 0x20000u) return "SWIM";
    return "IO";
}
} // namespace

void QuadraMachine::traceIo(u32 off, bool write, u8 v) {
    if (ioTraceBudget_ <= 0 || !onDiag) return;
    const u32 f = static_cast<u32>(frameCounter_);
    if (f < ioTraceFromFrame_ || f > ioTraceToFrame_) return;
    const u32 pc = cpu_.pc;
    if (pc < ioTracePcLo_ || pc > ioTracePcHi_) return;
    // A poll loop repeats the same few accesses millions of times, and an
    // unrolled one cycles through several addresses reading one register.
    // Any access matching one of the last few printed lines is counted, not
    // printed: a spin costs one line, and the traffic on either side of it
    // -- which is the part worth reading -- still fits in the budget.
    const u64 key = (static_cast<u64>(pc) << 32) | (off << 12) |
                    (static_cast<u64>(v) << 1) | (write ? 1u : 0u);
    for (int i = 0; i < ioRecentLen_; ++i) {
        if (ioRecent_[i] == key) { ++ioRepeatCount_; return; }
    }
    flushIoCycle();
    if (ioRecentLen_ < static_cast<int>(sizeof ioRecent_ / sizeof ioRecent_[0]))
        ioRecent_[ioRecentLen_++] = key;
    else {
        for (int i = 1; i < ioRecentLen_; ++i) ioRecent_[i - 1] = ioRecent_[i];
        ioRecent_[ioRecentLen_ - 1] = key;
    }
    --ioTraceBudget_;
    char b[96];
    std::snprintf(b, sizeof b, "IOT %-7s %05X %c %02X pc=%08X",
                  ioBlockName(off), off, write ? 'W' : 'R', v, pc);
    onDiag(b);
}

void QuadraMachine::flushIoCycle() {
    if (ioRepeatCount_ > 0 && onDiag) {
        char b[96];
        std::snprintf(b, sizeof b, "IOT   ... repeats of the last %d line(s) x%u",
                      ioRecentLen_, ioRepeatCount_);
        onDiag(b);
    }
    ioRepeatCount_ = 0;
    // The window of recent accesses SURVIVES the flush: an unrolled poll
    // reads one register from several addresses in turn, and clearing here
    // would print a fresh line for each pass instead of counting the cycle.
}

u8 QuadraMachine::ioRead8(u32 addr) {
    const u8 v = ioRead8Impl(addr);
    if (ioTraceBudget_ > 0) traceIo(addr & 0x0003FFFFu, false, v);
    return v;
}

void QuadraMachine::ioWrite8(u32 addr, u8 v) {
    if (ioTraceBudget_ > 0) traceIo(addr & 0x0003FFFFu, true, v);
    ioWrite8Impl(addr, v);
}

u8 QuadraMachine::ioRead8Impl(u32 addr) {
    if ((addr & 0x00FF0000u) == 0x00FF0000u) {
        // IOSB chip id, every read in the top 64KB window.
        const u32 v = 0xA55A2BADu;
        return static_cast<u8>(v >> (8 * (3 - (addr & 3))));
    }
    const u32 off = addr & 0x0003FFFFu;
    if (off < 0x02000u) {
        const int vreg = (off >> 9) & 0xF;
        const u8 v = via1_->read(vreg);
        if (vreg == 10 && adbSrTraceBudget_ > 0) {
            --adbSrTraceBudget_;
            char b[48];
            std::snprintf(b, sizeof b, "SRread pc=%08X v=%02X", cpu_.pc, v);
            if (onDiag) onDiag(b);
        }
        return v;
    }
    if (off < 0x04000u) return via2_->read((off >> 9) & 0xF);
    if (off >= 0x08000u && off < 0x08008u) return macProm_[off & 7];
    if (off >= 0x0A000u && off < 0x0B100u) {
        const u16 v = sonicRegs_[(off >> 2) & 0x3F];
        return static_cast<u8>((off & 1) ? v : (v >> 8));
    }
    if (off >= 0x0C000u && off < 0x0E000u) {   // SCC
        // Ports at base+0/2/4/6 (ctl B, ctl A, data B, data A), aliased
        // through the window. SCCRd/SCCWr point at $50F0C020.
        return scc_->read(off & 6);
    }
    if (off >= 0x0E000u && off < 0x10000u) {
        const u32 v = djmemcRegs_[(off >> 2) & 0xF];
        return static_cast<u8>(v >> (8 * (3 - (off & 3))));
    }
    if (off >= 0x10000u && off < 0x10100u) {
        const int sreg = (off >> 4) & 0xF;
        const u8 v = scsi_->read(sreg);
        if (sreg == 2 && dataInPcBudget_ > 0 && onDiag) {
            --dataInPcBudget_;
            char b[48];
            std::snprintf(b, sizeof b, "FIFOrd pc=%08X v=%02X", cpu_.pc, v);
            onDiag(b);
        }
        if (scsiDiagBudget_ > 0) {
            --scsiDiagBudget_;
            logAccess("SCSI", addr, false, v);
        }
        return v;
    }
    if (off >= 0x10100u && off < 0x10104u) {
        // Pseudo-DMA handshake: touching the port without DRQ is a real
        // bus error on this hardware -- the fault IS the not-ready signal,
        // and the SCSI Manager paces its blind transfers on it.
        if (!scsi_->drq()) throw BusFault{addr, true, 1};
        // A byte-wide pseudo-DMA read hands over exactly one byte.
        if (dataInPcBudget_ > 0 && onDiag) {
            --dataInPcBudget_;
            char b[48];
            std::snprintf(b, sizeof b, "PDMArd8 pc=%08X", cpu_.pc);
            onDiag(b);
        }
        return scsi_->dma8Read();
    }
    if (off >= 0x14000u && off < 0x15000u) {
        const u8 v = easc_->read(off & 0xFFF);
        if ((off & 0xFFF) >= 0x800 && eascDiagBudget_ > 0) {
            --eascDiagBudget_;
            logAccess("EASC", addr, false, v);
        }
        return v;
    }
    if (off >= 0x18000u && off < 0x1A000u) {
        const u16 v = iosbRegs_[(off >> 2) & 0x1F];
        return static_cast<u8>((off & 1) ? v : (v >> 8));
    }
    if (off >= 0x1E000u && off < 0x20000u) {   // SWIM2 stub
        const int reg = static_cast<int>((off >> 9) & 7);
        u8 rv;
        switch (reg) {
        case 2: rv = 0; break;                 // error, reads clear
        case 3: rv = swimParams_[swimParamPtr_++ & 15]; break;
        case 4: rv = swimPhases_; break;       // presence probe reads back
        case 5: rv = swimSetup_; break;
        case 6: rv = swimMode_; break;
        case 7: {
            // Handshake: the drive's multiplexed sense line answers on bit 3.
            // Sense address = CA2:CA1:CA0:SEL -- CA lines from the phases
            // register's driven nibble, SEL from VIA1 PA5.
            const int sense = ((swimPhases_ & 4) << 1) | ((swimPhases_ & 2) << 1) |
                              ((swimPhases_ & 1) << 1) | (floppySel_ ? 1 : 0);
            // The mode register's devsel (bits 1-2, gated by motor-on bit 7)
            // names which drive is listening. This machine has one mechanism:
            // a probe of the second position (mode $84, measured) must float
            // the line high -- there is nothing there to pull it low --
            // or the ROM registers a phantom drive 2 and the System mounts
            // the same floppy twice. Probes with motor-on CLEAR (mode $18,
            // measured) are how the internal drive is found: those keep
            // answering from the mechanism.
            const bool drive2 = (swimMode_ & 0x80) && ((swimMode_ >> 1) & 3) == 2;
            const bool line = drive2 ? true : fd_->sense(sense, totalCycles_);
            // SWITCHED (address 6) is the one ACTIVE-HIGH status line: a high
            // answer tells the poll "this drive's disk just changed", and the
            // System flushes + ejects on it. Log every high reading.
            if ((sense & 0xF) == 0x6 && line && onDiag && cmdDiagBudget_ > 0) {
                --cmdDiagBudget_;
                char b[96];
                std::snprintf(b, sizeof b,
                              "SWITCHED high dev=%d d2=%d ph=%02X pc=%08X f=%u",
                              (swimMode_ & 0x80) ? (swimMode_ >> 1) & 3 : 0,
                              drive2 ? 1 : 0, swimPhases_, cpu_.pc,
                              static_cast<unsigned>(frameCounter_));
                onDiag(b);
            }
            rv = static_cast<u8>(0xF7 | (line ? 0x08 : 0));
            break;
        }
        default: rv = 0xFF; break;             // empty FIFO / floating
        }
        if (swimDiagBudget_ > 0) {
            --swimDiagBudget_;
            char b[80];
            std::snprintf(b, sizeof b, "SWIM R reg%d=%02X mode=%02X ph=%02X pc=%08X",
                          reg, rv, swimMode_, swimPhases_, cpu_.pc);
            if (onDiag) onDiag(b);
        }
        return rv;
    }
    logAccess("IO?", addr, false, 0);
    return 0xFF;
}

void QuadraMachine::ioWrite8Impl(u32 addr, u8 v) {
    if ((addr & 0x00FF0000u) == 0x00FF0000u) return;
    const u32 off = addr & 0x0003FFFFu;
    if (off < 0x02000u) { via1_->write((off >> 9) & 0xF, v); updateIpl(); return; }
    if (off < 0x04000u) {
        const int reg = (off >> 9) & 0xF;
        if ((reg == 13 || reg == 14) && via2DiagBudget_ > 0) {
            --via2DiagBudget_;
            char b[96];
            std::snprintf(b, sizeof b, "VIA2 reg%d W %02X (pc %08X)", reg, v, cpu_.pc);
            if (onDiag) onDiag(b);
        }
        via2_->write(reg, v);
        updateIpl();
        return;
    }
    if (off >= 0x0C000u && off < 0x0E000u) {
        scc_->write(off & 6, v);
        updateIpl();
        return;
    }
    if (off >= 0x0E000u && off < 0x10000u) {
        // djMEMC: echo what was written (bank config readback matters).
        u32& r = djmemcRegs_[(off >> 2) & 0xF];
        const int shift = 8 * (3 - (off & 3));
        r = (r & ~(0xFFu << shift)) | (static_cast<u32>(v) << shift);
        return;
    }
    if (off >= 0x10000u && off < 0x10100u) {
        const u32 reg = (off >> 4) & 0xF;
        if (reg <= 4 && cdbDiagBudget_ > 0) {
            --cdbDiagBudget_;
            char b[64];
            std::snprintf(b, sizeof b, "SREG%u W %02X", reg, v);
            if (onDiag) onDiag(b);
        }
        scsi_->write(static_cast<int>(reg), v);
        updateIpl();
        return;
    }
    if (off >= 0x10100u && off < 0x10104u) {
        if (!scsi_->drq()) throw BusFault{addr, false, 1};
        if (cdbDiagBudget_ > 0) {
            --cdbDiagBudget_;
            char b[48];
            std::snprintf(b, sizeof b, "SDMA8 W %02X", v);
            if (onDiag) onDiag(b);
        }
        scsi_->dma16Write(static_cast<u16>((v << 8) | v));
        return;
    }
    if (off >= 0x14000u && off < 0x15000u) { easc_->write(off & 0xFFF, v); return; }
    if (off >= 0x18000u && off < 0x1A000u) {
        u16& r = iosbRegs_[(off >> 2) & 0x1F];
        if (off & 1) r = static_cast<u16>((r & 0xFF00) | v);
        else r = static_cast<u16>((r & 0x00FF) | (v << 8));
        return;
    }
    if (off >= 0x1E000u && off < 0x20000u) {
        const int reg = static_cast<int>((off >> 9) & 7);
        if (swimDiagBudget_ > 0) {
            --swimDiagBudget_;
            char b[64];
            std::snprintf(b, sizeof b, "SWIM W reg%d=%02X pc=%08X", reg, v, cpu_.pc);
            if (onDiag) onDiag(b);
        }
        switch (reg) {
        case 3: swimParams_[swimParamPtr_++ & 15] = v; break;
        case 4: {
            // Phase lines: low nibble = CA0-CA2 + LSTRB levels, HIGH nibble =
            // the per-line output enables (MAME applefdintf update_phases: a
            // line is driven only when its enable bit is set). A command
            // strobe exists only when LSTRB is DRIVEN high: the ROM's
            // presence probe writes $0F -- all levels high, every driver
            // disabled -- and a latch that ignores the enables reads that as
            // EJECT (CA1=CA0=1, data=1) and throws the boot disk out the
            // moment eject requests are actually consumed.
            const u8 prev = swimPhasePrev_;
            swimPhases_ = v;
            const u8 driven = static_cast<u8>(v & (v >> 4) & 0x0F);
            swimPhasePrev_ = driven;
            if ((driven & 0x08) && !(prev & 0x08)) {
                const int reg35 = ((driven & 2) << 1) | ((driven & 1) << 1) |
                                  (floppySel_ ? 1 : 0);
                // A command reaches only the drive whose ENABLE is asserted:
                // devsel (mode bits 1-2, gated by motor-on) must name the
                // internal mechanism. The ROM's power-on phase walk strobes
                // an EJECT shape with NO drive selected (mode $40, measured)
                // and the driver's init ejects the absent second drive (mode
                // $84, measured) -- on the metal neither reaches our drive,
                // and a model that delivers them throws out the boot disk.
                const int devsel =
                    (swimMode_ & 0x80) ? (swimMode_ >> 1) & 3 : 0;
                if (onDiag && cmdDiagBudget_ > 0 &&
                    (reg35 == 6 || reg35 == 1 || devsel != 1)) {
                    --cmdDiagBudget_;
                    char b[112];
                    std::snprintf(b, sizeof b,
                                  "CMD strobe reg=%d data=%d pc=%08X mode=%02X sel=%d dev=%d f=%u%s",
                                  reg35, (driven & 4) ? 1 : 0, cpu_.pc, swimMode_,
                                  floppySel_ ? 1 : 0, devsel,
                                  static_cast<unsigned>(frameCounter_),
                                  devsel == 1 ? "" : " DROPPED");
                    onDiag(b);
                }
                if (devsel == 1)
                    fd_->command(reg35, (driven & 4) != 0, totalCycles_);
            }
            break;
        }
        case 5: swimSetup_ = v; break;
        case 6:
            swimMode_ = static_cast<u8>(swimMode_ & ~v);   // mode clear
            fd_->setEnabled((swimMode_ & 0x80) && ((swimMode_ >> 1) & 3) == 1,
                            totalCycles_);
            break;
        case 7:
            swimMode_ = static_cast<u8>(swimMode_ | v);    // mode set
            fd_->setEnabled((swimMode_ & 0x80) && ((swimMode_ >> 1) & 3) == 1,
                            totalCycles_);
            break;
        default: break;
        }
        return;
    }
    logAccess("IO?", addr, true, v);
}

u32 QuadraMachine::ioRead32(u32 addr) {
    if ((addr & 0x00FF0000u) == 0x00FF0000u) return 0xA55A2BADu;
    const u32 off = addr & 0x0003FFFFu;
    if (off >= 0x0E000u && off < 0x10000u) return djmemcRegs_[(off >> 2) & 0xF];
    if (off >= 0x10100u && off < 0x10104u) {
        if (!scsi_->drq()) throw BusFault{addr, true, 4};
        const u32 hi = scsi_->dma16Read();
        const u32 lo = scsi_->dma16Read();
        return (hi << 16) | lo;
    }
    if (off >= 0x18000u && off < 0x1A000u) return iosbRegs_[(off >> 2) & 0x1F];
    if (off >= 0x0A000u && off < 0x0B100u) return sonicRegs_[(off >> 2) & 0x3F];
    // Everything else decomposes into a single device access replicated.
    const u16 hi = read16(addr);
    const u16 lo = read16(addr + 2);
    return (static_cast<u32>(hi) << 16) | lo;
}

void QuadraMachine::ioWrite32(u32 addr, u32 v) {
    if ((addr & 0x00FF0000u) == 0x00FF0000u) return;
    const u32 off = addr & 0x0003FFFFu;
    if (off >= 0x0E000u && off < 0x10000u) {
        djmemcRegs_[(off >> 2) & 0xF] = v;
        return;
    }
    if (off >= 0x10100u && off < 0x10104u) {
        if (!scsi_->drq()) throw BusFault{addr, false, 4};
        if (cdbDiagBudget_ > 0) {
            --cdbDiagBudget_;
            char b[48];
            std::snprintf(b, sizeof b, "SDMA32 W %08X", v);
            if (onDiag) onDiag(b);
        }
        scsi_->dma16Write(static_cast<u16>(v >> 16));
        scsi_->dma16Write(static_cast<u16>(v));
        return;
    }
    if (off >= 0x18000u && off < 0x1A000u) {
        iosbRegs_[(off >> 2) & 0x1F] = static_cast<u16>(v);
        return;
    }
    if (off >= 0x0A000u && off < 0x0B100u) {
        sonicWrite((off >> 2) & 0x3F, static_cast<u16>(v));
        return;
    }
    write16(addr, static_cast<u16>(v >> 16));
    write16(addr + 2, static_cast<u16>(v));
}

void QuadraMachine::sonicWrite(u32 reg, u16 v) {
    if (reg == 0) {
        // CR: writing RST low releases reset; ST/STP/RXEN bits echo.
        if (!(v & 0x80)) v = static_cast<u16>(v & ~0x80);
        sonicRegs_[0] = v;
        return;
    }
    sonicRegs_[reg & 0x3F] = v;
}

// ---------------------------------------------------------------- time

void QuadraMachine::adbMaybeClock() {
    const int st = adb_->state();
    const bool canClock = st != 3 || adb_->responsePending();
    if (canClock && adbArmed_ && adbPending_ == 0) {
        adbArmed_ = false;
        adbPending_ = kAdbShiftCycles;
        adbPendingInput_ = adbArmedInput_;
    }
}

void QuadraMachine::tickDevices(int cpuCycles) {
    totalCycles_ += static_cast<u64>(cpuCycles);
    if (adbPending_ > 0) {
        adbPending_ -= cpuCycles;
        if (adbPending_ <= 0) {
            adbPending_ = 0;
            if (adbPendingInput_) {
                const u8 v = adb_->cpuShiftIn();
                via1_->completeShift(true, v);
            } else {
                const u8 v = via1_->shiftValue();
                adb_->cpuShiftOut(v);
                via1_->completeShift(false, 0);
            }
            updateIpl();
        }
    }
    // VIA1's 783.36 kHz clock is CPU/42.55 here (the Classic's was CPU/10);
    // accumulate in hundredths so the ratio holds over time.
    via1Remainder_ += cpuCycles * 100;
    while (via1Remainder_ >= 4255) {
        via1_->tick(1);
        via1Remainder_ -= 4255;
    }
    scsi_->tick(cpuCycles);
    scc_->tick(static_cast<s32>(cpuCycles));
    // (The eject request is taken at the frame boundary below, where the medium
    // is KEPT for the write-back. A second consumer here would race that one and
    // whichever won would decide whether the guest's writes survived -- this one
    // dropped them on the floor.)
    secondAcc_ += static_cast<u64>(cpuCycles);
    if (secondAcc_ >= kCpuHz) {
        secondAcc_ -= kCpuHz;
        rtc_->tickSecond();
        via1_->setCA2(true);
        ca2PulseSlices_ = 2;
        storageSecondTick();
    }
    // The whole machine cadence rides device time so that frame-driven and
    // single-stepped execution behave identically: audio samples per slice,
    // the 60.15 Hz VIA1 CA1 tick, the ADB idle wake, and the DAFB vertical
    // blank all come from this accumulator.
    audioAcc_ += cpuCycles;
    while (audioAcc_ >= kCyclesPerSlice) {
        audioAcc_ -= kCyclesPerSlice;
        const u8 sample = easc_->pullSample();
        if (audioOut_.size() < 8192) audioOut_.push_back(sample);

        const int slice = sliceInFrame_;
        if (slice == 0) via1_->setCA1(true);    // tick, rising edge
        if (slice == 8) via1_->setCA1(false);
        if (ca2PulseSlices_ > 0 && --ca2PulseSlices_ == 0) via1_->setCA2(false);
        if (slice == 4) adbIdleWake();

        if (++sliceInFrame_ >= kSlicesPerFrame) {
            sliceInFrame_ = 0;
            ++frameCounter_;
            storageFrameTick();
            // The eject is a mechanism on its own clock, not a strobe (the
            // Classic's lesson): advance it, and when it completes take the
            // medium out so the driver's next status poll finds the drive
            // empty. Without this a guest-commanded eject never finishes and
            // an installer can never ask for the next disk of a set.
            completeFloppyEject();
            fd_->tickEject(totalCycles_);
            if (fd_->takeEjectRequest()) {
                if (onDiag) {
                    char b[96];
                    std::snprintf(b, sizeof b,
                                  "floppy: guest ejected the disk (frame %u)",
                                  static_cast<unsigned>(frameCounter_));
                    onDiag(b);
                }
                fd_->removeDisk();
                // Ejecting does not destroy the disk: keep the bytes so the
                // host can still write back everything the guest changed.
                floppyEjected_ = std::move(floppy_);
                floppy_.clear();
            }
            // A disk the front end handed in while the drive was still full
            // goes in now that it is empty -- whichever eject emptied it.
            seatPendingFloppy();
            // DAFB VBL at ~66.67 Hz against the 60.15 Hz tick frame.
            vblAcc_ += 6667;
            dafb_->vblank();
            if (vblAcc_ >= 6015 * 2) {
                vblAcc_ -= 6015;
                dafb_->vblank();
            }
            vblAcc_ %= 6015;
        }
    }
    updateIpl();
}

void QuadraMachine::adbIdleWake() {
    const u32 polls = adb_->mousePolls() + adb_->kbdPolls();
    if (polls != adbLastPollTotal_) {
        adbLastPollTotal_ = polls;
        adbWakeStreak_ = 0;
    }
    if (adbPending_ == 0 && adb_->state() == 3 && adb_->wakeWorthPoking()) {
        if (adbWakeStreak_ < 32) {
            adb_->reStageLastTalk();
            via1_->completeShift(true, 0xFF);
            ++adbWakeStreak_;
        } else {
            adb_->flushStaleInput();
            adbWakeStreak_ = 0;
        }
    } else {
        adbWakeStreak_ = 0;
    }
}

// The ROM's .Sony Prime, served from the image at driver level: the real
// routine would run the SWIM2's MFM engine; this one moves the bytes and
// returns through jIODone exactly as the hardware path would have.
void QuadraMachine::servePrime() {
    // The System passes these with Memory Manager flag bits riding the high
    // byte; guestPtr drops them without truncating a real address (see there).
    const u32 pb = guestPtr(cpu_.a[0]);
    const u32 dce = guestPtr(cpu_.a[1]);
    const u16 trap = read16(pb + 0x06);
    // The buffer pointer arrives with Memory Manager flag bits riding the
    // high byte when the caller's heap manages 24-bit master pointers -- the
    // 7.5 Installer's does. Serving it raw bus-faults on the flag, the read
    // completes ioErr, and the Installer cancels the whole installation
    // ("leaving your disk untouched"). The HD hook already stripped it.
    const u32 buf = guestPtr(read32(pb + 0x20));
    const u32 req = read32(pb + 0x24);
    const u16 posMode = read16(pb + 0x2C);
    const u32 posOff = read32(pb + 0x2E);
    const u32 dctlPos = read32(dce + 0x10);

    // All four positioning modes, not just the two the File Manager usually
    // sends: a request that means "from the mark" is not the same as "at the
    // mark", and serving it as the latter reads the wrong part of the disk.
    u32 pos = dctlPos;
    switch (posMode & 3) {
    case 1: pos = posOff; break;                            // fsFromStart
    case 2: pos = static_cast<u32>(floppy_.size()) + posOff; break;   // fsFromLEOF
    case 3: pos = dctlPos + posOff; break;                  // fsFromMark
    default: break;                                         // fsAtMark
    }
    s16 result = 0;
    u32 done = 0;
    if (onDiag && primeDiagBudget_ > 0) {
        --primeDiagBudget_;
        char b[96];
        std::snprintf(b, sizeof b, "PRIME %s pos=%u req=%u buf=%08X",
                      (trap & 1) ? "write" : "read", pos, req, buf);
        onDiag(b);
    }
    if (!fd_->hasDisk()) {
        result = -65;                              // offLinErr
    } else if ((trap & 1) && fd_->readOnly) {
        result = -44;                              // wPrErr
    } else {
        const u32 size = static_cast<u32>(floppy_.size());
        if (pos >= size) {
            result = -39;                          // eofErr
        } else {
            u32 n = req;
            if (pos + n > size) { n = size - pos; result = -39; }
            if (trap & 1) {
                for (u32 i = 0; i < n; ++i) floppy_[pos + i] = read8(buf + i);
            } else {
                for (u32 i = 0; i < n; ++i) write8(buf + i, floppy_[pos + i]);
            }
            done = n;
        }
    }
    if (trap & 1) ++fdWriteCount_; else ++fdReadCount_;
    if (result != 0 && onDiag && hdErrBudget_ > 0) {
        --hdErrBudget_;
        char b[128];
        std::snprintf(b, sizeof b,
                      "floppy %s FAILED err=%d pos=%u req=%u mode=%u (size=%u)",
                      (trap & 1) ? "write" : "read", result, pos, req, posMode,
                      static_cast<u32>(floppy_.size()));
        onDiag(b);
    }
    write32(pb + 0x28, done);                      // ioActCount
    write32(dce + 0x10, pos + done);               // dCtlPosition
    write16(pb + 0x10, static_cast<u16>(result));  // ioResult (IODone rewrites)
    cpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    // Return through jIODone: A1 = DCE, D0 = result, jump via ($08FC).
    const u32 target = read32(0x08FC);
    cpu_.a[0] = target;
    cpu_.pc = target;
}

int QuadraMachine::stepInstruction() {
    if (cpu_.pc == 0x4080000Au || (overlay_ && cpu_.pc == romEntry_)) machineStarting();
    if (countPc_ && (cpu_.pc | 0x40000000u) == (countPc_ | 0x40000000u)) ++countPcHits_;
    if (serveStorageHooks(frameCounter_)) {
        tickDevices(40);
        return 40;
    }
    // The ROM's .Sony driver runs through both its 32-bit face and the 24-bit
    // alias; hook Prime at either and serve the request from the image.
    if (cpu_.pc == kSonyPrime || cpu_.pc == kSonyPrimeAlias) {
        try {
            servePrime();
        } catch (const BusFault& f) {
            // A malformed parameter block must not take the emulator down;
            // answer as a driver would and let the caller cope. Say what
            // faulted: an ioErr here fails whatever the guest was doing with
            // the floppy, and silence cost a day's hunt once already.
            if (onDiag) {
                char b[112];
                std::snprintf(b, sizeof b,
                              "sony prime FAULT pb=%08X dce=%08X addr=%08X (f %u)",
                              cpu_.a[0], cpu_.a[1], f.addr,
                              static_cast<unsigned>(frameCounter_));
                onDiag(b);
            }
            cpu_.d[0] = static_cast<u32>(-36);   // ioErr
            cpu_.pc = read32(0x08FC);
            cpu_.a[0] = cpu_.pc;
        }
        tickDevices(40);
        return 40;
    }
    const int c = cpu_.step();
    tickDevices(c);
    return c;
}

void QuadraMachine::runFrame() {
    // One 60.15 Hz frame's worth of machine time; the cadence itself lives
    // in tickDevices so stepping and frame-running are identical.
    const u64 target = totalCycles_ +
                       static_cast<u64>(kSlicesPerFrame) * kCyclesPerSlice;
    while (totalCycles_ < target) {
        stepInstruction();
        if (cpu_.halted) return;
    }

    if (bootTraceFrame_ < 3000) {
        if (onDiag && (bootTraceFrame_ % 60) == 0) {
            char b[128];
            std::snprintf(b, sizeof b,
                          "boot f=%u pc=%08X sr=%04X ov=%d scsiW=%u scr=%dx%d%s",
                          bootTraceFrame_, cpu_.pc, cpu_.getSR(), overlay_ ? 1 : 0,
                          scsi_->diagWrites, screenWidth(), screenHeight(),
                          cpu_.halted ? " HALTED" : "");
            onDiag(b);
        }
        ++bootTraceFrame_;
    }
}

// ---------------------------------------------------------------- frontend

QuadraMachine::ScsiDiag QuadraMachine::scsiDiag() const {
    ScsiDiag d{};
    d.writes = scsi_->diagWrites;
    d.selects = scsi_->diagSelects;
    d.commands = scsi_->diagCommands;
    d.lastCdbLen = scsi_->lastCdbLen;
    for (int i = 0; i < 12; ++i) d.lastCdb[i] = scsi_->lastCdb[i];
    return d;
}

int QuadraMachine::screenWidth() const { return dafb_->width(); }
int QuadraMachine::screenHeight() const { return dafb_->height(); }
void QuadraMachine::renderScreen(u32* argbOut) const { dafb_->render(argbOut); }

void QuadraMachine::drainAudio(std::vector<u8>& out) {
    out = std::move(audioOut_);
    audioOut_.clear();
}

void QuadraMachine::mouseMove(int dx, int dy, bool button) {
    adb_->injectMouse(dx, dy, button);
}

void QuadraMachine::keyEvent(u8 adbCode, bool down) {
    adb_->injectKey(adbCode, down);
}

u32 QuadraMachine::adbMousePolls() const { return adb_->mousePolls(); }
u32 QuadraMachine::adbKbdPolls() const { return adb_->kbdPolls(); }
u32 QuadraMachine::adbMouseReports() const { return adb_->mouseReports(); }
bool QuadraMachine::dafbVblEnabled() const { return dafb_->vblIntEnabled(); }
u32 QuadraMachine::dafbSwatchReg(int i) const { return dafb_->swatchReg(i); }
void QuadraMachine::forceVideoMode(int w, int h, int bpp) { dafb_->forceMode(w, h, bpp); }

// The displays this video port can drive, as the developer note's table of
// maximum pixel depths lists them, each with the sense wiring it presents.
// A display grounds the sense lines that read 0 in Apple's three-bit code
// (sense 2, 1, 0): 12-inch RGB is 010, 13-inch 110, the color portrait
// display 101, the 21-inch color display 000; the 16- and 19-inch displays
// ground nothing and tie two lines together. Each was checked against the
// raster the ROM then programs.
//
// Two codes that look right and are not: 100 (lines 0 and 1 grounded) is an
// NTSC monitor, an interlaced 640x480 raster in which the ROM centers a
// 512x384 desktop; and 011 (line 2 grounded) is the monochrome two-page
// display, 1152x870 in grays only.
const QuadraMachine::DisplayInfo* QuadraMachine::displays(int& count) {
    static const DisplayInfo kList[] = {
        {"12-inch RGB",       512,  384, 0x5, 0},
        {"13-inch RGB",       640,  480, 0x1, 0},
        {"15-inch Portrait",  640,  870, 0x2, 0},
        {"16-inch Color",     832,  624, 0x0, 4},
        {"19-inch Color",    1024,  768, 0x0, 1},
        {"21-inch Color",    1152,  870, 0x7, 0},
    };
    count = static_cast<int>(sizeof kList / sizeof kList[0]);
    return kList;
}

bool QuadraMachine::setDisplay(const char* name) {
    int n = 0;
    const DisplayInfo* list = displays(n);
    for (int i = 0; i < n; ++i) {
        if (!name || std::strcmp(list[i].name, name) != 0) continue;
        Dafb::MonitorWiring m;
        m.grounded = static_cast<u8>(list[i].grounded);
        m.pairs = static_cast<u8>(list[i].pairs);
        m.width = list[i].width;
        m.height = list[i].height;
        dafb_->setMonitor(m);
        return true;
    }
    return false;
}

void QuadraMachine::setMonitorSense(u32 grounded, u32 pairs) {
    Dafb::MonitorWiring m;
    m.grounded = static_cast<u8>(grounded & 7u);
    m.pairs = static_cast<u8>(pairs & 7u);
    // Adopt the raster of whichever listed display presents this wiring, so
    // a sense set by hand still renders at the right size.
    int n = 0;
    const DisplayInfo* list = displays(n);
    for (int i = 0; i < n; ++i) {
        if (list[i].grounded != static_cast<int>(m.grounded) ||
            list[i].pairs != static_cast<int>(m.pairs))
            continue;
        m.width = list[i].width;
        m.height = list[i].height;
        break;
    }
    dafb_->setMonitor(m);
}
u32 QuadraMachine::adbMouseBytesRead() const { return adb_->mouseBytesRead(); }
std::vector<u8> QuadraMachine::adbMouseBytesLog() const { return adb_->mouseBytesLog(); }
void QuadraMachine::adbClearCmdTrace() { adb_->clearCmdTrace(); }
std::vector<u8> QuadraMachine::adbCmdTrace() const { return adb_->cmdTrace(); }

// A Macintosh has one slot per drive. A disk handed in while another one is
// still in it used to replace the bytes underneath a mounted volume: everything
// the guest had written to the outgoing disk went with it (nothing kept it for
// the write-back), and the System went on believing the volume it had was still
// there -- reading a catalog that now belongs to a different disk. From the
// outside that is "the machine keeps asking me for a disk".
//
// So take the disk that is in there out first, properly -- flush, unmount,
// release the medium -- and seat the new one when the drive is actually empty.
// The eject has to happen at a frame boundary (it runs guest code), so the new
// disk waits in `floppyPending_` until then.
int QuadraMachine::insertFloppy(std::vector<u8> image, bool readOnly) {
    if (fd_->hasDisk() || floppyEjectPending_) {
        // Check the geometry now, on a copy, so the front end still gets an
        // immediate yes or no: the peel is destructive and the original has to
        // survive the wait.
        std::vector<u8> probe = image;
        if (!floppyGeometry(probe)) return 0;
        floppyPending_ = std::move(image);
        floppyPendingRO_ = readOnly;
        floppyPendingSet_ = true;
        floppyPendingDelay_ = 60;   // ~1s of empty drive before the next disk
        if (onDiag)
            onDiag("floppy: a disk is still in the drive -- taking it out first");
        ejectFloppy();
        return 1;
    }
    seatFloppy(std::move(image), readOnly);
    return floppyPresent() ? 1 : 0;
}

// Peel the containers and answer whether what is left is floppy media. Shared
// by the seat path and the "will this be accepted?" check above, so a disk that
// waits for the drive is judged by exactly the same rule as one that goes
// straight in.
bool QuadraMachine::floppyGeometry(std::vector<u8>& image) {
    if (macbinary::isMacBinary(image)) macbinary::split(image);
    if (dc42::isDiskCopy(image)) dc42::split(image);
    const std::size_t n = image.size();
    return n == 1474560 || n == 819200 || n == 409600;
}

void QuadraMachine::seatFloppy(std::vector<u8> image, bool readOnly) {
    insertFloppyNow(std::move(image), readOnly);
}

// Seat whatever was waiting for the drive to empty. Called at every frame edge,
// so it also runs the settle the swap needs.
//
// ⛔ The new disk must NOT go in on the same frame the old one came out. The
// driver learns a drive is empty by polling it, and the front end learns the
// same way -- so a medium that appears in the same instant is never seen to
// have left: the System goes on believing the drive is empty (measured: drive
// queue inPlace 0 with a disk in the drive, nothing mounts), and the host never
// gets its window to write the outgoing disk back to its file. A second of
// settling is also what taking one disk out and pushing the next one in
// actually looks like.
void QuadraMachine::seatPendingFloppy() {
    if (!floppyPendingSet_ || fd_->hasDisk() || floppyEjectPending_) return;
    if (floppyPendingDelay_ > 0) { --floppyPendingDelay_; return; }
    floppyPendingSet_ = false;
    std::vector<u8> img = std::move(floppyPending_);
    floppyPending_.clear();
    seatFloppy(std::move(img), floppyPendingRO_);
}

int QuadraMachine::insertFloppyNow(std::vector<u8> image, bool readOnly) {
    // Peel the wrappers the archived media wears -- MacBinary around DiskCopy
    // 4.2 around the sectors, either or both -- down to the raw disk data the
    // .Sony Prime hook serves. Most 7.1 install floppies are DiskCopy 4.2: an
    // 84-byte header in front of the sectors, so a 1.44 MB disk is 1,474,644
    // bytes on disk rather than 1,474,560, and read raw every sector lands 84
    // bytes out of place and nothing mounts.
    // Keep the containers rather than dropping them. The guest writes sectors,
    // but the file this disk came from wears a DiskCopy or MacBinary header
    // and has to go back to the host looking like the file it was -- writing
    // bare sectors over it would destroy the header its next reader needs.
    const char* container = "raw image";
    fdMbHeader_.clear(); fdMbResource_.clear();
    fdDcHeader_.clear(); fdDcTags_.clear();
    if (macbinary::isMacBinary(image)) {
        macbinary::Parts p = macbinary::split(image);
        fdMbHeader_ = std::move(p.header);
        fdMbResource_ = std::move(p.resource);
        container = "MacBinary";
    }
    if (dc42::isDiskCopy(image)) {
        dc42::Parts p = dc42::split(image);
        fdDcHeader_ = std::move(p.header);
        fdDcTags_ = std::move(p.tags);
        container = "DiskCopy 4.2";
    }

    // The three geometries a Macintosh SuperDrive frames. Size alone settles
    // the medium once the wrappers are off.
    const std::size_t n = image.size();
    const bool hd = n == 1474560;
    const bool dsGCR = n == 819200;   // 800K double-sided GCR
    const bool ssGCR = n == 409600;   // 400K single-sided GCR
    if (!hd && !dsGCR && !ssGCR) {
        if (onDiag) {
            char b[128];
            std::snprintf(b, sizeof b,
                          "floppy refused: %zu bytes (%s) is no floppy geometry -- "
                          "need 400K/800K/1.44 MB of sectors", n, container);
            onDiag(b);
        }
        return 0;
    }

    floppy_ = std::move(image);
    fd_->doubleSided = !ssGCR;
    fd_->insert(&floppy_, readOnly, hd);
    if (onDiag) {
        char b[128];
        std::snprintf(b, sizeof b, "floppy: %s disk in the internal drive (%s)",
                      hd ? "1.44 MB" : dsGCR ? "800K" : "400K", container);
        onDiag(b);
    }
    return 1;
}

// The front end's Eject menu item. It cannot pull the disk out here: this runs
// on the host's UI thread, and telling the guest about it means running guest
// code, which only the emulation thread may do. So stage it and let the next
// frame boundary carry it out.
//
// It also must not simply yank the medium. The System is holding the volume
// mounted -- its icon is on the desktop and its blocks are in the cache -- so
// a medium that vanishes underneath it leaves the disk on screen, the volume
// in the queue, and the guest asking for it back. That is exactly the reported
// symptom: the button appeared to do nothing and the OS kept the floppy.
void QuadraMachine::ejectFloppy() {
    if (floppy_.empty() && !fd_->hasDisk()) return;
    floppyEjectPending_ = true;
}

// Carry out a staged eject, at a frame boundary with the guest between
// instructions. Flush first so anything the System was still holding reaches
// the image the host keeps, then unmount so the volume leaves the desktop,
// then release the medium.
void QuadraMachine::completeFloppyEject() {
    if (!floppyEjectPending_) return;
    if (inDriver_ || announceInFlight_) return;
    if ((cpu_.getSR() & 0x0700) != 0) return;         // mid-interrupt
    if (addressingModeSwapped()) return;              // not now
    // Before the System is up there is no volume to unmount and no File
    // Manager to ask -- just take the disk.
    const bool fsUp =
        read32(0x0358) != 0 && (read8(0x0360) & 1) == 0 &&
        (((static_cast<u32>(read16(0x0360)) << 16) | read16(0x0362)) != 0xFFFFFFFFu);
    s16 fl = 0, ej = 0, un = 0;
    bool ejected = false;
    if (fsUp && volumeMountedFor(1)) {
        ejected = true;
        announceInFlight_ = true;
        u32 sd[8], sa[8];
        for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
        if (!hdMountPb_) {
            cpu_.d[0] = 80;
            execute68kTrap(0xA71E);                   // _NewPtr,Sys,Clear
            hdMountPb_ = guestPtr(cpu_.a[0]);
        }
        if (hdMountPb_) {
            auto call = [&](u16 trap) -> s16 {
                for (u32 i = 0; i < 80; ++i) write8(hdMountPb_ + i, 0);
                write16(hdMountPb_ + 22, 1);          // ioVRefNum = drive 1
                cpu_.a[0] = hdMountPb_;
                execute68kTrap(trap);
                return static_cast<s16>(cpu_.d[0] & 0xFFFF);
            };
            // Flush what the System is holding, then ASK THE FILE MANAGER TO
            // EJECT. Do not take the disk away behind its back.
            //
            // ⛔ Two ways of doing this are wrong, both measured on this
            // machine, both ending in the same corrupted pointer:
            //
            //  - _UnmountVol frees the VCB while the Finder is still holding a
            //    pointer to it for the disk's icon. It never asked for the
            //    unmount, so it never lets go: eleven frames later it walks the
            //    freed block and the machine bus-errors (BUSERR at 0002BBB6,
            //    A3 = A6A07A0E).
            //  - Simply removing the medium is no better. The volume stays ON
            //    LINE with nothing behind it, every access comes back offLinErr
            //    forever, and about five seconds later the Finder dies the same
            //    way. Taking the disk with no trap injected at all does it too,
            //    so the injection was never the problem -- the missing eject
            //    was.
            //
            // _Eject is what the Finder's own Special > Eject Disk issues. It
            // flushes the volume, marks it EJECTED -- still in the queue, so
            // every pointer to it stays good, and the icon dims instead of
            // dangling -- and commands the drive. The mechanism then runs on
            // its own clock and the medium comes out through the same path a
            // guest-commanded eject uses, which keeps it for the write-back.
            fl = call(0xA013);                        // _FlushVol
            ej = call(0xA017);                        // _Eject
            // ...and then ask to unmount the drive as well.
            //
            // On a volume _Eject has already taken off line this answers -35
            // nsvErr and changes nothing visible -- and yet WITHOUT it the next
            // disk inserted on this machine gets thrown straight back out
            // again, and with it the disks mount and stay. That is measured on
            // the user's own volume, three disks in a row, one variable at a
            // time; an extra _FlushVol in the same place -- same number of
            // injected traps, same cycles -- does NOT help, so it is the call
            // and not the timing.
            // What the File Manager settles in there is not yet understood.
            // Kept because the experiment is clean and the call is inert when
            // there is nothing to unmount; noted here so the next person knows
            // it is evidence, not reasoning.
            if (ejectUnmounts_) un = call(0xA00E);    // _UnmountVol
        }
        for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
        announceInFlight_ = false;
    }
    floppyEjectPending_ = false;
    if (ejected) {
        // The File Manager has it: the driver's eject is running, and
        // tickEject will hand the medium back at the frame it completes.
        if (onDiag) {
            char b[160];
            std::snprintf(b, sizeof b,
                          "floppy: front-end eject asked the File Manager "
                          "(flush %d, eject %d, unmount %d)", fl, ej, un);
            onDiag(b);
        }
        return;
    }
    // Nothing was mounted -- before the System is up, or a disk that never
    // mounted. There is no volume to lose and nobody holding a pointer to one,
    // so take the medium directly and tell the driver its drive is empty.
    if (!floppy_.empty()) floppyEjected_ = std::move(floppy_);
    floppy_.clear();
    fd_->removeDisk();
    clearDriveInPlace(1);
    if (onDiag) onDiag("floppy: front-end eject (no volume was mounted)");
}

// Mark a drive-queue entry as holding no disk. dsDiskInPlace lives in the
// DrvSts record three bytes ahead of the queue element; 0 means empty.
void QuadraMachine::clearDriveInPlace(u16 drive) {
    u32 e = guestPtr(read32(0x030A));                 // DrvQHdr.qHead
    for (int n = 0; e && n < 8; ++n) {
        if (read16(e + 6) == drive && e >= 3) {
            write8(e - 3, 0);
            return;
        }
        e = guestPtr(read32(e));
    }
}

bool QuadraMachine::floppyPresent() const { return fd_->hasDisk(); }

// The medium as the host's file should hold it: the guest's sectors with the
// containers the file arrived in put back around them, innermost first. An
// untouched disk reassembles to the identical file.
std::vector<u8> QuadraMachine::floppyFileImage(const std::vector<u8>& sectors) const {
    std::vector<u8> out = sectors;
    if (out.empty()) return out;
    if (fdDcHeader_.size() == dc42::kHeaderSize) {
        dc42::Parts p;
        p.header = fdDcHeader_;
        p.tags = fdDcTags_;
        out = dc42::rewrap(p, out);
    }
    if (fdMbHeader_.size() == macbinary::kHeaderSize) {
        macbinary::Parts p;
        p.header = fdMbHeader_;
        p.resource = fdMbResource_;
        out = macbinary::rewrap(p, out);
    }
    return out;
}

// What the guest wrote, ready to go back to the file it came from. Returns the
// disk still in the drive, or the one last ejected -- so a front end can save
// it either when the user asks or when the machine hands the disk back.
std::vector<u8> QuadraMachine::floppyForWriteBack() const {
    if (!floppy_.empty()) return floppyFileImage(floppy_);
    return floppyFileImage(floppyEjected_);
}

// 'PRAM', a version, the 256 bytes of XPRAM, then the clock. Written and read
// as bytes in a fixed order so a file survives a rebuild, and never
// interpreted -- see the header for why that matters.
// Decoded from the ROM's own dispatcher, not assumed: $408099B0 reads
// ($0E00,D2.w*4) for a Toolbox trap and JSRs ([$0400,D2.w*4]) for an OS one, so
// the tables are 1024 entries at $0E00 and 256 at $0400. A healthy System has no
// zero entry in either -- every unimplemented slot points at the ROM's
// unimplemented-trap handler.
std::string QuadraMachine::trapTableHealth() const {
    struct Table { const char* name; u32 base, count; u32 firstTrap; };
    static const Table kTables[2] = {{"OS", 0x0400, 256, 0xA000},
                                     {"Toolbox", 0x0E00, 1024, 0xA800}};
    std::string s;
    char b[160];
    for (const Table& t : kTables) {
        if (t.base + t.count * 4 > ram_.size()) continue;
        u32 zero = 0, firstAt = 0, lastAt = 0, runs = 0;
        bool inRun = false;
        for (u32 i = 0; i < t.count; ++i) {
            const u8* p = ram_.data() + t.base + i * 4;
            const bool isZero = !(p[0] || p[1] || p[2] || p[3]);
            if (isZero) {
                if (!zero++) firstAt = i;
                lastAt = i;
                if (!inRun) { ++runs; inRun = true; }
            } else {
                inRun = false;
            }
        }
        if (zero == 0) {
            std::snprintf(b, sizeof b, "   %s trap table: %u entries, all filled\n",
                          t.name, t.count);
            s += b;
            continue;
        }
        // The SHAPE is the evidence. One contiguous run of zeros is a block
        // write that landed on the table -- a length and a destination, and both
        // are readable from the range. Scattered zeros are something choosing
        // entries one at a time, which is a different culprit entirely.
        std::snprintf(b, sizeof b,
                      "   %s trap table: %u of %u entries DISPATCH TO ZERO\n",
                      t.name, zero, t.count);
        s += b;
        std::snprintf(b, sizeof b,
                      "      traps $%04X..$%04X, addresses $%04X..$%04X, "
                      "%u byte%s in %u run%s%s\n",
                      t.firstTrap + firstAt, t.firstTrap + lastAt,
                      t.base + firstAt * 4, t.base + lastAt * 4 + 3,
                      (lastAt - firstAt + 1) * 4, (lastAt - firstAt) ? "s" : "",
                      runs, runs == 1 ? "" : "s",
                      runs == 1 ? " (one contiguous block)" : "");
        s += b;
        // What sits either side of the hole: a block write leaves the entries
        // just outside it untouched, and those are the ROM addresses that say
        // where the table was healthy.
        const auto entry = [&](u32 i) -> u32 {
            const u8* p = ram_.data() + t.base + i * 4;
            return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | p[3];
        };
        std::snprintf(b, sizeof b, "      before $%04X, after $%04X\n",
                      firstAt ? entry(firstAt - 1) : 0,
                      lastAt + 1 < t.count ? entry(lastAt + 1) : 0);
        s += b;
    }
    return s;
}

u32 QuadraMachine::savePram(u8* out, u32 cap) const {
    if (!out || cap < kPramBlobBytes) return kPramBlobBytes;
    const std::array<u8, 256>& p = rtc_->xpram();
    std::memcpy(out, "PRAM", 4);
    out[4] = 1;                                       // version
    out[5] = out[6] = out[7] = 0;
    std::copy(p.begin(), p.end(), out + 8);
    const u32 s = rtc_->seconds();
    out[264] = u8(s >> 24); out[265] = u8(s >> 16);
    out[266] = u8(s >> 8);  out[267] = u8(s);
    return kPramBlobBytes;
}

bool QuadraMachine::loadPram(const u8* data, u32 len, u32 addSeconds) {
    if (!data || len < kPramBlobBytes) return false;
    if (std::memcmp(data, "PRAM", 4) != 0 || data[4] != 1) return false;
    std::array<u8, 256>& p = rtc_->xpram();
    std::copy(data + 8, data + 8 + 256, p.begin());
    const u32 s = (u32(data[264]) << 24) | (u32(data[265]) << 16) |
                  (u32(data[266]) << 8) | data[267];
    rtc_->setSeconds(s + addSeconds);
    return true;
}

void QuadraMachine::attachNet(bool attached, int busId) {
    netdev_->setAttached(attached, busId);
}
bool QuadraMachine::netAttached() const { return netdev_->attachedState(); }
bool QuadraMachine::netInject(const u8* frame, u32 len) {
    return netdev_->injectFrame(frame, len);
}
bool QuadraMachine::netDrain(std::vector<u8>& out) {
    return netdev_->drainFrame(out);
}

} // namespace openmac
