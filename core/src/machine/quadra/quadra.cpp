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
    : ram_(cfg.ramSize, 0),
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
      disk_(std::make_unique<ScsiDisk>()),
      disk2_(std::make_unique<ScsiDisk>()),
      cdrom_(std::make_unique<ScsiCdRom>()),
      netdev_(std::make_unique<ScsiEthernet>()),
      cpu_(*this) {
    u32 rs = 1;
    while (rs < rom_.size()) rs <<= 1;
    rom_.resize(rs, 0xFF);
    romMask_ = rs - 1;

    scc_->onDiag = [this](const char* s) { if (onDiag) onDiag(s); };
    // Default display: the 13-inch/14-inch RGB monitor the machine has always
    // reported. Its sense code is 6, which is one grounded line (bit 0).
    setMonitorSense(0x1, 0);
    scsi_->addTarget(disk_.get());
    scsi_->addTarget(disk2_.get());
    scsi_->addTarget(cdrom_.get());
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
        if (watchLen_ && addr - watchAddr_ < watchLen_ && watchBudget_ > 0 && onDiag) {
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
        if (watchLen_ && addr - watchAddr_ < watchLen_ + 1 && watchBudget_ > 0 && onDiag) {
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
        if (watchLen_ && addr - watchAddr_ < watchLen_ + 3 && watchBudget_ > 0 && onDiag) {
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
        announceHardDisks();
        // The CD's driver goes in on the same clock and for the same reason:
        // there is no unit table to install into until the System has built one.
        if (cdrom_->attachedState() && !cdDrvr_) installCdDriver();
        if (cdEjectPending_) completeCdEject();
        else if (cdMountPending_) mountCdVolume();
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
            // Claim the disk driver's Prime as soon as the ROM's boot scan has
            // installed it -- BEFORE the System's startup mount runs. Hooked
            // late, that mount goes through the SCSI Manager's discard drain,
            // fails, and the volume is written off for the rest of the session.
            if ((!hd_.empty() && !diskPrimePc_[0]) || (!hd2_.empty() && !diskPrimePc_[1]))
                findDiskDriverPrime();
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

// Is a volume already on line for this drive? Walk the VCB queue ($0356 is the
// header, qHead at $0358); each VCB carries its drive number at vcbDrvNum
// (+$6E). This is how we tell "the System mounted it itself" from "nobody has".
bool QuadraMachine::volumeMountedFor(u16 drive) {
    u32 vcb = guestPtr(read32(0x0358));
    for (int n = 0; vcb && vcb + 80 < ram_.size() && n < 16; ++n) {
        if (read16(vcb + 72) == drive) return true;   // vcbDrvNum
        vcb = guestPtr(read32(vcb));                  // qLink
    }
    return false;
}

// Locate the Prime entry of each installed .ScsiHD driver, so the machine can
// serve its requests directly. The unit table ($011C) holds a handle per unit;
// the DCE's dCtlDriver points at the DRVR, whose header carries the Prime
// offset. Cached once the System has installed the driver.
void QuadraMachine::findDiskDriverPrime() {
    static const int kUnits[2] = {1, 33};
    const u32 uTable = guestPtr(read32(0x011C));
    if (!uTable || uTable + 256 >= ram_.size()) return;
    for (int i = 0; i < 2; ++i) {
        if (diskPrimePc_[i]) continue;
        const u32 h = guestPtr(read32(uTable + static_cast<u32>(kUnits[i]) * 4u));
        if (!h || h + 4 >= ram_.size()) continue;
        const u32 dce = guestPtr(read32(h));
        if (!dce || dce + 4 >= ram_.size()) continue;
        const u32 drvr = guestPtr(read32(dce));       // dCtlDriver
        if (!drvr || drvr + 0x20 >= ram_.size()) continue;
        // Only OUR driver: its name field is ".ScsiHD".
        if (read8(drvr + 0x12) != 7 || read8(drvr + 0x13) != '.' ||
            read8(drvr + 0x14) != 'S' || read8(drvr + 0x15) != 'c')
            continue;
        const u32 prime = drvr + read16(drvr + 0x0A);
        diskPrimePc_[i] = prime;
        diskCtlPc_[i] = drvr + read16(drvr + 0x0C);      // drvrCtl
        diskStatusPc_[i] = drvr + read16(drvr + 0x0E);   // drvrStatus
        if (onDiag) {
            char b[72];
            std::snprintf(b, sizeof b, "hd: driver unit %d Prime at %08X",
                          kUnits[i], prime);
            onDiag(b);
        }
    }
}

// The on-disk SCSI driver's Prime, served from the image the way the .Sony
// hook serves floppies. The ROM's old-API SCSI Manager takes our driver's
// read down its DISCARD drain (measured: every byte of a correct MDB read out
// at ROM $D1CCC and thrown away), so the volume never mounts. Moving the
// bytes here keeps the guest's own driver, DCE and completion path intact --
// only the transfer itself is done by the machine -- and gives the writes the
// installer needs as well.
void QuadraMachine::serveDiskPrime(int unit) {
    const u32 pb = guestPtr(cpu_.a[0]);
    const u32 dce = guestPtr(cpu_.a[1]);
    const u16 trap = read16(pb + 0x06);
    const u32 buf = guestPtr(read32(pb + 0x20));
    const u32 req = read32(pb + 0x24);
    const u16 posMode = read16(pb + 0x2C);
    const u32 posOff = read32(pb + 0x2E);
    const u32 dctlPos = read32(dce + 0x10);

    std::vector<u8>& img = unit ? hd2_ : hd_;
    const bool ro = unit ? hd2RO_ : hdRO_;
    const u32 base = (unit ? hfsImageOffset2_ : hfsImageOffset_);
    std::vector<u8>& backing = unit ? scsiImage2_ : scsiImage_;

    u32 pos = dctlPos;
    switch (posMode & 3) {
    case 1: pos = posOff; break;                                   // fsFromStart
    case 2: pos = static_cast<u32>(img.size()) + posOff; break;    // fsFromLEOF
    case 3: pos = dctlPos + posOff; break;                         // fsFromMark
    default: break;                                                // fsAtMark
    }
    s16 result = 0;
    u32 done = 0;
    if (img.empty() || backing.empty()) {
        result = -65;                                  // offLinErr
    } else if ((trap & 1) && ro) {
        result = -44;                                  // wPrErr
    } else {
        const u32 size = static_cast<u32>(img.size());
        if (pos >= size) {
            result = -39;                              // eofErr
        } else {
            u32 n = req;
            if (pos + n > size) { n = size - pos; result = -39; }
            // The guest addresses the VOLUME; the backing store is the whole
            // partitioned disk, so the partition offset goes on here.
            const u32 at = base + pos;
            if (trap & 1) {
                for (u32 i = 0; i < n; ++i) backing[at + i] = read8(buf + i);
            } else {
                for (u32 i = 0; i < n; ++i) write8(buf + i, backing[at + i]);
            }
            done = n;
        }
    }
    if (trap & 1) ++hdWriteCount_; else ++hdReadCount_;
    // Every request the volume's own driver makes, in order. Two boots of the
    // same disk that diverge are best compared here: this is what the guest
    // asked the medium for, before any interpretation.
    if (onDiag && hdTraceBudget_ > 0) {
        --hdTraceBudget_;
        char b[128];
        std::snprintf(b, sizeof b, "HDIO %s pos=%u(blk %u) req=%u -> %u err=%d",
                      (trap & 1) ? "write" : "read ", pos, pos / 512, req, done,
                      result);
        onDiag(b);
    }
    if (result != 0 && onDiag && hdErrBudget_ > 0) {
        --hdErrBudget_;
        char b[128];
        std::snprintf(b, sizeof b,
                      "hd %s FAILED err=%d pos=%u req=%u buf=%08X (size=%u)",
                      (trap & 1) ? "write" : "read", result, pos, req, buf,
                      static_cast<u32>(img.size()));
        onDiag(b);
    }
    write32(pb + 0x28, done);                          // ioActCount
    write32(dce + 0x10, pos + done);                   // dCtlPosition
    write16(pb + 0x10, static_cast<u16>(result));      // ioResult
    cpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const u32 target = read32(0x08FC);                 // jIODone
    cpu_.a[0] = target;
    cpu_.pc = target;
}

// Where the disc's HFS volume begins. Apple's own CD masters put an Apple
// partition map at the front and the volume inside an "Apple_HFS" partition;
// a plain HFS master has its MDB two blocks in and starts at zero. The map's
// entries are addressed in 512-byte blocks even on a disc whose sectors are
// 2048, which is why this walks blocks rather than sectors.
//
// Reference: Inside Macintosh: Devices, "The SCSI Manager" (partition map), and
// the same walk in BasiliskII/src/cdrom.cpp.
u32 QuadraMachine::findHfsPartition(const std::vector<u8>& disc) {
    for (std::size_t blk = 0; blk < 64; ++blk) {
        const std::size_t at = blk * 512;
        if (at + 512 > disc.size()) break;
        const u8* m = disc.data() + at;
        if (m[0] != 0x50 || m[1] != 0x4D) continue;            // 'PM'
        // pmPartName at +16, pmParType at +48, both C strings in 32 bytes.
        const char* type = reinterpret_cast<const char*>(m + 48);
        if (std::strncmp(type, "Apple_HFS", 32) != 0) continue;
        const u32 start = (u32(m[8]) << 24) | (u32(m[9]) << 16) |
                          (u32(m[10]) << 8) | m[11];           // pmPyPartStart
        return start * 512u;
    }
    return 0;   // a bare HFS master, or a disc with no HFS on it at all
}

// Build the .AppleCD driver in the System heap and hand it to the Device
// Manager. Its Open/Prime/Control/Status entry points are four addresses this
// machine watches; when the guest's Device Manager jumps to one, stepInstruction
// serves the request in C++ and returns through jIODone -- the same shape as the
// hard disk's Prime hook, so the guest's DCE, queue and completion path stay its
// own and only the transfer is ours.
//
// This runs once the System is up, for the same reason the hard disk's mount
// does: a driver installed during the ROM's boot scan is installed into a
// machine that has not built its unit table yet.
void QuadraMachine::installCdDriver() {
    if (cdDrvr_ || inDriver_ || announceInFlight_) return;
    if (read32(0x0358) == 0) return;                  // System never came up
    if ((((static_cast<u32>(read16(0x0360)) << 16) | read16(0x0362)) == 0xFFFFFFFFu))
        return;                                       // FS queue not built
    if (read8(0x0360) & 1) return;                    // FSBusy: a file op is running
    if ((cpu_.getSR() & 0x0700) != 0) return;         // mid-interrupt: not now
    if (++cdInstallTries_ > 30) return;

    // Find the slot BEFORE allocating anything: the unit table is not built
    // until the System is well under way, and an attempt that gives up after
    // the allocation leaks a block of the System heap every time it retries.
    const u32 uTable = guestPtr(read32(0x011C));
    const u32 units = read16(0x01D2);                 // UnitNtryCnt
    u32 unit = 0;
    for (u32 i = 32; i < units && i < 128; ++i) {     // 0-31 are Apple's
        if (uTable + i * 4u + 3 >= ram_.size()) break;
        if (read32(uTable + i * 4u) == 0) { unit = i; break; }
    }
    if (!uTable || !unit) return;                     // not yet; try again later

    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }

    // DRVR header: flags, delay, event mask, menu, then the five entry-point
    // offsets, then the name as a Pascal string. Our entry points are stubs the
    // guest never actually executes, but they have to BE somewhere.
    const u32 kHeader = 0x1C;                         // header + ".AppleCD" + pad
    cpu_.d[0] = kHeader + 0x20;
    execute68kTrap(0xA71E);                           // _NewPtr,Sys,Clear
    const u32 drvr = guestPtr(cpu_.a[0]);
    if (!drvr) {
        for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
        announceInFlight_ = false;
        return;
    }
    // dReadEnable | dWritEnable | dCtlEnable | dStatEnable | dNeedLock.
    write16(drvr + 0x00, 0x4F00);                     // drvrFlags
    write16(drvr + 0x02, 0);                          // drvrDelay
    write16(drvr + 0x04, 0);                          // drvrEMask
    write16(drvr + 0x06, 0);                          // drvrMenu
    write16(drvr + 0x08, u16(kHeader + 0x00));        // drvrOpen
    write16(drvr + 0x0A, u16(kHeader + 0x08));        // drvrPrime
    write16(drvr + 0x0C, u16(kHeader + 0x10));        // drvrCtl
    write16(drvr + 0x0E, u16(kHeader + 0x18));        // drvrStatus
    write16(drvr + 0x10, u16(kHeader + 0x00));        // drvrClose -> the Open stub
    const char* name = "\010.AppleCD";
    for (int i = 0; i < 9; ++i) write8(drvr + 0x12 + u32(i), u8(name[i]));
    // Each stub is MOVEQ #0,D0 / JMP ([jIODone]) -- a correct driver on its own,
    // so nothing is stranded if the hook ever misses.
    for (u32 off : {0x00u, 0x08u, 0x10u, 0x18u}) {
        const u32 at = drvr + kHeader + off;
        write16(at + 0, 0x7000);                      // MOVEQ #0,D0
        write16(at + 2, 0x2078); write16(at + 4, 0x08FC);   // MOVEA.L $08FC.W,A0
        write16(at + 6, 0x4ED0);                      // JMP (A0)
    }
    cdDrvr_ = drvr;
    cdPrimePc_ = drvr + kHeader + 0x08;
    cdCtlPc_ = drvr + kHeader + 0x10;
    cdStatusPc_ = drvr + kHeader + 0x18;

    // Into the unit table by hand rather than through _DrvrInstall. That trap
    // takes a refNum the caller has to have picked already, and picking one by
    // trying them in turn means writing over whatever is there when the guess
    // is wrong -- which is exactly what it did: low memory came back as
    // $6DB6DB6D and the machine was gone. Finding an empty slot first is the
    // same work without the guessing.
    const s16 refNum = static_cast<s16>(~unit);
    cdRefNum_ = refNum;
    // The unit table holds HANDLES to device control entries, so the DCE is a
    // relocatable block -- and it must not move under the Device Manager, hence
    // the lock.
    cpu_.d[0] = 50;                                   // AuxDCE
    execute68kTrap(0xA722);                           // _NewHandle,Sys,Clear
    const u32 dceH = cpu_.a[0];
    if (dceH) { cpu_.a[0] = dceH; execute68kTrap(0xA029); }   // _HLock
    const u32 dce = dceH ? guestPtr(read32(guestPtr(dceH))) : 0;
    if (!dce) {
        cdDrvr_ = cdPrimePc_ = cdCtlPc_ = cdStatusPc_ = 0;
        for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
        announceInFlight_ = false;
        if (onDiag) onDiag("cd: no memory for the driver's control entry");
        return;
    }
    write32(dce + 0x00, drvr);                        // dCtlDriver (a pointer)
    write16(dce + 0x04, u16(0x4F00 | 0x0020));        // dCtlFlags | dOpened
    write16(dce + 0x18, static_cast<u16>(refNum));    // dCtlRefNum
    write32(uTable + unit * 4u, dceH);

    // The drive itself: a DrvSts the drive queue links through, then AddDrive.
    cpu_.d[0] = 22;                                   // SIZEOF DrvSts
    execute68kTrap(0xA71E);                           // _NewPtr,Sys,Clear
    cdStatus_ = guestPtr(cpu_.a[0]);
    if (cdStatus_) {
        write8(cdStatus_ + 2, 0x80);                  // dsWriteProt: locked
        write8(cdStatus_ + 3, cdrom_->discPresent() ? 1 : 0);   // dsDiskInPlace
        write8(cdStatus_ + 4, 1);                     // dsInstalled
        write8(cdStatus_ + 5, 1);                     // dsSides
        cdDrive_ = 6;                                 // past the two SCSI seats
        cpu_.d[0] = (u32(cdDrive_) << 16) | u16(refNum);
        cpu_.a[0] = cdStatus_ + 6;                    // the DrvQEl itself
        execute68kTrap(0xA04E);                       // _AddDrive
        cdMountPending_ = cdrom_->discPresent();
        cdMountTries_ = 0;
    }
    for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
    if (onDiag) {
        char b[144];
        std::snprintf(b, sizeof b,
                      "cd: .AppleCD installed at unit %u (refNum %d), drive %u, "
                      "Prime at %08X", unit, refNum, cdDrive_, cdPrimePc_);
        onDiag(b);
    }
}

// Ask the File Manager to mount the disc, the same way the hard disks are
// mounted: _MountVol on the drive, gated on the System being settled. A
// disk-inserted event would be the other way, and the hard disk's history says
// that event makes 7.5's startup re-run a mount it has already failed.
void QuadraMachine::mountCdVolume() {
    if (!cdMountPending_ || !cdDrive_) return;
    if (inDriver_ || announceInFlight_) return;
    if (read32(0x0358) == 0) return;                  // System never came up
    if (read8(0x0360) & 1) return;                    // FSBusy
    if ((cpu_.getSR() & 0x0700) != 0) return;         // mid-interrupt
    if (volumeMountedFor(cdDrive_)) { cdMountPending_ = false; return; }
    if (++cdMountTries_ > 15) { cdMountPending_ = false; return; }
    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
    if (!hdMountPb_) {
        cpu_.d[0] = 80;
        execute68kTrap(0xA71E);                       // _NewPtr,Sys,Clear
        hdMountPb_ = guestPtr(cpu_.a[0]);
    }
    s16 res = -108;
    if (hdMountPb_) {
        for (u32 i = 0; i < 80; ++i) write8(hdMountPb_ + i, 0);
        write16(hdMountPb_ + 22, cdDrive_);           // ioVRefNum = the drive
        cpu_.a[0] = hdMountPb_;
        execute68kTrap(0xA00F);                       // _MountVol
        res = static_cast<s16>(cpu_.d[0] & 0xFFFF);
    }
    for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
    if (res == 0 || res == -55) cdMountPending_ = false;   // mounted, or already
    if (onDiag) {
        char b[80];
        std::snprintf(b, sizeof b, "cd: mount drive %u -> %d", cdDrive_, res);
        onDiag(b);
    }
}

// The CD driver's Prime: read-only, served straight out of the disc image at
// the HFS partition's offset. A CD is addressed by the File Manager in 512-byte
// logical blocks whatever the drive's own sector size is, and the image is a
// flat byte run, so the translation is one addition.
void QuadraMachine::serveCdPrime() {
    const u32 pb = guestPtr(cpu_.a[0]);
    const u32 dce = guestPtr(cpu_.a[1]);
    const u16 trap = read16(pb + 0x06);
    const u32 buf = guestPtr(read32(pb + 0x20));
    const u32 req = read32(pb + 0x24);
    const u16 posMode = read16(pb + 0x2C);
    const u32 posOff = read32(pb + 0x2E);
    const u32 dctlPos = read32(dce + 0x10);
    const std::vector<u8>& disc = cdrom_->media();
    const u32 volSize = disc.size() > cdHfsOffset_
                            ? static_cast<u32>(disc.size()) - cdHfsOffset_ : 0;

    u32 pos = dctlPos;
    switch (posMode & 3) {
    case 1: pos = posOff; break;                                   // fsFromStart
    case 2: pos = volSize + posOff; break;                         // fsFromLEOF
    case 3: pos = dctlPos + posOff; break;                         // fsFromMark
    default: break;                                                // fsAtMark
    }
    s16 result = 0;
    u32 done = 0;
    if (!cdrom_->discPresent() || volSize == 0) {
        result = -65;                                  // offLinErr
    } else if (trap & 1) {
        result = -44;                                  // wPrErr: it is a CD
    } else if (pos >= volSize) {
        result = -39;                                  // eofErr
    } else {
        u32 n = req;
        if (pos + n > volSize) { n = volSize - pos; result = -39; }
        const u32 at = cdHfsOffset_ + pos;
        for (u32 i = 0; i < n; ++i) write8(buf + i, disc[at + i]);
        done = n;
    }
    write32(pb + 0x28, done);                          // ioActCount
    write32(dce + 0x10, pos + done);                   // dCtlPosition
    write16(pb + 0x10, static_cast<u16>(result));      // ioResult
    cpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const u32 target = read32(0x08FC);                 // jIODone
    cpu_.a[0] = target;
    cpu_.pc = target;
}

// Control and Status for the CD. The one that matters is csCode 7, Eject: the
// Finder's drag-to-Trash and Special > Eject Disk both come through here, and a
// drive that answers "done" without letting go leaves an icon on a disc the
// host has already taken back.
void QuadraMachine::serveCdCtlStatus(bool status) {
    const u32 pb = guestPtr(cpu_.a[0]);
    const u16 csCode = read16(pb + 0x1A);
    s16 result = 0;
    if (!status) {
        switch (csCode) {
        case 7:                                        // Eject
            cdrom_->eject();
            cdHfsOffset_ = 0;
            if (cdStatus_) write8(cdStatus_ + 3, 0);   // dsDiskInPlace
            if (onDiag) onDiag("cd: the guest ejected the disc");
            break;
        case 21: case 22:                              // icon / drive info
            result = -17;                              // controlErr: not ours
            break;
        case 5: case 6: case 8: case 9: case 23: case 24:
            break;                                     // verify, format, tag: fine
        default:
            break;
        }
    } else if (csCode == 8) {                          // DriveStatus
        if (cdStatus_)
            for (u32 i = 0; i < 22; ++i) write8(pb + 0x1C + i, read8(cdStatus_ + i));
    }
    write16(pb + 0x10, static_cast<u16>(result));      // ioResult
    cpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const u32 target = read32(0x08FC);                 // jIODone
    cpu_.a[0] = target;
    cpu_.pc = target;
}

// The driver's Control and Status entries, answered the way the Classic
// answers them for its own hard disk. Our on-disk driver stubs both to "no
// error" without touching the parameter block, so anything that asks the
// drive about itself -- a format list, a drive size -- reads back whatever
// was in the block already.
void QuadraMachine::serveDiskCtlStatus(int unit, bool status) {
    const u32 pb = guestPtr(cpu_.a[0]);
    const u16 csCode = read16(pb + 0x1A);
    std::vector<u8>& img = unit ? hd2_ : hd_;
    s16 result = 0;
    if (status) {
        // csCode 6, the format list: one format covering the whole medium.
        if (csCode == 6) {
            write16(pb + 0x1C, 1);
            write32(pb + 0x1E, static_cast<u32>(img.size() / 512));
        }
    } else {
        // KillIO is refused; a fixed disk cannot be ejected and the rest
        // (verify, format, tag buffer, track cache) are done already. The
        // icon calls (21 = drive icon, 22 = media icon) must be refused too:
        // they answer with a POINTER in csParam, and "no error" over whatever
        // the block happened to hold sends the caller off to draw from a wild
        // address. controlErr makes the caller use its default icon instead.
        if (csCode == 1 || csCode == 21 || csCode == 22)
            result = -17;                           // controlErr
    }
    write16(pb + 0x10, static_cast<u16>(result));   // ioResult
    cpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const u32 target = read32(0x08FC);              // jIODone
    cpu_.a[0] = target;
    cpu_.pc = target;
}

// Run one A-line trap to completion on the guest CPU: the opcode goes in a
// scratch cell at the top of RAM, the PC points at it, and stepping continues
// until control returns to the following word (the dispatcher adjusts the
// return PC past the A-line). The Classic's proven injection shape.
void QuadraMachine::execute68kTrap(u16 trap) {
    // ApplScratch, the twelve low-memory bytes reserved for applications. The
    // Classic put this word at the top of RAM, which works on a machine with
    // no MMU -- but here the '040 runs the System's own address space, and
    // fetching from the far top of physical RAM lands somewhere the guest has
    // not mapped: the CPU faults, the guard loop spins on the fault, and the
    // machine ends up executing address 10. Low memory is identity-mapped on
    // every Macintosh, because the vector table and the globals live there.
    //
    // ⛔ And PUT BACK what was there. Those twelve bytes are reserved for the
    // running APPLICATION -- the System does not touch them, so an application
    // is entitled to leave something in them across a Toolbox call. Writing the
    // trap word here and walking away corrupts the first two bytes of whatever
    // the Finder was keeping, permanently, on every injected trap: the disk
    // announce does it up to fifteen times a second while the desktop is up,
    // and the eject does it again. The Finder reads its own scratch back, gets
    // half a trap word where a pointer used to be, follows it, and the machine
    // bus-errors seconds later somewhere that has nothing to do with disks
    // (measured: A3 = A6A07A0E at 0002BBB6, every front-end eject).
    const u32 scratch = 0x0A78;
    const u8 was0 = read8(scratch), was1 = read8(scratch + 1);
    write8(scratch, static_cast<u8>(trap >> 8));
    write8(scratch + 1, static_cast<u8>(trap & 0xFF));
    const u32 savedPc = cpu_.pc;
    const u16 savedSr = cpu_.getSR();
    cpu_.pc = scratch;
    for (int guard = 0; guard < 64000000 && cpu_.pc != scratch + 2 && !cpu_.halted;
         ++guard) {
        stepInstruction();
    }
    cpu_.pc = savedPc;
    cpu_.setSR(savedSr);
    write8(scratch, was0);
    write8(scratch + 1, was1);
}

s32 QuadraMachine::gestaltQuery(u32 selector, u32& response) {
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
    cpu_.d[0] = selector;
    execute68kTrap(0xA1AD);                       // _Gestalt
    const s32 err = static_cast<s16>(cpu_.d[0] & 0xFFFF);
    response = cpu_.a[0];
    for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
    return err;
}

// A drive that has sat in the queue since the ROM scan never gets mounted:
// its disk-inserted announcement fell in the window startup flushes. Posting
// the event once the System is up makes the insertion the same event a user's
// insertion is -- the System mounts the volume and the Finder shows it.
void QuadraMachine::announceHardDisks() {
    if (suppressHdAnnounce) return;
    if (announceInFlight_) return;
    if (!hdAnnouncePending_ && !hd2AnnouncePending_) return;
    if (read32(0x0358) == 0) return;              // no live VCB: System not up
    if (read8(0x0360) & 1) return;                // FSBusy: a file op is running
    if ((cpu_.getSR() & 0x0700) != 0) return;     // mid-interrupt: not now
    // Never from inside a driver request we are serving: the guest's own
    // driver call is still in flight and a nested File Manager call there is
    // the re-entrancy the Classic learned to refuse.
    if (inDriver_) return;
    // One second's grace for the System's own startup mount, then do it
    // ourselves. Waiting longer means an application is already running and
    // has enumerated its disks -- the 7.5 Installer builds its destination
    // list at launch, so a volume mounted after that never appears in it.
    if (++hdAnnounceDelay_ < 2) return;
    // If the System already mounted the volume by itself -- which it does once
    // the driver's Prime is hooked before startup -- there is nothing to
    // announce. Injecting anyway disturbs whatever is running: the 7.5
    // Installer loses its window and hangs on a watch cursor (measured).
    if (volumeMountedFor(4)) hdAnnouncePending_ = false;
    if (volumeMountedFor(5)) hd2AnnouncePending_ = false;
    if (!hdAnnouncePending_ && !hd2AnnouncePending_) return;
    // The File Manager's own queue must be initialised, or _MountVol enqueues
    // into a header the System has not built yet (the Classic address-errored
    // on exactly this).
    if ((((static_cast<u32>(read16(0x0360)) << 16) | read16(0x0362)) == 0xFFFFFFFFu))
        return;
    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
    // A param block from the System heap, not a corner of RAM we picked: the
    // trap keeps it only for the call, but the heap is the one place nothing
    // else claims. Allocated once, reused for every retry.
    if (!hdMountPb_) {
        cpu_.d[0] = 80;
        execute68kTrap(0xA71E);                   // _NewPtr,Sys,Clear
        // Strip only what is not already a RAM address: above 16 MB the
        // high byte is significant, below it it may carry flag bits.
        hdMountPb_ = guestPtr(cpu_.a[0]);
    }
    // Mount the volume -- and post NO event. The applications that must see
    // the disk (the Finder, the 7.5 Installer) enumerate the VCB queue, so
    // the mount alone puts the volume in front of them. Posting a diskEvt as
    // well poisons the 7.5 startup: with the event in the queue, the System's
    // event processing re-runs a drive-1 mount mid-startup, the re-mount
    // comes back volOnLinErr, and the recovery FLUSHES AND EJECTS the boot
    // floppy and asks for it back by name (measured -- the please-insert
    // screen at frame ~1700 appears with the event and not without it).
    auto mountDrive = [&](u16 drive) -> bool {
        if (!hdMountPb_) return false;
        for (u32 i = 0; i < 80; ++i) write8(hdMountPb_ + i, 0);
        write16(hdMountPb_ + 22, drive);          // ioVRefNum = drive number
        cpu_.a[0] = hdMountPb_;
        execute68kTrap(0xA00F);                   // _MountVol
        const s16 res = static_cast<s16>(cpu_.d[0] & 0xFFFF);
        if (onDiag) {
            char b[72];
            std::snprintf(b, sizeof b, "hd: drive %u mount result %d", drive, res);
            onDiag(b);
        }
        // 0 = mounted, -55 = volOnLinErr = already on line. Either way, done.
        return res == 0 || res == -55;
    };
    ++hdMountTries_;
    if (hdAnnouncePending_ && mountDrive(4)) hdAnnouncePending_ = false;
    if (hd2AnnouncePending_ && mountDrive(5)) hd2AnnouncePending_ = false;
    if (hdMountTries_ >= 15) { hdAnnouncePending_ = hd2AnnouncePending_ = false; }
    for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
}

// Closing the host application is, to the guest, having its power cut. The
// System marks a volume "in use" on its disk the moment it mounts it for
// writing and only clears that mark on unmount, so a machine that is simply
// switched off leaves every volume flagged unclean, and blocks the System is
// still holding in its cache never reach the image at all.
//
// The flag itself is not a problem -- this ROM rebuilds an unclean volume and
// mounts it, the way a real Macintosh does (see hfs::shrinkExtentsTree for the
// one thing that used to stop it). The lost cache blocks are.
//
// So do what Special > Shut Down does, on the way out: flush each mounted
// volume and unmount it. The guest's own File Manager writes its cached
// blocks through our driver -- which we serve synchronously, so everything
// has landed in the image by the time this returns -- and sets the clean
// mark itself. Nothing is faked: the flag is set by the code that owns it.
// Record in the image that a volume is consistent on disk. drAtrb bit 8 says
// "this volume was unmounted cleanly", which is the File Manager's way of
// promising that nothing it was holding is missing from the medium. After a
// _FlushVol that returned noErr, and with the machine about to be destroyed,
// that promise is simply true -- the flush is what makes it true, and this
// only writes it down. The startup volume cannot be unmounted (its System
// file is open), so there is no other way for the guest to state it.
//
// Both copies of the MDB are updated, as a real unmount does: block 2 and the
// alternate in the second-to-last block of the volume.
// Returns true only when a flag actually had to be changed, so a caller can
// tell "this volume was left mounted" from "it was already clean".
bool QuadraMachine::markVolumeCleanIn(std::vector<u8>& backing, u32 base,
                                      u32 volumeSize) {
    if (backing.empty() || volumeSize < 2048) return false;
    bool changed = false;
    // Primary MDB at volume block 2, alternate 1024 bytes from the end.
    for (u32 off : {1024u, volumeSize - 1024u}) {
        const u32 at = base + off;
        if (at + 12 > backing.size()) continue;
        if (backing[at] != 0x42 || backing[at + 1] != 0x44) continue;   // 'BD'
        if (backing[at + 0x0A] & 0x01) continue;                        // clean
        backing[at + 0x0A] = static_cast<u8>(backing[at + 0x0A] | 0x01);
        changed = true;
    }
    return changed;
}

bool QuadraMachine::markVolumeClean(u16 drive) {
    const int unit = drive == 4 ? 0 : 1;
    if (unit ? hd2RO_ : hdRO_) return false;
    const std::vector<u8>& img = unit ? hd2_ : hd_;
    if (img.empty()) return false;
    return markVolumeCleanIn(unit ? scsiImage2_ : scsiImage_,
                             unit ? hfsImageOffset2_ : hfsImageOffset_,
                             static_cast<u32>(img.size()));
}

bool QuadraMachine::shutdownVolumes() {
    if (hd_.empty() && hd2_.empty()) return false;
    if (inDriver_ || announceInFlight_) return false;
    if (read32(0x0358) == 0) return false;            // System never came up
    if ((((static_cast<u32>(read16(0x0360)) << 16) | read16(0x0362)) == 0xFFFFFFFFu))
        return false;                                 // FS queue not built
    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
    if (!hdMountPb_) {
        cpu_.d[0] = 80;
        execute68kTrap(0xA71E);                       // _NewPtr,Sys,Clear
        hdMountPb_ = guestPtr(cpu_.a[0]);
    }
    bool any = false;
    if (hdMountPb_) {
        for (u16 drive : {u16(4), u16(5)}) {
            if (!volumeMountedFor(drive)) continue;
            auto call = [&](u16 trap) -> s16 {
                for (u32 i = 0; i < 80; ++i) write8(hdMountPb_ + i, 0);
                write16(hdMountPb_ + 22, drive);      // ioVRefNum
                cpu_.a[0] = hdMountPb_;
                execute68kTrap(trap);
                return static_cast<s16>(cpu_.d[0] & 0xFFFF);
            };
            // Flush first: the File Manager writes every cached block of this
            // volume through our driver, which we serve synchronously, so
            // when this returns the image on the host holds everything the
            // guest ever wrote.
            const s16 fl = call(0xA013);              // _FlushVol
            // Then try the real unmount. It succeeds for a data volume, and
            // fails -47 (fBsyErr) for the startup volume because the System
            // file is open on it -- a real Shut Down cannot unmount that one
            // either.
            const s16 un = call(0xA00E);              // _UnmountVol
            bool marked = false;
            if (fl == 0 && un != 0) marked = markVolumeClean(drive);
            any = any || un == 0 || marked;
            if (onDiag) {
                char b[112];
                std::snprintf(b, sizeof b,
                              "hd: drive %u flush %d, unmount %d%s", drive, fl,
                              un, marked ? ", marked cleanly unmounted" : "");
                onDiag(b);
            }
        }
    }
    for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
    return any;
}

// The first half of a drop-box republish: get the guest off the second volume
// so the host may rebuild it. Only drive 5 -- the boot volume is not ours to
// disturb, and unlike shutdownVolumes this is not a shutdown: the machine goes
// on running with one fewer disk for the moment it takes to build the next one.
//
// The flush is the part that matters. The File Manager writes every cached
// block of the volume out through our driver, which we serve synchronously, so
// by the time this returns anything the guest wrote is in the image the caller
// is about to read. Unmounting then makes the guest forget its cached catalog
// and bitmap -- which is exactly why a rebuilt volume may be put in its place.
// Editing the image under a mounted volume instead would leave the guest
// reading its old catalog against the new disk's blocks.
//
// Returns true when the volume is off line afterwards, which includes it never
// having been mounted -- the caller's next step is the same either way.
bool QuadraMachine::unmountSecondDisk() {
    if (hd2_.empty()) return true;
    if (inDriver_ || announceInFlight_) return false;
    if (read32(0x0358) == 0) return true;             // System never came up
    if ((((static_cast<u32>(read16(0x0360)) << 16) | read16(0x0362)) == 0xFFFFFFFFu))
        return true;                                  // FS queue not built
    if (read8(0x0360) & 1) return false;              // FSBusy: a file op is running
    if ((cpu_.getSR() & 0x0700) != 0) return false;   // mid-interrupt: not now
    if (!volumeMountedFor(5)) return true;            // already off line
    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
    if (!hdMountPb_) {
        cpu_.d[0] = 80;
        execute68kTrap(0xA71E);                       // _NewPtr,Sys,Clear
        hdMountPb_ = guestPtr(cpu_.a[0]);
    }
    s16 fl = 0, un = 0;
    if (hdMountPb_) {
        auto call = [&](u16 trap) -> s16 {
            for (u32 i = 0; i < 80; ++i) write8(hdMountPb_ + i, 0);
            write16(hdMountPb_ + 22, 5);              // ioVRefNum = drive number
            cpu_.a[0] = hdMountPb_;
            execute68kTrap(trap);
            return static_cast<s16>(cpu_.d[0] & 0xFFFF);
        };
        fl = call(0xA013);                            // _FlushVol
        un = call(0xA00E);                            // _UnmountVol
    }
    for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
    const bool off = !volumeMountedFor(5);
    if (onDiag) {
        char b[112];
        std::snprintf(b, sizeof b, "hd2: flush %d, unmount %d -- volume %s", fl, un,
                      off ? "off line" : "STILL MOUNTED");
        onDiag(b);
    }
    return off;
}

// Read a file the way an application does: the guest's File Manager walks the
// catalog, resolves the extents and calls the driver, and we serve the driver.
// Everything an archiver depends on is therefore in the loop -- which is the
// point. Our own reader can say the bytes are on the volume; only this can say
// the guest is able to fetch them.
bool QuadraMachine::readFileThroughGuest(const std::string& name,
                                         std::vector<u8>& out, std::string& why) {
    out.clear();
    why.clear();
    if (hd2_.empty()) { why = "no volume on the second seat"; return false; }
    if (inDriver_ || announceInFlight_) { why = "busy"; return false; }
    if (read32(0x0358) == 0) { why = "the System is not up"; return false; }
    if ((((static_cast<u32>(read16(0x0360)) << 16) | read16(0x0362)) == 0xFFFFFFFFu)) {
        why = "the File Manager queue is not built";
        return false;
    }
    if (read8(0x0360) & 1) { why = "the File Manager is busy"; return false; }
    if (name.empty() || name.size() > 31) { why = "bad name"; return false; }

    constexpr u32 kChunk = 32768;
    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }

    auto newPtr = [&](u32 bytes) -> u32 {
        cpu_.d[0] = bytes;
        execute68kTrap(0xA71E);                       // _NewPtr,Sys,Clear
        return guestPtr(cpu_.a[0]);
    };
    const u32 pb = newPtr(80);
    const u32 nameBuf = newPtr(64);
    const u32 dataBuf = newPtr(kChunk);
    bool ok = false;
    if (!pb || !nameBuf || !dataBuf) {
        why = "the guest heap would not give us working space";
    } else {
        // A Pascal string: length byte, then the characters.
        write8(nameBuf, static_cast<u8>(name.size()));
        for (std::size_t i = 0; i < name.size(); ++i)
            write8(nameBuf + 1 + static_cast<u32>(i), static_cast<u8>(name[i]));

        auto call = [&](u16 trap) -> s16 {
            cpu_.a[0] = pb;
            execute68kTrap(trap);
            return static_cast<s16>(cpu_.d[0] & 0xFFFF);
        };
        auto clearPb = [&]() { for (u32 i = 0; i < 80; ++i) write8(pb + i, 0); };

        clearPb();
        write32(pb + 18, nameBuf);                    // ioNamePtr
        write16(pb + 22, 5);                          // ioVRefNum = drive number
        write8(pb + 27, 1);                           // ioPermssn = fsRdPerm
        const s16 openErr = call(0xA000);             // _Open
        if (openErr != 0) {
            char b[96];
            std::snprintf(b, sizeof b, "the guest could not open it (_Open %d)", openErr);
            why = b;
        } else {
            const u16 refNum = read16(pb + 24);
            s16 readErr = 0;
            for (;;) {
                clearPb();
                write16(pb + 24, refNum);             // ioRefNum
                write32(pb + 32, dataBuf);            // ioBuffer
                write32(pb + 36, kChunk);             // ioReqCount
                write16(pb + 44, 0);                  // ioPosMode = fsAtMark
                readErr = call(0xA002);               // _Read
                const u32 got = read32(pb + 40);      // ioActCount
                for (u32 i = 0; i < got; ++i) out.push_back(read8(dataBuf + i));
                // -39 eofErr with a short count is the normal end of a file.
                if (readErr != 0 || got == 0) break;
            }
            clearPb();
            write16(pb + 24, refNum);
            call(0xA001);                             // _Close
            if (readErr != 0 && readErr != -39) {
                char b[96];
                std::snprintf(b, sizeof b, "the guest's read failed (_Read %d) after %zu bytes",
                              readErr, out.size());
                why = b;
            } else {
                ok = true;
            }
        }
    }
    for (u32 p : {pb, nameBuf, dataBuf}) {
        if (!p) continue;
        cpu_.a[0] = p;
        execute68kTrap(0xA01F);                       // _DisposPtr
    }
    for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
    return ok;
}

int QuadraMachine::stepInstruction() {
    if (countPc_ && (cpu_.pc | 0x40000000u) == (countPc_ | 0x40000000u)) ++countPcHits_;
    // The ROM's .Sony driver runs through both its 32-bit face and the 24-bit
    // alias; hook Prime at either and serve the request from the image.
    if ((diskPrimePc_[0] && cpu_.pc == diskPrimePc_[0]) ||
        (diskPrimePc_[1] && cpu_.pc == diskPrimePc_[1])) {
        const int unit = (diskPrimePc_[0] && cpu_.pc == diskPrimePc_[0]) ? 0 : 1;
        inDriver_ = true;
        try { serveDiskPrime(unit); } catch (const BusFault& f) {
            if (onDiag) {
                char b[112];
                std::snprintf(b, sizeof b,
                              "hd prime FAULT pb=%08X dce=%08X addr=%08X (f %u)",
                              cpu_.a[0], cpu_.a[1], f.addr,
                              static_cast<unsigned>(frameCounter_));
                onDiag(b);
            }
            cpu_.d[0] = static_cast<u32>(-36);
            cpu_.pc = read32(0x08FC);
            cpu_.a[0] = cpu_.pc;
        }
        inDriver_ = false;
        tickDevices(40);
        return 40;
    }
    if (cdPrimePc_ && cpu_.pc == cdPrimePc_) {
        inDriver_ = true;
        try { serveCdPrime(); } catch (const BusFault& f) {
            if (onDiag) {
                char b[112];
                std::snprintf(b, sizeof b, "cd prime FAULT pb=%08X addr=%08X (f %u)",
                              cpu_.a[0], f.addr, static_cast<unsigned>(frameCounter_));
                onDiag(b);
            }
            cpu_.d[0] = static_cast<u32>(-36);
            cpu_.pc = read32(0x08FC);
            cpu_.a[0] = cpu_.pc;
        }
        inDriver_ = false;
        tickDevices(40);
        return 40;
    }
    if ((cdCtlPc_ && cpu_.pc == cdCtlPc_) || (cdStatusPc_ && cpu_.pc == cdStatusPc_)) {
        const bool isStatus = cdStatusPc_ && cpu_.pc == cdStatusPc_;
        inDriver_ = true;
        try { serveCdCtlStatus(isStatus); } catch (const BusFault&) {
            cpu_.d[0] = static_cast<u32>(-36);
            cpu_.pc = read32(0x08FC);
            cpu_.a[0] = cpu_.pc;
        }
        inDriver_ = false;
        tickDevices(40);
        return 40;
    }
    for (int u = 0; u < 2; ++u) {
        if ((diskCtlPc_[u] && cpu_.pc == diskCtlPc_[u]) ||
            (diskStatusPc_[u] && cpu_.pc == diskStatusPc_[u])) {
            const bool isStatus = diskStatusPc_[u] && cpu_.pc == diskStatusPc_[u];
            inDriver_ = true;
            try { serveDiskCtlStatus(u, isStatus); } catch (const BusFault& f) {
                if (onDiag) {
                    char b[112];
                    std::snprintf(b, sizeof b,
                                  "hd ctl/status FAULT pb=%08X addr=%08X (f %u)",
                                  cpu_.a[0], f.addr,
                                  static_cast<unsigned>(frameCounter_));
                    onDiag(b);
                }
                cpu_.d[0] = static_cast<u32>(-36);
                cpu_.pc = read32(0x08FC);
                cpu_.a[0] = cpu_.pc;
            }
            inDriver_ = false;
            tickDevices(40);
            return 40;
        }
    }
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
// maximum pixel depths lists them, each with the sense wiring it presents --
// measured from the ROM by setting the wiring and reading back the screen the
// System configured (CrsrPin), not assumed.
const QuadraMachine::DisplayInfo* QuadraMachine::displays(int& count) {
    static const DisplayInfo kList[] = {
        {"12-inch RGB",       512,  384, 0x3, 0},
        {"13-inch RGB",       640,  480, 0x1, 0},
        {"15-inch Portrait",  640,  870, 0x2, 0},
        {"16-inch Color",     832,  624, 0x0, 4},
        {"19-inch Color",    1024,  768, 0x0, 1},
        {"21-inch Color",    1152,  870, 0x4, 0},
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

void QuadraMachine::insertHardDisk(std::vector<u8> image, bool readOnly) {
    hd_ = std::move(image);
    hdRO_ = readOnly;
    if (hd_.empty()) {
        disk_->detach();
        scsiImage_.clear();
        return;
    }
    scsiImage_ = scsi::buildAppleScsiDisk(hd_, scsi::buildScsiDriverPortable(0, 4, 1));
    hfsImageOffset_ = static_cast<u32>(scsiImage_.size() - hd_.size());
    disk_->attach(&scsiImage_, 0);
    disk_->readOnly = readOnly;
    // A volume left flagged "in use" is one whose machine stopped without
    // unmounting it -- a crash, a kill, the host losing power. That is not a
    // damaged volume and a real Macintosh mounts it: this ROM rebuilds it first
    // (recounting the catalog and remaking the bitmap), then puts up System
    // 7.5's "may not have been shut down properly" notice.
    //
    // What it cannot do is rebuild a volume whose extents-overflow B*-tree is
    // bigger than its own budget arithmetic can measure -- see
    // hfs::shrinkExtentsTree. Volumes this formatter made before 2026-08-03 are
    // all over that line, which is why one bad exit used to leave a disk that
    // would never boot again. Put them back inside it, once, in place, giving
    // back only nodes the tree is not using.
    if (!readOnly && !suppressVolumeRepair) {
        std::string why;
        const bool changed =
            hfs::shrinkExtentsTree(scsiImage_, hfsImageOffset_,
                                   static_cast<u32>(hd_.size()),
                                   hfs::kRepairableExtentsBytes, why);
        if (onDiag && !why.empty())
            onDiag((std::string("hd: ") + why).c_str());
        (void)changed;
    }
    hdAnnouncePending_ = true;
    hdAnnounceDelay_ = 0;
    hdMountTries_ = 0;
}

const std::vector<u8>& QuadraMachine::hardDiskImage() const {
    if (!scsiImage_.empty() &&
        static_cast<std::size_t>(hfsImageOffset_) + hd_.size() <= scsiImage_.size())
        hd_.assign(scsiImage_.begin() + hfsImageOffset_,
                   scsiImage_.begin() + hfsImageOffset_ + hd_.size());
    return hd_;
}

void QuadraMachine::insertHardDisk2(std::vector<u8> image, bool readOnly) {
    hd2_ = std::move(image);
    hd2RO_ = readOnly;
    if (hd2_.empty()) {
        disk2_->detach();
        scsiImage2_.clear();
        return;
    }
    scsiImage2_ = scsi::buildAppleScsiDisk(hd2_, scsi::buildScsiDriverPortable(1, 5, 33));
    hfsImageOffset2_ = static_cast<u32>(scsiImage2_.size() - hd2_.size());
    disk2_->attach(&scsiImage2_, 1);
    disk2_->readOnly = readOnly;
    // The same repair the boot disk gets: a volume put on this seat may be an
    // old image of the user's too, and it will be mounted by the same ROM.
    if (!readOnly && !suppressVolumeRepair) {
        std::string why;
        hfs::shrinkExtentsTree(scsiImage2_, hfsImageOffset2_,
                               static_cast<u32>(hd2_.size()),
                               hfs::kRepairableExtentsBytes, why);
        if (onDiag && !why.empty())
            onDiag((std::string("hd2: ") + why).c_str());
    }
    hd2AnnouncePending_ = true;
    hdAnnounceDelay_ = 0;
    // The try counter is a give-up for one disk's announcement, not a budget
    // for the machine's lifetime. A drop box republishes this seat over and
    // over; without this reset the boot's own tries exhaust the cap and every
    // later volume is dropped without a single _MountVol ever being issued.
    hdMountTries_ = 0;
}

void QuadraMachine::detachHardDisk2() {
    hd2_.clear();
    scsiImage2_.clear();
    disk2_->detach();
}

const std::vector<u8>& QuadraMachine::hardDisk2Image() const {
    if (!scsiImage2_.empty() &&
        static_cast<std::size_t>(hfsImageOffset2_) + hd2_.size() <= scsiImage2_.size())
        hd2_.assign(scsiImage2_.begin() + hfsImageOffset2_,
                    scsiImage2_.begin() + hfsImageOffset2_ + hd2_.size());
    return hd2_;
}

// 'PRAM', a version, the 256 bytes of XPRAM, then the clock. Written and read
// as bytes in a fixed order so a file survives a rebuild, and never
// interpreted -- see the header for why that matters.
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

void QuadraMachine::attachCdRom(bool attached, int busId) {
    cdrom_->setAttached(attached, busId);
    if (onDiag) {
        char b[64];
        std::snprintf(b, sizeof b, "cd: drive %s (SCSI ID %d)",
                      attached ? "attached" : "detached", busId & 7);
        onDiag(b);
    }
}
bool QuadraMachine::cdRomAttached() const { return cdrom_->attachedState(); }
// The Classic has peeled CD containers since its drive was built; this seat
// never did, so a raw-sector .bin went in with its 2352-byte framing still on
// it, and a file the drive cannot serve was accepted in silence. Same code and
// same words on both machines now.
int QuadraMachine::insertCd(std::vector<u8> image) {
    cd::Medium m = cd::normalize(std::move(image));
    if (onDiag) {
        char b[240];
        std::snprintf(b, sizeof b, "cd: %s%s", m.ok ? "inserted -- " : "refused: ",
                      m.desc);
        onDiag(b);
    }
    if (!m.ok) return 0;
    cdrom_->insert(std::move(m.data));
    // Where the volume starts on this disc, and the drive's own view of it.
    cdHfsOffset_ = findHfsPartition(cdrom_->media());
    if (onDiag && cdHfsOffset_) {
        char b[96];
        std::snprintf(b, sizeof b, "cd: Apple_HFS partition at byte %u",
                      cdHfsOffset_);
        onDiag(b);
    }
    if (cdStatus_) write8(cdStatus_ + 3, 1);          // dsDiskInPlace
    cdMountPending_ = true;
    cdMountTries_ = 0;
    return 1;
}
// The front end's Eject only STAGES: the disc has a volume mounted on it, and
// taking the medium out from under one leaves the guest holding a volume with
// nothing behind it -- offLinErr forever, which is exactly what the floppy path
// had to learn. completeCdEject unmounts first, at a frame edge, on the
// emulation thread.
void QuadraMachine::ejectCd() {
    if (!cdrom_->discPresent()) return;
    cdEjectPending_ = true;
}

void QuadraMachine::completeCdEject() {
    if (!cdEjectPending_) return;
    if (inDriver_ || announceInFlight_) return;
    if (cdDrive_ && read32(0x0358) != 0 && !(read8(0x0360) & 1) &&
        (cpu_.getSR() & 0x0700) == 0 && volumeMountedFor(cdDrive_)) {
        announceInFlight_ = true;
        u32 sd[8], sa[8];
        for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
        if (!hdMountPb_) {
            cpu_.d[0] = 80;
            execute68kTrap(0xA71E);                   // _NewPtr,Sys,Clear
            hdMountPb_ = guestPtr(cpu_.a[0]);
        }
        s16 un = -1;
        if (hdMountPb_) {
            for (u32 i = 0; i < 80; ++i) write8(hdMountPb_ + i, 0);
            write16(hdMountPb_ + 22, cdDrive_);       // ioVRefNum
            cpu_.a[0] = hdMountPb_;
            execute68kTrap(0xA00E);                   // _UnmountVol
            un = static_cast<s16>(cpu_.d[0] & 0xFFFF);
        }
        for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
        announceInFlight_ = false;
        if (un != 0) {
            // The guest is still using it (an open file, an application running
            // off the disc). A real drive refuses too; try again next tick.
            if (onDiag && cdEjectTries_ == 0) {
                char b[80];
                std::snprintf(b, sizeof b, "cd: the guest is still using the disc (%d)", un);
                onDiag(b);
            }
            if (++cdEjectTries_ < 20) return;
        }
    }
    cdEjectPending_ = false;
    cdEjectTries_ = 0;
    cdrom_->eject();
    cdHfsOffset_ = 0;
    cdMountPending_ = false;
    if (cdStatus_) write8(cdStatus_ + 3, 0);          // dsDiskInPlace
    if (onDiag) onDiag("cd: disc taken out by the host");
}
bool QuadraMachine::cdPresent() const { return cdrom_->discPresent(); }

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
