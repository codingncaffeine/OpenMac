#include "openmac/se30.hpp"

#include "../adb.hpp"
#include "../adb_via.hpp"
#include "../asc.hpp"
#include "../direct_floppy.hpp"
#include "../quadra/scc.hpp"
#include "../rtc.hpp"
#include "../scsi.hpp"
#include "../scsicd.hpp"
#include "../scsinet.hpp"
#include "../via.hpp"
#include "video_rom.hpp"
#include "video.hpp"

#include <cstdio>
#include <cstring>

// Macintosh SE/30. Every address, line and constant here is in the hardware
// dossier with its source; the short form:
//
//   $0000 0000  RAM, two banks; VIA2 port A bits 7-6 pick the address line
//               (A20, A22, A24, A26) that selects bank B
//   $4000 0000  ROM, repeating; over the RAM space while the overlay is on
//   $50F0 0000  VIA1      $50F0 2000  VIA2      $50F0 4000  serial chip
//   $50F0 6000  SCSI data with handshake        $50F1 0000  SCSI registers
//   $50F1 2000  SCSI data                       $50F1 4000  sound chip
//   $50F1 6000  floppy chip
//
// The SCSI chip has three faces. $50F1 0000 is its eight registers, n at
// n * $10. The other two reach its data register through /DACK, whatever
// the offset (the ROM reads at +$60 and writes at +0): $50F1 2000 completes
// the cycle at once, $50F0 6000 holds it until the chip's data request and
// ends it in a bus error when none comes. The chip's data request and
// interrupt outputs also go to VIA2's CA2 and CB2.
//   $FE00 0000  video RAM, 64 KB repeating      $FEFF 0000  video ROM
//
// The device window repeats every $2 0000 through $5000 0000-$57FF FFFF.
// Anything else is a bus error (the board's 44-microsecond watchdog).

namespace openmac {

namespace {

constexpr u64 kCpuHz = 15667200;        // 68030 at 15.6672 MHz, also the dot clock
constexpr int kClocksPerLine = 704;     // 512 shown and 192 blanked dots
constexpr int kLinesPerFrame = 370;     // 342 shown and 28 blanked lines
constexpr int kVisibleLines = 342;
constexpr int kClocksPerViaTick = 20;   // the VIAs count 783.36 kHz
// ADB: a byte completes about 0.69 ms after the transceiver is told to move
// it, the Classic's figure at this machine's clock.
constexpr int kAdbShiftClocks = 10880;
// VIA2 port B inputs: bit 6 low (sound always stereo), bits 5-4 low (no slot
// transfer error), bit 3 low (the machine's identity), the rest high.
constexpr u8 kVia2PortB = 0x87;
// The board ends any bus cycle that lasts 44 microseconds in a bus error
// (Designing Cards and Drivers, 3rd ed., the SE/30's watchdog timer).
constexpr int kWatchdogClocks = 689;
// Bus cycles (Guide to the Macintosh Family Hardware 2nd ed., ch. 3). The
// processor model counts two clocks for an operand's access and none for an
// instruction fetch (the 68030 manual's cache-case times); the board's
// cycles are longer, and takeCyclePenalty() hands the difference over. RAM
// and ROM answer an asynchronous cycle (three clocks) with one wait state.
// Every device is an 8-bit port, one cycle a byte: the floppy chip and the
// SCSI chip with one wait state, the sound chip with two for a read and one
// for a write; two accesses to the serial chip are held 2.2 microseconds
// apart; a VIA access starts on the VIA's own 783.36 kHz clock and ends half
// of one later (the rule the IIfx's VIAs follow). The video RAM's wait
// states are in no document in hand: it is given the other devices' cycle.
// With these a real SE/30's Speedometer 3.06 figures are met to within 3%
// (the dossier, "Speed").
constexpr int kNominalCycle = 2;
constexpr int kMemoryCycle = 4;
constexpr int kDeviceCycle = 4;
constexpr int kSoundReadCycle = 5;
constexpr int kSoundWriteCycle = 4;
constexpr int kSerialCycle = 4;
constexpr int kSerialGapClocks = 35;
constexpr int kVideoCycle = 4;
// A target's time from the last byte of a write to its Status: 100
// microseconds. A model constant; no document gives a drive's figure, and
// only its order matters (the ROM reads the phase lines right after the last
// byte and expects them unchanged, see the dossier).
constexpr int kScsiStatusDelayClocks = 1567;

u32 be32(const u8* p) {
    return (static_cast<u32>(p[0]) << 24) | (static_cast<u32>(p[1]) << 16) |
           (static_cast<u32>(p[2]) << 8) | p[3];
}

// A bank holds four equal SIMMs of 256 KB, 1 MB, 4 MB or 16 MB.
u32 bankBytes(u32 mb) {
    return (mb == 1 || mb == 4 || mb == 16 || mb == 64) ? mb << 20 : 0;
}

struct RamSet { u32 total, a, b; };
constexpr RamSet kRamSets[13] = {
    {1, 1, 0},   {2, 1, 1},    {4, 4, 0},   {5, 4, 1},   {8, 4, 4},
    {16, 16, 0}, {17, 16, 1},  {20, 16, 4}, {32, 16, 16},
    {65, 64, 1}, {68, 64, 4},  {80, 64, 16}, {128, 64, 64},
};

} // namespace

const std::array<u32, 13>& Se30Machine::ramSizesMb() {
    static const std::array<u32, 13> sizes = [] {
        std::array<u32, 13> s{};
        for (std::size_t i = 0; i < s.size(); ++i) s[i] = kRamSets[i].total;
        return s;
    }();
    return sizes;
}

bool Se30Machine::banksForTotal(u32 totalMb, u32& bankAMb, u32& bankBMb) {
    for (const RamSet& s : kRamSets) {
        if (s.total != totalMb) continue;
        bankAMb = s.a;
        bankBMb = s.b;
        return true;
    }
    return false;
}

Se30Machine::Se30Machine(std::vector<u8> rom, const Config& cfg)
    : MacHostServices(*this, cpu_, ram_),
      rom_(std::move(rom)),
      videoRom_(cfg.videoRom.empty() && !cfg.emptyVideoSlot ? Se30VideoRom::build() : cfg.videoRom),
      ownVideoRom_(cfg.videoRom.empty() && !cfg.emptyVideoSlot),
      bankA_(bankBytes(cfg.bankAMb)),
      bankB_(bankBytes(cfg.bankBMb)),
      via1_(std::make_unique<Via6522>()),
      via2_(std::make_unique<Via6522>()),
      rtc_(std::make_unique<Rtc>()),
      adb_(std::make_unique<AdbTransceiver>()),
      adbPort_(std::make_unique<AdbViaPort>(*adb_, *via1_, kAdbShiftClocks)),
      scc_(std::make_unique<Scc8530>()),
      scsi_(std::make_unique<Ncr5380>()),
      netdev_(std::make_unique<ScsiEthernet>()),
      asc_(std::make_unique<Asc>()),
      floppy_(std::make_unique<DirectFloppy>(kCpuHz)),
      video_(std::make_unique<Se30Video>()),
      cpu_(*this) {
    ram_.assign(static_cast<std::size_t>(bankA_) + bankB_, 0);
    u32 rs = 1;
    while (rs < rom_.size()) rs <<= 1;
    rom_.resize(rs, 0xFF);
    romMask_ = rs - 1;
    romEntry_ = be32(&rom_[4]);
    if (!videoRom_.empty()) {
        // The declaration ROM sits at the top of the slot's space: a file
        // shorter than a power of two is padded at its start.
        u32 vs = 1;
        while (vs < videoRom_.size()) vs <<= 1;
        videoRom_.insert(videoRom_.begin(), vs - videoRom_.size(), 0xFF);
        videoRomMask_ = vs - 1;
    }
    wireDevices();
    reset();
}

Se30Machine::~Se30Machine() = default;

void Se30Machine::wireDevices() {
    // ---- VIA1 (dossier, "VIA1") ---------------------------------------------
    // Port A inputs read high: bit 7 the serial chip's wait/request line at
    // rest, bit 6 the machine's identity until the video ROM makes it the
    // screen-buffer select.
    via1_->inA = [] { return static_cast<u8>(0xFF); };
    via1_->inB = [this] {
        u8 v = 0xFF;
        if (!rtc_->dataOut()) v = static_cast<u8>(v & ~0x01);
        if (!adb_->intLine()) v = static_cast<u8>(v & ~0x08);
        return v;
    };
    via1_->outA = [this](u8 value, u8 ddr) {
        // Bit 4 is the overlay line; an input reads high, which is the
        // overlay on. Bit 6 picks the screen buffer.
        overlay_ = !(ddr & 0x10) || (value & 0x10);
        video_->selectMainBuffer(!(ddr & 0x40) || (value & 0x40));
        floppy_->setHeadSelect((value & 0x20) != 0);      // bit 5: the drive's SEL line
    };
    via1_->outB = [this](u8 value, u8 ddr) {
        const u8 eff = static_cast<u8>(value | ~ddr);
        rtc_->setLines((eff & 0x01) != 0, (eff & 0x02) != 0, (eff & 0x04) != 0);
        adbPort_->setState((eff >> 4) & 3);
        video_->setInterruptEnabled((eff & 0x40) == 0);
        updateIpl();
    };
    via1_->srArmed = [this](bool input) { adbPort_->armed(input); };
    via1_->srDisarmed = [this] { adbPort_->disarmed(); };

    // ---- VIA2 (dossier, "VIA2") ---------------------------------------------
    via2_->inA = [this] {
        // Bits 5-0 are the slot interrupt lines, low = asking. Slot $E is
        // the screen; the direct slot's three lines and slots $C, $D rest.
        return static_cast<u8>(0xFFu & ~(video_->interruptRequested() ? 0x20u : 0u));
    };
    via2_->inB = [] { return kVia2PortB; };
    via2_->outA = [this](u8 value, u8 ddr) {
        sizeBits_ = static_cast<u8>(static_cast<u8>(value | ~ddr) >> 6);
        updateRamDecode();
    };

    video_->onInterrupt = [this](bool asking) {
        if (asking) ++screenInterrupts_;
        via2_->setCA1(!asking);   // all slot requests ORed, low = asking
        updateIpl();
    };
    asc_->onIrq = [this](bool level) {
        via2_->setCB1(!level);
        updateIpl();
    };
    asc_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    floppy_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    scc_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    scc_->setClocks(kCpuHz, 3672000);

    // ---- SCSI (dossier, "SCSI") ---------------------------------------------
    // One bus: the two hard-disk seats at IDs 0 and 1, the CD-ROM drive
    // (ID 3 when attached) and the network adapter (ID 4 when attached).
    scsi_->dmaNeedsPhaseMatch = true;     // the board holds a cycle on the DRQ pin
    scsi_->statusDelayClocks = kScsiStatusDelayClocks;
    scsi_->addTarget(disk_.get());
    scsi_->addTarget(disk2_.get());
    scsi_->addTarget(cdrom_.get());
    scsi_->addTarget(netdev_.get());
    // The vendor mode page Apple's CD-ROM driver looks for, and a tray, as on
    // the Quadras. (This ROM does not start up from a CD.)
    cdrom_->setStartupDrive(true);
    cdrom_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };
    netdev_->onDiag = [this](const char* m) { if (onDiag) onDiag(m); };

    cpu_.onResetInstruction = [this] { resetPeripherals(); };
}

// The RESET line: the processor's RESET instruction pulses it, and power on
// holds it. Both VIAs go back to all inputs, which puts the overlay on, the
// RAM-size bits at 11 and the screen interrupt off.
void Se30Machine::resetPeripherals() {
    via1_->reset();
    via2_->reset();
    overlay_ = true;
    sizeBits_ = 3;
    updateRamDecode();
    video_->reset();
    adb_->reset();
    adbPort_->reset();
    scc_->reset();
    scsi_->reset();
    asc_->reset();
    floppy_->reset();
    floppy_->setHeadSelect(false);
    // The lines' resting levels, set after the reset so no edge is latched:
    // the slot and sound requests rest high; the SCSI chip's two outputs are
    // active high and rest low.
    via2Pb7_ = via2_->pb7();
    via1_->setCA1(via2Pb7_);
    via1_->setCA2(true);
    via2_->setCA1(true);
    via2_->setCB1(true);
    via2_->setCA2(false);
    via2_->setCB2(false);
    secondPulseLines_ = 0;
    busWaitClocks_ = 0;
    serialFreeAt_ = 0;
    fetchesCounted_ = cpu_.cacheStats030().instructionMisses;
    updateIpl();
}

void Se30Machine::reset() {
    rtc_->reset();            // protocol state only; time and parameter RAM survive
    resetPeripherals();
    screenInterrupts_ = 0;
    secondsUnanswered_ = 0;
    unansweredAtFrame_ = {};
    totalCycles_ = 0;
    viaRemainder_ = 0;
    lineAcc_ = 0;
    line_ = 0;
    secondAcc_ = 0;
    audioOut_.clear();
    audioStereo_.clear();
    cpu_.reset();
    fetchesCounted_ = 0;      // the processor's own counts start again with it
}

void Se30Machine::updateIpl() {
    // Autovectors: level 4 the serial chip, 2 VIA2, 1 VIA1.
    int ipl = 0;
    if (scc_->irqAsserted()) ipl = 4;
    else if (via2_->irqAsserted()) ipl = 2;
    else if (via1_->irqAsserted()) ipl = 1;
    cpu_.setIrqLevel(ipl);
}

// ---- RAM ------------------------------------------------------------------------

void Se30Machine::updateRamDecode() {
    const u32 line = 0x100000u << (2 * sizeBits_);
    identityRam_ = bankA_ != 0 && line == bankA_ && bankB_ <= bankA_;
}

u32 Se30Machine::ramIndex(u32 addr) const {
    // One address line selects the bank; the lines above it are not decoded,
    // and each bank repeats by its own size.
    const u32 line = 0x100000u << (2 * sizeBits_);
    if (addr & line) return bankB_ ? bankA_ + (addr & (bankB_ - 1)) : kNoRam;
    return bankA_ ? (addr & (bankA_ - 1)) : kNoRam;
}

// The chip's two outputs as VIA2 sees them: data request on CA2, interrupt
// on CB2, both high when asserted. Nothing moves on the bus between two
// accesses to the chip, so the lines are brought up to date after each one.
void Se30Machine::updateScsiLines() {
    via2_->setCA2(scsi_->dmaRequestAsserted());
    via2_->setCB2(scsi_->irqAsserted());
    updateIpl();
}

// A cycle at the handshake address: true when the chip asks for a byte, and
// the cycle completes with it. With none coming the board's watchdog ends
// the cycle 44 microseconds on, in a bus error (false).
bool Se30Machine::scsiHandshakeCycle() {
    ++scsiHandshakeCycles_;
    if (scsi_->dmaRequestAsserted()) return true;
    ++scsiHandshakeTimeouts_;
    return false;
}

// A VIA cycle begins on the next edge of the VIA's clock and ends half a
// VIA clock later: its length from now, in processor clocks.
int Se30Machine::viaCycleClocks() const {
    const u64 now = totalCycles_ + static_cast<u64>(busWaitClocks_ > 0 ? busWaitClocks_ : 0);
    const int phase = static_cast<int>(now % kClocksPerViaTick);
    return (kClocksPerViaTick - phase) % kClocksPerViaTick + kClocksPerViaTick / 2;
}

// The serial chip wants 2.2 microseconds between two accesses: the board
// holds the second one off for what is left of that.
int Se30Machine::serialCycleClocks() {
    const u64 now = totalCycles_ + static_cast<u64>(busWaitClocks_ > 0 ? busWaitClocks_ : 0);
    int clocks = kSerialCycle;
    if (now < serialFreeAt_) clocks += static_cast<int>(serialFreeAt_ - now);
    serialFreeAt_ = now + static_cast<u64>(clocks) + kSerialGapClocks;
    return clocks;
}

int Se30Machine::takeCyclePenalty() {
    // An instruction fetch the 68030's cache could not answer was a memory
    // cycle too, and the processor's times count none of it: the two clocks
    // every other access already carries are added here.
    const u64 fetches = cpu_.cacheStats030().instructionMisses;
    const int clocks = busWaitClocks_ + static_cast<int>(fetches - fetchesCounted_) * kNominalCycle;
    fetchesCounted_ = fetches;
    busWaitClocks_ = 0;
    return clocks;
}

// ---- bus ------------------------------------------------------------------------

namespace {

constexpr u32 kRamSpaceEnd = 0x40000000u;
constexpr u32 kRomEnd = 0x50000000u;
constexpr u32 kDeviceEnd = 0x58000000u;
constexpr u32 kVideoBase = 0xFE000000u;
constexpr u32 kVideoRomBase = 0xFEFF0000u;
// What an empty bank's cycle leaves on the data bus.
constexpr u8 kFloatingBus = 0x71;

} // namespace

bool Se30Machine::deviceRead8(u32 addr, u8& out) {
    if (addr < kRamSpaceEnd) {
        if (overlay_) {
            out = rom_[addr & romMask_];
        } else {
            const u32 at = ramIndex(addr);
            out = at == kNoRam ? kFloatingBus : ram_[at];
        }
        return true;
    }
    if (addr < kRomEnd) {
        out = rom_[addr & romMask_];
        return true;
    }
    if (addr < kDeviceEnd) {
        const u32 off = addr & 0x1FFFFu;
        switch (off >> 13) {
            case 0: busWaitClocks_ += viaCycleClocks(); out = via1_->read(static_cast<int>((off >> 9) & 15)); updateIpl(); return true;
            case 1: busWaitClocks_ += viaCycleClocks(); out = via2_->read(static_cast<int>((off >> 9) & 15)); updateIpl(); return true;
            case 2: busWaitClocks_ += serialCycleClocks(); out = scc_->read(off & 6u); updateIpl(); return true;
            case 3:                                    // SCSI data, cycle held for the data request
                if (!scsiHandshakeCycle()) return false;
                busWaitClocks_ += kDeviceCycle;
                out = scsi_->read(6);
                updateScsiLines();
                return true;
            case 8: {                                  // SCSI registers
                busWaitClocks_ += kDeviceCycle;
                const int reg = static_cast<int>((off >> 4) & 7);
                // Register 6 without /DACK shows the byte and moves nothing.
                out = reg == 6 ? scsi_->inputDataNoAck() : scsi_->read(reg);
                updateScsiLines();
                return true;
            }
            case 9:                                    // SCSI data, no wait
                ++scsiPlainCycles_;
                busWaitClocks_ += kDeviceCycle;
                out = scsi_->read(6);
                updateScsiLines();
                return true;
            case 10: busWaitClocks_ += kSoundReadCycle; out = asc_->read(off & 0xFFFu); return true;
            case 11: busWaitClocks_ += kDeviceCycle; out = floppy_->access(static_cast<int>((off >> 9) & 15), false, 0, totalCycles_); return true;
            default: return false;
        }
    }
    if (addr >= kVideoBase) {
        if (addr < kVideoRomBase) {
            busWaitClocks_ += kVideoCycle;
            out = video_->read(addr);
            return true;
        }
        if (addr <= 0xFEFFFFFFu && !videoRom_.empty()) {
            busWaitClocks_ += kVideoCycle;
            out = videoRom_[addr & videoRomMask_];
            return true;
        }
    }
    return false;
}

bool Se30Machine::deviceWrite8(u32 addr, u8 value) {
    if (addr < kRamSpaceEnd) {
        if (overlay_) return true;            // RAM is not reachable under the overlay
        const u32 at = ramIndex(addr);
        if (at != kNoRam) ram_[at] = value;
        return true;
    }
    if (addr < kRomEnd) return true;
    if (addr < kDeviceEnd) {
        const u32 off = addr & 0x1FFFFu;
        switch (off >> 13) {
            case 0: busWaitClocks_ += viaCycleClocks(); via1_->write(static_cast<int>((off >> 9) & 15), value); updateIpl(); return true;
            case 1: busWaitClocks_ += viaCycleClocks(); via2_->write(static_cast<int>((off >> 9) & 15), value); updateIpl(); return true;
            case 2: busWaitClocks_ += serialCycleClocks(); scc_->write(off & 6u, value); updateIpl(); return true;
            case 3:
                if (!scsiHandshakeCycle()) return false;
                busWaitClocks_ += kDeviceCycle;
                scsi_->write(0, value);
                updateScsiLines();
                return true;
            case 8:
                busWaitClocks_ += kDeviceCycle;
                scsi_->write(static_cast<int>((off >> 4) & 7), value);
                updateScsiLines();
                return true;
            case 9:
                ++scsiPlainCycles_;
                busWaitClocks_ += kDeviceCycle;
                scsi_->write(0, value);
                updateScsiLines();
                return true;
            case 10: busWaitClocks_ += kSoundWriteCycle; asc_->write(off & 0xFFFu, value); return true;
            case 11: busWaitClocks_ += kDeviceCycle; (void)floppy_->access(static_cast<int>((off >> 9) & 15), true, value, totalCycles_); return true;
            default: return false;
        }
    }
    if (addr >= kVideoBase) {
        if (addr < kVideoRomBase) {
            busWaitClocks_ += kVideoCycle;
            video_->write(addr, value);
            return true;
        }
        if (addr <= 0xFEFFFFFFu && !videoRom_.empty()) { busWaitClocks_ += kVideoCycle; return true; }
    }
    return false;
}

// The six bus functions. RAM and ROM are 32 bits wide: one cycle whatever
// the operand's size. Everything else is an 8-bit port: one cycle a byte,
// at consecutive addresses, each already counted where the device answered.
// A cycle nobody answers is ended by the watchdog.
void Se30Machine::chargeAccess(u32 addr) {
    busWaitClocks_ += (addr < kRomEnd ? kMemoryCycle : 0) - kNominalCycle;
}

u8 Se30Machine::read8(u32 addr) {
    if (identityRam_ && !overlay_ && addr < ram_.size()) {
        busWaitClocks_ += kMemoryCycle - kNominalCycle;
        return ram_[addr];
    }
    u8 v = 0;
    if (!deviceRead8(addr, v)) { busWaitClocks_ += kWatchdogClocks; throw BusFault{addr, true, 1}; }
    chargeAccess(addr);
    return v;
}

u16 Se30Machine::read16(u32 addr) {
    if (identityRam_ && !overlay_ && addr < ram_.size() && ram_.size() - addr >= 2) {
        busWaitClocks_ += kMemoryCycle - kNominalCycle;
        return static_cast<u16>((ram_[addr] << 8) | ram_[addr + 1]);
    }
    u8 hi = 0, lo = 0;
    if (!deviceRead8(addr, hi) || !deviceRead8(addr + 1, lo)) {
        busWaitClocks_ += kWatchdogClocks;
        throw BusFault{addr, true, 2};
    }
    chargeAccess(addr);
    return static_cast<u16>((hi << 8) | lo);
}

u32 Se30Machine::read32(u32 addr) {
    if (identityRam_ && !overlay_ && addr < ram_.size() && ram_.size() - addr >= 4) {
        busWaitClocks_ += kMemoryCycle - kNominalCycle;
        return be32(&ram_[addr]);
    }
    u32 v = 0;
    for (u32 i = 0; i < 4; ++i) {
        u8 b = 0;
        if (!deviceRead8(addr + i, b)) { busWaitClocks_ += kWatchdogClocks; throw BusFault{addr, true, 4}; }
        v = (v << 8) | b;
    }
    chargeAccess(addr);
    return v;
}

void Se30Machine::write8(u32 addr, u8 value) {
    if (identityRam_ && !overlay_ && addr < ram_.size()) {
        busWaitClocks_ += kMemoryCycle - kNominalCycle;
        ram_[addr] = value;
        return;
    }
    if (!deviceWrite8(addr, value)) { busWaitClocks_ += kWatchdogClocks; throw BusFault{addr, false, 1}; }
    chargeAccess(addr);
}

void Se30Machine::write16(u32 addr, u16 value) {
    if (identityRam_ && !overlay_ && addr < ram_.size() && ram_.size() - addr >= 2) {
        busWaitClocks_ += kMemoryCycle - kNominalCycle;
        ram_[addr] = static_cast<u8>(value >> 8);
        ram_[addr + 1] = static_cast<u8>(value);
        return;
    }
    if (!deviceWrite8(addr, static_cast<u8>(value >> 8)) || !deviceWrite8(addr + 1, static_cast<u8>(value))) {
        busWaitClocks_ += kWatchdogClocks;
        throw BusFault{addr, false, 2};
    }
    chargeAccess(addr);
}

void Se30Machine::write32(u32 addr, u32 value) {
    if (identityRam_ && !overlay_ && addr < ram_.size() && ram_.size() - addr >= 4) {
        busWaitClocks_ += kMemoryCycle - kNominalCycle;
        ram_[addr] = static_cast<u8>(value >> 24);
        ram_[addr + 1] = static_cast<u8>(value >> 16);
        ram_[addr + 2] = static_cast<u8>(value >> 8);
        ram_[addr + 3] = static_cast<u8>(value);
        return;
    }
    for (u32 i = 0; i < 4; ++i) {
        if (!deviceWrite8(addr + i, static_cast<u8>(value >> (8 * (3 - i))))) {
            busWaitClocks_ += kWatchdogClocks;
            throw BusFault{addr, false, 4};
        }
    }
    chargeAccess(addr);
}

// ---- time -----------------------------------------------------------------------

void Se30Machine::tickDevices(int cpuCycles) {
    totalCycles_ += static_cast<u64>(cpuCycles);

    viaRemainder_ += cpuCycles;
    if (viaRemainder_ >= kClocksPerViaTick) {
        const int ticks = viaRemainder_ / kClocksPerViaTick;
        viaRemainder_ %= kClocksPerViaTick;
        via1_->tick(ticks);
        via2_->tick(ticks);
        // VIA2's timer 1 output (port B bit 7) is the 60.15 Hz tick on VIA1's CA1.
        const bool pb7 = via2_->pb7();
        if (pb7 != via2Pb7_) {
            via2Pb7_ = pb7;
            via1_->setCA1(pb7);
        }
    }
    adbPort_->tick(cpuCycles);
    scc_->tick(static_cast<s32>(cpuCycles));
    if (scsi_->advance(cpuCycles)) updateScsiLines();

    secondAcc_ += static_cast<u64>(cpuCycles);
    if (secondAcc_ >= kCpuHz) {
        secondAcc_ -= kCpuHz;
        rtc_->tickSecond();
        if (via1_->ifr() & 0x01) {     // the last pulse's flag is still standing
            if (secondsUnanswered_ < unansweredAtFrame_.size())
                unansweredAtFrame_[secondsUnanswered_] = static_cast<u32>(frameCounter_);
            ++secondsUnanswered_;
        }
        via1_->setCA2(false);          // the clock chip's one-second pulse
        secondPulseLines_ = 2;
        storageSecondTick();
    }

    lineAcc_ += cpuCycles;
    while (lineAcc_ >= kClocksPerLine) {
        lineAcc_ -= kClocksPerLine;
        // One sound sample a line: 22,254.5 Hz, the sound chip's own rate.
        const Asc::MixedSample sample = asc_->pullMixedSample();
        if (audioOut_.size() < 8192) audioOut_.push_back(sample.mono);
        if (audioStereo_.size() < 2 * 8192) {
            audioStereo_.push_back(sample.left);
            audioStereo_.push_back(sample.right);
        }
        if (secondPulseLines_ > 0 && --secondPulseLines_ == 0) via1_->setCA2(true);
        ++line_;
        if (line_ == kVisibleLines) {
            video_->verticalBlank();
            adbPort_->frameWake();
        } else if (line_ >= kLinesPerFrame) {
            line_ = 0;
            ++frameCounter_;
            if (floppyEjectPending_) {
                // Four seconds for the Macintosh to act on the host's Eject,
                // then the disk is taken out regardless.
                if (!floppy_->present()) {
                    floppyEjectPending_ = false;
                } else if (++floppyEjectFrames_ > 240) {
                    floppyEjectPending_ = false;
                    if (onDiag) onDiag("floppy: the Macintosh did not eject the disk; taken out");
                    floppy_->eject();
                }
            }
            storageFrameTick();
        }
    }
    updateIpl();
}

int Se30Machine::stepInstruction() {
    if (cpu_.pc == 0x4080000Au || (overlay_ && cpu_.pc == romEntry_)) machineStarting();
    // The board's longer bus cycles reach the processor through
    // takeCyclePenalty(), inside step(). What the host's own reads and
    // stores of guest memory added is not machine time and is dropped.
    if (serveStorageHooks(frameCounter_)) {
        busWaitClocks_ = 0;
        tickDevices(40);
        return 40;
    }
    const int c = cpu_.step();
    if (cpu_.heldAtCall) {
        // The CPU stopped in front of a call instead of executing it: the
        // moment work for the host was waiting for (MacHostServices).
        cpu_.heldAtCall = false;
        storageAtCall();
        busWaitClocks_ = 0;
        return 0;
    }
    tickDevices(c);
    busWaitClocks_ = 0;
    return c;
}

void Se30Machine::runFrame() {
    const u64 target = totalCycles_ + static_cast<u64>(kLinesPerFrame) * kClocksPerLine;
    while (totalCycles_ < target) {
        stepInstruction();
        if (cpu_.halted) return;
    }
}

// ---- front-end surface ----------------------------------------------------------

void Se30Machine::keyEvent(u8 adbCode, bool down) { adb_->injectKey(adbCode, down); }
void Se30Machine::mouseEvent(int dx, int dy, bool button) { adb_->injectMouse(dx, dy, button); }

bool Se30Machine::insertFloppy(std::vector<u8> image, bool readOnly) {
    return floppy_->insert(std::move(image), readOnly);
}
void Se30Machine::ejectFloppy() {
    if (!floppy_->present()) return;
    // Low memory describes a System only once this start-up has an
    // application running; before that it may still hold the last one's.
    if (overlay_ || !startupFinished() || !(fileSystemUp() && volumeMountedFor(1))) {
        // Nothing mounted from the drive: no volume to lose.
        floppy_->eject();
        return;
    }
    floppyEjectPending_ = true;
    floppyEjectAsked_ = false;
    floppyEjectFrames_ = 0;
    askForCall();
}

bool Se30Machine::boardWorkWanted() {
    return floppyEjectPending_ && !floppyEjectAsked_ && floppy_->present() && fileSystemUp() && volumeMountedFor(1);
}

void Se30Machine::boardWorkAtCall() { askGuestToEjectFloppy(); }

// The host's Eject for a disk the System has mounted, with the CPU in front
// of one of the guest's event calls: _FlushVol, then _Eject, on drive 1. That
// is what the Finder's own Eject Disk issues. The File Manager marks the
// volume ejected and has the ROM's driver command the drive, and the disk
// leaves through the mechanism as on a guest's own eject.
void Se30Machine::askGuestToEjectFloppy() {
    if (inDriver_ || announceInFlight_) return;
    if ((cpu_.getSR() & 0x0700) != 0) return;         // inside an interrupt
    if (addressingModeSwapped()) return;
    if ((read8(0x0360) & 1) != 0) return;             // the File Manager is in a call of its own
    announceInFlight_ = true;
    u32 sd[8], sa[8];
    for (int i = 0; i < 8; ++i) { sd[i] = cpu_.d[i]; sa[i] = cpu_.a[i]; }
    if (!hdMountPb_) {
        cpu_.d[0] = 80;
        execute68kTrap(0xA71E);                       // _NewPtr,Sys,Clear
        hdMountPb_ = guestPtr(cpu_.a[0]);
    }
    s16 flushed = 0, ejected = 0;
    if (hdMountPb_) {
        const auto call = [&](u16 trap) -> s16 {
            primeParamBlock(hdMountPb_, 1);           // drive 1
            cpu_.a[0] = hdMountPb_;
            execute68kTrap(trap);
            return static_cast<s16>(cpu_.d[0] & 0xFFFF);
        };
        flushed = call(0xA013);                       // _FlushVol
        ejected = call(0xA017);                       // _Eject
    }
    for (int i = 0; i < 8; ++i) { cpu_.d[i] = sd[i]; cpu_.a[i] = sa[i]; }
    announceInFlight_ = false;
    if (onDiag) {
        char b[96];
        std::snprintf(b, sizeof b, "floppy: the host's Eject asked the File Manager (flush %d, eject %d)", flushed, ejected);
        onDiag(b);
    }
    // Asked once. If the disk is still in the drive four seconds after the
    // host's Eject, the frame clock takes it out.
    if (ejected == 0) floppyEjectAsked_ = true;
}
bool Se30Machine::floppyPresent() const { return floppy_->present(); }
u32 Se30Machine::floppyEjects() const { return floppy_->ejects(); }
const std::vector<u8>& Se30Machine::floppySectors() { return floppy_->sectors(); }
std::vector<u8> Se30Machine::floppyForWriteBack() { return floppy_->fileImage(); }
Se30Machine::FloppyCounts Se30Machine::floppyCounts() const {
    return {floppy_->bytesRead(), floppy_->bytesWritten(), floppy_->tracksWritten(), floppy_->chip().ismSelected(),
            floppy_->mechanism().track};
}

void Se30Machine::attachNet(bool attached, int busId) { netdev_->setAttached(attached, busId); }
bool Se30Machine::netAttached() const { return netdev_->attachedState(); }
bool Se30Machine::netInject(const u8* frame, u32 len) { return netdev_->injectFrame(frame, len); }
bool Se30Machine::netDrain(std::vector<u8>& out) { return netdev_->drainFrame(out); }

Se30Machine::ScsiCounts Se30Machine::scsiCounts() const {
    ScsiCounts c{};
    c.selections = scsi_->diagSelects;
    c.commands = scsi_->diagCommands;
    c.bytesIn = scsi_->diagDataInBytes;
    c.bytesOut = scsi_->diagDataOutBytes;
    for (std::size_t id = 0; id < c.commandsById.size(); ++id) c.commandsById[id] = scsi_->diagCommandsById[id];
    c.plainDataCycles = scsiPlainCycles_;
    c.handshakeDataCycles = scsiHandshakeCycles_;
    c.handshakeTimeouts = scsiHandshakeTimeouts_;
    return c;
}

int Se30Machine::screenWidth() const { return Se30Video::kWidth; }
int Se30Machine::screenHeight() const { return Se30Video::kHeight; }
void Se30Machine::renderScreen(u32* argbOut) const { video_->render(argbOut); }

void Se30Machine::drainAudio(std::vector<u8>& out) {
    // The samples since the last call, replacing what the caller's buffer
    // held: the other machines' contract, which the C API relies on.
    out = std::move(audioOut_);
    audioOut_.clear();
}

void Se30Machine::drainAudioStereo(std::vector<u8>& out) {
    out = std::move(audioStereo_);
    audioStereo_.clear();
}

const std::array<u8, 256>& Se30Machine::pram() const { return rtc_->xpram(); }

// 'PRAM', a version, the 256 bytes, then the clock: the Quadras' layout, so
// a front end stores every machine's file the same way. Asked without a
// buffer (or with one too small) it answers the size.
u32 Se30Machine::savePram(u8* out, u32 cap) const {
    if (!out || cap < kPramBlobBytes) return kPramBlobBytes;
    std::memcpy(out, "PRAM", 4);
    out[4] = 0; out[5] = 0; out[6] = 0; out[7] = 1;
    std::memcpy(out + 8, rtc_->xpram().data(), 256);
    const u32 s = rtc_->seconds();
    out[264] = static_cast<u8>(s >> 24);
    out[265] = static_cast<u8>(s >> 16);
    out[266] = static_cast<u8>(s >> 8);
    out[267] = static_cast<u8>(s);
    return kPramBlobBytes;
}

bool Se30Machine::loadPram(const u8* data, u32 len, u32 addSeconds) {
    if (len != kPramBlobBytes || std::memcmp(data, "PRAM", 4) != 0) return false;
    if (be32(data + 4) != 1) return false;
    std::memcpy(rtc_->xpram().data(), data + 8, 256);
    rtc_->setSeconds(be32(data + 264) + addSeconds);
    return true;
}

Se30Machine::BoardState Se30Machine::boardState() const {
    return {sizeBits_, 0x100000u << (2 * sizeBits_), video_->mainBufferShown(), video_->interruptEnabled(),
            video_->interruptRequested(), screenInterrupts_, secondsUnanswered_, unansweredAtFrame_};
}

std::string Se30Machine::diagnosticReport() const {
    char b[640];
    std::snprintf(b, sizeof b,
                  "   cpu: pc=%08X sr=%04X vbr=%08X tc=%08X halted=%d stopped=%d\n"
                  "   board: overlay=%d RAM-size bits=%u (bank B from %08X), bank A %u MB, bank B %u MB\n"
                  "   VIA1: ora=%02X ddra=%02X orb=%02X ddrb=%02X acr=%02X pcr=%02X ifr=%02X ier=%02X\n"
                  "   VIA2: ora=%02X ddra=%02X orb=%02X ddrb=%02X acr=%02X pcr=%02X ifr=%02X ier=%02X\n"
                  "   screen: %s buffer, interrupt %s%s, %llu requests\n",
                  cpu_.pc, cpu_.getSR(), cpu_.vbr, cpu_.tc, cpu_.halted ? 1 : 0, cpu_.stopped ? 1 : 0,
                  overlay_ ? 1 : 0, sizeBits_, 0x100000u << (2 * sizeBits_), bankA_ >> 20, bankB_ >> 20,
                  via1_->ora(), via1_->ddra(), via1_->orb(), via1_->ddrb(), via1_->acr(), via1_->pcr(), via1_->ifr(), via1_->ier(),
                  via2_->ora(), via2_->ddra(), via2_->orb(), via2_->ddrb(), via2_->acr(), via2_->pcr(), via2_->ifr(), via2_->ier(),
                  video_->mainBufferShown() ? "main" : "alternate", video_->interruptEnabled() ? "enabled" : "off",
                  video_->interruptRequested() ? ", asking" : "", static_cast<unsigned long long>(screenInterrupts_));
    std::string report = b;
    char sound[1024];
    asc_->debugState(sound, sizeof sound);
    report += "   sound chip: ";
    report += sound;
    report += "\n";
    return report;
}

} // namespace openmac
