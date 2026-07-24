#pragma once

// Sony 3.5" floppy drive mechanism, as an IWM/SWIM-equipped Macintosh sees it.
//
// The drive multiplexes ~16 status lines onto a single sense wire; which one is
// presented is chosen by a 4-bit address CA2:CA1:CA0:SEL driven by the controller
// (CA0-2) and the VIA (SEL). Commands go the other way: the address CA1:CA0:SEL
// picks a control register, CA2 carries the data bit, and a pulse on LSTRB latches
// it. The ROM's .Sony driver packs both directions into one byte D0, unpacked by
// its address routine at $435154 as CA1:CA0:SEL:CA2 -- so for a command the low
// bit is the data and the upper three select the register.
//
// Status lines are ACTIVE LOW: the value returned here is the electrical line
// level, so `false` means the condition is asserted. Inside Macintosh III-36
// states each polarity explicitly, e.g. "CSTIN is 0 only when a disk is in the
// drive", "WRTPRT is 0 whenever the disk is locked", "TK0 goes to 0 only if the
// head is at track 0", "DRVIN is always 0 if the selected disk drive is
// physically connected... otherwise it floats to 1".
//
// Reference: Inside Macintosh Vol. III pp.33-36 (status/control register tables
// and polarities); SWIM Chip User's Reference Rev 1.5 pp.5-8; Guide to the
// Macintosh Family Hardware 2nd ed. Ch.9; Macintosh Classic Developer Note p.5
// (the Classic has one internal 1.4 MB SuperDrive and an external drive port).
// Clean-room from the specs.

#include "openmac/types.hpp"

#include <vector>

namespace openmac {

class SonyDrive {
public:
    // ---- configuration -------------------------------------------------
    bool installed   = false;   // drive physically connected to this port
    bool doubleSided = true;    // 800K/1.4MB mechanism (SIDES)
    bool superDrive  = true;    // FDHD: can do 1.4 MB MFM media

    // ---- media ---------------------------------------------------------
    // Not owned. Null (or empty) means no disk in the drive.
    std::vector<u8>* image = nullptr;
    bool readOnly    = false;
    bool hdMedia     = false;   // 1.4 MB high-density media rather than GCR

    bool hasDisk() const { return image != nullptr && !image->empty(); }

    // ---- mechanism state -----------------------------------------------
    int  track     = 0;         // 0..79
    bool headUpper = false;     // side select
    bool stepIn    = true;      // step direction: true = toward track 79

    void reset() {
        track = 0;
        headUpper = false;
        stepIn = true;
        motorWanted_ = false;
        stepDoneAt_ = motorUpAt_ = 0;
        diskSwitched_ = false;
        ejectAsserted_ = false;
        ejectSince_ = 0;
    }

    // A disk was inserted: latch the disk-switched line the driver polls to
    // notice the change, and park the head.
    void insert(std::vector<u8>* img, bool ro, bool hd) {
        image = img;
        readOnly = ro;
        hdMedia = hd;
        diskSwitched_ = true;
        track = 0;
    }

    void removeDisk() {
        image = nullptr;
        diskSwitched_ = true;
        motorWanted_ = false;
    }

    // True once the medium has been ejected by an EJECT command (the machine
    // polls this to drop the image and tell the host).
    bool takeEjectRequest() {
        const bool e = ejectRequested_;
        ejectRequested_ = false;
        return e;
    }

    bool motorRunning(u64 now) const {
        // The motor only runs when the drive is enabled and a disk is present
        // (Inside Macintosh III-36), and takes ~400 ms to reach speed.
        return motorWanted_ && hasDisk() && now >= motorUpAt_;
    }
    bool motorCommanded() const { return motorWanted_; }
    bool stepping(u64 now) const { return now < stepDoneAt_; }

    // ---- status lines --------------------------------------------------
    // `addr` is CA2:CA1:CA0:SEL. Returns the line level; active-low, so false
    // means asserted. An unconnected drive floats every line high.
    // Investigation aid: force individual status lines. Bit n of overrideMask
    // makes line n read the level in bit n of overrideValue. Several of the
    // SuperDrive-era line assignments are undocumented in both Inside Macintosh
    // (1985, single-sided drives only) and the SWIM reference, so the way to
    // settle them is to sweep them and watch what the ROM's own driver does.
    u16 overrideMask = 0;
    u16 overrideValue = 0;

    bool sense(int addr, u64 now) const {
        if (!installed) return true;
        if (overrideMask & (1u << (addr & 15)))
            return (overrideValue & (1u << (addr & 15))) != 0;
        switch (addr & 15) {
            case 0x0: return !stepIn;              // DIRTN: 0 = toward track 79
            case 0x1: return !hasDisk();           // CSTIN: 0 = disk in place
            case 0x2: return !stepping(now);       // STEP: 0 = still stepping
            case 0x3: return !(readOnly);          // WRTPRT: 0 = write protected
            case 0x4: return !motorRunning(now);   // MOTORON: 0 = motor running
            case 0x5: return track != 0;           // TK0: 0 = head at track 0
            case 0x6: return !diskSwitched_;       // SWITCHED: 0 = disk changed
            case 0x7: return tach(now);            // TACH: 60 pulses per rev
            case 0x8: return true;                 // RDDATA lower head (P3)
            case 0x9: return true;                 // RDDATA upper head (P3)
            case 0xA: return !superDrive;          // 0 = drive handles HD media
            case 0xB: return !hdMedia;             // 0 = HD disk in the drive
            case 0xC: return doubleSided;          // SIDES: 1 = double sided
            // READY: high once the spindle is at speed. The driver polls this
            // right after turning the motor on and waits for it to go high,
            // giving up after a thousand tries ($435366); answering low while a
            // disk is in the drive costs that whole timeout on every spin-up.
            case 0xD: return motorRunning(now);
            case 0xE: return false;                // INSTALLED: 0 = drive present
            case 0xF: return false;                // DRVIN: 0 = drive connected
            default:  return true;
        }
    }

    // ---- control registers ---------------------------------------------
    // Latched by an LSTRB pulse. `reg` is CA1:CA0:SEL, `data` is the CA2 level.
    // Inside Macintosh III-35 lists four: DIRTN, STEP, MOTORON and EJECT.
    void command(int reg, bool data, u64 now) {
        if (!installed) return;
        switch (reg & 7) {
            case 0x0:   // DIRTN: 0 steps toward track 79, 1 toward track 0
                stepIn = !data;
                break;
            case 0x1:
                // UNSETTLED. Inside Macintosh III-35 lists only the four SEL=0
                // registers below; this is a SEL=1 extended register that the
                // 1985 table predates. The Classic ROM's .Sony driver writes it
                // (and nothing else) to the internal drive before every attempt
                // to read, ~36 times per boot, and never writes MOTORON at all.
                // Treating it as a motor enable is what makes the surface start
                // delivering disk bytes, so it is at least motor-related -- but
                // it has not been confirmed, and the driver still does not
                // complete a sector read, so do not treat this as settled.
                if (!motorWanted_) motorUpAt_ = now + kSpinUpCycles;
                motorWanted_ = true;
                break;
            case 0x2:   // STEP: writing 0 steps one track; done after ~12 ms
                if (!data) {
                    const int next = track + (stepIn ? 1 : -1);
                    track = next < 0 ? 0 : (next > 79 ? 79 : next);
                    stepDoneAt_ = now + kStepCycles;
                }
                break;
            case 0x4:   // MOTORON: 0 turns the motor on, 1 turns it off
                if (!data) {
                    if (!motorWanted_) motorUpAt_ = now + kSpinUpCycles;
                    motorWanted_ = true;
                } else {
                    motorWanted_ = false;
                }
                break;
            case 0x6:   // EJECT: writing 1 ejects, but LSTRB must be held ~750 ms
                ejectAsserted_ = data;
                if (data) ejectSince_ = now;
                break;
            default:
                break;
        }
    }

    // Called while LSTRB stays asserted, so a held eject strobe can mature. The
    // Macintosh Plus is known to pulse eject briefly during boot without ejecting
    // anything, which is exactly what the hold time protects against.
    void tickStrobe(u64 now) {
        if (ejectAsserted_ && hasDisk() && now - ejectSince_ >= kEjectHoldCycles) {
            ejectAsserted_ = false;
            ejectRequested_ = true;
        }
    }

    // Reading the disk-switched line is what clears it.
    void clearSwitched() { diskSwitched_ = false; }

    // ---- the rotating surface -------------------------------------------
    //
    // The track under the head is held as a nibble stream and rotates past at a
    // fixed bit-cell rate: at 7.8336 MHz with 2 us cells a disk byte arrives
    // every 128 CPU cycles, and the zone's rotational speed decides how many
    // bytes fit on the track rather than how fast they arrive.

    static constexpr u64 kCyclesPerByte = 128;

    // Install the nibble stream for the current head position. Called by the
    // machine whenever the track, side or medium changes.
    void setTrackData(std::vector<u8> nibbles) {
        trackData_ = std::move(nibbles);
        if (bytePos_ >= trackData_.size()) bytePos_ = 0;
    }
    const std::vector<u8>& trackData() const { return trackData_; }
    bool trackLoaded() const { return !trackData_.empty(); }
    void invalidateTrack() { trackData_.clear(); bytePos_ = 0; }

    // Advance the surface to `now` and return the byte under the head, or 0 if
    // the motor is stopped or the track is blank. Each byte is handed out once.
    //
    // `steps` counts byte times since the byte we last handed over, so `steps-1`
    // bytes rotated past unread and the steps'th is the one now under the head.
    // Skipping `steps` instead would drop a byte every time the caller keeps up
    // (steps == 1), which is the normal case: the ROM's read loop polls roughly
    // every 40 cycles against a 128-cycle byte time. The driver would then see
    // every other disk byte -- enough to look like data is flowing while no
    // address mark can ever match.
    u8 readNibble(u64 now) {
        if (!motorRunning(now) || trackData_.empty()) { lastByteAt_ = now; return 0; }
        if (now < lastByteAt_ + kCyclesPerByte) return 0;
        const std::size_t n = trackData_.size();
        const u64 steps = (now - lastByteAt_) / kCyclesPerByte;
        bytePos_ = (bytePos_ + static_cast<std::size_t>((steps - 1) % n)) % n;
        lastByteAt_ += steps * kCyclesPerByte;
        const u8 b = trackData_[bytePos_];
        bytePos_ = (bytePos_ + 1) % n;
        return b;
    }

    std::size_t bytePos() const { return bytePos_; }

private:
    // 7.8336 MHz CPU/FCLK. Step ~12 ms (Inside Macintosh III-36 "about 12 msec"),
    // motor up to speed within 400 ms, eject needs LSTRB held ~750 ms.
    static constexpr u64 kCpuHz          = 7833600ull;
    static constexpr u64 kStepCycles     = kCpuHz * 12 / 1000;
    static constexpr u64 kSpinUpCycles   = kCpuHz * 400 / 1000;
    static constexpr u64 kEjectHoldCycles = kCpuHz * 750 / 1000;

    // TACH produces 60 pulses per revolution. GCR media spins at a speed set by
    // which of the five zones the head is over; HD/MFM media spins at a constant
    // 300 rpm. Expressed as a square wave over the current cycle count.
    bool tach(u64 now) const {
        const u64 rpm = hdMedia ? 300u : zoneRpm(track);
        const u64 pulsesPerSec = rpm * 60u / 60u;   // 60 pulses/rev * rev/sec
        if (pulsesPerSec == 0) return true;
        const u64 halfPeriod = kCpuHz / (pulsesPerSec * 2u);
        return halfPeriod == 0 ? true : ((now / halfPeriod) & 1u) != 0;
    }

public:
    // Zoned constant-linear-velocity: the outer tracks hold more sectors and turn
    // slower. 16 tracks per zone, 12/11/10/9/8 sectors, 1600 sectors on an 800K
    // double-sided disk.
    static u64 zoneRpm(int trk) {
        static const u64 kRpm[5] = {402, 438, 482, 536, 603};
        int z = trk / 16;
        if (z < 0) z = 0;
        if (z > 4) z = 4;
        return kRpm[z];
    }
    static int zoneSectors(int trk) {
        static const int kSec[5] = {12, 11, 10, 9, 8};
        int z = trk / 16;
        if (z < 0) z = 0;
        if (z > 4) z = 4;
        return kSec[z];
    }

private:
    bool motorWanted_   = false;
    bool diskSwitched_  = false;
    bool ejectAsserted_ = false;
    bool ejectRequested_ = false;
    u64  stepDoneAt_ = 0;
    u64  motorUpAt_  = 0;
    u64  ejectSince_ = 0;

    std::vector<u8> trackData_;   // nibble stream of the track under the head
    std::size_t bytePos_ = 0;
    u64 lastByteAt_ = 0;
};

} // namespace openmac
