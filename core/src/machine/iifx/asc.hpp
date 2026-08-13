#pragma once

// First-generation Apple Sound Chip (ASC) used by the Macintosh IIfx.
//
// This is deliberately separate from the later EASC/Batman device used by
// Quadras.  The original part has 2 KiB of internal RAM which is either two
// 1 KiB sample FIFOs or four 512-byte wavetable banks.  In wavetable mode each
// voice owns a 24-bit 9.15 phase accumulator.  Apple documents 22.2545 kHz and
// 44.1 kHz clocks; original-hardware ASCTester results establish version $00
// and the clear-on-read FIFO interrupt register.
//
// References used for the observable behaviour:
//   Guide to the Macintosh Family Hardware, 2nd ed., pp. 436-439
//   Apple F19 Theory of Operation, Sound Interface
//   MAME asc_device and its original-hardware ASCTester results
//   Snow and QEMU ASC implementations (independent cross-checks)
//
// No firmware, sampled chime, or other Apple data is embedded here.  The ROM
// fills the wavetable RAM and programs the voices itself.

#include "openmac/types.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <functional>

namespace openmac {

class IifxStateCodec;

class Asc {
public:
    struct StereoSample {
        u8 left = 0x80;
        u8 right = 0x80;
    };

    static constexpr u32 kRamBytes = 0x800;
    static constexpr u32 kFifoBytes = 0x400;
    static constexpr u32 kTableBytes = 0x200;
    static constexpr u32 kClassicRate = 22254;
    static constexpr u32 kCdRate = 44100;

    Asc() { reset(); }

    void reset() {
        std::fill(std::begin(ram_), std::end(ram_), u8{0});
        std::fill(std::begin(regs_), std::end(regs_), u8{0});
        std::fill(std::begin(phase_), std::end(phase_), u32{0});
        std::fill(std::begin(increment_), std::end(increment_), u32{0});
        std::fill(std::begin(fifoRead_), std::end(fifoRead_), u16{0});
        std::fill(std::begin(fifoWrite_), std::end(fifoWrite_), u16{0});
        std::fill(std::begin(fifoLevel_), std::end(fifoLevel_), u16{0});
        lastFifo_[0] = lastFifo_[1] = 0x80;
        lastOutput_[0] = lastOutput_[1] = 0x80;
        linePending_ = false;

        ramWrites_ = 0;
        fifoWrites_[0] = fifoWrites_[1] = 0;
        irqTransitions_ = 0;
        producedSamples_ = 0;
        nonSilentSamples_ = 0;
        mode2Selections_ = 0;
        outputMin_ = outputMax_ = 0x80;
        historyHead_ = 0;
        historyCount_ = 0;
        historySequence_ = 0;
        std::fill(std::begin(historyOffset_), std::end(historyOffset_), u16{0});
        std::fill(std::begin(historyValue_), std::end(historyValue_), u8{0});
        std::fill(std::begin(historyOrder_), std::end(historyOrder_), u32{0});
        diagBudget_ = 48;
    }

    // CPU-visible window at $50010000-$50011fff.
    u8 read(u32 offset) {
        offset &= 0x1FFFu;
        if (offset < kRamBytes) return ram_[offset];
        if (offset >= 0x1000u) return 0xFF;

        if (offset >= 0x810u && offset <= 0x82Fu)
            return readVoiceByte(offset);

        switch (offset) {
        case 0x800: return 0x00; // original ASC version
        case 0x804: {
            const u8 result = regs_[kFifoStatus];
            regs_[kFifoStatus] = 0;
            setIrq(false);
            return result;
        }
        // These EASC interrupt-control locations do not exist on the original
        // part.  ASCTester observes zero and writes do not stick.
        case 0xF09:
        case 0xF29:
            return 0;
        default:
            return regs_[offset - 0x800u];
        }
    }

    void write(u32 offset, u8 value) {
        offset &= 0x1FFFu;
        if (offset < kRamBytes) {
            writeRam(offset, value);
            return;
        }
        if (offset >= 0x1000u) return;

        rememberWrite(static_cast<u16>(offset), value);

        if (offset == 0xE00u) {
            // Digital test hook present on the original device.  It is useful
            // to hardware tests and harmless to normal Macintosh software.
            raiseStatus(0x0F);
            return;
        }
        if (offset == 0xF09u || offset == 0xF29u) return;
        if (offset >= 0x810u && offset <= 0x82Fu) {
            writeVoiceByte(offset, value);
            return;
        }

        const u32 reg = offset - 0x800u;
        switch (offset) {
        case 0x800: // version is read-only
            return;
        case 0x801: {
            const u8 next = static_cast<u8>(value & 3u);
            if (next != regs_[kMode]) {
                clearFifos(false);
                if (next == 2) ++mode2Selections_;
            }
            regs_[kMode] = next;
            diag("ASC mode<-%02X (RAM writes=%u)", next,
                 static_cast<u32>(std::min<u64>(ramWrites_, 0xFFFFFFFFu)));
            return;
        }
        case 0x802:
            regs_[kControl] = value;
            return;
        case 0x803:
            regs_[kFifoMode] = value;
            if ((value & 0x80u) != 0) clearFifos(true);
            return;
        case 0x804:
            // Retain writable readback used by low-level test programs.  A
            // subsequent read still clears both the byte and interrupt line.
            regs_[kFifoStatus] = static_cast<u8>(value & 0x0Fu);
            setIrq(regs_[kFifoStatus] != 0);
            return;
        case 0x805:
            // Bits 0-3 select one-shot tables.  The IIfx ROM chime uses free-
            // running voices, but preserving the control byte is essential for
            // correct readback/checkpointing and future one-shot refinement.
            regs_[kWavetableControl] = static_cast<u8>(value & 0x0Fu);
            return;
        case 0x806:
            regs_[kVolume] = value;
            diag("ASC volume<-%02X (output level=%u)", value, volumeLevel());
            return;
        case 0x807:
            regs_[kClock] = static_cast<u8>(value & 3u);
            return;
        default:
            regs_[reg] = value;
            return;
        }
    }

    StereoSample pullStereoSample() {
        StereoSample sample;
        switch (mode()) {
        case 1:
            sample = fifoSample();
            break;
        case 2:
            sample = wavetableSample();
            break;
        default:
            // Mode zero disables the sound output.  Keeping digital silence
            // here avoids turning the held PWM duty cycle into host-side DC.
            sample = {};
            break;
        }

        sample.left = applyVolume(sample.left);
        sample.right = applyVolume(sample.right);
        lastOutput_[0] = sample.left;
        lastOutput_[1] = sample.right;
        ++producedSamples_;
        if (sample.left != 0x80 || sample.right != 0x80)
            ++nonSilentSamples_;
        outputMin_ = std::min(outputMin_, sample.left);
        outputMax_ = std::max(outputMax_, sample.left);
        return sample;
    }

    // The IIfx's internal speaker is wired to the left ASC/Sony path.  Classic
    // Mac OS sends mono sounds to that channel when no stereo jack is present.
    u8 pullSample() { return pullStereoSample().left; }

    u32 sampleRate() const {
        switch (clock()) {
        case 2: return 22050;
        case 3: return kCdRate;
        // Selector 1 is undefined on the original part.  Retaining the classic
        // rate is deterministic and matches the reset/ROM-used selector zero.
        default: return kClassicRate;
        }
    }

    u8 mode() const { return static_cast<u8>(regs_[kMode] & 3u); }
    u8 control() const { return regs_[kControl]; }
    u8 fifoMode() const { return regs_[kFifoMode]; }
    u8 fifoStatus() const { return regs_[kFifoStatus]; }
    u8 wavetableControl() const { return regs_[kWavetableControl]; }
    u8 volume() const { return regs_[kVolume]; }
    u8 volumeLevel() const {
        // The externally audible eight-step level is bits 7..5, driven to the
        // two Sony output stages.  In particular, the IIfx ROM writes $40 for
        // its startup chime: bits 4..2 are zero there but the sound is not
        // muted.  QEMU independently models the same high-bit output level.
        return static_cast<u8>((volume() >> 5) & 7u);
    }
    u8 internalVolumeLevel() const {
        return static_cast<u8>((volume() >> 2) & 7u);
    }
    u8 clock() const { return static_cast<u8>(regs_[kClock] & 3u); }
    bool stereo() const { return (control() & 0x02u) != 0; }
    bool irqAsserted() const { return linePending_; }
    u16 fifoLevel(int channel) const {
        return channel >= 0 && channel < 2 ? fifoLevel_[channel] : 0;
    }
    u32 phase(int voice) const {
        return voice >= 0 && voice < 4 ? phase_[voice] : 0;
    }
    u32 increment(int voice) const {
        return voice >= 0 && voice < 4 ? increment_[voice] : 0;
    }
    u64 ramWrites() const { return ramWrites_; }
    u64 fifoWrites(int channel) const {
        return channel >= 0 && channel < 2 ? fifoWrites_[channel] : 0;
    }
    u64 irqTransitions() const { return irqTransitions_; }
    u64 producedSamples() const { return producedSamples_; }
    u64 nonSilentSamples() const { return nonSilentSamples_; }
    u64 mode2Selections() const { return mode2Selections_; }
    u8 outputMin() const { return outputMin_; }
    u8 outputMax() const { return outputMax_; }

    std::function<void(bool level)> onIrq;
    std::function<void(const char* message)> onDiag;

    void debugState(char* out, std::size_t cap) const {
        if (!out || cap == 0) return;
        int written = std::snprintf(
            out, cap,
            "mode=%02X control=%02X stereo=%d volume=%02X sony/internal=%u/%u "
            "clock=%02X/%u "
            "fifo=%u/%u status=%02X irq=%d irqTransitions=%llu "
            "ramWrites=%llu fifoWrites=%llu/%llu mode2=%llu "
            "samples=%llu nonSilent=%llu min/max=%02X/%02X "
            "phase=%06X/%06X/%06X/%06X inc=%06X/%06X/%06X/%06X history=",
            mode(), control(), stereo() ? 1 : 0, volume(), volumeLevel(),
            internalVolumeLevel(), clock(), sampleRate(),
            static_cast<unsigned>(fifoLevel_[0]),
            static_cast<unsigned>(fifoLevel_[1]), fifoStatus(),
            linePending_ ? 1 : 0,
            static_cast<unsigned long long>(irqTransitions_),
            static_cast<unsigned long long>(ramWrites_),
            static_cast<unsigned long long>(fifoWrites_[0]),
            static_cast<unsigned long long>(fifoWrites_[1]),
            static_cast<unsigned long long>(mode2Selections_),
            static_cast<unsigned long long>(producedSamples_),
            static_cast<unsigned long long>(nonSilentSamples_), outputMin_,
            outputMax_, phase_[0], phase_[1], phase_[2], phase_[3],
            increment_[0], increment_[1], increment_[2], increment_[3]);
        if (written < 0) {
            out[0] = '\0';
            return;
        }
        std::size_t used = std::min<std::size_t>(static_cast<std::size_t>(written),
                                                 cap - 1u);
        const u8 shown = std::min<u8>(historyCount_, 8u);
        for (u8 back = shown; back > 0 && used + 1u < cap; --back) {
            const u8 index = static_cast<u8>(
                (historyHead_ + kHistory - back) % kHistory);
            const int count = std::snprintf(
                out + used, cap - used, "%s%u:%03X=%02X",
                used != 0 && out[used - 1] != '=' ? "," : "",
                historyOrder_[index], historyOffset_[index],
                historyValue_[index]);
            if (count <= 0) break;
            used += std::min<std::size_t>(static_cast<std::size_t>(count),
                                          cap - used - 1u);
        }
    }

private:
    friend class IifxStateCodec;

    enum Register : u32 {
        kMode = 0x001,
        kControl = 0x002,
        kFifoMode = 0x003,
        kFifoStatus = 0x004,
        kWavetableControl = 0x005,
        kVolume = 0x006,
        kClock = 0x007,
    };

    static constexpr u16 kFifoFull = 0x03FF;
    static constexpr u8 kHistory = 32;

    void writeRam(u32 offset, u8 value) {
        if (mode() != 1) {
            ram_[offset] = value;
            ++ramWrites_;
            if (ramWrites_ == 1)
                diag("ASC first wavetable RAM write @%03X=%02X", offset, value);
            return;
        }

        const int channel = offset < kFifoBytes ? 0 : 1;
        if (channel == 1 && !stereo()) return;
        ++fifoWrites_[channel];
        if (fifoLevel_[channel] >= kFifoFull) {
            raiseStatus(static_cast<u8>(0x02u << (channel * 2)));
            return;
        }

        const u32 base = static_cast<u32>(channel) * kFifoBytes;
        ram_[base + fifoWrite_[channel]] = value;
        fifoWrite_[channel] = static_cast<u16>(
            (fifoWrite_[channel] + 1u) & (kFifoBytes - 1u));
        ++fifoLevel_[channel];

        const u8 half = static_cast<u8>(0x01u << (channel * 2));
        const u8 full = static_cast<u8>(0x02u << (channel * 2));
        regs_[kFifoStatus] &= static_cast<u8>(~full);
        if (fifoLevel_[channel] >= kFifoBytes / 2u)
            regs_[kFifoStatus] &= static_cast<u8>(~half);
        if (fifoLevel_[channel] == kFifoFull) raiseStatus(full);
    }

    u8 popFifo(int channel) {
        const u16 before = fifoLevel_[channel];
        if (before != 0) {
            const u32 base = static_cast<u32>(channel) * kFifoBytes;
            lastFifo_[channel] = ram_[base + fifoRead_[channel]];
            fifoRead_[channel] = static_cast<u16>(
                (fifoRead_[channel] + 1u) & (kFifoBytes - 1u));
            --fifoLevel_[channel];
        }

        const u8 half = static_cast<u8>(0x01u << (channel * 2));
        const u8 empty = static_cast<u8>(0x02u << (channel * 2));
        if (before >= kFifoBytes / 2u &&
            fifoLevel_[channel] < kFifoBytes / 2u) {
            regs_[kFifoStatus] &= static_cast<u8>(~empty);
            raiseStatus(half);
        }
        if (before == 1) raiseStatus(empty);
        return lastFifo_[channel];
    }

    StereoSample fifoSample() {
        StereoSample result;
        result.left = popFifo(0);
        result.right = stereo() ? popFifo(1) : result.left;
        return result;
    }

    StereoSample wavetableSample() {
        unsigned voices[4]{};
        for (int voice = 0; voice < 4; ++voice) {
            phase_[voice] = (phase_[voice] + increment_[voice]) & 0x00FFFFFFu;
            const u32 index = (phase_[voice] >> 15) & 0x01FFu;
            voices[voice] =
                ram_[static_cast<u32>(voice) * kTableBytes + index];
        }

        StereoSample result;
        if (stereo()) {
            // The two tables in FIFO/RAM A feed the left ASC path and the two
            // tables in FIFO/RAM B feed the right path.  The ASC adds the raw
            // table values; it does not average offset-binary samples.  The
            // ROM therefore constructs two-voice tables with enough headroom.
            result.left = rawByte(voices[0] + voices[1]);
            result.right = rawByte(voices[2] + voices[3]);
        } else {
            // Apple's boot synthesizer deliberately builds 6-bit tables (its
            // initial values are $01, $10, $30, and $3F), so four voices sum
            // directly into one 8-bit PWM duty value centered near $80.
            const unsigned mono = voices[0] + voices[1] +
                                  voices[2] + voices[3];
            result.left = result.right = rawByte(mono);
        }
        return result;
    }

    u8 applyVolume(u8 sample) const {
        const int level = volumeLevel();
        if (level == 0) return 0x80;
        const int centered = static_cast<int>(sample) - 0x80;
        return centeredByte((centered * level) / 7);
    }

    static u8 centeredByte(int centered) {
        return static_cast<u8>(std::clamp(0x80 + centered, 0, 255));
    }

    static u8 rawByte(unsigned value) {
        return static_cast<u8>(std::min(value, 255u));
    }

    void clearFifos(bool markEmpty) {
        fifoRead_[0] = fifoRead_[1] = 0;
        fifoWrite_[0] = fifoWrite_[1] = 0;
        fifoLevel_[0] = fifoLevel_[1] = 0;
        lastFifo_[0] = lastFifo_[1] = 0x80;
        regs_[kFifoStatus] = markEmpty ? 0x0A : 0;
        setIrq(false);
    }

    void raiseStatus(u8 bits) {
        regs_[kFifoStatus] |= static_cast<u8>(bits & 0x0Fu);
        if ((bits & 0x0Fu) != 0) setIrq(true);
    }

    void setIrq(bool level) {
        if (level == linePending_) return;
        linePending_ = level;
        ++irqTransitions_;
        if (onIrq) onIrq(level);
    }

    u8 readVoiceByte(u32 offset) const {
        const u32 relative = offset - 0x810u;
        const int voice = static_cast<int>(relative >> 3);
        const u32 lane = relative & 7u;
        const u32 value = lane < 4u ? phase_[voice] : increment_[voice];
        const u32 byte = lane & 3u;
        if (byte == 0) return 0;
        return static_cast<u8>(value >> ((3u - byte) * 8u));
    }

    void writeVoiceByte(u32 offset, u8 value) {
        const u32 relative = offset - 0x810u;
        const int voice = static_cast<int>(relative >> 3);
        const u32 lane = relative & 7u;
        const u32 byte = lane & 3u;
        if (byte == 0) return; // only the low 24 bits are implemented
        u32& target = lane < 4u ? phase_[voice] : increment_[voice];
        const u32 shift = (3u - byte) * 8u;
        target = (target & ~(0xFFu << shift)) |
                 (static_cast<u32>(value) << shift);
        target &= 0x00FFFFFFu;
    }

    void rememberWrite(u16 offset, u8 value) {
        historyOffset_[historyHead_] = offset;
        historyValue_[historyHead_] = value;
        historyOrder_[historyHead_] = ++historySequence_;
        historyHead_ = static_cast<u8>((historyHead_ + 1u) % kHistory);
        if (historyCount_ < kHistory) ++historyCount_;
    }

    void diag(const char* format, u32 first, u32 second) {
        if (!onDiag || diagBudget_ <= 0) return;
        --diagBudget_;
        char message[112];
        std::snprintf(message, sizeof message, format, first, second);
        onDiag(message);
    }

    u8 ram_[kRamBytes]{};
    u8 regs_[0x800]{};
    u32 phase_[4]{};
    u32 increment_[4]{};
    u16 fifoRead_[2]{};
    u16 fifoWrite_[2]{};
    u16 fifoLevel_[2]{};
    u8 lastFifo_[2]{0x80, 0x80};
    u8 lastOutput_[2]{0x80, 0x80};
    bool linePending_ = false;

    u64 ramWrites_ = 0;
    u64 fifoWrites_[2]{};
    u64 irqTransitions_ = 0;
    u64 producedSamples_ = 0;
    u64 nonSilentSamples_ = 0;
    u64 mode2Selections_ = 0;
    u8 outputMin_ = 0x80;
    u8 outputMax_ = 0x80;

    u16 historyOffset_[kHistory]{};
    u8 historyValue_[kHistory]{};
    u32 historyOrder_[kHistory]{};
    u8 historyHead_ = 0;
    u8 historyCount_ = 0;
    u32 historySequence_ = 0;
    int diagBudget_ = 48;
};

} // namespace openmac
