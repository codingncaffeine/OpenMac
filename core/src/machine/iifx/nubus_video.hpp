#pragma once

// A Macintosh II-family video card in NuBus slot $9.
//
// This is deliberately a slot device, not a shortcut bolted onto QuickDraw. The
// IIfx ROM finds the card by walking its declaration ROM, validates the Apple
// format block and CRC, loads the 68030 driver from the sDriver record, and uses
// the mode records to obtain the framebuffer geometry. With no external
// firmware the small, documented-format fallback ROM implements a 512 KB
// Macintosh II video card. Supplying an Apple 8*24 GC declaration ROM instead
// selects the real card layout: lane-0 ROM in standard slot space and the
// control blocks plus two megabytes of VRAM in slot $9 super-space. The Apple
// ROM bytes are never baked into OpenMac.
//
// References: Apple, Designing Cards and Drivers for the Macintosh Family,
// 2nd ed. (1990), chapters 8, 9, and 11; Guide to the Macintosh Family
// Hardware, 2nd ed. (1990), chapter 12; Macintosh Display Card 8*24 GC Design
// Documentation (1991). The fallback ROM below is constructed solely from
// those published formats and interfaces and contains no Apple firmware.

#include "openmac/types.hpp"
#include "am29000.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace openmac {

class IifxStateCodec;

class IifxNuBusVideo {
public:
    static constexpr u8 kSlot = 9;
    static constexpr u32 kSlotBase = 0xF9000000u;
    static constexpr u32 kSuperSlotBase = 0x90000000u;
    static constexpr u32 kFrameBufferOffset = 0x00001000u;
    // Keep control registers above the 512 KiB framebuffer
    // ($001000-$080FFF), but inside the one-megabyte 24-bit slot aperture.
    static constexpr u32 kFallbackControlOffset = 0x00090100u;
    static constexpr u32 kVramBytes = 512u * 1024u;
    static constexpr u32 kGcVramBytes = 2u * 1024u * 1024u;
    static constexpr u32 kGcVramOffset = 0x0C000000u;
    static constexpr u32 kGcDramBytes = 2u * 1024u * 1024u;
    static constexpr u32 kGcExpansionDramBytes = 8u * 1024u * 1024u;
    static constexpr u32 kGcExpansionDramOffset = 0x0D000000u;
    static constexpr u32 kGcSharedDramBytes = 64u * 1024u;
    static constexpr u32 kGcSramBytes = 64u * 1024u;
    static constexpr u32 kGcVectorBytes = 256u * sizeof(u32);
    // Dolphin has three physically distinct memories: 64 KiB instruction
    // SRAM, two megabytes of GCOS data DRAM, and two megabytes of display
    // VRAM. The host uploads GCOS data through super-slot +$02000000; MFB
    // presents that DRAM to the Am29000 at physical $4C000000.
    static constexpr u32 kGcHostDramOffset = 0x02000000u;
    // MFB presents the board's two-megabyte frame buffer directly on the
    // Am29000 data bus at $41000000.  GCOS seeds this address in its global
    // register block and the accelerated QuickDraw rasterizers use it rather
    // than taking the slower NuBus-facing $9C aperture back through RDNC.
    static constexpr u32 kGcProcessorVramBase = 0x41000000u;
    static constexpr u32 kGcProcessorDramBase = 0x4C000000u;
    static constexpr u32 kGcProcessorExpansionDramBase = 0x4D000000u;
    // AC842 exposes its byte-wide RAMDAC directly to the Am29000 here.  The
    // same registers appear to the Macintosh in super-slot space at +$06C00000.
    static constexpr u32 kGcProcessorRamdacBase = 0x46C00000u;
    static constexpr u32 kGcProcessorSramBase = 0x02000000u;
    static constexpr u32 kGcProcessorDataSramBase = 0x42000000u;
    // Dolphin's host loader places the card's cold-start routine at SRAM
    // offset $3000.  MFB presents that line as the Am29000's instruction-ROM
    // reset entry, then the routine installs the normal vector table and
    // transfers into GCOS.
    static constexpr u32 kGcResetEntry = kGcProcessorSramBase + 0x3000u;
    static constexpr u32 kRomOffset = 0x00FF0000u;
    static constexpr u32 kRomBytes = 64u * 1024u;
    static constexpr int kWidth = 640;
    static constexpr int kHeight = 480;
    // 640x480 high-resolution monitor timing used by Apple's JMFB setup:
    // 20 MHz / 21 * 127 / 4 pixel clock, 864 x 525 total raster. Expressing
    // the phase in 1/127 CPU-cycle units preserves its fractional frame
    // period exactly on the IIfx's 40 MHz clock.
    static constexpr u64 kGcTimingUnitsPerCpuCycle = 127u;
    static constexpr u64 kGcTimingFrameUnits =
        2u * 84u * 864u * 525u;
    static constexpr u64 kGcTimingActiveUnits =
        kGcTimingFrameUnits * 480u / 525u;

    explicit IifxNuBusVideo(std::vector<u8> declarationRom = {})
        : genuineGc_(!declarationRom.empty()),
          vram_(genuineGc_ ? kGcVramBytes : kVramBytes, 0xAA),
          gcDram_(genuineGc_ ? kGcDramBytes : 0u, 0),
          gcExpansionDram_(genuineGc_ ? kGcExpansionDramBytes : 0u, 0),
          rom_(genuineGc_ ? std::move(declarationRom) : buildDeclarationRom()) {
        if (genuineGc_) validateGcDeclarationRom();
        gcCpu_.readInstruction = [this](u32 address, bool translated) {
            return gcReadInstruction(address, translated);
        };
        gcCpu_.readVector = [this](u32 address) {
            return gcReadVector(address);
        };
        gcCpu_.readData = [this](u32 address, bool inputOutput) {
            return gcReadData(address, inputOutput);
        };
        gcCpu_.peekData = [this](u32 address, bool inputOutput) {
            return gcPeekData(address, inputOutput);
        };
        gcCpu_.writeData = [this](u32 address, u32 value, bool inputOutput) {
            gcWriteData(address, value, inputOutput);
        };
        reset();
    }

    void setGcBusMasterCallbacks(std::function<u32(u32)> read32,
                                 std::function<void(u32, u32)> write32) {
        gcBusMasterRead32_ = std::move(read32);
        gcBusMasterWrite32_ = std::move(write32);
    }

    void reset() {
        mode_ = 0x80;
        page_ = 0;
        control_ = 0x0042;             // display transfer on, RGB packing off
        preload_ = 248;
        base_ = 0;
        // The genuine GC pads its 640-pixel scanlines to a power-of-two MFB
        // pitch (128 bytes at 1 bpp). Its actual stride register has not been
        // identified yet, so zero selects the card-ROM geometry below.
        stride_ = genuineGc_ ? 0u : 20u;
        ramdacMode_ = 0;
        ramdacControl_ = 0x80;
        ramdacConvolution_ = false;
        mfbBaseShift_ = 0;
        mfbBaseBits_ = 0;
        mfbStrideShift_ = 0;
        mfbStrideBits_ = 0;
        clutAddress_ = clutComponent_ = 0;
        serialShift_ = 0;
        serialBits_ = 0;
        serialCommands_ = 0;
        lastSerialCommand_ = 0;
        timingEchoWrites_ = 0;
        timingEchoPending_ = false;
        timingEchoValue_ = 0;
        timingEchoLane_ = 0;
        timingPhase_ = 0;
        gcCpuPhase_ = 0;
        gcResetControl_ = 0;
        gcInterruptControl_ = 0;
        gcMacInterruptControl_ = 0;
        gcCacheControl_ = 0;
        gcCacheFillAddress_ = 0;
        gcCacheFillWordsRemaining_ = 0;
        gcCacheFillActive_ = false;
        gcHostRequest_ = false;
        gcMacRequest_ = false;
        gcCpuReleased_ = false;
        gcCpu_.reset(genuineGc_ ? kGcResetEntry : 0u);
        gcInstructionReads_ = gcDataReads_ = gcDataWrites_ = 0;
        gcProcessorVramWrites_ = 0;
        gcUnknownDataReads_ = gcUnknownDataWrites_ = 0;
        gcFirstUnknownDataAddress_ = 0;
        gcUnknownTraceCount = 0;
        gcQuickDrawWriteTraceCount = 0;
        gcFramebufferReadTraceCount = 0;
        gcWorkBufferReadTraceCount = 0;
        gcReadRegionCounts.fill(0);
        gcReadRegionFirstAddresses.fill(0);
        gcReadRegionFirstPcs.fill(0);
        gcWriteRegionCounts.fill(0);
        gcWriteRegionFirstAddresses.fill(0);
        gcWriteRegionFirstValues.fill(0);
        gcWriteRegionFirstPcs.fill(0);
        gcLocalWriteSiteCount = 0;
        gcLocalWriteSitePcs.fill(0);
        gcLocalWriteSiteRegions.fill(0);
        gcLocalWriteSiteCounts.fill(0);
        gcLocalWriteSiteMinAddresses.fill(0);
        gcLocalWriteSiteMaxAddresses.fill(0);
        gcLocalWriteSiteFirstValues.fill(0);
        gcLocalWriteSiteLastValues.fill(0);
        gcMfbWriteCounts.fill(0);
        gcMfbWriteFirstValues.fill(0);
        gcMfbWriteLastValues.fill(0);
        gcMfbWriteLastPcs.fill(0);
        gcMfbWriteTraceCount = 0;
        gcHostCommandCount = 0;
        gcHostSelectorCounts.fill(0);
        gcCompletionTraceCount = 0;
        gcAbnormalCompletionCaptured = false;
        gcAbnormalCompletionRecentCount = 0;
        gcLowReadTraceCount = 0;
        gcPointerReadTraceCount = 0;
        gcMapTraceCount = 0;
        gcSyncTraceCount = 0;
        gcSyncSnapshotCaptured = false;
        gcSemaphoreMutationCount = 0;
        gcPointerMfbSnapshotCaptured = false;
        gcRasterMfbSnapshotCaptured = false;
        gcLowWriteSnapshotCaptured = false;
        gcHeapOverwriteCaptured = false;
        gcRasterStoreSnapshotCaptured = false;
        gcUnknownSnapshotCaptured = false;
        gcInterruptGate_ = !genuineGc_;
        vblankDisable_ = true;
        vblankEnabled_ = false;
        vblankIrq_ = false;
        reads = romReads = superReads = writes = vramWrites = 0;
        firstGcReadCount = firstGcWriteCount = 0;
        vblankCount = vblankAssertions = vblankAcks = 0;
        frameVramWrites_ = lastFrameVramWrites = 0;
        firstReadCount = 0;
        std::fill(vram_.begin(), vram_.end(), u8{0xAA});
        std::fill(gcDram_.begin(), gcDram_.end(), u8{0});
        std::fill(gcExpansionDram_.begin(), gcExpansionDram_.end(), u8{0});
        gcSram_.fill(0);
        resetGcVectorStore();
        for (u32 index = 0; index < palette_.size(); ++index) {
            const u32 component = 255u - index;
            palette_[index] = 0xFF000000u | (component << 16) |
                              (component << 8) | component;
        }
        jmfbShadow_.fill(0);
        crtcShadow_.fill(0);
        gcMfbShadow_.fill(0);
    }

    u8 readStandard(u32 slotOffset) const {
        ++reads;
        slotOffset &= 0x00FFFFFFu;
        if (firstReadCount < firstReadOffsets.size())
            firstReadOffsets[firstReadCount++] = slotOffset;
        if (genuineGc_) {
            // RDNC exposes the low 64 KiB of GC data DRAM in standard slot
            // space for 24-bit-compatible Macintosh software.  For example,
            // a tagged pointer $F9908DD4 is translated by the Macintosh MMU
            // to slot-9 address $F9008DD4.  This is deliberately distinct
            // from MFB's 64 KiB instruction cache SRAM at super-slot $9000.
            const u32 compatibleOffset = slotOffset & 0x000FFFFFu;
            if (compatibleOffset < kGcSharedDramBytes)
                return gcDram_[compatibleOffset];
            const u32 physicalBytes = static_cast<u32>(rom_.size()) * 4u;
            const u32 physicalBase = 0x01000000u - physicalBytes;
            if (slotOffset >= physicalBase &&
                ((slotOffset - physicalBase) & 3u) == 0) {
                const std::size_t index = (slotOffset - physicalBase) >> 2;
                if (index < rom_.size()) {
                    ++romReads;
                    return rom_[index];
                }
            }
            // Invalid declaration-ROM byte lanes are electrically open. The
            // machine bus represents an unacknowledged byte as $FF while the
            // Slot Manager searches the four possible top addresses.
            return 0xFF;
        }
        if (slotOffset == 0) return static_cast<u8>(mode_ >> 8);
        if (slotOffset == 1) return static_cast<u8>(mode_);
        if (slotOffset == 2) return static_cast<u8>(page_ >> 8);
        if (slotOffset == 3) return static_cast<u8>(page_);
        if (slotOffset >= kFrameBufferOffset &&
            slotOffset < kFrameBufferOffset + vram_.size())
            return vram_[slotOffset - kFrameBufferOffset];
        if (slotOffset >= kFallbackControlOffset &&
            slotOffset < kFallbackControlOffset + 0x100u) {
            const u32 relative = slotOffset - kFallbackControlOffset;
            if ((relative & ~3u) == 0x0C0u)
                return static_cast<u8>((relative & 3u) == 3u ? 0x0E : 0);
            return static_cast<u8>(crtcShadow_[(relative >> 2) & 0x7Fu] >>
                                   ((3u - (relative & 3u)) * 8u));
        }
        if (slotOffset >= kRomOffset) {
            ++romReads;
            return rom_[slotOffset - kRomOffset];
        }
        return 0xFF;
    }

    void writeStandard(u32 slotOffset, u8 value) {
        ++writes;
        slotOffset &= 0x00FFFFFFu;
        if (genuineGc_) {
            const u32 compatibleOffset = slotOffset & 0x000FFFFFu;
            if (compatibleOffset < kGcSharedDramBytes) {
                gcDram_[compatibleOffset] = value;
                if (compatibleOffset >= 0x8DD4u &&
                    compatibleOffset <= 0x8DD7u)
                    noteGcSemaphoreMutation(1);
                if (compatibleOffset == 0x8DD7u)
                    noteGcHostCommand(1);
            }
            return;
        }
        if (slotOffset == 0) {
            mode_ = static_cast<u16>((mode_ & 0x00FFu) | (u16(value) << 8));
            return;
        }
        if (slotOffset == 1) {
            mode_ = static_cast<u16>((mode_ & 0xFF00u) | value);
            if (mode_ < 0x80 || mode_ > 0x83) mode_ = 0x80;
            return;
        }
        if (slotOffset == 2) {
            page_ = static_cast<u16>((page_ & 0x00FFu) | (u16(value) << 8));
            return;
        }
        if (slotOffset == 3) {
            page_ = static_cast<u16>((page_ & 0xFF00u) | value);
            return;
        }
        if (slotOffset >= kFrameBufferOffset &&
            slotOffset < kFrameBufferOffset + vram_.size()) {
            vram_[slotOffset - kFrameBufferOffset] = value;
            ++vramWrites;
            ++frameVramWrites_;
            return;
        }
        if (slotOffset >= kFallbackControlOffset &&
            slotOffset < kFallbackControlOffset + 0x100u) {
            const u32 relative = slotOffset - kFallbackControlOffset;
            const u32 reg = (relative >> 2) & 0x7Fu;
            mergeRegister(crtcShadow_[reg], relative & 3u, value);
            if ((relative & ~3u) == 0x03Cu) {
                vblankDisable_ = (crtcShadow_[reg] & 2u) != 0;
                updateVblankEnable();
            } else if ((relative & ~3u) == 0x048u) {
                acknowledgeVblank();
            }
            return;
        }
    }

    // Compatibility aliases used by older direct card tests. Machine bus
    // decoding calls the explicit standard/super-space entry points.
    u8 read(u32 slotOffset) const { return readStandard(slotOffset); }
    void write(u32 slotOffset, u8 value) { writeStandard(slotOffset, value); }

    u8 readSuper(u32 slotOffset, u64 machineCycles = 0) const {
        ++reads;
        ++superReads;
        slotOffset &= 0x0FFFFFFFu;
        if (!genuineGc_) return 0xFF;
        // The first 64 KiB of Dolphin's nominal $9C display aperture is the
        // shared GCOS control/data window. Screen memory begins at $9C010000;
        // the loader places the Am29000 boot descriptor at $9C006400, which
        // MFB exposes to the accelerator as DRAM address $4C006400.
        if (slotOffset >= kGcVramOffset &&
            slotOffset < kGcVramOffset + kGcSharedDramBytes)
            return gcDram_[slotOffset - kGcVramOffset];
        u32 vramOffset = 0;
        bool padding = false;
        if (translateVramAddress(slotOffset, vramOffset, padding))
            return padding ? 0 : vram_[vramOffset];
        // Dolphin's Am29000 executes from a mandatory 64 KiB static-RAM
        // window at the bottom of super-slot space.  The declaration-ROM
        // primary-init code writes one longword every $100 bytes through the
        // first 128 KiB and verifies that only the first 64 KiB retained it;
        // consequently this must neither be omitted nor mirrored.
        if (slotOffset < kGcSramBytes) return gcSram_[slotOffset];
        // The +$02000000 aperture is the host view of accelerator DRAM. The
        // Apple loader clears $92003800-$9200FFFF and uploads GCOS data here.
        if (slotOffset >= kGcHostDramOffset &&
            slotOffset < kGcHostDramOffset + kGcDramBytes)
            return gcDram_[slotOffset - kGcHostDramOffset];
        // Apple's primary init probes the optional GC DRAM through the $9D
        // super-slot aperture.  A fully populated card has eight distinct
        // megabytes here; the probe deliberately compares locations four
        // megabytes apart to distinguish populated and aliased SIMM banks.
        if (slotOffset >= kGcExpansionDramOffset &&
            slotOffset < kGcExpansionDramOffset + kGcExpansionDramBytes)
            return gcExpansionDram_[slotOffset - kGcExpansionDramOffset];

        // The declaration-ROM primary-init routine samples bit 7 of these
        // three byte-wide inputs with BFEXTU.  An AppleColor High-Resolution
        // RGB display has the direct sense code 6 (binary 110).
        if (slotOffset == 0x04000044u || slotOffset == 0x04000048u)
            return recordGcRead(slotOffset, 0x80u);
        if (slotOffset == 0x0400004Cu)
            return recordGcRead(slotOffset, 0x00u);

        const auto registerByte = [](u32 value, u32 lane) {
            return static_cast<u8>(value >> ((3u - lane) * 8u));
        };
        if (slotOffset >= 0x04000028u && slotOffset < 0x0400002Cu)
            return recordGcRead(slotOffset,
                registerByte(gcResetControl_, slotOffset & 3u));
        if (slotOffset >= 0x04000050u && slotOffset < 0x04000054u)
            return recordGcRead(slotOffset,
                registerByte(gcInterruptControl_, slotOffset & 3u));
        if (slotOffset >= 0x04000054u && slotOffset < 0x04000058u)
            return recordGcRead(slotOffset,
                registerByte(gcMacInterruptControl_, slotOffset & 3u));
        if (slotOffset >= 0x04000000u && slotOffset < 0x04000010u) {
            const u32 reg = (slotOffset - 0x04000000u) >> 2;
            const u32 lane = slotOffset & 3u;
            switch (reg) {
            case 0: return recordGcRead(slotOffset,
                                       registerByte(control_, lane));
            case 1: return recordGcRead(slotOffset,
                                       registerByte(preload_, lane));
            case 2: return recordGcRead(slotOffset,
                                       registerByte(jmfbShadow_[reg], lane));
            case 3: return recordGcRead(slotOffset,
                                       registerByte(stride_, lane));
            default: break;
            }
        }
        if (slotOffset >= 0x04400000u && slotOffset < 0x04400200u) {
            const u32 relative = slotOffset - 0x04400000u;
            if ((relative & ~3u) == 0x1C0u)
                return recordGcRead(slotOffset, registerByte(
                    beamPosition(machineCycles), relative & 3u));
            if ((relative & ~3u) == 0x1CCu)
                return recordGcRead(slotOffset, 0);
            return recordGcRead(slotOffset,
                registerByte(crtcShadow_[(relative >> 2) & 0x7Fu],
                             relative & 3u));
        }
        if (slotOffset >= 0x06C00000u && slotOffset < 0x06C00010u) {
            const u32 relative = slotOffset - 0x06C00000u;
            // The GC's super-slot glue wires the AC842/RAMDAC byte to lane
            // zero.  The declaration ROM consequently writes a significant
            // byte in bits 31..24 of a longword and expects it back there.
            // Treating all four lanes as aliases made the three trailing zero
            // byte cycles erase every valid mode write.
            if ((relative & 3u) != 0)
                return recordGcRead(slotOffset, 0);
            switch (relative >> 2) {
            case 0: return recordGcRead(slotOffset, clutAddress_);
            case 2: return recordGcRead(slotOffset, ramdacControl_);
            default: return recordGcRead(slotOffset, 0);
            }
        }
        if (slotOffset >= 0x04C00000u && slotOffset < 0x04C00004u) {
            const u32 port = timingPort();
            const u32 lane = slotOffset & 3u;
            if (lane == 3u) {
                timingEchoPending_ = true;
                timingEchoValue_ = port;
                timingEchoLane_ = 0;
            }
            return recordGcRead(slotOffset, registerByte(port, lane));
        }
        return recordGcRead(slotOffset, 0);
    }

    void writeSuper(u32 slotOffset, u8 value) {
        ++writes;
        slotOffset &= 0x0FFFFFFFu;
        if (!genuineGc_) return;
        if (slotOffset >= kGcVramOffset &&
            slotOffset < kGcVramOffset + kGcSharedDramBytes) {
            const u32 offset = slotOffset - kGcVramOffset;
            gcDram_[offset] = value;
            if (offset >= 0x8DD4u && offset <= 0x8DD7u)
                noteGcSemaphoreMutation(2);
            if (offset == 0x8DD7u) noteGcHostCommand(2);
            return;
        }
        u32 vramOffset = 0;
        bool padding = false;
        if (translateVramAddress(slotOffset, vramOffset, padding)) {
            // Direct QuickDraw PixMaps contain xRGB longwords.  Dolphin's
            // packed aperture discards X and places RGB contiguously in its
            // physical VRAM, exactly as the display engine consumes it.
            if (!padding) {
                vram_[vramOffset] = value;
                ++vramWrites;
                ++frameVramWrites_;
            }
            return;
        }
        if (slotOffset < kGcSramBytes) {
            gcSram_[slotOffset] = value;
            return;
        }
        if (slotOffset >= kGcHostDramOffset &&
            slotOffset < kGcHostDramOffset + kGcDramBytes) {
            const u32 offset = slotOffset - kGcHostDramOffset;
            gcDram_[offset] = value;
            if (offset >= 0x8DD4u && offset <= 0x8DD7u)
                noteGcSemaphoreMutation(3);
            if (offset == 0x8DD7u) noteGcHostCommand(3);
            return;
        }
        if (slotOffset >= kGcExpansionDramOffset &&
            slotOffset < kGcExpansionDramOffset + kGcExpansionDramBytes) {
            gcExpansionDram_[slotOffset - kGcExpansionDramOffset] = value;
            return;
        }
        if (firstGcWriteCount < firstGcWriteOffsets.size()) {
            firstGcWriteOffsets[firstGcWriteCount] = slotOffset;
            firstGcWriteValues[firstGcWriteCount] = value;
            ++firstGcWriteCount;
        }

        // The declaration-ROM primary init holds the Am29000 in reset with a
        // zero longword at +$28, uploads GC OS, then writes $FFFFFFFF to
        // release it.  +$50 is the companion interrupt/control latch used by
        // the host-side IPC bootstrap.
        if (slotOffset >= 0x04000028u && slotOffset < 0x0400002Cu) {
            const u32 lane = slotOffset & 3u;
            mergeRegister(gcResetControl_, lane, value);
            if (lane == 3u) {
                const bool release = gcResetControl_ != 0;
                if (!release) {
                    gcCpuReleased_ = false;
                    gcCpu_.reset(kGcResetEntry);
                    gcCpuPhase_ = 0;
                } else if (!gcCpuReleased_) {
                    gcCpu_.reset(kGcResetEntry);
                    gcCpuReleased_ = true;
                }
            }
            return;
        }
        if (slotOffset >= 0x04000050u && slotOffset < 0x04000054u) {
            mergeRegister(gcInterruptControl_, slotOffset & 3u, value);
            if ((slotOffset & 3u) == 3u) {
                gcHostRequest_ = gcInterruptControl_ != 0;
                // MFB combines the host doorbell onto INTR2.  Its firmware
                // vector reads +$64 to identify this source before clearing
                // the request through +$50.
                gcMfbShadow_[0x64u >> 2u] =
                    gcHostRequest_ ? 0xFFFFFFFFu : 0u;
                // MFB combines the latched host doorbell onto INTR2.  The
                // Am29000 keeps it pending while DI/DA mask interrupts and
                // presents it once GCOS enables external requests.
                gcCpu_.setInput(2, gcHostRequest_);
            }
            return;
        }
        if (slotOffset >= 0x04000054u && slotOffset < 0x04000058u) {
            mergeRegister(gcMacInterruptControl_, slotOffset & 3u, value);
            if ((slotOffset & 3u) == 3u && gcMacInterruptControl_ == 0)
                gcMacRequest_ = false;
            return;
        }

        // The GC ROM sends twelve bits, most significant first, to the card's
        // serial control port. A 68030 longword write reaches this byte bus as
        // four cycles; only lane zero carries the serial bit.
        if (slotOffset >= 0x04C00000u && slotOffset < 0x04C00004u) {
            const u32 lane = slotOffset & 3u;
            if (timingEchoPending_) {
                // During raster synchronization the ROM repeatedly reads the
                // timing port and writes the returned longword straight back.
                // Those are wait-state cycles, not serial command bits.
                const u8 expected = static_cast<u8>(
                    timingEchoValue_ >> ((3u - lane) * 8u));
                if (lane == timingEchoLane_ && value == expected) {
                    if (lane == 0) ++timingEchoWrites_;
                    if (lane == 3) timingEchoPending_ = false;
                    else ++timingEchoLane_;
                    return;
                }
                timingEchoPending_ = false;
            }
            if (lane == 0) {
                serialShift_ = static_cast<u16>((serialShift_ << 1) |
                                                 ((value >> 7) & 1u));
                if (++serialBits_ == 12) {
                    const u16 command = static_cast<u16>(serialShift_ & 0x0FFFu);
                    lastSerialCommand_ = command;
                    ++serialCommands_;
                    if (command == 1) gcInterruptGate_ = true;
                    else if (command == 3) gcInterruptGate_ = false;
                    serialBits_ = 0;
                    serialShift_ = 0;
                    updateVblankEnable();
                }
            }
            return;
        }

        // Dolphin does not expose the four parallel JMFB registers used by
        // the ordinary Display Card 8*24 at these offsets.  Its 68030 driver
        // shifts the framebuffer base (20 bits) and row stride (12 bits),
        // most-significant bit first, through lane zero.  The authentic
        // 341-0266 ROM emits base $02000 for ScrnBase $9C010000 and stride
        // $010/$020/$028/$050/$140 for 1/2/4/8/24-bit 640x480 modes.
        if ((slotOffset & 3u) == 0u && slotOffset == 0x04000000u) {
            mfbBaseShift_ = ((mfbBaseShift_ << 1) | ((value >> 7) & 1u)) &
                            0x000FFFFFu;
            if (++mfbBaseBits_ == 20u) {
                base_ = mfbBaseShift_;
                mfbBaseShift_ = 0;
                mfbBaseBits_ = 0;
            }
            return;
        }
        if ((slotOffset & 3u) == 0u && slotOffset == 0x040000A0u) {
            mfbStrideShift_ = static_cast<u16>(
                ((mfbStrideShift_ << 1) | ((value >> 7) & 1u)) & 0x0FFFu);
            if (++mfbStrideBits_ == 12u) {
                stride_ = mfbStrideShift_;
                mfbStrideShift_ = 0;
                mfbStrideBits_ = 0;
            }
            return;
        }

        if (slotOffset >= 0x04000000u && slotOffset < 0x04000010u) {
            const u32 reg = (slotOffset - 0x04000000u) >> 2;
            const u32 lane = slotOffset & 3u;
            mergeRegister(jmfbShadow_[reg], lane, value);
            switch (reg) {
            case 0: control_ = static_cast<u16>(jmfbShadow_[reg]); break;
            case 1: preload_ = static_cast<u16>(jmfbShadow_[reg]); break;
            // +$08 is used by the genuine ROM as a control/sense-drive port.
            // It is not the display-start register; treating it as one made
            // the A0/40/80/00 initialization pattern corrupt the framebuffer
            // address on every cold boot.
            case 2: break;
            case 3: stride_ = jmfbShadow_[reg]; break;
            default: break;
            }
            return;
        }
        if (slotOffset >= 0x04400000u && slotOffset < 0x04400200u) {
            const u32 relative = slotOffset - 0x04400000u;
            const u32 reg = (relative >> 2) & 0x7Fu;
            mergeRegister(crtcShadow_[reg], relative & 3u, value);
            if ((relative & ~3u) == 0x03Cu) {
                vblankDisable_ = (crtcShadow_[reg] & 2u) != 0;
                updateVblankEnable();
            } else if ((relative & ~3u) == 0x048u) {
                acknowledgeVblank();
            }
            return;
        }
        if (slotOffset >= 0x06C00000u && slotOffset < 0x06C00010u) {
            const u32 relative = slotOffset - 0x06C00000u;
            if ((relative & 3u) != 0) return;
            const u32 reg = relative >> 2;
            // The byte-wide RAMDAC is connected to lane zero in the GC
            // super-slot interface. Longword accesses therefore carry the
            // register value in bits 31..24; the remaining byte cycles are
            // electrically inactive and must not overwrite it.
            if (reg == 0) {
                writeClutAddress(value);
            } else if (reg == 1) {
                writeClutData(value);
            } else if (reg == 2) {
                writeRamdacControl(value);
            }
        }
    }

    void onVblank() {
        ++vblankCount;
        lastFrameVramWrites = frameVramWrites_;
        frameVramWrites_ = 0;
        if (vblankEnabled_) {
            if (!vblankIrq_) ++vblankAssertions;
            vblankIrq_ = true;
        }
    }

    void tick(u64 cpuCycles) {
        if (!genuineGc_ || cpuCycles == 0) return;
        if (gcCpuReleased_ && !gcCpu_.faulted()) {
            // The IIfx runs at 40 MHz and Dolphin's Am29000 at 30 MHz.
            // Preserve the exact 3:4 ratio without host-time dependence.
            gcCpuPhase_ += cpuCycles * 3u;
            const int slots = static_cast<int>(gcCpuPhase_ / 4u);
            gcCpuPhase_ %= 4u;
            if (slots != 0) gcCpu_.run(slots);
        }
        u64 units = cpuCycles * kGcTimingUnitsPerCpuCycle;
        while (units != 0) {
            const u64 edge = timingPhase_ < kGcTimingActiveUnits
                ? kGcTimingActiveUnits : kGcTimingFrameUnits;
            const u64 distance = edge - timingPhase_;
            const u64 advance = std::min(units, distance);
            timingPhase_ += advance;
            units -= advance;
            if (timingPhase_ == kGcTimingActiveUnits) onVblank();
            if (timingPhase_ == kGcTimingFrameUnits) timingPhase_ = 0;
        }
    }

    bool irqAsserted() const { return vblankIrq_ || gcMacRequest_; }
    bool vblankEnabled() const { return vblankEnabled_; }
    bool genuineGc() const { return genuineGc_; }
    u32 displayBase() const {
        if (!genuineGc_) return 0;
        // Dolphin expresses the host-visible xRGB aperture in eight-byte
        // units.  In direct mode the aperture packs every four host bytes
        // into three physical RGB bytes, so the scanout base uses six-byte
        // units.  Apple's $02000 base consequently maps $010000 -> $00C000.
        const u32 unit = packedRgb() ? 6u : 8u;
        const u32 programmed = base_ * unit;
        // Primary init publishes $9C010000 as ScrnBase. Until the MFB display
        // start register is decoded separately from the +$08 sense/control
        // port, preserve that card-ROM-defined initial page exactly.
        return programmed ? programmed
                          : (packedRgb() ? 0x0000C000u : 0x00010000u);
    }
    u32 displayStride() const {
        const u32 programmed = stride_ * (packedRgb() ? 6u : 8u);
        if (programmed) return programmed;
        // QuickDraw publishes 2560 xRGB bytes, while the packed scanout side
        // stores the same 640 pixels as 1920 contiguous RGB bytes.
        return bitsPerPixel() == 24 ? static_cast<u32>(kWidth * 3)
                                    : 128u * static_cast<u32>(bitsPerPixel());
    }
    u16 control() const { return control_; }
    u32 baseRegister() const { return base_; }
    u32 strideRegister() const { return stride_; }
    u8 ramdacMode() const { return ramdacMode_; }
    u16 lastSerialCommand() const { return lastSerialCommand_; }
    u64 serialCommands() const { return serialCommands_; }
    u64 timingEchoWrites() const { return timingEchoWrites_; }
    bool gcProcessorReleased() const { return gcCpuReleased_; }
    bool gcProcessorFaulted() const { return gcCpu_.faulted(); }
    const std::string& gcProcessorFaultReason() const {
        return gcCpu_.faultReason();
    }
    u64 gcProcessorInstructions() const { return gcCpu_.instructions(); }
    u32 gcProcessorTimerCounter() const { return gcCpu_.timerCounter(); }
    u32 gcProcessorTimerReload() const { return gcCpu_.timerReload(); }
    void setGcProcessorPcWatch(u32 pc) { gcCpu_.setDiagnosticPcWatch(pc); }
    u64 gcProcessorPcWatchHits() const {
        return gcCpu_.diagnosticPcWatchHits();
    }
    u32 gcProcessorPc() const { return gcCpu_.pc(); }
    u32 gcProcessorInstructionPc() const { return gcCpu_.instructionPc(); }
    u32 gcProcessorPc0() const { return gcCpu_.pc0(); }
    u32 gcProcessorPc1() const { return gcCpu_.pc1(); }
    u32 gcProcessorPc2() const { return gcCpu_.pc2(); }
    u32 gcProcessorNextPc() const { return gcCpu_.nextPc(); }
    u32 gcProcessorIretPc() const { return gcCpu_.iretPc(); }
    u32 gcProcessorInstruction() const {
        return gcCpu_.executingInstruction();
    }
    u32 gcProcessorPrefetchedInstruction() const {
        return gcCpu_.prefetchedInstruction();
    }
    u32 gcProcessorPipelineFlags() const { return gcCpu_.pipelineFlags(); }
    u32 gcProcessorNextPipelineFlags() const {
        return gcCpu_.nextPipelineFlags();
    }
    u32 gcProcessorOldStatus() const { return gcCpu_.oldProcessorStatus(); }
    std::size_t gcProcessorRecentInstructionCount() const {
        return gcCpu_.recentInstructionCount();
    }
    u32 gcProcessorRecentPc(std::size_t back) const {
        return gcCpu_.recentInstructionPc(back);
    }
    u32 gcProcessorRecentInstruction(std::size_t back) const {
        return gcCpu_.recentInstruction(back);
    }
    u32 gcProcessorStatus() const { return gcCpu_.processorStatus(); }
    u32 gcProcessorConfiguration() const { return gcCpu_.configuration(); }
    u32 gcProcessorVectorBase() const { return gcCpu_.vectorAreaBase(); }
    u32 gcProcessorMmu() const { return gcCpu_.mmuConfiguration(); }
    u32 gcProcessorTlbRegister(std::size_t index) const {
        return gcCpu_.tlbRegister(index);
    }
    u32 gcProcessorRegister(std::size_t index) const {
        return gcCpu_.registerValue(index);
    }
    u32 gcProcessorRegisterWindowBase() const {
        return gcCpu_.registerWindowBase();
    }
    std::size_t gcProcessorRegister64ChangeCount() const {
        return gcCpu_.diagnosticRegister64ChangeCount();
    }
    u32 gcProcessorRegister64ChangePc(std::size_t back) const {
        return gcCpu_.diagnosticRegister64ChangePc(back);
    }
    u32 gcProcessorRegister64ChangeInstruction(std::size_t back) const {
        return gcCpu_.diagnosticRegister64ChangeInstruction(back);
    }
    u32 gcProcessorRegister64ChangeOldValue(std::size_t back) const {
        return gcCpu_.diagnosticRegister64ChangeOldValue(back);
    }
    u32 gcProcessorRegister64ChangeNewValue(std::size_t back) const {
        return gcCpu_.diagnosticRegister64ChangeNewValue(back);
    }
    u32 gcDiagnosticVramWord(u32 offset) const {
        if ((offset & 3u) != 0 || offset > vram_.size() - 4u) return 0;
        return (u32(vram_[offset]) << 24) |
               (u32(vram_[offset + 1u]) << 16) |
               (u32(vram_[offset + 2u]) << 8) |
               u32(vram_[offset + 3u]);
    }
    u32 gcDiagnosticDramWord(u32 offset) const {
        return gcReadDramWord(offset);
    }
    u32 gcDiagnosticExpansionDramWord(u32 offset) const {
        return gcReadExpansionDramWord(offset);
    }
    u32 gcDiagnosticReadProcessorData(u32 address,
                                      bool inputOutput = false) {
        return gcReadData(address, inputOutput);
    }
    void gcDiagnosticWriteProcessorData(u32 address, u32 value,
                                        bool inputOutput = false) {
        gcWriteData(address, value, inputOutput);
    }
    u32 gcDiagnosticSramWord(u32 offset) const {
        return gcReadSramWord(offset);
    }
    u32 gcDiagnosticVectorWord(u32 offset) const {
        return gcReadVector(offset);
    }
    bool gcHostRequestPending() const { return gcHostRequest_; }
    bool gcMacRequestPending() const { return gcMacRequest_; }
    u32 gcMfbDiagnosticRegister(u32 offset) const {
        if ((offset & 3u) != 0 || offset >= 0x200u) return 0;
        return gcMfbShadow_[offset >> 2u];
    }
    u64 gcUnknownDataAccesses() const {
        return gcUnknownDataReads_ + gcUnknownDataWrites_;
    }
    u64 gcProcessorDataReads() const { return gcDataReads_; }
    u64 gcProcessorDataWrites() const { return gcDataWrites_; }
    // Diagnostic count of bytes written to screen memory by the Am29000,
    // distinct from Macintosh/NuBus writes to the same physical VRAM.
    u64 gcProcessorVramWrites() const { return gcProcessorVramWrites_; }
    u32 gcFirstUnknownDataAddress() const { return gcFirstUnknownDataAddress_; }

    int width() const { return kWidth; }
    int height() const { return kHeight; }
    int bitsPerPixel() const {
        if (mode_ == 0x84) return 24;
        const int shift = std::clamp<int>(mode_ - 0x80, 0, 3);
        return 1 << shift;
    }
    u16 mode() const { return mode_; }
    mutable u64 reads = 0;
    mutable u64 romReads = 0;
    mutable u64 superReads = 0;
    u64 writes = 0;
    u64 vramWrites = 0;
    u64 vblankCount = 0;
    u64 vblankAssertions = 0;
    u64 vblankAcks = 0;
    u64 lastFrameVramWrites = 0;
    mutable std::array<u32, 64> firstReadOffsets{};
    mutable std::size_t firstReadCount = 0;
    mutable std::array<u32, 128> firstGcReadOffsets{};
    mutable std::array<u8, 128> firstGcReadValues{};
    mutable std::size_t firstGcReadCount = 0;
    std::array<u32, 128> firstGcWriteOffsets{};
    std::array<u8, 128> firstGcWriteValues{};
    std::size_t firstGcWriteCount = 0;
    std::array<u32, 128> gcUnknownTraceAddresses{};
    std::array<u32, 128> gcUnknownTracePcs{};
    std::array<u32, 128> gcUnknownTraceValues{};
    std::array<u8, 128> gcUnknownTraceWrites{};
    std::size_t gcUnknownTraceCount = 0;
    // First physical writes issued while GC QuickDraw's loaded image is
    // executing.  These are diagnostic-only and deliberately omitted from
    // save states so a replay starts with a clean capture window.
    std::array<u32, 512> gcQuickDrawWriteTraceAddresses{};
    std::array<u32, 512> gcQuickDrawWriteTracePcs{};
    std::array<u32, 512> gcQuickDrawWriteTraceValues{};
    std::size_t gcQuickDrawWriteTraceCount = 0;
    std::array<u32, 512> gcFramebufferReadTraceAddresses{};
    std::array<u32, 512> gcFramebufferReadTracePcs{};
    std::array<u32, 512> gcFramebufferReadTraceValues{};
    std::size_t gcFramebufferReadTraceCount = 0;
    // Reads which consume the first observed accelerated raster work buffer.
    // This narrow, post-store history is diagnostic-only and intentionally
    // omitted from save states so checkpoint replays begin with a clean log.
    std::array<u32, 2048> gcWorkBufferReadTraceAddresses{};
    std::array<u32, 2048> gcWorkBufferReadTracePcs{};
    std::array<u32, 2048> gcWorkBufferReadTraceValues{};
    std::size_t gcWorkBufferReadTraceCount = 0;
    std::array<u64, 256> gcReadRegionCounts{};
    std::array<u32, 256> gcReadRegionFirstAddresses{};
    std::array<u32, 256> gcReadRegionFirstPcs{};
    // Address-region census for Am29000 stores. This is diagnostic-only and
    // intentionally starts fresh after reset or state restore.
    std::array<u64, 256> gcWriteRegionCounts{};
    std::array<u32, 256> gcWriteRegionFirstAddresses{};
    std::array<u32, 256> gcWriteRegionFirstValues{};
    std::array<u32, 256> gcWriteRegionFirstPcs{};
    // Per-instruction census for writes to the GC's local DRAM and VRAM.
    // This separates raster loops from one-off allocator/IPC traffic without
    // recording every store. Like the other trace fields, it is diagnostic
    // only and deliberately omitted from save states.
    std::array<u32, 512> gcLocalWriteSitePcs{};
    std::array<u8, 512> gcLocalWriteSiteRegions{};
    std::array<u64, 512> gcLocalWriteSiteCounts{};
    std::array<u32, 512> gcLocalWriteSiteMinAddresses{};
    std::array<u32, 512> gcLocalWriteSiteMaxAddresses{};
    std::array<u32, 512> gcLocalWriteSiteFirstValues{};
    std::array<u32, 512> gcLocalWriteSiteLastValues{};
    std::size_t gcLocalWriteSiteCount = 0;
    // Bounded histories for the two interfaces that delimit an accelerated
    // operation: host GCQD commands entering shared DRAM and Am29000 writes
    // to MFB. These are diagnostic-only and intentionally restart after a
    // reset or state restore.
    std::array<u64, 128> gcMfbWriteCounts{};
    std::array<u32, 128> gcMfbWriteFirstValues{};
    std::array<u32, 128> gcMfbWriteLastValues{};
    std::array<u32, 128> gcMfbWriteLastPcs{};
    std::array<u32, 1024> gcMfbWriteTraceAddresses{};
    std::array<u32, 1024> gcMfbWriteTraceValues{};
    std::array<u32, 1024> gcMfbWriteTracePcs{};
    std::size_t gcMfbWriteTraceCount = 0;
    std::array<u32, 512> gcHostCommandValues{};
    std::array<u8, 512> gcHostCommandSources{};
    std::size_t gcHostCommandCount = 0;
    std::array<u64, 0x2000> gcHostSelectorCounts{};
    std::array<u32, 512> gcCompletionTraceAddresses{};
    std::array<u32, 512> gcCompletionTraceValues{};
    std::array<u32, 512> gcCompletionTracePcs{};
    std::array<std::array<u32, 16>, 512> gcCompletionTraceRegisters{};
    std::size_t gcCompletionTraceCount = 0;
    // Preserve the accelerator state at the first non-success completion.
    // The Am29000 can execute dozens of instructions before the Macintosh
    // observes the completion word, so its ordinary recent-PC ring no longer
    // contains the branch which selected the error path by report time.
    bool gcAbnormalCompletionCaptured = false;
    u32 gcAbnormalCompletionAddress = 0;
    u32 gcAbnormalCompletionValue = 0;
    u32 gcAbnormalCompletionPc = 0;
    u64 gcAbnormalCompletionInstructions = 0;
    u32 gcAbnormalCompletionCps = 0;
    u32 gcAbnormalCompletionOps = 0;
    u32 gcAbnormalCompletionPc0 = 0;
    u32 gcAbnormalCompletionPc1 = 0;
    u32 gcAbnormalCompletionPc2 = 0;
    u32 gcAbnormalCompletionIretPc = 0;
    std::array<u32, 192> gcAbnormalCompletionRegisters{};
    std::array<u32, 512> gcAbnormalCompletionRecentPcs{};
    std::array<u32, 512> gcAbnormalCompletionRecentInstructions{};
    std::size_t gcAbnormalCompletionRecentCount = 0;
    std::array<u32, 256> gcLowReadTraceAddresses{};
    std::array<u32, 256> gcLowReadTracePcs{};
    std::array<u32, 256> gcLowReadTraceValues{};
    std::array<u32, 256> gcLowReadTraceCps{};
    std::array<u32, 256> gcLowReadTraceOps{};
    std::array<u32, 256> gcLowReadTraceInstructions{};
    std::size_t gcLowReadTraceCount = 0;
    std::array<u32, 128> gcPointerReadTraceAddresses{};
    std::array<u32, 128> gcPointerReadTracePcs{};
    std::array<u32, 128> gcPointerReadTraceValues{};
    std::size_t gcPointerReadTraceCount = 0;
    std::array<u32, 1024> gcMapTraceBases{};
    std::array<u32, 1024> gcMapTraceLengths{};
    std::array<u32, 1024> gcMapTracePhysicals{};
    std::array<u32, 1024> gcMapTracePcs{};
    std::size_t gcMapTraceCount = 0;
    std::array<u32, 256> gcSyncTraceAddresses{};
    std::array<u32, 256> gcSyncTracePcs{};
    std::array<u32, 256> gcSyncTraceValues{};
    std::array<u8, 256> gcSyncTraceWrites{};
    std::size_t gcSyncTraceCount = 0;
    bool gcSyncSnapshotCaptured = false;
    std::array<u32, 128> gcSyncRegisterSnapshot{};
    std::array<u32, 64> gcSemaphoreMutationValues{};
    std::array<u32, 64> gcSemaphoreMutationPcs{};
    std::array<u8, 64> gcSemaphoreMutationSources{};
    std::size_t gcSemaphoreMutationCount = 0;
    bool gcPointerMfbSnapshotCaptured = false;
    bool gcRasterMfbSnapshotCaptured = false;
    std::array<u32, 128> gcPointerMfbSnapshot{};
    std::array<u32, 128> gcRasterMfbSnapshot{};
    std::array<u32, 128> gcPointerRegisterSnapshot{};
    std::array<u32, 128> gcRasterRegisterSnapshot{};
    bool gcLowWriteSnapshotCaptured = false;
    u32 gcLowWriteSnapshotAddress = 0;
    u32 gcLowWriteSnapshotValue = 0;
    u32 gcLowWriteSnapshotPc = 0;
    std::array<u32, 128> gcLowWriteRegisterSnapshot{};
    std::array<u32, 128> gcLowWriteMfbSnapshot{};
    std::array<u32, 256> gcLowWriteMapSnapshot{};
    bool gcHeapOverwriteCaptured = false;
    u32 gcHeapOverwritePc = 0;
    u32 gcHeapOverwriteValue = 0;
    std::array<u32, 32> gcHeapOverwriteGlobals{};
    std::array<u32, 24> gcHeapOverwriteRegisters{};
    std::array<u32, 128> gcHeapOverwriteMfb{};
    bool gcRasterStoreSnapshotCaptured = false;
    u32 gcRasterStoreSnapshotAddress = 0;
    u32 gcRasterStoreSnapshotValue = 0;
    u32 gcRasterStoreSnapshotPc = 0;
    std::array<u32, 128> gcRasterStoreRegisterSnapshot{};
    bool gcUnknownSnapshotCaptured = false;
    u32 gcUnknownSnapshotAddress = 0;
    u32 gcUnknownSnapshotPc = 0;
    std::array<u32, 128> gcUnknownRegisterSnapshot{};

    void render(u32* argb) const {
        if (!argb) return;
        const int bpp = bitsPerPixel();
        const u32 start = genuineGc_ ? displayBase() : 0;
        const u32 rowBytes = genuineGc_ ? displayStride() :
            static_cast<u32>((kWidth * bpp) / 8);
        if (start >= vram_.size()) {
            std::fill_n(argb, static_cast<std::size_t>(kWidth) * kHeight,
                        0xFF000000u);
            return;
        }
        for (int y = 0; y < kHeight; ++y) {
            const std::size_t rowOffset = start + static_cast<std::size_t>(y) * rowBytes;
            if (rowOffset + rowBytes > vram_.size()) break;
            const u8* row = vram_.data() + rowOffset;
            for (int x = 0; x < kWidth; ++x) {
                if (bpp == 24) {
                    const std::size_t pixel = static_cast<std::size_t>(x) * 3u;
                    if (pixel + 2 >= rowBytes) break;
                    argb[static_cast<std::size_t>(y) * kWidth + x] =
                        0xFF000000u | (u32(row[pixel]) << 16) |
                        (u32(row[pixel + 1]) << 8) | row[pixel + 2];
                    continue;
                }
                const u32 levels = (1u << bpp) - 1u;
                const int bit = x * bpp;
                const int shift = 8 - bpp - (bit & 7);
                const u32 index = (row[bit >> 3] >> shift) & levels;
                // Macintosh indexed value zero is white and the maximum value
                // is black before a CLUT is programmed.
                if (genuineGc_) {
                    argb[static_cast<std::size_t>(y) * kWidth + x] =
                        // The GC RAMDAC uses the packed pixel value directly
                        // as its CLUT address.  In particular, the declaration
                        // ROM programs entries 0 and 1 for the 1-bit startup
                        // screen; scaling 1 to 128 turns black into mid-grey.
                        palette_[index];
                } else {
                    const u32 component = 255u - (index * 255u / levels);
                    argb[static_cast<std::size_t>(y) * kWidth + x] =
                        0xFF000000u | (component << 16) |
                        (component << 8) | component;
                }
            }
        }
    }

    const std::vector<u8>& declarationRom() const { return rom_; }

private:
    friend class IifxStateCodec;

    bool packedRgb() const { return genuineGc_ && mode_ == 0x84; }

    void writeClutAddress(u8 value) {
        clutAddress_ = value;
        clutComponent_ = 0;
    }

    u8 clutData() const {
        const u32 color = palette_[clutAddress_];
        switch (clutComponent_) {
        case 0: return static_cast<u8>(color >> 16);
        case 1: return static_cast<u8>(color >> 8);
        default: return static_cast<u8>(color);
        }
    }

    void advanceClut() {
        if (++clutComponent_ == 3) {
            ++clutAddress_;
            clutComponent_ = 0;
        }
    }

    u8 readClutData() {
        const u8 value = clutData();
        advanceClut();
        return value;
    }

    void writeClutData(u8 value) {
        clutColor_[clutComponent_] = value;
        if (clutComponent_ == 2) {
            palette_[clutAddress_] = 0xFF000000u |
                (u32(clutColor_[0]) << 16) |
                (u32(clutColor_[1]) << 8) | clutColor_[2];
        }
        advanceClut();
    }

    void writeRamdacControl(u8 value) {
        ramdacControl_ = value;
        ramdacMode_ = static_cast<u8>((value >> 1) & 0x0Fu);
        ramdacConvolution_ = (value & 1u) != 0;
        switch (ramdacMode_) {
        case 0x0: mode_ = 0x80; break;
        case 0x4: mode_ = 0x81; break;
        case 0x8: mode_ = 0x82; break;
        case 0xC: mode_ = 0x83; break;
        // AC842 PBCTRL bits $1C select direct 24-bit color.  The Dolphin ROM
        // writes $9C, whose decoded mode is $E.  Mode $D belongs to the
        // different, non-accelerated JMFB interface.
        case 0xE: mode_ = 0x84; break;
        default: break;
        }
    }

    u32 gcPeekRamdacWord(u32 address) const {
        switch ((address - kGcProcessorRamdacBase) >> 2u) {
        case 0: return u32(clutAddress_) << 24;
        case 1: return u32(clutData()) << 24;
        // PBCTRL is write-only.  The reset firmware reaches this address
        // through an as-yet-uninitialised queue pointer and deliberately
        // waits while it reads as zero; echoing the programmed mode lets it
        // escape that wait before the host has installed the queue.
        case 2: return 0;
        default: return 0;
        }
    }

    u32 gcReadRamdacWord(u32 address) {
        if (address == kGcProcessorRamdacBase + 4u)
            return u32(readClutData()) << 24;
        return gcPeekRamdacWord(address);
    }

    static u8 gcRamdacLowByte(u32 value) {
        // GCOS uses ordinary word stores with the byte in bits 7..0, while
        // the reset firmware also uses byte stores which reach the callback
        // in bits 31..24 after the CPU's word merge.
        return (value & 0x00FFFFFFu) != 0 || value == 0
            ? static_cast<u8>(value)
            : static_cast<u8>(value >> 24);
    }

    void gcWriteRamdacWord(u32 address, u32 value) {
        switch ((address - kGcProcessorRamdacBase) >> 2u) {
        case 0: writeClutAddress(gcRamdacLowByte(value)); break;
        case 1: writeClutData(static_cast<u8>(value >> 24)); break;
        case 2: writeRamdacControl(gcRamdacLowByte(value)); break;
        default: break;
        }
    }

    u32 gcReadSramWord(u32 address) const {
        if (address > kGcSramBytes - 4u) return 0;
        return (u32(gcSram_[address]) << 24) |
               (u32(gcSram_[address + 1u]) << 16) |
               (u32(gcSram_[address + 2u]) << 8) |
               u32(gcSram_[address + 3u]);
    }

    void gcWriteSramWord(u32 address, u32 value) {
        if (address > kGcSramBytes - 4u) return;
        gcSram_[address] = static_cast<u8>(value >> 24);
        gcSram_[address + 1u] = static_cast<u8>(value >> 16);
        gcSram_[address + 2u] = static_cast<u8>(value >> 8);
        gcSram_[address + 3u] = static_cast<u8>(value);
    }

    void noteGcSemaphoreMutation(u8 source) {
        if (gcSemaphoreMutationCount >= gcSemaphoreMutationValues.size())
            return;
        const std::size_t index = gcSemaphoreMutationCount++;
        gcSemaphoreMutationValues[index] = gcReadDramWord(0x8DD4u);
        gcSemaphoreMutationPcs[index] = gcCpu_.instructionPc();
        gcSemaphoreMutationSources[index] = source;
    }

    void noteGcHostCommand(u8 source) {
        const u32 command = gcReadDramWord(0x8DD4u);
        if (command < gcHostSelectorCounts.size())
            ++gcHostSelectorCounts[command];
        if (gcHostCommandCount >= gcHostCommandValues.size()) return;
        const std::size_t index = gcHostCommandCount++;
        gcHostCommandValues[index] = command;
        gcHostCommandSources[index] = source;
    }

    void noteGcCompletion(u32 address, u32 value, u32 pc) {
        if (value != 3u && value != 4u &&
            !gcAbnormalCompletionCaptured) {
            gcAbnormalCompletionCaptured = true;
            gcAbnormalCompletionAddress = address;
            gcAbnormalCompletionValue = value;
            gcAbnormalCompletionPc = pc;
            gcAbnormalCompletionInstructions = gcCpu_.instructions();
            gcAbnormalCompletionCps = gcCpu_.processorStatus();
            gcAbnormalCompletionOps = gcCpu_.oldProcessorStatus();
            gcAbnormalCompletionPc0 = gcCpu_.pc0();
            gcAbnormalCompletionPc1 = gcCpu_.pc1();
            gcAbnormalCompletionPc2 = gcCpu_.pc2();
            gcAbnormalCompletionIretPc = gcCpu_.iretPc();
            for (std::size_t index = 0;
                 index < gcAbnormalCompletionRegisters.size(); ++index)
                gcAbnormalCompletionRegisters[index] =
                    gcCpu_.registerValue(64u + index);
            gcAbnormalCompletionRecentCount = std::min(
                gcCpu_.recentInstructionCount(),
                gcAbnormalCompletionRecentPcs.size());
            for (std::size_t back = 0;
                 back < gcAbnormalCompletionRecentCount; ++back) {
                gcAbnormalCompletionRecentPcs[back] =
                    gcCpu_.recentInstructionPc(back);
                gcAbnormalCompletionRecentInstructions[back] =
                    gcCpu_.recentInstruction(back);
            }
        }
        if (gcCompletionTraceCount >= gcCompletionTraceAddresses.size())
            return;
        const std::size_t index = gcCompletionTraceCount++;
        gcCompletionTraceAddresses[index] = address;
        gcCompletionTraceValues[index] = value;
        gcCompletionTracePcs[index] = pc;
        for (std::size_t reg = 0;
             reg < gcCompletionTraceRegisters[index].size(); ++reg)
            gcCompletionTraceRegisters[index][reg] =
                gcCpu_.registerValue(128u + reg);
    }

    u32 gcReadDramWord(u32 address) const {
        if (gcDram_.size() < 4u || (address & 3u) != 0 ||
            address > gcDram_.size() - 4u) return 0;
        return (u32(gcDram_[address]) << 24) |
               (u32(gcDram_[address + 1u]) << 16) |
               (u32(gcDram_[address + 2u]) << 8) |
               u32(gcDram_[address + 3u]);
    }

    bool isGcHostCompletionAddress(u32 address) const {
        constexpr u32 kIpcCompletionWord = 0x00008C20u;
        const u32 completion = gcReadDramWord(kIpcCompletionWord) & ~3u;
        return completion >= kGcVectorBytes && completion < 0x01000000u &&
               address == completion;
    }

    void gcWriteDramWord(u32 address, u32 value) {
        if (gcDram_.size() < 4u || (address & 3u) != 0 ||
            address > gcDram_.size() - 4u) return;
        gcDram_[address] = static_cast<u8>(value >> 24);
        gcDram_[address + 1u] = static_cast<u8>(value >> 16);
        gcDram_[address + 2u] = static_cast<u8>(value >> 8);
        gcDram_[address + 3u] = static_cast<u8>(value);
        if (address == 0x8DD4u) noteGcSemaphoreMutation(4);
        constexpr u32 kMapTable = 0x316Cu;
        if (address >= kMapTable + 8u &&
            address < kMapTable + 1024u * 12u &&
            ((address - kMapTable) % 12u) == 8u &&
            gcMapTraceCount < gcMapTraceBases.size()) {
            const std::size_t index = gcMapTraceCount++;
            gcMapTraceBases[index] = gcReadDramWord(address - 8u);
            gcMapTraceLengths[index] = gcReadDramWord(address - 4u);
            gcMapTracePhysicals[index] = value;
            gcMapTracePcs[index] = gcCpu_.instructionPc();
        }
    }

    u32 gcReadExpansionDramWord(u32 address) const {
        if (gcExpansionDram_.size() < 4u || (address & 3u) != 0 ||
            address > gcExpansionDram_.size() - 4u) return 0;
        return (u32(gcExpansionDram_[address]) << 24) |
               (u32(gcExpansionDram_[address + 1u]) << 16) |
               (u32(gcExpansionDram_[address + 2u]) << 8) |
               u32(gcExpansionDram_[address + 3u]);
    }

    void gcWriteExpansionDramWord(u32 address, u32 value) {
        if (gcExpansionDram_.size() < 4u || (address & 3u) != 0 ||
            address > gcExpansionDram_.size() - 4u) return;
        gcExpansionDram_[address] = static_cast<u8>(value >> 24);
        gcExpansionDram_[address + 1u] = static_cast<u8>(value >> 16);
        gcExpansionDram_[address + 2u] = static_cast<u8>(value >> 8);
        gcExpansionDram_[address + 3u] = static_cast<u8>(value);
    }

    u32 gcReadSuperWord(u32 address) const {
        const u32 slotOffset = address - kSuperSlotBase;
        return (u32(readSuper(slotOffset)) << 24) |
               (u32(readSuper(slotOffset + 1u)) << 16) |
               (u32(readSuper(slotOffset + 2u)) << 8) |
               u32(readSuper(slotOffset + 3u));
    }

    void gcWriteSuperWord(u32 address, u32 value) {
        const u32 slotOffset = address - kSuperSlotBase;
        writeSuper(slotOffset, static_cast<u8>(value >> 24));
        writeSuper(slotOffset + 1u, static_cast<u8>(value >> 16));
        writeSuper(slotOffset + 2u, static_cast<u8>(value >> 8));
        writeSuper(slotOffset + 3u, static_cast<u8>(value));
    }

    u32 gcReadStandardWord(u32 address) const {
        const u32 slotOffset = address - kSlotBase;
        return (u32(readStandard(slotOffset)) << 24) |
               (u32(readStandard(slotOffset + 1u)) << 16) |
               (u32(readStandard(slotOffset + 2u)) << 8) |
               u32(readStandard(slotOffset + 3u));
    }

    void gcWriteStandardWord(u32 address, u32 value) {
        const u32 slotOffset = address - kSlotBase;
        writeStandard(slotOffset, static_cast<u8>(value >> 24));
        writeStandard(slotOffset + 1u, static_cast<u8>(value >> 16));
        writeStandard(slotOffset + 2u, static_cast<u8>(value >> 8));
        writeStandard(slotOffset + 3u, static_cast<u8>(value));
    }

    u32 gcSupplyCacheFillWord(u32 value) {
        if (gcCacheFillActive_ && gcCacheFillWordsRemaining_ != 0 &&
            gcCacheFillAddress_ <= kGcSramBytes - 4u) {
            gcWriteSramWord(gcCacheFillAddress_, value);
            gcCacheFillAddress_ += 4u;
            --gcCacheFillWordsRemaining_;
        }
        return value;
    }

    u32 gcReadVector(u32 address) const {
        if (address >= kGcProcessorDramBase &&
            address <= kGcProcessorDramBase + kGcVectorBytes - 4u)
            return gcReadDramWord(address - kGcProcessorDramBase);
        const u32 offset = address & (kGcVectorBytes - 1u);
        if ((address & 3u) != 0 || offset > kGcVectorBytes - 4u) return 0;
        return (u32(gcVectorRam_[offset]) << 24) |
               (u32(gcVectorRam_[offset + 1u]) << 16) |
               (u32(gcVectorRam_[offset + 2u]) << 8) |
               u32(gcVectorRam_[offset + 3u]);
    }

    void gcWriteVector(u32 address, u32 value) {
        const u32 offset = address & (kGcVectorBytes - 1u);
        if ((address & 3u) != 0 || offset > kGcVectorBytes - 4u) return;
        gcVectorRam_[offset] = static_cast<u8>(value >> 24);
        gcVectorRam_[offset + 1u] = static_cast<u8>(value >> 16);
        gcVectorRam_[offset + 2u] = static_cast<u8>(value >> 8);
        gcVectorRam_[offset + 3u] = static_cast<u8>(value);
    }

    void resetGcVectorStore() {
        gcVectorRam_.fill(0);
        // Dolphin's reset firmware constructs all 256 vectors in GC DRAM and
        // then points VAB at $4C000000.  There is no host-synthesized TLB-miss
        // vector: inventing one bypasses the card's own MMU bootstrap.
    }

    u32 gcReadInstruction(u32 address, bool translated) {
        ++gcInstructionReads_;
        static_cast<void>(translated);
        if ((address & 3u) == 0 && address <= kGcSramBytes - 4u)
            return gcReadSramWord(address);
        if ((address & 3u) == 0 &&
            address >= kGcProcessorSramBase &&
            address <= kGcProcessorSramBase + kGcSramBytes - 4u)
            return gcReadSramWord(address - kGcProcessorSramBase);
        // A translated instruction-SRAM page carries MFB's data-space select
        // in bit 30.  The firmware's first real TLB entry maps virtual $F000
        // to $42003000; it is the same physical SRAM line fetched directly
        // as $02003000 during reset.
        if ((address & 3u) == 0 &&
            address >= kGcProcessorDataSramBase &&
            address <= kGcProcessorDataSramBase + kGcSramBytes - 4u)
            return gcReadSramWord(address - kGcProcessorDataSramBase);
        if ((address & 3u) == 0 &&
            address >= kGcProcessorDramBase &&
            address <= kGcProcessorDramBase + kGcDramBytes - 4u)
            return gcReadDramWord(address - kGcProcessorDramBase);
        if ((address & 3u) == 0 &&
            address >= kGcProcessorExpansionDramBase &&
            address <= kGcProcessorExpansionDramBase +
                       kGcExpansionDramBytes - 4u)
            return gcReadExpansionDramWord(
                address - kGcProcessorExpansionDramBase);
        if ((address & 3u) == 0 &&
            address >= 0x9D000000u &&
            address <= 0x9D000000u + kGcExpansionDramBytes - 4u)
            return gcReadExpansionDramWord(address - 0x9D000000u);
        {
            noteUnknownGcDataAddress(address, 0, false);
            ++gcUnknownDataReads_;
            return 0;
        }
    }

    u32 gcPeekData(u32 address, bool inputOutput) {
        if ((address & 3u) == 0 &&
            address >= kGcProcessorRamdacBase &&
            address < kGcProcessorRamdacBase + 0x10u)
            return gcPeekRamdacWord(address);
        return gcReadData(address, inputOutput);
    }

    u32 gcReadData(u32 address, bool inputOutput) {
        ++gcDataReads_;
        const u32 readPc = gcCpu_.instructionPc();
        const std::size_t readRegion = address >> 24u;
        if (gcReadRegionCounts[readRegion]++ == 0) {
            gcReadRegionFirstAddresses[readRegion] = address;
            gcReadRegionFirstPcs[readRegion] = readPc;
        }
        const auto noteFramebufferRead = [&](u32 value) {
            const bool processorVram =
                address >= kGcProcessorVramBase &&
                address < kGcProcessorVramBase + kGcVramBytes;
            const bool superVram = address >= kSuperSlotBase + kGcVramOffset +
                    kGcSharedDramBytes &&
                address < kSuperSlotBase + kGcVramOffset + kGcVramBytes;
            if ((processorVram || superVram) &&
                gcFramebufferReadTraceCount <
                    gcFramebufferReadTraceAddresses.size()) {
                const std::size_t index = gcFramebufferReadTraceCount++;
                gcFramebufferReadTraceAddresses[index] = address;
                gcFramebufferReadTracePcs[index] = readPc;
                gcFramebufferReadTraceValues[index] = value;
            }
            return value;
        };
        if ((address & 0x0000FFFFu) == 0x00008DD4u &&
            gcSyncTraceCount < gcSyncTraceAddresses.size()) {
            const std::size_t index = gcSyncTraceCount++;
            gcSyncTraceAddresses[index] = address;
            gcSyncTracePcs[index] = readPc;
            gcSyncTraceValues[index] = 0;
            gcSyncTraceWrites[index] = 0;
        }
        if (address == 0x9C008DD4u && readPc == 0x9D7F0058u &&
            !gcSyncSnapshotCaptured) {
            gcSyncSnapshotCaptured = true;
            for (std::size_t index = 0;
                 index < gcSyncRegisterSnapshot.size(); ++index)
                gcSyncRegisterSnapshot[index] =
                    gcCpu_.registerValue(64u + index);
        }
        const auto notePointerRead = [&](u32 value) {
            if (readPc < 0x9D7D4240u || readPc > 0x9D7D4310u ||
                gcPointerReadTraceCount >= gcPointerReadTraceAddresses.size())
                return value;
            const std::size_t index = gcPointerReadTraceCount++;
            gcPointerReadTraceAddresses[index] = address;
            gcPointerReadTracePcs[index] = readPc;
            gcPointerReadTraceValues[index] = value;
            return value;
        };
        if ((address & 3u) == 0 && address < kGcVectorBytes)
            return notePointerRead(gcReadVector(address));
        if ((address & 3u) == 0 &&
            address >= kGcProcessorVramBase &&
            address <= kGcProcessorVramBase + kGcVramBytes - 4u)
            return notePointerRead(noteFramebufferRead(
                gcDiagnosticVramWord(address - kGcProcessorVramBase)));
        if ((address & 3u) == 0 &&
            address >= kGcProcessorDataSramBase &&
            address <= kGcProcessorDataSramBase + kGcSramBytes - 4u)
            return notePointerRead(
                gcReadSramWord(address - kGcProcessorDataSramBase));
        // RDNC's $FC00xxxx alias is the same low data-DRAM window that the Mac
        // sees in the bottom 64 KiB of slot-9 standard space.  It is not MFB's
        // physically separate instruction-cache SRAM.
        if (!inputOutput && (address & 3u) == 0 &&
            address >= 0xFC000000u &&
            address <= 0xFC000000u + kGcSharedDramBytes - 4u)
            return notePointerRead(gcReadDramWord(address - 0xFC000000u));
        if ((address & 3u) == 0 &&
            address >= 0x44000000u && address < 0x44000200u) {
            // Dolphin's reset loader samples the NuBus slot ID one bit at a
            // time from these four active-low MFB inputs.  They are sense
            // lines, not writable shadow registers: the loader sets a slot-ID
            // bit when the corresponding input reads zero, then constructs
            // GCOS's $9xxxxxxx local address prefix from that nibble.
            if (address >= 0x4400000Cu && address <= 0x44000018u) {
                const u32 bit = 3u - ((address - 0x4400000Cu) >> 2u);
                return notePointerRead(
                    ((kSlot >> bit) & 1u) != 0 ? 0u : 0xFFFFFFFFu);
            }
            if (address == 0x44000050u)
                return notePointerRead(gcInterruptControl_);
            if (address == 0x44000054u)
                return notePointerRead(gcMacInterruptControl_);
            return notePointerRead(
                gcMfbShadow_[(address - 0x44000000u) >> 2u]);
        }
        if (address == 0x46000000u)
            return notePointerRead(gcCacheControl_);
        if ((address & 3u) == 0 &&
            address >= kGcProcessorRamdacBase &&
            address < kGcProcessorRamdacBase + 0x10u)
            return notePointerRead(gcReadRamdacWord(address));
        if ((address & 3u) == 0 &&
            address >= kGcProcessorDramBase &&
            address <= kGcProcessorDramBase + kGcDramBytes - 4u)
            return notePointerRead(gcSupplyCacheFillWord(
                gcReadDramWord(address - kGcProcessorDramBase)));
        if ((address & 3u) == 0 &&
            address >= kGcProcessorExpansionDramBase &&
            address <= kGcProcessorExpansionDramBase +
                       kGcExpansionDramBytes - 4u)
            return notePointerRead(gcSupplyCacheFillWord(
                gcReadExpansionDramWord(
                    address - kGcProcessorExpansionDramBase)));
        // RDNC loops the card's complete slot-9 super-space back locally.
        // This includes GCOS's $9D expansion heap, the $9C framebuffer/IPC
        // window, and the lower SRAM/control apertures.
        if ((address & 3u) == 0 &&
            address >= kSuperSlotBase && address <= 0x9FFFFFFFu - 3u) {
            const u32 value = noteFramebufferRead(
                gcSupplyCacheFillWord(gcReadSuperWord(address)));
            if (gcRasterStoreSnapshotCaptured &&
                address >= 0x9D00A000u && address < 0x9D00C000u &&
                gcWorkBufferReadTraceCount <
                    gcWorkBufferReadTraceAddresses.size()) {
                const std::size_t index = gcWorkBufferReadTraceCount++;
                gcWorkBufferReadTraceAddresses[index] = address;
                gcWorkBufferReadTracePcs[index] = readPc;
                gcWorkBufferReadTraceValues[index] = value;
            }
            return notePointerRead(value);
        }
        // Standard slot addresses can retain 24-bit pointer tag bits in
        // AD23..AD20.  Dolphin, like Apple's recommended card interface,
        // ignores those four bits in its one-megabyte compatibility window.
        if ((address & 3u) == 0 &&
            address >= kSlotBase && address <= kSlotBase + 0x00FFFFFCu)
            return notePointerRead(
                gcSupplyCacheFillWord(gcReadStandardWord(address)));
        // Slot $0 is the Macintosh logic board on NuBus.  In particular,
        // $F0800000-$F0FFFFFF is the system-ROM alias used by GCQD when it
        // caches ROM-resident QuickDraw structures.
        if ((address & 3u) == 0 && address >= 0xF0800000u &&
            address <= 0xF0FFFFFCu && gcBusMasterRead32_)
            return notePointerRead(
                gcSupplyCacheFillWord(gcBusMasterRead32_(address)));
        // RDNC accepts ordinary Macintosh physical addresses directly.  In
        // 32-bit addressing mode the IIfx can place GCQD's BitMaps, regions,
        // and stack frames anywhere in its 128 MiB RAM space; restricting
        // these cycles to 24 bits makes otherwise valid accelerator commands
        // see zero-filled structures above 16 MiB.  GCQD uses AS=0 loads for
        // these Macintosh structures, so AS is not an MFB/RDNC chip select.
        // Local display memory has its explicit $41000000 aperture.  The first
        // 1 KiB remains the accelerator's vector store; the rest of the low
        // 128 MiB is the IIfx system-memory window.
        if ((address & 3u) == 0 && address >= kGcVectorBytes &&
            address < 0x08000000u && gcBusMasterRead32_) {
            const bool pointerRead = readPc == 0x9D7D42E0u;
            const bool rasterRead = readPc >= 0x9D7C0000u &&
                                    readPc < 0x9D7C1000u;
            const auto captureSnapshot = [&](std::array<u32, 128>& mfb,
                                             std::array<u32, 128>& regs) {
                mfb = gcMfbShadow_;
                for (std::size_t index = 0; index < regs.size(); ++index)
                    regs[index] = gcCpu_.registerValue(64u + index);
            };
            if (pointerRead && !gcPointerMfbSnapshotCaptured) {
                gcPointerMfbSnapshotCaptured = true;
                captureSnapshot(gcPointerMfbSnapshot,
                                gcPointerRegisterSnapshot);
            }
            if (rasterRead && !gcRasterMfbSnapshotCaptured) {
                gcRasterMfbSnapshotCaptured = true;
                captureSnapshot(gcRasterMfbSnapshot,
                                gcRasterRegisterSnapshot);
            }
            const u32 value = gcSupplyCacheFillWord(
                gcBusMasterRead32_(address));
            if ((rasterRead || pointerRead) &&
                gcLowReadTraceCount < gcLowReadTraceAddresses.size()) {
                const std::size_t index = gcLowReadTraceCount++;
                gcLowReadTraceAddresses[index] = address;
                gcLowReadTracePcs[index] = readPc;
                gcLowReadTraceValues[index] = value;
                gcLowReadTraceCps[index] = gcCpu_.processorStatus();
                gcLowReadTraceOps[index] = gcCpu_.oldProcessorStatus();
                gcLowReadTraceInstructions[index] =
                    gcCpu_.executingInstruction();
            }
            return notePointerRead(value);
        }
        // In 24-bit Macintosh addressing mode the Memory Manager stores
        // handle-state flags in the otherwise-unused high byte of master
        // pointers.  GC QuickDraw imports Macintosh structures directly and
        // therefore presents locked handles such as $80005098 to RDNC.  The
        // NuBus master cycle addresses the low 24 bits, just as StripAddress
        // does on the Macintosh side.
        if ((address & 3u) == 0 &&
            (address & 0xFF000000u) == 0x80000000u &&
            (address & 0x00FFFFFFu) >= kGcVectorBytes &&
            gcBusMasterRead32_)
            return notePointerRead(gcSupplyCacheFillWord(
                gcBusMasterRead32_(address & 0x00FFFFFFu)));
        // RDNC exposes Macintosh physical memory through its cache-coherent
        // $FC aperture. The low 24 bits are the NuBus master address.
        if (inputOutput && (address & 3u) == 0 &&
            (address & 0xFF000000u) == 0xFC000000u &&
            gcBusMasterRead32_)
            return notePointerRead(gcSupplyCacheFillWord(
                gcBusMasterRead32_(address & 0x00FFFFFFu)));
        noteUnknownGcDataAddress(address, 0, false);
        ++gcUnknownDataReads_;
        return notePointerRead(0);
    }

    void gcWriteData(u32 address, u32 value, bool inputOutput) {
        ++gcDataWrites_;
        const u32 writePc = gcCpu_.instructionPc();
        const std::size_t writeRegion = address >> 24u;
        if (gcWriteRegionCounts[writeRegion]++ == 0) {
            gcWriteRegionFirstAddresses[writeRegion] = address;
            gcWriteRegionFirstValues[writeRegion] = value;
            gcWriteRegionFirstPcs[writeRegion] = writePc;
        }
        if (writeRegion == 0x41u || writeRegion == 0x4Cu ||
            writeRegion == 0x4Du || writeRegion == 0x9Cu ||
            writeRegion == 0x9Du) {
            std::size_t site = 0;
            while (site < gcLocalWriteSiteCount &&
                   (gcLocalWriteSitePcs[site] != writePc ||
                    gcLocalWriteSiteRegions[site] != writeRegion))
                ++site;
            if (site == gcLocalWriteSiteCount &&
                site < gcLocalWriteSitePcs.size()) {
                ++gcLocalWriteSiteCount;
                gcLocalWriteSitePcs[site] = writePc;
                gcLocalWriteSiteRegions[site] =
                    static_cast<u8>(writeRegion);
                gcLocalWriteSiteMinAddresses[site] = address;
                gcLocalWriteSiteMaxAddresses[site] = address;
                gcLocalWriteSiteFirstValues[site] = value;
            }
            if (site < gcLocalWriteSiteCount) {
                ++gcLocalWriteSiteCounts[site];
                gcLocalWriteSiteMinAddresses[site] =
                    std::min(gcLocalWriteSiteMinAddresses[site], address);
                gcLocalWriteSiteMaxAddresses[site] =
                    std::max(gcLocalWriteSiteMaxAddresses[site], address);
                gcLocalWriteSiteLastValues[site] = value;
            }
        }
        if ((address & 3u) == 0 && isGcHostCompletionAddress(address))
            noteGcCompletion(address, value, writePc);
        if ((address & 0x0000FFFFu) == 0x00008DD4u &&
            gcSyncTraceCount < gcSyncTraceAddresses.size()) {
            const std::size_t index = gcSyncTraceCount++;
            gcSyncTraceAddresses[index] = address;
            gcSyncTracePcs[index] = writePc;
            gcSyncTraceValues[index] = value;
            gcSyncTraceWrites[index] = 1;
        }
        if (((address >= kGcVectorBytes && address < 0x01000000u) ||
             (address >= kGcProcessorVramBase &&
              address < kGcProcessorVramBase + kGcVramBytes) ||
             (address & 0xFF000000u) == 0xFC000000u) &&
            gcQuickDrawWriteTraceCount <
                gcQuickDrawWriteTraceAddresses.size()) {
            const std::size_t index = gcQuickDrawWriteTraceCount++;
            gcQuickDrawWriteTraceAddresses[index] = address;
            gcQuickDrawWriteTracePcs[index] = writePc;
            gcQuickDrawWriteTraceValues[index] = value;
        }
        if (!inputOutput && (address & 3u) == 0 &&
            address >= kGcVectorBytes &&
            address < 0x01000000u && !isGcHostCompletionAddress(address) &&
            writePc >= 0x9D000000u && !gcLowWriteSnapshotCaptured) {
            gcLowWriteSnapshotCaptured = true;
            gcLowWriteSnapshotAddress = address;
            gcLowWriteSnapshotValue = value;
            gcLowWriteSnapshotPc = writePc;
            for (std::size_t index = 0;
                 index < gcLowWriteRegisterSnapshot.size(); ++index)
                gcLowWriteRegisterSnapshot[index] =
                    gcCpu_.registerValue(64u + index);
            gcLowWriteMfbSnapshot = gcMfbShadow_;
            for (std::size_t index = 0;
                 index < gcLowWriteMapSnapshot.size(); ++index)
                gcLowWriteMapSnapshot[index] =
                    gcReadDramWord(0x316Cu + static_cast<u32>(index) * 4u);
        }
        if (address == 0x0000B184u && !gcHeapOverwriteCaptured) {
            gcHeapOverwriteCaptured = true;
            gcHeapOverwritePc = writePc;
            gcHeapOverwriteValue = value;
            for (std::size_t index = 0;
                 index < gcHeapOverwriteGlobals.size(); ++index)
                gcHeapOverwriteGlobals[index] =
                    gcCpu_.registerValue(96u + index);
            for (std::size_t index = 0;
                 index < gcHeapOverwriteRegisters.size(); ++index)
                gcHeapOverwriteRegisters[index] =
                    gcCpu_.registerValue(128u + index);
            gcHeapOverwriteMfb = gcMfbShadow_;
        }
        if (writePc == 0x9D7C071Cu && !gcRasterStoreSnapshotCaptured) {
            gcRasterStoreSnapshotCaptured = true;
            gcRasterStoreSnapshotAddress = address;
            gcRasterStoreSnapshotValue = value;
            gcRasterStoreSnapshotPc = writePc;
            for (std::size_t index = 0;
                 index < gcRasterStoreRegisterSnapshot.size(); ++index)
                gcRasterStoreRegisterSnapshot[index] =
                    gcCpu_.registerValue(64u + index);
        }
        if ((address & 3u) == 0 && address < kGcVectorBytes) {
            gcWriteVector(address, value);
            return;
        }
        if ((address & 3u) == 0 &&
            address >= kGcProcessorVramBase &&
            address <= kGcProcessorVramBase + kGcVramBytes - 4u) {
            const u32 offset = address - kGcProcessorVramBase;
            vram_[offset] = static_cast<u8>(value >> 24);
            vram_[offset + 1u] = static_cast<u8>(value >> 16);
            vram_[offset + 2u] = static_cast<u8>(value >> 8);
            vram_[offset + 3u] = static_cast<u8>(value);
            writes += 4u;
            vramWrites += 4u;
            frameVramWrites_ += 4u;
            gcProcessorVramWrites_ += 4u;
            return;
        }
        if ((address & 3u) == 0 &&
            address >= kGcProcessorDataSramBase &&
            address <= kGcProcessorDataSramBase + kGcSramBytes - 4u) {
            // GCOS arms MFB's instruction-cache fill path, then writes the
            // destination page address into the word immediately preceding
            // that page.  This is an MFB tag command rather than an SRAM
            // store: committing it would replace the final instruction in
            // the preceding one-KiB cache page. Subsequent burst reads supply
            // the new page's cache data.
            if (gcCacheFillActive_ && value == address + 4u &&
                value >= kGcProcessorDataSramBase &&
                value < kGcProcessorDataSramBase + kGcSramBytes) {
                gcCacheFillAddress_ = value - kGcProcessorDataSramBase;
                gcCacheFillWordsRemaining_ = 1024u / sizeof(u32);
                return;
            }
            gcWriteSramWord(address - kGcProcessorDataSramBase, value);
            return;
        }
        if (!inputOutput && (address & 3u) == 0 &&
            address >= 0xFC000000u &&
            address <= 0xFC000000u + kGcSharedDramBytes - 4u) {
            gcWriteDramWord(address - 0xFC000000u, value);
            return;
        }
        if ((address & 3u) == 0 &&
            address >= 0x44000000u && address < 0x44000200u) {
            const std::size_t registerIndex =
                (address - 0x44000000u) >> 2u;
            if (gcMfbWriteCounts[registerIndex]++ == 0)
                gcMfbWriteFirstValues[registerIndex] = value;
            gcMfbWriteLastValues[registerIndex] = value;
            gcMfbWriteLastPcs[registerIndex] = writePc;
            if (gcMfbWriteTraceCount < gcMfbWriteTraceAddresses.size()) {
                const std::size_t index = gcMfbWriteTraceCount++;
                gcMfbWriteTraceAddresses[index] = address;
                gcMfbWriteTraceValues[index] = value;
                gcMfbWriteTracePcs[index] = writePc;
            }
            gcMfbShadow_[registerIndex] = value;
            if (address == 0x44000050u) {
                gcInterruptControl_ = value;
                if (value == 0) {
                    gcHostRequest_ = false;
                    gcMfbShadow_[0x64u >> 2u] = 0;
                    gcCpu_.setInput(2, false);
                }
            } else if (address == 0x44000054u) {
                gcMacInterruptControl_ = value;
                // GCOS asserts +$54, waits until MFB reflects the request at
                // +$68, and then deasserts +$54.  The NuBus-facing request is
                // latched until the Macintosh acknowledges it, so the short
                // accelerator-side pulse cannot be lost between CPU slices.
                gcMfbShadow_[0x68u >> 2u] = value != 0 ? 0xFFFFFFFFu : 0u;
                if (value != 0) gcMacRequest_ = true;
            } else if (address == 0x44000088u) {
                gcCacheFillActive_ = value != 0;
                if (!gcCacheFillActive_) gcCacheFillWordsRemaining_ = 0;
            }
            return;
        }
        if (address == 0x46000000u) {
            gcCacheControl_ = value;
            return;
        }
        if ((address & 3u) == 0 &&
            address >= kGcProcessorRamdacBase &&
            address < kGcProcessorRamdacBase + 0x10u) {
            gcWriteRamdacWord(address, value);
            return;
        }
        if ((address & 3u) == 0 &&
            address >= kGcProcessorDramBase &&
            address <= kGcProcessorDramBase + kGcDramBytes - 4u) {
            gcWriteDramWord(address - kGcProcessorDramBase, value);
            return;
        }
        if ((address & 3u) == 0 &&
            address >= kGcProcessorExpansionDramBase &&
            address <= kGcProcessorExpansionDramBase +
                       kGcExpansionDramBytes - 4u) {
            gcWriteExpansionDramWord(
                address - kGcProcessorExpansionDramBase, value);
            return;
        }
        if ((address & 3u) == 0 &&
            address >= kSuperSlotBase && address <= 0x9FFFFFFFu - 3u) {
            const u64 before = vramWrites;
            gcWriteSuperWord(address, value);
            gcProcessorVramWrites_ += vramWrites - before;
            return;
        }
        if ((address & 3u) == 0 &&
            address >= kSlotBase && address <= kSlotBase + 0x00FFFFFCu) {
            gcWriteStandardWord(address, value);
            return;
        }
        if ((address & 3u) == 0 && address >= kGcVectorBytes &&
            address < 0x08000000u && gcBusMasterWrite32_) {
            gcBusMasterWrite32_(address, value);
            return;
        }
        if ((address & 3u) == 0 &&
            (address & 0xFF000000u) == 0x80000000u &&
            (address & 0x00FFFFFFu) >= kGcVectorBytes &&
            gcBusMasterWrite32_) {
            gcBusMasterWrite32_(address & 0x00FFFFFFu, value);
            return;
        }
        if (inputOutput && (address & 3u) == 0 &&
            (address & 0xFF000000u) == 0xFC000000u &&
            gcBusMasterWrite32_) {
            gcBusMasterWrite32_(address & 0x00FFFFFFu, value);
            return;
        }
        noteUnknownGcDataAddress(address, value, true);
        ++gcUnknownDataWrites_;
    }

    void noteUnknownGcDataAddress(u32 address, u32 value, bool write) {
        if (gcUnknownDataReads_ + gcUnknownDataWrites_ == 0) {
            gcFirstUnknownDataAddress_ = address;
            gcUnknownSnapshotCaptured = true;
            gcUnknownSnapshotAddress = address;
            gcUnknownSnapshotPc = gcCpu_.instructionPc();
            for (std::size_t index = 0;
                 index < gcUnknownRegisterSnapshot.size(); ++index)
                gcUnknownRegisterSnapshot[index] =
                    gcCpu_.registerValue(64u + index);
        }
        if (gcUnknownTraceCount < gcUnknownTraceAddresses.size()) {
            gcUnknownTraceAddresses[gcUnknownTraceCount] = address;
            gcUnknownTracePcs[gcUnknownTraceCount] = gcCpu_.instructionPc();
            gcUnknownTraceValues[gcUnknownTraceCount] = value;
            gcUnknownTraceWrites[gcUnknownTraceCount] = write ? 1u : 0u;
            ++gcUnknownTraceCount;
        }
    }

    bool translateVramAddress(u32 slotOffset, u32& physical,
                              bool& padding) const {
        if (slotOffset < kGcVramOffset) return false;
        const u32 logical = slotOffset - kGcVramOffset;
        if (!packedRgb()) {
            if (logical >= vram_.size()) return false;
            physical = logical;
            padding = false;
            return true;
        }

        const u32 lane = logical & 3u;
        const u64 groupBase = static_cast<u64>(logical >> 2) * 3u;
        if (lane == 0u) {
            if (groupBase >= vram_.size()) return false;
            physical = static_cast<u32>(groupBase);
            padding = true;
            return true;
        }
        const u64 packed = groupBase + lane - 1u;
        if (packed >= vram_.size()) return false;
        physical = static_cast<u32>(packed);
        padding = false;
        return true;
    }

    static void mergeRegister(u32& destination, u32 lane, u8 value) {
        const u32 shift = (3u - lane) * 8u;
        destination = (destination & ~(0xFFu << shift)) | (u32(value) << shift);
    }

    u8 recordGcRead(u32 offset, u8 value) const {
        if (firstGcReadCount < firstGcReadOffsets.size()) {
            firstGcReadOffsets[firstGcReadCount] = offset;
            firstGcReadValues[firstGcReadCount] = value;
            ++firstGcReadCount;
        }
        return value;
    }

    u32 beamPosition(u64) const {
        // The JMFB CRTC exposes its beam state in the low byte.  The GC
        // declaration-ROM driver reads the register as a longword and tests
        // bit 31 while deliberately burning NuBus cycles between samples,
        // waiting for a high -> low -> high vertical timing sequence.  Tie
        // this to emulated 40 MHz time, never host time or read count, so
        // stepping and save-state replay see the same edge.  The default
        // high-resolution display has 480 active lines in a 525-line raster.
        return timingPhase_ < kGcTimingActiveUnits
            ? 0x0000000Eu : 0x00000007u;
    }

    u32 timingPort() const {
        // The GC ROM synchronizes serial programming to the raster by reading
        // this port, echoing it back for a bounded number of NuBus cycles, and
        // testing D0 bit 31 for a high -> low -> high sequence.
        return timingPhase_ < kGcTimingActiveUnits ? 0x80000000u : 0u;
    }

    void updateVblankEnable() {
        vblankEnabled_ = !vblankDisable_ && gcInterruptGate_;
        if (!vblankEnabled_) vblankIrq_ = false;
    }

    void acknowledgeVblank() {
        if (vblankIrq_) ++vblankAcks;
        vblankIrq_ = false;
    }

    void validateGcDeclarationRom() const {
        if ((rom_.size() != 32u * 1024u && rom_.size() != 64u * 1024u) ||
            rom_.size() < 20 || rom_[rom_.size() - 1] != 0xE1u ||
            rom_[rom_.size() - 6] != 0x5Au ||
            rom_[rom_.size() - 5] != 0x93u ||
            rom_[rom_.size() - 4] != 0x2Bu ||
            rom_[rom_.size() - 3] != 0xC7u)
            throw std::invalid_argument(
                "invalid Macintosh Display Card 8*24 GC declaration ROM");

        const std::size_t format = rom_.size() - 20u;
        const u32 length = (u32(rom_[format + 4]) << 24) |
                           (u32(rom_[format + 5]) << 16) |
                           (u32(rom_[format + 6]) << 8) | rom_[format + 7];
        const u32 expected = (u32(rom_[format + 8]) << 24) |
                             (u32(rom_[format + 9]) << 16) |
                             (u32(rom_[format + 10]) << 8) | rom_[format + 11];
        if (length < 20 || length > rom_.size())
            throw std::invalid_argument(
                "8*24 GC declaration ROM has an invalid length");
        u32 crc = 0;
        const std::size_t begin = rom_.size() - length;
        for (std::size_t index = begin; index < rom_.size(); ++index) {
            crc = (crc << 1) | (crc >> 31);
            if (index < format + 8 || index >= format + 12) crc += rom_[index];
        }
        if (crc != expected)
            throw std::invalid_argument(
                "8*24 GC declaration ROM CRC does not match");
    }

    struct RomBuilder {
        struct Fixup { std::size_t entry; std::string target; };
        std::vector<u8> data;
        std::unordered_map<std::string, std::size_t> labels;
        std::vector<Fixup> fixups;

        void label(const char* name) { labels.emplace(name, data.size()); }
        void byte(u8 v) { data.push_back(v); }
        void word(u16 v) { byte(static_cast<u8>(v >> 8)); byte(static_cast<u8>(v)); }
        void longword(u32 v) { word(static_cast<u16>(v >> 16)); word(static_cast<u16>(v)); }
        void cstring(const char* s) {
            while (*s) byte(static_cast<u8>(*s++));
            byte(0);
        }
        void offsetEntry(u8 id, const char* target) {
            const std::size_t at = data.size();
            longword(u32(id) << 24);
            fixups.push_back({at, target});
        }
        void dataEntry(u8 id, u32 value) {
            if (value > 0x00FFFFFFu) throw std::logic_error("sResource data overflow");
            longword((u32(id) << 24) | value);
        }
        void endList() { longword(0xFF000000u); }
        void patchOffsets() {
            for (const Fixup& f : fixups) {
                const auto it = labels.find(f.target);
                if (it == labels.end()) throw std::logic_error("missing declaration-ROM label");
                const std::int64_t delta = static_cast<std::int64_t>(it->second) -
                                           static_cast<std::int64_t>(f.entry);
                if (delta < -0x800000 || delta > 0x7FFFFF)
                    throw std::logic_error("declaration-ROM offset overflow");
                const u32 encoded = static_cast<u32>(delta) & 0x00FFFFFFu;
                data[f.entry + 1] = static_cast<u8>(encoded >> 16);
                data[f.entry + 2] = static_cast<u8>(encoded >> 8);
                data[f.entry + 3] = static_cast<u8>(encoded);
            }
        }
    };

    struct CodeBuilder {
        struct Branch { std::size_t displacement; std::string target; };
        struct PcRelative { std::size_t displacement; std::string target; };
        std::vector<u8> code;
        std::unordered_map<std::string, std::size_t> labels;
        std::vector<Branch> branches;
        std::vector<PcRelative> pcRelative;

        void label(const char* name) { labels.emplace(name, code.size()); }
        void word(u16 v) {
            code.push_back(static_cast<u8>(v >> 8));
            code.push_back(static_cast<u8>(v));
        }
        void longword(u32 v) { word(static_cast<u16>(v >> 16)); word(static_cast<u16>(v)); }
        void branch(u16 opcode, const char* target) {
            word(opcode);
            const std::size_t at = code.size();
            word(0);
            branches.push_back({at, target});
        }
        void pcAddress(u16 opcode, const char* target) {
            word(opcode);
            const std::size_t at = code.size();
            word(0);
            pcRelative.push_back({at, target});
        }
        void patch() {
            for (const Branch& b : branches) {
                const auto it = labels.find(b.target);
                if (it == labels.end()) throw std::logic_error("missing driver label");
                const std::int64_t delta = static_cast<std::int64_t>(it->second) -
                                           static_cast<std::int64_t>(b.displacement + 2);
                if (delta < -32768 || delta > 32767)
                    throw std::logic_error("driver branch overflow");
                const u16 d = static_cast<u16>(static_cast<std::int16_t>(delta));
                code[b.displacement] = static_cast<u8>(d >> 8);
                code[b.displacement + 1] = static_cast<u8>(d);
            }
            for (const PcRelative& p : pcRelative) {
                const auto it = labels.find(p.target);
                if (it == labels.end()) throw std::logic_error("missing driver PC-relative label");
                const std::int64_t delta = static_cast<std::int64_t>(it->second) -
                                           static_cast<std::int64_t>(p.displacement + 2);
                if (delta < -32768 || delta > 32767)
                    throw std::logic_error("driver PC-relative displacement overflow");
                const u16 d = static_cast<u16>(static_cast<std::int16_t>(delta));
                code[p.displacement] = static_cast<u8>(d >> 8);
                code[p.displacement + 1] = static_cast<u8>(d);
            }
        }
    };

    static void emitDriverExit(CodeBuilder& c) {
        c.word(0x0828); c.word(0x0009); c.word(0x0006); // BTST #9,ioTrap(A0)
        c.branch(0x6600, "driver_rts");                 // immediate call: RTS
        c.word(0x2078); c.word(0x08FC);                 // MOVEA.L (jIODone).W,A0
        c.word(0x4ED0);                                 // JMP (A0)
    }

    static std::vector<u8> buildDriver() {
        CodeBuilder c;

        c.word(0x4C00);             // control, status, needs-lock
        c.word(0); c.word(0); c.word(0);
        const std::size_t openOff = c.code.size(); c.word(0);
        c.word(0);                   // no Prime routine
        const std::size_t ctlOff = c.code.size(); c.word(0);
        const std::size_t statOff = c.code.size(); c.word(0);
        const std::size_t closeOff = c.code.size(); c.word(0);
        constexpr char kDriverName[] = ".Display_Video_Apple_TFB";
        c.code.push_back(static_cast<u8>(sizeof(kDriverName) - 1));
        c.code.insert(c.code.end(), kDriverName, kDriverName + sizeof(kDriverName) - 1);
        if (c.code.size() & 1u) c.code.push_back(0);
        c.word(0x0100);

        c.label("open");
        c.word(0x2F08);                              // MOVE.L A0,-(SP)
        c.word(0x2F0A);                              // MOVE.L A2,-(SP)
        c.word(0x2F0B);                              // MOVE.L A3,-(SP)
        c.word(0x2469); c.word(0x002A);             // MOVEA.L dCtlDevBase(A1),A2
        c.word(0x34BC); c.word(0x0080);             // mode $80
        c.word(0x357C); c.word(0); c.word(2);       // page zero
        c.word(0x200A);
        c.word(0x0680); c.longword(kFrameBufferOffset);
        c.word(0x2340); c.word(0x0010);             // dCtlPosition = framebuffer
        c.word(0x337C); c.word(0x0080); c.word(0x0022); // private mode in dCtlDelay
        c.word(0x4269); c.word(0x0024);             // private page in dCtlMask
        c.pcAddress(0x41FA, "slot_queue");          // LEA slot_queue(PC),A0
        c.word(0x4290);                              // CLR.L (A0): qLink
        c.word(0x317C); c.word(6); c.word(4);       // SQType = slot interrupt
        c.word(0x4268); c.word(6);                  // SQPrio = 0
        c.pcAddress(0x47FA, "slot_handler");        // LEA slot_handler(PC),A3
        c.word(0x214B); c.word(8);                  // SQAddr = A3
        c.word(0x264A);                              // MOVEA.L A2,A3
        c.word(0xD7FC); c.longword(kFallbackControlOffset - 0x100u);
        c.word(0x214B); c.word(12);                 // SQParm = CRTC base
        c.word(0x7000);                              // MOVEQ #0,D0
        c.word(0x1029); c.word(0x0028);             // slot byte from DCE
        c.word(0xA075);                              // _SIntInstall
        // Slot Manager traps preserve the documented driver-call registers,
        // not our scratch A3.  Re-form the CRTC address after the trap before
        // enabling VBL; otherwise the returned A3 aliases a system-heap
        // object and the register write silently lands in RAM.
        c.word(0x264A);                              // MOVEA.L A2,A3
        c.word(0xD7FC); c.longword(kFallbackControlOffset - 0x100u);
        c.word(0x42AB); c.word(0x013C);             // enable VBL interrupts
        c.word(0x265F); c.word(0x245F); c.word(0x205F);
        c.word(0x7000); c.word(0x4E75);

        c.label("control");
        c.word(0x2F01); c.word(0x2F0A); c.word(0x2F0B);
        c.word(0x3028); c.word(0x001A);             // csCode
        c.word(0x0C40); c.word(2);
        c.branch(0x6700, "control_set");
        c.word(0x0C40); c.word(9);
        c.branch(0x6300, "control_good");
        c.word(0x70EF);                              // controlErr
        c.branch(0x6000, "control_done");
        c.label("control_set");
        c.word(0x2468); c.word(0x001C);             // video csParam pointer
        c.word(0x2669); c.word(0x002A);             // card base
        c.word(0x3212); c.word(0x3681);             // mode -> card
        c.word(0x3341); c.word(0x0022);             // save mode
        c.word(0x322A); c.word(0x0006);             // page
        c.word(0x3741); c.word(0x0002);             // page -> card
        c.word(0x3341); c.word(0x0024);             // save page
        c.word(0x220B);
        c.word(0x0681); c.longword(kFrameBufferOffset);
        c.word(0x2541); c.word(0x0008);             // csBaseAddr
        c.word(0x2341); c.word(0x0010);             // save base
        c.label("control_good"); c.word(0x7000);
        c.label("control_done");
        c.word(0x265F); c.word(0x245F); c.word(0x221F);
        emitDriverExit(c);

        c.label("status");
        c.word(0x2F01); c.word(0x2F0A); c.word(0x2F0B);
        c.word(0x3028); c.word(0x001A);
        c.word(0x0C40); c.word(2); c.branch(0x6500, "status_bad");
        c.word(0x0C40); c.word(9); c.branch(0x6200, "status_bad");
        c.word(0x2468); c.word(0x001C);             // video csParam pointer
        c.word(0x2669); c.word(0x002A);
        c.word(0x0C40); c.word(2); c.branch(0x6700, "status_mode");
        c.word(0x0C40); c.word(4); c.branch(0x6700, "status_pages");
        c.word(0x0C40); c.word(5); c.branch(0x6700, "status_base");
        c.word(0x0C40); c.word(6); c.branch(0x6700, "status_gray");
        c.word(0x0C40); c.word(7); c.branch(0x6700, "status_interrupt");
        c.word(0x0C40); c.word(8); c.branch(0x6700, "status_gamma");
        c.word(0x0C40); c.word(9); c.branch(0x6700, "status_default");
        c.branch(0x6000, "status_good");             // GetEntries
        c.label("status_mode");
        c.word(0x3229); c.word(0x0022); c.word(0x3481);
        c.word(0x3229); c.word(0x0024); c.word(0x3541); c.word(0x0006);
        c.word(0x2229); c.word(0x0010); c.word(0x2541); c.word(0x0008);
        c.branch(0x6000, "status_good");
        c.label("status_pages");
        c.word(0x3229); c.word(0x0022); c.word(0x3481);
        c.word(0x357C); c.word(1); c.word(0x0006);
        c.branch(0x6000, "status_good");
        c.label("status_base");
        c.word(0x2229); c.word(0x0010); c.word(0x2541); c.word(0x0008);
        c.branch(0x6000, "status_good");
        c.label("status_gray"); c.word(0x4212); c.branch(0x6000, "status_good");
        c.label("status_interrupt"); c.word(0x4212); c.branch(0x6000, "status_good");
        c.label("status_gamma"); c.word(0x4292); c.branch(0x6000, "status_good");
        c.label("status_default"); c.word(0x14BC); c.word(0x0080); c.branch(0x6000, "status_good");
        c.label("status_bad"); c.word(0x70EE); c.branch(0x6000, "status_done");
        c.label("status_good"); c.word(0x7000);
        c.label("status_done");
        c.word(0x265F); c.word(0x245F); c.word(0x221F);
        emitDriverExit(c);

        c.label("close"); c.word(0x7000); c.word(0x4E75);
        c.label("driver_rts"); c.word(0x4E75);

        // Slot interrupt polling routine.  The Slot Manager supplies SQParm
        // in A1.  Clear the card request before calling the system VBL manager
        // exactly as Apple's documented sample and production GC driver do.
        c.label("slot_handler");
        c.word(0x48E7); c.word(0x00C0);             // MOVEM.L A0-A1,-(SP)
        c.word(0x42A9); c.word(0x0148);             // clear slot interrupt
        c.word(0x7009);                              // slot 9
        c.word(0x2078); c.word(0x0D28);             // jVBLTask
        c.word(0x4E90);                              // JSR (A0)
        c.word(0x4CDF); c.word(0x0300);             // MOVEM.L (SP)+,A0-A1
        c.word(0x7001); c.word(0x4E75);             // serviced / RTS
        c.label("slot_queue");
        for (int index = 0; index < 8; ++index) c.word(0);
        c.patch();

        auto putHeaderOffset = [&](std::size_t at, const char* label) {
            const u16 value = static_cast<u16>(c.labels.at(label));
            c.code[at] = static_cast<u8>(value >> 8);
            c.code[at + 1] = static_cast<u8>(value);
        };
        putHeaderOffset(openOff, "open");
        putHeaderOffset(ctlOff, "control");
        putHeaderOffset(statOff, "status");
        putHeaderOffset(closeOff, "close");
        return c.code;
    }

    static void appendVideoParams(RomBuilder& r, int bpp) {
        r.longword(46);               // physical sBlock size
        r.longword(kFrameBufferOffset);
        r.word(static_cast<u16>((kWidth * bpp) / 8));
        r.word(0); r.word(0); r.word(kHeight); r.word(kWidth);
        r.word(1);                    // PixMap version
        r.word(0); r.longword(0);     // packing fields
        r.longword(72u << 16); r.longword(72u << 16);
        r.word(0);                    // chunky indexed
        r.word(static_cast<u16>(bpp));
        r.word(1); r.word(static_cast<u16>(bpp));
        r.longword(0);
    }

    static std::vector<u8> buildDeclarationRom() {
        RomBuilder r;
        r.label("directory");
        r.offsetEntry(1, "board_resource");
        r.offsetEntry(0x80, "video_resource");
        r.endList();

        r.label("board_resource");
        r.offsetEntry(1, "board_type");
        r.offsetEntry(2, "board_name");
        r.dataEntry(32, 1);
        r.endList();

        r.label("video_resource");
        r.offsetEntry(1, "video_type");
        r.offsetEntry(2, "video_name");
        r.offsetEntry(4, "driver_directory");
        // This conventional framebuffer fits in standard slot space, so use
        // Apple's $Fssx_xxxx form that survives both 24- and 32-bit mode.
        r.dataEntry(7, 2);             // fOpenAtStart, 24-bit compatible
        r.dataEntry(8, 1);
        r.offsetEntry(10, "minor_base");
        r.offsetEntry(11, "minor_length");
        r.offsetEntry(0x80, "mode_1_list");
        r.offsetEntry(0x81, "mode_2_list");
        r.offsetEntry(0x82, "mode_4_list");
        r.offsetEntry(0x83, "mode_8_list");
        r.endList();

        r.label("board_type");
        r.word(1); r.word(0); r.word(0); r.word(0);
        r.label("board_name"); r.cstring("Macintosh II Video Card");
        r.label("video_type");
        r.word(3); r.word(1); r.word(1); r.word(1); // Display/Video/Apple/TFB
        r.label("video_name"); r.cstring("Display_Video_Apple_TFB");
        r.label("minor_base"); r.longword(0);
        r.label("minor_length"); r.longword(0x00100000u);

        r.label("driver_directory");
        // The IIfx ROM's cold-start Slot Manager asks for the 68020 entry and
        // then the 68000 fallback. Later System software can ask for the 68030
        // entry. This driver uses the common 68000 instruction subset, so the
        // same sBlock is valid under all three Apple-defined OS IDs.
        r.offsetEntry(1, "driver_block");
        r.offsetEntry(2, "driver_block");
        r.offsetEntry(3, "driver_block");
        r.endList();
        r.label("driver_block");
        const std::vector<u8> driver = buildDriver();
        r.longword(static_cast<u32>(driver.size() + 4));
        r.data.insert(r.data.end(), driver.begin(), driver.end());
        if (r.data.size() & 1u) r.byte(0);

        constexpr std::array<int, 4> depths{1, 2, 4, 8};
        for (int depth : depths) {
            const std::string list = "mode_" + std::to_string(depth) + "_list";
            const std::string params = "mode_" + std::to_string(depth) + "_params";
            r.labels.emplace(list, r.data.size());
            r.offsetEntry(1, params.c_str());
            r.dataEntry(3, 1);
            r.dataEntry(4, 0);
            r.endList();
            r.labels.emplace(params, r.data.size());
            appendVideoParams(r, depth);
        }

        r.patchOffsets();
        constexpr std::size_t kFormatBytes = 20;
        if (r.data.size() > kRomBytes - kFormatBytes)
            throw std::logic_error("IIfx video declaration ROM overflow");
        r.data.resize(kRomBytes - kFormatBytes, 0);
        const std::size_t format = r.data.size();
        const std::int32_t directoryDelta =
            static_cast<std::int32_t>(r.labels.at("directory")) -
            static_cast<std::int32_t>(format);
        // fhDirOffset is a signed 24-bit self-relative value with a required
        // zero marker byte in bits 31..24 (Figure 8-5).  The ROM explicitly
        // rejects a sign-extended 32-bit negative value here.
        r.longword(static_cast<u32>(directoryDelta) & 0x00FFFFFFu);
        r.longword(kRomBytes);
        const std::size_t crcOffset = r.data.size();
        r.longword(0);
        r.byte(1);                      // revision
        r.byte(1);                      // Apple format
        r.longword(0x5A932BC7u);
        r.byte(0);
        r.byte(0x0F);                   // all four NuBus byte lanes

        u32 crc = 0;
        for (std::size_t i = 0; i < r.data.size(); ++i) {
            crc = (crc << 1) | (crc >> 31);
            if (i < crcOffset || i >= crcOffset + 4) crc += r.data[i];
        }
        r.data[crcOffset + 0] = static_cast<u8>(crc >> 24);
        r.data[crcOffset + 1] = static_cast<u8>(crc >> 16);
        r.data[crcOffset + 2] = static_cast<u8>(crc >> 8);
        r.data[crcOffset + 3] = static_cast<u8>(crc);
        return r.data;
    }

    bool genuineGc_ = false;
    std::vector<u8> vram_;
    std::vector<u8> gcDram_;
    std::vector<u8> gcExpansionDram_;
    std::vector<u8> rom_;
    std::array<u8, kGcSramBytes> gcSram_{};
    std::array<u8, kGcVectorBytes> gcVectorRam_{};
    Am29000 gcCpu_;
    std::function<u32(u32)> gcBusMasterRead32_;
    std::function<void(u32, u32)> gcBusMasterWrite32_;
    u16 mode_ = 0x80;
    u16 page_ = 0;
    u16 control_ = 0x0042;
    u16 preload_ = 248;
    u32 base_ = 0;
    u32 stride_ = 20;
    std::array<u32, 4> jmfbShadow_{};
    std::array<u32, 128> crtcShadow_{};
    std::array<u32, 128> gcMfbShadow_{};
    std::array<u32, 256> palette_{};
    std::array<u8, 3> clutColor_{};
    u8 clutAddress_ = 0;
    u8 clutComponent_ = 0;
    u8 ramdacMode_ = 0;
    u8 ramdacControl_ = 0x80;
    bool ramdacConvolution_ = false;
    u32 mfbBaseShift_ = 0;
    u8 mfbBaseBits_ = 0;
    u16 mfbStrideShift_ = 0;
    u8 mfbStrideBits_ = 0;
    u16 serialShift_ = 0;
    u8 serialBits_ = 0;
    u64 serialCommands_ = 0;
    u16 lastSerialCommand_ = 0;
    u64 timingPhase_ = 0;
    u64 gcCpuPhase_ = 0;
    u32 gcResetControl_ = 0;
    u32 gcInterruptControl_ = 0;
    u32 gcMacInterruptControl_ = 0;
    u32 gcCacheControl_ = 0;
    u32 gcCacheFillAddress_ = 0;
    u32 gcCacheFillWordsRemaining_ = 0;
    bool gcHostRequest_ = false;
    bool gcMacRequest_ = false;
    bool gcCacheFillActive_ = false;
    bool gcCpuReleased_ = false;
    u64 gcInstructionReads_ = 0;
    u64 gcDataReads_ = 0;
    u64 gcDataWrites_ = 0;
    u64 gcProcessorVramWrites_ = 0;
    u64 gcUnknownDataReads_ = 0;
    u64 gcUnknownDataWrites_ = 0;
    u32 gcFirstUnknownDataAddress_ = 0;
    u64 timingEchoWrites_ = 0;
    mutable bool timingEchoPending_ = false;
    mutable u32 timingEchoValue_ = 0;
    mutable u8 timingEchoLane_ = 0;
    bool gcInterruptGate_ = false;
    bool vblankDisable_ = true;
    bool vblankEnabled_ = false;
    bool vblankIrq_ = false;
    u64 frameVramWrites_ = 0;
};

} // namespace openmac
