#pragma once

// The memory controller of the Macintosh Quadra 950 (and of the 900 and 700,
// which share its decoder): four banks of RAM placed in the low 256 MB by
// three programmable boundaries.
//
// No published document describes the registers; what follows is what the
// 950's ROM does with them (hardware dossier, "Memory controller").
//
//   * Registers are longwords at +$00, +$04, ... and ONE BIT wide. The ROM
//     loads a value by writing it to the first register, shifting it right
//     one place, writing it to the next, and so on.
//   * Registers 0-17 are three six-bit boundaries in units of 4 MB, least
//     significant bit first: the end of bank A, of bank B, of bank C. Bank D
//     runs from the third boundary to the top of the 256 MB.
//   * Registers 18-31 are other settings (refresh, timing). They are kept and
//     read back, and have no effect here.
//   * +$80..+$BC are written during setup and after each boundary; they are
//     kept and have no effect here.
//
// A bank answers anywhere inside its range at (address modulo its size): the
// ROM sizes memory by giving every bank a 64 MB range and looking for where
// its contents repeat. An empty bank drives nothing, so a read there never
// returns what was just written.

#include "openmac/types.hpp"

#include <array>
#include <cstddef>

namespace openmac {

class MemoryController950 {
    friend class QuadraStateCodec;
public:
    static constexpr u32 kSpaceBytes = 0x10000000u;   // 256 MB of RAM space
    static constexpr u32 kUnitShift = 22;             // boundaries count 4 MB
    static constexpr u32 kNoRam = 0xFFFFFFFFu;
    static constexpr u8 kFloatingBus = 0xFF;          // what an empty bank reads as

    // Bank sizes in megabytes: 0 (empty) or 4, 8, 16, 32, 64.
    static bool validBankMb(u32 mb) {
        return mb == 0 || mb == 4 || mb == 8 || mb == 16 || mb == 32 || mb == 64;
    }

    explicit MemoryController950(const std::array<u32, 4>& bankMb = {4, 4, 0, 0}) {
        configure(bankMb);
    }

    // Fit the SIMMs. Bank k's bytes sit in one flat array, banks back to
    // back in A-D order; flatBytes() is the size to give that array.
    void configure(const std::array<u32, 4>& bankMb) {
        u32 base = 0;
        for (int k = 0; k < 4; ++k) {
            bankUnits_[k] = validBankMb(bankMb[static_cast<std::size_t>(k)])
                ? bankMb[static_cast<std::size_t>(k)] / 4 : 0;
            bankBase_[k] = base;
            base += bankUnits_[k];
        }
        flatUnits_ = base;
        reset();
    }
    u32 flatBytes() const { return flatUnits_ << kUnitShift; }
    u32 bankBytes(int k) const { return bankUnits_[k] << kUnitShift; }

    // Power on. The registers' real reset values are not known; the ROM
    // writes all of them before it touches RAM.
    void reset() {
        bits_.fill(0);
        other_.fill(0);
        rebuild();
    }

    // Register window, offsets from the controller's base.
    u32 read(u32 off) const {
        off &= 0xFCu;
        if (off < 0x80) return bits_[off >> 2];
        return other_[(off - 0x80) >> 2];
    }
    void write(u32 off, u32 value) {
        off &= 0xFCu;
        if (off < 0x80) {
            const u32 index = off >> 2;
            const u8 bit = static_cast<u8>(value & 1);
            if (bits_[index] == bit) return;
            bits_[index] = bit;
            if (index < 18) rebuild();
        } else {
            other_[(off - 0x80) >> 2] = value;
        }
    }

    // Boundary k (0-2), in bytes.
    u32 boundaryBytes(int k) const {
        u32 units = 0;
        for (int i = 0; i < 6; ++i) units |= static_cast<u32>(bits_[static_cast<std::size_t>(k * 6 + i)]) << i;
        return units << kUnitShift;
    }

    // Where a RAM-space address lives in the flat array, or kNoRam for an
    // empty bank. addr must be below kSpaceBytes.
    u32 translate(u32 addr) const {
        const u32 unit = unitMap_[addr >> kUnitShift];
        if (unit == kNoRam) return kNoRam;
        return (unit << kUnitShift) | (addr & ((1u << kUnitShift) - 1));
    }

    // True once the boundaries put every fitted byte at its own address from
    // zero up, so that address == flat index and the board can skip translate.
    bool identityMapped() const { return identity_; }

private:
    void rebuild() {
        const u32 b[3] = {boundaryBytes(0) >> kUnitShift, boundaryBytes(1) >> kUnitShift,
                          boundaryBytes(2) >> kUnitShift};
        identity_ = true;
        for (u32 u = 0; u < 64; ++u) {
            const int k = u < b[0] ? 0 : u < b[1] ? 1 : u < b[2] ? 2 : 3;
            if (bankUnits_[k] == 0) {
                unitMap_[u] = kNoRam;
            } else {
                unitMap_[u] = bankBase_[k] + (u & (bankUnits_[k] - 1));
            }
            // Identity: the first flatUnits_ units map to themselves.
            if (u < flatUnits_ && unitMap_[u] != u) identity_ = false;
        }
    }

    std::array<u8, 32> bits_{};
    std::array<u32, 16> other_{};
    u32 bankUnits_[4] = {0, 0, 0, 0};   // size of each bank, in 4 MB units
    u32 bankBase_[4] = {0, 0, 0, 0};    // its place in the flat array, in units
    u32 flatUnits_ = 0;
    std::array<u32, 64> unitMap_{};
    bool identity_ = false;
};

} // namespace openmac
