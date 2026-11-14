#pragma once

// Builders for a NuBus declaration ROM: the resource directory with its
// self-relative offsets and the format block that ends the ROM, and 68000
// machine code with branch and PC-relative fixups for the routines such a ROM
// carries (the start-up routine and the driver).
//
// Reference: Designing Cards and Drivers for the Macintosh Family, 3rd ed.,
// ch. 8 (the declaration ROM's layout, the format block, the checksum).
// Clean-room.

#include "openmac/types.hpp"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace openmac::declrom {

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
            // A 68k displacement counts from the address of the
            // displacement word itself, not from the word after it.
            const std::int64_t delta = static_cast<std::int64_t>(it->second) -
                                       static_cast<std::int64_t>(b.displacement);
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
                                       static_cast<std::int64_t>(p.displacement);
            if (delta < -32768 || delta > 32767)
                throw std::logic_error("driver PC-relative displacement overflow");
            const u16 d = static_cast<u16>(static_cast<std::int16_t>(delta));
            code[p.displacement] = static_cast<u8>(d >> 8);
            code[p.displacement + 1] = static_cast<u8>(d);
        }
    }
};

// Ends a declaration ROM of `romBytes` bytes: pads the builder's data to the
// format block's place, then writes the block (the offset to the directory,
// the length, the checksum, revision 1, Apple's format, the test pattern and
// the byte lanes) with the checksum of the whole image.
inline void finish(RomBuilder& r, std::size_t romBytes, u8 byteLanes) {
    constexpr std::size_t kFormatBytes = 20;
    r.patchOffsets();
    if (r.data.size() > romBytes - kFormatBytes) throw std::logic_error("declaration ROM overflow");
    r.data.resize(romBytes - kFormatBytes, 0);
    const std::size_t format = r.data.size();
    // The offset is 24 bits, self-relative, under a zero byte.
    const std::int32_t directoryDelta =
        static_cast<std::int32_t>(r.labels.at("directory")) - static_cast<std::int32_t>(format);
    r.longword(static_cast<u32>(directoryDelta) & 0x00FFFFFFu);
    r.longword(static_cast<u32>(romBytes));
    const std::size_t crcOffset = r.data.size();
    r.longword(0);
    r.byte(1);                      // revision
    r.byte(1);                      // Apple's format
    r.longword(0x5A932BC7u);
    r.byte(0);
    r.byte(byteLanes);
    u32 crc = 0;
    for (std::size_t i = 0; i < r.data.size(); ++i) {
        crc = (crc << 1) | (crc >> 31);
        if (i < crcOffset || i >= crcOffset + 4) crc += r.data[i];
    }
    r.data[crcOffset + 0] = static_cast<u8>(crc >> 24);
    r.data[crcOffset + 1] = static_cast<u8>(crc >> 16);
    r.data[crcOffset + 2] = static_cast<u8>(crc >> 8);
    r.data[crcOffset + 3] = static_cast<u8>(crc);
}

} // namespace openmac::declrom
