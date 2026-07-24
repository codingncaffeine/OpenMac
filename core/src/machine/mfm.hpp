#pragma once

// MFM track codec for 1.4 MB (high-density) Macintosh floppies.
//
// Where GCR leaves everything to software, MFM is framed by the SWIM itself:
// the chip finds address marks, hands the CPU whole bytes and runs the CRC, so
// what has to be modelled is a byte stream plus the one thing a byte cannot
// carry -- which of those bytes are *mark* bytes, written with a transition
// missing between two adjacent zero bits so that they cannot occur in ordinary
// data. The Handshake register's bit 0 reports "the next byte is a mark", and
// reading a mark through the Data register rather than the Mark register is an
// error, so the flags are as much a part of the medium as the bytes.
//
// Track layout is the usual System 34 double-density format the Mac shares with
// everyone else, at 500 kbit/s and 300 rpm -- 12500 bytes to a revolution, 18
// sectors of 512:
//
//   80 x $4E                                  gap 4a
//   12 x $00 . 3 x $C2(mark) . $FC            index address mark
//   50 x $4E                                  gap 1
//   per sector:
//     12 x $00 . 3 x $A1(mark) . $FE          ID address mark
//     track . side . sector . size($02)       sector id, sector numbered from 1
//     CRC16                                   over the marks and the id
//     22 x $4E . 12 x $00                     gap 2
//     3 x $A1(mark) . $FB                     data address mark
//     512 data bytes . CRC16                  over the marks and the data
//     84 x $4E                                gap 3
//   $4E to the end of the revolution          gap 4b
//
// The ROM carries the `A1 A1 A1 FE` pattern at $435D94 and reads exactly four
// mark-register bytes before the sector id (SWIM ref p.32), which is the shape
// this produces.
//
// Reference: SWIM Chip User's Reference Rev 1.5 pp.9, 19-25, 32 (CRC generator,
// mark bytes, and the read sequence); Macintosh Classic Developer Note p.5.
// Clean-room from the specs.

#include "openmac/types.hpp"

#include <cstddef>
#include <vector>

namespace openmac::mfm {

constexpr int kSectorsPerTrack = 18;
constexpr int kTracks = 80;
constexpr int kSides = 2;
constexpr std::size_t kSectorBytes = 512;
constexpr std::size_t kTrackBytes = 12500;      // 500 kbit/s at 300 rpm
constexpr std::size_t kImageBytes =
    static_cast<std::size_t>(kTracks) * kSides * kSectorsPerTrack * kSectorBytes;   // 1474560

// CCITT-16, G(x) = x^16 + x^12 + x^5 + 1, initialised to all ones; the CRC over
// a field and its own two CRC bytes comes out zero (SWIM ref p.9).
inline u16 crc16(const u8* p, std::size_t n, u16 crc = 0xFFFF) {
    for (std::size_t i = 0; i < n; ++i) {
        crc ^= static_cast<u16>(static_cast<u16>(p[i]) << 8);
        for (int b = 0; b < 8; ++b)
            crc = static_cast<u16>((crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1));
    }
    return crc;
}

inline u16 crc16Update(u16 crc, u8 byte) { return crc16(&byte, 1, crc); }

// Logical block of a (track, side, sector) triple. Sectors are numbered from 1.
inline std::size_t blockOf(int track, int side, int sector) {
    return (static_cast<std::size_t>(track) * kSides + static_cast<std::size_t>(side)) *
               kSectorsPerTrack + static_cast<std::size_t>(sector - 1);
}

// Build one track. `out` receives the byte stream and `marks` a flag per byte.
inline void buildTrack(const std::vector<u8>& image, int track, int side,
                       std::vector<u8>& out, std::vector<u8>& marks) {
    out.clear();
    marks.clear();
    out.reserve(kTrackBytes);
    marks.reserve(kTrackBytes);
    auto put = [&](u8 b, bool mark = false) { out.push_back(b); marks.push_back(mark ? 1 : 0); };
    auto fill = [&](std::size_t n, u8 b) { for (std::size_t i = 0; i < n; ++i) put(b); };

    fill(80, 0x4E);                                   // gap 4a
    fill(12, 0x00);
    for (int i = 0; i < 3; ++i) put(0xC2, true);      // index mark
    put(0xFC);
    fill(50, 0x4E);                                   // gap 1

    for (int s = 1; s <= kSectorsPerTrack; ++s) {
        fill(12, 0x00);
        u8 id[8] = {0xA1, 0xA1, 0xA1, 0xFE,
                    static_cast<u8>(track), static_cast<u8>(side),
                    static_cast<u8>(s), 0x02};
        for (int i = 0; i < 3; ++i) put(0xA1, true);
        for (int i = 3; i < 8; ++i) put(id[i]);
        const u16 idCrc = crc16(id, 8);
        put(static_cast<u8>(idCrc >> 8));
        put(static_cast<u8>(idCrc));

        fill(22, 0x4E);                               // gap 2
        fill(12, 0x00);
        for (int i = 0; i < 3; ++i) put(0xA1, true);
        put(0xFB);                                    // data address mark

        const std::size_t off = blockOf(track, side, s) * kSectorBytes;
        u8 head[4] = {0xA1, 0xA1, 0xA1, 0xFB};
        u16 crc = crc16(head, 4);
        for (std::size_t i = 0; i < kSectorBytes; ++i) {
            const u8 b = (off + i < image.size()) ? image[off + i] : 0;
            put(b);
            crc = crc16Update(crc, b);
        }
        put(static_cast<u8>(crc >> 8));
        put(static_cast<u8>(crc));

        fill(84, 0x4E);                               // gap 3
    }
    while (out.size() < kTrackBytes) put(0x4E);       // gap 4b
}

// Pull every sector whose marks, id and CRCs check out back into `image`, the
// way a drive would if it re-read what was just written. Returns the count.
inline int decodeTrack(const std::vector<u8>& trk, const std::vector<u8>& marks,
                       std::vector<u8>& image, int track, int side) {
    if (trk.size() != marks.size() || trk.size() < 64) return 0;
    const std::size_t n = trk.size();
    auto at = [&](std::size_t i) { return trk[i % n]; };
    auto isMark = [&](std::size_t i) { return marks[i % n] != 0; };

    int found = 0;
    for (std::size_t i = 0; i < n; ++i) {
        // An ID address mark: three mark bytes then $FE.
        if (!(isMark(i) && isMark(i + 1) && isMark(i + 2))) continue;
        if (!(at(i) == 0xA1 && at(i + 1) == 0xA1 && at(i + 2) == 0xA1)) continue;
        if (at(i + 3) != 0xFE) continue;
        u8 id[8];
        for (int k = 0; k < 8; ++k) id[k] = at(i + static_cast<std::size_t>(k));
        const u16 want = static_cast<u16>((at(i + 8) << 8) | at(i + 9));
        if (crc16(id, 8) != want) continue;
        const int sector = id[6];
        if (id[4] != static_cast<u8>(track) || id[5] != static_cast<u8>(side)) continue;
        if (sector < 1 || sector > kSectorsPerTrack || id[7] != 0x02) continue;

        // The data address mark follows within gap 2 plus its sync field.
        std::size_t j = i + 10;
        const std::size_t limit = j + 64;
        for (; j < limit; ++j) {
            if (isMark(j) && isMark(j + 1) && isMark(j + 2) && at(j) == 0xA1 &&
                at(j + 1) == 0xA1 && at(j + 2) == 0xA1 && at(j + 3) == 0xFB)
                break;
        }
        if (j >= limit) continue;

        u8 head[4] = {at(j), at(j + 1), at(j + 2), at(j + 3)};
        u16 crc = crc16(head, 4);
        const std::size_t data = j + 4;
        for (std::size_t k = 0; k < kSectorBytes; ++k) crc = crc16Update(crc, at(data + k));
        const u16 dwant = static_cast<u16>((at(data + kSectorBytes) << 8) |
                                            at(data + kSectorBytes + 1));
        if (crc != dwant) continue;

        const std::size_t off = blockOf(track, side, sector) * kSectorBytes;
        if (off + kSectorBytes <= image.size())
            for (std::size_t k = 0; k < kSectorBytes; ++k) image[off + k] = at(data + k);
        ++found;
    }
    return found;
}

} // namespace openmac::mfm
