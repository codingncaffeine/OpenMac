#pragma once

// Apple GCR (group-coded recording) track codec for 400K/800K Macintosh floppies.
//
// The IWM is a dumb byte pump: it hands the CPU whatever eight-bit "disk bytes"
// pass under the head, and the ROM's own .Sony driver does the mark matching,
// 6-and-2 decoding and checksumming in software. So to make a real disk image
// readable we have to lay out an actual nibble stream, byte for byte, the way a
// real formatter would.
//
// Track layout (one sector), per the SWIM Chip User's Reference pp.5-6:
//
//   36 x $FF self-sync
//   $D5 $AA $96                          address prologue
//   track(low) sector side/track-high format checksum    each one 6-and-2 byte
//   $DE $AA $FF                          address epilogue + pad
//   5 x $FF self-sync
//   $D5 $AA $AD                          data prologue
//   sector                               6-and-2 byte, duplicate of the header's
//   699 bytes                            12 tag + 512 data, nibblized
//   4 bytes                              24-bit checksum, nibblized
//   $DE $AA $FF                          data epilogue + pad
//
// Geometry is zoned: 16 tracks per zone, 12/11/10/9/8 sectors per track from the
// outside in, 800 sectors per side, so a double-sided disk holds 1600 x 512 =
// 819200 bytes. Sectors are laid down 2:1 interleaved.
//
// The 64-entry disk-byte alphabet below is the one the Classic ROM itself
// carries at $4359D8 -- every value has bit 7 set and no more than one pair of
// adjacent zero bits, which is what makes the stream self-clocking and lets the
// controller find byte boundaries.
//
// Reference: SWIM Chip User's Reference Rev 1.5 pp.5-6; Apple 699-0285-A drive
// specification (sector layout and interleave); Inside Macintosh Vol. II/III.
// Clean-room from the specs, cross-checked against the ROM's own table.

#include "openmac/types.hpp"

#include <cstddef>
#include <vector>

namespace openmac::gcr {

// 6-bit value -> disk byte. Identical to the ROM's table at $4359D8.
inline const u8* encodeTable() {
    static const u8 kTable[64] = {
        0x96, 0x97, 0x9A, 0x9B, 0x9D, 0x9E, 0x9F, 0xA6,
        0xA7, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB2, 0xB3,
        0xB4, 0xB5, 0xB6, 0xB7, 0xB9, 0xBA, 0xBB, 0xBC,
        0xBD, 0xBE, 0xBF, 0xCB, 0xCD, 0xCE, 0xCF, 0xD3,
        0xD6, 0xD7, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE,
        0xDF, 0xE5, 0xE6, 0xE7, 0xE9, 0xEA, 0xEB, 0xEC,
        0xED, 0xEE, 0xEF, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6,
        0xF7, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF,
    };
    return kTable;
}

// Disk byte -> 6-bit value, 0xFF for values that are not valid disk bytes.
inline const u8* decodeTable() {
    static u8 kInv[256];
    static bool built = false;
    if (!built) {
        for (int i = 0; i < 256; ++i) kInv[i] = 0xFF;
        for (int i = 0; i < 64; ++i) kInv[encodeTable()[i]] = static_cast<u8>(i);
        built = true;
    }
    return kInv;
}

// ---- geometry ----------------------------------------------------------

inline int sectorsOnTrack(int track) {
    static const int kSec[5] = {12, 11, 10, 9, 8};
    int z = track / 16;
    if (z < 0) z = 0;
    if (z > 4) z = 4;
    return kSec[z];
}

// Logical sectors preceding `track` on one side.
inline int trackStartSector(int track) {
    int n = 0;
    for (int t = 0; t < track; ++t) n += sectorsOnTrack(t);
    return n;
}

inline int sectorsPerSide() { return trackStartSector(80); }   // 800

// 2:1 interleave: place logical sector i at physical slot (i*2) mod n, stepping
// forward whenever the slot is taken (Apple 699-0285-A).
inline void interleaveOrder(int n, int* physicalOfLogical) {
    std::vector<int> slot(static_cast<std::size_t>(n), -1);
    int p = 0;
    for (int i = 0; i < n; ++i) {
        while (slot[static_cast<std::size_t>(p)] >= 0) p = (p + 1) % n;
        slot[static_cast<std::size_t>(p)] = i;
        p = (p + 2) % n;
    }
    for (int s = 0; s < n; ++s) physicalOfLogical[slot[static_cast<std::size_t>(s)]] = s;
}

// ---- the data-field scramble and checksum -----------------------------
//
// Three source bytes at a time, each XORed with a rolling accumulator and each
// accumulator carried into the next. The three accumulators become the sector's
// 24-bit checksum. This is the one part of the format the primary documentation
// renders ambiguously (the SWIM reference prints encode and decode side by side
// in two columns that every surviving scan garbles), so the implementation here
// is defined by its round trip: decodeData is the exact inverse of encodeData,
// and the unit test drives real sector data through both.

struct Checksum { u8 a, b, c; };

// Scramble `in` (524 bytes) into `out`, returning the 24-bit checksum.
inline Checksum encodeData(const u8* in, std::size_t n, u8* out) {
    u8 a = 0, b = 0, c = 0;
    unsigned carry = 0;
    for (std::size_t i = 0; i < n; i += 3) {
        const u8 v0 = in[i];
        const u8 v1 = (i + 1 < n) ? in[i + 1] : 0;
        const u8 v2 = (i + 2 < n) ? in[i + 2] : 0;
        // Rotate C left through the saved carry, then scramble the group.
        const unsigned newCarry = (c >> 7) & 1u;
        c = static_cast<u8>((c << 1) | carry);
        carry = newCarry;

        unsigned s = static_cast<unsigned>(a) + v0 + carry;
        a = static_cast<u8>(s);
        carry = (s > 0xFF) ? 1u : 0u;
        out[i] = static_cast<u8>(v0 ^ c);

        s = static_cast<unsigned>(b) + v1 + carry;
        b = static_cast<u8>(s);
        carry = (s > 0xFF) ? 1u : 0u;
        if (i + 1 < n) out[i + 1] = static_cast<u8>(v1 ^ a);

        s = static_cast<unsigned>(c) + v2 + carry;
        c = static_cast<u8>(s);
        carry = (s > 0xFF) ? 1u : 0u;
        if (i + 2 < n) out[i + 2] = static_cast<u8>(v2 ^ b);
    }
    return {a, b, c};
}

// Exact inverse of encodeData.
inline Checksum decodeData(const u8* in, std::size_t n, u8* out) {
    u8 a = 0, b = 0, c = 0;
    unsigned carry = 0;
    for (std::size_t i = 0; i < n; i += 3) {
        const unsigned newCarry = (c >> 7) & 1u;
        c = static_cast<u8>((c << 1) | carry);
        carry = newCarry;

        const u8 v0 = static_cast<u8>(in[i] ^ c);
        unsigned s = static_cast<unsigned>(a) + v0 + carry;
        a = static_cast<u8>(s);
        carry = (s > 0xFF) ? 1u : 0u;
        out[i] = v0;

        const u8 v1 = (i + 1 < n) ? static_cast<u8>(in[i + 1] ^ a) : 0;
        s = static_cast<unsigned>(b) + v1 + carry;
        b = static_cast<u8>(s);
        carry = (s > 0xFF) ? 1u : 0u;
        if (i + 1 < n) out[i + 1] = v1;

        const u8 v2 = (i + 2 < n) ? static_cast<u8>(in[i + 2] ^ b) : 0;
        s = static_cast<unsigned>(c) + v2 + carry;
        c = static_cast<u8>(s);
        carry = (s > 0xFF) ? 1u : 0u;
        if (i + 2 < n) out[i + 2] = v2;
    }
    return {a, b, c};
}

// ---- 6-and-2 nibblization ---------------------------------------------
//
// Three scrambled bytes become four disk bytes: one gathering the low two bits
// of each (bit-reversed into a 6-bit field), then the high six bits of each.

inline void nibblize(const u8* in, std::size_t n, std::vector<u8>& out) {
    const u8* enc = encodeTable();
    for (std::size_t i = 0; i < n; i += 3) {
        const u8 v0 = in[i];
        const u8 v1 = (i + 1 < n) ? in[i + 1] : 0;
        const u8 v2 = (i + 2 < n) ? in[i + 2] : 0;
        const u8 lo = static_cast<u8>(((v0 & 3) << 4) | ((v1 & 3) << 2) | (v2 & 3));
        out.push_back(enc[lo & 0x3F]);
        out.push_back(enc[(v0 >> 2) & 0x3F]);
        if (i + 1 < n) out.push_back(enc[(v1 >> 2) & 0x3F]);
        if (i + 2 < n) out.push_back(enc[(v2 >> 2) & 0x3F]);
    }
}

// Number of disk bytes nibblize() produces for n source bytes. The last group
// is short when n is not a multiple of three (524 is not), and carries only as
// many high-bit bytes as it has source bytes.
inline std::size_t nibblizedSize(std::size_t n) {
    const std::size_t full = n / 3, rem = n % 3;
    return full * 4 + (rem ? rem + 1 : 0);
}

// Inverse of nibblize. Returns false if a byte is not a valid disk byte.
inline bool denibblize(const u8* in, u8* out, std::size_t n) {
    const u8* dec = decodeTable();
    std::size_t src = 0, dst = 0;
    while (dst < n) {
        const u8 lo = dec[in[src++]];
        if (lo == 0xFF) return false;
        const std::size_t take = (n - dst) < 3 ? (n - dst) : 3;
        for (std::size_t k = 0; k < take; ++k) {
            const u8 hi = dec[in[src++]];
            if (hi == 0xFF) return false;
            const u8 bits = static_cast<u8>((lo >> (4 - 2 * k)) & 3);
            out[dst++] = static_cast<u8>((hi << 2) | bits);
        }
    }
    return true;
}

// ---- track assembly ----------------------------------------------------

constexpr u8 kSync = 0xFF;
constexpr std::size_t kTagBytes = 12;
constexpr std::size_t kSectorBytes = 512;
constexpr std::size_t kPayload = kTagBytes + kSectorBytes;   // 524

// Build the complete nibble stream for one track of one side. `image` is a raw
// linear 400K/800K volume; `tags` may be null, in which case the tag bytes are
// zero. `format` is the disk's format byte ($22 for a Mac 800K disk).
inline std::vector<u8> buildTrack(const std::vector<u8>& image, int track, int side,
                                  int sides, u8 format) {
    const int n = sectorsOnTrack(track);
    std::vector<int> physOf(static_cast<std::size_t>(n));
    interleaveOrder(n, physOf.data());

    // Sides are interleaved per track in a linear image: this track's side-0
    // sectors, then its side-1 sectors, then the next track.
    const int start = trackStartSector(track) * sides + side * n;

    std::vector<u8> trk;
    trk.reserve(static_cast<std::size_t>(n) * 800 + 64);

    std::vector<int> logicalOf(static_cast<std::size_t>(n));
    for (int lg = 0; lg < n; ++lg) logicalOf[static_cast<std::size_t>(physOf[static_cast<std::size_t>(lg)])] = lg;

    for (int phys = 0; phys < n; ++phys) {
        const int sector = logicalOf[static_cast<std::size_t>(phys)];
        const u8 trkLow = static_cast<u8>(track & 0x3F);
        const u8 sideHi = static_cast<u8>((side ? 0x20 : 0x00) | (track >= 64 ? 1 : 0));
        const u8 csum = static_cast<u8>((trkLow ^ sector ^ sideHi ^ format) & 0x3F);
        const u8* enc = encodeTable();

        for (int i = 0; i < 36; ++i) trk.push_back(kSync);
        trk.push_back(0xD5); trk.push_back(0xAA); trk.push_back(0x96);
        trk.push_back(enc[trkLow & 0x3F]);
        trk.push_back(enc[sector & 0x3F]);
        trk.push_back(enc[sideHi & 0x3F]);
        trk.push_back(enc[format & 0x3F]);
        trk.push_back(enc[csum & 0x3F]);
        trk.push_back(0xDE); trk.push_back(0xAA); trk.push_back(0xFF);

        for (int i = 0; i < 5; ++i) trk.push_back(kSync);
        trk.push_back(0xD5); trk.push_back(0xAA); trk.push_back(0xAD);
        trk.push_back(enc[sector & 0x3F]);

        u8 raw[kPayload] = {};
        const std::size_t off = static_cast<std::size_t>(start + sector) * kSectorBytes;
        if (off + kSectorBytes <= image.size())
            std::copy(image.begin() + static_cast<std::ptrdiff_t>(off),
                      image.begin() + static_cast<std::ptrdiff_t>(off + kSectorBytes),
                      raw + kTagBytes);
        u8 scrambled[kPayload];
        const Checksum ck = encodeData(raw, kPayload, scrambled);
        nibblize(scrambled, kPayload, trk);
        // The 24-bit checksum rides in four disk bytes: the gathered low bits
        // first, then the high six of each accumulator.
        trk.push_back(enc[(((ck.a & 3) << 4) | ((ck.b & 3) << 2) | (ck.c & 3)) & 0x3F]);
        trk.push_back(enc[(ck.a >> 2) & 0x3F]);
        trk.push_back(enc[(ck.b >> 2) & 0x3F]);
        trk.push_back(enc[(ck.c >> 2) & 0x3F]);
        trk.push_back(0xDE); trk.push_back(0xAA); trk.push_back(0xFF);
    }
    return trk;
}

// Pull every sector back out of a nibble stream, writing 512-byte sectors into
// `image` at their logical positions. Returns how many sectors decoded cleanly.
inline int decodeTrack(const std::vector<u8>& trk, std::vector<u8>& image,
                       int track, int sides) {
    const u8* dec = decodeTable();
    const int n = sectorsOnTrack(track);
    const int before = trackStartSector(track) * sides;
    int found = 0;
    std::size_t i = 0;
    const std::size_t len = trk.size();
    int curSector = -1, curSide = 0;

    while (i + 3 < len) {
        if (trk[i] == 0xD5 && trk[i + 1] == 0xAA && trk[i + 2] == 0x96 && i + 8 < len) {
            const u8 t  = dec[trk[i + 3]];
            const u8 s  = dec[trk[i + 4]];
            const u8 sh = dec[trk[i + 5]];
            const u8 f  = dec[trk[i + 6]];
            const u8 ck = dec[trk[i + 7]];
            if (t != 0xFF && s != 0xFF && sh != 0xFF && f != 0xFF && ck != 0xFF &&
                ((t ^ s ^ sh ^ f) & 0x3F) == ck) {
                curSector = s;
                curSide = (sh & 0x20) ? 1 : 0;
            }
            i += 8;
            continue;
        }
        const std::size_t nibs = nibblizedSize(kPayload);
        if (trk[i] == 0xD5 && trk[i + 1] == 0xAA && trk[i + 2] == 0xAD && curSector >= 0 &&
            i + 4 + nibs + 4 <= len) {
            const std::size_t p = i + 4;   // skip prologue + sector byte
            u8 scrambled[kPayload];
            if (denibblize(&trk[p], scrambled, kPayload)) {
                u8 raw[kPayload];
                const Checksum ck = decodeData(scrambled, kPayload, raw);
                const std::size_t cp = p + nibs;
                const u8 lo = dec[trk[cp]], ha = dec[trk[cp + 1]],
                         hb = dec[trk[cp + 2]], hc = dec[trk[cp + 3]];
                if (lo != 0xFF && ha != 0xFF && hb != 0xFF && hc != 0xFF) {
                    const u8 a = static_cast<u8>((ha << 2) | ((lo >> 4) & 3));
                    const u8 b = static_cast<u8>((hb << 2) | ((lo >> 2) & 3));
                    const u8 c = static_cast<u8>((hc << 2) | (lo & 3));
                    if (a == ck.a && b == ck.b && c == ck.c && curSector < n) {
                        const std::size_t off =
                            static_cast<std::size_t>(before + curSide * n + curSector) * kSectorBytes;
                        if (off + kSectorBytes <= image.size())
                            std::copy(raw + kTagBytes, raw + kPayload,
                                      image.begin() + static_cast<std::ptrdiff_t>(off));
                        ++found;
                    }
                }
            }
            i += 4 + nibs + 4;
            curSector = -1;
            continue;
        }
        ++i;
    }
    return found;
}

} // namespace openmac::gcr
