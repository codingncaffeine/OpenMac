#pragma once

// CD-ROM media containers. The drive serves a flat byte run of 2048-byte user
// sectors (or 512-byte device blocks for a Mac master); the image files people
// actually have wrap that run a few ways:
//
//   .iso .cdr .toast .img  2048-byte user sectors: ISO 9660 ('CD001' at sector
//                          16 + 1) or High Sierra ('CDROM' at 16 + 9), an
//                          Apple-partitioned master ('ER' at 0) or a bare HFS
//                          master ('BD' at byte 1024). Disk Utility's .cdr and
//                          most Toast images are exactly this.
//   .bin (+.cue) .mdf .img raw sectors as the laser sees them: 12-byte sync,
//                          4-byte header, then the 2048 user bytes at +16
//                          (mode 1) or +24 (mode 2, behind the XA subheader),
//                          then error correction; 2352 bytes each, 2448 with
//                          subchannel data; recognised by the sync mark itself,
//                          so a renamed image still opens. 2336-byte MODE2
//                          images (subheader + user data, no sync) are found
//                          by the layout behind that framing.
//   .dmg                   Disk Utility's UDIF: a 512-byte-sector device image
//                          cut into raw / zero-filled / zlib chunks behind an
//                          XML chunk table and a 'koly' trailer; decoded whole
//                          (bzip2 / ADC / LZFSE chunks are refused by name).
//
// normalize() peels the framing and passes the flat run through, naming what
// it saw -- or why it refused -- in words meant for the person who picked the
// file. Detection is by content: UDIF trailer, sync mark, partition map, ISO
// descriptor, HFS signature, then the 2336 layout. Audio tracks in a .cue are
// a later phase; the data track is what mounts software.

#include "openmac/types.hpp"

#include <cstring>
#include <string>
#include <vector>

namespace openmac::cd {

struct Medium {
    std::vector<u8> data;      // flat bytes as the drive will serve them
    char desc[240] = {0};      // what it is (accepted) or why not (refused)
    bool ok = false;
};

inline bool hasRawSync(const std::vector<u8>& f, std::size_t off) {
    static const u8 sync[12] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    return f.size() >= off + 16 && std::memcmp(f.data() + off, sync, 12) == 0;
}

Medium normalize(std::vector<u8> file);

// zlib (RFC 1950) around DEFLATE (RFC 1951), written from the specifications;
// `cap` bounds the plain size. The adler32 is verified.
bool zlibInflate(const u8* src, std::size_t n, std::vector<u8>& out, std::size_t cap);
// A Disk Utility UDIF image: true when the 'koly' trailer is there; decode
// expands every chunk into the 512-byte-sector device image it describes.
bool isUdif(const std::vector<u8>& file);
bool udifDecode(const std::vector<u8>& file, std::vector<u8>& device, std::string& why);

// Where a disc's HFS volume starts inside its flat image: the Apple partition
// map's first Apple_HFS entry (512-byte map blocks whatever
// the disc's own sector size), or 0 for a bare HFS master -- and 0 as well
// for a disc with no HFS on it at all, which the caller tells apart by looking
// for the 'BD' signature at byte 1024 of the result.
inline u32 findHfsPartition(const std::vector<u8>& disc) {
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

// True when the flat image holds an HFS volume the guest can mount at the
// offset findHfsPartition() reports.
inline bool hasHfsVolume(const std::vector<u8>& disc) {
    const std::size_t at = static_cast<std::size_t>(findHfsPartition(disc)) + 1024u;
    return at + 2u <= disc.size() && disc[at] == 0x42 && disc[at + 1u] == 0x44;
}

} // namespace openmac::cd
