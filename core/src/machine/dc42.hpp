#pragma once

// DiskCopy 4.2 disk images.
//
// Much of the archived Macintosh software ships this way rather than as a raw
// sector dump: Apple's DiskCopy wrote an 84-byte header in front of the disk
// data, and a great many .img files in circulation are really this format. The
// difference is invisible in a file listing -- a 1.4 MB image is 1474644 bytes
// instead of 1474560 -- and reading one as though it were raw puts every sector
// 84 bytes out of place, so nothing mounts and the machine shows the flashing
// question mark of a disk with no System on it.
//
// Header layout (big-endian), per Apple's DiskCopy 4.2 documentation:
//
//   0      Pascal string, 64 bytes: the disk's name
//   64     u32  data size in bytes
//   68     u32  tag size in bytes
//   72     u32  data checksum
//   76     u32  tag checksum
//   80     u8   disk encoding
//   81     u8   format byte
//   82     u16  $0100 -- the private word that identifies the format
//   84     the disk data, then the tag bytes
//
// Tags are the twelve bytes per sector that the file system used for recovery.
// The GCR surface synthesises them as zeroes -- no disk needs them to mount --
// but the file's own tag block is kept and put back on the way out, along with
// its checksum field untouched, so an image that came in with tags leaves with
// them and an unchanged disk round-trips byte for byte. (The guest cannot
// change a tag through this drive: the surface never carries real ones.)

#include "openmac/types.hpp"

#include <cstddef>
#include <vector>

namespace openmac::dc42 {

constexpr std::size_t kHeaderSize = 84;

inline u32 be32(const std::vector<u8>& v, std::size_t at) {
    return (static_cast<u32>(v[at]) << 24) | (static_cast<u32>(v[at + 1]) << 16) |
           (static_cast<u32>(v[at + 2]) << 8) | static_cast<u32>(v[at + 3]);
}

// Is this a DiskCopy 4.2 image? Identified by the $0100 word at 82 together with
// a length that exactly accounts for the header, the data and the tags -- both,
// so a raw image whose bytes happen to look like the magic is not mistaken for
// one.
inline bool isDiskCopy(const std::vector<u8>& img) {
    if (img.size() <= kHeaderSize + 2) return false;
    if (!(img[82] == 0x01 && img[83] == 0x00)) return false;
    const u32 dataSize = be32(img, 64), tagSize = be32(img, 68);
    if (dataSize == 0 || (dataSize & 0x1FF) != 0) return false;
    return kHeaderSize + static_cast<std::size_t>(dataSize) +
               static_cast<std::size_t>(tagSize) == img.size();
}

inline u32 dataSize(const std::vector<u8>& img) { return be32(img, 64); }

// The pieces of a DiskCopy file that are not sector data, kept so the file can
// be put back together on the way out.
struct Parts {
    std::vector<u8> header;   // the 84 bytes
    std::vector<u8> tags;     // the tag block that followed the data, if any
    bool wrapped() const { return header.size() == kHeaderSize; }
};

// Split a loaded file into the sectors the drive sees and the pieces we must
// keep. `img` is left holding just the disk data. A raw image is left alone
// and the returned parts are empty.
inline Parts split(std::vector<u8>& img) {
    if (!isDiskCopy(img)) return {};
    Parts p;
    const std::size_t data = dataSize(img);
    p.header.assign(img.begin(), img.begin() + kHeaderSize);
    p.tags.assign(img.begin() + static_cast<std::ptrdiff_t>(kHeaderSize + data), img.end());
    std::vector<u8> sectors(img.begin() + kHeaderSize,
                            img.begin() + static_cast<std::ptrdiff_t>(kHeaderSize + data));
    img = std::move(sectors);
    return p;
}

// Compatibility shim for callers that only want the header and are content to
// lose the tags.
inline std::vector<u8> takeHeader(std::vector<u8>& img) { return split(img).header; }

// Apple's checksum: for every 16-bit word, add it to the running total and then
// rotate the total right one bit. Used for both the data and tag fields.
inline u32 checksum(const u8* p, std::size_t n) {
    u32 sum = 0;
    for (std::size_t i = 0; i + 1 < n; i += 2) {
        sum += (static_cast<u32>(p[i]) << 8) | p[i + 1];
        sum = (sum >> 1) | (sum << 31);
    }
    return sum;
}

// Put the file back together around a (possibly rewritten) data block: sizes
// and the data checksum refreshed, the original tag block and its checksum
// field carried through unchanged. The guest cannot alter tags through the
// drive, so what came in is still true on the way out -- and an untouched disk
// reassembles to the identical file, tag-checksum quirks and all.
inline std::vector<u8> rewrap(const Parts& p, const std::vector<u8>& data) {
    if (!p.wrapped()) return data;
    std::vector<u8> out(p.header);
    const u32 size = static_cast<u32>(data.size());
    out[64] = static_cast<u8>(size >> 24); out[65] = static_cast<u8>(size >> 16);
    out[66] = static_cast<u8>(size >> 8);  out[67] = static_cast<u8>(size);
    const u32 tagSize = static_cast<u32>(p.tags.size());
    out[68] = static_cast<u8>(tagSize >> 24); out[69] = static_cast<u8>(tagSize >> 16);
    out[70] = static_cast<u8>(tagSize >> 8);  out[71] = static_cast<u8>(tagSize);
    const u32 ck = checksum(data.data(), data.size());
    out[72] = static_cast<u8>(ck >> 24); out[73] = static_cast<u8>(ck >> 16);
    out[74] = static_cast<u8>(ck >> 8);  out[75] = static_cast<u8>(ck);
    out.insert(out.end(), data.begin(), data.end());
    out.insert(out.end(), p.tags.begin(), p.tags.end());
    return out;
}

// The old shape, for callers that dropped the tags.
inline std::vector<u8> rewrap(const std::vector<u8>& header, const std::vector<u8>& data) {
    Parts p;
    p.header = header;
    return rewrap(p, data);
}

} // namespace openmac::dc42
