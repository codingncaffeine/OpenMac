#pragma once

// MacBinary containers.
//
// A Macintosh file is two forks and a Finder record, and none of that survives
// a foreign file system -- so nearly everything that left a Mac travelled in
// MacBinary: a 128-byte header carrying the name, type/creator and fork sizes,
// then the data fork, then the resource fork, each zero-padded to a 128-byte
// boundary. Archived floppy images are no exception; a great many ".img" files
// in circulation are really MacBinary around a DiskCopy image or a raw dump,
// and reading one as sectors puts everything 128 bytes out of place, so nothing
// mounts and the System offers to initialise somebody's master disk.
//
// Header layout (big-endian), per the MacBinary II standard:
//
//   0        u8   old version -- must be zero
//   1        u8   filename length, 1..63
//   2-64     the filename
//   65-68    file type          69-72   file creator
//   73       Finder flags       74      must be zero
//   75-80    icon position + folder id
//   81       protected flag     82      must be zero
//   83       u32  data fork length
//   87       u32  resource fork length
//   91/95    u32  created / modified
//   122/123  MacBinary II version written / needed (129; zero in MacBinary I)
//   124      u16  CRC of bytes 0-123 (CCITT, init 0; zero in MacBinary I)
//
// Identification leans on the CRC when one is present; a MacBinary I file has
// none, so there the structure has to carry the argument: the zero bytes where
// the standard demands them, a plausible name, and fork lengths that account
// for the file.

#include "openmac/types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace openmac::macbinary {

constexpr std::size_t kHeaderSize = 128;

inline u32 be32(const u8* p) {
    return (static_cast<u32>(p[0]) << 24) | (static_cast<u32>(p[1]) << 16) |
           (static_cast<u32>(p[2]) << 8) | static_cast<u32>(p[3]);
}

// CCITT CRC-16 as MacBinary II specifies it: polynomial $1021, initialised to
// zero (the XMODEM convention -- the disk controllers use the same polynomial
// but start from all ones).
inline u16 crc(const u8* p, std::size_t n) {
    u16 c = 0;
    for (std::size_t i = 0; i < n; ++i) {
        c ^= static_cast<u16>(static_cast<u16>(p[i]) << 8);
        for (int b = 0; b < 8; ++b)
            c = static_cast<u16>((c & 0x8000) ? ((c << 1) ^ 0x1021) : (c << 1));
    }
    return c;
}

inline std::size_t pad128(std::size_t n) { return (n + 127) & ~static_cast<std::size_t>(127); }

inline bool isMacBinary(const std::vector<u8>& f) {
    if (f.size() < kHeaderSize) return false;
    if (f[0] != 0 || f[74] != 0) return false;
    const unsigned nameLen = f[1];
    if (nameLen < 1 || nameLen > 63) return false;
    const u32 dataLen = be32(&f[83]), rsrcLen = be32(&f[87]);
    if (dataLen > 0x00FFFFFFu || rsrcLen > 0x00FFFFFFu) return false;
    if (f.size() < kHeaderSize + dataLen + rsrcLen) return false;
    const u16 stored = static_cast<u16>((f[124] << 8) | f[125]);
    if (stored != 0) return crc(f.data(), 124) == stored;   // MacBinary II/III
    // MacBinary I carries no CRC, so ask more of the structure: the byte the
    // standard zeroes at 82, and a total length the two forks account for
    // (each padded to 128, with nothing but slack beyond).
    if (f[82] != 0) return false;
    const std::size_t padded = kHeaderSize + pad128(dataLen) + pad128(rsrcLen);
    return f.size() <= padded;
}

// The pieces of a MacBinary file, kept so the file can be put back together
// byte-for-byte around whatever the guest did to the data fork.
struct Parts {
    std::vector<u8> header;     // the 128 bytes
    std::vector<u8> resource;   // the resource fork, unpadded
    bool wrapped() const { return header.size() == kHeaderSize; }
    std::string type() const { return fourcc(65); }
    std::string creator() const { return fourcc(69); }
    std::string name() const {
        if (!wrapped()) return {};
        const unsigned n = header[1] <= 63 ? header[1] : 63;
        return std::string(reinterpret_cast<const char*>(&header[2]), n);
    }
private:
    std::string fourcc(std::size_t at) const {
        if (!wrapped()) return {};
        std::string s;
        for (std::size_t i = 0; i < 4; ++i) {
            const char c = static_cast<char>(header[at + i]);
            s += (c >= 0x20 && c < 0x7F) ? c : '?';
        }
        return s;
    }
};

// Split a MacBinary file: `f` is left holding just the data fork, and the
// header and resource fork come back so the file can be reassembled. A file
// that is not MacBinary is left alone and the returned parts are empty.
inline Parts split(std::vector<u8>& f) {
    if (!isMacBinary(f)) return {};
    Parts p;
    p.header.assign(f.begin(), f.begin() + kHeaderSize);
    const u32 dataLen = be32(&p.header[83]), rsrcLen = be32(&p.header[87]);
    const std::size_t rsrcAt = kHeaderSize + pad128(dataLen);
    if (rsrcLen && rsrcAt + rsrcLen <= f.size())
        p.resource.assign(f.begin() + static_cast<std::ptrdiff_t>(rsrcAt),
                          f.begin() + static_cast<std::ptrdiff_t>(rsrcAt + rsrcLen));
    std::vector<u8> data(f.begin() + kHeaderSize,
                         f.begin() + static_cast<std::ptrdiff_t>(kHeaderSize + dataLen));
    f = std::move(data);
    return p;
}

// Reassemble the file around a (possibly rewritten) data fork: header with the
// data-fork length refreshed, data padded to 128, resource fork padded to 128.
// The CRC is recomputed only when the original carried one -- a MacBinary I
// file stays a MacBinary I file.
inline std::vector<u8> rewrap(const Parts& p, const std::vector<u8>& data) {
    if (!p.wrapped()) return data;
    std::vector<u8> out(p.header);
    const u32 size = static_cast<u32>(data.size());
    out[83] = static_cast<u8>(size >> 24); out[84] = static_cast<u8>(size >> 16);
    out[85] = static_cast<u8>(size >> 8);  out[86] = static_cast<u8>(size);
    const u16 hadCrc = static_cast<u16>((p.header[124] << 8) | p.header[125]);
    if (hadCrc != 0) {
        const u16 c = crc(out.data(), 124);
        out[124] = static_cast<u8>(c >> 8);
        out[125] = static_cast<u8>(c);
    }
    out.insert(out.end(), data.begin(), data.end());
    out.resize(kHeaderSize + pad128(data.size()), 0);
    if (!p.resource.empty()) {
        out.insert(out.end(), p.resource.begin(), p.resource.end());
        out.resize(out.size() + (pad128(p.resource.size()) - p.resource.size()), 0);
    }
    return out;
}

} // namespace openmac::macbinary
