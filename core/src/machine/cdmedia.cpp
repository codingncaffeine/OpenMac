#include "cdmedia.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

// CD-ROM media containers: the decoders behind cd::normalize(). Everything here
// is written from the specifications named at each piece -- there is no
// third-party code in the core to hand the job to:
//   DEFLATE       RFC 1951; the zlib wrapper RFC 1950 (Disk Utility compresses
//                 UDIF chunks with zlib)
//   UDIF          Apple's Disk Utility image: a 512-byte-sector device image
//                 cut into chunks (raw, zero-filled, zlib) behind an XML chunk
//                 table ('mish' blocks, base64 in a plist) and a 512-byte
//                 'koly' trailer
//   raw sectors   ECMA-130 / "Yellow Book": 12-byte sync, 4-byte header, then
//                 the 2048 user bytes at +16 (mode 1) or +24 (mode 2, behind
//                 the 8-byte XA subheader), then EDC/ECC; a 2336-byte image
//                 keeps only the subheader in front (+8); 2448 adds 96 bytes
//                 of subchannel behind every sector

namespace openmac::cd {

namespace {

u32 be32(const u8* p) {
    return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | p[3];
}
u64 be64(const u8* p) { return (u64(be32(p)) << 32) | be32(p + 4); }

// -------------------------------------------------------------- inflate --

struct BitIn {
    const u8* p;
    std::size_t n, at = 0;
    u32 bit = 0;
    bool bad = false;
    u32 take(u32 count) {
        u32 v = 0;
        for (u32 k = 0; k < count; ++k) {
            if (at >= n) { bad = true; return 0; }
            v |= u32((p[at] >> bit) & 1u) << k;
            if (++bit == 8) { bit = 0; ++at; }
        }
        return v;
    }
    void align() { if (bit) { bit = 0; ++at; } }
};

// Canonical Huffman decode (RFC 1951 3.2.2): count the codes of each bit
// length, then walk the incoming code one bit at a time against the count
// table. Slow and simple; a UDIF chunk is at most a megabyte.
struct Huff {
    u16 count[16] = {};
    u16 sym[288] = {};
    bool build(const u8* lens, u32 n) {
        for (u16& c : count) c = 0;
        for (u32 k = 0; k < n; ++k) ++count[lens[k]];
        count[0] = 0;
        // An over-subscribed code cannot decode; reject it here rather than
        // reading garbage.
        int left = 1;
        for (u32 l = 1; l < 16; ++l) {
            left <<= 1;
            left -= count[l];
            if (left < 0) return false;
        }
        u16 offs[16] = {};
        for (u32 l = 1; l < 15; ++l) offs[l + 1] = static_cast<u16>(offs[l] + count[l]);
        for (u32 k = 0; k < n; ++k)
            if (lens[k]) sym[offs[lens[k]]++] = static_cast<u16>(k);
        return true;
    }
    int decode(BitIn& in) const {
        u32 code = 0, first = 0, index = 0;
        for (u32 len = 1; len < 16; ++len) {
            code |= in.take(1);
            const u32 c = count[len];
            if (code < first + c) return sym[index + (code - first)];
            index += c;
            first = (first + c) << 1;
            code <<= 1;
        }
        return -1;
    }
};

bool inflateRaw(const u8* src, std::size_t n, std::vector<u8>& out, std::size_t cap) {
    static const u16 kLenBase[29] = {3,  4,  5,  6,  7,  8,  9,  10, 11,  13,
                                     15, 17, 19, 23, 27, 31, 35, 43, 51,  59,
                                     67, 83, 99, 115, 131, 163, 195, 227, 258};
    static const u8 kLenExtra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
                                     2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
    static const u16 kDistBase[30] = {1,    2,    3,    4,    5,    7,    9,    13,
                                      17,   25,   33,   49,   65,   97,   129,  193,
                                      257,  385,  513,  769,  1025, 1537, 2049, 3073,
                                      4097, 6145, 8193, 12289, 16385, 24577};
    static const u8 kDistExtra[30] = {0, 0, 0, 0, 1, 1, 2, 2,  3,  3,  4,  4,  5,  5,  6,
                                      6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
    static const u8 kClOrder[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5,
                                    11, 4, 12, 3, 13, 2, 14, 1, 15};
    BitIn in{src, n};
    for (;;) {
        const u32 fin = in.take(1);
        const u32 type = in.take(2);
        if (in.bad) return false;
        if (type == 0) {                                   // stored
            in.align();
            if (in.at + 4 > in.n) return false;
            const u32 len = u32(in.p[in.at]) | (u32(in.p[in.at + 1]) << 8);
            const u32 nlen = u32(in.p[in.at + 2]) | (u32(in.p[in.at + 3]) << 8);
            in.at += 4;
            if ((len ^ 0xFFFFu) != nlen || in.at + len > in.n || out.size() + len > cap)
                return false;
            out.insert(out.end(), in.p + in.at, in.p + in.at + len);
            in.at += len;
        } else if (type == 1 || type == 2) {
            Huff lit, dist;
            if (type == 1) {                               // fixed tables, 3.2.6
                u8 lens[288];
                for (u32 k = 0; k < 144; ++k) lens[k] = 8;
                for (u32 k = 144; k < 256; ++k) lens[k] = 9;
                for (u32 k = 256; k < 280; ++k) lens[k] = 7;
                for (u32 k = 280; k < 288; ++k) lens[k] = 8;
                if (!lit.build(lens, 288)) return false;
                u8 dlens[30];
                for (u32 k = 0; k < 30; ++k) dlens[k] = 5;
                if (!dist.build(dlens, 30)) return false;
            } else {                                       // dynamic tables, 3.2.7
                const u32 hlit = in.take(5) + 257;
                const u32 hdist = in.take(5) + 1;
                const u32 hclen = in.take(4) + 4;
                if (hlit > 286 || hdist > 30) return false;
                u8 cl[19] = {};
                for (u32 k = 0; k < hclen; ++k) cl[kClOrder[k]] = static_cast<u8>(in.take(3));
                Huff clh;
                if (in.bad || !clh.build(cl, 19)) return false;
                u8 lens[288 + 30] = {};
                u32 got = 0;
                while (got < hlit + hdist) {
                    const int s = clh.decode(in);
                    if (s < 0 || in.bad) return false;
                    if (s < 16) {
                        lens[got++] = static_cast<u8>(s);
                    } else if (s == 16) {
                        if (got == 0) return false;
                        const u8 prev = lens[got - 1];
                        u32 rep = 3 + in.take(2);
                        while (rep-- && got < hlit + hdist) lens[got++] = prev;
                    } else {
                        u32 rep = s == 17 ? 3 + in.take(3) : 11 + in.take(7);
                        while (rep-- && got < hlit + hdist) lens[got++] = 0;
                    }
                }
                if (!lit.build(lens, hlit) || !dist.build(lens + hlit, hdist)) return false;
            }
            for (;;) {
                const int s = lit.decode(in);
                if (s < 0 || in.bad) return false;
                if (s < 256) {
                    if (out.size() >= cap) return false;
                    out.push_back(static_cast<u8>(s));
                } else if (s == 256) {
                    break;
                } else {
                    if (s - 257 >= 29) return false;
                    const u32 len = kLenBase[s - 257] + in.take(kLenExtra[s - 257]);
                    const int d = dist.decode(in);
                    if (d < 0 || d >= 30 || in.bad) return false;
                    const u32 back = kDistBase[d] + in.take(kDistExtra[d]);
                    if (back > out.size() || out.size() + len > cap) return false;
                    // Byte-at-a-time on purpose: a distance shorter than the
                    // length repeats the just-written bytes.
                    for (u32 k = 0; k < len; ++k) out.push_back(out[out.size() - back]);
                }
            }
        } else {
            return false;
        }
        if (fin) return !in.bad;
    }
}

u32 adler32(const u8* p, std::size_t n) {
    u32 a = 1, b = 0;
    for (std::size_t k = 0; k < n; ++k) {
        a = (a + p[k]) % 65521u;
        b = (b + a) % 65521u;
    }
    return (b << 16) | a;
}

// -------------------------------------------------------------- base64 --

bool base64Decode(const char* s, const char* end, std::vector<u8>& out) {
    int have = 0;
    u32 acc = 0;
    for (; s != end; ++s) {
        const char c = *s;
        int v;
        if (c >= 'A' && c <= 'Z') v = c - 'A';
        else if (c >= 'a' && c <= 'z') v = c - 'a' + 26;
        else if (c >= '0' && c <= '9') v = c - '0' + 52;
        else if (c == '+') v = 62;
        else if (c == '/') v = 63;
        else if (c == '=' || c == '\n' || c == '\r' || c == ' ' || c == '\t') continue;
        else return false;
        acc = (acc << 6) | u32(v);
        if (++have == 4) {
            out.push_back(static_cast<u8>(acc >> 16));
            out.push_back(static_cast<u8>(acc >> 8));
            out.push_back(static_cast<u8>(acc));
            have = 0;
            acc = 0;
        }
    }
    if (have == 2) out.push_back(static_cast<u8>(acc >> 4));
    else if (have == 3) {
        out.push_back(static_cast<u8>(acc >> 10));
        out.push_back(static_cast<u8>(acc >> 2));
    } else if (have == 1) return false;
    return true;
}

// The user-data view of a sector-framed image: every stored sector of
// `stride` bytes contributes 2048 bytes taken from `dataOff` (or, for raw
// sectors, from the offset the sector's own mode byte selects).
std::vector<u8> unframe(const std::vector<u8>& file, std::size_t stride,
                        std::size_t dataOff, bool perSectorMode) {
    const std::size_t sectors = file.size() / stride;
    std::vector<u8> out(sectors * 2048u, 0);
    for (std::size_t s = 0; s < sectors; ++s) {
        const u8* sec = file.data() + s * stride;
        std::size_t off = dataOff;
        if (perSectorMode) {
            const u8 mode = sec[15];
            if (mode == 0) continue;               // an empty (mode 0) sector
            off = mode == 2 ? 24 : 16;
        }
        std::memcpy(out.data() + s * 2048u, sec + off, 2048u);
    }
    return out;
}

bool isoDescriptorAt(const std::vector<u8>& f, std::size_t at) {
    return f.size() >= at + 16 &&
           (std::memcmp(f.data() + at + 1, "CD001", 5) == 0 ||
            std::memcmp(f.data() + at + 9, "CDROM", 5) == 0);
}
bool sig2(const std::vector<u8>& f, std::size_t at, char a, char b) {
    return f.size() >= at + 2 && f[at] == u8(a) && f[at + 1] == u8(b);
}

} // namespace

bool zlibInflate(const u8* src, std::size_t n, std::vector<u8>& out, std::size_t cap) {
    if (n < 6 || (src[0] & 0x0F) != 8 || (src[1] & 0x20) != 0 ||
        ((u32(src[0]) << 8) | src[1]) % 31 != 0)
        return false;
    out.clear();
    out.reserve(cap);
    if (!inflateRaw(src + 2, n - 2, out, cap)) return false;
    return adler32(out.data(), out.size()) == be32(src + n - 4);
}

bool isUdif(const std::vector<u8>& file) {
    return file.size() >= 512 &&
           std::memcmp(file.data() + file.size() - 512, "koly", 4) == 0;
}

bool udifDecode(const std::vector<u8>& file, std::vector<u8>& device, std::string& why) {
    if (!isUdif(file)) { why = "not a UDIF image (no 'koly' trailer)"; return false; }
    const u8* koly = file.data() + file.size() - 512;
    const u32 version = be32(koly + 4);
    if (version != 4) {
        why = "UDIF version " + std::to_string(version) + " is not supported";
        return false;
    }
    const u64 dataFork = be64(koly + 24);
    const u64 xmlOff = be64(koly + 216);
    const u64 xmlLen = be64(koly + 224);
    const u64 kolySectors = be64(koly + 492);
    if (!xmlLen || xmlOff + xmlLen > file.size()) {
        why = "the UDIF has no XML chunk table (resource-fork images are not supported)";
        return false;
    }
    const std::string xml(reinterpret_cast<const char*>(file.data() + xmlOff),
                          static_cast<std::size_t>(xmlLen));

    struct Run { u32 type; u64 sector, sectors, off, len; };
    std::vector<Run> runs;
    // Every <data> block that follows a <key>blkx</key> is one partition's
    // 'mish' run table, base64-encoded.
    const std::size_t blkx = xml.find("<key>blkx</key>");
    if (blkx == std::string::npos) { why = "the UDIF XML carries no blkx table"; return false; }
    const std::size_t arrEnd = xml.find("</array>", blkx);
    std::size_t at = blkx;
    u64 maxSector = 0;
    for (;;) {
        const std::size_t d0 = xml.find("<data>", at);
        if (d0 == std::string::npos || (arrEnd != std::string::npos && d0 > arrEnd)) break;
        const std::size_t d1 = xml.find("</data>", d0);
        if (d1 == std::string::npos) break;
        at = d1 + 7;
        std::vector<u8> mish;
        if (!base64Decode(xml.data() + d0 + 6, xml.data() + d1, mish)) {
            why = "the UDIF blkx data does not decode";
            return false;
        }
        if (mish.size() < 208 || be32(mish.data()) != 0x6D697368u)
            continue;                                   // not a mish block
        const u64 partStart = be64(mish.data() + 8);
        const u64 tableDataOff = be64(mish.data() + 24);
        const u32 chunks = be32(mish.data() + 200);
        if (mish.size() < 204 + std::size_t(chunks) * 40) {
            why = "the UDIF chunk table is truncated";
            return false;
        }
        for (u32 k = 0; k < chunks; ++k) {
            const u8* c = mish.data() + 204 + std::size_t(k) * 40;
            Run r;
            r.type = be32(c);
            r.sector = partStart + be64(c + 8);
            r.sectors = be64(c + 16);
            r.off = dataFork + tableDataOff + be64(c + 24);
            r.len = be64(c + 32);
            if (r.type == 0xFFFFFFFFu || r.type == 0x7FFFFFFEu) continue;   // terminator / comment
            if (r.type != 0 && r.type != 1 && r.type != 2 && r.type != 0x80000005u) {
                const char* what = r.type == 0x80000004u ? "ADC"
                                 : r.type == 0x80000006u ? "bzip2"
                                 : r.type == 0x80000007u ? "LZFSE"
                                 : r.type == 0x80000008u ? "LZMA"
                                                         : "an unknown codec";
                char b[128];
                std::snprintf(b, sizeof b,
                              "the UDIF uses %s-compressed chunks (type %08x) -- "
                              "convert it with Disk Utility (read-only or zlib)",
                              what, r.type);
                why = b;
                return false;
            }
            if (!r.sectors) continue;
            runs.push_back(r);
            if (r.sector + r.sectors > maxSector) maxSector = r.sector + r.sectors;
        }
    }
    if (runs.empty()) { why = "the UDIF chunk table is empty"; return false; }
    const u64 devSectors = std::max(kolySectors, maxSector);
    if (devSectors > (u64(1) << 22)) {                  // 2 GiB of 512-byte sectors
        why = "the UDIF describes a device larger than 2 GB";
        return false;
    }
    device.assign(static_cast<std::size_t>(devSectors * 512u), 0);
    for (const Run& r : runs) {
        const std::size_t dst = static_cast<std::size_t>(r.sector * 512u);
        const std::size_t bytes = static_cast<std::size_t>(r.sectors * 512u);
        if (dst + bytes > device.size()) { why = "a UDIF chunk lies outside the device"; return false; }
        switch (r.type) {
        case 0: case 2:                                     // zero-fill / unallocated
            break;
        case 1: {                                           // raw
            if (r.off + r.len > file.size()) { why = "a UDIF raw chunk lies outside the file"; return false; }
            std::memcpy(device.data() + dst, file.data() + r.off,
                        static_cast<std::size_t>(std::min<u64>(r.len, bytes)));
            break;
        }
        case 0x80000005u: {                                 // zlib
            if (r.off + r.len > file.size()) { why = "a UDIF zlib chunk lies outside the file"; return false; }
            std::vector<u8> plain;
            if (!zlibInflate(file.data() + r.off, static_cast<std::size_t>(r.len), plain, bytes)) {
                why = "a UDIF zlib chunk does not inflate (corrupt image?)";
                return false;
            }
            std::memcpy(device.data() + dst, plain.data(), std::min(plain.size(), bytes));
            break;
        }
        default: break;
        }
    }
    return true;
}

Medium normalize(std::vector<u8> file) {
    Medium m;
    const double mb = static_cast<double>(file.size()) / (1024.0 * 1024.0);
    std::string prefix;

    // Disk Utility's UDIF (.dmg): decode the whole device, then look at what
    // it holds exactly as if the bytes had arrived flat.
    if (isUdif(file)) {
        std::vector<u8> device;
        std::string why;
        if (!udifDecode(file, device, why)) {
            std::snprintf(m.desc, sizeof m.desc, "%s", why.c_str());
            return m;
        }
        char b[64];
        std::snprintf(b, sizeof b, "UDIF disk image (%.0f MB packed) holding ", mb);
        prefix = b;
        file = std::move(device);
    }
    const double mbFlat = static_cast<double>(file.size()) / (1024.0 * 1024.0);

    // Raw sectors: the sync pattern opens every sector. 2352 bytes each, or
    // 2448 with 96 bytes of subchannel behind each one. MODE1 carries its
    // 2048 user bytes at +16, MODE2 (XA) at +24 behind the subheader.
    if (hasRawSync(file, 0)) {
        const std::size_t stride = (!hasRawSync(file, 2352) && hasRawSync(file, 2448)) ? 2448 : 2352;
        const u8 mode = file[15];
        if (mode != 1 && mode != 2) {
            std::snprintf(m.desc, sizeof m.desc,
                          "a raw-sector CD image whose first sector is MODE%u -- only "
                          "MODE1 and MODE2 data discs are supported", mode);
            return m;
        }
        m.data = unframe(file, stride, 16, true);
        m.ok = true;
        std::snprintf(m.desc, sizeof m.desc,
                      "%sraw MODE%u CD image (%.0f MB, %zu-byte sectors) -- %zu data sectors extracted",
                      prefix.c_str(), mode, mbFlat, stride, m.data.size() / 2048u);
        return m;
    }

    // Apple-partitioned master: Driver Descriptor Map at block 0. Checked
    // before ISO because hybrid HFS/ISO discs carry both, and the partition
    // map is the truth about where things live.
    if (sig2(file, 0, 'E', 'R')) {
        m.data = std::move(file);
        m.ok = true;
        std::snprintf(m.desc, sizeof m.desc, "%sApple-partitioned CD (%.0f MB)%s", prefix.c_str(), mbFlat,
                      hasHfsVolume(m.data) ? "" : " with no HFS volume in its map");
        return m;
    }

    // ISO 9660 / High Sierra: a volume descriptor at sector 16.
    if (isoDescriptorAt(file, 16u * 2048u)) {
        m.data = std::move(file);
        m.ok = true;
        std::snprintf(m.desc, sizeof m.desc, "%sISO 9660 CD image (%.0f MB)%s", prefix.c_str(), mbFlat,
                      hasHfsVolume(m.data) ? " with an HFS volume" : " -- no HFS volume, so System 7 needs Foreign File Access to mount it");
        return m;
    }

    // A bare HFS volume master: the MDB signature two blocks in. HFS Plus
    // ('H+') is refused by name: no System this emulator runs can read it.
    if (sig2(file, 1024, 'B', 'D')) {
        m.data = std::move(file);
        m.ok = true;
        std::snprintf(m.desc, sizeof m.desc, "%sHFS CD master (%.0f MB)", prefix.c_str(), mbFlat);
        return m;
    }
    if (sig2(file, 1024, 'H', '+') || sig2(file, 1024, 'H', 'X')) {
        std::snprintf(m.desc, sizeof m.desc,
                      "%san HFS Plus volume -- System 7 cannot read HFS Plus; the disc "
                      "needs to be an HFS (Standard) or ISO 9660 master", prefix.c_str());
        return m;
    }

    // MODE2 sectors stored without sync/header (2336 bytes: 8-byte subheader
    // then the 2048 user bytes): recognised by finding a known layout behind
    // that framing, since there is no sync mark to see.
    if (file.size() % 2336u == 0 && file.size() >= 17u * 2336u &&
        (isoDescriptorAt(file, 16u * 2336u + 8u) || sig2(file, 8, 'E', 'R') ||
         sig2(file, 8 + 1024, 'B', 'D'))) {
        m.data = unframe(file, 2336, 8, false);
        m.ok = true;
        std::snprintf(m.desc, sizeof m.desc,
                      "%sraw MODE2/2336 CD image (%.0f MB) -- %zu data sectors extracted",
                      prefix.c_str(), mbFlat, m.data.size() / 2048u);
        return m;
    }

    std::snprintf(m.desc, sizeof m.desc,
                  "%snot a CD image this drive recognises -- expected ISO 9660, raw "
                  "2352/2336/2448-byte MODE1 or MODE2 sectors, a UDIF .dmg, an "
                  "Apple-partitioned master, or a bare HFS master", prefix.c_str());
    return m;
}

} // namespace openmac::cd
