// Host-side HFS reader — the sync-back half of the folder disk.
//
// Walks a volume image's catalog and extracts forks, including from a volume
// the GUEST has modified: extents are resolved through the initial records
// AND the extents-overflow B*-tree, so files and B*-tree files the System has
// grown or fragmented still read correctly. Everything is bounds-checked —
// the image is the guest's work product and gets no benefit of the doubt.
//
// Same clean-room grounding as the formatter/builder: Inside Macintosh IV/V
// "The File Manager", field packing cross-checked against libhfs.

#include "openmac/hfs.hpp"

#include <cstring>

namespace openmac::hfs {
namespace {

constexpr u32 kBlockSize = 512;

u16 rd16(const u8* p) { return u16((p[0] << 8) | p[1]); }
u32 rd32(const u8* p) {
    return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | p[3];
}

struct Geo {
    u32 alBlkSiz = 0, alBlSt = 0, nmAlBlks = 0, vlen = 0;
    u32 ctFlSize = 0, xtFlSize = 0;
    u16 ctExt[6] = {}, xtExt[6] = {};   // 3 (start, count) pairs each
};

bool readGeo(const std::vector<u8>& img, Geo& g) {
    if (img.size() < 4 * kBlockSize) return false;
    const u8* m = img.data() + 2 * kBlockSize;
    if (rd16(m) != 0x4244) return false;   // 'BD'
    g.nmAlBlks = rd16(m + 0x12);
    g.alBlkSiz = rd32(m + 0x14);
    g.alBlSt = rd16(m + 0x1C);
    g.vlen = u32(img.size() / kBlockSize);
    g.xtFlSize = rd32(m + 0x82);
    for (int i = 0; i < 6; ++i) g.xtExt[i] = rd16(m + 0x86 + 2 * i);
    g.ctFlSize = rd32(m + 0x92);
    for (int i = 0; i < 6; ++i) g.ctExt[i] = rd16(m + 0x96 + 2 * i);
    if (g.alBlkSiz == 0 || g.alBlkSiz % kBlockSize) return false;
    return true;
}

// A fork's allocation-block runs, resolved lazily against the image.
struct ForkMap {
    std::vector<std::pair<u16, u16>> runs;   // (start AB, count)
    u32 abCovered() const {
        u32 n = 0;
        for (auto& r : runs) n += r.second;
        return n;
    }
};

// Copy `len` bytes starting at fork byte `off` out of the mapped runs.
bool copyOut(const std::vector<u8>& img, const Geo& g, const ForkMap& fm,
             u32 off, u32 len, u8* dst) {
    u32 pos = 0;   // fork byte position at the start of the current run
    for (auto& r : fm.runs) {
        const u32 runBytes = u32(r.second) * g.alBlkSiz;
        if (off < pos + runBytes && off + len > pos) {
            const u32 from = off > pos ? off - pos : 0;
            const u32 dstOff = pos + from - off;
            u32 n = std::min(runBytes - from, len - dstOff);
            const u64 src =
                (u64(g.alBlSt) + u64(r.first) * (g.alBlkSiz / kBlockSize)) *
                    kBlockSize +
                from;
            if (src + n > img.size()) return false;
            std::memcpy(dst + dstOff, img.data() + src, n);
        }
        pos += runBytes;
        if (pos >= off + len) return true;
    }
    return pos >= off + len;
}

// Read one 512-byte node of a B*-tree file through its fork map.
bool readNode(const std::vector<u8>& img, const Geo& g, const ForkMap& fm,
              u32 node, u8* out) {
    return copyOut(img, g, fm, node * kBlockSize, kBlockSize, out);
}

// Walk a tree's leaf chain, calling visit(recordBytes, recordLen) for every
// leaf record. The node count is capped so a corrupt chain cannot spin.
template <typename F>
bool walkLeaves(const std::vector<u8>& img, const Geo& g, const ForkMap& fm,
                F&& visit) {
    u8 nd[kBlockSize];
    if (!readNode(img, g, fm, 0, nd)) return false;
    u32 leaf = rd32(nd + 0x0e + 10);   // header record: bthFNode
    u32 hops = 0;
    while (leaf && hops++ < 4096) {
        if (!readNode(img, g, fm, leaf, nd)) return false;
        if (nd[8] != 0xFF) return false;   // not a leaf: chain is broken
        const u16 nrecs = rd16(nd + 10);
        for (u16 i = 0; i < nrecs && i < 128; ++i) {
            const u16 off = rd16(nd + kBlockSize - 2 - 2 * i);
            const u16 end = rd16(nd + kBlockSize - 4 - 2 * i);
            if (off < 14 || end <= off || end > kBlockSize - 2 * (nrecs + 1))
                continue;
            visit(nd + off, u16(end - off));
        }
        leaf = rd32(nd + 0);   // ndFLink
    }
    return true;
}

// The extents-overflow tree's own map comes straight from the MDB.
ForkMap extTreeMap(const Geo& g) {
    ForkMap fm;
    for (int i = 0; i < 3; ++i)
        if (g.xtExt[2 * i + 1]) fm.runs.push_back({g.xtExt[2 * i], g.xtExt[2 * i + 1]});
    return fm;
}

// Resolve a fork: initial extents from its record, continuation from the
// extents-overflow tree keyed (forkType, fileID, startAB).
ForkMap resolveFork(const std::vector<u8>& img, const Geo& g, u32 fileId,
                    u8 forkType, const u16 initial[6], u32 pyLen) {
    ForkMap fm;
    for (int i = 0; i < 3; ++i)
        if (initial[2 * i + 1]) fm.runs.push_back({initial[2 * i], initial[2 * i + 1]});
    const u32 needABs = g.alBlkSiz ? (pyLen + g.alBlkSiz - 1) / g.alBlkSiz : 0;
    int guard = 0;
    while (fm.abCovered() < needABs && guard++ < 64) {
        const u32 want = fm.abCovered();
        bool found = false;
        ForkMap xt = extTreeMap(g);
        walkLeaves(img, g, xt, [&](const u8* rec, u16 len) {
            if (found || len < 8 + 12 || rec[0] != 7) return;
            const u8 fk = rec[1];
            const u32 fn = rd32(rec + 2);
            const u32 st = rd16(rec + 6);
            if (fk == forkType && fn == fileId && st == want) {
                const u8* d = rec + 8;
                for (int i = 0; i < 3; ++i) {
                    const u16 s = rd16(d + 4 * i), c = rd16(d + 4 * i + 2);
                    if (c) fm.runs.push_back({s, c});
                }
                found = true;
            }
        });
        if (!found) break;
    }
    return fm;
}

ForkMap catalogMap(const std::vector<u8>& img, const Geo& g) {
    return resolveFork(img, g, 4 /* catalog file CNID */, 0x00, g.ctExt, g.ctFlSize);
}

} // namespace

bool listVolume(const std::vector<u8>& img, std::vector<Item>& items) {
    Geo g;
    if (!readGeo(img, g)) return false;
    ForkMap cat = catalogMap(img, g);
    std::vector<Item> out;
    const bool ok = walkLeaves(img, g, cat, [&](const u8* rec, u16 len) {
        const u8 keyLen = rec[0];
        if (keyLen < 6 || u16(1 + keyLen) + 2 > len) return;
        const u32 parId = rd32(rec + 2);
        const u8 nameLen = rec[6];
        if (nameLen > 31 || 7 + nameLen > 1 + keyLen) return;
        const u8* d = rec + 1 + keyLen;
        const u16 dataLen = u16(len - (1 + keyLen));
        Item it;
        it.parent = parId;
        it.name.assign(reinterpret_cast<const char*>(rec + 7), nameLen);
        if (d[0] == 1 && dataLen >= 70) {          // directory record
            it.isDir = true;
            it.id = rd32(d + 6);
            it.crDate = rd32(d + 10);
            it.mdDate = rd32(d + 14);
            out.push_back(std::move(it));
        } else if (d[0] == 2 && dataLen >= 102) {  // file record
            it.isDir = false;
            it.type = rd32(d + 4);
            it.creator = rd32(d + 8);
            it.fdFlags = rd16(d + 12);
            it.id = rd32(d + 20);
            it.dataLen = rd32(d + 26);
            it.rsrcLen = rd32(d + 36);
            it.crDate = rd32(d + 44);
            it.mdDate = rd32(d + 48);
            out.push_back(std::move(it));
        }
        // Thread records (types 3/4) carry no content worth listing.
    });
    if (!ok) return false;
    // Root (id 2) first, then everything else in catalog order.
    for (std::size_t i = 0; i < out.size(); ++i)
        if (out[i].isDir && out[i].id == 2 && i != 0) {
            std::swap(out[0], out[i]);
            break;
        }
    items = std::move(out);
    return true;
}

bool readFork(const std::vector<u8>& img, u32 fileId, bool rsrc, std::vector<u8>& out) {
    Geo g;
    if (!readGeo(img, g)) return false;
    ForkMap cat = catalogMap(img, g);
    bool found = false;
    u32 lgLen = 0, pyLen = 0;
    u16 initial[6] = {};
    walkLeaves(img, g, cat, [&](const u8* rec, u16 len) {
        if (found) return;
        const u8 keyLen = rec[0];
        if (keyLen < 6 || u16(1 + keyLen) + 102 > len) return;
        const u8* d = rec + 1 + keyLen;
        if (d[0] != 2 || rd32(d + 20) != fileId) return;
        found = true;
        if (!rsrc) {
            lgLen = rd32(d + 26);
            pyLen = rd32(d + 30);
            for (int i = 0; i < 6; ++i) initial[i] = rd16(d + 74 + 2 * i);
        } else {
            lgLen = rd32(d + 36);
            pyLen = rd32(d + 40);
            for (int i = 0; i < 6; ++i) initial[i] = rd16(d + 86 + 2 * i);
        }
    });
    if (!found) return false;
    if (lgLen == 0) { out.clear(); return true; }
    if (pyLen < lgLen) pyLen = lgLen;
    if (u64(lgLen) > img.size()) return false;   // nonsense length
    ForkMap fm = resolveFork(img, g, fileId, rsrc ? 0xFF : 0x00, initial, pyLen);
    out.assign(lgLen, 0);
    return copyOut(img, g, fm, 0, lgLen, out.data());
}

} // namespace openmac::hfs
