// Host-side HFS repair — bringing a volume back inside the ROM's own reach.
//
// The Quadra's ROM rebuilds a volume that was not unmounted cleanly before it
// mounts it, and it bounds its walk of the extents-overflow B*-tree by
// ceil(fileBytes / 12), computed at $4080F456 with a SIXTEEN-BIT DIVU. Past
// 12 * 65535 bytes the quotient does not fit; a 68000 leaves the destination
// untouched on divide overflow, the ceil wrapper answers 1, and the rebuild
// then gives up the moment it finds a single extents record. The volume is
// refused -- the blinking question mark.
//
// Our formatter used to give the extents file 2048 nodes (1 MB on 8 KB
// allocation blocks), which is over that line. Apple's own tools never went
// near it: an Apple 1.44 MB System disk gives the extents file 11,264 bytes and
// a 10 MB volume 81,920. The formatter is fixed; this is for the volumes that
// already exist and hold the user's data, where reformatting is not an answer.
//
// Same clean-room grounding as the rest of core/src/hfs: Inside Macintosh IV/V
// "The File Manager" for the MDB and B*-tree layouts.

#include "openmac/hfs.hpp"

#include <cstdarg>
#include <cstdio>

namespace openmac::hfs {
namespace {

constexpr u32 kBlockSize = 512;
constexpr u32 kSigWord = 0x4244;      // 'BD'

// The header node's map record covers this many nodes, which is also why the
// old formatter stopped at 2048: one map record, no continuation nodes.
constexpr u32 kMapRecOff = 248, kMapRecBytes = 256;

u16 rd16(const u8* p) { return u16((p[0] << 8) | p[1]); }
u32 rd32(const u8* p) {
    return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | p[3];
}
void wr16(u8* p, u16 v) { p[0] = u8(v >> 8); p[1] = u8(v); }
void wr32(u8* p, u32 v) {
    p[0] = u8(v >> 24); p[1] = u8(v >> 16); p[2] = u8(v >> 8); p[3] = u8(v);
}

bool bitSet(const u8* map, u32 i) { return (map[i >> 3] & (0x80u >> (i & 7u))) != 0; }
void clearBit(u8* map, u32 i) { map[i >> 3] = u8(map[i >> 3] & ~(0x80u >> (i & 7u))); }

void say(std::string& why, const char* fmt, ...) {
    char buf[220];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    why = buf;
}

} // namespace

bool shrinkExtentsTree(std::vector<u8>& img, u32 base, u32 volumeBytes,
                       u32 maxBytes, std::string& why) {
    why.clear();
    if (volumeBytes < 8u * kBlockSize ||
        u64(base) + volumeBytes > img.size()) {
        say(why, "not an HFS volume: the image is too small to hold one");
        return false;
    }
    u8* vol = img.data() + base;
    u8* mdb = vol + 2 * kBlockSize;
    if (rd16(mdb) != kSigWord) {
        say(why, "not an HFS volume: no 'BD' signature at block 2");
        return false;
    }

    const u32 nmAlBlks = rd16(mdb + 0x12);
    const u32 alBlkSiz = rd32(mdb + 0x14);
    const u32 vbmSt    = rd16(mdb + 0x0E);
    const u32 alBlSt   = rd16(mdb + 0x1C);
    const u32 xtFlSize = rd32(mdb + 0x82);
    const u32 xtStart  = rd16(mdb + 0x86);
    const u32 xtBlks   = rd16(mdb + 0x88);

    if (alBlkSiz == 0 || alBlkSiz % kBlockSize || nmAlBlks == 0) {
        say(why, "not an HFS volume: allocation block size %u is impossible",
            alBlkSiz);
        return false;
    }
    if (xtFlSize <= maxBytes) return false;   // nothing to do, and nothing to say

    // Everything below is a refusal with a reason: a volume shaped in a way
    // this repair was not written for is left exactly as it was found.
    for (int i = 2; i < 6; ++i) {
        if (rd16(mdb + 0x86 + 2 * i) != 0) {
            say(why, "the extents tree is in more than one piece -- left alone");
            return false;
        }
    }
    if (u64(xtBlks) * alBlkSiz != xtFlSize) {
        say(why, "the extents tree's size (%u) and its extent (%u blocks of %u) "
                 "disagree -- left alone", xtFlSize, xtBlks, alBlkSiz);
        return false;
    }
    if (u64(xtStart) + xtBlks > nmAlBlks) {
        say(why, "the extents tree runs past the end of the volume -- left alone");
        return false;
    }

    const u32 newBlks = maxBytes / alBlkSiz;
    if (newBlks == 0 || newBlks >= xtBlks) return false;
    const u32 newBytes = newBlks * alBlkSiz;
    const u32 oldNodes = xtFlSize / kBlockSize;
    const u32 newNodes = newBytes / kBlockSize;

    // The header node is the tree's first node, and the tree's first node is
    // the first logical block of its first allocation block.
    const u64 hdrOff = u64(alBlSt) * kBlockSize + u64(xtStart) * alBlkSiz;
    if (hdrOff + kBlockSize > volumeBytes) {
        say(why, "the extents tree's header node is outside the volume -- left alone");
        return false;
    }
    u8* hdr = vol + hdrOff;
    if (hdr[8] != 0x01 || rd16(hdr + 32) != kBlockSize) {
        say(why, "the extents tree does not begin with a 512-byte header node "
                 "-- left alone");
        return false;
    }
    const u32 bthNNodes = rd32(hdr + 36);
    const u32 bthFree   = rd32(hdr + 40);
    if (bthNNodes != oldNodes || bthFree > bthNNodes) {
        say(why, "the extents tree says %u nodes where its file holds %u "
                 "-- left alone", bthNNodes, oldNodes);
        return false;
    }
    if (oldNodes > kMapRecBytes * 8) {
        say(why, "the extents tree is larger than its own node map -- left alone");
        return false;
    }

    // Only free nodes may go. The header node's map record has one bit per
    // node, in use = 1; if anything above the new end is live, this volume is
    // not one we can cut, and saying so is the whole point of checking.
    const u8* map = hdr + kMapRecOff;
    for (u32 n = newNodes; n < oldNodes; ++n) {
        if (bitSet(map, n)) {
            say(why, "the extents tree is using node %u, past where it would be "
                     "cut -- left alone", n);
            return false;
        }
    }

    // The volume bitmap has to agree that we own the blocks we are giving back.
    const u64 vbmOff = u64(vbmSt) * kBlockSize;
    const u32 firstFreed = xtStart + newBlks, freedBlks = xtBlks - newBlks;
    if (vbmOff + (u64(xtStart + xtBlks) + 7) / 8 > volumeBytes) {
        say(why, "the volume bitmap does not reach the extents tree -- left alone");
        return false;
    }
    u8* vbm = vol + vbmOff;
    for (u32 ab = firstFreed; ab < xtStart + xtBlks; ++ab) {
        if (!bitSet(vbm, ab)) {
            say(why, "allocation block %u is already free, so the extents tree "
                     "is not where the volume says -- left alone", ab);
            return false;
        }
    }

    // ---- commit; from here nothing can fail -----------------------------
    wr32(hdr + 36, newNodes);                       // bthNNodes
    wr32(hdr + 40, bthFree - (oldNodes - newNodes));// bthFree: only free ones left
    for (u32 ab = firstFreed; ab < xtStart + xtBlks; ++ab) clearBit(vbm, ab);

    const u32 freeBks = rd16(mdb + 0x22) + freedBlks;
    // The alternate MDB in the second-to-last block is what a repair tool reads
    // when the primary is unreadable, so the same four fields move there too --
    // and only those four. Copying the whole block over it would be easier and
    // would throw away whatever else that copy legitimately held.
    u8* alt = vol + volumeBytes - 1024;
    for (u8* m : {mdb, alt}) {
        if (m == alt && rd16(alt) != kSigWord) continue;   // no usable backup
        wr32(m + 0x82, newBytes);                    // drXTFlSize
        wr16(m + 0x88, u16(newBlks));                // drXTExtRec[0].count
        if (rd32(m + 0x4A) > newBytes) wr32(m + 0x4A, newBytes);   // drXTClpSiz
        wr16(m + 0x22, u16(freeBks < 0x10000u ? freeBks : 0xFFFFu));  // drFreeBks
    }

    say(why, "the extents tree was %u bytes, past the %u the ROM's own repair "
             "can walk; cut to %u (%u unused nodes returned to the volume)",
        xtFlSize, maxBytes, newBytes, oldNodes - newNodes);
    return true;
}

} // namespace openmac::hfs
