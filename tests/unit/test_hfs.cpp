#include <doctest/doctest.h>
#include <openmac/hfs.hpp>

#include <cstdio>
#include <string>
#include <vector>

using namespace openmac;

namespace {

// Big-endian readers into the raw image.
u16 rd16(const std::vector<u8>& b, std::size_t off) {
    return static_cast<u16>((b[off] << 8) | b[off + 1]);
}
u32 rd32(const std::vector<u8>& b, std::size_t off) {
    return (static_cast<u32>(b[off]) << 24) | (static_cast<u32>(b[off + 1]) << 16) |
           (static_cast<u32>(b[off + 2]) << 8) | static_cast<u32>(b[off + 3]);
}

constexpr std::size_t kMdbOff = 0x400;  // MDB = logical block 2

// A short hexdump of the MDB, shown via doctest INFO() when a check fails.
std::string mdbHexdump(const std::vector<u8>& img) {
    std::string out = "MDB @0x400:\n";
    char line[80];
    for (std::size_t row = 0; row < 64u; row += 16u) {
        std::snprintf(line, sizeof(line), "%03zx:", row);
        out += line;
        for (std::size_t i = 0; i < 16u; ++i) {
            std::snprintf(line, sizeof(line), " %02x",
                          static_cast<unsigned>(img[kMdbOff + row + i]));
            out += line;
        }
        out += "\n";
    }
    return out;
}

// Read the drVN Pascal string (28 bytes at MDB+0x24) back into a std::string.
std::string readVolumeName(const std::vector<u8>& img) {
    const std::size_t vn = kMdbOff + 0x24;
    const u8 len = img[vn];
    return std::string(reinterpret_cast<const char*>(&img[vn + 1]), len);
}

} // namespace

TEST_CASE("formatVolume produces a mountable, empty 20 MB HFS volume") {
    const u32 sizeBytes = 20u * 1024 * 1024;
    const std::string volName = "OpenMac HD";
    std::vector<u8> img = hfs::formatVolume(sizeBytes, volName);

    INFO(mdbHexdump(img));

    // Exact requested length, multiple of 512.
    REQUIRE(img.size() == sizeBytes);

    // Boot blocks (logical blocks 0-1) are zeroed: non-bootable data volume.
    for (std::size_t i = 0; i < 1024u; ++i)
        CHECK(img[i] == 0);

    // --- Master Directory Block ------------------------------------------
    CHECK(rd16(img, kMdbOff + 0x00) == 0x4244);        // drSigWord 'BD'
    CHECK((rd16(img, kMdbOff + 0x0A) & 0x0100) != 0);  // drAtrb: unmounted cleanly
    CHECK(rd16(img, kMdbOff + 0x0E) == 3);             // drVBMSt
    CHECK(rd32(img, kMdbOff + 0x1E) == 16);            // drNxtCNID
    CHECK(rd16(img, kMdbOff + 0x0C) == 0);             // drNmFls (empty root)

    // Volume name round-trips.
    CHECK(readVolumeName(img) == volName);

    const u32 nmAlBlks = rd16(img, kMdbOff + 0x12);    // drNmAlBlks
    const u32 alBlkSiz = rd32(img, kMdbOff + 0x14);    // drAlBlkSiz
    const u32 alBlSt   = rd16(img, kMdbOff + 0x1C);    // drAlBlSt
    const u32 freeBks  = rd16(img, kMdbOff + 0x22);    // drFreeBks

    // Allocation-block count stays below the 16-bit ceiling and the volume
    // (allocation area + overhead) fits within the requested size.
    CHECK(nmAlBlks < 65536u);
    CHECK(alBlkSiz % 512u == 0);
    const std::size_t allocBytes = static_cast<std::size_t>(alBlSt) * 512u +
                                   static_cast<std::size_t>(nmAlBlks) * alBlkSiz;
    CHECK(allocBytes + 2u * 512u <= sizeBytes);  // + alt-MDB and reserved trailer

    // Free blocks == total minus the two B*-tree files' allocation.
    const u32 xtFlSize = rd32(img, kMdbOff + 0x82);    // drXTFlSize
    const u32 ctFlSize = rd32(img, kMdbOff + 0x92);    // drCTFlSize
    const u32 btAllocBlks = xtFlSize / alBlkSiz + ctFlSize / alBlkSiz;
    CHECK(freeBks == nmAlBlks - btAllocBlks);

    // The exact geometry chosen for a 20 MB volume (documented in the report).
    CHECK(alBlkSiz == 512u);
    CHECK(nmAlBlks == 40945u);
    CHECK(freeBks == 40942u);

    // Backup MDB (logical block vlen-2) is an exact copy of the primary MDB.
    const std::size_t vlen = sizeBytes / 512u;
    const std::size_t altOff = (vlen - 2) * 512u;
    bool altMatches = true;
    for (std::size_t i = 0; i < 512u; ++i)
        altMatches = altMatches && (img[altOff + i] == img[kMdbOff + i]);
    CHECK(altMatches);

    // --- locate the B*-tree files via the MDB extent records --------------
    const u32 lpa = alBlkSiz / 512u;
    const u32 xtFirstAB = rd16(img, kMdbOff + 0x86);   // drXTExtRec[0].xdrStABN
    const u32 ctFirstAB = rd16(img, kMdbOff + 0x96);   // drCTExtRec[0].xdrStABN
    const std::size_t xtHdr = static_cast<std::size_t>(alBlSt + xtFirstAB * lpa) * 512u;
    const std::size_t ctHdr = static_cast<std::size_t>(alBlSt + ctFirstAB * lpa) * 512u;
    const std::size_t ctLeaf = ctHdr + 512u;  // node 1

    // Both header nodes: ndType == header (0x01) and bthNodeSize == 512.
    CHECK(img[xtHdr + 8] == 0x01);
    CHECK(rd16(img, xtHdr + 0x0e + 18) == 512);        // extents bthNodeSize
    CHECK(img[ctHdr + 8] == 0x01);
    CHECK(rd16(img, ctHdr + 0x0e + 18) == 512);        // catalog bthNodeSize

    // Catalog header advertises a one-level tree rooted at node 1.
    CHECK(rd16(img, ctHdr + 0x0e + 0) == 1);           // bthDepth
    CHECK(rd32(img, ctHdr + 0x0e + 2) == 1);           // bthRoot
    CHECK(rd32(img, ctHdr + 0x0e + 6) == 2);           // bthNRecs (dir + thread)

    // --- catalog leaf node (the root directory) ---------------------------
    CHECK(img[ctLeaf + 8] == 0xFF);                    // ndType == leaf
    CHECK(rd16(img, ctLeaf + 10) == 2);                // ndNRecs == 2

    // First record's key: offset table's last entry (roff[0]) points to it.
    const u32 roff0 = rd16(img, ctLeaf + 512 - 2);
    CHECK(roff0 == 0x00e);
    // Catalog key layout: keyLen(1), reserved(1), parID(4) -> parID at +2.
    CHECK(rd32(img, ctLeaf + roff0 + 2) == 1);         // root dir key parID == ROOTPAR

    // Second record is the root-directory thread (parID == ROOTDIR).
    const u32 roff1 = rd16(img, ctLeaf + 512 - 4);
    CHECK(rd32(img, ctLeaf + roff1 + 2) == 2);

    // Volume bitmap (logical block 3): the used allocation blocks are marked.
    const std::size_t vbm = 3u * 512u;
    for (u32 ab = 0; ab < btAllocBlks; ++ab)
        CHECK((img[vbm + (ab >> 3)] & (0x80u >> (ab & 7u))) != 0);
    // The first block past the B*-trees is free.
    CHECK((img[vbm + (btAllocBlks >> 3)] & (0x80u >> (btAllocBlks & 7u))) == 0);
}

TEST_CASE("formatVolume name handling: clamp to 27 chars and default empty") {
    // A 4 MB volume with an over-long name is clamped to 27 characters.
    std::vector<u8> a = hfs::formatVolume(4u * 1024 * 1024,
                                          "This Name Is Definitely Way Too Long");
    CHECK(readVolumeName(a) == "This Name Is Definitely Way");  // 27 chars
    CHECK(rd16(a, kMdbOff + 0x00) == 0x4244);

    // An empty name defaults to "Untitled".
    std::vector<u8> b = hfs::formatVolume(4u * 1024 * 1024, "");
    CHECK(readVolumeName(b) == "Untitled");
}
