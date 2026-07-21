// Host-side HFS (Macintosh Hierarchical File System) volume formatter.
//
// This produces a valid, empty HFS volume image byte-for-byte, with no
// dependency on the emulator, on <ctime>, or on the host filesystem. The
// on-disk layout was cross-checked against Inside Macintosh vol. IV/V ("The
// File Manager") and Robert Leslie's libhfs (hfsutils): the MDB field packing
// mirrors libhfs low.c l_putmdb (whose own ASSERTs pin the field offsets to
// 64/124/146/162), the B*-tree node/record layout mirrors btree.c/node.c/
// record.c, and the volume geometry mirrors hfs.c hfs_format. All multibyte
// integers are big-endian; the logical block size is 512 bytes.

#include "openmac/hfs.hpp"

#include <array>

namespace openmac::hfs {
namespace {

constexpr u32 kBlockSize = 512;         // HFS logical block size
constexpr u16 kSigWord   = 0x4244;      // 'BD' — HFS volume signature (drSigWord)
constexpr u16 kAtrbUnmounted = 0x0100;  // drAtrb bit 8: volume was unmounted cleanly

// Catalog node IDs (CNIDs) reserved by the File Manager.
constexpr u32 kCnidRootPar = 1;   // parent of the root directory
constexpr u32 kCnidRootDir = 2;   // the root directory itself
constexpr u32 kFirstCnid   = 16;  // first CNID available to new files/dirs

// B*-tree node descriptor type codes (ndType).
constexpr u8 kNdHeader = 0x01;
constexpr u8 kNdLeaf   = 0xFF;

// Catalog record data type codes (cdrType).
constexpr u8 kCdrDir    = 1;
constexpr u8 kCdrThread = 3;

// Header-node record offsets, fixed by the format (see libhfs bt_readhdr):
// record 0 = BTHdrRec (106 bytes), record 1 = reserved (128 bytes),
// record 2 = node-allocation bitmap (256 bytes).
constexpr u16 kHdrRoff0 = 0x00e;
constexpr u16 kHdrRoff1 = 0x078;
constexpr u16 kHdrRoff2 = 0x0f8;
constexpr u16 kHdrRoff3 = 0x1f8;

// A fixed, valid Macintosh timestamp: seconds since the HFS epoch of
// 1904-01-01 00:00:00. This value is ~1996; it is a compile-time constant so
// the formatter never calls a time function (and output is reproducible).
constexpr u32 kFixedDate = 2903299200u;

// Extents-overflow tree keys are always 7 bytes; catalog keys are at most
// 0x25 (a 31-character name: 5 + (31+1) + 0).
constexpr u16 kExtKeyLen = 0x07;
constexpr u16 kCatKeyLen = 0x25;

// --- big-endian stores into a raw block ---------------------------------

void put16(u8* p, u16 v) {
    p[0] = static_cast<u8>(v >> 8);
    p[1] = static_cast<u8>(v);
}

void put32(u8* p, u32 v) {
    p[0] = static_cast<u8>(v >> 24);
    p[1] = static_cast<u8>(v >> 16);
    p[2] = static_cast<u8>(v >> 8);
    p[3] = static_cast<u8>(v);
}

// A cursor that appends big-endian fields to a byte buffer. Used to build
// variable-length catalog records before they are placed into a node.
struct Writer {
    std::vector<u8> data;

    void u8v(u8 v) { data.push_back(v); }
    void u16v(u16 v) {
        data.push_back(static_cast<u8>(v >> 8));
        data.push_back(static_cast<u8>(v));
    }
    void u32v(u32 v) {
        data.push_back(static_cast<u8>(v >> 24));
        data.push_back(static_cast<u8>(v >> 16));
        data.push_back(static_cast<u8>(v >> 8));
        data.push_back(static_cast<u8>(v));
    }
    void zeros(std::size_t n) { data.insert(data.end(), n, static_cast<u8>(0)); }
    void str(const std::string& s) {
        data.insert(data.end(), s.begin(), s.end());
    }
};

// Build a packed catalog record key: {keyLen, reserved, parID, CName}.
// The key length byte counts every byte that follows it, padded so the record
// data begins on an even boundary (libhfs r_makecatkey / r_packcatkey).
Writer catKey(u32 parID, const std::string& name) {
    const u32 nameLen = static_cast<u32>(name.size());
    const u32 cnamePascal = 1 + nameLen;                 // length byte + chars
    const u32 keyLen = 5 + cnamePascal + (cnamePascal & 1);

    Writer w;
    w.u8v(static_cast<u8>(keyLen));  // ckrKeyLen
    w.u8v(0);                        // ckrResrv1
    w.u32v(parID);                   // ckrParID
    w.u8v(static_cast<u8>(nameLen)); // CName length byte
    w.str(name);                     // CName characters
    w.zeros(cnamePascal & 1);        // pad to make (1 + keyLen) even
    return w;                        // total emitted == 1 + keyLen (even)
}

// Root-directory record data (cdrDirRec), 70 bytes.
void appendDirData(Writer& w, u32 dirID) {
    w.u8v(kCdrDir);      // cdrType
    w.u8v(0);            // cdrResrv2
    w.u16v(0);           // dirFlags
    w.u16v(0);           // dirVal (valence: empty directory)
    w.u32v(dirID);       // dirDirID
    w.u32v(kFixedDate);  // dirCrDat
    w.u32v(kFixedDate);  // dirMdDat
    w.u32v(0);           // dirBkDat
    w.zeros(16);         // DInfo  (Finder info, zeroed)
    w.zeros(16);         // DXInfo (additional Finder info, zeroed)
    w.zeros(16);         // dirResrv[4]
}

// Root-directory thread record data (cdrThdRec), 46 bytes.
void appendThreadData(Writer& w, u32 parID, const std::string& name) {
    w.u8v(kCdrThread);                     // cdrType
    w.u8v(0);                              // cdrResrv2
    w.zeros(8);                            // thdResrv[2]
    w.u32v(parID);                         // thdParID
    w.u8v(static_cast<u8>(name.size()));   // thdCName length byte
    w.str(name);                           // thdCName characters
    w.zeros(31u - name.size());            // pad the Str31 field to 32 bytes total
}

// Construct a B*-tree header node (node 0) for an empty/initial tree.
// usedNodes is the number of leading nodes marked in-use in the node bitmap
// (1 for the extents tree = header only; 2 for the catalog tree = header+leaf).
std::array<u8, kBlockSize> buildHeaderNode(u16 depth, u32 root, u32 nRecs,
                                           u32 firstLeaf, u32 lastLeaf,
                                           u16 keyLen, u32 totalNodes,
                                           u32 freeNodes, u32 usedNodes) {
    std::array<u8, kBlockSize> nd{};
    u8* b = nd.data();

    // NodeDescriptor
    put32(b + 0, 0);              // ndFLink
    put32(b + 4, 0);              // ndBLink
    b[8] = kNdHeader;             // ndType
    b[9] = 0;                     // ndNHeight (header nodes are level 0)
    put16(b + 10, 3);             // ndNRecs (header always has 3 records)
    put16(b + 12, 0);             // ndResv2

    // Record 0: BTHdrRec
    u8* h = b + kHdrRoff0;
    put16(h + 0, depth);          // bthDepth
    put32(h + 2, root);           // bthRoot
    put32(h + 6, nRecs);          // bthNRecs (number of leaf records in tree)
    put32(h + 10, firstLeaf);     // bthFNode
    put32(h + 14, lastLeaf);      // bthLNode
    put16(h + 18, kBlockSize);    // bthNodeSize
    put16(h + 20, keyLen);        // bthKeyLen
    put32(h + 22, totalNodes);    // bthNNodes
    put32(h + 26, freeNodes);     // bthFree
    // bthResv[76] stays zero.

    // Record 1 (reserved, 128 bytes) stays zero.

    // Record 2: node-allocation bitmap. Bit n (MSB-first within each byte)
    // marks node n as in use.
    u8* map = b + kHdrRoff2;
    for (u32 n = 0; n < usedNodes; ++n)
        map[n >> 3] |= static_cast<u8>(0x80u >> (n & 7u));

    // Record offset table: 4 entries (ndNRecs+1) at the end of the node,
    // stored in reverse so roff[0] occupies the final two bytes.
    put16(b + kBlockSize - 2, kHdrRoff0);
    put16(b + kBlockSize - 4, kHdrRoff1);
    put16(b + kBlockSize - 6, kHdrRoff2);
    put16(b + kBlockSize - 8, kHdrRoff3);

    return nd;
}

} // namespace

std::vector<u8> formatVolume(u32 sizeBytes, const std::string& volumeName) {
    // Clamp the volume name to the HFS limit of 27 characters.
    std::string name = volumeName;
    if (name.empty())
        name = "Untitled";
    if (name.size() > 27)
        name.resize(27);

    std::vector<u8> img(sizeBytes, 0);

    // --- volume geometry (mirrors libhfs hfs_format) --------------------
    const u32 vlen = sizeBytes / kBlockSize;               // logical blocks
    const u32 lpa  = 1 + ((vlen - 6) >> 16);               // log. blocks / alloc block
    const u32 alBlkSiz = lpa * kBlockSize;                 // allocation block size
    const u32 vbmSt = 3;                                   // bitmap starts at block 3
    const u32 vbmSz = (vlen / lpa + 0x0fffu) >> 12;        // bitmap length, in blocks
    const u32 alBlSt = vbmSt + vbmSz;                      // first alloc block's log. block
    const u32 nmAlBlks = (vlen - 2 - alBlSt) / lpa;        // number of allocation blocks

    // The extents and catalog B*-tree files, sized minimally: the extents file
    // holds just its header node; the catalog file holds its header node plus
    // one leaf node (the root directory). Files are whole numbers of allocation
    // blocks, laid out contiguously from allocation block 0.
    const u32 nodesPerAB = lpa;                            // 512-byte nodes per alloc block
    const u32 extAllocBlks = 1;
    const u32 catAllocBlks = (2 + nodesPerAB - 1) / nodesPerAB;
    const u32 extNodes = extAllocBlks * nodesPerAB;
    const u32 catNodes = catAllocBlks * nodesPerAB;

    const u32 extFirstAB = 0;
    const u32 catFirstAB = extAllocBlks;
    const u32 usedAllocBlks = extAllocBlks + catAllocBlks;
    const u32 freeBks = nmAlBlks - usedAllocBlks;

    const u32 xtFlSize = extAllocBlks * alBlkSiz;
    const u32 ctFlSize = catAllocBlks * alBlkSiz;

    // Clump size (growth increment) — a non-zero multiple of the alloc block
    // size, as libhfs computes it.
    u32 clump = (nmAlBlks / 128u) * alBlkSiz;
    if (clump < alBlkSiz)
        clump = alBlkSiz * 4u;

    // Logical-block position of each file's first node.
    const u32 extFirstBlk = alBlSt + extFirstAB * lpa;
    const u32 catFirstBlk = alBlSt + catFirstAB * lpa;

    // --- boot blocks (logical blocks 0-1): left zeroed --------------------

    // --- Master Directory Block (logical block 2) -------------------------
    std::array<u8, kBlockSize> mdb{};
    {
        std::size_t p = 0;
        u8* b = mdb.data();
        auto w16 = [&](u16 v) { put16(b + p, v); p += 2; };
        auto w32 = [&](u32 v) { put32(b + p, v); p += 4; };

        w16(kSigWord);                 // drSigWord   @0x00
        w32(kFixedDate);               // drCrDate    @0x02
        w32(kFixedDate);               // drLsMod     @0x06
        w16(kAtrbUnmounted);           // drAtrb      @0x0A
        w16(0);                        // drNmFls     @0x0C
        w16(static_cast<u16>(vbmSt));  // drVBMSt     @0x0E
        w16(static_cast<u16>(usedAllocBlks)); // drAllocPtr @0x10 (next free AB hint)
        w16(static_cast<u16>(nmAlBlks));      // drNmAlBlks @0x12
        w32(alBlkSiz);                 // drAlBlkSiz  @0x14
        w32(alBlkSiz << 2);            // drClpSiz    @0x18
        w16(static_cast<u16>(alBlSt)); // drAlBlSt    @0x1C
        w32(kFirstCnid);               // drNxtCNID   @0x1E
        w16(static_cast<u16>(freeBks));// drFreeBks   @0x22

        // drVN: volume name as a Pascal string in 28 bytes.  @0x24
        const std::size_t vnStart = p;
        b[p++] = static_cast<u8>(name.size());
        for (char c : name)
            b[p++] = static_cast<u8>(c);
        p = vnStart + 28;              // remainder already zero

        w32(0);                        // drVolBkUp   @0x40
        w16(0);                        // drVSeqNum   @0x44
        w32(0);                        // drWrCnt     @0x46
        w32(clump);                    // drXTClpSiz  @0x4A
        w32(clump);                    // drCTClpSiz  @0x4E
        w16(0);                        // drNmRtDirs  @0x52
        w32(0);                        // drFilCnt    @0x54
        w32(0);                        // drDirCnt    @0x58
        p += 32;                       // drFndrInfo[8] (zeroed) @0x5C -> 0x7C
        w16(0);                        // drEmbedSigWord         @0x7C
        w16(0);                        // drEmbedExtent.xdrStABN @0x7E
        w16(0);                        // drEmbedExtent.xdrNumABlks @0x80

        w32(xtFlSize);                 // drXTFlSize  @0x82
        w16(static_cast<u16>(extFirstAB));   // drXTExtRec[0].start
        w16(static_cast<u16>(extAllocBlks)); // drXTExtRec[0].count
        w16(0); w16(0);                // drXTExtRec[1]
        w16(0); w16(0);                // drXTExtRec[2]

        w32(ctFlSize);                 // drCTFlSize  @0x92
        w16(static_cast<u16>(catFirstAB));   // drCTExtRec[0].start
        w16(static_cast<u16>(catAllocBlks)); // drCTExtRec[0].count
        w16(0); w16(0);                // drCTExtRec[1]
        w16(0); w16(0);                // drCTExtRec[2]
        // p is now 162 (0xA2); the rest of the block stays zero.
    }
    // The MDB lives at logical block 2, and a backup copy at block vlen-2.
    std::copy(mdb.begin(), mdb.end(), img.begin() + 2 * kBlockSize);
    std::copy(mdb.begin(), mdb.end(),
              img.begin() + static_cast<std::size_t>(vlen - 2) * kBlockSize);

    // --- volume bitmap (starts at logical block 3) ------------------------
    // Mark every allocation block consumed by the two B*-tree files. Bits are
    // MSB-first within each byte, matching libhfs BMSET.
    {
        u8* vbm = img.data() + static_cast<std::size_t>(vbmSt) * kBlockSize;
        for (u32 ab = 0; ab < usedAllocBlks; ++ab)
            vbm[ab >> 3] |= static_cast<u8>(0x80u >> (ab & 7u));
    }

    // --- extents-overflow B*-tree file: one header node, empty tree -------
    {
        auto node = buildHeaderNode(/*depth*/ 0, /*root*/ 0, /*nRecs*/ 0,
                                    /*firstLeaf*/ 0, /*lastLeaf*/ 0,
                                    kExtKeyLen, /*totalNodes*/ extNodes,
                                    /*freeNodes*/ extNodes - 1, /*usedNodes*/ 1);
        std::copy(node.begin(), node.end(),
                  img.begin() + static_cast<std::size_t>(extFirstBlk) * kBlockSize);
    }

    // --- catalog B*-tree file: header node + one leaf node ----------------
    {
        // Header node (node 0): a one-level tree whose root is the leaf (node 1).
        auto header = buildHeaderNode(/*depth*/ 1, /*root*/ 1, /*nRecs*/ 2,
                                      /*firstLeaf*/ 1, /*lastLeaf*/ 1,
                                      kCatKeyLen, /*totalNodes*/ catNodes,
                                      /*freeNodes*/ catNodes - 2, /*usedNodes*/ 2);
        std::copy(header.begin(), header.end(),
                  img.begin() + static_cast<std::size_t>(catFirstBlk) * kBlockSize);

        // Leaf node (node 1): the root directory record and its thread record,
        // ordered by key (parID 1 sorts before parID 2).
        std::array<u8, kBlockSize> leaf{};
        u8* b = leaf.data();
        put32(b + 0, 0);       // ndFLink
        put32(b + 4, 0);       // ndBLink
        b[8] = kNdLeaf;        // ndType
        b[9] = 1;              // ndNHeight (leaf level)
        put16(b + 10, 2);      // ndNRecs
        put16(b + 12, 0);      // ndResv2

        // Record 0: root directory record. key {parID=ROOTPAR, CName=name}.
        Writer rec0 = catKey(kCnidRootPar, name);
        appendDirData(rec0, kCnidRootDir);

        // Record 1: root directory thread. key {parID=ROOTDIR, CName=""}.
        Writer rec1 = catKey(kCnidRootDir, std::string());
        appendThreadData(rec1, kCnidRootPar, name);

        const u16 off0 = kHdrRoff0;  // 0x00e: first record
        const u16 off1 = static_cast<u16>(off0 + rec0.data.size());
        const u16 off2 = static_cast<u16>(off1 + rec1.data.size());

        std::copy(rec0.data.begin(), rec0.data.end(), b + off0);
        std::copy(rec1.data.begin(), rec1.data.end(), b + off1);

        // Offset table: 3 entries (ndNRecs+1) at the end, reversed.
        put16(b + kBlockSize - 2, off0);
        put16(b + kBlockSize - 4, off1);
        put16(b + kBlockSize - 6, off2);

        std::copy(leaf.begin(), leaf.end(),
                  img.begin() + static_cast<std::size_t>(catFirstBlk + 1) * kBlockSize);
    }

    return img;
}

} // namespace openmac::hfs
