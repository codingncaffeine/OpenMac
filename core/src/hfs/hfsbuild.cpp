// Host-side builder for POPULATED HFS volumes — the folder-disk path.
//
// Same clean-room grounding as the formatter (Inside Macintosh IV/V "The File
// Manager", cross-checked against libhfs's field packing), extended to a real
// catalog: multi-leaf B*-tree with index levels, file records with both forks,
// Finder info, dates. Layout strategy keeps everything simple and robust:
//
//   - every fork is allocated CONTIGUOUSLY, so each fits its record's first
//     extent and the extents-overflow tree stays empty;
//   - catalog nodes are laid out header, then leaves in key order, then index
//     levels bottom-up, with spare free nodes so the guest can add files;
//   - names are canonicalized to an ASCII subset on which our key order and
//     the ROM's RelString collation agree (a mis-sorted key is a file the
//     guest can never find, so the subset is enforced, not trusted).

#include "openmac/hfs.hpp"

#include <algorithm>
#include <array>
#include <cstring>

namespace openmac::hfs {
namespace {

constexpr u32 kBlockSize = 512;
constexpr u16 kSigWord = 0x4244;
constexpr u16 kAtrbUnmounted = 0x0100;
constexpr u32 kCnidRootPar = 1;
constexpr u32 kCnidRootDir = 2;
constexpr u32 kFirstCnid = 16;
constexpr u8 kNdIndex = 0x00;
constexpr u8 kNdHeader = 0x01;
constexpr u8 kNdLeaf = 0xFF;
constexpr u8 kCdrDir = 1;
constexpr u8 kCdrFil = 2;
constexpr u8 kCdrThread = 3;
constexpr u16 kHdrRoff0 = 0x00e;
constexpr u16 kHdrRoff1 = 0x078;
constexpr u16 kHdrRoff2 = 0x0f8;
constexpr u16 kHdrRoff3 = 0x1f8;
constexpr u32 kFixedDate = 2903299200u;   // ~1996, HFS epoch seconds
constexpr u16 kExtKeyLen = 0x07;
constexpr u16 kCatKeyLen = 0x25;
// One header-node allocation map (256 bytes) covers this many nodes; staying
// under it means never emitting map continuation nodes.
constexpr u32 kMaxCatNodes = 256u * 8u;

void put16(u8* p, u16 v) { p[0] = u8(v >> 8); p[1] = u8(v); }
void put32(u8* p, u32 v) {
    p[0] = u8(v >> 24); p[1] = u8(v >> 16); p[2] = u8(v >> 8); p[3] = u8(v);
}

struct Writer {
    std::vector<u8> data;
    void u8v(u8 v) { data.push_back(v); }
    void u16v(u16 v) { data.push_back(u8(v >> 8)); data.push_back(u8(v)); }
    void u32v(u32 v) {
        data.push_back(u8(v >> 24)); data.push_back(u8(v >> 16));
        data.push_back(u8(v >> 8)); data.push_back(u8(v));
    }
    void zeros(std::size_t n) { data.insert(data.end(), n, u8(0)); }
    void str(const std::string& s) { data.insert(data.end(), s.begin(), s.end()); }
};

Writer catKey(u32 parID, const std::string& name) {
    const u32 nameLen = static_cast<u32>(name.size());
    const u32 cnamePascal = 1 + nameLen;
    const u32 keyLen = 5 + cnamePascal + (cnamePascal & 1);
    Writer w;
    w.u8v(u8(keyLen));
    w.u8v(0);
    w.u32v(parID);
    w.u8v(u8(nameLen));
    w.str(name);
    w.zeros(cnamePascal & 1);
    return w;
}

// The safe subset: characters whose case-folded ASCII order matches the Mac's
// own collation. Everything else becomes '_' before it enters a key.
bool safeChar(u8 c) {
    if (c >= 'A' && c <= 'Z') return true;
    if (c >= 'a' && c <= 'z') return true;
    if (c >= '0' && c <= '9') return true;
    switch (c) {
        case ' ': case '!': case '#': case '$': case '%': case '&': case '\'':
        case '(': case ')': case '+': case ',': case '-': case '.': case '@':
        case '_': case '~':
            return true;
        default:
            return false;
    }
}

std::string canonicalName(const std::string& in) {
    std::string out;
    for (char ch : in) {
        u8 c = static_cast<u8>(ch);
        out.push_back(safeChar(c) ? static_cast<char>(c) : '_');
        if (out.size() == 31) break;
    }
    if (out.empty()) out = "_";
    // HFS forbids ':' (the path separator); the subset already excludes it.
    return out;
}

u8 fold(u8 c) { return (c >= 'a' && c <= 'z') ? u8(c - 'a' + 'A') : c; }

// Catalog key order: parent id, then case-folded name.
int compareKeys(u32 parA, const std::string& a, u32 parB, const std::string& b) {
    if (parA != parB) return parA < parB ? -1 : 1;
    const std::size_t n = std::min(a.size(), b.size());
    for (std::size_t i = 0; i < n; ++i) {
        const u8 fa = fold(u8(a[i])), fb = fold(u8(b[i]));
        if (fa != fb) return fa < fb ? -1 : 1;
    }
    if (a.size() != b.size()) return a.size() < b.size() ? -1 : 1;
    return 0;
}

struct Entry {
    bool isDir = false;
    u32 id = 0, parent = 0;
    std::string name;
    u32 type = 0, creator = 0;
    u16 fdFlags = 0;
    u32 cr = kFixedDate, md = kFixedDate;
    std::vector<u8> data, rsrc;
    u16 valence = 0;                       // dirs: children count
    u16 dataStart = 0, dataABs = 0;        // layout results, allocation blocks
    u16 rsrcStart = 0, rsrcABs = 0;
};

struct Record {
    u32 parID;
    std::string name;      // key name ("" for threads)
    std::vector<u8> bytes; // packed key + data
};

std::array<u8, kBlockSize> headerNode(u16 depth, u32 root, u32 nRecs, u32 firstLeaf,
                                      u32 lastLeaf, u16 keyLen, u32 totalNodes,
                                      u32 usedNodes) {
    std::array<u8, kBlockSize> nd{};
    u8* b = nd.data();
    b[8] = kNdHeader;
    put16(b + 10, 3);
    u8* h = b + kHdrRoff0;
    put16(h + 0, depth);
    put32(h + 2, root);
    put32(h + 6, nRecs);
    put32(h + 10, firstLeaf);
    put32(h + 14, lastLeaf);
    put16(h + 18, kBlockSize);
    put16(h + 20, keyLen);
    put32(h + 22, totalNodes);
    put32(h + 26, totalNodes - usedNodes);
    u8* map = b + kHdrRoff2;
    for (u32 n = 0; n < usedNodes; ++n) map[n >> 3] |= u8(0x80u >> (n & 7u));
    put16(b + kBlockSize - 2, kHdrRoff0);
    put16(b + kBlockSize - 4, kHdrRoff1);
    put16(b + kBlockSize - 6, kHdrRoff2);
    put16(b + kBlockSize - 8, kHdrRoff3);
    return nd;
}

// Pack records greedily into 512-byte nodes: 14-byte descriptor, records from
// 0x0E, offset table (2 bytes per record + sentinel) growing from the end.
std::vector<std::vector<const Record*>> packNodes(const std::vector<Record>& recs) {
    std::vector<std::vector<const Record*>> nodes;
    std::vector<const Record*> cur;
    std::size_t used = 0;
    for (const Record& r : recs) {
        const std::size_t need = r.bytes.size();
        if (!cur.empty() &&
            14 + used + need + 2 * (cur.size() + 2) > kBlockSize) {
            nodes.push_back(cur);
            cur.clear();
            used = 0;
        }
        cur.push_back(&r);
        used += need;
    }
    if (!cur.empty()) nodes.push_back(cur);
    return nodes;
}

void serializeNode(u8* b, u8 type, u8 height, u32 fLink, u32 bLink,
                   const std::vector<const Record*>& recs) {
    put32(b + 0, fLink);
    put32(b + 4, bLink);
    b[8] = type;
    b[9] = height;
    put16(b + 10, u16(recs.size()));
    u16 off = kHdrRoff0;
    std::size_t slot = 0;
    for (const Record* r : recs) {
        std::memcpy(b + off, r->bytes.data(), r->bytes.size());
        put16(b + kBlockSize - 2 - 2 * slot, off);
        off = u16(off + r->bytes.size());
        ++slot;
    }
    put16(b + kBlockSize - 2 - 2 * slot, off);   // free-space sentinel
}

} // namespace

struct VolumeBuilder::Impl {
    std::string volName;
    std::vector<Entry> entries;   // [0] is the root
    u32 nextId = kFirstCnid;
    std::string error;

    Entry* find(u32 id) {
        for (Entry& e : entries)
            if (e.id == id) return &e;
        return nullptr;
    }
    bool nameTaken(u32 parent, const std::string& name) {
        for (const Entry& e : entries)
            if (e.parent == parent &&
                compareKeys(0, e.name, 0, name) == 0 && e.id != kCnidRootDir)
                return true;
        return false;
    }

    // Canonicalization can fold two host names into one Mac name; number the
    // later arrivals ("name~2", "name~3", ...) within the 31-character limit.
    std::string uniqueName(u32 parent, std::string name) {
        int n = 2;
        const std::string base = name;
        while (nameTaken(parent, name)) {
            const std::string suffix = "~" + std::to_string(n++);
            std::string b = base;
            if (b.size() + suffix.size() > 31) b.resize(31 - suffix.size());
            name = b + suffix;
        }
        return name;
    }
};

VolumeBuilder::VolumeBuilder(const std::string& volumeName) : impl_(new Impl) {
    std::string name = canonicalName(volumeName.empty() ? "Untitled" : volumeName);
    if (name.size() > 27) name.resize(27);
    impl_->volName = name;
    Entry root;
    root.isDir = true;
    root.id = kCnidRootDir;
    root.parent = kCnidRootPar;
    root.name = name;
    impl_->entries.push_back(std::move(root));
}

VolumeBuilder::~VolumeBuilder() { delete impl_; }

const std::string& VolumeBuilder::why() const { return impl_->error; }

u32 VolumeBuilder::addDir(u32 parent, const std::string& name, u32 crDate, u32 mdDate) {
    Entry* par = impl_->find(parent);
    if (!par || !par->isDir) { impl_->error = "no such parent directory"; return 0; }
    Entry e;
    e.isDir = true;
    e.id = impl_->nextId++;
    e.parent = parent;
    e.name = impl_->uniqueName(parent, canonicalName(name));
    if (crDate) e.cr = crDate;
    if (mdDate) e.md = mdDate;
    par->valence++;
    impl_->entries.push_back(std::move(e));
    return impl_->entries.back().id;
}

void VolumeBuilder::addFile(u32 parent, const std::string& name, u32 type,
                            u32 creator, u16 fdFlags, std::vector<u8> dataFork,
                            std::vector<u8> rsrcFork, u32 crDate, u32 mdDate) {
    Entry* par = impl_->find(parent);
    if (!par || !par->isDir) { impl_->error = "no such parent directory"; return; }
    Entry e;
    e.id = impl_->nextId++;
    e.parent = parent;
    e.name = impl_->uniqueName(parent, canonicalName(name));
    e.type = type;
    e.creator = creator;
    e.fdFlags = fdFlags;
    e.data = std::move(dataFork);
    e.rsrc = std::move(rsrcFork);
    if (crDate) e.cr = crDate;
    if (mdDate) e.md = mdDate;
    par->valence++;
    impl_->entries.push_back(std::move(e));
}

std::vector<u8> VolumeBuilder::build(u32 sizeBytes) {
    Impl& im = *impl_;
    im.error.clear();

    // ---- catalog records, in key order ---------------------------------
    std::vector<Record> recs;
    for (const Entry& e : im.entries) {
        if (e.isDir) {
            Record dir;
            dir.parID = e.parent;
            dir.name = e.name;
            Writer w = catKey(e.parent, e.name);
            w.u8v(kCdrDir);
            w.u8v(0);
            w.u16v(0);              // dirFlags
            w.u16v(e.valence);
            w.u32v(e.id);
            w.u32v(e.cr);
            w.u32v(e.md);
            w.u32v(0);              // dirBkDat
            w.zeros(16);            // DInfo
            w.zeros(16);            // DXInfo
            w.zeros(16);            // dirResrv
            dir.bytes = std::move(w.data);
            recs.push_back(std::move(dir));

            Record thd;             // every directory carries a thread record
            thd.parID = e.id;
            thd.name.clear();
            Writer t = catKey(e.id, std::string());
            t.u8v(kCdrThread);
            t.u8v(0);
            t.zeros(8);
            t.u32v(e.parent);
            t.u8v(u8(e.name.size()));
            t.str(e.name);
            t.zeros(31u - e.name.size());
            thd.bytes = std::move(t.data);
            recs.push_back(std::move(thd));
        }
    }
    // Files get their layout later; keep pointers to patch extents in.
    struct FilePatch { const Entry* e; std::size_t recIndex; };
    std::vector<FilePatch> patches;
    for (const Entry& e : im.entries) {
        if (e.isDir) continue;
        Record f;
        f.parID = e.parent;
        f.name = e.name;
        Writer w = catKey(e.parent, e.name);
        w.u8v(kCdrFil);
        w.u8v(0);
        w.u8v(0);                          // filFlags
        w.u8v(0);                          // filTyp
        w.u32v(e.type);                    // FInfo.fdType
        w.u32v(e.creator);                 // FInfo.fdCreator
        w.u16v(e.fdFlags);                 // FInfo.fdFlags
        w.u32v(0);                         // FInfo.fdLocation
        w.u16v(0);                         // FInfo.fdFldr
        w.u32v(e.id);                      // filFlNum
        w.u16v(0);                         // filStBlk   (patched)
        w.u32v(u32(e.data.size()));        // filLgLen
        w.u32v(0);                         // filPyLen   (patched)
        w.u16v(0);                         // filRStBlk  (patched)
        w.u32v(u32(e.rsrc.size()));        // filRLgLen
        w.u32v(0);                         // filRPyLen  (patched)
        w.u32v(e.cr);
        w.u32v(e.md);
        w.u32v(0);                         // filBkDat
        w.zeros(16);                       // FXInfo
        w.u16v(0);                         // filClpSize
        w.zeros(12);                       // filExtRec  (patched)
        w.zeros(12);                       // filRExtRec (patched)
        w.u32v(0);                         // filResrv
        f.bytes = std::move(w.data);
        recs.push_back(std::move(f));
        patches.push_back({&e, recs.size() - 1});
    }
    std::sort(recs.begin(), recs.end(), [](const Record& a, const Record& b) {
        return compareKeys(a.parID, a.name, b.parID, b.name) < 0;
    });
    // Re-locate the file records after the sort (record identity = key).
    for (FilePatch& p : patches) {
        for (std::size_t i = 0; i < recs.size(); ++i)
            if (recs[i].parID == p.e->parent && recs[i].name == p.e->name &&
                recs[i].bytes.size() > 6 &&
                recs[i].bytes[1 + recs[i].bytes[0]] == kCdrFil) {
                p.recIndex = i;
                break;
            }
    }

    // ---- catalog tree shape --------------------------------------------
    const auto leaves = packNodes(recs);
    const u32 nLeaves = u32(leaves.size());
    // Index levels: level[i] = (first key, child node number). Node numbers:
    // header 0, leaves 1..nLeaves, then index levels bottom-up.
    struct IndexLevel { std::vector<Record> recs; std::vector<std::vector<const Record*>> nodes; };
    std::vector<IndexLevel> levels;
    {
        u32 childFirst = 1;
        const std::vector<std::vector<const Record*>>* children = &leaves;
        while (children->size() > 1) {
            IndexLevel lvl;
            u32 nodeNum = childFirst;
            for (const auto& child : *children) {
                const Record* first = child.front();
                Record ix;
                ix.parID = first->parID;
                ix.name = first->name;
                // Index record = the child's first key + the child node number.
                const u8 keyTotal = u8(1 + first->bytes[0]);
                ix.bytes.assign(first->bytes.begin(), first->bytes.begin() + keyTotal);
                ix.bytes.resize(keyTotal + 4);
                put32(ix.bytes.data() + keyTotal, nodeNum);
                lvl.recs.push_back(std::move(ix));
                ++nodeNum;
            }
            lvl.nodes = packNodes(lvl.recs);
            childFirst += u32(children->size());
            levels.push_back(std::move(lvl));
            children = &levels.back().nodes;
        }
    }
    u32 catUsed = 1 + nLeaves;
    for (const auto& lvl : levels) catUsed += u32(lvl.nodes.size());
    const u16 depth = u16(1 + levels.size());
    const u32 rootNode = nLeaves <= 1 ? 1 : catUsed - 1;
    if (catUsed > kMaxCatNodes) {
        im.error = "too many items for a v1 folder disk (catalog needs " +
                   std::to_string(catUsed) + " nodes, cap " +
                   std::to_string(kMaxCatNodes) + ") — split the folder";
        return {};
    }
    // Spare nodes so the guest can create files without growing the file.
    u32 catNodes = std::min(kMaxCatNodes, std::max(catUsed * 2, catUsed + 16));

    // ---- geometry: find a size whose allocation blocks fit everything ---
    u32 counts[3] = {0, 0, 0};   // filled per size attempt: ext ABs, cat ABs, fork ABs
    u32 vlen = 0, lpa = 0, alBlkSiz = 0, vbmSt = 3, vbmSz = 0, alBlSt = 0, nmAlBlks = 0;
    auto tryGeometry = [&](u32 bytes) -> bool {
        vlen = bytes / kBlockSize;
        if (vlen < 100) return false;
        lpa = 1 + ((vlen - 6) >> 16);
        alBlkSiz = lpa * kBlockSize;
        vbmSz = (vlen / lpa + 0x0fffu) >> 12;
        alBlSt = vbmSt + vbmSz;
        if (vlen < alBlSt + 2 + lpa) return false;
        nmAlBlks = (vlen - 2 - alBlSt) / lpa;
        if (nmAlBlks > 0xFFF0) return false;   // u16 field
        const u32 nodesPerAB = lpa;
        const u32 extABs = std::max(1u, 8u / nodesPerAB + (8u % nodesPerAB ? 1u : 0u));
        const u32 catABs = (catNodes + nodesPerAB - 1) / nodesPerAB;
        u32 forkABs = 0;
        for (const Entry& e : im.entries) {
            if (e.isDir) continue;
            forkABs += u32((e.data.size() + alBlkSiz - 1) / alBlkSiz);
            forkABs += u32((e.rsrc.size() + alBlkSiz - 1) / alBlkSiz);
        }
        counts[0] = extABs;
        counts[1] = catABs;
        counts[2] = forkABs;
        // Auto-sized volumes keep a fifth free for the guest to write into. An
        // explicitly-sized volume is packed tight (a write-protected transfer
        // floppy has no use for headroom) with just a safety floor.
        const u32 headroom = sizeBytes ? 16u : nmAlBlks / 5 + 4;
        return extABs + catABs + forkABs + headroom <= nmAlBlks;
    };
    if (sizeBytes) {
        if (sizeBytes % kBlockSize || !tryGeometry(sizeBytes)) {
            im.error = "the requested size does not fit the content";
            return {};
        }
    } else {
        u64 content = 0;
        for (const Entry& e : im.entries)
            if (!e.isDir) content += e.data.size() + e.rsrc.size() + 1024;
        u64 auto64 = content + content / 4 + 8u * 1024 * 1024;
        auto64 = (auto64 + kBlockSize - 1) / kBlockSize * kBlockSize;
        if (auto64 > 0x7F000000ull) { im.error = "content exceeds the 2 GB HFS ceiling"; return {}; }
        u32 sz = u32(auto64);
        while (!tryGeometry(sz)) {
            if (sz > 0x7F000000u) { im.error = "content exceeds the 2 GB HFS ceiling"; return {}; }
            sz += std::max(sz / 4, 4u * 1024 * 1024);
        }
        sizeBytes = sz;
    }
    const u32 extABs = counts[0], catABs = counts[1];
    catNodes = catABs * lpa;   // the file rounds up to whole allocation blocks

    // ---- allocation-block layout ---------------------------------------
    u32 nextAB = 0;
    const u32 extFirstAB = nextAB; nextAB += extABs;
    const u32 catFirstAB = nextAB; nextAB += catABs;
    for (FilePatch& p : patches) {
        Entry& e = const_cast<Entry&>(*p.e);
        if (!e.data.empty()) {
            e.dataStart = u16(nextAB);
            e.dataABs = u16((e.data.size() + alBlkSiz - 1) / alBlkSiz);
            nextAB += e.dataABs;
        }
        if (!e.rsrc.empty()) {
            e.rsrcStart = u16(nextAB);
            e.rsrcABs = u16((e.rsrc.size() + alBlkSiz - 1) / alBlkSiz);
            nextAB += e.rsrcABs;
        }
    }
    const u32 usedABs = nextAB;
    const u32 freeBks = nmAlBlks - usedABs;

    // Patch layout into the file records. Field offsets inside the record
    // data: type(1) resv(1) flags(1) typ(1) FInfo(16) flNum(4) = 24, then
    // StBlk(2) LgLen(4) PyLen(4) RStBlk(2) RLgLen(4) RPyLen(4) dates(12)
    // FXInfo(16) ClpSize(2) ExtRec(12) RExtRec(12).
    for (const FilePatch& p : patches) {
        const Entry& e = *p.e;
        Record& r = recs[p.recIndex];
        u8* d = r.bytes.data() + 1 + r.bytes[0];   // past the packed key
        put16(d + 24, e.dataStart);
        put32(d + 30, u32(e.dataABs) * alBlkSiz);
        put16(d + 34, e.rsrcStart);
        put32(d + 40, u32(e.rsrcABs) * alBlkSiz);
        u8* ext = d + 74;
        put16(ext + 0, e.dataStart);
        put16(ext + 2, e.dataABs);
        u8* rxt = d + 86;
        put16(rxt + 0, e.rsrcStart);
        put16(rxt + 2, e.rsrcABs);
    }

    // ---- serialize ------------------------------------------------------
    std::vector<u8> img(sizeBytes, 0);
    u32 nmFls = 0, nmRtDirs = 0, filCnt = 0, dirCnt = 0;
    for (const Entry& e : im.entries) {
        if (e.id == kCnidRootDir) continue;
        if (e.isDir) { ++dirCnt; if (e.parent == kCnidRootDir) ++nmRtDirs; }
        else { ++filCnt; if (e.parent == kCnidRootDir) ++nmFls; }
    }

    // MDB (block 2) + alternate (vlen-2).
    {
        u8* b = img.data() + 2 * kBlockSize;
        put16(b + 0x00, kSigWord);
        put32(b + 0x02, kFixedDate);
        put32(b + 0x06, kFixedDate);
        put16(b + 0x0A, kAtrbUnmounted);
        put16(b + 0x0C, u16(nmFls));
        put16(b + 0x0E, u16(vbmSt));
        put16(b + 0x10, u16(usedABs));
        put16(b + 0x12, u16(nmAlBlks));
        put32(b + 0x14, alBlkSiz);
        put32(b + 0x18, alBlkSiz << 2);
        put16(b + 0x1C, u16(alBlSt));
        put32(b + 0x1E, im.nextId);
        put16(b + 0x22, u16(freeBks));
        b[0x24] = u8(im.volName.size());
        std::memcpy(b + 0x25, im.volName.data(), im.volName.size());
        put32(b + 0x4A, u32(extABs) * alBlkSiz);   // drXTClpSiz
        put32(b + 0x4E, u32(catABs) * alBlkSiz);   // drCTClpSiz
        put16(b + 0x52, u16(nmRtDirs));
        put32(b + 0x54, filCnt);
        put32(b + 0x58, dirCnt);
        put32(b + 0x82, u32(extABs) * alBlkSiz);   // drXTFlSize
        put16(b + 0x86, u16(extFirstAB));
        put16(b + 0x88, u16(extABs));
        put32(b + 0x92, u32(catABs) * alBlkSiz);   // drCTFlSize
        put16(b + 0x96, u16(catFirstAB));
        put16(b + 0x98, u16(catABs));
        std::memcpy(img.data() + std::size_t(vlen - 2) * kBlockSize, b, kBlockSize);
    }

    // Volume bitmap.
    {
        u8* vbm = img.data() + std::size_t(vbmSt) * kBlockSize;
        for (u32 ab = 0; ab < usedABs; ++ab)
            vbm[ab >> 3] |= u8(0x80u >> (ab & 7u));
    }

    const u32 extFirstBlk = alBlSt + extFirstAB * lpa;
    const u32 catFirstBlk = alBlSt + catFirstAB * lpa;
    const u32 extNodes = extABs * lpa;

    // Extents tree: empty (header node only).
    {
        auto node = headerNode(0, 0, 0, 0, 0, kExtKeyLen, extNodes, 1);
        std::memcpy(img.data() + std::size_t(extFirstBlk) * kBlockSize,
                    node.data(), kBlockSize);
    }

    // Catalog tree: header, leaves (chained), index levels bottom-up.
    {
        auto header = headerNode(depth, rootNode, u32(recs.size()), 1,
                                 nLeaves ? nLeaves : 1, kCatKeyLen, catNodes, catUsed);
        std::memcpy(img.data() + std::size_t(catFirstBlk) * kBlockSize,
                    header.data(), kBlockSize);
        for (u32 i = 0; i < nLeaves; ++i) {
            u8* b = img.data() + std::size_t(catFirstBlk + 1 + i) * kBlockSize;
            serializeNode(b, kNdLeaf, 1,
                          i + 1 < nLeaves ? (1 + i + 1) : 0,
                          i > 0 ? (1 + i - 1) : 0, leaves[i]);
        }
        u32 nodeBase = 1 + nLeaves;
        u8 height = 2;
        for (const auto& lvl : levels) {
            for (std::size_t i = 0; i < lvl.nodes.size(); ++i) {
                u8* b = img.data() +
                        std::size_t(catFirstBlk + nodeBase + i) * kBlockSize;
                serializeNode(b, kNdIndex, height, 0, 0, lvl.nodes[i]);
            }
            nodeBase += u32(lvl.nodes.size());
            ++height;
        }
    }

    // Fork contents.
    for (const Entry& e : im.entries) {
        if (e.isDir) continue;
        if (!e.data.empty())
            std::memcpy(img.data() +
                            std::size_t(alBlSt + u32(e.dataStart) * lpa) * kBlockSize,
                        e.data.data(), e.data.size());
        if (!e.rsrc.empty())
            std::memcpy(img.data() +
                            std::size_t(alBlSt + u32(e.rsrcStart) * lpa) * kBlockSize,
                        e.rsrc.data(), e.rsrc.size());
    }

    return img;
}

} // namespace openmac::hfs
