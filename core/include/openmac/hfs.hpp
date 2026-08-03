#pragma once

#include "openmac/types.hpp"

#include <string>
#include <vector>

namespace openmac::hfs {

// Host-side formatter for a blank Macintosh HFS ("Hierarchical File System")
// volume. This is a pure bytes-in / bytes-out data-structure routine: it emits
// a byte image that classic Mac OS's _MountVol will accept and mount as an
// empty, writable volume. There is no emulator coupling.
//
// sizeBytes must be a multiple of 512 (the HFS logical block size); the caller
// is responsible for that. The returned vector is exactly sizeBytes long.
//
// The image contains, in logical-block order:
//   0..1  boot blocks (zeroed; this is a non-bootable data volume)
//   2     Master Directory Block (drSigWord == 0x4244 'BD')
//   3..   volume bitmap
//   ...   extents-overflow B*-tree file (one clump; header node in use)
//   ...   catalog B*-tree file (one clump; header + one leaf node in use,
//         the leaf holding the root directory record and its thread)
//   n-2   alternate (backup) MDB
//   n-1   reserved
//
// All multibyte integers are stored big-endian, as HFS requires.
//
// volumeName is used both for the MDB volume name (drVN) and the root
// directory's catalog name; it is clamped to 27 characters (the HFS volume
// name limit). An empty name becomes "Untitled".
std::vector<u8> formatVolume(u32 sizeBytes, const std::string& volumeName);

// ---- populated volumes (the folder-disk path) --------------------------
//
// VolumeBuilder emits a volume that already CONTAINS a directory tree: real
// catalog B*-tree (multi-leaf, with index levels as needed), both forks per
// file, Finder info, dates. Every fork is laid out contiguously, so nothing
// ever spills into the extents-overflow tree; the catalog file gets spare
// nodes so the guest can add files.
//
// Names must already be MacRoman bytes, at most 31 characters. To keep our
// B*-tree key order identical to the ROM's own RelString comparison, names
// are canonicalized to a safe ASCII subset (letters, digits, space, and
// . _ - ( ) & ' + , ! # $ % @ ~) — anything else becomes '_'. Within that
// subset, case-insensitive ASCII order and the Mac's collation agree; outside
// it they do not, and a mis-sorted key is a file the guest can never find.
//
// Dates are seconds since 1904-01-01 (the HFS epoch); pass 0 for "a fixed
// valid date".
class VolumeBuilder {
public:
    explicit VolumeBuilder(const std::string& volumeName);
    ~VolumeBuilder();
    VolumeBuilder(const VolumeBuilder&) = delete;
    VolumeBuilder& operator=(const VolumeBuilder&) = delete;

    // Add a directory under `parent` (the root is id 2). Returns the new
    // directory's id, for use as a parent in later calls.
    u32 addDir(u32 parent, const std::string& name, u32 crDate = 0, u32 mdDate = 0);

    // Add a file under `parent`. type/creator are the four-character Finder
    // codes packed big-endian ('TEXT' = 0x54455854); fdFlags is the Finder
    // flags word.
    void addFile(u32 parent, const std::string& name, u32 type, u32 creator,
                 u16 fdFlags, std::vector<u8> dataFork, std::vector<u8> rsrcFork,
                 u32 crDate = 0, u32 mdDate = 0);

    // Lay out and serialize the volume. sizeBytes 0 sizes it automatically
    // (content + headroom); an explicit size must fit the content or the
    // build fails. Returns an empty vector on failure with why() set.
    std::vector<u8> build(u32 sizeBytes = 0);
    const std::string& why() const;

private:
    struct Impl;
    Impl* impl_;
};

// ---- reading a volume back (the sync-back path) ------------------------
//
// Walks a volume image's catalog — including one the guest has grown or
// fragmented (extents-overflow consulted for both B*-tree files and forks) —
// and lists every directory and file with its metadata. Fork contents come
// out via readFork.
struct Item {
    u32 id = 0;          // CNID
    u32 parent = 0;      // parent directory CNID (root's parent is 1)
    bool isDir = false;
    std::string name;    // MacRoman bytes as stored
    u32 type = 0, creator = 0;
    u16 fdFlags = 0;
    u32 crDate = 0, mdDate = 0;   // HFS-epoch seconds
    u32 dataLen = 0, rsrcLen = 0;
};

// Lists the whole catalog, root first. Returns false (with items untouched)
// if the image is not a mountable HFS volume.
bool listVolume(const std::vector<u8>& img, std::vector<Item>& items);

// Reads one fork of a file by CNID. Returns false if the file or its blocks
// cannot be resolved.
bool readFork(const std::vector<u8>& img, u32 fileId, bool rsrc, std::vector<u8>& out);

// ---- making an old volume repairable again -----------------------------
//
// When a volume was not unmounted cleanly the Quadra's ROM rebuilds it before
// mounting, and it bounds its walk of the extents-overflow B*-tree by
// ceil(fileBytes / 12) -- computed with a SIXTEEN-BIT divide. Past 12 * 65535
// bytes that quotient does not fit, the divide leaves its destination
// untouched, and the budget collapses to 1: the ROM then refuses any volume
// whose extents tree holds even a single record. Volumes this formatter made
// before 2026-08-03 have a 2048-node (1 MB) extents file and are over that
// line, which is what "the disk went corrupt and won't boot" always was.
//
// shrinkExtentsTree cuts the extents file back to `maxBytes` IN PLACE, and only
// ever gives back nodes the tree is not using: it refuses unless every node the
// header node's allocation map marks in use lies below the new end. Everything
// it touches -- drXTFlSize, drXTClpSiz, drXTExtRec, bthNNodes/bthFree in the
// header node, the volume bitmap, drFreeBks, both MDBs -- moves together.
//
// `base` is where the volume starts inside `img` (0 for a bare volume, the
// partition's offset inside a wrapped SCSI image). Returns true only when the
// image was changed; `why` always says what happened, in words meant for a log
// the user may read.
bool shrinkExtentsTree(std::vector<u8>& img, u32 base, u32 volumeBytes,
                       u32 maxBytes, std::string& why);

// The largest extents-overflow file the ROM's rebuild can still walk, halved so
// a volume may grow once more before it is out of reach. The formatter and the
// repair share it, so there is one number and not two that can drift.
constexpr u32 kRepairableExtentsBytes = 12u * 32768u;

} // namespace openmac::hfs
