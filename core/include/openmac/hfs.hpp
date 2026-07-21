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

} // namespace openmac::hfs
