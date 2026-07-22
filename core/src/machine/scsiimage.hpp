#pragma once

// Build an Apple-partitioned SCSI disk image around a raw HFS volume, the way a
// real Mac hard disk is laid out so the Classic ROM's boot can read it:
//
//   block 0            Driver Descriptor Map (DDM, 'ER')
//   blocks 1..N        Apple Partition Map (APM, 'PM'), one 512-byte entry each:
//                        [1] Apple_partition_map (the map itself)
//                        [2] Apple_Driver43       (the disk's 68k driver)
//                        [3] Apple_HFS            (the volume)
//   driver partition   the 68k driver bytes
//   HFS partition      the raw HFS volume
//
// The ROM reads block 0 -> the DDM's driver descriptor -> the driver partition,
// validates and runs the driver, which installs itself with _AddDrive; the APM's
// Apple_HFS entry (bootable) is then mounted. All multi-byte fields are big-endian.
//
// Reference: Inside Macintosh: Devices, "SCSI Manager" + "The Driver Descriptor
// Record" and "Partition Maps"; APM layout is public/clean-room.

#include "openmac/types.hpp"

#include <cstring>
#include <string>
#include <vector>

namespace openmac::scsi {

inline void put16(std::vector<u8>& d, std::size_t off, u16 v) {
    d[off] = u8(v >> 8); d[off + 1] = u8(v);
}
inline void put32(std::vector<u8>& d, std::size_t off, u32 v) {
    d[off] = u8(v >> 24); d[off + 1] = u8(v >> 16);
    d[off + 2] = u8(v >> 8); d[off + 3] = u8(v);
}
inline void putStr(std::vector<u8>& d, std::size_t off, std::size_t max, const char* s) {
    std::size_t n = std::strlen(s);
    if (n > max) n = max;
    std::memcpy(d.data() + off, s, n);   // remaining bytes stay zero
}

// Partition status flags (pmPartStatus): valid, allocated, in use, readable,
// writable, and (for the HFS partition) bootable + a mounted-at-startup hint.
constexpr u32 kPartValid = 0x00000001, kPartAlloc = 0x00000002, kPartInUse = 0x00000004;
constexpr u32 kPartReadable = 0x00000010, kPartWritable = 0x00000020;
constexpr u32 kPartBootable = 0x00000008, kPartMountAtStartup = 0x40000000;

// A simple additive checksum of the driver bytes, as pmBootChecksum.
inline u16 driverChecksum(const std::vector<u8>& driver) {
    u16 sum = 0;
    for (u8 b : driver) { sum += b; sum = static_cast<u16>((sum << 1) | (sum >> 15)); }
    return sum;
}

// Wrap `hfs` (a raw HFS volume) and `driver` (68k driver bytes) into a full
// Apple-partitioned disk image. `driverLoadAddr`/`driverEntryOff` describe where
// the ROM should load and enter the driver.
inline std::vector<u8> buildAppleScsiDisk(const std::vector<u8>& hfs,
                                          const std::vector<u8>& driver,
                                          u32 driverLoadAddr = 0x00000000,
                                          u32 driverEntryOff = 0x00000000) {
    constexpr u32 kBlk = 512;
    const u32 mapEntries = 3;                        // map, driver, hfs
    const u32 mapStart = 1;                          // APM begins at block 1
    const u32 driverStart = mapStart + mapEntries;   // driver partition
    const u32 driverBlocks = driver.empty() ? 1u
                           : static_cast<u32>((driver.size() + kBlk - 1) / kBlk);
    const u32 hfsStart = driverStart + driverBlocks;
    const u32 hfsBlocks = static_cast<u32>((hfs.size() + kBlk - 1) / kBlk);
    const u32 totalBlocks = hfsStart + hfsBlocks;

    std::vector<u8> img(static_cast<std::size_t>(totalBlocks) * kBlk, 0);

    // --- block 0: Driver Descriptor Map ---------------------------------
    put16(img, 0, 0x4552);            // sbSig 'ER'
    put16(img, 2, u16(kBlk));         // sbBlkSize
    put32(img, 4, totalBlocks);       // sbBlkCount
    put16(img, 8, 0);                 // sbDevType
    put16(img, 10, 0);                // sbDevId
    put32(img, 12, 0);                // sbData
    put16(img, 16, 1);                // sbDrvrCount
    put32(img, 18, driverStart);      // ddBlock (driver first block)
    put16(img, 22, u16(driverBlocks));// ddSize (blocks)
    put16(img, 24, 0x0001);           // ddType (Macintosh SCSI driver)

    auto writeEntry = [&](u32 block, u32 pyStart, u32 blkCnt, const char* name,
                          const char* type, u32 status, bool bootable) {
        std::size_t o = static_cast<std::size_t>(block) * kBlk;
        put16(img, o + 0, 0x504D);            // pmSig 'PM'
        put16(img, o + 2, 0);                 // pmSigPad
        put32(img, o + 4, mapEntries);        // pmMapBlkCnt
        put32(img, o + 8, pyStart);           // pmPyPartStart
        put32(img, o + 12, blkCnt);           // pmPartBlkCnt
        putStr(img, o + 16, 32, name);        // pmPartName
        putStr(img, o + 48, 32, type);        // pmParType
        put32(img, o + 80, 0);                // pmLgDataStart
        put32(img, o + 84, blkCnt);           // pmDataCnt
        put32(img, o + 88, status);           // pmPartStatus
        if (bootable) {
            put32(img, o + 92, 0);                             // pmLgBootStart
            put32(img, o + 96, u32(driver.size()));            // pmBootSize
            put32(img, o + 100, driverLoadAddr);               // pmBootAddr
            put32(img, o + 104, 0);                            // pmBootAddr2
            put32(img, o + 108, driverEntryOff);               // pmBootEntry
            put32(img, o + 112, 0);                            // pmBootEntry2
            put32(img, o + 116, driverChecksum(driver));       // pmBootCksum
            putStr(img, o + 120, 16, "68000");                 // pmProcessor
        }
    };

    // --- blocks 1..3: the partition map --------------------------------
    writeEntry(mapStart + 0, mapStart, mapEntries, "Apple", "Apple_partition_map",
               kPartValid | kPartAlloc | kPartInUse | kPartReadable | kPartWritable, false);
    writeEntry(mapStart + 1, driverStart, driverBlocks, "Macintosh", "Apple_Driver43",
               kPartValid | kPartAlloc | kPartInUse | kPartReadable | kPartWritable | kPartBootable,
               true);
    writeEntry(mapStart + 2, hfsStart, hfsBlocks, "MacOS", "Apple_HFS",
               kPartValid | kPartAlloc | kPartInUse | kPartReadable | kPartWritable |
               kPartBootable | kPartMountAtStartup, false);

    // --- driver + HFS payloads -----------------------------------------
    if (!driver.empty())
        std::memcpy(img.data() + static_cast<std::size_t>(driverStart) * kBlk,
                    driver.data(), driver.size());
    if (!hfs.empty())
        std::memcpy(img.data() + static_cast<std::size_t>(hfsStart) * kBlk,
                    hfs.data(), hfs.size());

    return img;
}

} // namespace openmac::scsi
