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
    u16 sum = 0;   // matches ROM $40427C: sum += byte; ROL.W #1; and 0 -> 0xFFFF
    for (u8 b : driver) { sum += b; sum = static_cast<u16>((sum << 1) | (sum >> 15)); }
    return sum ? sum : static_cast<u16>(0xFFFF);
}

// The Apple_Driver43 partition holds raw installer code, not a DRVR resource: the ROM
// JSRs to it at offset 0 (ROM $404110) with A0 = the Apple_HFS partition entry, A3 =
// the driver. Confirmed live: a probe that dropped a marker at $0CFC and RTS'd ran at
// RAM $1FBC and the boot reached the desktop cleanly. This stub just returns; the real
// installer (build the DQE, _AddDrive, wire Prime to SCSI Manager I/O) replaces it.
inline std::vector<u8> buildScsiDriver() {
    // Full clean-room disk driver loaded from the disk's Apple_Driver43 partition. The
    // ROM JSRs the installer at offset 0 (A0 = HFS partition entry). It installs the
    // embedded DRVR at refNum -2 by hand-building the Device Control Entry + unit-table
    // slot the exact way the ROM does (a unit-table entry is a *locked Handle* whose
    // master pointer -> the DCE; dCtlDriver is a plain pointer to the DRVR, dCtlRefNum =
    // -2). Layout verified against the ROM's own .Sony DCE with the debugger's
    // dump-struct. Then it _AddDrives drive 4. The DRVR's Prime drives real SCSI
    // transfers (reusing the ROM's SCSI-Manager read at $4041D4). This cut marks Open
    // ($09E0 @ $0CFA) and Prime ($5000 @ $0CFC) so the mount chain can be traced.
    //
    // Traps preserve A2-A6/D3-D7, so the driver pointer lives in A2, the DCE in A3, the
    // handle body in A4 across the _NewPtr calls. A5 is CurrentA5 -- never touched.
    return {
        // ---- installer (offset 0): manual DCE install + _AddDrive ----
        0x45, 0xFA, 0x00, 0x5A,             // LEA drvr(PC),A2        A2 = &DRVR (drvr @ +0x5C)
        0x26, 0x28, 0x00, 0x08,             // MOVE.L 8(A0),D3        D3 = pmPyPartStart (before A0 clobbered)
        0x20, 0x3C, 0x00, 0x00, 0x00, 0x30, // MOVE.L #$30,D0
        0xA7, 0x1E,                         // _NewPtr,Sys,Clear      A0 = DCE
        0x26, 0x48,                         // MOVEA.L A0,A3          A3 = DCE
        0x27, 0x43, 0x00, 0x14,             // MOVE.L D3,$14(A3)      dCtlStorage = partition phys start
        0x70, 0x04,                         // MOVEQ #4,D0
        0xA7, 0x1E,                         // _NewPtr,Sys,Clear      A0 = master ptr (handle body)
        0x28, 0x48,                         // MOVEA.L A0,A4          A4 = handle
        0x20, 0x0B,                         // MOVE.L A3,D0           D0 = DCE ptr
        0x00, 0x80, 0x80, 0x00, 0x00, 0x00, // ORI.L #$80000000,D0    set locked-master-ptr flag
        0x28, 0x80,                         // MOVE.L D0,(A4)         *handle = flagged DCE ptr
        0x22, 0x78, 0x01, 0x1C,             // MOVEA.L ($011C).W,A1   A1 = UTableBase
        0x23, 0x4C, 0x00, 0x04,             // MOVE.L A4,4(A1)        UTableBase[1] = handle (refNum -2)
        0x26, 0x8A,                         // MOVE.L A2,(A3)         DCE.dCtlDriver = &DRVR
        0x37, 0x7C, 0x4F, 0x20, 0x00, 0x04, // MOVE.W #$4F20,4(A3)    dCtlFlags: NeedLock|R|W|Ctl|Stat|dOpened
        0x37, 0x7C, 0xFF, 0xFE, 0x00, 0x18, // MOVE.W #-2,$18(A3)     DCE.dCtlRefNum = -2
        0x70, 0x1E,                         // MOVEQ #30,D0
        0xA7, 0x1E,                         // _NewPtr,Sys,Clear      A0 = DrvSts
        0x11, 0x7C, 0x00, 0x01, 0x00, 0x04, // MOVE.B #1,4(A0)        dsInstalled
        0x11, 0x7C, 0x00, 0x08, 0x00, 0x03, // MOVE.B #8,3(A0)        dsDiskInPlace
        0x22, 0x48,                         // MOVEA.L A0,A1
        0x20, 0x3C, 0x00, 0x04, 0xFF, 0xFE, // MOVE.L #$0004FFFE,D0   drive 4 | refNum -2
        0x41, 0xE9, 0x00, 0x06,             // LEA 6(A1),A0          &dsQLink
        0xA0, 0x4E,                         // _AddDrive
        0x4E, 0x75,                         // RTS
        // ---- DRVR (offset 0x5C) ----
        0x4F, 0x00,                         // drvrFlags: NeedLock|dReadEnable|dWritEnable|dCtlEnable|dStatEnable
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // delay/emask/menu
        0x00, 0x1A,                         // drvrOpen   = 0x1A
        0x00, 0x1E,                         // drvrPrime  = 0x1E
        0x00, 0x78,                         // drvrCtl    = 0x78
        0x00, 0x80,                         // drvrStatus = 0x80
        0x00, 0x88,                         // drvrClose  = 0x88
        0x07, '.', 'S', 'c', 's', 'i', 'H', 'D',   // drvrName ".ScsiHD", ends even at 0x1A
        // Open (0x1A): dOpened is preset, so this is only a safety no-op
        0x70, 0x00, 0x4E, 0x75,             // MOVEQ #0,D0; RTS
        // Prime (0x1E): read/write blocks over SCSI via the ROM's block routine ($4041D4).
        // A0 = I/O param block, A1 = DCE. Save the callee-preserved regs we touch, plus A1
        // -- $4041D4 clobbers A1, but jIODone needs A1 = DCE at completion.
        0x2F, 0x09,                         // MOVE.L A1,-(A7)   save DCE across the SCSI call
        0x2F, 0x03,                         // MOVE.L D3,-(A7)
        0x2F, 0x04,                         // MOVE.L D4,-(A7)
        0x2F, 0x05,                         // MOVE.L D5,-(A7)
        0x2F, 0x06,                         // MOVE.L D6,-(A7)
        0x2F, 0x0A,                         // MOVE.L A2,-(A7)
        0x26, 0x28, 0x00, 0x2E,             // MOVE.L $2E(A0),D3      ioPosOffset (bytes)
        0xE0, 0x8B,                         // LSR.L #8,D3
        0xE2, 0x8B,                         // LSR.L #1,D3            D3 = block within partition (/512)
        0xD6, 0xA9, 0x00, 0x14,             // ADD.L $14(A1),D3       + dCtlStorage -> absolute LBA
        0x24, 0x28, 0x00, 0x24,             // MOVE.L $24(A0),D2      ioReqCount (bytes)
        0xE0, 0x8A,                         // LSR.L #8,D2
        0xE2, 0x8A,                         // LSR.L #1,D2            D2 = block count
        0x24, 0x68, 0x00, 0x20,             // MOVEA.L $20(A0),A2     ioBuffer -> A2
        0x21, 0x68, 0x00, 0x24, 0x00, 0x28, // MOVE.L $24(A0),$28(A0) ioActCount = ioReqCount
        0x7A, 0x00,                         // MOVEQ #0,D5            SCSI target id 0 (both paths)
        0x28, 0x3C, 0x00, 0x00, 0x02, 0x00, // MOVE.L #512,D4         block size (both paths; MULU'd to bytes)
        0x30, 0x28, 0x00, 0x06,             // MOVE.W $06(A0),D0      ioTrap: _Read=$A002, _Write=$A003
        0x08, 0x00, 0x00, 0x00,             // BTST #0,D0             odd trap number => write
        0x67, 0x06,                         // BEQ.S .read
        0x61, 0x00, 0x00, 0x30,             // BSR.W wr6 (@ +0x8C)    write: real SCSI WRITE(6)
        0x60, 0x06,                         // BRA.S .done
        0x4E, 0xB9, 0x00, 0x40, 0x41, 0xD4, // .read: JSR $004041D4   SCSI READ(6); D0 = result
        0x24, 0x5F,                         // .done: MOVEA.L (A7)+,A2
        0x2C, 0x1F,                         // MOVE.L (A7)+,D6
        0x2A, 0x1F,                         // MOVE.L (A7)+,D5
        0x28, 0x1F,                         // MOVE.L (A7)+,D4
        0x26, 0x1F,                         // MOVE.L (A7)+,D3
        0x22, 0x5F,                         // MOVEA.L (A7)+,A1  restore DCE ($4041D4 clobbered it)
        0x20, 0x78, 0x08, 0xFC,             // MOVEA.L (jIODone).W,A0  A1=DCE, D0=result
        0x4E, 0xD0,                         // JMP (A0)  -- IODone dequeues the request + sets ioResult
        // Control (0x78): accept + complete with noErr via IODone
        0x70, 0x00, 0x20, 0x78, 0x08, 0xFC, 0x4E, 0xD0,
        // Status (0x80): accept + complete with noErr via IODone
        0x70, 0x00, 0x20, 0x78, 0x08, 0xFC, 0x4E, 0xD0,
        // Close (0x88): immediate no-op
        0x70, 0x00, 0x4E, 0x75,
        // wr6 (0x8C): SCSI WRITE(6) subroutine -- same shape as the ROM's read at $4041D4
        // but CDB opcode $0A and SCSIWrite (selector 6). In: D3=block, D2=count, D4=block
        // size, D5=target, A2=buffer. Out: D0 = SCSI status (0 = GOOD). Uses $09FA CDB/
        // status scratch and the reserved-stack discipline of $4041D4; leaves A1 intact.
        0x2F, 0x07,                         // MOVE.L D7,-(A7)
        0x7E, 0x00,                         // MOVEQ #0,D7
        0x41, 0xF8, 0x09, 0xFA,             // LEA $09FA,A0
        0x10, 0xFC, 0x00, 0x0A,             // MOVE.B #$0A,(A0)+      WRITE(6)
        0x48, 0x43,                         // SWAP D3
        0x02, 0x03, 0x00, 0x1F,             // ANDI.B #$1F,D3
        0x10, 0xC3,                         // MOVE.B D3,(A0)+        LBA[20:16]
        0x48, 0x43,                         // SWAP D3
        0x30, 0xC3,                         // MOVE.W D3,(A0)+        LBA[15:0]
        0x10, 0xC2,                         // MOVE.B D2,(A0)+        length
        0x42, 0x18,                         // CLR.B (A0)+            control
        0xC8, 0xC2,                         // MULU D2,D4             D4 = block size * count = bytes
        0x9E, 0xFC, 0x00, 0x14,             // SUBA.W #$14,A7         reserve TIB
        0x2C, 0x0F,                         // MOVE.L A7,D6           D6 = TIB ptr
        0x55, 0x8F,                         // SUBQ.L #2,A7          reserve result word
        0x3F, 0x3C, 0x00, 0x01,             // MOVE.W #1,-(A7)        SCSIGet
        0xA8, 0x15,                         // _SCSIDispatch
        0x3E, 0x17,                         // MOVE.W (A7),D7
        0x66, 0x4E,                         // BNE.S .err
        0x3F, 0x05,                         // MOVE.W D5,-(A7)        target id
        0x3F, 0x3C, 0x00, 0x02,             // MOVE.W #2,-(A7)        SCSISelect
        0xA8, 0x15,                         // _SCSIDispatch
        0x3E, 0x17,                         // MOVE.W (A7),D7
        0x66, 0x42,                         // BNE.S .err
        0x48, 0x78, 0x09, 0xFA,             // PEA $09FA              CDB ptr
        0x3F, 0x3C, 0x00, 0x06,             // MOVE.W #6,-(A7)        CDB length
        0x3F, 0x3C, 0x00, 0x03,             // MOVE.W #3,-(A7)        SCSICmd
        0xA8, 0x15,                         // _SCSIDispatch
        0x3E, 0x17,                         // MOVE.W (A7),D7
        0x66, 0x18,                         // BNE.S .compl
        0x20, 0x46,                         // MOVE.L D6,A0           A0 = TIB
        0x30, 0xFC, 0x00, 0x01,             // MOVE.W #1,(A0)+        scInc
        0x20, 0xCA,                         // MOVE.L A2,(A0)+        buffer
        0x20, 0xC4,                         // MOVE.L D4,(A0)+        byte count
        0x30, 0xBC, 0x00, 0x07,             // MOVE.W #7,(A0)         scStop
        0x2F, 0x06,                         // MOVE.L D6,-(A7)        push TIB pointer (SCSIRead/Write arg)
        0x3F, 0x3C, 0x00, 0x06,             // MOVE.W #6,-(A7)        SCSIWrite (selector 6)
        0xA8, 0x15,                         // _SCSIDispatch
        0x3E, 0x17,                         // MOVE.W (A7),D7
        // .compl:
        0x48, 0x78, 0x09, 0xFA,             // PEA $09FA              status buffer
        0x48, 0x78, 0x09, 0xFC,             // PEA $09FC              message buffer
        0x2F, 0x3C, 0x00, 0x00, 0x00, 0x00, // MOVE.L #0,-(A7)        timeout
        0x3F, 0x3C, 0x00, 0x04,             // MOVE.W #4,-(A7)        SCSIComplete
        0xA8, 0x15,                         // _SCSIDispatch
        0x3E, 0x38, 0x09, 0xFA,             // MOVE.W $09FA,D7        status byte -> result (0 = GOOD)
        // .err:
        0xDE, 0xFC, 0x00, 0x16,             // ADDA.W #$16,A7         release reserved stack
        0x30, 0x07,                         // MOVE.W D7,D0
        0x2E, 0x1F,                         // MOVE.L (A7)+,D7
        0x4E, 0x75,                         // RTS
    };
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
    // The driver partition carries pmBootSize + pmBootChecksum (set by bootable=true),
    // which the ROM's driver install (ROM $40427C) validates before running the driver.
    // But it must NOT carry the kPartBootable status flag, or the ROM tries to *boot*
    // the disk -- jump to boot code -- and crashes. The HFS partition mounts as data.
    writeEntry(mapStart + 1, driverStart, driverBlocks, "Macintosh", "Apple_Driver43",
               kPartValid | kPartAlloc | kPartInUse | kPartReadable | kPartWritable,
               true);
    writeEntry(mapStart + 2, hfsStart, hfsBlocks, "MacOS", "Apple_HFS",
               kPartValid | kPartAlloc | kPartInUse | kPartReadable | kPartWritable |
               kPartMountAtStartup, false);

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
