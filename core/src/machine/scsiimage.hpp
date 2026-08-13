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
//
// Parameterized so a SECOND disk can carry its own copy: scsiId feeds D5 for
// both the ROM read helper and our WRITE(6) path, driveNum is the _AddDrive
// number, and unit picks the unit-table slot (refNum = -(unit+1)); the first
// disk keeps its historical (0, 4, 1), a second uses the real-world SCSI
// convention of unit 32 + ID.
inline std::vector<u8> buildScsiDriver(u8 scsiId = 0, u16 driveNum = 4, u8 unit = 1) {
    const u16 refNum = static_cast<u16>(~unit);         // -(unit+1)
    const u8 refHi = static_cast<u8>(refNum >> 8), refLo = static_cast<u8>(refNum);
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
        0x45, 0xFA, 0x00, 0x52,             // LEA drvr(PC),A2        A2 = &DRVR (drvr @ +0x54)
        0x26, 0x28, 0x00, 0x08,             // MOVE.L 8(A0),D3        D3 = pmPyPartStart (before A0 clobbered)
        0x20, 0x3C, 0x00, 0x00, 0x00, 0x30, // MOVE.L #$30,D0
        0xA7, 0x1E,                         // _NewPtr,Sys,Clear      A0 = DCE
        0x26, 0x48,                         // MOVEA.L A0,A3          A3 = DCE
        0x27, 0x43, 0x00, 0x14,             // MOVE.L D3,$14(A3)      dCtlStorage = partition phys start
        0x70, 0x04,                         // MOVEQ #4,D0
        0xA7, 0x1E,                         // _NewPtr,Sys,Clear      A0 = master ptr (handle body)
        0x28, 0x48,                         // MOVEA.L A0,A4          A4 = handle
        // The master pointer goes in CLEAN. A unit-table entry is a locked
        // Handle, and in 24-bit mode the Memory Manager keeps that lock in the
        // pointer's own high byte -- so this used to ORI.L #$80000000 to look
        // like the real article. That costs nothing while the machine folds
        // high bytes away, and kills it the moment it stops: with 32-bit
        // addressing switched on the ROM maps $80000000-$FFFFFFFF transparently
        // (dtt1 = $807FC040), so the very first IODone -- which fetches this
        // pointer back out of the unit table and does BTST D4,$0004(A1) on it --
        // drives $8000DDE4 onto the bus, where nothing answers. Bus error, Sad
        // Mac, before a single block of the System is read. A 32-bit master
        // pointer carries no flag bits at all and nothing consults this one in
        // 24-bit mode either, so unflagged is right in both modes.
        0x28, 0x8B,                         // MOVE.L A3,(A4)         *handle = DCE ptr
        0x22, 0x78, 0x01, 0x1C,             // MOVEA.L ($011C).W,A1   A1 = UTableBase
        0x23, 0x4C,                         // MOVE.L A4,d16(A1)      UTableBase[unit] = handle
        static_cast<u8>((unit * 4u) >> 8), static_cast<u8>(unit * 4u),
        0x26, 0x8A,                         // MOVE.L A2,(A3)         DCE.dCtlDriver = &DRVR
        0x37, 0x7C, 0x4F, 0x20, 0x00, 0x04, // MOVE.W #$4F20,4(A3)    dCtlFlags: NeedLock|R|W|Ctl|Stat|dOpened
        0x37, 0x7C, refHi, refLo, 0x00, 0x18, // MOVE.W #refNum,$18(A3)  DCE.dCtlRefNum
        0x70, 0x1E,                         // MOVEQ #30,D0
        0xA7, 0x1E,                         // _NewPtr,Sys,Clear      A0 = DrvSts
        0x11, 0x7C, 0x00, 0x01, 0x00, 0x04, // MOVE.B #1,4(A0)        dsInstalled
        0x11, 0x7C, 0x00, 0x08, 0x00, 0x03, // MOVE.B #8,3(A0)        dsDiskInPlace
        0x22, 0x48,                         // MOVEA.L A0,A1
        0x20, 0x3C,                         // MOVE.L #(drive<<16)|refNum,D0
        static_cast<u8>(driveNum >> 8), static_cast<u8>(driveNum), refHi, refLo,
        0x41, 0xE9, 0x00, 0x06,             // LEA 6(A1),A0          &dsQLink
        0xA0, 0x4E,                         // _AddDrive
        0x4E, 0x75,                         // RTS
        // ---- DRVR (offset 0x54) ----
        0x4F, 0x00,                         // drvrFlags: NeedLock|dReadEnable|dWritEnable|dCtlEnable|dStatEnable
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // delay/emask/menu
        0x00, 0x1A,                         // drvrOpen   = 0x1A
        0x00, 0x1E,                         // drvrPrime  = 0x1E
        0x00, 0xB0,                         // drvrCtl    = 0xB0
        0x00, 0xB8,                         // drvrStatus = 0xB8
        0x00, 0xC0,                         // drvrClose  = 0xC0
        0x07, '.', 'S', 'c', 's', 'i', 'H', 'D',   // drvrName ".ScsiHD", ends even at 0x1A
        // Open (0x1A): dOpened is preset, so this is only a safety no-op
        0x70, 0x00, 0x4E, 0x75,             // MOVEQ #0,D0; RTS
        // Prime (0x1E): read/write blocks over SCSI, at most 64 blocks per bus
        // transaction. A READ(6)/WRITE(6) CDB carries a ONE-BYTE block count: the
        // File Manager routinely asks for hundreds of KB in one request (the
        // Resource Manager writing a big System file), and a count that truncates
        // to the low byte moves the wrong number of blocks -- the transfer ends in
        // the wrong bus phase, the request completes with scPhaseErr, and the data
        // silently never lands (that was the installed-System corruption). All loop
        // state lives in the param block itself (ioActCount accumulates per chunk),
        // because the ROM's read helper clobbers registers freely.
        // A0 = I/O param block, A1 = DCE.
        0x2F, 0x09,                         // MOVE.L A1,-(A7)
        0x2F, 0x03,                         // MOVE.L D3,-(A7)
        0x2F, 0x04,                         // MOVE.L D4,-(A7)
        0x2F, 0x05,                         // MOVE.L D5,-(A7)
        0x2F, 0x06,                         // MOVE.L D6,-(A7)
        0x2F, 0x07,                         // MOVE.L D7,-(A7)
        0x2F, 0x0A,                         // MOVE.L A2,-(A7)
        0x42, 0xA8, 0x00, 0x28,             // CLR.L $28(A0)          ioActCount = 0
        // .loop (p+18):
        0x2E, 0x28, 0x00, 0x24,             // MOVE.L $24(A0),D7      ioReqCount
        0x9E, 0xA8, 0x00, 0x28,             // SUB.L $28(A0),D7       - done so far
        0xE0, 0x8F,                         // LSR.L #8,D7
        0xE2, 0x8F,                         // LSR.L #1,D7            D7 = blocks remaining
        0x67, 0x00, 0x00, 0x5C,             // BEQ.W .done            nothing left -> noErr
        0x74, 0x40,                         // MOVEQ #64,D2
        0xBE, 0x82,                         // CMP.L D2,D7
        0x6C, 0x02,                         // BGE.S .have            chunk = min(remaining, 64)
        0x24, 0x07,                         // MOVE.L D7,D2
        // .have (p+42):
        0x26, 0x28, 0x00, 0x2E,             // MOVE.L $2E(A0),D3      ioPosOffset (bytes)
        0xD6, 0xA8, 0x00, 0x28,             // ADD.L $28(A0),D3       + progress
        0xE0, 0x8B,                         // LSR.L #8,D3
        0xE2, 0x8B,                         // LSR.L #1,D3            D3 = partition block
        0xD6, 0xA9, 0x00, 0x14,             // ADD.L $14(A1),D3       + dCtlStorage -> absolute LBA
        0x24, 0x68, 0x00, 0x20,             // MOVEA.L $20(A0),A2     ioBuffer
        0xD5, 0xE8, 0x00, 0x28,             // ADDA.L $28(A0),A2      + progress
        0x28, 0x3C, 0x00, 0x00, 0x02, 0x00, // MOVE.L #512,D4         block size
        0x7A, scsiId,                       // MOVEQ #scsiId,D5       SCSI target
        0x2F, 0x08,                         // MOVE.L A0,-(A7)        PB/DCE/chunk survive the call
        0x2F, 0x09,                         // MOVE.L A1,-(A7)
        0x2F, 0x02,                         // MOVE.L D2,-(A7)
        0x30, 0x28, 0x00, 0x06,             // MOVE.W $06(A0),D0      ioTrap: _Read=$A002, _Write=$A003
        0x08, 0x00, 0x00, 0x00,             // BTST #0,D0             odd trap => write. MOVE sets CCR,
        0x67, 0x06,                         // BEQ.S .read            so the test sits RIGHT before the branch
        0x61, 0x00, 0x00, 0x4A,             // BSR.W wr6 (@ 0xC4)     write: real SCSI WRITE(6)
        0x60, 0x06,                         // BRA.S .chk
        0x4E, 0xB9, 0x00, 0x40, 0x41, 0xD4, // .read: JSR $004041D4   SCSI READ(6); D0 = result
        // .chk (p+102):
        0x24, 0x1F,                         // MOVE.L (A7)+,D2
        0x22, 0x5F,                         // MOVEA.L (A7)+,A1
        0x20, 0x5F,                         // MOVEA.L (A7)+,A0
        0x4A, 0x40,                         // TST.W D0
        0x66, 0x0E,                         // BNE.S .out             error completes with D0
        0xE1, 0x8A,                         // LSL.L #8,D2
        0xE3, 0x8A,                         // LSL.L #1,D2            D2 = chunk bytes
        0xD5, 0xA8, 0x00, 0x28,             // ADD.L D2,$28(A0)       ioActCount += chunk
        0x60, 0x00, 0xFF, 0x98,             // BRA.W .loop
        // .done (p+124):
        0x70, 0x00,                         // MOVEQ #0,D0
        // .out (p+126):
        0x24, 0x5F,                         // MOVEA.L (A7)+,A2
        0x2E, 0x1F,                         // MOVE.L (A7)+,D7
        0x2C, 0x1F,                         // MOVE.L (A7)+,D6
        0x2A, 0x1F,                         // MOVE.L (A7)+,D5
        0x28, 0x1F,                         // MOVE.L (A7)+,D4
        0x26, 0x1F,                         // MOVE.L (A7)+,D3
        0x22, 0x5F,                         // MOVEA.L (A7)+,A1       DCE back for jIODone
        0x20, 0x78, 0x08, 0xFC,             // MOVEA.L (jIODone).W,A0  A1=DCE, D0=result
        0x4E, 0xD0,                         // JMP (A0)  -- IODone dequeues the request + sets ioResult
        // Control (0xB0): accept + complete with noErr via IODone
        0x70, 0x00, 0x20, 0x78, 0x08, 0xFC, 0x4E, 0xD0,
        // Status (0xB8): accept + complete with noErr via IODone
        0x70, 0x00, 0x20, 0x78, 0x08, 0xFC, 0x4E, 0xD0,
        // Close (0xC0): immediate no-op
        0x70, 0x00, 0x4E, 0x75,
        // wr6 (0xC4): SCSI WRITE(6) subroutine -- same shape as the ROM's read at $4041D4
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

// The same driver with BOTH transfer directions going through _SCSIDispatch
// (SCSIGet/Select/Cmd/Read-or-Write/Complete) instead of the Classic ROM's
// internal read helper. The SCSI Manager API is hardware-independent, so this
// driver works on any machine whose ROM provides it -- it is what the Quadra
// build uses, where there is no 5380 and no $4041D4. Same installer, same
// DRVR shape; Prime dispatches on the ioTrap's low bit into a shared
// six-byte-CDB transfer subroutine parameterized by opcode (D1) and
// _SCSIDispatch selector (D0).
inline std::vector<u8> buildScsiDriverPortable(u8 scsiId = 0, u16 driveNum = 4,
                                               u8 unit = 1) {
    const u16 refNum = static_cast<u16>(~unit);
    const u8 refHi = static_cast<u8>(refNum >> 8), refLo = static_cast<u8>(refNum);
    return {
        // ---- installer (offset 0): identical to buildScsiDriver ----
        0x45, 0xFA, 0x00, 0x52,             // LEA drvr(PC),A2        (drvr @ +0x54)
        0x26, 0x28, 0x00, 0x08,             // MOVE.L 8(A0),D3
        0x20, 0x3C, 0x00, 0x00, 0x00, 0x30, // MOVE.L #$30,D0
        0xA7, 0x1E,                         // _NewPtr,Sys,Clear
        0x26, 0x48,                         // MOVEA.L A0,A3
        0x27, 0x43, 0x00, 0x14,             // MOVE.L D3,$14(A3)
        0x70, 0x04,                         // MOVEQ #4,D0
        0xA7, 0x1E,                         // _NewPtr,Sys,Clear
        0x28, 0x48,                         // MOVEA.L A0,A4
        0x28, 0x8B,                         // MOVE.L A3,(A4)  unflagged -- see buildScsiDriver
        0x22, 0x78, 0x01, 0x1C,             // MOVEA.L ($011C).W,A1
        0x23, 0x4C,                         // MOVE.L A4,d16(A1)
        static_cast<u8>((unit * 4u) >> 8), static_cast<u8>(unit * 4u),
        0x26, 0x8A,                         // MOVE.L A2,(A3)
        0x37, 0x7C, 0x4F, 0x20, 0x00, 0x04, // MOVE.W #$4F20,4(A3)
        0x37, 0x7C, refHi, refLo, 0x00, 0x18, // MOVE.W #refNum,$18(A3)
        0x70, 0x1E,                         // MOVEQ #30,D0
        0xA7, 0x1E,                         // _NewPtr,Sys,Clear
        0x11, 0x7C, 0x00, 0x01, 0x00, 0x04, // MOVE.B #1,4(A0)
        0x11, 0x7C, 0x00, 0x08, 0x00, 0x03, // MOVE.B #8,3(A0)
        0x22, 0x48,                         // MOVEA.L A0,A1
        0x20, 0x3C,                         // MOVE.L #(drive<<16)|refNum,D0
        static_cast<u8>(driveNum >> 8), static_cast<u8>(driveNum), refHi, refLo,
        0x41, 0xE9, 0x00, 0x06,             // LEA 6(A1),A0
        0xA0, 0x4E,                         // _AddDrive
        0x4E, 0x75,                         // RTS
        // ---- DRVR (offset 0x54) ----
        0x4F, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x1A,                         // drvrOpen   = 0x1A
        0x00, 0x1E,                         // drvrPrime  = 0x1E
        0x00, 0xB2,                         // drvrCtl    = 0xB2
        0x00, 0xBA,                         // drvrStatus = 0xBA
        0x00, 0xC2,                         // drvrClose  = 0xC2
        0x07, '.', 'S', 'c', 's', 'i', 'H', 'D',
        // Open (0x1A)
        0x70, 0x00, 0x4E, 0x75,
        // Prime (0x1E): chunked loop as in buildScsiDriver; direction picks
        // CDB opcode + dispatch selector for the shared xfer6.
        0x2F, 0x09,                         // p+0   MOVE.L A1,-(A7)
        0x2F, 0x03,                         // p+2
        0x2F, 0x04,                         // p+4
        0x2F, 0x05,                         // p+6
        0x2F, 0x06,                         // p+8
        0x2F, 0x07,                         // p+10
        0x2F, 0x0A,                         // p+12
        0x42, 0xA8, 0x00, 0x28,             // p+14  CLR.L $28(A0)
        // .loop (p+18)
        0x2E, 0x28, 0x00, 0x24,             // p+18
        0x9E, 0xA8, 0x00, 0x28,             // p+22
        0xE0, 0x8F,                         // p+26
        0xE2, 0x8F,                         // p+28
        0x67, 0x00, 0x00, 0x5E,             // p+30  BEQ.W .done (p+126)
        0x74, 0x40,                         // p+34
        0xBE, 0x82,                         // p+36
        0x6C, 0x02,                         // p+38
        0x24, 0x07,                         // p+40
        // .have (p+42)
        0x26, 0x28, 0x00, 0x2E,             // p+42
        0xD6, 0xA8, 0x00, 0x28,             // p+46
        0xE0, 0x8B,                         // p+50
        0xE2, 0x8B,                         // p+52
        0xD6, 0xA9, 0x00, 0x14,             // p+54
        0x24, 0x68, 0x00, 0x20,             // p+58
        0xD5, 0xE8, 0x00, 0x28,             // p+62
        0x28, 0x3C, 0x00, 0x00, 0x02, 0x00, // p+66  MOVE.L #512,D4
        0x7A, scsiId,                       // p+72  MOVEQ #scsiId,D5
        0x2F, 0x08,                         // p+74
        0x2F, 0x09,                         // p+76
        0x2F, 0x02,                         // p+78
        0x30, 0x28, 0x00, 0x06,             // p+80  MOVE.W $06(A0),D0
        0x08, 0x00, 0x00, 0x00,             // p+84  BTST #0,D0
        0x67, 0x06,                         // p+88  BEQ.S .read (p+96)
        0x72, 0x0A,                         // p+90  MOVEQ #$0A,D1  (WRITE(6))
        0x70, 0x06,                         // p+92  MOVEQ #6,D0    (SCSIWrite)
        0x60, 0x04,                         // p+94  BRA.S .go (p+100)
        0x72, 0x08,                         // p+96  .read: MOVEQ #$08,D1
        0x70, 0x05,                         // p+98  MOVEQ #5,D0    (SCSIRead)
        0x61, 0x00, 0x00, 0x42,             // p+100 .go: BSR.W xfer6 (0xC6)
        // .chk (p+104)
        0x24, 0x1F,                         // p+104
        0x22, 0x5F,                         // p+106
        0x20, 0x5F,                         // p+108
        0x4A, 0x40,                         // p+110
        0x66, 0x0E,                         // p+112 BNE.S .out (p+128)
        0xE1, 0x8A,                         // p+114
        0xE3, 0x8A,                         // p+116
        0xD5, 0xA8, 0x00, 0x28,             // p+118
        0x60, 0x00, 0xFF, 0x96,             // p+122 BRA.W .loop
        // .done (p+126)
        0x70, 0x00,
        // .out (p+128)
        0x24, 0x5F,                         // p+128
        0x2E, 0x1F,                         // p+130
        0x2C, 0x1F,                         // p+132
        0x2A, 0x1F,                         // p+134
        0x28, 0x1F,                         // p+136
        0x26, 0x1F,                         // p+138
        0x22, 0x5F,                         // p+140
        0x20, 0x78, 0x08, 0xFC,             // p+142
        0x4E, 0xD0,                         // p+146
        // Control (0xB2)
        0x70, 0x00, 0x20, 0x78, 0x08, 0xFC, 0x4E, 0xD0,
        // Status (0xBA)
        0x70, 0x00, 0x20, 0x78, 0x08, 0xFC, 0x4E, 0xD0,
        // Close (0xC2)
        0x70, 0x00, 0x4E, 0x75,
        // xfer6 (0xC6): six-byte-CDB transfer via _SCSIDispatch. In: D1.B =
        // CDB opcode, D0.W = data-phase selector (5 = SCSIRead, 6 =
        // SCSIWrite), D3 = LBA, D2 = block count, D4 = block size, D5 =
        // target, A2 = buffer. Out: D0 = status (0 = GOOD).
        0x2F, 0x07,                         // q+0   MOVE.L D7,-(A7)
        0x7E, 0x00,                         // q+2   MOVEQ #0,D7
        0x41, 0xF8, 0x09, 0xFA,             // q+4   LEA $09FA,A0
        0x10, 0xC1,                         // q+8   MOVE.B D1,(A0)+  opcode
        0x48, 0x43,                         // q+10  SWAP D3
        0x02, 0x03, 0x00, 0x1F,             // q+12  ANDI.B #$1F,D3
        0x10, 0xC3,                         // q+16
        0x48, 0x43,                         // q+18
        0x30, 0xC3,                         // q+20
        0x10, 0xC2,                         // q+22
        0x42, 0x18,                         // q+24
        0xC8, 0xC2,                         // q+26  MULU D2,D4
        // D0 is volatile across _SCSIDispatch. D3 is no longer needed after
        // the CDB has been assembled and is preserved by the ROM dispatcher,
        // so keep the caller's read/write selector there until the data phase.
        0x36, 0x00,                         // q+28  MOVE.W D0,D3
        0x9E, 0xFC, 0x00, 0x14,             // q+30  SUBA.W #$14,A7
        0x2C, 0x0F,                         // q+34  MOVE.L A7,D6
        0x55, 0x8F,                         // q+36  SUBQ.L #2,A7
        0x3F, 0x3C, 0x00, 0x01,             // q+38  SCSIGet
        0xA8, 0x15,                         // q+42
        0x3E, 0x17,                         // q+44
        0x66, 0x4C,                         // q+46  BNE.S .err (q+124)
        0x3F, 0x05,                         // q+48
        0x3F, 0x3C, 0x00, 0x02,             // q+50  SCSISelect
        0xA8, 0x15,                         // q+54
        0x3E, 0x17,                         // q+56
        0x66, 0x40,                         // q+58  BNE.S .err
        0x48, 0x78, 0x09, 0xFA,             // q+60  PEA $09FA
        0x3F, 0x3C, 0x00, 0x06,             // q+64  CDB length 6
        0x3F, 0x3C, 0x00, 0x03,             // q+68  SCSICmd
        0xA8, 0x15,                         // q+72
        0x3E, 0x17,                         // q+74
        0x66, 0x16,                         // q+76  BNE.S .compl (q+100)
        0x20, 0x46,                         // q+78  MOVE.L D6,A0
        0x30, 0xFC, 0x00, 0x01,             // q+80  MOVE.W #1,(A0)+  scInc
        0x20, 0xCA,                         // q+84
        0x20, 0xC4,                         // q+86
        0x30, 0xBC, 0x00, 0x07,             // q+88  MOVE.W #7,(A0)   scStop
        0x2F, 0x06,                         // q+92  MOVE.L D6,-(A7)  TIB
        0x3F, 0x03,                         // q+94  MOVE.W D3,-(A7)  saved selector
        0xA8, 0x15,                         // q+96
        0x3E, 0x17,                         // q+98
        // .compl (q+100)
        0x48, 0x78, 0x09, 0xFA,             // q+100
        0x48, 0x78, 0x09, 0xFC,             // q+104
        0x2F, 0x3C, 0x00, 0x00, 0x00, 0x00, // q+108
        0x3F, 0x3C, 0x00, 0x04,             // q+114 SCSIComplete
        0xA8, 0x15,                         // q+118
        0x3E, 0x38, 0x09, 0xFA,             // q+120
        // .err (q+124)
        0xDE, 0xFC, 0x00, 0x16,             // q+124 ADDA.W #$16,A7
        0x30, 0x07,                         // q+128 MOVE.W D7,D0
        0x2E, 0x1F,                         // q+130
        0x4E, 0x75,                         // q+132 RTS
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
