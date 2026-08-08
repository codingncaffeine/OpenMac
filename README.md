# OpenMac

A from-scratch Macintosh emulator for Windows: two machines over one core, with
every chip implemented from documentation.

- **Macintosh Classic** — 68000 (validated against the SingleStepTests suite at
  100% state accuracy), VIA 6522, ADB, IWM floppy, NCR 5380 SCSI, real-time
  clock, and the scanline sound buffer. Boots System 6.0.8 and System 7.0.1 to a
  fully working desktop, and the ROM's built-in System 6.0.3 ROM disk.
- **Quadra 650** — 68040 with its FPU and MMU, DAFB video, 53C96 SCSI, SWIM2
  floppy, Z8530 SCC, the Apple Sound Chip, and VIA1/VIA2. Boots System 7.5 and
  7.5.3 from a hard disk to the Finder desktop, with 8 to 136 MB of RAM and six
  display sizes from 512×384 to 1152×870.

**The [wiki](https://github.com/codingncaffeine/OpenMac/wiki) has the full story**
— the architecture, per-chip hardware notes, and how each subsystem was brought up.

## Getting started

1. Build (below) or pick up a [release](https://github.com/codingncaffeine/OpenMac/releases).
2. Machine ▸ Model to choose the Classic or the Quadra 650.
3. File ▸ Open ROM… and choose the matching ROM dump.
4. Insert a floppy image, or create a hard disk and install a System onto it, and boot.

## What it does

- **Floppies** — 400K, 800K and 1.44 MB images, DiskCopy 4.2 and MacBinary
  containers unwrapped on the way in and rebuilt on the way out; what the Mac
  writes goes back to the file it came from
- **Hard disks** — create a blank HFS volume in the app, install a System onto
  it, and boot from it; volumes are settled however the session ends, so a disk
  stays bootable
- **CD-ROM** — attach a disc image and it mounts on the desktop
- **Drop box** — a folder on your PC, kept mounted inside the Mac as a volume;
  files dropped on the window land in it
- **Parameter RAM persists**, so the startup disk, sound volume, 32-bit
  addressing and the date survive quitting
- **Help ▸ Capture Diagnostics** writes a snapshot of what the machine is doing

## ROMs and software

OpenMac contains no Apple code. You supply your own Macintosh ROM dump and Apple
system software, dumped from media and hardware you own. Nothing of the sort is
included in — or may be contributed to — this repository.

## Building

The core is dependency-free C++20; the front end is a .NET 11 WPF app.

```
cmake -B build
cmake --build build --config Release
dotnet build gui -c Release
```

MIT licensed. Clean-room: primary sources are the chip datasheets and Apple's
published hardware documentation; other emulators were consulted only as
behavioral references, never copied.
