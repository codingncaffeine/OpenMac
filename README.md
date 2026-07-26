# OpenMac

A from-scratch Macintosh Classic emulator for Windows. Every chip is implemented
from documentation — the 68000 CPU (validated against the SingleStepTests suite),
VIA 6522, ADB, IWM floppy, NCR 5380 SCSI, real-time clock, and sound — under a
native dark-themed front end.

It boots System 6.0.8 and System 7.0.1 to a fully working desktop, and can boot
the Classic ROM's built-in System 6.0.3 ROM disk.

**The [wiki](https://github.com/codingncaffeine/OpenMac/wiki) has the full story**
— the architecture, per-chip hardware notes, and how each subsystem was brought up.

## Getting started

1. Build (below) or pick up a [release](https://github.com/codingncaffeine/OpenMac/releases).
2. File ▸ Open ROM… and choose your Macintosh Classic ROM dump.
3. Insert a floppy image, or create a hard disk and install a System onto it, and boot.

## ROMs and software

OpenMac contains no Apple code. You supply your own Macintosh Classic ROM dump and
Apple system software, dumped from media and hardware you own. Nothing of the sort
is included in — or may be contributed to — this repository.

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
