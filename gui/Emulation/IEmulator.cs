namespace OpenMac.Gui.Emulation;

/// <summary>
/// The front-end's view of the emulator core. A stub implementation drives the
/// UI today; a native P/Invoke implementation over the C++ core's C ABI drops in
/// once that bridge is built, with no UI changes.
/// </summary>
public interface IEmulator : IDisposable
{
    int ScreenWidth { get; }
    int ScreenHeight { get; }

    /// <summary>Short backend id shown in the UI, e.g. "stub" or "native".</summary>
    string BackendName { get; }
    bool IsRealCore { get; }

    bool IsRomLoaded { get; }
    string? RomPath { get; }
    string? FloppyPath { get; }
    bool HardDiskAttached { get; }
    string? HardDiskPath { get; }

    void LoadRom(string path, int ramMB, bool bootRomDisk);
    void Reset();

    /// <summary>
    /// Copy the latest framebuffer into a ScreenWidth*ScreenHeight*4 BGRA buffer.
    /// Returns false when no new frame is ready since the last call (nothing to
    /// blit). The backend drives emulation and audio on its own thread; the UI
    /// only polls this to display the most recent frame.
    /// </summary>
    bool TryGetFrame(byte[] bgra);

    /// <summary>
    /// Put a disk image in the drive. Returns false if the core refused the file
    /// because it is not floppy media (an NDIF image, an application, an
    /// archive...); the drive is then left untouched and <see cref="MediumNote"/>
    /// says why, in words meant for the person who chose the file.
    /// </summary>
    bool InsertFloppy(string path);
    void EjectFloppy();

    /// <summary>The core's description of the last medium offered to a drive
    /// (0 = internal, 1 = external): geometry for an accepted disk, the reason
    /// for a refusal.</summary>
    string MediumNote(int drive);

    /// <summary>Path of the disk in the external drive, or null.</summary>
    string? ExternalFloppyPath { get; }
    /// <summary>Whether a drive is connected to the machine's external port.</summary>
    bool ExternalDriveAttached { get; }
    void SetExternalDrive(bool attached);
    bool InsertExternalFloppy(string path);
    void EjectExternalFloppy();

    /// <summary>
    /// Insert floppies with the write-protect tab set, and never copy one back
    /// to the file it came from. Takes effect on the next insertion.
    /// </summary>
    bool WriteProtectFloppies { get; set; }

    /// <summary>
    /// True once since the last call if a drive's contents changed without the
    /// front end asking -- the guest ejected a disk. The caller refreshes its
    /// menus; the backend has already saved whatever was written to it.
    /// </summary>
    bool ConsumeDiskStateChanged();

    void AttachHardDisk(string path);
    void DetachHardDisk();

    // ---- CD-ROM ----
    // The drive is a SCSI device (attached to the bus; the ROM and the Apple CD
    // software find it during the boot-time bus scan). A disc is media: put in
    // any time, noticed by the driver's polling, always read-only — nothing is
    // ever copied back out. The guest ejects discs itself (drag to the Trash),
    // so the front end polls CdPresent like it does the floppies.
    bool CdRomAttached { get; }
    string? CdPath { get; }
    void SetCdRomAttached(bool attached);
    bool InsertCd(string path);
    void EjectCd();
    bool CdPresent { get; }
    string CdMediumNote();

    void MouseMove(int dx, int dy, bool button);
    void MouseButton(bool down);
    void KeyEvent(int adbCode, bool down);
}
