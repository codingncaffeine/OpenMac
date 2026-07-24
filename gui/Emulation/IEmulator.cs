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

    void InsertFloppy(string path);
    void EjectFloppy();

    /// <summary>Path of the disk in the external drive, or null.</summary>
    string? ExternalFloppyPath { get; }
    /// <summary>Whether a drive is connected to the machine's external port.</summary>
    bool ExternalDriveAttached { get; }
    void SetExternalDrive(bool attached);
    void InsertExternalFloppy(string path);
    void EjectExternalFloppy();

    /// <summary>
    /// True once since the last call if a drive's contents changed without the
    /// front end asking -- the guest ejected a disk. The caller refreshes its
    /// menus; the backend has already saved whatever was written to it.
    /// </summary>
    bool ConsumeDiskStateChanged();

    void AttachHardDisk(string path);
    void DetachHardDisk();

    void MouseMove(int dx, int dy, bool button);
    void MouseButton(bool down);
    void KeyEvent(int adbCode, bool down);
}
