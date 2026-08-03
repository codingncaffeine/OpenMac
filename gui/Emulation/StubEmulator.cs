namespace OpenMac.Gui.Emulation;

/// <summary>
/// Placeholder backend: no real 68000 execution. It tracks machine state so the
/// whole UI (menus, dialogs, status bar, disk attach/detach) is fully exercisable
/// and renders a classic-Mac-style preview into the screen so the layout reads
/// correctly. Swap for the native backend once the C ABI lands.
/// </summary>
public sealed class StubEmulator : IEmulator
{
    public int ScreenWidth => 512;
    public int ScreenHeight => 342;
    public string BackendName => "stub";
    public bool IsRealCore => false;

    public bool IsRomLoaded { get; private set; }
    public string? RomPath { get; private set; }
    public string? FloppyPath { get; private set; }
    public bool HardDiskAttached { get; private set; }
    public string? HardDiskPath { get; private set; }

    private readonly byte[] _frame = new byte[512 * 342 * 4];
    private bool _dirty = true;

    public StubEmulator() => RenderPreview();

    public void LoadRom(string path, int ramMB, bool bootRomDisk)
    {
        RomPath = path;
        IsRomLoaded = true;
        _dirty = true;
    }

    public void Reset() => _dirty = true;

    public bool TryGetFrame(byte[] bgra)
    {
        // The preview is static, so only hand it out when it actually changed.
        if (!_dirty) return false;
        Buffer.BlockCopy(_frame, 0, bgra, 0, Math.Min(bgra.Length, _frame.Length));
        _dirty = false;
        return true;
    }

    private void RenderPreview()
    {
        // A classic-Mac desktop preview: white menu bar over a 50% gray dither.
        for (int y = 0; y < ScreenHeight; y++)
        {
            for (int x = 0; x < ScreenWidth; x++)
            {
                int i = (y * ScreenWidth + x) * 4;
                bool white = y < 20 || ((x ^ y) & 1) == 0;
                byte v = white ? (byte)0xFF : (byte)0x00;
                _frame[i] = v; _frame[i + 1] = v; _frame[i + 2] = v; _frame[i + 3] = 0xFF;
            }
        }
    }

    public bool InsertFloppy(string path) { FloppyPath = path; return true; }
    public void EjectFloppy() => FloppyPath = null;
    public string MediumNote(int drive) => "";

    public string? ExternalFloppyPath { get; private set; }
    public bool ExternalDriveAttached { get; private set; }
    public void SetExternalDrive(bool attached)
    {
        ExternalDriveAttached = attached;
        if (!attached) ExternalFloppyPath = null;
    }
    public bool InsertExternalFloppy(string path)
    {
        ExternalDriveAttached = true;
        ExternalFloppyPath = path;
        return true;
    }
    public void EjectExternalFloppy() => ExternalFloppyPath = null;
    public bool WriteProtectFloppies { get; set; }
    public bool ConsumeDiskStateChanged() => false;

    public void AttachHardDisk(string path)
    {
        HardDiskPath = path;
        HardDiskAttached = true;
    }

    public void DetachHardDisk()
    {
        HardDiskPath = null;
        HardDiskAttached = false;
    }

    public bool NetworkingEnabled { get; private set; }
    public void SetNetworking(bool enabled) => NetworkingEnabled = enabled;

    public string? FolderDiskPath { get; private set; }
    public bool AttachFolderDisk(string folder, out string error)
    {
        error = "";
        FolderDiskPath = folder;
        return true;
    }
    public void DetachFolderDisk() => FolderDiskPath = null;

    // No volume exists to republish, so a request is accepted and does nothing:
    // the preview backend's job is to let the UI be driven, not to emulate.
    public bool RepublishFolderDisk(string? addFile, out string error)
    {
        error = "";
        return FolderDiskPath is not null;
    }
    public bool RetargetFolderDisk(string folder, out string error)
    {
        error = "";
        FolderDiskPath = folder;
        return true;
    }
    public bool RepublishPending => false;

    public string? TransferDiskLabel { get; private set; }
    public bool TransferDiskResident => true;
    public bool AttachTransferDisk(string filePath, out string error)
    {
        error = "";
        TransferDiskLabel = System.IO.Path.GetFileName(filePath);
        return true;
    }

    public bool CdRomAttached { get; private set; }
    public string? CdPath { get; private set; }
    public void SetCdRomAttached(bool attached)
    {
        CdRomAttached = attached;
        if (!attached) CdPath = null;
    }
    public bool InsertCd(string path)
    {
        CdRomAttached = true;
        CdPath = path;
        return true;
    }
    public void EjectCd() => CdPath = null;
    public bool CdPresent => CdPath is not null;
    public string CdMediumNote() => "";
    public string DiagnosticReport() => "";

    public void MouseMove(int dx, int dy, bool button) { }
    public void MouseButton(bool down) { }
    public void KeyEvent(int adbCode, bool down) { }

    public void Dispose() { }
}
