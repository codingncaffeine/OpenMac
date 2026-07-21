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

    private int _phase;

    public void LoadRom(string path, int ramMB, bool bootRomDisk)
    {
        RomPath = path;
        IsRomLoaded = true;
        _phase = 0;
    }

    public void Reset() => _phase = 0;

    public void RunFrame() => _phase++;

    public void RenderTo(byte[] bgra)
    {
        // A classic-Mac desktop preview: white menu bar over a 50% gray dither.
        for (int y = 0; y < ScreenHeight; y++)
        {
            for (int x = 0; x < ScreenWidth; x++)
            {
                int i = (y * ScreenWidth + x) * 4;
                bool white = y < 20 || ((x ^ y) & 1) == 0;
                byte v = white ? (byte)0xFF : (byte)0x00;
                bgra[i] = v; bgra[i + 1] = v; bgra[i + 2] = v; bgra[i + 3] = 0xFF;
            }
        }
    }

    public void InsertFloppy(string path) => FloppyPath = path;
    public void EjectFloppy() => FloppyPath = null;

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

    public void MouseMove(int dx, int dy, bool button) { }
    public void MouseButton(bool down) { }
    public void KeyEvent(int adbCode, bool down) { }

    public void Dispose() { }
}
