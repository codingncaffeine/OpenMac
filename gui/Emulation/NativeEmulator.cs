using System.IO;
using System.Runtime.InteropServices;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// The real backend: drives the C++ 68000 core through openmac_c.dll. Faults are
/// always streamed to the log; the verbose channels (traps/IRQ/ADB) are opt-in.
/// </summary>
public sealed class NativeEmulator : IEmulator
{
    public int ScreenWidth => 512;
    public int ScreenHeight => 342;
    public string BackendName => "native";
    public bool IsRealCore => true;

    public bool IsRomLoaded => _h != IntPtr.Zero;
    public string? RomPath { get; private set; }
    public string? FloppyPath { get; private set; }
    public bool HardDiskAttached { get; private set; }
    public string? HardDiskPath { get; private set; }

    private IntPtr _h;
    private readonly Native.LogCallback _logCb;   // kept alive against GC
    private readonly WaveAudio _audio;

    public NativeEmulator()
    {
        _logCb = OnCoreLog;
        _audio = new WaveAudio(22254);            // ROM-synthesized chime + system sounds
    }

    private void OnCoreLog(IntPtr user, IntPtr line)
    {
        string? s = Marshal.PtrToStringAnsi(line);
        if (!string.IsNullOrEmpty(s)) Log.Line("[core] " + s);
    }

    public void LoadRom(string path, int ramMB, bool bootRomDisk)
    {
        WriteBackHardDisk();   // persist the current disk's guest writes before teardown
        Destroy();
        byte[] rom = File.ReadAllBytes(path);
        _h = Native.omac_create(rom, (nuint)rom.Length, (uint)ramMB);
        if (_h == IntPtr.Zero)
            throw new InvalidOperationException("core failed to initialize (bad ROM or size?)");
        // Boot the built-in ROM disk (System 6.0.3 from ROM) instead of an inserted
        // disk -- the emulated equivalent of holding Cmd-Opt-X-O at power-on.
        if (bootRomDisk) Native.omac_set_force_rom_disk(_h, 1);
        RomPath = path;
        FloppyPath = null;
        HardDiskAttached = false;
        HardDiskPath = null;

        // Enable fault logging. The core only BUFFERS lines; we drain them off
        // the CPU exception path via omac_poll_log in RunFrame, so logging can't
        // destabilize emulation the way the in-handler callback did.
        _ = _logCb;
        Native.omac_debug_enable(_h, Native.DbgExcept);
        Log.Line($"[core] created — {ramMB} MB, ROM {Path.GetFileName(path)}");
    }

    public void Reset()
    {
        if (_h != IntPtr.Zero) Native.omac_reset(_h);
    }

    public void RunFrame()
    {
        if (_h == IntPtr.Zero) return;
        Native.omac_run_frame(_h);
        DrainLog();
        DrainAudio();
    }

    private readonly byte[] _audioBuf = new byte[8192];

    private void DrainAudio()
    {
        if (!_audio.Ok) return;
        int n = (int)Native.omac_drain_audio(_h, _audioBuf, (nuint)_audioBuf.Length);
        if (n > 0) _audio.Feed(_audioBuf, n);
    }

    private readonly byte[] _logPoll = new byte[65536];

    private void DrainLog()
    {
        Native.omac_poll_log(_h, _logPoll, (nuint)_logPoll.Length);
        if (_logPoll[0] == 0) return;
        int len = Array.IndexOf(_logPoll, (byte)0);
        if (len < 0) len = _logPoll.Length;
        string s = System.Text.Encoding.ASCII.GetString(_logPoll, 0, len);
        foreach (string line in s.Split('\n', StringSplitOptions.RemoveEmptyEntries))
            Log.Line("[core] " + line);
    }

    public void RenderTo(byte[] bgra)
    {
        if (_h != IntPtr.Zero) Native.omac_render(_h, bgra);
        else Array.Clear(bgra);
    }

    public void InsertFloppy(string path)
    {
        if (_h == IntPtr.Zero) return;
        byte[] img = File.ReadAllBytes(path);
        Native.omac_insert_floppy(_h, img, (nuint)img.Length, 0);
        FloppyPath = path;
    }

    public void EjectFloppy()
    {
        if (_h != IntPtr.Zero) Native.omac_eject_floppy(_h);
        FloppyPath = null;
    }

    public void AttachHardDisk(string path)
    {
        if (_h == IntPtr.Zero) return;
        WriteBackHardDisk();   // persist a previously-attached disk before switching
        byte[] img = File.ReadAllBytes(path);
        Native.omac_insert_harddisk(_h, img, (nuint)img.Length, 0);
        HardDiskPath = path;
        HardDiskAttached = true;
    }

    public void DetachHardDisk()
    {
        WriteBackHardDisk();
        HardDiskPath = null;
        HardDiskAttached = false;
    }

    /// <summary>Persist the live hard-disk image (with guest writes) back to its file.</summary>
    private void WriteBackHardDisk()
    {
        if (_h == IntPtr.Zero || !HardDiskAttached || string.IsNullOrEmpty(HardDiskPath)) return;
        try
        {
            nuint size = Native.omac_harddisk_data(_h, null, 0);   // query size
            if (size == 0) return;
            var buf = new byte[size];
            nuint n = Native.omac_harddisk_data(_h, buf, size);
            if (n > 0) File.WriteAllBytes(HardDiskPath!, buf);
        }
        catch { /* best-effort persistence */ }
    }

    public void MouseMove(int dx, int dy, bool button)
    {
        if (_h != IntPtr.Zero) Native.omac_mouse(_h, dx, dy, button ? 1 : 0);
    }

    public void MouseButton(bool down)
    {
        if (_h != IntPtr.Zero) Native.omac_mouse(_h, 0, 0, down ? 1 : 0);
    }

    public void KeyEvent(int adbCode, bool down)
    {
        if (_h != IntPtr.Zero) Native.omac_key(_h, adbCode, down ? 1 : 0);
    }

    public string AudioStats() => _audio.Ok ? _audio.Stats() : "audio: (no device)";

    private void Destroy()
    {
        if (_h != IntPtr.Zero)
        {
            Native.omac_destroy(_h);
            _h = IntPtr.Zero;
        }
    }

    public void Dispose()
    {
        WriteBackHardDisk();
        Destroy();
        _audio.Dispose();
    }
}
