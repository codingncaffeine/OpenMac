using System.IO;
using System.Runtime.InteropServices;
using System.Threading;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// The real backend: drives the C++ 68000 core through openmac_c.dll. Faults are
/// always streamed to the log; the verbose channels (traps/IRQ/ADB) are opt-in.
///
/// Emulation runs on a dedicated thread (<see cref="RunLoop"/>), paced off a
/// monotonic clock at the Mac's 60.147 Hz vertical rate. That thread produces the
/// audio samples and feeds waveOut, so playback is never stalled by the WPF UI
/// thread (framebuffer blits, layout, input, GC) — the cause of the old choppy
/// sound. The UI thread only polls <see cref="TryGetFrame"/> to display the latest
/// frame. All native-core (_h) access is serialized by <c>_sync</c>.
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

    // --- threading ---
    // _sync serializes every native call on _h (worker frame loop + UI-thread
    // input/load/disk/dispose). _frameLock guards the published framebuffer that
    // the UI polls. The two locks never nest, so there is no deadlock: the worker
    // releases _sync before taking _frameLock, and the UI takes _frameLock alone.
    private readonly object _sync = new();
    private readonly object _frameLock = new();
    private readonly byte[] _emuFrame = new byte[512 * 342 * 4];    // worker render target
    private readonly byte[] _sharedFrame = new byte[512 * 342 * 4]; // latest frame for the UI
    private bool _frameDirty;
    private readonly Thread _worker;
    private volatile bool _stop;
    private const double FrameSeconds = 1.0 / 60.147;   // Mac vertical rate (22254.5/370 Hz)

    public NativeEmulator()
    {
        _logCb = OnCoreLog;
        _audio = new WaveAudio(22254);            // ROM-synthesized chime + system sounds
        _worker = new Thread(RunLoop) { IsBackground = true, Name = "OpenMac-Emu" };
        _worker.Start();
    }

    private void OnCoreLog(IntPtr user, IntPtr line)
    {
        string? s = Marshal.PtrToStringAnsi(line);
        if (!string.IsNullOrEmpty(s)) Log.Line("[core] " + s);
    }

    public void LoadRom(string path, int ramMB, bool bootRomDisk)
    {
        WriteBackHardDisk();   // persist the current disk's guest writes before teardown
        byte[] rom = File.ReadAllBytes(path);
        lock (_sync)
        {
            Destroy();
            _h = Native.omac_create(rom, (nuint)rom.Length, (uint)ramMB);
            if (_h == IntPtr.Zero)
                throw new InvalidOperationException("core failed to initialize (bad ROM or size?)");
            // Boot the built-in ROM disk (System 6.0.3 from ROM) instead of an inserted
            // disk -- the emulated equivalent of holding Cmd-Opt-X-O at power-on.
            if (bootRomDisk) Native.omac_set_force_rom_disk(_h, 1);
            // Enable fault logging. The core only BUFFERS lines; we drain them off
            // the CPU exception path via omac_poll_log in the frame loop, so logging
            // can't destabilize emulation the way the in-handler callback did.
            _ = _logCb;
            Native.omac_debug_enable(_h, Native.DbgExcept);
        }
        RomPath = path;
        FloppyPath = null;
        HardDiskAttached = false;
        HardDiskPath = null;
        Log.Line($"[core] created — {ramMB} MB, ROM {Path.GetFileName(path)}");
    }

    public void Reset()
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_reset(_h); }
    }

    // ---- emulation thread ----

    /// <summary>
    /// Dedicated frame loop: catches up owed 60.147 Hz frames off a monotonic
    /// clock (capped so a stall can't spiral), produces audio, and publishes the
    /// latest framebuffer for the UI. Never throttled by window focus.
    /// </summary>
    private void RunLoop()
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        double freq = System.Diagnostics.Stopwatch.Frequency;
        long last = sw.ElapsedTicks;
        double acc = 0, fpsElapsed = 0;
        int fpsFrames = 0;

        while (!_stop)
        {
            IntPtr h;
            lock (_sync) h = _h;
            if (h == IntPtr.Zero)
            {
                Thread.Sleep(10);          // idle cheaply until a ROM is loaded
                last = sw.ElapsedTicks;    // don't bank idle time as owed frames
                acc = 0;
                continue;
            }

            long now = sw.ElapsedTicks;
            double dt = (now - last) / freq;
            last = now;
            if (dt > 0.25) dt = 0.25;      // after a stall, resync rather than fast-forward
            acc += dt;
            fpsElapsed += dt;

            int ran = 0;
            while (acc >= FrameSeconds && ran < 4)   // catch up owed frames, capped
            {
                RunOneFrame();
                acc -= FrameSeconds;
                ran++;
                fpsFrames++;
            }
            if (acc > FrameSeconds) acc = FrameSeconds;   // bound the backlog
            if (ran > 0) PublishFrame();

            // Health line once a second: fps ~60 with underruns=0 is healthy;
            // low fps => pacing, high underruns => buffering.
            if (fpsElapsed >= 1.0)
            {
                string audio = _audio.Ok ? _audio.Stats() : "audio: (no device)";
                Log.Line($"perf: fps={fpsFrames / fpsElapsed:F1}  {audio}");
                fpsFrames = 0;
                fpsElapsed = 0;
            }

            Thread.Sleep(1);   // timeBeginPeriod(1) keeps this near 1 ms
        }
    }

    private void RunOneFrame()
    {
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            Native.omac_run_frame(_h);
            DrainLog();
            DrainAudio();
        }
    }

    private void PublishFrame()
    {
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            Native.omac_render(_h, _emuFrame);   // render into the worker's private buffer
        }
        lock (_frameLock)
        {
            Buffer.BlockCopy(_emuFrame, 0, _sharedFrame, 0, _emuFrame.Length);
            _frameDirty = true;
        }
    }

    public bool TryGetFrame(byte[] bgra)
    {
        lock (_frameLock)
        {
            if (!_frameDirty) return false;
            Buffer.BlockCopy(_sharedFrame, 0, bgra, 0, Math.Min(bgra.Length, _sharedFrame.Length));
            _frameDirty = false;
            return true;
        }
    }

    private readonly byte[] _audioBuf = new byte[8192];

    // Called on the emulation thread under _sync.
    private void DrainAudio()
    {
        if (!_audio.Ok) return;
        int n = (int)Native.omac_drain_audio(_h, _audioBuf, (nuint)_audioBuf.Length);
        if (n > 0) _audio.Feed(_audioBuf, n);
    }

    private readonly byte[] _logPoll = new byte[65536];

    // Called on the emulation thread under _sync.
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

    public void InsertFloppy(string path)
    {
        if (_h == IntPtr.Zero) return;
        byte[] img = File.ReadAllBytes(path);
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            Native.omac_insert_floppy(_h, img, (nuint)img.Length, 0);
        }
        FloppyPath = path;
    }

    public void EjectFloppy()
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_eject_floppy(_h); }
        FloppyPath = null;
    }

    public void AttachHardDisk(string path)
    {
        if (_h == IntPtr.Zero) return;
        WriteBackHardDisk();   // persist a previously-attached disk before switching
        byte[] img = File.ReadAllBytes(path);
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            Native.omac_insert_harddisk(_h, img, (nuint)img.Length, 0);
        }
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
        if (!HardDiskAttached || string.IsNullOrEmpty(HardDiskPath)) return;
        byte[]? buf = null;
        try
        {
            lock (_sync)
            {
                if (_h == IntPtr.Zero) return;
                nuint size = Native.omac_harddisk_data(_h, null, 0);   // query size
                if (size == 0) return;
                buf = new byte[size];
                nuint n = Native.omac_harddisk_data(_h, buf, size);
                if (n == 0) return;
            }
            File.WriteAllBytes(HardDiskPath!, buf!);   // file I/O outside the lock (don't stall the worker)
        }
        catch { /* best-effort persistence */ }
    }

    public void MouseMove(int dx, int dy, bool button)
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_mouse(_h, dx, dy, button ? 1 : 0); }
    }

    public void MouseButton(bool down)
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_mouse(_h, 0, 0, down ? 1 : 0); }
    }

    public void KeyEvent(int adbCode, bool down)
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_key(_h, adbCode, down ? 1 : 0); }
    }

    // Caller must hold _sync, or guarantee no other thread touches _h (Dispose,
    // after the worker has been joined).
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
        _stop = true;
        _worker.Join();        // stop the frame loop before touching _h or _audio
        WriteBackHardDisk();
        Destroy();
        _audio.Dispose();
    }
}
