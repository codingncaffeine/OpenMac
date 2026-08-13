using System.IO;
using System.Threading;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// Macintosh IIfx backend. The worker owns the native 40 MHz 68030 machine;
/// the UI only exchanges completed frames, audio, media and ADB events with it.
/// Model state is deliberately separate from the Classic and Quadra stores.
/// </summary>
public sealed class IifxEmulator : IEmulator
{
    public int ScreenWidth => _screenW;
    public int ScreenHeight => _screenH;
    public string BackendName => "native-iifx";
    public bool IsRealCore => true;
    public bool IsRomLoaded => _h != IntPtr.Zero;
    public string? RomPath { get; private set; }
    public string? FloppyPath { get; private set; }
    public string? ExternalFloppyPath => null;
    public bool ExternalDriveAttached => false;
    public bool HardDiskAttached { get; private set; }
    public string? HardDiskPath { get; private set; }
    public string? VideoRomPath { get; set; }

    private int _screenW = 640, _screenH = 480;
    private IntPtr _h;
    private WaveAudio _audio = new(22254);
    private int _audioRate = 22254;
    private readonly object _sync = new();
    private readonly object _frameLock = new();
    private byte[] _emuFrame = new byte[640 * 480 * 4];
    private byte[] _sharedFrame = new byte[640 * 480 * 4];
    private readonly byte[] _audioBuf = new byte[8192];
    private readonly byte[] _logPoll = new byte[65536];
    private bool _frameDirty;
    private readonly Thread _worker;
    private volatile bool _stop;
    private const double FrameSeconds = 1.0 / 60.15;

    public IifxEmulator()
    {
        _worker = new Thread(RunLoop) { IsBackground = true, Name = "OpenMac-IIfx" };
        _worker.Start();
    }

    public void LoadRom(string path, int ramMB, bool bootRomDisk)
    {
        WriteBackFloppy();
        WriteBackHardDisk();
        SavePram();
        byte[] rom = File.ReadAllBytes(path);
        byte[]? videoRom = !string.IsNullOrEmpty(VideoRomPath) &&
                           File.Exists(VideoRomPath)
            ? File.ReadAllBytes(VideoRomPath) : null;
        lock (_sync)
        {
            DestroyLocked();
            _h = Native.omac_fx_create_with_video_rom(
                rom, (nuint)rom.Length, videoRom,
                (nuint)(videoRom?.Length ?? 0), (uint)ramMB);
            if (_h == IntPtr.Zero)
                throw new InvalidOperationException(
                    "Macintosh IIfx core failed to initialize (wrong system/video ROM or RAM configuration)." );

            byte[]? pram = PramStore.Load("iifx", out uint pramAge);
            if (pram is not null)
            {
                if (Native.omac_fx_pram_load(_h, pram, (nuint)pram.Length,
                                             pramAge) != 0)
                    Log.Line($"[core] IIfx parameter RAM restored ({pramAge}s since it was saved)");
                else
                    Log.Line($"[core] IIfx parameter RAM file was rejected ({pram.Length} bytes)");
            }

            _screenW = Native.omac_fx_screen_w(_h);
            _screenH = Native.omac_fx_screen_h(_h);
            int bytes = checked(_screenW * _screenH * 4);
            if (_emuFrame.Length != bytes)
            {
                _emuFrame = new byte[bytes];
                lock (_frameLock) _sharedFrame = new byte[bytes];
            }
        }
        RomPath = path;
        FloppyPath = null;
        _lastFloppyPresent = false;
        _floppyReadOnly = false;
        HardDiskAttached = false;
        HardDiskPath = null;
        string video = videoRom is null ? "OpenMac fallback video"
            : $"8•24 GC ROM {Path.GetFileName(VideoRomPath)}";
        Log.Line($"[core] Macintosh IIfx created — {ramMB} MB, ROM {Path.GetFileName(path)}, "
                 + $"slot-$9 {video} ({_screenW}x{_screenH})");
    }

    public void Reset()
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_fx_reset(_h); }
    }

    private void RunLoop()
    {
        var clock = System.Diagnostics.Stopwatch.StartNew();
        double frequency = System.Diagnostics.Stopwatch.Frequency;
        long last = clock.ElapsedTicks;
        double accumulator = 0;

        while (!_stop)
        {
            lock (_sync)
            {
                if (_h == IntPtr.Zero)
                {
                    last = clock.ElapsedTicks;
                    accumulator = 0;
                }
            }
            if (_h == IntPtr.Zero) { Thread.Sleep(10); continue; }

            long now = clock.ElapsedTicks;
            double elapsed = (now - last) / frequency;
            last = now;
            if (elapsed > 0.25) elapsed = 0.25;
            accumulator += elapsed;

            int frames = 0;
            while (accumulator >= FrameSeconds && frames < 4)
            {
                lock (_sync)
                {
                    if (_h == IntPtr.Zero) break;
                    Native.omac_fx_run_frame(_h);
                    int rate = checked((int)Native.omac_fx_audio_rate(_h));
                    if (rate != _audioRate)
                    {
                        _audio.Dispose();
                        _audio = new WaveAudio(rate);
                        _audioRate = rate;
                    }
                    int count = (int)Native.omac_fx_drain_audio(
                        _h, _audioBuf, (nuint)_audioBuf.Length);
                    if (count > 0 && _audio.Ok) _audio.Feed(_audioBuf, count);
                }
                accumulator -= FrameSeconds;
                frames++;
            }
            if (accumulator > FrameSeconds) accumulator = FrameSeconds;
            if (frames > 0)
            {
                DrainLog();
                PublishFrame();
            }
            Thread.Sleep(1);
        }
    }

    private void PublishFrame()
    {
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            Native.omac_fx_render(_h, _emuFrame);
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
            Buffer.BlockCopy(_sharedFrame, 0, bgra, 0,
                             Math.Min(bgra.Length, _sharedFrame.Length));
            _frameDirty = false;
            return true;
        }
    }

    private void DrainLog()
    {
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            _logPoll[0] = 0;
            Native.omac_fx_poll_log(_h, _logPoll, (nuint)_logPoll.Length);
        }
        if (_logPoll[0] == 0) return;
        int length = Array.IndexOf(_logPoll, (byte)0);
        if (length < 0) length = _logPoll.Length;
        string text = System.Text.Encoding.ASCII.GetString(_logPoll, 0, length);
        foreach (string line in text.Split('\n', StringSplitOptions.RemoveEmptyEntries))
            Log.Line("[core] " + line);
    }

    // ---- internal FDHD/SuperDrive -------------------------------------
    public bool InsertFloppy(string path)
    {
        if (_h == IntPtr.Zero) return false;
        // Save the medium currently in the drive before a replacement (or the
        // write-protect toggle's re-seat) can overwrite it.  Use the tab state
        // with which that medium was inserted, not the newly selected state.
        WriteBackFloppy();
        byte[] image;
        try { image = File.ReadAllBytes(path); }
        catch { return false; }
        int accepted;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return false;
            accepted = Native.omac_fx_insert_floppy(
                _h, image, (nuint)image.Length, WriteProtectFloppies ? 1 : 0);
        }
        if (accepted == 0) return false;
        FloppyPath = path;
        _floppyReadOnly = WriteProtectFloppies;
        _lastFloppyPresent = true;
        Log.Line($"[core] IIfx SuperDrive: {Path.GetFileName(path)}");
        return true;
    }

    public void EjectFloppy()
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_fx_eject_floppy(_h); }
    }

    public string MediumNote(int drive) => drive == 0
        ? "The Macintosh IIfx internal SuperDrive accepts raw 400K, 800K and 1.44 MB images, DiskCopy 4.2 images, and MacBinary-wrapped images."
        : "No external floppy drive is attached to this Macintosh IIfx configuration.";
    public void SetExternalDrive(bool attached) { }
    public bool InsertExternalFloppy(string path) => false;
    public void EjectExternalFloppy() { }
    public bool WriteProtectFloppies { get; set; }
    public bool ConsumeDiskStateChanged()
    {
        bool present;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return false;
            present = Native.omac_fx_floppy_present(_h) != 0;
        }
        if (present == _lastFloppyPresent) return false;
        _lastFloppyPresent = present;
        if (!present && FloppyPath is not null)
        {
            WriteBackFloppy();
            Log.Line($"[disk] the IIfx ejected {Path.GetFileName(FloppyPath)}");
            FloppyPath = null;
        }
        return true;
    }

    private bool _lastFloppyPresent;
    private bool _floppyReadOnly;

    private void WriteBackFloppy()
    {
        string? path = FloppyPath;
        if (string.IsNullOrEmpty(path) || _floppyReadOnly) return;
        try
        {
            byte[] image;
            lock (_sync)
            {
                if (_h == IntPtr.Zero) return;
                nuint size = Native.omac_fx_floppy_writeback(_h, null, 0);
                if (size == 0) return;
                image = new byte[size];
                if (Native.omac_fx_floppy_writeback(_h, image, size) == 0) return;
            }
            File.WriteAllBytes(path!, image);
        }
        catch (Exception ex) { Log.Line("IIfx floppy write-back failed: " + ex.Message); }
    }

    public void AttachHardDisk(string path)
    {
        if (_h == IntPtr.Zero) return;
        WriteBackHardDisk();
        byte[] image = File.ReadAllBytes(path);
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            Native.omac_fx_insert_harddisk(_h, image, (nuint)image.Length, 0);
        }
        HardDiskPath = path;
        HardDiskAttached = true;
        Log.Line($"[core] IIfx SCSI disk: {Path.GetFileName(path)} "
                 + $"({image.Length / (1024 * 1024)} MB)");
    }

    public void DetachHardDisk()
    {
        WriteBackHardDisk();
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_fx_detach_harddisk(_h); }
        HardDiskPath = null;
        HardDiskAttached = false;
    }

    private void SettleHardDisk()
    {
        if (!HardDiskAttached) return;
        try
        {
            lock (_sync)
            {
                if (_h == IntPtr.Zero) return;
                if (Native.omac_fx_shutdown_harddisk(_h) != 0)
                    Log.Line("IIfx hard disk: guest cache flushed and volume unmounted cleanly");
            }
        }
        catch (Exception ex) { Log.Line("IIfx hard-disk shutdown failed: " + ex.Message); }
    }

    private void WriteBackHardDisk()
    {
        string? path = HardDiskAttached ? HardDiskPath : null;
        if (string.IsNullOrEmpty(path)) return;
        // File Manager caches belong to the guest. Ask it to flush/unmount
        // before copying the backing image, on every path that can replace or
        // destroy the machine (disk swap, RAM/ROM restart, and application exit).
        SettleHardDisk();
        try
        {
            byte[] image;
            lock (_sync)
            {
                if (_h == IntPtr.Zero) return;
                nuint size = Native.omac_fx_harddisk_data(_h, null, 0);
                if (size == 0) return;
                image = new byte[size];
                if (Native.omac_fx_harddisk_data(_h, image, size) == 0) return;
            }
            File.WriteAllBytes(path!, image);
        }
        catch (Exception ex) { Log.Line("IIfx hard-disk write-back failed: " + ex.Message); }
    }

    public bool NetworkingEnabled => false;
    public void SetNetworking(bool enabled) { }
    public string? FolderDiskPath => null;
    public bool AttachFolderDisk(string folder, out string error)
    { error = "The IIfx currently exposes its startup SCSI disk only."; return false; }
    public void DetachFolderDisk() { }
    public bool RepublishFolderDisk(string? addFile, out string error)
    { error = "No IIfx folder disk is attached."; return false; }
    public bool RetargetFolderDisk(string folder, out string error)
    { error = "No IIfx folder disk is attached."; return false; }
    public bool AttachSecondDisk(string imagePath, out string error)
    { error = "The IIfx second SCSI seat is not enabled."; return false; }
    public bool RepublishPending => false;
    public string? TransferDiskLabel => null;
    public bool TransferDiskResident => false;
    public bool AttachTransferDisk(string filePath, out string error)
    { error = "The IIfx second SCSI seat is not enabled."; return false; }
    public bool CdRomAttached => false;
    public string? CdPath => null;
    public void SetCdRomAttached(bool attached) { }
    public bool InsertCd(string path) => false;
    public void EjectCd() { }
    public bool CdPresent => false;
    public string CdMediumNote() => "No SCSI CD-ROM is attached to this IIfx configuration.";

    public string DiagnosticReport()
    {
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return "";
            nuint size = Native.omac_fx_diagnostics(_h, null, 0);
            if (size == 0) return "";
            byte[] buffer = new byte[size];
            nuint count = Native.omac_fx_diagnostics(_h, buffer, size);
            return System.Text.Encoding.ASCII.GetString(buffer, 0, (int)count);
        }
    }

    public void MouseMove(int dx, int dy, bool button)
    { lock (_sync) { if (_h != IntPtr.Zero) Native.omac_fx_mouse(_h, dx, dy, button ? 1 : 0); } }
    public void MouseButton(bool down)
    { lock (_sync) { if (_h != IntPtr.Zero) Native.omac_fx_mouse(_h, 0, 0, down ? 1 : 0); } }
    public void KeyEvent(int adbCode, bool down)
    { lock (_sync) { if (_h != IntPtr.Zero) Native.omac_fx_key(_h, adbCode, down ? 1 : 0); } }

    private void SavePram()
    {
        try
        {
            byte[] blob;
            lock (_sync)
            {
                if (_h == IntPtr.Zero) return;
                nuint size = Native.omac_fx_pram_save(_h, null, 0);
                if (size == 0) return;
                blob = new byte[size];
                if (Native.omac_fx_pram_save(_h, blob, size) == 0) return;
            }
            if (PramStore.Save("iifx", blob))
                Log.Line($"[core] IIfx parameter RAM saved ({blob.Length} bytes)");
        }
        catch (Exception ex) { Log.Line("IIfx PRAM save failed: " + ex.Message); }
    }

    public void Dispose()
    {
        _stop = true;
        _worker.Join();
        WriteBackFloppy();
        WriteBackHardDisk();
        SavePram();
        lock (_sync) DestroyLocked();
        _audio.Dispose();
    }

    private void DestroyLocked()
    {
        if (_h == IntPtr.Zero) return;
        Native.omac_fx_destroy(_h);
        _h = IntPtr.Zero;
    }
}
