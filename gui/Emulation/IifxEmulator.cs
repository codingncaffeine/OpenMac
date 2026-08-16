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
    private double _speedPercent;
    public double SpeedPercent => Volatile.Read(ref _speedPercent);

    public IifxEmulator()
    {
        _worker = new Thread(RunLoop) { IsBackground = true, Name = "OpenMac-IIfx" };
        _worker.Start();
    }

    public void LoadRom(string path, int ramMB, bool bootRomDisk)
    {
        WriteBackFloppy();
        SyncFolderDisk();   // the drop box's volume dies with the machine
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
            // The seat's volume goes back on before the machine runs a frame,
            // so the ROM's startup scan finds its driver -- this is the
            // restart that makes a mid-session transfer disk appear.
            if (_seatImage is { } seat)
                Native.omac_fx_insert_harddisk2(_h, seat, (nuint)seat.Length,
                                                _seatReadOnly ? 1 : 0);
        }
        RomPath = path;
        FloppyPath = null;
        _lastFloppyPresent = false;
        _floppyReadOnly = false;
        HardDiskAttached = false;
        HardDiskPath = null;
        _seat?.Cancel();   // a republish in flight belonged to the old machine
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
        double accumulator = 0, speedElapsed = 0;
        int speedFrames = 0;

        while (!_stop)
        {
            lock (_sync)
            {
                if (_h == IntPtr.Zero)
                {
                    last = clock.ElapsedTicks;
                    accumulator = 0;
                    speedElapsed = 0;
                    speedFrames = 0;
                    Volatile.Write(ref _speedPercent, 0);
                }
            }
            if (_h == IntPtr.Zero) { Thread.Sleep(10); continue; }

            long now = clock.ElapsedTicks;
            double elapsed = (now - last) / frequency;
            last = now;
            if (elapsed > 0.25) elapsed = 0.25;
            accumulator += elapsed;
            speedElapsed += elapsed;

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
                speedFrames++;
            }
            if (accumulator > FrameSeconds) accumulator = FrameSeconds;
            if (frames > 0)
            {
                DrainLog();
                // A drop box republish is carried out here, one stage per
                // batch: the trap injections it needs belong to the thread
                // that owns the CPU, and only between frames.
                _seat?.Pump();
                PublishFrame();
            }
            // Speed over the last wall second: frames actually run against
            // the frames a real machine would have run. Owed frames beyond the
            // catch-up cap are dropped, so this is the honest number.
            if (speedElapsed >= 1.0)
            {
                Volatile.Write(ref _speedPercent,
                    speedFrames * FrameSeconds / speedElapsed * 100.0);
                speedFrames = 0;
                speedElapsed = 0;
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

    // ---- folder disk / drop box / transfer disk (second SCSI seat, ID 1 / drive 5) ----
    //
    // The same seat the Quadra offers, with one IIfx-specific rule made
    // explicit: the ROM loads the seat's driver during its startup bus scan
    // and never again, so a volume put here while the machine is running
    // needs a restart to mount. The seat is therefore remembered across
    // LoadRom and put back on the fresh machine before it runs a frame --
    // which is exactly what "Restart now so it appears?" relies on.
    private DropBoxSeat? _seat;
    private byte[]? _seatImage;      // a read-only seat volume, kept to survive a restart
    private bool _seatReadOnly;

    private DropBoxSeat Seat => _seat ??= new DropBoxSeat(
        unmount: () =>
        {
            lock (_sync)
                return _h == IntPtr.Zero || Native.omac_fx_unmount_harddisk2(_h) != 0;
        },
        readImage: ReadSeatImage,
        insert: img => PutOnSeat(img, false));

    private void PutOnSeat(byte[] img, bool readOnly)
    {
        // A folder disk is rebuilt from its folder on every boot by the window;
        // only a transfer/second disk has nothing but this image to come back from.
        _seatImage = readOnly ? img : null;
        _seatReadOnly = readOnly;
        lock (_sync)
        {
            if (_h != IntPtr.Zero)
                Native.omac_fx_insert_harddisk2(_h, img, (nuint)img.Length, readOnly ? 1 : 0);
        }
    }

    private void ClearSeat()
    {
        _seatImage = null;
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_fx_detach_harddisk2(_h); }
    }

    public string? FolderDiskPath => _seat?.Folder;

    public bool AttachFolderDisk(string folder, out string error)
    {
        error = "";
        if (_h == IntPtr.Zero) { error = "no machine"; return false; }
        SyncFolderDisk();   // a previously attached folder gets its changes first
        byte[]? img = FolderDisk.Build(folder, out error);
        if (img is null) return false;
        PutOnSeat(img, false);
        Seat.Folder = folder;
        TransferDiskLabel = null;
        Log.Line($"[disk] IIfx folder disk built from {folder} "
                 + $"({img.Length / (1024 * 1024)} MB volume)");
        return true;
    }

    public void DetachFolderDisk()
    {
        // The Mac puts the volume away BEFORE the disk leaves the bus. Pulling
        // the disk from under a mounted volume leaves the System holding a
        // catalog for blocks that are gone -- and, worse, for whatever disk
        // is put on the seat next: the drive number is the same, so its
        // volume then looks "already mounted" and never comes up.
        PutAwaySeatVolume();
        SyncFolderDisk();
        Seat.Cancel();
        Seat.Folder = null;
        ClearSeat();
        TransferDiskLabel = null;
    }

    /// <summary>Flush and unmount the seat's volume through the guest, retrying
    /// across frames while the File Manager is busy. False (and a log line)
    /// only when the Mac keeps a file open on it.</summary>
    private bool PutAwaySeatVolume()
    {
        for (int attempt = 0; attempt < 100; ++attempt)
        {
            bool offLine;
            lock (_sync)
                offLine = _h == IntPtr.Zero || Native.omac_fx_unmount_harddisk2(_h) != 0;
            if (offLine) return true;
            Thread.Sleep(20);   // let the worker run a frame or two
        }
        Log.Line("[disk] the Mac would not let go of the seat's volume (a file on it is open)");
        return false;
    }

    /// <summary>Make the seat free for another transfer disk. A previous one
    /// the Mac has mounted is flushed and unmounted (the guest must let go of
    /// its catalog before the blocks change under it); one the Mac never saw
    /// -- no driver resident, or the restart it needed was declined -- needs
    /// nothing. Refuses only when the Mac still has a file open on it.</summary>
    private bool ReleaseSeatForSwap(out string error)
    {
        error = "";
        if (TransferDiskLabel is not { } prev || !TransferDiskResident) return true;
        if (PutAwaySeatVolume()) return true;
        error = $"the transfer disk “{prev}” is still in use in the Mac (a file on it is open); "
              + "put it away first, then drop the new one";
        return false;
    }

    public bool RepublishFolderDisk(string? addFile, out string error) =>
        Seat.Request(addFile, out error);

    public bool RetargetFolderDisk(string folder, out string error) =>
        Seat.Request(null, folder, out error);

    public bool AttachSecondDisk(string imagePath, out string error)
    {
        error = "";
        if (_h == IntPtr.Zero) { error = "no machine"; return false; }
        byte[] img;
        try { img = File.ReadAllBytes(imagePath); }
        catch (Exception ex) { error = ex.Message; return false; }
        Seat.Cancel();
        Seat.Folder = null;          // the seat now holds a disk, not a folder
        PutOnSeat(img, true);        // read-only
        TransferDiskLabel = Path.GetFileName(imagePath);
        Log.Line($"[disk] IIfx second disk: {TransferDiskLabel} "
                 + $"({img.Length / (1024 * 1024)} MB, read-only)");
        return true;
    }

    public bool RepublishPending => _seat?.Pending == true;

    public string? TransferDiskLabel { get; private set; }

    public bool TransferDiskResident
    {
        get
        {
            lock (_sync)
                return _h != IntPtr.Zero && Native.omac_fx_harddisk2_booted(_h) != 0;
        }
    }

    public bool AttachTransferDisk(string filePath, out string error)
    {
        error = "";
        if (_h == IntPtr.Zero) { error = "no machine"; return false; }
        if (!ReleaseSeatForSwap(out error)) return false;
        byte[]? img = FolderDisk.BuildTransferVolume(filePath, 0, out error);
        if (img is null) return false;
        // Software-locked in both MDB copies (drAtrb bit 15), so the System
        // mounts it read-only and never tries to write. An unlocked volume that
        // silently drops writes leaves the guest's cached view diverging from
        // the disk.
        img[1024 + 10] |= 0x80;
        int altMdb = img.Length - 2 * 512;
        if (altMdb > 0) img[altMdb + 10] |= 0x80;
        Seat.Cancel();
        Seat.Folder = null;
        PutOnSeat(img, true);        // read-only
        TransferDiskLabel = Path.GetFileName(filePath);
        Log.Line($"[disk] IIfx transfer disk built for {TransferDiskLabel} "
                 + $"({img.Length / (1024 * 1024)} MB volume, read-only)");
        return true;
    }

    /// <summary>Write the guest's folder-disk changes back to the host folder.</summary>
    private void SyncFolderDisk() => _seat?.Sync();

    private byte[]? ReadSeatImage()
    {
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return null;
            nuint size = Native.omac_fx_harddisk2_data(_h, null, 0);
            if (size == 0) return null;
            byte[] img = new byte[size];
            if (Native.omac_fx_harddisk2_data(_h, img, size) == 0) return null;
            return img;
        }
    }

    // ---- CD-ROM (AppleCD-class target on the SCSI bus; the machine installs
    // its own .AppleCD driver, so no Apple CD-ROM software is needed) ----
    public bool CdRomAttached { get; private set; }
    public string? CdPath { get; private set; }

    public void SetCdRomAttached(bool attached)
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_fx_attach_cd(_h, attached ? 1 : 0, 3); }
        CdRomAttached = attached;
        if (!attached) EjectCd();
    }

    public bool InsertCd(string path)
    {
        if (_h == IntPtr.Zero) return false;
        // A .cue is a text sheet naming the real data file; load that one.
        string mediaPath = path;
        if (Path.GetExtension(path).Equals(".cue", StringComparison.OrdinalIgnoreCase))
        {
            try
            {
                var m = System.Text.RegularExpressions.Regex.Match(
                    File.ReadAllText(path), "FILE\\s+\"([^\"]+)\"",
                    System.Text.RegularExpressions.RegexOptions.IgnoreCase);
                if (m.Success)
                    mediaPath = Path.Combine(Path.GetDirectoryName(path) ?? "", m.Groups[1].Value);
            }
            catch { return false; }
        }
        byte[] img;
        try { img = File.ReadAllBytes(mediaPath); }
        catch { return false; }
        int ok;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return false;
            if (Native.omac_fx_cd_attached(_h) == 0) Native.omac_fx_attach_cd(_h, 1, 3);
            ok = Native.omac_fx_insert_cd(_h, img, (nuint)img.Length);
        }
        if (ok == 0) return false;
        CdRomAttached = true;
        CdPath = path;
        Log.Line($"[core] IIfx CD: {Path.GetFileName(path)}");
        return true;
    }

    public void EjectCd()
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_fx_eject_cd(_h); }
        CdPath = null;
    }

    public bool CdPresent
    {
        get { lock (_sync) return _h != IntPtr.Zero && Native.omac_fx_cd_present(_h) != 0; }
    }

    public string CdMediumNote() =>
        "The SCSI CD-ROM takes .iso/.cdr/.toast masters (ISO 9660, Apple-partitioned or bare HFS), "
        + "raw-sector .bin/.cue/.mdf (MODE1 or MODE2, 2352/2336/2448 bytes) and Disk Utility .dmg "
        + "(UDIF, zlib). An HFS disc mounts on the desktop once the System is up; an ISO-9660-only "
        + "disc needs Foreign File Access in the guest. The ROM does not boot from a CD.";

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
        SyncFolderDisk();   // the drop box's volume dies with the machine
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
