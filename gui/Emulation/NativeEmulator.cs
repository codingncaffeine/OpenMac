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
    public string? ExternalFloppyPath { get; private set; }
    public bool ExternalDriveAttached { get; private set; }
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
        WriteBackFloppy();
        WriteBackExternalFloppy();
        WriteBackHardDisk();   // persist guest writes on every medium before teardown
        SyncFolderDisk();
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
        ExternalFloppyPath = null;
        ExternalDriveAttached = false;
        HardDiskAttached = false;
        HardDiskPath = null;
        CdRomAttached = false;
        CdPath = null;
        FolderDiskPath = null;
        TransferDiskLabel = null;
        Log.Line($"[core] created — {ramMB} MB, ROM {Path.GetFileName(path)}");
        // The adapter is per-machine state; re-attach it on the fresh machine.
        if (NetworkingEnabled)
            lock (_sync) { if (_h != IntPtr.Zero) Native.omac_net_attach(_h, 1, 4); }
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
        PumpNetwork();
        NoticeGuestEjects();
    }

    private volatile bool _diskStateDirty;

    /// <summary>True once per change; the UI polls this to refresh its menus.</summary>
    public bool ConsumeDiskStateChanged()
    {
        if (!_diskStateDirty) return false;
        _diskStateDirty = false;
        return true;
    }

    // The guest takes disks out by itself: the startup scan drops a floppy with
    // no System on it, an installer swaps between disks, the Finder obeys a drag
    // to the Trash. Nothing tells the front end, so notice it here -- otherwise
    // the menu keeps offering to eject an empty drive and, worse, whatever the
    // guest wrote to that disk is never saved back to the file it came from.
    private void NoticeGuestEjects()
    {
        bool internalGone, externalGone, cdGone;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            internalGone = FloppyPath is not null && Native.omac_floppy_present(_h, 0) == 0;
            externalGone = ExternalFloppyPath is not null && Native.omac_floppy_present(_h, 1) == 0;
            cdGone = CdPath is not null && Native.omac_cd_present(_h) == 0;
        }
        if (internalGone)
        {
            WriteBackFloppy();      // the core keeps the medium after an eject
            Log.Line($"[disk] the machine ejected {Path.GetFileName(FloppyPath)}");
            FloppyPath = null;
        }
        if (externalGone)
        {
            WriteBackExternalFloppy();
            Log.Line("[disk] the machine ejected " +
                     $"{Path.GetFileName(ExternalFloppyPath)} (external drive)");
            ExternalFloppyPath = null;
        }
        if (cdGone)
        {
            // Nothing to write back: CDs are read-only.
            Log.Line($"[disk] the machine ejected the CD {Path.GetFileName(CdPath)}");
            CdPath = null;
        }
        if (internalGone || externalGone || cdGone) _diskStateDirty = true;
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

    /// <summary>
    /// Put disks in with the write-protect tab set, the way a locked disk goes
    /// in on a real Mac. Off by default: the guest's writes are copied back into
    /// the image the disk came from, which is what a real floppy does.
    /// </summary>
    public bool WriteProtectFloppies { get; set; }

    public bool InsertFloppy(string path)
    {
        if (_h == IntPtr.Zero) return false;
        WriteBackFloppy();   // save the outgoing disk before it is replaced
        byte[] img = File.ReadAllBytes(path);
        int ok;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return false;
            ok = Native.omac_insert_floppy(_h, img, (nuint)img.Length,
                                           WriteProtectFloppies ? 1 : 0);
        }
        // A refused file is not "the disk in the drive": the core left the drive
        // exactly as it was, so the previous path (and its write-back) stands.
        if (ok != 0) FloppyPath = path;
        return ok != 0;
    }

    public string MediumNote(int drive)
    {
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return "";
            nuint n = Native.omac_floppy_medium(_h, drive, null, 0);
            if (n == 0) return "";
            byte[] buf = new byte[n + 1];
            Native.omac_floppy_medium(_h, drive, buf, (nuint)buf.Length);
            return System.Text.Encoding.ASCII.GetString(buf, 0, (int)n);
        }
    }

    public void EjectFloppy()
    {
        WriteBackFloppy();
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_eject_floppy(_h); }
        FloppyPath = null;
    }

    // ---- external drive ----
    public void SetExternalDrive(bool attached)
    {
        if (!attached) { WriteBackExternalFloppy(); ExternalFloppyPath = null; }
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            Native.omac_set_external_drive(_h, attached ? 1 : 0);
        }
        ExternalDriveAttached = attached;
    }

    public bool InsertExternalFloppy(string path)
    {
        if (_h == IntPtr.Zero) return false;
        WriteBackExternalFloppy();
        byte[] img = File.ReadAllBytes(path);
        int ok;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return false;
            ok = Native.omac_insert_floppy2(_h, img, (nuint)img.Length,
                                            WriteProtectFloppies ? 1 : 0);
        }
        if (ok != 0)
        {
            ExternalDriveAttached = true;
            ExternalFloppyPath = path;
        }
        return ok != 0;
    }

    public void EjectExternalFloppy()
    {
        WriteBackExternalFloppy();
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_eject_floppy2(_h); }
        ExternalFloppyPath = null;
    }

    // The guest writes to the disk itself now -- the ROM's own driver puts
    // sectors down through the emulated chip -- so whatever it changed has to be
    // copied back to the file the disk came from, or the session's work is lost
    // when the image is replaced or the machine goes away.
    private void WriteBackFloppy() => WriteBack(FloppyPath, Native.omac_floppy_data);
    private void WriteBackExternalFloppy() =>
        WriteBack(ExternalFloppyPath, Native.omac_floppy2_data);

    private delegate nuint MediumReader(IntPtr h, byte[]? outBuf, nuint cap);

    private void WriteBack(string? path, MediumReader read)
    {
        if (string.IsNullOrEmpty(path)) return;
        // A locked disk never goes back to its file. The guest could not have
        // changed it, and overwriting somebody's master image on the strength of
        // a bug in our own write path is not a risk worth carrying for a disk we
        // were told to treat as read-only.
        if (WriteProtectFloppies) return;
        byte[]? buf = null;
        try
        {
            lock (_sync)
            {
                if (_h == IntPtr.Zero) return;
                nuint size = read(_h, null, 0);          // query size
                if (size == 0) return;
                buf = new byte[size];
                if (read(_h, buf, size) == 0) return;
            }
            File.WriteAllBytes(path!, buf!);   // outside the lock: don't stall the worker
        }
        catch { /* best-effort persistence */ }
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

    // ---- networking ----
    public bool NetworkingEnabled { get; private set; }
    private SlirpNat? _nat;
    private readonly System.Collections.Concurrent.ConcurrentQueue<byte[]> _netToGuest = new();
    private readonly byte[] _netFrame = new byte[1600];

    public void SetNetworking(bool enabled)
    {
        lock (_sync)
        {
            if (_h != IntPtr.Zero) Native.omac_net_attach(_h, enabled ? 1 : 0, 4);
        }
        if (enabled && _nat is null)
        {
            _nat = new SlirpNat(f => _netToGuest.Enqueue(f), Log.Line);
            Log.Line("net: NAT up — guest network 10.0.2.0/24 (BOOTP serves 10.0.2.15)");
        }
        else if (!enabled && _nat is not null)
        {
            _nat.Dispose();
            _nat = null;
            while (_netToGuest.TryDequeue(out _)) { }
            Log.Line("net: NAT down");
        }
        NetworkingEnabled = enabled;
    }

    // Called on the emulation thread each frame: move guest frames to the NAT
    // and NAT frames to the guest. Guest frames are processed outside _sync so
    // socket work never stalls the frame loop.
    private void PumpNetwork()
    {
        if (_nat is null) return;
        List<byte[]>? outbound = null;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            for (int i = 0; i < 32; ++i)
            {
                nuint n = Native.omac_net_drain(_h, _netFrame, (nuint)_netFrame.Length);
                if (n == 0) break;
                byte[] f = new byte[n];
                Buffer.BlockCopy(_netFrame, 0, f, 0, (int)n);
                (outbound ??= new List<byte[]>()).Add(f);
            }
            while (_netToGuest.TryDequeue(out byte[]? inFrame))
                Native.omac_net_inject(_h, inFrame, (nuint)inFrame.Length);
        }
        if (outbound is not null)
            foreach (byte[] f in outbound)
                _nat.OnGuestFrame(f);
    }

    // ---- folder disk ----
    public string? FolderDiskPath { get; private set; }

    public bool AttachFolderDisk(string folder, out string error)
    {
        error = "";
        if (_h == IntPtr.Zero) { error = "no machine"; return false; }
        SyncFolderDisk();   // a previously attached folder gets its changes first
        byte[]? img = FolderDisk.Build(folder, out error);
        if (img is null) return false;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return false;
            Native.omac_insert_harddisk2(_h, img, (nuint)img.Length, 0);
        }
        FolderDiskPath = folder;
        Log.Line($"[disk] folder disk built from {folder} ({img.Length / (1024 * 1024)} MB volume)");
        return true;
    }

    public void DetachFolderDisk()
    {
        SyncFolderDisk();
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_detach_harddisk2(_h); }
        FolderDiskPath = null;
    }

    // ---- transfer disk (shares the second-disk seat with the folder disk) ----
    public string? TransferDiskLabel { get; private set; }

    public bool TransferDiskResident
    {
        get
        {
            lock (_sync) { return _h != IntPtr.Zero && Native.omac_harddisk2_booted(_h) != 0; }
        }
    }

    public bool AttachTransferDisk(string filePath, out string error)
    {
        error = "";
        if (_h == IntPtr.Zero) { error = "no machine"; return false; }
        byte[]? img = FolderDisk.BuildTransferVolume(filePath, 0, out error);
        if (img is null) return false;
        // Mark the volume software-locked (drAtrb bit 15 in both MDB copies)
        // so the System MOUNTS it read-only and never writes. Without the bit
        // the Finder writes its Desktop file into a disk that silently drops
        // writes, and the guest's cached view quietly diverges from the disk.
        img[1024 + 10] |= 0x80;
        int altMdb = img.Length - 2 * 512;
        if (altMdb > 0) img[altMdb + 10] |= 0x80;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return false;
            Native.omac_insert_harddisk2(_h, img, (nuint)img.Length, 1);   // read-only
        }
        TransferDiskLabel = Path.GetFileName(filePath);
        Log.Line($"[disk] transfer disk built for {TransferDiskLabel} "
                 + $"({img.Length / (1024 * 1024)} MB volume, read-only)");
        return true;
    }

    /// <summary>Write the guest's folder-disk changes back to the host folder.</summary>
    private void SyncFolderDisk()
    {
        if (string.IsNullOrEmpty(FolderDiskPath)) return;
        byte[]? img = null;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            nuint size = Native.omac_harddisk2_data(_h, null, 0);
            if (size == 0) return;
            img = new byte[size];
            if (Native.omac_harddisk2_data(_h, img, size) == 0) return;
        }
        var r = FolderDisk.SyncBack(img!, FolderDiskPath!);   // file I/O off the lock
        Log.Line(r.Error is null
            ? $"[disk] folder disk synced: {r.Updated} updated, {r.Added} added, {r.Removed} moved to _openmac-removed"
            : $"[disk] folder disk sync FAILED: {r.Error}");
    }

    // ---- CD-ROM ----
    public bool CdRomAttached { get; private set; }
    public string? CdPath { get; private set; }

    public void SetCdRomAttached(bool attached)
    {
        lock (_sync)
        {
            if (_h != IntPtr.Zero) Native.omac_cd_attach(_h, attached ? 1 : 0, 3);
        }
        CdRomAttached = attached;
        if (!attached) CdPath = null;
    }

    public bool InsertCd(string path)
    {
        if (_h == IntPtr.Zero) return false;
        // A .cue is a text sheet naming the real data file; load that one.
        string mediaPath = path;
        if (Path.GetExtension(path).Equals(".cue", StringComparison.OrdinalIgnoreCase))
        {
            var m = System.Text.RegularExpressions.Regex.Match(
                File.ReadAllText(path), "FILE\\s+\"([^\"]+)\"",
                System.Text.RegularExpressions.RegexOptions.IgnoreCase);
            if (m.Success)
                mediaPath = Path.Combine(Path.GetDirectoryName(path) ?? "", m.Groups[1].Value);
        }
        byte[] img = File.ReadAllBytes(mediaPath);
        int ok;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return false;
            if (Native.omac_cd_attached(_h) == 0) Native.omac_cd_attach(_h, 1, 3);
            ok = Native.omac_cd_insert(_h, img, (nuint)img.Length);
        }
        if (ok != 0)
        {
            CdRomAttached = true;
            CdPath = path;
        }
        return ok != 0;
    }

    public void EjectCd()
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_cd_eject(_h); }
        CdPath = null;
    }

    public bool CdPresent
    {
        get
        {
            lock (_sync) { return _h != IntPtr.Zero && Native.omac_cd_present(_h) != 0; }
        }
    }

    // The Classic backend has its own monitor views rather than one snapshot;
    // the capture menu item is a Quadra facility for now.
    public string DiagnosticReport() => "";

    public string CdMediumNote()
    {
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return "";
            nuint n = Native.omac_cd_medium(_h, null, 0);
            if (n == 0) return "";
            byte[] buf = new byte[n + 1];
            Native.omac_cd_medium(_h, buf, (nuint)buf.Length);
            return System.Text.Encoding.ASCII.GetString(buf, 0, (int)n);
        }
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
        WriteBackFloppy();
        WriteBackExternalFloppy();
        WriteBackHardDisk();
        SyncFolderDisk();
        Destroy();
        _nat?.Dispose();
        _audio.Dispose();
    }
}
