using System.IO;
using System.Threading;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// The Quadra 650 backend: drives the C++ 68040 machine through the omac_q_*
/// surface of openmac_c.dll. Same threading shape as <see cref="NativeEmulator"/>:
/// a dedicated frame loop paced at the vertical rate produces frames and audio;
/// the UI polls <see cref="TryGetFrame"/>. Media beyond the SCSI hard disk
/// (floppies, CD, folder disks, networking) is not on this machine yet — those
/// members answer honestly and change nothing.
/// </summary>
public sealed class QuadraEmulator : IEmulator
{
    // The screen is whatever the monitor on the video port is, so this follows
    // the machine rather than naming a size. Cached because the window asks
    // for it before a ROM is loaded, when there is no machine to ask.
    public int ScreenWidth => _screenW;
    public int ScreenHeight => _screenH;
    private int _screenW = 640, _screenH = 480;

    /// <summary>Which monitor is plugged in, by name from <see cref="Displays"/>.
    /// The ROM senses the monitor when it starts, so this takes effect at the
    /// next ROM load -- exactly as swapping a real display would.</summary>
    public string? Monitor { get; set; }

    /// <summary>The displays this video port can drive, with their sizes.</summary>
    public static IEnumerable<(string Name, int W, int H)> Displays()
    {
        for (int i = 0; ; i++)
        {
            IntPtr p = Native.omac_q_display_name(i, out int w, out int h);
            if (p == IntPtr.Zero) yield break;
            string? name = System.Runtime.InteropServices.Marshal.PtrToStringAnsi(p);
            if (name is null) yield break;
            yield return (name, w, h);
        }
    }
    public string BackendName => "native-quadra";
    public bool IsRealCore => true;

    public bool IsRomLoaded => _h != IntPtr.Zero;
    public string? RomPath { get; private set; }
    public string? FloppyPath { get; private set; }
    public string? ExternalFloppyPath => null;
    public bool ExternalDriveAttached => false;
    public bool HardDiskAttached { get; private set; }
    public string? HardDiskPath { get; private set; }

    private IntPtr _h;
    private readonly WaveAudio _audio;
    private readonly object _sync = new();
    private readonly object _frameLock = new();
    // Sized to the monitor in use, not to one resolution: a 21-inch screen is
    // more than three times the pixels of the 13-inch these used to assume,
    // and the core renders as many as the display has.
    private byte[] _emuFrame = new byte[640 * 480 * 4];
    private byte[] _sharedFrame = new byte[640 * 480 * 4];
    private readonly byte[] _audioBuf = new byte[8192];
    private bool _frameDirty;
    private long _loudFed;   // non-silent samples handed to the audio device
    private readonly Thread _worker;
    private volatile bool _stop;
    private const double FrameSeconds = 1.0 / 60.147;

    public QuadraEmulator()
    {
        _audio = new WaveAudio(22254);
        _worker = new Thread(RunLoop) { IsBackground = true, Name = "OpenMac-Quadra" };
        _worker.Start();
    }

    public void LoadRom(string path, int ramMB, bool bootRomDisk)
    {
        // Settle before persisting, exactly as closing does. Loading a ROM
        // tears the running machine down -- it is how the monitor is changed --
        // and writing the disk out while its volume is still mounted saves it
        // in the "in use" state, which is a disk that will not boot next time.
        SettleVolumes();
        SyncFolderDisk();   // the drop box's volume dies with the machine
        WriteBackHardDisk();
        byte[] rom = File.ReadAllBytes(path);
        lock (_sync)
        {
            Destroy();
            _h = Native.omac_q_create(rom, (nuint)rom.Length, (uint)ramMB);
            if (_h == IntPtr.Zero)
                throw new InvalidOperationException("Quadra core failed to initialize (bad ROM or size?)");
            // Plug the monitor in before the machine runs a single frame: the
            // ROM senses the video port during startup and never looks again.
            if (!string.IsNullOrEmpty(Monitor) &&
                Native.omac_q_set_display(_h, Monitor!) == 0)
                Log.Line($"[core] unknown monitor '{Monitor}', keeping the default");
            _screenW = Native.omac_q_screen_w(_h);
            _screenH = Native.omac_q_screen_h(_h);
            const int guard = 4;   // the core writes w*h pixels, nothing more
            int bytes = _screenW * _screenH * guard;
            if (_emuFrame.Length < bytes)
            {
                _emuFrame = new byte[bytes];
                lock (_frameLock) _sharedFrame = new byte[bytes];
            }
        }
        RomPath = path;
        HardDiskAttached = false;
        HardDiskPath = null;
        Log.Line($"[core] Quadra 650 created — {ramMB} MB, ROM {Path.GetFileName(path)}, "
                 + $"monitor {Monitor ?? "13-inch RGB"} ({_screenW}x{_screenH})");
    }

    public void Reset()
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_q_reset(_h); }
    }

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
                Thread.Sleep(10);
                last = sw.ElapsedTicks;
                acc = 0;
                continue;
            }

            long now = sw.ElapsedTicks;
            double dt = (now - last) / freq;
            last = now;
            if (dt > 0.25) dt = 0.25;
            acc += dt;
            fpsElapsed += dt;

            int ran = 0;
            while (acc >= FrameSeconds && ran < 4)
            {
                lock (_sync)
                {
                    if (_h == IntPtr.Zero) break;
                    Native.omac_q_run_frame(_h);
                    if (_audio.Ok)
                    {
                        int n = (int)Native.omac_q_drain_audio(_h, _audioBuf, (nuint)_audioBuf.Length);
                        if (n > 0)
                        {
                            for (int k = 0; k < n; k++)
                                if (_audioBuf[k] > 0x84 || _audioBuf[k] < 0x7C) _loudFed++;
                            _audio.Feed(_audioBuf, n);
                        }
                    }
                }
                acc -= FrameSeconds;
                ran++;
                fpsFrames++;
            }
            if (ran > 0) DrainLog();
            // A drop box republish is carried out here, one stage per batch:
            // the trap injections it needs belong to the thread that owns the
            // CPU, and only between frames.
            if (ran > 0) _seat?.Pump();
            if (acc > FrameSeconds) acc = FrameSeconds;
            if (ran > 0) PublishFrame();

            if (fpsElapsed >= 1.0)
            {
                string audio = _audio.Ok ? _audio.Stats() : "audio: (no device)";
                Log.Line($"perf: fps={fpsFrames / fpsElapsed:F1}  {audio} loud={_loudFed}");
                fpsFrames = 0;
                fpsElapsed = 0;
            }

            Thread.Sleep(1);
        }
    }

    private readonly byte[] _logPoll = new byte[65536];

    // The machine's own account of itself: media events, and the guest faults
    // behind a bomb box. Drained on the emulation thread, where the buffer is
    // filled, so nothing is torn between the two.
    private void DrainLog()
    {
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            Native.omac_q_poll_log(_h, _logPoll, (nuint)_logPoll.Length);
        }
        if (_logPoll[0] == 0) return;
        int len = Array.IndexOf(_logPoll, (byte)0);
        if (len < 0) len = _logPoll.Length;
        string s = System.Text.Encoding.ASCII.GetString(_logPoll, 0, len);
        foreach (string line in s.Split('\n', StringSplitOptions.RemoveEmptyEntries))
            Log.Line("[core] " + line);
    }

    private void PublishFrame()
    {
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            Native.omac_q_render(_h, _emuFrame);
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

    // ---- floppy (internal SuperDrive, served through the .Sony Prime hook) ----
    public bool InsertFloppy(string path)
    {
        if (_h == IntPtr.Zero) return false;
        byte[] img;
        try { img = File.ReadAllBytes(path); }
        catch { return false; }
        int ok;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return false;
            ok = Native.omac_q_insert_floppy(_h, img, (nuint)img.Length, WriteProtectFloppies ? 1 : 0);
        }
        if (ok == 0) return false;
        FloppyPath = path;
        Log.Line($"[core] Quadra floppy: {Path.GetFileName(path)}");
        return true;
    }

    public void EjectFloppy()
    {
        // The machine carries the eject out a frame later, on its own thread.
        // ConsumeDiskStateChanged notices the drive emptying and saves the
        // medium then -- but only while it still knows which file to save it
        // to, so the path stays put until it has.
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_q_eject_floppy(_h); }
    }

    public string MediumNote(int drive) =>
        drive == 0
            ? "The internal SuperDrive takes a 400K/800K/1.44 MB dump or a DiskCopy 4.2 / MacBinary image. "
              + "System 7.5 boots the Quadra 650 (7.1 needs a System Enabler this build doesn't carry)."
            : "The Quadra 650 has one internal floppy drive; there is no external drive port on this machine.";

    public void SetExternalDrive(bool attached) { }
    public bool InsertExternalFloppy(string path) => false;
    public void EjectExternalFloppy() { }
    public bool WriteProtectFloppies { get; set; }

    // The drive empties without the front end being told: the Eject menu item
    // is carried out a frame later by the machine, an installer swaps disks by
    // itself, the Finder obeys a drag to the Trash. Notice it here, or the
    // menus keep offering to eject a drive with nothing in it.
    public bool ConsumeDiskStateChanged()
    {
        bool present;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return false;
            present = Native.omac_q_floppy_present(_h) != 0;
        }
        if (present == _lastFloppyPresent) return false;
        _lastFloppyPresent = present;
        if (!present && FloppyPath is not null)
        {
            // Save first, then forget the path. Whatever the guest wrote to
            // this disk only exists in the core until now, and the core drops
            // it when the next disk goes in.
            WriteBackFloppy();
            Log.Line($"[disk] the machine ejected {Path.GetFileName(FloppyPath)}");
            FloppyPath = null;
        }
        return true;
    }

    private bool _lastFloppyPresent;

    // Put the medium back in the file it came from, wearing whatever container
    // that file wore. A locked disk never goes back: the guest could not have
    // changed it, and overwriting somebody's master image on the strength of a
    // bug in our own write path is not a risk worth carrying.
    /// <summary>Flush and unmount the guest's volumes, so the image about to be
    /// written out is one that will boot. Must run before every write-back:
    /// closing the window, and loading a ROM, which is how the machine is
    /// restarted onto a different monitor.</summary>
    private void SettleVolumes()
    {
        try
        {
            // Log either way. Reporting only success made a silent no-op read
            // exactly like a clean shutdown, and that hid the fact that this
            // had never run once across a whole day of sessions.
            lock (_sync)
            {
                if (_h == IntPtr.Zero) return;
                Log.Line(Native.omac_q_shutdown_volumes(_h) != 0
                    ? "hard disk: volumes flushed and marked cleanly unmounted"
                    : "hard disk: nothing to settle (no volume was mounted)");
            }
        }
        catch (Exception ex) { Log.Line("volume shutdown failed: " + ex.Message); }
    }

    private void WriteBackFloppy()
    {
        string? path = FloppyPath;
        if (string.IsNullOrEmpty(path) || WriteProtectFloppies) return;
        byte[]? buf = null;
        try
        {
            lock (_sync)
            {
                if (_h == IntPtr.Zero) return;
                nuint size = Native.omac_q_floppy_writeback(_h, null, 0);
                if (size == 0) return;
                buf = new byte[size];
                if (Native.omac_q_floppy_writeback(_h, buf, size) == 0) return;
            }
            File.WriteAllBytes(path!, buf!);   // outside the lock: don't stall the worker
        }
        catch (Exception ex) { Log.Line("floppy write-back failed: " + ex.Message); }
    }
    public bool NetworkingEnabled => false;
    public void SetNetworking(bool enabled) { }

    // ---- folder disk / drop box (second SCSI seat, ID 1 / drive 5) ----
    //
    // A host folder served as a real HFS volume. Attaching builds the volume
    // from the folder; the guest's changes come back to the folder whenever the
    // machine settles. The drop box is the same volume kept permanently in
    // place: a file dropped on the window lands in the folder, and the volume
    // is REPUBLISHED so the Mac sees it.
    //
    // Republishing is a swap, not an edit. The guest caches a mounted volume's
    // catalog, extents and bitmap, so writing a new file into the image behind
    // its back leaves it reading yesterday's catalog against today's blocks.
    // Instead the volume is flushed and unmounted, rebuilt on the host, and put
    // back -- the same thing that happens when a removable cartridge is
    // swapped, which is a sequence this System handles natively.
    private DropBoxSeat? _seat;

    private DropBoxSeat Seat => _seat ??= new DropBoxSeat(
        unmount: () =>
        {
            lock (_sync)
                return _h == IntPtr.Zero || Native.omac_q_unmount_harddisk2(_h) != 0;
        },
        readImage: ReadSeatImage,
        insert: img =>
        {
            lock (_sync)
            {
                if (_h != IntPtr.Zero)
                    Native.omac_q_insert_harddisk2(_h, img, (nuint)img.Length, 0);
            }
        });

    public string? FolderDiskPath => _seat?.Folder;

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
            Native.omac_q_insert_harddisk2(_h, img, (nuint)img.Length, 0);
        }
        Seat.Folder = folder;
        TransferDiskLabel = null;
        Log.Line($"[disk] folder disk built from {folder} "
                 + $"({img.Length / (1024 * 1024)} MB volume)");
        return true;
    }

    public void DetachFolderDisk()
    {
        SyncFolderDisk();
        Seat.Cancel();
        Seat.Folder = null;
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_q_detach_harddisk2(_h); }
    }

    public bool RepublishFolderDisk(string? addFile, out string error) =>
        Seat.Request(addFile, out error);

    public bool RetargetFolderDisk(string folder, out string error) =>
        Seat.Request(null, folder, out error);

    public bool RepublishPending => _seat?.Pending == true;

    // ---- transfer disk (shares the seat with the folder disk) ----
    public string? TransferDiskLabel { get; private set; }

    public bool TransferDiskResident
    {
        get
        {
            lock (_sync)
                return _h != IntPtr.Zero && Native.omac_q_harddisk2_booted(_h) != 0;
        }
    }

    public bool AttachTransferDisk(string filePath, out string error)
    {
        error = "";
        if (_h == IntPtr.Zero) { error = "no machine"; return false; }
        byte[]? img = FolderDisk.BuildTransferVolume(filePath, 0, out error);
        if (img is null) return false;
        // Software-locked in both MDB copies (drAtrb bit 15), so the System
        // mounts it read-only and never tries to write. An unlocked volume that
        // silently drops writes leaves the guest's cached view diverging from
        // the disk.
        img[1024 + 10] |= 0x80;
        int altMdb = img.Length - 2 * 512;
        if (altMdb > 0) img[altMdb + 10] |= 0x80;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return false;
            Native.omac_q_insert_harddisk2(_h, img, (nuint)img.Length, 1);   // read-only
        }
        TransferDiskLabel = Path.GetFileName(filePath);
        Log.Line($"[disk] transfer disk built for {TransferDiskLabel} "
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
            nuint size = Native.omac_q_harddisk2_data(_h, null, 0);
            if (size == 0) return null;
            byte[] img = new byte[size];
            if (Native.omac_q_harddisk2_data(_h, img, size) == 0) return null;
            return img;
        }
    }

    // ---- CD-ROM (AppleCD-class target on the SCSI bus) ----
    public bool CdRomAttached { get; private set; }
    public string? CdPath { get; private set; }
    public void SetCdRomAttached(bool attached)
    {
        CdRomAttached = attached;
        if (!attached) EjectCd();
    }

    public bool InsertCd(string path)
    {
        if (_h == IntPtr.Zero) return false;
        byte[] img;
        try { img = File.ReadAllBytes(path); }
        catch { return false; }
        int ok;
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return false;
            ok = Native.omac_q_insert_cd(_h, img, (nuint)img.Length);
        }
        if (ok == 0) return false;
        CdRomAttached = true;
        CdPath = path;
        Log.Line($"[core] Quadra CD: {Path.GetFileName(path)}");
        return true;
    }

    public void EjectCd()
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_q_eject_cd(_h); }
        CdPath = null;
    }

    public bool CdPresent
    {
        get { lock (_sync) return _h != IntPtr.Zero && Native.omac_q_cd_present(_h) != 0; }
    }

    public string CdMediumNote() =>
        "The SCSI CD-ROM takes a raw ISO or Apple-partitioned disc image. The 7.1 install CD is "
        + "readable, but the ROM does not auto-boot a CD — boot a floppy, then mount the disc.";

    public string DiagnosticReport()
    {
        if (_h == IntPtr.Zero) return "";
        nuint size = Native.omac_q_diagnostics(_h, null, 0);
        if (size == 0) return "";
        var buf = new byte[size];
        nuint n = Native.omac_q_diagnostics(_h, buf, size);
        return System.Text.Encoding.ASCII.GetString(buf, 0, (int)n);
    }

    // ---- hard disk ----
    public void AttachHardDisk(string path)
    {
        if (_h == IntPtr.Zero) return;
        WriteBackHardDisk();
        byte[] img = File.ReadAllBytes(path);
        lock (_sync)
        {
            if (_h == IntPtr.Zero) return;
            Native.omac_q_insert_harddisk(_h, img, (nuint)img.Length, 0);
        }
        HardDiskPath = path;
        HardDiskAttached = true;
    }

    public void DetachHardDisk()
    {
        WriteBackHardDisk();
        HardDiskAttached = false;
        HardDiskPath = null;
    }

    private void WriteBackHardDisk()
    {
        if (!HardDiskAttached || string.IsNullOrEmpty(HardDiskPath)) return;
        byte[]? buf = null;
        try
        {
            lock (_sync)
            {
                if (_h == IntPtr.Zero) return;
                nuint size = Native.omac_q_harddisk_data(_h, null, 0);
                if (size == 0) return;
                buf = new byte[size];
                nuint n = Native.omac_q_harddisk_data(_h, buf, size);
                if (n == 0) return;
            }
            File.WriteAllBytes(HardDiskPath!, buf!);
        }
        catch { /* best-effort persistence */ }
    }

    // ---- input ----
    public void MouseMove(int dx, int dy, bool button)
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_q_mouse(_h, dx, dy, button ? 1 : 0); }
    }

    public void MouseButton(bool down)
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_q_mouse(_h, 0, 0, down ? 1 : 0); }
    }

    public void KeyEvent(int adbCode, bool down)
    {
        lock (_sync) { if (_h != IntPtr.Zero) Native.omac_q_key(_h, adbCode, down ? 1 : 0); }
    }

    public void Dispose()
    {
        _stop = true;
        _worker.Join();
        // Closing the window is, to the guest, having its power cut. Settle
        // the volumes first -- flush what the System is still holding and
        // mark them cleanly unmounted -- or the image we persist below is
        // missing those blocks and carries the "in use" flag that makes the
        // Quadra ROM refuse the disk at the next boot. The emulation thread
        // has already stopped, so the guest CPU is ours to run.
        SettleVolumes();
        // Whatever the guest saved into the drop box only exists in the volume
        // image until this runs -- and the image dies with the machine.
        SyncFolderDisk();
        // A disk still in the drive at closing time has never been saved --
        // nothing ejected it -- so save it here, as the hard disk is saved.
        WriteBackFloppy();
        WriteBackHardDisk();
        Destroy();
        _audio.Dispose();
    }

    private void Destroy()
    {
        if (_h != IntPtr.Zero)
        {
            Native.omac_q_destroy(_h);
            _h = IntPtr.Zero;
        }
    }
}
