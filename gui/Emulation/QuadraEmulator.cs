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
    public int ScreenWidth => 640;
    public int ScreenHeight => 480;
    public string BackendName => "native-quadra";
    public bool IsRealCore => true;

    public bool IsRomLoaded => _h != IntPtr.Zero;
    public string? RomPath { get; private set; }
    public string? FloppyPath => null;
    public string? ExternalFloppyPath => null;
    public bool ExternalDriveAttached => false;
    public bool HardDiskAttached { get; private set; }
    public string? HardDiskPath { get; private set; }

    private IntPtr _h;
    private readonly WaveAudio _audio;
    private readonly object _sync = new();
    private readonly object _frameLock = new();
    private readonly byte[] _emuFrame = new byte[640 * 480 * 4];
    private readonly byte[] _sharedFrame = new byte[640 * 480 * 4];
    private readonly byte[] _audioBuf = new byte[8192];
    private bool _frameDirty;
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
        WriteBackHardDisk();
        byte[] rom = File.ReadAllBytes(path);
        lock (_sync)
        {
            Destroy();
            _h = Native.omac_q_create(rom, (nuint)rom.Length, (uint)ramMB);
            if (_h == IntPtr.Zero)
                throw new InvalidOperationException("Quadra core failed to initialize (bad ROM or size?)");
        }
        RomPath = path;
        HardDiskAttached = false;
        HardDiskPath = null;
        Log.Line($"[core] Quadra 650 created — {ramMB} MB, ROM {Path.GetFileName(path)}");
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
                        if (n > 0) _audio.Feed(_audioBuf, n);
                    }
                }
                acc -= FrameSeconds;
                ran++;
                fpsFrames++;
            }
            if (acc > FrameSeconds) acc = FrameSeconds;
            if (ran > 0) PublishFrame();

            if (fpsElapsed >= 1.0)
            {
                string audio = _audio.Ok ? _audio.Stats() : "audio: (no device)";
                Log.Line($"perf: fps={fpsFrames / fpsElapsed:F1}  {audio}");
                fpsFrames = 0;
                fpsElapsed = 0;
            }

            Thread.Sleep(1);
        }
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

    // ---- media the Quadra build doesn't carry yet ----
    public bool InsertFloppy(string path) => false;
    public void EjectFloppy() { }
    public string MediumNote(int drive) => "The Quadra 650 build has no floppy drive yet — boot from a SCSI hard disk.";
    public void SetExternalDrive(bool attached) { }
    public bool InsertExternalFloppy(string path) => false;
    public void EjectExternalFloppy() { }
    public bool WriteProtectFloppies { get; set; }
    public bool ConsumeDiskStateChanged() => false;
    public bool NetworkingEnabled => false;
    public void SetNetworking(bool enabled) { }
    public string? FolderDiskPath => null;
    public bool AttachFolderDisk(string folder, out string error)
    {
        error = "Folder disks are not on the Quadra 650 yet.";
        return false;
    }
    public void DetachFolderDisk() { }
    public string? TransferDiskLabel => null;
    public bool TransferDiskResident => false;
    public bool AttachTransferDisk(string filePath, out string error)
    {
        error = "Transfer disks are not on the Quadra 650 yet.";
        return false;
    }
    public bool CdRomAttached => false;
    public string? CdPath => null;
    public void SetCdRomAttached(bool attached) { }
    public bool InsertCd(string path) => false;
    public void EjectCd() { }
    public bool CdPresent => false;
    public string CdMediumNote() => "The CD-ROM drive is not on the Quadra 650 yet.";

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
