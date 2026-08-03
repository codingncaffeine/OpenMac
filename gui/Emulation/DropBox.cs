using System.IO;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// The drop box: one host folder kept in front of the Mac as a real HFS volume
/// on the second SCSI seat. A file dropped on the window lands in the folder,
/// and the volume is REPUBLISHED so the Mac sees it — no new disk per file, no
/// restart, nothing to drag to the Trash afterwards.
///
/// Republishing is a swap, not an edit. A mounted HFS volume is not the disk:
/// the guest caches its catalog, its extents and its allocation bitmap, so
/// writing a new file into the image behind its back leaves it reading
/// yesterday's catalog against today's blocks. The volume is instead flushed
/// and unmounted, rebuilt on the host, and put back — which is what happens
/// when a removable cartridge is swapped, and a sequence the System already
/// handles natively.
///
/// The stages are split across threads on purpose. Guest-side steps (the
/// unmount, the insert) only happen between frames on the emulation thread,
/// because injected traps belong to the thread that owns the CPU. The rebuild
/// is host file I/O and slow enough that doing it there would visibly stall the
/// machine, so it runs on a task in between. <see cref="Pump"/> drives one
/// stage per frame batch; everything else is the caller asking.
///
/// Both machines use this — the Classic and the Quadra differ only in which
/// native entry points do the unmount, the read and the insert, which is what
/// the three delegates are.
/// </summary>
internal sealed class DropBoxSeat
{
    private enum Stage { Idle, WantUnmount, Building, WantInsert }

    private readonly Func<bool> _unmount;      // flush + unmount drive 5; true = off line
    private readonly Func<byte[]?> _readImage; // the seat's volume image, or null
    private readonly Action<byte[]> _insert;   // put a rebuilt volume on the seat

    private volatile Stage _stage = Stage.Idle;
    private int _tries;
    private byte[]? _rebuilt;
    private readonly object _rebuiltLock = new();

    // ~5 s at 60 Hz. The core refuses an unmount while a file operation is in
    // flight, inside its own driver, or mid-interrupt — all of which pass
    // within a frame or two. A file the guest still has OPEN on the volume
    // never passes, and that is what this bound is for.
    private const int MaxTries = 300;

    public DropBoxSeat(Func<bool> unmount, Func<byte[]?> readImage, Action<byte[]> insert)
    {
        _unmount = unmount;
        _readImage = readImage;
        _insert = insert;
    }

    /// <summary>The host folder on the seat, or null when nothing is attached.</summary>
    public string? Folder { get; set; }

    /// <summary>True while a republish is still working through its stages.</summary>
    public bool Pending => _stage != Stage.Idle;

    public void Cancel() => _stage = Stage.Idle;

    /// <summary>
    /// Copy <paramref name="addFile"/> into the folder (when given) and start a
    /// republish. Returns false only when there is no folder or the copy failed
    /// — the guest-side swap itself happens over the next few frames.
    /// </summary>
    public bool Request(string? addFile, out string error)
    {
        error = "";
        string? folder = Folder;
        if (string.IsNullOrEmpty(folder)) { error = "no drop box is attached"; return false; }
        if (!string.IsNullOrEmpty(addFile))
        {
            try { CopyIn(folder!, addFile!); }
            catch (Exception ex) { error = ex.Message; return false; }
        }
        lock (_rebuiltLock) _rebuilt = null;
        _tries = 0;
        _stage = Stage.WantUnmount;
        return true;
    }

    // Never quietly overwrite what is already in the user's folder: a second
    // Game.sit becomes "Game (2).sit", the way a Downloads folder behaves.
    // Dropping a file that already lives in the folder is a plain republish.
    private static void CopyIn(string folder, string addFile)
    {
        Directory.CreateDirectory(folder);
        string dest = Path.Combine(folder, Path.GetFileName(addFile));
        if (SamePath(dest, addFile)) return;
        if (File.Exists(dest))
        {
            string stem = Path.GetFileNameWithoutExtension(dest);
            string ext = Path.GetExtension(dest);
            for (int n = 2; File.Exists(dest); ++n)
                dest = Path.Combine(folder, $"{stem} ({n}){ext}");
        }
        File.Copy(addFile, dest);
        Log.Line($"[dropbox] {Path.GetFileName(dest)} -> {folder}");
    }

    private static bool SamePath(string a, string b)
    {
        try
        {
            return string.Equals(Path.GetFullPath(a), Path.GetFullPath(b),
                                 StringComparison.OrdinalIgnoreCase);
        }
        catch { return false; }
    }

    /// <summary>Fold the guest's current volume back into the host folder.</summary>
    public void Sync()
    {
        string? folder = Folder;
        if (string.IsNullOrEmpty(folder)) return;
        byte[]? img = _readImage();
        if (img is null) return;
        var r = FolderDisk.SyncBack(img, folder!);
        Log.Line(r.Error is null
            ? $"[dropbox] synced: {r.Updated} updated, {r.Added} added, "
              + $"{r.Removed} moved to _openmac-removed"
            : $"[dropbox] sync FAILED: {r.Error}");
    }

    /// <summary>One stage per call. Emulation thread only.</summary>
    public void Pump()
    {
        switch (_stage)
        {
            case Stage.Idle:
            case Stage.Building:            // the background task moves this on
                return;

            case Stage.WantUnmount:
            {
                if (!_unmount())
                {
                    if (++_tries < MaxTries) return;
                    Log.Line("[dropbox] the Mac would not let go of the volume — "
                             + "something is still open on it. Republish abandoned; "
                             + "close it in the Finder and drop the file again.");
                    _stage = Stage.Idle;
                    return;
                }
                // The image now holds everything the guest wrote. Fold that back
                // into the folder and rebuild — off this thread.
                byte[]? img = _readImage();
                string? folder = Folder;
                _stage = Stage.Building;
                System.Threading.Tasks.Task.Run(() => Rebuild(img, folder));
                return;
            }

            case Stage.WantInsert:
            {
                byte[]? img;
                lock (_rebuiltLock) { img = _rebuilt; _rebuilt = null; }
                if (img is not null)
                {
                    _insert(img);
                    Log.Line($"[dropbox] republished ({img.Length / (1024 * 1024)} MB) — "
                             + "the volume comes back in a moment");
                }
                _stage = Stage.Idle;
                return;
            }
        }
    }

    private void Rebuild(byte[]? current, string? folder)
    {
        byte[]? next = null;
        try
        {
            if (string.IsNullOrEmpty(folder)) return;
            // Sync BEFORE rebuilding: whatever the guest wrote to the volume is
            // only in the image, and the rebuild reads the folder. Skipping this
            // would throw away everything saved to the drop box since it was
            // last published.
            if (current is not null)
            {
                var s = FolderDisk.SyncBack(current, folder!);
                if (s.Error is not null)
                    Log.Line($"[dropbox] sync back failed: {s.Error} — "
                             + "rebuilding from the folder as it stands");
            }
            next = FolderDisk.Build(folder!, out string buildError);
            if (next is null)
                Log.Line($"[dropbox] rebuild failed: {buildError} — "
                         + "the volume stays off line");
        }
        catch (Exception ex)
        {
            Log.Line("[dropbox] rebuild threw: " + ex.Message);
            next = null;
        }
        finally
        {
            lock (_rebuiltLock) _rebuilt = next;
            _stage = Stage.WantInsert;
        }
    }

    // ---- the default location -------------------------------------------

    /// <summary>
    /// Where a drop box lives when the user has not chosen a folder: a plainly
    /// named folder under Documents, made on demand. Somewhere they can find in
    /// Explorer without being told, and which survives the app being reinstalled
    /// — a temp directory would silently eat the files they put in it.
    /// </summary>
    public static string DefaultFolder =>
        Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments),
            "OpenMac", "Drop Box");

    /// <summary>The default folder, created if it is not there yet. Returns null
    /// if it could not be made, so the caller can say so rather than fail
    /// somewhere less obvious.</summary>
    public static string? EnsureDefaultFolder()
    {
        try
        {
            string dir = DefaultFolder;
            Directory.CreateDirectory(dir);
            return dir;
        }
        catch (Exception ex)
        {
            Log.Line("[dropbox] could not create the default folder: " + ex.Message);
            return null;
        }
    }
}
