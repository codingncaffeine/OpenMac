using System.IO;
using System.Runtime.InteropServices;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// The folder disk's two halves: build an HFS volume from a host folder
/// (through the core's builder), and sync a returned volume image back to the
/// folder. MacBinary (.bin) files decode into forked Mac files on the way in;
/// files the guest gave a resource fork come back out as MacBinary. Host
/// files are never silently deleted — files the guest removed move to
/// _openmac-removed. The volume is truth at sync time, the folder at build
/// time; editing the folder while the machine runs is the later live-sync
/// phase's problem, not this one's.
/// </summary>
internal static class FolderDisk
{
    private const string RemovedDir = "_openmac-removed";

    // Finder infrastructure the guest creates on every volume; syncing it to
    // the host folder would just be noise.
    private static readonly string[] GuestNoise =
    {
        "Desktop", "Desktop DB", "Desktop DF", "Desktop Folder", "Trash",
        "Temporary Items", "Network Trash Folder", "TheVolumeSettingsFolder",
        "Move&Rename", "AutoRecover",
    };

    /// <summary>
    /// Is this a root-level item the guest made for its own use, rather than
    /// something the user put there? Extensions name their scratch folders after
    /// themselves — SpaceSaver writes "SpaceSaver Temporary Items" — so the
    /// suffix has to count, not just the exact names. Getting this wrong is
    /// visible: the folder is created on the host, and then it is the only thing
    /// the next rebuild has to work from.
    /// </summary>
    private static bool IsGuestNoise(string name) =>
        Array.Exists(GuestNoise, n => n.Equals(name, StringComparison.OrdinalIgnoreCase)) ||
        name.EndsWith("Temporary Items", StringComparison.OrdinalIgnoreCase);

    private static readonly string[] HostNoise = { "thumbs.db", "desktop.ini" };

    // ---- native surface -------------------------------------------------
    [StructLayout(LayoutKind.Sequential)]
    private struct HfsItem
    {
        public uint id, parent;
        public int isDir;
        public uint type, creator;
        public uint fdFlags;
        public uint crDate, mdDate;
        public uint dataLen, rsrcLen;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 64)] public byte[] name;
    }

    private const string Dll = "openmac_c";

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr omac_hfsb_begin([MarshalAs(UnmanagedType.LPStr)] string name);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    private static extern uint omac_hfsb_add_dir(IntPtr b, uint parent,
        [MarshalAs(UnmanagedType.LPStr)] string name, uint cr, uint md);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    private static extern void omac_hfsb_add_file(IntPtr b, uint parent,
        [MarshalAs(UnmanagedType.LPStr)] string name, uint type, uint creator,
        ushort fdFlags, byte[]? data, nuint dataLen, byte[]? rsrc, nuint rsrcLen,
        uint cr, uint md);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    private static extern nuint omac_hfsb_build(IntPtr b, byte[]? outBuf, nuint cap);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    private static extern nuint omac_hfsb_build_sized(IntPtr b, uint sizeBytes,
                                                      byte[]? outBuf, nuint cap);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    private static extern nuint omac_hfsb_error(IntPtr b, byte[] outBuf, nuint cap);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    private static extern void omac_hfsb_free(IntPtr b);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr omac_hfsr_open(byte[] img, nuint len);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    private static extern int omac_hfsr_count(IntPtr r);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    private static extern int omac_hfsr_item(IntPtr r, int index, out HfsItem item);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    private static extern nuint omac_hfsr_fork(IntPtr r, uint fileId, int rsrc,
                                               byte[]? outBuf, nuint cap);
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    private static extern void omac_hfsr_free(IntPtr r);

    // ---- dates ----------------------------------------------------------
    private static readonly DateTime HfsEpoch = new(1904, 1, 1);

    private static uint ToHfsDate(DateTime local)
    {
        double s = (local - HfsEpoch).TotalSeconds;
        return s <= 0 ? 0 : s >= uint.MaxValue ? uint.MaxValue : (uint)s;
    }

    private static DateTime FromHfsDate(uint secs) => HfsEpoch.AddSeconds(secs);

    // ---- building -------------------------------------------------------

    /// <summary>Build an HFS volume image from a folder. Null on failure with
    /// the builder's reason (or the walk's) in <paramref name="error"/>.</summary>
    public static byte[]? Build(string folder, out string error)
    {
        error = "";
        IntPtr b = omac_hfsb_begin(Path.GetFileName(folder.TrimEnd('\\', '/')));
        if (b == IntPtr.Zero) { error = "builder failed to start"; return null; }
        try
        {
            AddTree(b, 2, new DirectoryInfo(folder));
            nuint size = omac_hfsb_build(b, null, 0);
            if (size == 0)
            {
                byte[] msg = new byte[512];
                omac_hfsb_error(b, msg, (nuint)msg.Length);
                int n = Array.IndexOf(msg, (byte)0);
                error = System.Text.Encoding.ASCII.GetString(msg, 0, n < 0 ? msg.Length : n);
                if (error.Length == 0) error = "the folder did not fit an HFS volume";
                return null;
            }
            byte[] img = new byte[size];
            omac_hfsb_build(b, img, (nuint)img.Length);
            return img;
        }
        catch (Exception ex)
        {
            error = ex.Message;
            return null;
        }
        finally
        {
            omac_hfsb_free(b);
        }
    }

    private static void AddTree(IntPtr b, uint parent, DirectoryInfo dir)
    {
        foreach (var sub in dir.EnumerateDirectories())
        {
            if (sub.Name.Equals(RemovedDir, StringComparison.OrdinalIgnoreCase)) continue;
            // Guest scratch that an earlier sync left on the host: do not build
            // it back into the volume. The guest makes its own when it wants
            // one, and a stale copy is just clutter on the Mac's desktop.
            if (parent == 2 && IsGuestNoise(sub.Name)) continue;
            uint id = omac_hfsb_add_dir(b, parent, sub.Name,
                ToHfsDate(sub.CreationTime), ToHfsDate(sub.LastWriteTime));
            if (id != 0) AddTree(b, id, sub);
        }
        foreach (var f in dir.EnumerateFiles())
        {
            if (Array.Exists(HostNoise,
                    n => n.Equals(f.Name, StringComparison.OrdinalIgnoreCase)))
                continue;
            byte[] bytes;
            try { bytes = File.ReadAllBytes(f.FullName); } catch { continue; }
            uint cr = ToHfsDate(f.CreationTime), md = ToHfsDate(f.LastWriteTime);
            // MacBinary is sniffed whatever the file is called, as the transfer
            // volume already does. A Macintosh application lives in its
            // RESOURCE fork, which a plain copy off the web does not carry, so
            // a wrapper that does is the only way most software arrives intact
            // -- and period downloads wear one under every extension there is,
            // or none. Refusing to look unless the name ends in .bin means an
            // application arrives as an empty icon that will not open.
            if (MacBinary.TryDecode(bytes, out var mb))
            {
                // The HOST name is the guest name, minus a .bin if it wore one,
                // so the sync-back mapping stays bijective regardless of what
                // the MacBinary header called the file.
                string guestName = f.Extension.Equals(".bin", StringComparison.OrdinalIgnoreCase)
                    ? Path.GetFileNameWithoutExtension(f.Name) : f.Name;
                omac_hfsb_add_file(b, parent, guestName,
                    mb.Type, mb.Creator, mb.Flags, mb.Data, (nuint)mb.Data.Length,
                    mb.Rsrc, (nuint)mb.Rsrc.Length, cr, md);
            }
            else
            {
                (uint type, uint creator) = InferTypeCreator(f.Extension);
                omac_hfsb_add_file(b, parent, f.Name, type, creator, 0,
                    bytes, (nuint)bytes.Length, null, 0, cr, md);
            }
        }
    }

    private static (uint, uint) InferTypeCreator(string ext) => ext.ToLowerInvariant() switch
    {
        ".txt" or ".text" or ".md" => (0x54455854u, 0x74747874u),   // TEXT/ttxt
        ".sit" => (0x53495444u, 0x53495421u),                        // SITD/SIT!
        ".sea" => (0x4150504Cu, 0x61757374u),                        // APPL/aust
        ".cpt" => (0x50414354u, 0x43504354u),                        // PACT/CPCT
        ".hqx" => (0x54455854u, 0x426E4871u),                        // TEXT/BnHq
        ".zip" => (0x5A495020u, 0x5A495020u),                        // ZIP /ZIP
        ".lha" or ".lzh" => (0x4C484120u, 0x4C415243u),              // LHA /LARC
        _ => (0x3F3F3F3Fu, 0x3F3F3F3Fu),                             // ????/????
    };

    // ---- transfer floppy -------------------------------------------------

    /// <summary>Exactly a 1.44 MB floppy: the size the internal drive takes.</summary>
    public const int TransferFloppyBytes = 1474560;

    /// <summary>
    /// Wrap one loose host file (a .sit, a .sea, anything StuffIt-era) in a
    /// tight-packed 1.44 MB HFS floppy image, named after the file. MacBinary
    /// wrapping is sniffed regardless of extension — period downloads often
    /// wear it silently — so forked files arrive whole. The caller inserts the
    /// image write-protected; the guest only copies off it.
    /// </summary>
    public static byte[]? BuildTransferFloppy(string filePath, out string error) =>
        BuildTransferVolume(filePath, TransferFloppyBytes, out error);

    /// <summary>Same wrapping at whatever size fits: sizeBytes 0 auto-sizes,
    /// which is how multi-megabyte archives ride the second SCSI disk.</summary>
    public static byte[]? BuildTransferVolume(string filePath, uint sizeBytes,
                                              out string error)
    {
        error = "";
        byte[] bytes;
        try { bytes = File.ReadAllBytes(filePath); }
        catch (Exception ex) { error = ex.Message; return null; }
        string stem = Path.GetFileNameWithoutExtension(filePath);
        IntPtr b = omac_hfsb_begin(stem.Length == 0 ? "Transfer" : stem);
        if (b == IntPtr.Zero) { error = "builder failed to start"; return null; }
        try
        {
            uint cr = ToHfsDate(File.GetCreationTime(filePath));
            uint md = ToHfsDate(File.GetLastWriteTime(filePath));
            if (MacBinary.TryDecode(bytes, out var mb))
                omac_hfsb_add_file(b, 2, stem, mb.Type, mb.Creator, mb.Flags,
                    mb.Data, (nuint)mb.Data.Length, mb.Rsrc, (nuint)mb.Rsrc.Length,
                    cr, md);
            else
            {
                (uint type, uint creator) = InferTypeCreator(Path.GetExtension(filePath));
                omac_hfsb_add_file(b, 2, Path.GetFileName(filePath), type, creator, 0,
                    bytes, (nuint)bytes.Length, null, 0, cr, md);
            }
            nuint size = sizeBytes != 0 ? omac_hfsb_build_sized(b, sizeBytes, null, 0)
                                        : omac_hfsb_build(b, null, 0);
            if (size == 0)
            {
                byte[] msg = new byte[512];
                omac_hfsb_error(b, msg, (nuint)msg.Length);
                int n = Array.IndexOf(msg, (byte)0);
                error = System.Text.Encoding.ASCII.GetString(msg, 0, n < 0 ? msg.Length : n);
                if (error.Length == 0)
                    error = sizeBytes != 0 ? "it does not fit a 1.44 MB transfer floppy"
                                           : "the volume build failed";
                return null;
            }
            byte[] img = new byte[size];
            if (sizeBytes != 0) omac_hfsb_build_sized(b, sizeBytes, img, (nuint)img.Length);
            else omac_hfsb_build(b, img, (nuint)img.Length);
            return img;
        }
        catch (Exception ex)
        {
            error = ex.Message;
            return null;
        }
        finally
        {
            omac_hfsb_free(b);
        }
    }

    // ---- sync back ------------------------------------------------------

    public readonly record struct SyncResult(int Updated, int Added, int Removed, string? Error);

    /// <summary>Write the guest's changes on a volume image back into the folder.</summary>
    public static SyncResult SyncBack(byte[] image, string folder)
    {
        IntPtr r = omac_hfsr_open(image, (nuint)image.Length);
        if (r == IntPtr.Zero)
            return new SyncResult(0, 0, 0,
                "the volume did not read back as HFS — folder left untouched");
        try
        {
            int count = omac_hfsr_count(r);
            var items = new List<HfsItem>(count);
            for (int i = 0; i < count; ++i)
                if (omac_hfsr_item(r, i, out var it) != 0)
                    items.Add(it);

            // Guest paths, rooted at the folder. The root itself is item id 2.
            var pathOf = new Dictionary<uint, string> { [2] = "" };
            // Parents come before children in catalog order is NOT guaranteed
            // for ids, so resolve iteratively.
            bool progress = true;
            while (progress)
            {
                progress = false;
                foreach (var it in items)
                {
                    if (it.isDir == 0 || pathOf.ContainsKey(it.id)) continue;
                    // A root-level noise directory never gets a path, so nothing
                    // underneath it can resolve one either and the whole subtree
                    // stays out of the host folder. That matters most for Trash:
                    // giving it a path would copy a file the user just threw away
                    // back to the host as "Trash\thing", while the original was
                    // shelved for having moved.
                    if (it.parent == 2 && IsGuestNoise(NameOf(it))) continue;
                    if (pathOf.TryGetValue(it.parent, out string? pp))
                    {
                        pathOf[it.id] = Path.Combine(pp, HostName(it));
                        progress = true;
                    }
                }
            }

            int updated = 0, added = 0, removed = 0;
            var guestPaths = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

            foreach (var it in items)
            {
                if (it.id == 2) continue;
                if (!pathOf.TryGetValue(it.parent, out string? parentPath)) continue;
                string name = HostName(it);
                if (parentPath.Length == 0 && IsGuestNoise(NameOf(it))) continue;
                if (it.isDir != 0)
                {
                    string dir = Path.Combine(folder, parentPath, name);
                    guestPaths.Add(Path.Combine(parentPath, name));
                    if (!Directory.Exists(dir)) { Directory.CreateDirectory(dir); ++added; }
                    continue;
                }

                byte[] data = ReadFork(r, it.id, false, it.dataLen);
                byte[] rsrc = it.rsrcLen != 0 ? ReadFork(r, it.id, true, it.rsrcLen)
                                              : Array.Empty<byte>();
                bool asMacBinary = rsrc.Length != 0;
                // A forked file goes back as MacBinary. Which host name it goes
                // back UNDER has to be the one it came in under, or the pairing
                // stops being one-to-one: a wrapper called "App.sea" would come
                // back as "App.sea.bin" beside it, and the next build would add
                // BOTH to the volume under the same guest name. So if the host
                // already holds this file as a wrapper under its own name, write
                // there; only mint a .bin when there is nothing to write back to.
                string plainRel = Path.Combine(parentPath, name);
                string rel = !asMacBinary || IsMacBinaryFile(Path.Combine(folder, plainRel))
                    ? plainRel
                    : Path.Combine(parentPath, name + ".bin");
                guestPaths.Add(rel);
                string full = Path.Combine(folder, rel);
                byte[] outBytes = asMacBinary
                    ? MacBinary.Encode(NameOf(it), it.type, it.creator,
                                       (ushort)it.fdFlags, data, rsrc,
                                       it.crDate, it.mdDate)
                    : data;

                var fi = new FileInfo(full);
                bool changed = !fi.Exists || fi.Length != outBytes.Length ||
                               Math.Abs((fi.LastWriteTime - FromHfsDate(it.mdDate))
                                            .TotalSeconds) > 2;
                if (!changed) continue;
                Directory.CreateDirectory(Path.GetDirectoryName(full)!);
                File.WriteAllBytes(full, outBytes);
                try { File.SetLastWriteTime(full, FromHfsDate(it.mdDate)); } catch { }
                if (fi.Exists) ++updated; else ++added;
            }

            // Files the guest deleted: move to the removed shelf, never erase.
            string removedRoot = Path.Combine(folder, RemovedDir);
            foreach (string file in Directory.EnumerateFiles(folder, "*",
                         SearchOption.AllDirectories))
            {
                string rel = Path.GetRelativePath(folder, file);
                if (rel.StartsWith(RemovedDir, StringComparison.OrdinalIgnoreCase)) continue;
                if (Array.Exists(HostNoise, n =>
                        n.Equals(Path.GetFileName(file), StringComparison.OrdinalIgnoreCase)))
                    continue;
                // A MacBinary source whose decoded twin still exists is present.
                string relNoBin = rel.EndsWith(".bin", StringComparison.OrdinalIgnoreCase)
                    ? rel[..^4] : rel;
                if (guestPaths.Contains(rel) || guestPaths.Contains(relNoBin) ||
                    guestPaths.Contains(rel + ".bin"))
                    continue;
                string dest = Path.Combine(removedRoot, rel);
                Directory.CreateDirectory(Path.GetDirectoryName(dest)!);
                try
                {
                    if (File.Exists(dest)) File.Delete(dest);
                    File.Move(file, dest);
                    ++removed;
                }
                catch { /* locked file: leave it */ }
            }
            return new SyncResult(updated, added, removed, null);
        }
        catch (Exception ex)
        {
            return new SyncResult(0, 0, 0, ex.Message);
        }
        finally
        {
            omac_hfsr_free(r);
        }
    }

    /// <summary>Does this host file already carry a MacBinary wrapper? Only the
    /// header is read: this runs per file during a sync and the answer is in the
    /// first 128 bytes.</summary>
    private static bool IsMacBinaryFile(string path)
    {
        try
        {
            var fi = new FileInfo(path);
            if (!fi.Exists || fi.Length < 128) return false;
            byte[] head = new byte[128];
            using (var fs = File.OpenRead(path))
                if (fs.Read(head, 0, 128) != 128) return false;
            // Same shape MacBinary.TryDecode insists on, minus the fork lengths
            // it cannot check without the whole file.
            if (head[0] != 0 || head[74] != 0) return false;
            int nameLen = head[1];
            if (nameLen < 1 || nameLen > 63) return false;
            uint dataLen = ((uint)head[83] << 24) | ((uint)head[84] << 16) |
                           ((uint)head[85] << 8) | head[86];
            uint rsrcLen = ((uint)head[87] << 24) | ((uint)head[88] << 16) |
                           ((uint)head[89] << 8) | head[90];
            if (dataLen > 0x00FFFFFF || rsrcLen > 0x00FFFFFF) return false;
            long need = 128 + ((dataLen + 127) & ~127u) + ((rsrcLen + 127) & ~127u);
            return fi.Length >= need;
        }
        catch { return false; }
    }

    private static byte[] ReadFork(IntPtr r, uint id, bool rsrc, uint len)
    {
        if (len == 0) return Array.Empty<byte>();
        byte[] buf = new byte[len];
        nuint n = omac_hfsr_fork(r, id, rsrc ? 1 : 0, buf, (nuint)buf.Length);
        if (n != (nuint)buf.Length) Array.Resize(ref buf, (int)n);
        return buf;
    }

    private static string NameOf(in HfsItem it)
    {
        int n = Array.IndexOf(it.name, (byte)0);
        return System.Text.Encoding.ASCII.GetString(it.name, 0, n < 0 ? it.name.Length : n);
    }

    /// <summary>Guest name made safe for a Windows path.</summary>
    private static string HostName(in HfsItem it)
    {
        string s = NameOf(it);
        char[] bad = Path.GetInvalidFileNameChars();
        var sb = new System.Text.StringBuilder(s.Length);
        foreach (char c in s) sb.Append(Array.IndexOf(bad, c) >= 0 ? '_' : c);
        string outName = sb.ToString().TrimEnd(' ', '.');
        return outName.Length == 0 ? "_" : outName;
    }
}

/// <summary>MacBinary I: the 128-byte header + padded forks that archived Mac
/// software travels in. Encode writes MacBinary I (universally readable);
/// decode accepts I and II.</summary>
internal static class MacBinary
{
    public readonly record struct Decoded(uint Type, uint Creator, ushort Flags,
                                          byte[] Data, byte[] Rsrc);

    public static bool TryDecode(byte[] b, out Decoded d)
    {
        d = default;
        if (b.Length < 128 || b[0] != 0 || b[74] != 0) return false;
        int nameLen = b[1];
        if (nameLen < 1 || nameLen > 63) return false;
        uint dataLen = Be32(b, 83), rsrcLen = Be32(b, 87);
        long need = 128 + Pad128(dataLen) + Pad128(rsrcLen);
        if (dataLen > 0x00FFFFFF || rsrcLen > 0x00FFFFFF || b.Length < need) return false;
        byte[] data = new byte[dataLen];
        Array.Copy(b, 128, data, 0, dataLen);
        byte[] rsrc = new byte[rsrcLen];
        Array.Copy(b, 128 + Pad128(dataLen), rsrc, 0, rsrcLen);
        d = new Decoded(Be32(b, 65), Be32(b, 69),
                        (ushort)((b[73] << 8) | b[101]), data, rsrc);
        return true;
    }

    public static byte[] Encode(string name, uint type, uint creator, ushort flags,
                                byte[] data, byte[] rsrc, uint crDate, uint mdDate)
    {
        byte[] o = new byte[128 + Pad128((uint)data.Length) + Pad128((uint)rsrc.Length)];
        byte[] nm = System.Text.Encoding.ASCII.GetBytes(name);
        int nlen = Math.Min(nm.Length, 63);
        o[1] = (byte)nlen;
        Array.Copy(nm, 0, o, 2, nlen);
        PutBe32(o, 65, type);
        PutBe32(o, 69, creator);
        o[73] = (byte)(flags >> 8);
        PutBe32(o, 83, (uint)data.Length);
        PutBe32(o, 87, (uint)rsrc.Length);
        PutBe32(o, 91, crDate);
        PutBe32(o, 95, mdDate);
        o[101] = (byte)flags;
        Array.Copy(data, 0, o, 128, data.Length);
        Array.Copy(rsrc, 0, o, 128 + (int)Pad128((uint)data.Length), rsrc.Length);
        return o;
    }

    private static long Pad128(uint n) => (n + 127) & ~127u;
    private static uint Be32(byte[] b, int p) =>
        ((uint)b[p] << 24) | ((uint)b[p + 1] << 16) | ((uint)b[p + 2] << 8) | b[p + 3];
    private static void PutBe32(byte[] b, int p, uint v)
    {
        b[p] = (byte)(v >> 24); b[p + 1] = (byte)(v >> 16);
        b[p + 2] = (byte)(v >> 8); b[p + 3] = (byte)v;
    }
}
