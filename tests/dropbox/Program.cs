using System.IO;
using System.Runtime.InteropServices;
using OpenMac.Gui.Emulation;

/// <summary>Reads a built volume back, for assertions. Its own P/Invokes rather
/// than a test hatch opened in FolderDisk: the production type keeps its reader
/// private, and this is checking the same DLL either way.</summary>
static class FolderDiskProbe
{
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

    private static string NameOf(in HfsItem it)
    {
        int n = Array.IndexOf(it.name, (byte)0);
        return System.Text.Encoding.ASCII.GetString(it.name, 0, n < 0 ? it.name.Length : n);
    }

    public static List<(string Name, bool IsDir, uint DataLen)> List(byte[] img)
    {
        var outp = new List<(string, bool, uint)>();
        IntPtr r = omac_hfsr_open(img, (nuint)img.Length);
        if (r == IntPtr.Zero) return outp;
        try
        {
            int count = omac_hfsr_count(r);
            for (int i = 0; i < count; ++i)
                if (omac_hfsr_item(r, i, out var it) != 0 && it.id != 2)
                    outp.Add((NameOf(it), it.isDir != 0, it.dataLen));
        }
        finally { omac_hfsr_free(r); }
        return outp;
    }

    public static byte[]? Rsrc(byte[] img, string name) => Fork(img, name, true);

    public static byte[]? Fork(byte[] img, string name) => Fork(img, name, false);

    private static byte[]? Fork(byte[] img, string name, bool rsrc)
    {
        IntPtr r = omac_hfsr_open(img, (nuint)img.Length);
        if (r == IntPtr.Zero) return null;
        try
        {
            int count = omac_hfsr_count(r);
            for (int i = 0; i < count; ++i)
            {
                if (omac_hfsr_item(r, i, out var it) == 0) continue;
                if (it.isDir != 0 || NameOf(it) != name) continue;
                byte[] buf = new byte[rsrc ? it.rsrcLen : it.dataLen];
                nuint n = omac_hfsr_fork(r, it.id, rsrc ? 1 : 0, buf, (nuint)buf.Length);
                if (n != (nuint)buf.Length) Array.Resize(ref buf, (int)n);
                return buf;
            }
        }
        finally { omac_hfsr_free(r); }
        return null;
    }
}

// Drop-box republish harness. Drives the SHIPPED DropBoxSeat + FolderDisk with
// fake native delegates, so the sequence under test is the one the app runs --
// the C++ trace tool has no sync-back at all, which is exactly how the "dropped
// file never arrives" bug got past it.
//
// The guest is modelled by handing the seat a volume image that carries what a
// real guest leaves behind: its desktop database and an extension's scratch
// folder. That is the input the bug needed; a volume built straight from the
// host folder would have passed either way.

static class Program
{
    static int failures;

    static void Check(bool ok, string what)
    {
        Console.WriteLine((ok ? "  PASS  " : "  FAIL  ") + what);
        if (!ok) failures++;
    }

    static string NewDir(string name)
    {
        string p = Path.Combine(Path.GetTempPath(), "omac-dbtest", name);
        if (Directory.Exists(p)) Directory.Delete(p, true);
        Directory.CreateDirectory(p);
        return p;
    }

    static List<string> VolumeNames(byte[] img)
    {
        var names = new List<string>();
        foreach (var (name, _, _) in FolderDiskProbe.List(img)) names.Add(name);
        return names;
    }

    static int Main()
    {
        Console.WriteLine("== drop box republish ==");

        // A drop box that already holds one file, as a returning user's would.
        string folder = NewDir("box");
        byte[] keep = new byte[40000];
        for (int i = 0; i < keep.Length; i++) keep[i] = (byte)(i * 7);
        File.WriteAllBytes(Path.Combine(folder, "existing.sit"), keep);

        // The file about to be dropped, from somewhere else entirely.
        string outside = NewDir("outside");
        string dropped = Path.Combine(outside, "Archive.hqx");
        byte[] droppedBytes = new byte[12345];
        for (int i = 0; i < droppedBytes.Length; i++) droppedBytes[i] = (byte)(i ^ 0x5A);
        File.WriteAllBytes(dropped, droppedBytes);

        // What the guest hands back: the folder's contents plus the scratch a
        // real System leaves on a mounted volume.
        string asGuestLeftIt = NewDir("guest");
        File.WriteAllBytes(Path.Combine(asGuestLeftIt, "existing.sit"), keep);
        File.WriteAllBytes(Path.Combine(asGuestLeftIt, "Desktop DB"), new byte[2048]);
        File.WriteAllBytes(Path.Combine(asGuestLeftIt, "Desktop DF"), new byte[2]);
        Directory.CreateDirectory(Path.Combine(asGuestLeftIt, "SpaceSaver Temporary Items"));
        byte[]? guestImage = FolderDisk.Build(asGuestLeftIt, out string be);
        Check(guestImage is not null, "a stand-in guest volume builds" + (be.Length > 0 ? " -- " + be : ""));
        if (guestImage is null) return 1;

        byte[]? published = null;
        var seat = new DropBoxSeat(
            unmount: () => true,
            readImage: () => guestImage,
            insert: img => published = img)
        { Folder = folder };

        Check(seat.Request(dropped, out string err), "the drop is accepted" +
              (err.Length > 0 ? " -- " + err : ""));

        // Pump until the state machine settles, the way the frame loop does.
        for (int i = 0; i < 400 && (seat.Pending || published is null); i++)
        {
            seat.Pump();
            Thread.Sleep(10);
        }
        Check(published is not null, "a rebuilt volume comes back");
        if (published is null) return 1;

        var names = VolumeNames(published);
        Console.WriteLine("  volume: " + string.Join(", ", names));

        Check(names.Contains("Archive.hqx"),
              "THE DROPPED FILE IS ON THE VOLUME");
        Check(names.Contains("existing.sit"),
              "the file that was already there survived");
        Check(!names.Exists(n => n.EndsWith("Temporary Items", StringComparison.OrdinalIgnoreCase)),
              "guest scratch was not rebuilt into the volume");

        // And on the host side.
        Check(File.Exists(Path.Combine(folder, "Archive.hqx")),
              "the dropped file is in the host folder");
        Check(!Directory.Exists(Path.Combine(folder, "_openmac-removed", "Archive.hqx")) &&
              !File.Exists(Path.Combine(folder, "_openmac-removed", "Archive.hqx")),
              "the dropped file was NOT shelved as a deletion");
        Check(!Directory.Exists(Path.Combine(folder, "SpaceSaver Temporary Items")),
              "guest scratch did not leak into the host folder");

        // The bytes have to survive the whole round trip.
        var got = FolderDiskProbe.Fork(published, "Archive.hqx");
        Check(got is not null && got.Length == droppedBytes.Length &&
              got.AsSpan().SequenceEqual(droppedBytes),
              $"the dropped file's bytes are intact ({got?.Length ?? -1} of {droppedBytes.Length})");

        // A republish with nothing new must not disturb what is there -- this is
        // the "Refresh Now" path, and it runs over the folder the drop just left.
        Console.WriteLine("== refresh with no new file ==");
        guestImage = published;
        published = null;
        Check(seat.Request(null, out _), "a bare refresh is accepted");
        for (int i = 0; i < 400 && (seat.Pending || published is null); i++)
        {
            seat.Pump();
            Thread.Sleep(10);
        }
        Check(published is not null, "the refresh republishes");
        if (published is not null)
        {
            var again = VolumeNames(published);
            Console.WriteLine("  volume: " + string.Join(", ", again));
            Check(again.Contains("Archive.hqx") && again.Contains("existing.sit"),
                  "both files are still there after a refresh");
        }

        // Moving the drop box to another folder while a volume is mounted. The
        // outgoing folder must still receive what the guest saved, and the
        // incoming folder must become the volume.
        Console.WriteLine("== move to another folder ==");
        string second = NewDir("box2");
        File.WriteAllBytes(Path.Combine(second, "other.sit"), new byte[9000]);

        // The guest saved something new into the drop box before the move.
        string guestSaved = NewDir("guest2");
        File.WriteAllBytes(Path.Combine(guestSaved, "existing.sit"), keep);
        File.WriteAllBytes(Path.Combine(guestSaved, "Archive.hqx"), droppedBytes);
        File.WriteAllBytes(Path.Combine(guestSaved, "MadeOnTheMac"), new byte[777]);
        guestImage = FolderDisk.Build(guestSaved, out _);
        published = null;
        Check(seat.Request(null, second, out _), "the move is accepted");
        for (int i = 0; i < 400 && (seat.Pending || published is null); i++)
        {
            seat.Pump();
            Thread.Sleep(10);
        }
        Check(published is not null, "the move republishes");
        if (published is not null)
        {
            var moved = VolumeNames(published);
            Console.WriteLine("  volume: " + string.Join(", ", moved));
            Check(moved.Contains("other.sit"), "the new folder is now the volume");
            Check(!moved.Contains("existing.sit"),
                  "the old folder's files are no longer on the volume");
        }
        Check(File.Exists(Path.Combine(folder, "MadeOnTheMac")),
              "what the Mac saved reached the OUTGOING folder before the move");
        Check(seat.Folder == second, "the seat now points at the new folder");

        // A Macintosh application lives in its RESOURCE fork, which a plain copy
        // off the web does not carry. MacBinary is how one arrives intact, and
        // period downloads wear it under every extension there is -- so it has
        // to be sniffed by content, not by the name ending in .bin. Getting this
        // wrong means an application shows up as an empty icon that will not
        // open, which is hard to tell from a corrupt download.
        Console.WriteLine("== a forked application arrives whole ==");
        string appBox = NewDir("appbox");
        byte[] appData = new byte[3000];
        byte[] appRsrc = new byte[7000];
        for (int i = 0; i < appData.Length; i++) appData[i] = (byte)(i % 251);
        for (int i = 0; i < appRsrc.Length; i++) appRsrc[i] = (byte)(255 - i % 253);
        // Wrapped, but named the way the download was -- not ".bin".
        byte[] wrapped = MacBinary.Encode("Expander", 0x4150504Cu /*APPL*/, 0x61757374u,
                                          0, appData, appRsrc, 0, 0);
        File.WriteAllBytes(Path.Combine(appBox, "Expander.sea"), wrapped);

        byte[]? appVol = FolderDisk.Build(appBox, out string abe);
        Check(appVol is not null, "the folder builds" + (abe.Length > 0 ? " -- " + abe : ""));
        if (appVol is not null)
        {
            var items = FolderDiskProbe.List(appVol);
            var app = items.Find(t => t.Name == "Expander.sea");
            Check(app.Name is not null, "the application is on the volume");
            Check(app.DataLen == (uint)appData.Length,
                  $"its data fork is the payload, not the wrapper ({app.DataLen} vs {appData.Length})");
            byte[]? rsrc = FolderDiskProbe.Rsrc(appVol, "Expander.sea");
            Check(rsrc is not null && rsrc.Length == appRsrc.Length &&
                  rsrc.AsSpan().SequenceEqual(appRsrc),
                  $"IT KEPT ITS RESOURCE FORK ({rsrc?.Length ?? -1} of {appRsrc.Length})");

            // And back out again: the wrapper must return to the name it came in
            // under, or the next build adds it twice under one guest name.
            var sr = FolderDisk.SyncBack(appVol, appBox);
            Check(sr.Error is null, "it syncs back" + (sr.Error is not null ? " -- " + sr.Error : ""));
            Check(File.Exists(Path.Combine(appBox, "Expander.sea")),
                  "it went back to the name it came in under");
            Check(!File.Exists(Path.Combine(appBox, "Expander.sea.bin")),
                  "no second copy was minted beside it");
            byte[]? rebuilt = FolderDisk.Build(appBox, out _);
            Check(rebuilt is not null &&
                  FolderDiskProbe.List(rebuilt).FindAll(t => t.Name.StartsWith("Expander")).Count == 1,
                  "a second build still sees exactly one of it");
        }

        Console.WriteLine(failures == 0 ? "\nALL PASS" : $"\n{failures} FAILED");
        return failures == 0 ? 0 : 1;
    }
}
