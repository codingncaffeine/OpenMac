using System.Runtime.InteropServices;

namespace OpenMac.Gui.Emulation;

/// <summary>P/Invoke surface over openmac_c.dll (the core's C ABI).</summary>
internal static class Native
{
    private const string Dll = "openmac_c";

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate void LogCallback(IntPtr user, IntPtr line);

    // Debug-enable flags (mirror OMAC_DBG_* in capi.h).
    public const uint DbgTraps = 0x01, DbgExcept = 0x02, DbgIrq = 0x04, DbgAdb = 0x08;

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr omac_create(byte[] rom, nuint romLen, uint ramMb);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_destroy(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_reset(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_set_force_rom_disk(IntPtr h, int on);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_run_frame(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_render(IntPtr h, byte[] argb);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_drain_audio(IntPtr h, byte[] outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_insert_floppy(IntPtr h, byte[] img, nuint len, int readOnly);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_eject_floppy(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_insert_harddisk(IntPtr h, byte[] img, nuint len, int readOnly);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_harddisk_data(IntPtr h, byte[]? outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_format_hfs(uint sizeBytes,
        [MarshalAs(UnmanagedType.LPStr)] string name, byte[] outBuf);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_mouse(IntPtr h, int dx, int dy, int button);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_key(IntPtr h, int adbCode, int down);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_set_log(IntPtr h, LogCallback fn, IntPtr user);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_debug_enable(IntPtr h, uint flags);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_poll_log(IntPtr h, byte[] outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr omac_version();
}
