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

    // Returns 1 if the drive took the disk, 0 if the file is not floppy media
    // (an NDIF image, an application, an archive...); the drive is then left
    // untouched and omac_floppy_medium says why.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_insert_floppy(IntPtr h, byte[] img, nuint len, int readOnly);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_eject_floppy(IntPtr h);

    // The last medium description for a drive (0 = internal, 1 = external):
    // geometry and container for an accepted disk, the reason for a refusal.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_floppy_medium(IntPtr h, int drive,
                                                  [Out] byte[]? outBuf, nuint cap);

    // The external drive port. Attaching a mechanism makes the ROM register a
    // second floppy drive; a disk put in after the machine has started mounts
    // like any other.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_set_external_drive(IntPtr h, int attached);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_insert_floppy2(IntPtr h, byte[] img, nuint len, int readOnly);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_eject_floppy2(IntPtr h);

    // Is there a disk in that drive right now? 0 = internal, 1 = external. The
    // guest ejects disks on its own, so this has to be asked rather than assumed.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_floppy_present(IntPtr h, int drive);

    // Copy a medium back out so a session's writes can be saved to the file the
    // disk came from. Pass a null buffer to ask how large it is.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_floppy_data(IntPtr h, byte[]? outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_floppy2_data(IntPtr h, byte[]? outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_insert_harddisk(IntPtr h, byte[] img, nuint len, int readOnly);

    // Second SCSI disk (the folder disk's seat).
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_insert_harddisk2(IntPtr h, byte[] img, nuint len, int readOnly);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_detach_harddisk2(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_harddisk2_data(IntPtr h, byte[]? outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_harddisk2_booted(IntPtr h);

    // CD-ROM: the drive is a bus device (attach), a disc is media (insert).
    // Discs are read-only; the guest can eject them itself, so presence is
    // polled like the floppies.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_cd_attach(IntPtr h, int attached, int scsiId);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_cd_attached(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_cd_insert(IntPtr h, byte[] img, nuint len);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_cd_eject(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_cd_present(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_cd_medium(IntPtr h, [Out] byte[]? outBuf, nuint cap);

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

    // Networking: raw Ethernet frames to/from the DaynaPORT target.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_net_attach(IntPtr h, int attached, int scsiId);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_net_inject(IntPtr h, byte[] frame, nuint len);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_net_drain(IntPtr h, byte[] outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr omac_version();

    // ---- Quadra 650 (separate opaque handle; omac_q_* in capi.h) ----
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr omac_q_create(byte[] rom, nuint romLen, uint ramMb);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_destroy(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_reset(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_run_frame(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_q_screen_w(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_q_screen_h(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_render(IntPtr h, byte[] argb);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_q_drain_audio(IntPtr h, byte[] outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_mouse(IntPtr h, int dx, int dy, int button);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_key(IntPtr h, int adbCode, int down);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_insert_harddisk(IntPtr h, byte[] img, nuint len, int readOnly);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_q_harddisk_data(IntPtr h, byte[]? outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_unmount_harddisk2(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_flush_volumes(IntPtr h);

    // The second SCSI disk (ID 1, drive 5): the drop box / folder disk seat.
    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_insert_harddisk2(IntPtr h, byte[] img, nuint len, int readOnly);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_detach_harddisk2(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_q_harddisk2_data(IntPtr h, byte[]? outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_q_harddisk2_booted(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_q_unmount_harddisk2(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_q_diagnostics(IntPtr h, byte[]? outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_poll_log(IntPtr h, byte[] outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_q_shutdown_volumes(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern IntPtr omac_q_display_name(int index, out int w, out int h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_q_set_display(IntPtr h, [MarshalAs(UnmanagedType.LPStr)] string name);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_q_floppy_writeback(IntPtr h, byte[]? outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_q_insert_floppy(IntPtr h, byte[] img, nuint len, int readOnly);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_eject_floppy(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_q_floppy_present(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_q_floppy_data(IntPtr h, byte[]? outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_pram_save(IntPtr h, byte[]? outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_pram_load(IntPtr h, byte[] data, nuint len, uint addSeconds);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern nuint omac_q_pram_save(IntPtr h, byte[]? outBuf, nuint cap);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_q_pram_load(IntPtr h, byte[] data, nuint len, uint addSeconds);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_attach_cd(IntPtr h, int attached, int busId);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_q_cd_attached(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_q_insert_cd(IntPtr h, byte[] img, nuint len);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern void omac_q_eject_cd(IntPtr h);

    [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
    public static extern int omac_q_cd_present(IntPtr h);
}
