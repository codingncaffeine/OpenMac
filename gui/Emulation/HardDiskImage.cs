using System.IO;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// Creates hard-disk image files formatted as empty, mountable HFS volumes via
/// the native formatter (omac_format_hfs -> openmac::hfs::formatVolume). The Mac
/// Classic can't format a .Sony-attached hard disk itself, so the image must be a
/// real HFS volume up front or it won't mount.
/// </summary>
public static class HardDiskImage
{
    /// <summary>True once <see cref="CreateBlank"/> produces a real HFS volume.</summary>
    public static bool ProducesRealHfs => NativeFormatter.IsAvailable;

    public static readonly int[] CommonSizesMB = { 20, 40, 80, 120, 240, 500 };

    /// <summary>
    /// Write a hard-disk image of <paramref name="sizeMB"/> megabytes to
    /// <paramref name="path"/>, formatted as an empty HFS volume named
    /// <paramref name="volumeName"/>. Throws if the native formatter is missing
    /// (a blank image would not mount, so none is written).
    /// </summary>
    public static void CreateBlank(string path, int sizeMB, string volumeName)
    {
        long sizeBytes = (long)sizeMB * 1024 * 1024;

        byte[]? formatted = NativeFormatter.TryFormat(sizeBytes, volumeName);
        if (formatted is null)
            throw new InvalidOperationException(
                "Could not format the disk image: the native HFS formatter " +
                "(omac_format_hfs in openmac_c.dll) failed or is missing. A blank " +
                "image would not mount, so no file was written.");

        File.WriteAllBytes(path, formatted);
    }
}

/// <summary>
/// Thin seam over the (not-yet-present) native HFS formatter. When the C ABI
/// export exists, implement <see cref="TryFormat"/> as a P/Invoke and flip
/// <see cref="IsAvailable"/>.
/// </summary>
internal static class NativeFormatter
{
    private static bool? _available;

    public static bool IsAvailable
    {
        get
        {
            if (_available.HasValue) return _available.Value;
            try
            {
                var probe = new byte[1024 * 1024];   // 1 MB is a valid HFS size
                _available = Native.omac_format_hfs((uint)probe.Length, "Probe", probe) == 0;
            }
            catch { _available = false; }
            return _available.Value;
        }
    }

    public static byte[]? TryFormat(long sizeBytes, string volumeName)
    {
        if (sizeBytes <= 0 || sizeBytes > uint.MaxValue) return null;
        try
        {
            var buf = new byte[sizeBytes];
            return Native.omac_format_hfs((uint)sizeBytes, volumeName, buf) == 0 ? buf : null;
        }
        catch { return null; }
    }
}
