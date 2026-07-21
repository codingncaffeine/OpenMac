using System.IO;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// Creates blank hard-disk image files.
///
/// TODO (wire-up): the native core is growing a real HFS formatter
/// (<c>openmac::hfs::formatVolume</c>). Once that is exposed through the C ABI,
/// <see cref="CreateBlank"/> should call it so the produced image is a genuine,
/// mountable HFS volume. Until then it writes a zeroed image — the machine sees
/// an uninitialized disk (which is still a valid, useful placeholder for the UI).
/// </summary>
public static class HardDiskImage
{
    /// <summary>True once <see cref="CreateBlank"/> produces a real HFS volume.</summary>
    public static bool ProducesRealHfs => NativeFormatter.IsAvailable;

    public static readonly int[] CommonSizesMB = { 20, 40, 80, 120, 240, 500 };

    /// <summary>
    /// Write a blank hard-disk image of <paramref name="sizeMB"/> megabytes to
    /// <paramref name="path"/>, formatted (when the native formatter is present)
    /// as an empty HFS volume named <paramref name="volumeName"/>.
    /// </summary>
    public static void CreateBlank(string path, int sizeMB, string volumeName)
    {
        long sizeBytes = (long)sizeMB * 1024 * 1024;

        byte[]? formatted = NativeFormatter.TryFormat(sizeBytes, volumeName);
        if (formatted is not null)
        {
            File.WriteAllBytes(path, formatted);
            return;
        }

        // Placeholder: allocate a zeroed image on disk without holding it all in
        // memory. The disk mounts as "uninitialized" until the formatter is wired.
        using var fs = new FileStream(path, FileMode.Create, FileAccess.Write);
        fs.SetLength(sizeBytes);
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
