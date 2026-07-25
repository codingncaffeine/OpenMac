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
        Log.Line($"create hard disk: \"{volumeName}\" {sizeMB} MB -> {path}");

        byte[] formatted = NativeFormatter.Format(sizeBytes, volumeName);
        File.WriteAllBytes(path, formatted);

        long onDisk = new FileInfo(path).Length;
        Log.Line($"create hard disk: wrote {onDisk:N0} bytes");
        if (onDisk != sizeBytes)
            throw new IOException(
                $"The image should be {sizeBytes:N0} bytes but {onDisk:N0} reached the disk. " +
                "The drive may be full.");
    }
}

/// <summary>
/// The native HFS formatter (omac_format_hfs -> openmac::hfs::formatVolume).
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
                int rc = Native.omac_format_hfs((uint)probe.Length, "Probe", probe);
                _available = rc == 0;
                if (rc != 0) Log.Line($"hfs formatter: probe returned {rc}");
            }
            catch (Exception ex)
            {
                // Almost always the DLL failing to load. Saying so beats reporting
                // "the formatter is missing" for every possible cause.
                _available = false;
                Log.Line($"hfs formatter: unavailable -- {ex.GetType().Name}: {ex.Message}");
            }
            return _available.Value;
        }
    }

    /// <summary>
    /// Format a volume of <paramref name="sizeBytes"/> bytes. Throws with the
    /// actual reason rather than returning null: a blank image would not mount,
    /// so the caller must not write one, and "it failed" is not a diagnosis.
    /// </summary>
    public static byte[] Format(long sizeBytes, string volumeName)
    {
        if (sizeBytes <= 0 || sizeBytes > uint.MaxValue)
            throw new ArgumentOutOfRangeException(nameof(sizeBytes),
                $"{sizeBytes:N0} bytes is not a size the formatter can produce.");

        byte[] buf;
        try { buf = new byte[sizeBytes]; }
        catch (OutOfMemoryException)
        {
            throw new InvalidOperationException(
                $"Not enough memory to build a {sizeBytes / (1024 * 1024)} MB image. " +
                "Try a smaller size.");
        }

        int rc;
        try { rc = Native.omac_format_hfs((uint)sizeBytes, volumeName, buf); }
        catch (Exception ex)
        {
            Log.Line($"hfs formatter: {ex.GetType().Name}: {ex.Message}");
            throw new InvalidOperationException(
                "Could not reach the native HFS formatter (omac_format_hfs in " +
                $"openmac_c.dll): {ex.Message}", ex);
        }

        if (rc != 0)
            throw new InvalidOperationException(
                $"The native HFS formatter rejected a {sizeBytes / (1024 * 1024)} MB " +
                $"volume named \"{volumeName}\" (code {rc}).");

        return buf;
    }
}
