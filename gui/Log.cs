using System.IO;

namespace OpenMac.Gui;

/// <summary>
/// Simple append-to-file logger. Lives next to settings under %APPDATA%\OpenMac
/// so startups (and, once the native core is linked, emulation events and
/// crashes) are diagnosable after the fact.
/// </summary>
internal static class Log
{
    private static readonly object Gate = new();
    private static string? _path;

    public static string Path => _path ??= System.IO.Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "OpenMac", "openmac.log");

    public static void Init()
    {
        try
        {
            Directory.CreateDirectory(System.IO.Path.GetDirectoryName(Path)!);
            // Keep history across launches, but don't let it grow without bound.
            if (File.Exists(Path) && new FileInfo(Path).Length > 512 * 1024)
                File.WriteAllText(Path, "");
            Line("======== session start ========");
        }
        catch { /* logging must never crash the app */ }
    }

    public static void Line(string message)
    {
        try
        {
            lock (Gate)
                File.AppendAllText(Path, $"{DateTime.Now:yyyy-MM-dd HH:mm:ss.fff}  {message}{Environment.NewLine}");
        }
        catch { /* ignore */ }
    }
}
