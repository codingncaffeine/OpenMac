using System.IO;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// A file's size as its reader will see it: a symbolic link is measured by the
/// file it leads to, not by the link (which FileInfo.Length reports on Linux).
/// </summary>
internal static class FileSize
{
    public static long Of(string path)
    {
        var info = new FileInfo(path);
        if (info.LinkTarget is null) return info.Length;
        return (File.ResolveLinkTarget(path, returnFinalTarget: true) as FileInfo ?? info).Length;
    }
}
