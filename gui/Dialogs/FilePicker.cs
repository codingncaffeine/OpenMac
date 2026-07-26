using System.IO;
using System.Windows;
using Microsoft.Win32;

namespace OpenMac.Gui.Dialogs;

/// <summary>
/// File dialogs that come back to where you were last time you chose that kind
/// of file. ROMs, floppy images and hard-disk images live in different folders
/// and are picked weeks apart, so one shared "last folder" is worse than none —
/// it sends you to whichever you touched most recently.
///
/// Two mechanisms, because either alone leaves a gap: <see cref="Settings"/>
/// holds the folder we last used, which works on a fresh profile and can be
/// inspected; and each purpose gets its own <c>ClientGuid</c>, which gives the
/// Windows dialog a separate recent-places list per purpose instead of the one
/// it otherwise shares across the whole application.
/// </summary>
internal static class FilePicker
{
    // Purposes are by kind of file, not by menu item: both floppy drives draw
    // from the same image library, and attaching a hard disk starts where the
    // last one was created.
    public const string Rom = "rom";
    public const string Floppy = "floppy";
    public const string HardDisk = "harddisk";
    public const string Cd = "cd";

    private static Guid GuidFor(string purpose) => purpose switch
    {
        Rom      => new Guid("7f3a1c94-0f6e-4a2d-9c11-4b6b0a5e1d01"),
        Floppy   => new Guid("7f3a1c94-0f6e-4a2d-9c11-4b6b0a5e1d02"),
        HardDisk => new Guid("7f3a1c94-0f6e-4a2d-9c11-4b6b0a5e1d03"),
        Cd       => new Guid("7f3a1c94-0f6e-4a2d-9c11-4b6b0a5e1d04"),
        _        => new Guid("7f3a1c94-0f6e-4a2d-9c11-4b6b0a5e1d00"),
    };

    /// <summary>
    /// Ask for an existing file. Returns null if the user backed out; otherwise
    /// the chosen path, with its folder remembered for this purpose.
    /// </summary>
    public static string? Open(Window owner, Settings settings, string purpose,
                              string title, string filter, string? preselect = null)
    {
        var dlg = new OpenFileDialog
        {
            Title = title,
            Filter = filter,
            ClientGuid = GuidFor(purpose),
        };
        Aim(dlg, settings, purpose, preselect);
        if (dlg.ShowDialog(owner) != true) return null;
        settings.RememberFolder(purpose, dlg.FileName);
        return dlg.FileName;
    }

    /// <summary>
    /// Ask where to write a new file. Returns null if the user backed out.
    /// </summary>
    public static string? Save(Window owner, Settings settings, string purpose,
                               string title, string filter, string suggestedName,
                               string defaultExt, string? fallback = null)
    {
        var dlg = new SaveFileDialog
        {
            Title = title,
            Filter = filter,
            AddExtension = true,
            DefaultExt = defaultExt,
            ClientGuid = GuidFor(purpose),
        };
        Aim(dlg, settings, purpose, null, fallback);
        dlg.FileName = suggestedName;      // after Aim, which must not name the file here
        if (dlg.ShowDialog(owner) != true) return null;
        settings.RememberFolder(purpose, dlg.FileName);
        return dlg.FileName;
    }

    // Point the dialog at the file used last if it is still there, otherwise at
    // the folder. Naming the file is what actually opens that folder with the
    // entry already selected; InitialDirectory alone only suggests a starting
    // point, and Windows may prefer its own recent list.
    //
    // `fallback` covers the first run after this was added, and any profile with
    // no folder history: an install that has been opening the same ROM for weeks
    // already knows where that ROM lives, so there is no reason to start it at
    // Documents.
    private static void Aim(FileDialog dlg, Settings settings, string purpose,
                            string? preselect, string? fallback = null)
    {
        if (!string.IsNullOrEmpty(preselect) && File.Exists(preselect))
        {
            dlg.FileName = preselect;
            dlg.InitialDirectory = Path.GetDirectoryName(preselect)!;
            return;
        }
        if (settings.FolderFor(purpose) is { } dir) { dlg.InitialDirectory = dir; return; }
        foreach (string? candidate in new[] { preselect, fallback })
        {
            if (string.IsNullOrEmpty(candidate)) continue;
            string? parent = Path.GetDirectoryName(candidate);
            if (!string.IsNullOrEmpty(parent) && Directory.Exists(parent))
            {
                dlg.InitialDirectory = parent;
                return;
            }
        }
    }
}
