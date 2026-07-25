using System.IO;
using System.Text.Json;

namespace OpenMac.Gui;

/// <summary>Persisted app settings (JSON under %APPDATA%\OpenMac).</summary>
public sealed class Settings
{
    public int RamMB { get; set; } = 4;
    public bool BootRomDisk { get; set; }
    public int Scale { get; set; } = 2;              // 0 = fit, else fixed multiplier
    public string? LastRom { get; set; }
    public string? LastFloppy { get; set; }
    /// <summary>Disk left in the external drive, and whether that drive is connected.</summary>
    public string? LastExternalFloppy { get; set; }
    public bool ExternalDrive { get; set; }
    public string? LastHardDisk { get; set; }
    public List<string> RecentRoms { get; set; } = new();

    /// <summary>
    /// Where the file dialogs last landed, one entry per kind of file. ROMs,
    /// disk images and hard-disk images live in different places and are chosen
    /// weeks apart, so a single "last folder" sends you back to whichever you
    /// touched most recently rather than the one you are actually looking for.
    /// </summary>
    public Dictionary<string, string> LastFolders { get; set; } = new();

    /// <summary>The remembered folder for <paramref name="purpose"/>, if it still exists.</summary>
    public string? FolderFor(string purpose) =>
        LastFolders.TryGetValue(purpose, out string? dir) && Directory.Exists(dir) ? dir : null;

    /// <summary>Remember the folder <paramref name="filePath"/> came from.</summary>
    public void RememberFolder(string purpose, string filePath)
    {
        string? dir = Path.GetDirectoryName(filePath);
        if (string.IsNullOrEmpty(dir)) return;
        LastFolders[purpose] = dir;
        Save();
    }

    private static string Dir =>
        Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "OpenMac");
    private static string FilePath => Path.Combine(Dir, "settings.json");

    private static readonly JsonSerializerOptions JsonOpts = new() { WriteIndented = true };

    public static Settings Load()
    {
        try
        {
            if (File.Exists(FilePath))
                return JsonSerializer.Deserialize<Settings>(File.ReadAllText(FilePath)) ?? new Settings();
        }
        catch { /* fall through to defaults */ }
        return new Settings();
    }

    public void Save()
    {
        try
        {
            Directory.CreateDirectory(Dir);
            File.WriteAllText(FilePath, JsonSerializer.Serialize(this, JsonOpts));
        }
        catch { /* non-fatal */ }
    }

    public void PushRecentRom(string path)
    {
        RecentRoms.RemoveAll(p => string.Equals(p, path, StringComparison.OrdinalIgnoreCase));
        RecentRoms.Insert(0, path);
        if (RecentRoms.Count > 8) RecentRoms.RemoveRange(8, RecentRoms.Count - 8);
        LastRom = path;
    }
}
