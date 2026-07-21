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
    public string? LastHardDisk { get; set; }
    public List<string> RecentRoms { get; set; } = new();

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
