using System.IO;
using System.Text.Json;

namespace OpenMac.Gui;

/// <summary>Persisted app settings (JSON under %APPDATA%\OpenMac).</summary>
public sealed class Settings
{
    public int RamMB { get; set; } = 4;
    public bool BootRomDisk { get; set; }

    /// <summary>Which machine the GUI runs: "classic", "iifx", "quadra650", "quadra950" or "se30". Each model
    /// remembers its own ROM, floppy and hard disk — they are not interchangeable.</summary>
    public string Model { get; set; } = "classic";
    public string? LastRomIifx { get; set; }
    public string? LastFloppyIifx { get; set; }
    public string? LastHardDiskIifx { get; set; }
    public string? VideoRomIifx { get; set; }
    public int RamMBIifx { get; set; } = 8;
    public string? LastRomQuadra { get; set; }
    public string? LastFloppyQuadra { get; set; }
    public string? LastHardDiskQuadra { get; set; }
    public int RamMBQuadra { get; set; } = 8;

    /// <summary>Which monitor is plugged into the Quadra's video port. The
    /// ROM senses it at startup and the display's own resolution follows, so
    /// this is how a resolution is chosen. Empty means the default 13-inch.</summary>
    public string? MonitorQuadra { get; set; }

    public string? LastRomQuadra950 { get; set; }
    public string? LastFloppyQuadra950 { get; set; }
    public string? LastHardDiskQuadra950 { get; set; }
    public int RamMBQuadra950 { get; set; } = 8;
    /// <summary>Video RAM fitted in the Quadra 950: 1 MB as shipped, or 2 MB
    /// with the sockets filled (which is what the deeper color modes need).</summary>
    public int VramMBQuadra950 { get; set; } = 2;
    public string? MonitorQuadra950 { get; set; }

    public string? LastRomSe30 { get; set; }
    public string? LastFloppySe30 { get; set; }
    public string? LastHardDiskSe30 { get; set; }
    public int RamMBSe30 { get; set; } = 8;
    /// <summary>The SE/30's 8 KB video ROM file (part 341-0650): its built-in
    /// screen is a card in slot $E with a declaration ROM of its own.</summary>
    public string? VideoRomSe30 { get; set; }

    [System.Text.Json.Serialization.JsonIgnore]
    public bool IsSe30 => Model == "se30";
    /// <summary>The Macintosh Classic: the model that is none of the others.</summary>
    [System.Text.Json.Serialization.JsonIgnore]
    public bool IsClassic => !IsQuadra && !IsIifx && !IsSe30;
    /// <summary>Either Quadra: the two share one backend.</summary>
    [System.Text.Json.Serialization.JsonIgnore]
    public bool IsQuadra => Model == "quadra650" || Model == "quadra950";
    [System.Text.Json.Serialization.JsonIgnore]
    public bool IsQuadra950 => Model == "quadra950";
    /// <summary>The monitor on the current Quadra's video port.</summary>
    [System.Text.Json.Serialization.JsonIgnore]
    public string? ModelMonitor
    {
        get => IsQuadra950 ? MonitorQuadra950 : MonitorQuadra;
        set
        {
            if (IsQuadra950) MonitorQuadra950 = value;
            else MonitorQuadra = value;
        }
    }
    [System.Text.Json.Serialization.JsonIgnore]
    public bool IsIifx => Model == "iifx";
    [System.Text.Json.Serialization.JsonIgnore]
    public string? ModelLastRom
    {
        get => IsIifx ? LastRomIifx : IsSe30 ? LastRomSe30 : IsQuadra950 ? LastRomQuadra950 : IsQuadra ? LastRomQuadra : LastRom;
        set
        {
            if (IsIifx) LastRomIifx = value;
            else if (IsSe30) LastRomSe30 = value;
            else if (IsQuadra950) LastRomQuadra950 = value;
            else if (IsQuadra) LastRomQuadra = value;
            else LastRom = value;
        }
    }
    [System.Text.Json.Serialization.JsonIgnore]
    public string? ModelLastHardDisk
    {
        get => IsIifx ? LastHardDiskIifx : IsSe30 ? LastHardDiskSe30 : IsQuadra950 ? LastHardDiskQuadra950 : IsQuadra ? LastHardDiskQuadra : LastHardDisk;
        set
        {
            if (IsIifx) LastHardDiskIifx = value;
            else if (IsSe30) LastHardDiskSe30 = value;
            else if (IsQuadra950) LastHardDiskQuadra950 = value;
            else if (IsQuadra) LastHardDiskQuadra = value;
            else LastHardDisk = value;
        }
    }
    [System.Text.Json.Serialization.JsonIgnore]
    public string? ModelLastFloppy
    {
        get => IsIifx ? LastFloppyIifx : IsSe30 ? LastFloppySe30 : IsQuadra950 ? LastFloppyQuadra950 : IsQuadra ? LastFloppyQuadra : LastFloppy;
        set
        {
            if (IsIifx) LastFloppyIifx = value;
            else if (IsSe30) LastFloppySe30 = value;
            else if (IsQuadra950) LastFloppyQuadra950 = value;
            else if (IsQuadra) LastFloppyQuadra = value;
            else LastFloppy = value;
        }
    }
    [System.Text.Json.Serialization.JsonIgnore]
    public int ModelRamMB => IsIifx ? RamMBIifx : IsSe30 ? RamMBSe30 : IsQuadra950 ? RamMBQuadra950 : IsQuadra ? RamMBQuadra : RamMB;

    public int Scale { get; set; } = 2;              // 0 = fit, else fixed multiplier
    public string? LastRom { get; set; }
    public string? LastFloppy { get; set; }
    /// <summary>Disk left in the external drive, and whether that drive is connected.</summary>
    public string? LastExternalFloppy { get; set; }
    public bool ExternalDrive { get; set; }
    public string? LastHardDisk { get; set; }
    /// <summary>The CD-ROM drive on the SCSI bus, and the disc left in it.</summary>
    public bool CdRomAttached { get; set; }
    public string? LastCd { get; set; }
    /// <summary>Host folder served to the Mac as a disk (rebuilt each start).</summary>
    public string? LastFolderDisk { get; set; }
    /// <summary>
    /// The drop box: keep <see cref="LastFolderDisk"/> in front of the Mac for
    /// the whole session, so a file dropped on the window arrives in the Finder
    /// instead of making a disk of its own. Off until asked for — it puts a
    /// second disk on the SCSI bus, which is a change to the machine, not a
    /// display preference.
    /// </summary>
    public bool DropBox { get; set; }
    /// <summary>DaynaPORT adapter + user-mode NAT.</summary>
    public bool Networking { get; set; }
    /// <summary>
    /// The write-protect tab, as a physical one: off unless asked for. A Mac
    /// writes to its floppies constantly and they stay perfectly usable, so
    /// locking them by default would be papering over a write path that has to
    /// be correct anyway. This is here for the same reason the tab is on a real
    /// disk -- to protect a master on purpose -- not as a safety net.
    /// </summary>
    public bool WriteProtectFloppies { get; set; }
    /// <summary>
    /// Hold the Shift key down inside the Mac through the start of every boot,
    /// so System 6/7 starts with extensions off. Exists because holding the
    /// real key for that long trips Windows' sticky/filter-keys accessibility
    /// traps on the host. Stays on until unchecked, like the physical habit.
    /// </summary>
    public bool BootExtensionsOff { get; set; }
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
            {
                Settings settings =
                    JsonSerializer.Deserialize<Settings>(File.ReadAllText(FilePath)) ?? new Settings();
                int[] legalIifxRam = { 4, 8, 16, 20, 32, 64, 68, 80, 128 };
                if (Array.IndexOf(legalIifxRam, settings.RamMBIifx) < 0)
                    settings.RamMBIifx = 8;
                return settings;
            }
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
        ModelLastRom = path;
    }
}
