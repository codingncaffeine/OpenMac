using System;
using System.IO;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// The battery behind the clock/PRAM chip.
/// </summary>
/// <remarks>
/// Parameter RAM is the one part of a Macintosh that survives being switched
/// off, and everything a user sets that is not a file lives in it: 32-bit
/// addressing, the startup disk, the alert volume, the date. Held only in the
/// emulated chip it came back blank every launch, the ROM reset it to defaults,
/// and the machine forgot every choice — which is how a machine with 136 MB
/// fitted kept booting in 24-bit mode with 8 MB usable.
///
/// The blob is opaque here on purpose: the core hands over 256 bytes of XPRAM
/// exactly as the guest wrote them plus the clock, and nothing on this side
/// interprets a byte of it. Whatever a control panel stores is what comes back.
///
/// One file per machine model, because the machines have different PRAM
/// contents and writing one over the other would be worse than forgetting.
/// </remarks>
public static class PramStore
{
    private static string Dir =>
        Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
                     "OpenMac");

    private static string PathFor(string model) => Path.Combine(Dir, $"pram-{model}.bin");
    private static string StampFor(string model) => Path.Combine(Dir, $"pram-{model}.when");

    /// <summary>
    /// The saved contents, or null if there are none. <paramref name="addSeconds"/>
    /// comes back as the wall time the machine was switched off for, so the
    /// guest's clock can carry on the way a battery-backed one would instead of
    /// resuming where it stopped.
    /// </summary>
    public static byte[]? Load(string model, out uint addSeconds)
    {
        addSeconds = 0;
        try
        {
            string path = PathFor(model);
            if (!File.Exists(path)) return null;
            byte[] blob = File.ReadAllBytes(path);
            // A clock that ran backwards would be worse than one that stood
            // still, so an unreadable or future stamp simply adds nothing.
            if (File.Exists(StampFor(model)) &&
                long.TryParse(File.ReadAllText(StampFor(model)).Trim(), out long saved))
            {
                long now = DateTimeOffset.UtcNow.ToUnixTimeSeconds();
                long gap = now - saved;
                if (gap > 0 && gap < int.MaxValue) addSeconds = (uint)gap;
            }
            return blob;
        }
        catch (Exception ex)
        {
            Log.Line($"{model} PRAM load failed: {ex.Message}");
            return null;
        }
    }

    /// <summary>
    /// Write the contents back, with the moment they were written. A false
    /// result is logged so a persistence failure is never mistaken for a guest
    /// that simply did not change parameter RAM.
    /// </summary>
    public static bool Save(string model, byte[] blob)
    {
        if (blob.Length == 0)
        {
            Log.Line($"{model} PRAM save rejected an empty blob");
            return false;
        }
        try
        {
            Directory.CreateDirectory(Dir);
            File.WriteAllBytes(PathFor(model), blob);
            File.WriteAllText(StampFor(model),
                              DateTimeOffset.UtcNow.ToUnixTimeSeconds().ToString());
            return true;
        }
        catch (Exception ex)
        {
            Log.Line($"{model} PRAM save failed: {ex.Message}");
            return false;
        }
    }
}
