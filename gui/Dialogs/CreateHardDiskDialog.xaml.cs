using System.IO;
using System.Windows;
using Microsoft.Win32;
using OpenMac.Gui.Emulation;

namespace OpenMac.Gui.Dialogs;

public partial class CreateHardDiskDialog : Window
{
    /// <summary>Path of the image created when the dialog returns true.</summary>
    public string? CreatedPath { get; private set; }

    public CreateHardDiskDialog()
    {
        InitializeComponent();
        WindowTheming.ApplyDarkTitleBar(this);

        foreach (int mb in HardDiskImage.CommonSizesMB)
            SizeBox.Items.Add($"{mb} MB");
        SizeBox.SelectedIndex = 0;

        FormatNote.Text = HardDiskImage.ProducesRealHfs
            ? "Creates an empty, ready-to-use HFS volume."
            : "Creates the image now; it will mount as an uninitialized disk until "
              + "the built-in HFS formatter is enabled, then you can re-create it ready to use.";
    }

    private int SelectedSizeMB => HardDiskImage.CommonSizesMB[SizeBox.SelectedIndex];

    private void Browse_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new SaveFileDialog
        {
            Title = "Save hard-disk image",
            Filter = "Disk image (*.img)|*.img|All files (*.*)|*.*",
            FileName = SafeFileName(NameBox.Text) + ".img",
            AddExtension = true,
            DefaultExt = ".img",
        };
        if (dlg.ShowDialog(this) == true)
            PathBox.Text = dlg.FileName;
        Log.Line($"create hard disk: save-as -> \"{PathBox.Text}\"");
    }

    private void Create_Click(object sender, RoutedEventArgs e)
    {
        string name = NameBox.Text.Trim();
        if (name.Length == 0) { Warn("Please enter a volume name."); return; }

        // The path box is read-only, so the only way to fill it is Browse. If the
        // user goes straight to Create, offer the file dialog once -- and if they
        // leave it without choosing, say so. Returning in silence here is
        // indistinguishable from the button doing nothing at all.
        if (string.IsNullOrWhiteSpace(PathBox.Text))
        {
            Browse_Click(sender, e);
            if (string.IsNullOrWhiteSpace(PathBox.Text))
            {
                Log.Line("create hard disk: cancelled at the file dialog, nothing written");
                Warn("Choose where to save the image first, with the Browse button.");
                return;
            }
        }

        try
        {
            CreateBtn.IsEnabled = false;
            HardDiskImage.CreateBlank(PathBox.Text, SelectedSizeMB, name);
            CreatedPath = PathBox.Text;
            DialogResult = true;
        }
        catch (Exception ex)
        {
            CreateBtn.IsEnabled = true;
            Log.Line($"create hard disk FAILED: {ex.GetType().Name}: {ex.Message}");
            Warn("Could not create the image:\n\n" + ex.Message);
        }
    }

    private void Warn(string msg) =>
        MessageBox.Show(this, msg, "Create Hard Disk", MessageBoxButton.OK, MessageBoxImage.Warning);

    private static string SafeFileName(string s)
    {
        string name = string.Concat(s.Where(c => !Path.GetInvalidFileNameChars().Contains(c))).Trim();
        return name.Length == 0 ? "OpenMac HD" : name;
    }
}
