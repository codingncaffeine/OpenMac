using System.IO;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using OpenMac.Gui.Dialogs;
using OpenMac.Gui.Emulation;

namespace OpenMac.Gui;

public partial class MainWindow : Window
{
    // Both drives take the same images, so they offer the same filter and share
    // the remembered folder.
    // .bin is here because much archived software travels as MacBinary
    // (".img.bin"); the core strips that wrapper on insertion.
    private const string DiskImageFilter =
        "Disk image (*.img;*.image;*.dsk;*.dc42;*.bin)|*.img;*.image;*.dsk;*.dc42;*.bin|"
        + "All files (*.*)|*.*";

    private readonly Settings _settings;
    private IEmulator _emulator;
    private WriteableBitmap _bitmap = null!;
    private byte[] _bgra = null!;

    // The backend runs emulation and audio on its own thread, so playback is never
    // stalled by this UI thread. The window only displays the latest frame:
    // CompositionTarget.Rendering fires once per display refresh and blits whatever
    // new frame the emulator has produced since the previous refresh.
    private EventHandler? _renderHandler;

    private bool _mouseLocked;
    private KeyboardHook? _keyHook;        // swallows host combos while input is captured
    private bool _ignoreUpAfterLock;
    private int _lockCx, _lockCy;          // window-center reference, physical screen px
    private string _baseTitle = "OpenMac";
    private bool _fullscreen;
    private WindowStyle _savedStyle;
    private WindowState _savedState;

    public MainWindow()
    {
        InitializeComponent();
        WindowTheming.ApplyDarkTitleBar(this);

        _settings = Settings.Load();
        _emulator = CreateBackend(_settings);
        _emulator.WriteProtectFloppies = _settings.WriteProtectFloppies;
        RebuildScreen();

        StatusBackend.Text = _emulator.IsRealCore ? "core: native" : "core: stub (not linked)";
        Log.Line($"GUI ready — {_emulator.BackendName} backend, screen {_emulator.ScreenWidth}x{_emulator.ScreenHeight}");

        timeBeginPeriod(1);                       // sharpen OS timer resolution for the emu thread's pacing
        _renderHandler = (_, _) => Tick();
        CompositionTarget.Rendering += _renderHandler;

        WireInput();
        // The captured-input keyboard hook lives for the window's lifetime and
        // does nothing until capture switches it on. It reads _emulator through
        // this, so a backend swap never leaves it pointing at a dead machine.
        _keyHook = new KeyboardHook((code, down) => _emulator.KeyEvent(code, down), ToggleFullscreen);
        BuildRecentMenu();
        UpdateUi();

        Loaded += (_, _) =>
        {
            ApplyScale();
            if (!string.IsNullOrEmpty(_settings.ModelLastRom) && File.Exists(_settings.ModelLastRom))
                LoadRom(_settings.ModelLastRom!);
            // Files handed on the command line ride the same router as a drop,
            // so "openmac.exe game.img" and double-click associations both work.
            string[] args = Environment.GetCommandLineArgs();
            for (int i = 1; i < args.Length; i++)
                if (File.Exists(args[i])) RouteMedia(args[i]);
            if (args.Length > 1) UpdateUi();
        };
        Closing += (_, _) =>
        {
            if (_renderHandler != null) CompositionTarget.Rendering -= _renderHandler;
            timeEndPeriod(1);
            _keyHook?.Dispose();
            _emulator.Dispose();   // stop the emulation thread and persist the hard disk
            _settings.Save();
        };
    }

    /// <summary>Real core if openmac_c.dll loads; otherwise the stub preview.
    /// The settings' model picks which machine the native backend drives.</summary>
    private static IEmulator CreateBackend(Settings settings)
    {
        try
        {
            Native.omac_version();   // probes the native DLL; throws if it's missing
            if (settings.IsQuadra)
            {
                Log.Line("backend: native core (openmac_c.dll), Quadra 650");
                return new QuadraEmulator();
            }
            Log.Line("backend: native core (openmac_c.dll), Macintosh Classic");
            return new NativeEmulator();
        }
        catch (Exception ex)
        {
            Log.Line("backend: native core unavailable, using stub — " + ex.Message);
            return new StubEmulator();
        }
    }

    /// <summary>Size the framebuffer to the current machine's screen.</summary>
    private void RebuildScreen()
    {
        int w = _emulator.ScreenWidth, h = _emulator.ScreenHeight;
        _bgra = new byte[w * h * 4];
        _bitmap = new WriteableBitmap(w, h, 96, 96, PixelFormats.Bgra32, null);
        ScreenImage.Source = _bitmap;
    }

    /// <summary>Switch machine models: tear the current machine down (persisting
    /// its media), bring the other up at its own screen size, and load that
    /// model's remembered ROM if it is still around.</summary>
    private void SwitchModel(string model)
    {
        if (_settings.Model == model) return;
        Log.Line($"model switch: {_settings.Model} -> {model}");
        _emulator.Dispose();
        _settings.Model = model;
        _settings.Save();
        _emulator = CreateBackend(_settings);
        _emulator.WriteProtectFloppies = _settings.WriteProtectFloppies;
        RebuildScreen();
        ApplyScale();
        UpdateUi();
        if (!string.IsNullOrEmpty(_settings.ModelLastRom) && File.Exists(_settings.ModelLastRom))
            LoadRom(_settings.ModelLastRom!);
    }

    private void ModelClassic_Click(object sender, RoutedEventArgs e) => SwitchModel("classic");
    private void ModelQuadra_Click(object sender, RoutedEventArgs e) => SwitchModel("quadra650");

    private void Tick()
    {
        // Display only. The emulator produces frames (and audio) on its own thread;
        // copy the most recent one and blit it. TryGetFrame returns false when
        // nothing new has been produced since the previous refresh.
        if (_emulator.TryGetFrame(_bgra))
            _bitmap.WritePixels(new Int32Rect(0, 0, _emulator.ScreenWidth, _emulator.ScreenHeight),
                                _bgra, _emulator.ScreenWidth * 4, 0);
        // The machine ejects disks on its own; keep the menus and status honest.
        if (_emulator.ConsumeDiskStateChanged()) UpdateUi();
    }

    [DllImport("winmm.dll")] private static extern uint timeBeginPeriod(uint ms);
    [DllImport("winmm.dll")] private static extern uint timeEndPeriod(uint ms);

    // ---- input ----
    private void WireInput()
    {
        _baseTitle = Title;
        Deactivated += (_, _) => UnlockMouse();   // never leave the pointer trapped

        // Relative ("captured") mouse. On the first click we lock the pointer to
        // the window, hide the host cursor, and feed the Mac raw motion deltas,
        // re-centering the OS cursor after each move so it can travel forever
        // without hitting a screen edge. Middle-click (or losing focus) releases
        // it. This bypasses mapping the absolute cursor through the Viewbox scale,
        // which shrank motion and truncated sub-pixel movement away in the int cast.
        ScreenImage.MouseMove += (_, e) =>
        {
            if (!_mouseLocked || !_emulator.IsRomLoaded) return;
            if (!GetCursorPos(out POINT pt)) return;
            int dx = pt.X - _lockCx, dy = pt.Y - _lockCy;
            if (dx == 0 && dy == 0) return;                   // the warp-back itself
            bool down = e.LeftButton == MouseButtonState.Pressed;
            _emulator.MouseMove(dx, dy, down);
            SetCursorPos(_lockCx, _lockCy);                   // warp back to centre
        };
        ScreenImage.MouseLeftButtonDown += (_, _) =>
        {
            ScreenImage.Focus();
            if (!_mouseLocked) { LockMouse(); _ignoreUpAfterLock = true; return; }
            _emulator.MouseButton(true);
        };
        ScreenImage.MouseLeftButtonUp += (_, _) =>
        {
            if (_ignoreUpAfterLock) { _ignoreUpAfterLock = false; return; }
            _emulator.MouseButton(false);
        };
        ScreenImage.MouseDown += (_, e) =>
        {
            if (e.ChangedButton == MouseButton.Middle) UnlockMouse();
        };

        KeyDown += (_, e) =>
        {
            if (e.Key == Key.F11) { ToggleFullscreen(); e.Handled = true; return; }
            if (e.Key == Key.Escape && _fullscreen) { ToggleFullscreen(); e.Handled = true; return; }
            // A real ADB keyboard reports one DOWN per press; auto-repeat is the
            // guest OS's job (KeyThresh). Forwarding host repeats gives the Mac
            // phantom transitions, which garbles anything that counts keystrokes.
            if (e.IsRepeat) { e.Handled = true; return; }
            int code = AdbKeys.Map(e.SystemKey != Key.None ? e.SystemKey : e.Key);
            if (code >= 0) _emulator.KeyEvent(code, true);
        };
        KeyUp += (_, e) =>
        {
            int code = AdbKeys.Map(e.SystemKey != Key.None ? e.SystemKey : e.Key);
            if (code >= 0) _emulator.KeyEvent(code, false);
        };
    }

    // ---- Key Combos menu ----
    // ADB codes for the combo tokens. A combo presses left-to-right and releases
    // in reverse, spaced a few frames apart so the guest's ADB polling sees each
    // transition in order -- modifiers land first and lift last, like fingers.
    private static readonly Dictionary<string, int> ComboKeys = new()
    {
        ["cmd"] = 0x37, ["shift"] = 0x38, ["opt"] = 0x3A, ["ctrl"] = 0x3B,
        ["esc"] = 0x35, ["period"] = 0x2F,
        ["a"] = 0x00, ["c"] = 0x08, ["n"] = 0x2D, ["o"] = 0x1F, ["p"] = 0x23,
        ["q"] = 0x0C, ["s"] = 0x01, ["v"] = 0x09, ["w"] = 0x0D, ["x"] = 0x07,
        ["z"] = 0x06, ["1"] = 0x12, ["2"] = 0x13, ["3"] = 0x14,
    };

    private async void SendCombo_Click(object sender, RoutedEventArgs e)
    {
        if (!_emulator.IsRomLoaded) return;
        if ((sender as MenuItem)?.Tag is not string combo) return;
        int[] codes = combo.Split('+').Select(t => ComboKeys[t]).ToArray();
        foreach (int code in codes)
        {
            _emulator.KeyEvent(code, true);
            await Task.Delay(35);
        }
        for (int i = codes.Length - 1; i >= 0; i--)
        {
            _emulator.KeyEvent(codes[i], false);
            await Task.Delay(35);
        }
    }

    // ---- relative mouse capture ----
    private void LockMouse()
    {
        if (_mouseLocked) return;
        _mouseLocked = true;
        Mouse.Capture(ScreenImage);
        ScreenImage.Cursor = Cursors.None;
        RecenterCursor();
        if (_keyHook != null) _keyHook.Enabled = true;   // Cmd(Win)+Q, Alt+Tab etc. now reach the Mac
        Title = _baseTitle + "   —   input captured: keys go to the Mac (middle-click to release)";
    }

    private void UnlockMouse()
    {
        if (!_mouseLocked) return;
        _mouseLocked = false;
        _ignoreUpAfterLock = false;
        if (_keyHook != null) { _keyHook.Enabled = false; _keyHook.ReleaseAll(); }
        if (ScreenImage.IsMouseCaptured) ScreenImage.ReleaseMouseCapture();
        ScreenImage.Cursor = null;
        Title = _baseTitle;
    }

    // Park the OS cursor at the window's physical centre and remember that point;
    // motion deltas are measured from it and the cursor is warped back each move.
    private void RecenterCursor()
    {
        IntPtr hwnd = new System.Windows.Interop.WindowInteropHelper(this).Handle;
        if (hwnd != IntPtr.Zero && GetWindowRect(hwnd, out RECT r))
        {
            _lockCx = (r.Left + r.Right) / 2;
            _lockCy = (r.Top + r.Bottom) / 2;
            SetCursorPos(_lockCx, _lockCy);
        }
    }

    [StructLayout(LayoutKind.Sequential)] private struct POINT { public int X, Y; }
    [StructLayout(LayoutKind.Sequential)] private struct RECT { public int Left, Top, Right, Bottom; }
    [DllImport("user32.dll")] private static extern bool SetCursorPos(int x, int y);
    [DllImport("user32.dll")] private static extern bool GetCursorPos(out POINT p);
    [DllImport("user32.dll")] private static extern bool GetWindowRect(IntPtr hWnd, out RECT r);

    // ---- machine lifecycle ----
    private void LoadRom(string path)
    {
        // The two machines take different ROMs (Classic: 512 KB, Quadra
        // family: 1 MB). Feeding the wrong one wedges the CPU into a white
        // screen with no diagnostics -- refuse it with directions instead.
        long romLen = new FileInfo(path).Length;
        bool looksQuadra = romLen >= 1024 * 1024;
        if (looksQuadra != _settings.IsQuadra)
        {
            string msg = looksQuadra
                ? "This is a 1 MB Quadra-family ROM, but the current model is the Macintosh " +
                  "Classic.\n\nSwitch Machine > Model > Macintosh Quadra 650, then open this ROM there."
                : "This is a Classic-sized ROM, but the current model is the Quadra 650.\n\n" +
                  "Switch Machine > Model > Macintosh Classic, then open this ROM there.";
            Log.Line($"ROM refused (wrong model): {path} ({romLen} bytes, model={_settings.Model})");
            MessageBox.Show(this, msg, "OpenMac", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        Log.Line($"load ROM: {path}  (RAM={_settings.ModelRamMB} MB, bootRomDisk={_settings.BootRomDisk}, "
                 + $"floppies {(_settings.WriteProtectFloppies ? "write-protected" : "writable")})");
        try
        {
            _emulator.LoadRom(path, _settings.ModelRamMB, _settings.BootRomDisk);
            // Before any disk goes in: the tab is read at insertion.
            _emulator.WriteProtectFloppies = _settings.WriteProtectFloppies;
            // The Quadra build carries only the SCSI hard disk so far; the
            // Classic's other remembered media stay its own.
            if (!_settings.IsQuadra)
            {
                // A remembered path the core now refuses is forgotten rather
                // than retried on every boot; the refusal is in the log.
                if (!string.IsNullOrEmpty(_settings.LastFloppy) && File.Exists(_settings.LastFloppy) &&
                    !_emulator.InsertFloppy(_settings.LastFloppy!))
                {
                    Log.Line($"floppy refused: {_settings.LastFloppy} -- {_emulator.MediumNote(0)}");
                    _settings.LastFloppy = null;
                }
                if (_settings.ExternalDrive) _emulator.SetExternalDrive(true);
                if (!string.IsNullOrEmpty(_settings.LastExternalFloppy) &&
                    File.Exists(_settings.LastExternalFloppy) &&
                    !_emulator.InsertExternalFloppy(_settings.LastExternalFloppy!))
                {
                    Log.Line($"floppy refused: {_settings.LastExternalFloppy} -- {_emulator.MediumNote(1)}");
                    _settings.LastExternalFloppy = null;
                }
            }
            if (!string.IsNullOrEmpty(_settings.ModelLastHardDisk) && File.Exists(_settings.ModelLastHardDisk))
                _emulator.AttachHardDisk(_settings.ModelLastHardDisk!);
            if (!_settings.IsQuadra)
            {
                if (_settings.CdRomAttached) _emulator.SetCdRomAttached(true);
                if (!string.IsNullOrEmpty(_settings.LastCd) && File.Exists(_settings.LastCd) &&
                    !_emulator.InsertCd(_settings.LastCd!))
                {
                    Log.Line($"cd refused: {_settings.LastCd} -- {_emulator.CdMediumNote()}");
                    _settings.LastCd = null;
                }
                if (!string.IsNullOrEmpty(_settings.LastFolderDisk) &&
                    Directory.Exists(_settings.LastFolderDisk) &&
                    !_emulator.AttachFolderDisk(_settings.LastFolderDisk!, out string fdErr))
                {
                    Log.Line($"folder disk refused: {_settings.LastFolderDisk} -- {fdErr}");
                    _settings.LastFolderDisk = null;
                }
                if (_settings.Networking) _emulator.SetNetworking(true);
            }
        }
        catch (Exception ex)
        {
            Log.Line("  ROM load FAILED: " + ex);
            MessageBox.Show(this, "Could not load ROM:\n" + ex.Message, "OpenMac",
                            MessageBoxButton.OK, MessageBoxImage.Error);
            return;
        }
        Log.Line("  ROM loaded ok");
        _settings.PushRecentRom(path);
        _settings.Save();
        BuildRecentMenu();
        UpdateUi();
    }

    private void OpenRom_Click(object sender, RoutedEventArgs e)
    {
        if (FilePicker.Open(this, _settings, FilePicker.Rom, "Open Macintosh ROM",
                            "Macintosh ROM (*.rom;*.bin)|*.rom;*.bin|All files (*.*)|*.*",
                            _settings.ModelLastRom) is { } path)
            LoadRom(path);
    }

    private void BuildRecentMenu()
    {
        RecentMenu.Items.Clear();
        if (_settings.RecentRoms.Count == 0)
        {
            RecentMenu.Items.Add(new MenuItem { Header = "(none)", IsEnabled = false });
            return;
        }
        foreach (string path in _settings.RecentRoms)
        {
            var item = new MenuItem { Header = Path.GetFileName(path), ToolTip = path };
            string captured = path;
            item.Click += (_, _) => { if (File.Exists(captured)) LoadRom(captured); };
            RecentMenu.Items.Add(item);
        }
    }

    private void Reset_Click(object sender, RoutedEventArgs e)
    {
        // Full restart: reload the ROM so the SCSI bus is re-scanned and any hard disk
        // re-mounts (a warm reset leaves the prior boot's mount state behind).
        if (_emulator.RomPath is { } rom) LoadRom(rom);
        else _emulator.Reset();
    }

    private void Memory_Click(object sender, RoutedEventArgs e)
    {
        _settings.RamMB = int.Parse((string)((MenuItem)sender).Tag);
        _settings.Save();
        if (_emulator.IsRomLoaded && _emulator.RomPath is { } rom) LoadRom(rom);
        UpdateUi();
    }

    private void BootRomDisk_Click(object sender, RoutedEventArgs e)
    {
        _settings.BootRomDisk = !_settings.BootRomDisk;
        _settings.Save();
        // Reboot immediately so the toggle takes effect now (mirrors the RAM menu).
        if (_emulator.IsRomLoaded && _emulator.RomPath is { } rom) LoadRom(rom);
        UpdateUi();
    }

    // ---- disks ----
    // The core judges every file offered to a drive: containers (MacBinary,
    // DiskCopy 4.2) are stripped and mount; anything that is not floppy media is
    // refused with its nature named. A refusal leaves the drive untouched, so
    // don't remember the path as "the disk in the drive" -- surface the verdict.
    private bool TryInsert(string path, Func<string, bool> insert, int drive)
    {
        if (insert(path)) return true;
        string why = _emulator.MediumNote(drive);
        Log.Line($"floppy refused: {path} -- {why}");
        MessageBox.Show(this,
            Path.GetFileName(path) + " did not go in the drive.\n\n" + why,
            "Not a floppy", MessageBoxButton.OK, MessageBoxImage.Warning);
        return false;
    }

    private void InsertFloppy_Click(object sender, RoutedEventArgs e)
    {
        if (FilePicker.Open(this, _settings, FilePicker.Floppy, "Insert Floppy",
                            DiskImageFilter, _settings.LastFloppy) is { } path)
        {
            if (TryInsert(path, _emulator.InsertFloppy, 0))
            {
                _settings.LastFloppy = path;
                _settings.Save();
            }
            UpdateUi();
        }
    }

    private void EjectFloppy_Click(object sender, RoutedEventArgs e)
    {
        _emulator.EjectFloppy();
        _settings.LastFloppy = null;
        _settings.Save();
        UpdateUi();
    }

    // ---- external drive ----
    // A Classic has a second drive port. Connecting a mechanism makes the ROM
    // register a second floppy drive, and a disk put in after the machine has
    // started mounts like any other. A disk already sitting in it at power-on is
    // Disks go in locked by default. A disk image is usually a master somebody
    // else made, it cannot be re-made once overwritten, and mounting an HFS
    // volume read-write writes to it whether or not anyone asked -- the System
    // clears the volume-unmounted bit in the MDB as its first act. Unlocking is
    // a deliberate choice to let the Mac keep what it writes.
    private void WriteProtectFloppies_Click(object sender, RoutedEventArgs e)
    {
        bool on = WriteProtectItem.IsChecked;
        _settings.WriteProtectFloppies = on;
        _settings.Save();
        _emulator.WriteProtectFloppies = on;
        Log.Line($"floppies are now {(on ? "write-protected" : "writable")}");
        // The tab is read when a disk goes in, so re-seat whatever is already
        // in a drive rather than leaving the menu disagreeing with the machine.
        if (_emulator.FloppyPath is { } f && File.Exists(f)) _emulator.InsertFloppy(f);
        if (_emulator.ExternalFloppyPath is { } x && File.Exists(x))
            _emulator.InsertExternalFloppy(x);
        UpdateUi();
    }

    // thrown out by the ROM's own port probe, so put one in after booting.
    private void ExternalDrive_Click(object sender, RoutedEventArgs e)
    {
        bool on = ExternalDriveItem.IsChecked;
        _emulator.SetExternalDrive(on);
        if (!on) _settings.LastExternalFloppy = null;
        _settings.ExternalDrive = on;
        _settings.Save();
        // The ROM scans the drive ports once, at boot -- a drive connected to a
        // running machine is never noticed (nor would it be on a real Classic,
        // where plugging one in hot was against the manual). Reboot so the
        // toggle means what it says, the way the RAM menu does.
        if (_emulator.IsRomLoaded && _emulator.RomPath is { } rom) LoadRom(rom);
        UpdateUi();
    }

    private void InsertExternalFloppy_Click(object sender, RoutedEventArgs e)
    {
        if (FilePicker.Open(this, _settings, FilePicker.Floppy,
                            "Insert Floppy (External Drive)", DiskImageFilter,
                            _settings.LastExternalFloppy ?? _settings.LastFloppy) is { } path)
        {
            if (TryInsert(path, _emulator.InsertExternalFloppy, 1))
            {
                _settings.ExternalDrive = true;
                _settings.LastExternalFloppy = path;
                _settings.Save();
            }
            UpdateUi();
        }
    }

    private void EjectExternalFloppy_Click(object sender, RoutedEventArgs e)
    {
        _emulator.EjectExternalFloppy();
        _settings.LastExternalFloppy = null;
        _settings.Save();
        UpdateUi();
    }

    private void AttachHardDisk_Click(object sender, RoutedEventArgs e)
    {
        if (FilePicker.Open(this, _settings, FilePicker.HardDisk, "Attach Hard Disk",
                            "Disk image (*.img;*.dsk;*.hda)|*.img;*.dsk;*.hda|All files (*.*)|*.*",
                            _settings.LastHardDisk) is { } path)
            AttachHardDisk(path);
    }

    private void CreateHardDisk_Click(object sender, RoutedEventArgs e)
    {
        Log.Line("create hard disk: dialog opened");
        var dlg = new CreateHardDiskDialog(_settings) { Owner = this };
        bool made = dlg.ShowDialog() == true && dlg.CreatedPath is not null;
        Log.Line($"create hard disk: dialog closed, created={made}");
        if (made) AttachHardDisk(dlg.CreatedPath!);
    }

    private void AttachHardDisk(string path)
    {
        Log.Line($"attach hard disk: {path}");
        _emulator.AttachHardDisk(path);
        _settings.ModelLastHardDisk = path;
        _settings.Save();
        UpdateUi();
        // SCSI disks are found only during the ROM's boot scan, so one attached to a
        // running Mac won't appear until it restarts. Offer to restart now; LoadRom
        // re-attaches the disk (and any floppy) before the scan so it mounts.
        if (_emulator.IsRomLoaded && _emulator.RomPath is { } rom)
        {
            var r = MessageBox.Show(this,
                "The Mac scans the SCSI bus only at startup, so a newly attached hard " +
                "disk isn't visible until it restarts.\n\nRestart now to use it?",
                "Attach Hard Disk", MessageBoxButton.YesNo, MessageBoxImage.Question);
            if (r == MessageBoxResult.Yes) LoadRom(rom);
        }
    }

    private void DetachHardDisk_Click(object sender, RoutedEventArgs e)
    {
        _emulator.DetachHardDisk();
        _settings.ModelLastHardDisk = null;
        _settings.Save();
        UpdateUi();
    }

    // ---- CD-ROM ----
    // The drive is a SCSI device found during startup's bus scan, so attaching
    // one wants a restart; a disc put in later is noticed by the Apple CD
    // software's own polling, so inserting never does.
    private void CdDrive_Click(object sender, RoutedEventArgs e)
    {
        bool on = CdDriveItem.IsChecked;
        _emulator.SetCdRomAttached(on);
        if (!on) _settings.LastCd = null;
        _settings.CdRomAttached = on;
        _settings.Save();
        if (_emulator.IsRomLoaded && _emulator.RomPath is { } rom)
        {
            var r = MessageBox.Show(this,
                "The Mac scans the SCSI bus only at startup, so this change to the " +
                "CD-ROM drive isn't seen until it restarts.\n\nRestart now?",
                "CD-ROM Drive", MessageBoxButton.YesNo, MessageBoxImage.Question);
            if (r == MessageBoxResult.Yes) LoadRom(rom);
        }
        UpdateUi();
    }

    private void InsertCd_Click(object sender, RoutedEventArgs e)
    {
        if (FilePicker.Open(this, _settings, FilePicker.Cd, "Insert CD Image",
                "CD image (*.iso;*.toast;*.bin;*.cue;*.img;*.dsk)|*.iso;*.toast;*.bin;*.cue;*.img;*.dsk|"
                + "All files (*.*)|*.*",
                _settings.LastCd) is { } path)
        {
            InsertCdFrom(path);
            UpdateUi();
        }
    }

    /// <summary>Insert a CD image (from the menu or a drop), with the restart
    /// offer when the insertion also had to connect the drive. Returns whether
    /// the drive took the disc.</summary>
    private bool InsertCdFrom(string path)
    {
        bool driveWasAttached = _emulator.CdRomAttached;
        if (_emulator.InsertCd(path))
        {
            _settings.CdRomAttached = true;
            _settings.LastCd = path;
            _settings.Save();
            Log.Line($"cd inserted: {path} -- {_emulator.CdMediumNote()}");
            // Inserting also connected the drive; that half needs the boot scan.
            if (!driveWasAttached && _emulator.IsRomLoaded && _emulator.RomPath is { } rom)
            {
                var r = MessageBox.Show(this,
                    "The disc is in, but the drive itself was just connected, and the " +
                    "Mac scans the SCSI bus only at startup.\n\nRestart now so it appears?",
                    "CD-ROM Drive", MessageBoxButton.YesNo, MessageBoxImage.Question);
                if (r == MessageBoxResult.Yes) LoadRom(rom);
            }
            return true;
        }
        string why = _emulator.CdMediumNote();
        Log.Line($"cd refused: {path} -- {why}");
        MessageBox.Show(this,
            Path.GetFileName(path) + " did not go in the drive.\n\n" + why,
            "Not a CD image", MessageBoxButton.OK, MessageBoxImage.Warning);
        return false;
    }

    private void EjectCd_Click(object sender, RoutedEventArgs e)
    {
        _emulator.EjectCd();
        _settings.LastCd = null;
        _settings.Save();
        UpdateUi();
    }

    // ---- networking ----
    private void Networking_Click(object sender, RoutedEventArgs e)
    {
        bool on = NetworkingItem.IsChecked;
        _emulator.SetNetworking(on);
        _settings.Networking = on;
        _settings.Save();
        if (on && _emulator.IsRomLoaded && _emulator.RomPath is { } rom)
        {
            var r = MessageBox.Show(this,
                "The adapter is on the bus. The Dayna driver loads with the System, " +
                "so networking works fully after a restart.\n\nRestart now?",
                "Networking", MessageBoxButton.YesNo, MessageBoxImage.Question);
            if (r == MessageBoxResult.Yes) LoadRom(rom);
        }
        UpdateUi();
    }

    // ---- folder disk ----
    private void OpenFolderDisk_Click(object sender, RoutedEventArgs e)
    {
        if (!_emulator.IsRomLoaded)
        {
            MessageBox.Show(this, "Load a ROM first (File ▸ Open ROM…).", "OpenMac",
                            MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        if (_emulator.TransferDiskLabel is { } tdl)
        {
            MessageBox.Show(this,
                $"The transfer disk “{tdl}” is using the second SCSI seat. Restart the " +
                "machine to release it, then open the folder disk.",
                "Folder Disk", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        var dlg = new Microsoft.Win32.OpenFolderDialog { Title = "Open Host Folder as Disk" };
        if (!string.IsNullOrEmpty(_settings.LastFolderDisk) &&
            Directory.Exists(_settings.LastFolderDisk))
            dlg.InitialDirectory = _settings.LastFolderDisk;
        if (dlg.ShowDialog(this) != true) return;

        if (_emulator.AttachFolderDisk(dlg.FolderName, out string error))
        {
            _settings.LastFolderDisk = dlg.FolderName;
            _settings.Save();
            UpdateUi();
            // The volume mounts on its own a few seconds after the System is
            // up (the machine posts its mount); mid-session it appears without
            // a restart, but only if the boot-time bus scan installed the
            // second disk's driver — which happens whenever a folder disk was
            // attached at startup. First-time mid-session attach wants a
            // restart so the driver loads.
            if (_emulator.IsRomLoaded && _emulator.RomPath is { } rom)
            {
                var r = MessageBox.Show(this,
                    "The folder is on the bus. If no folder disk was attached when this " +
                    "Mac started, its driver hasn't been loaded by the startup scan yet." +
                    "\n\nRestart now so the disk appears?",
                    "Folder Disk", MessageBoxButton.YesNo, MessageBoxImage.Question);
                if (r == MessageBoxResult.Yes) LoadRom(rom);
            }
        }
        else
        {
            Log.Line($"folder disk refused: {dlg.FolderName} -- {error}");
            MessageBox.Show(this, "The folder did not become a disk.\n\n" + error,
                            "Folder Disk", MessageBoxButton.OK, MessageBoxImage.Warning);
        }
    }

    private void CloseFolderDisk_Click(object sender, RoutedEventArgs e)
    {
        _emulator.DetachFolderDisk();   // syncs changes back to the folder first
        _settings.LastFolderDisk = null;
        _settings.Save();
        UpdateUi();
        MessageBox.Show(this,
            "Changes were written back to the folder (the log has the counts). " +
            "The Mac may still show the volume until it restarts — like a SCSI " +
            "drive unplugged while running.",
            "Folder Disk", MessageBoxButton.OK, MessageBoxImage.Information);
    }

    // ---- drag & drop / media routing ----
    // Anything dropped on the window lands in the right place: ROMs load,
    // floppy-sized files go to the floppy drives on the core's own judgment
    // (containers stripped, non-media refused with its nature named), CD images
    // go to the CD drive, and partitioned or bare-HFS images attach as the hard
    // disk. Extensions only break the tie between shapes that are byte-identical
    // on purpose: an HFS master could be a CD or an HD, and .iso/.toast/.cue say
    // which was meant.
    private static readonly string[] CdOnlyExtensions = { ".iso", ".toast", ".cue" };

    // Loose Mac files — the archives and encodings the StuffIt-era tools open:
    // native .sit/.sea, BinHex, Compact Pro, and the DOS-side .zip/.lha the
    // Deluxe translators handle. (.bin MacBinary is sniffed separately, since
    // a .bin may also be a floppy or a raw CD.) They ride into the guest on a
    // write-protected 1.44 MB transfer floppy.
    private static readonly string[] TransferExtensions =
        { ".sit", ".sea", ".cpt", ".hqx", ".zip", ".lha", ".lzh" };

    private void Window_DragOver(object sender, DragEventArgs e)
    {
        e.Effects = e.Data.GetDataPresent(DataFormats.FileDrop)
            ? DragDropEffects.Copy : DragDropEffects.None;
        e.Handled = true;
    }

    private void Window_Drop(object sender, DragEventArgs e)
    {
        if (e.Data.GetData(DataFormats.FileDrop) is not string[] files) return;
        foreach (string f in files)
            if (File.Exists(f)) RouteMedia(f);
        UpdateUi();
    }

    private void RouteMedia(string path)
    {
        string ext = Path.GetExtension(path).ToLowerInvariant();
        if (ext == ".rom") { LoadRom(path); return; }
        if (!_emulator.IsRomLoaded)
        {
            Log.Line($"drop ignored (no ROM loaded): {path}");
            MessageBox.Show(this,
                "Load a ROM first (File ▸ Open ROM…), then drop disks in.",
                "OpenMac", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }

        long size;
        try { size = new FileInfo(path).Length; } catch { return; }

        // Archives and encodings go INTO the guest on a transfer floppy, not
        // into a drive as media.
        if (TransferExtensions.Contains(ext)) { TransferDrop(path, requireMacBinary: false); return; }

        // Floppy-sized files: let the core's judge look first. The internal
        // drive takes it unless it's occupied and the external one is free —
        // the two-disk install flow dropped as a pair lands one in each.
        bool triedFloppy = false;
        if (size <= 3_500_000 && !CdOnlyExtensions.Contains(ext))
        {
            triedFloppy = true;
            bool toExternal = _emulator.FloppyPath is not null &&
                              _emulator.ExternalDriveAttached &&
                              _emulator.ExternalFloppyPath is null;
            bool ok = toExternal ? _emulator.InsertExternalFloppy(path)
                                 : _emulator.InsertFloppy(path);
            if (ok)
            {
                if (toExternal) _settings.LastExternalFloppy = path;
                else _settings.LastFloppy = path;
                _settings.Save();
                Log.Line($"drop: {Path.GetFileName(path)} -> "
                         + $"{(toExternal ? "external" : "internal")} floppy drive");
                return;
            }
            // Not floppy media — let the other drives look at it.
        }

        // A .bin that isn't floppy media may be a MacBinary-wrapped loose file
        // (Game.sit.bin and friends): decode it onto a transfer floppy.
        if (ext == ".bin" && TransferDrop(path, requireMacBinary: true)) return;

        if (CdOnlyExtensions.Contains(ext) || LooksLikeCdImage(path))
        {
            if (InsertCdFrom(path))
                Log.Line($"drop: {Path.GetFileName(path)} -> CD-ROM drive");
            return;
        }

        if (LooksLikeHardDisk(path))
        {
            Log.Line($"drop: {Path.GetFileName(path)} -> hard disk");
            AttachHardDisk(path);
            return;
        }

        string why = triedFloppy ? _emulator.MediumNote(0)
                                 : "not floppy media, a CD image, or a hard-disk image";
        Log.Line($"drop refused: {path} -- {why}");
        MessageBox.Show(this,
            Path.GetFileName(path) + " did not go in any drive.\n\n" + why,
            "OpenMac", MessageBoxButton.OK, MessageBoxImage.Warning);
    }

    /// <summary>Put one loose Mac file into the guest on a write-protected
    /// 1.44 MB transfer floppy. With <paramref name="requireMacBinary"/> the
    /// file must decode as MacBinary (the quiet .bin fall-through); without it,
    /// failures are reported to the person who dropped the file.</summary>
    private bool TransferDrop(string path, bool requireMacBinary)
    {
        long size;
        try { size = new FileInfo(path).Length; } catch { return false; }
        if (size > 1_300_000)
        {
            // Too big for a floppy: ride the second SCSI disk instead, sized
            // to fit and read-only.
            if (requireMacBinary)
            {
                try
                {
                    if (!MacBinary.TryDecode(File.ReadAllBytes(path), out _)) return false;
                }
                catch { return false; }
            }
            return TransferBigDrop(path);
        }
        if (requireMacBinary)
        {
            try
            {
                if (!MacBinary.TryDecode(File.ReadAllBytes(path), out _)) return false;
            }
            catch { return false; }
        }

        byte[]? img = FolderDisk.BuildTransferFloppy(path, out string error);
        if (img is null)
        {
            if (!requireMacBinary)
            {
                Log.Line($"transfer refused: {path} -- {error}");
                MessageBox.Show(this,
                    Path.GetFileName(path) + " did not fit a transfer floppy.\n\n" + error +
                    "\n\nPut it in a folder and use File ▸ Open Host Folder as Disk instead.",
                    "OpenMac", MessageBoxButton.OK, MessageBoxImage.Information);
            }
            return false;
        }

        string temp = Path.Combine(Path.GetTempPath(), "OpenMac",
                                   Path.GetFileNameWithoutExtension(path) + ".transfer.img");
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(temp)!);
            File.WriteAllBytes(temp, img);
        }
        catch (Exception ex)
        {
            Log.Line($"transfer failed: {ex.Message}");
            return false;
        }

        bool toExternal = _emulator.FloppyPath is not null &&
                          _emulator.ExternalDriveAttached &&
                          _emulator.ExternalFloppyPath is null;
        // The disk goes in locked: the guest only copies off it, and the tab
        // means its own writes never chase a temp file. The user's global
        // write-protect choice comes right back.
        bool saved = _emulator.WriteProtectFloppies;
        _emulator.WriteProtectFloppies = true;
        bool ok = toExternal ? _emulator.InsertExternalFloppy(temp)
                             : _emulator.InsertFloppy(temp);
        _emulator.WriteProtectFloppies = saved;
        if (ok)
            Log.Line($"drop: {Path.GetFileName(path)} -> transfer floppy in the "
                     + $"{(toExternal ? "external" : "internal")} drive");
        return ok;
    }

    /// <summary>An archive too big for a floppy becomes a read-only transfer
    /// disk on the second SCSI seat. One at a time, and the seat is shared
    /// with the folder disk — the occupant is named rather than clobbered
    /// (swapping the image under a volume the Mac still has mounted crosses
    /// its cached catalog with the new disk's blocks).</summary>
    private bool TransferBigDrop(string path)
    {
        if (_emulator.FolderDiskPath is { } fd)
        {
            MessageBox.Show(this,
                Path.GetFileName(path) + " needs the second SCSI disk, but the folder disk " +
                $"“{Path.GetFileName(fd.TrimEnd('\\', '/'))}” is using it.\n\n" +
                "Close the folder disk first — or just copy the file into that folder " +
                "and restart; it rides in with the folder.",
                "Transfer Disk", MessageBoxButton.OK, MessageBoxImage.Information);
            return true;   // handled: told the user what to do
        }
        if (_emulator.TransferDiskLabel is { } prev)
        {
            MessageBox.Show(this,
                $"The transfer disk “{prev}” is still attached. Drag its volume to the " +
                "Trash in the Mac, then restart before dropping another big file.",
                "Transfer Disk", MessageBoxButton.OK, MessageBoxImage.Information);
            return true;
        }
        if (!_emulator.AttachTransferDisk(path, out string error))
        {
            Log.Line($"transfer refused: {path} -- {error}");
            MessageBox.Show(this,
                Path.GetFileName(path) + " did not become a transfer disk.\n\n" + error,
                "Transfer Disk", MessageBoxButton.OK, MessageBoxImage.Warning);
            return true;
        }
        UpdateUi();
        if (_emulator.TransferDiskResident)
        {
            Log.Line($"drop: {Path.GetFileName(path)} -> transfer disk (mounts in a few seconds)");
        }
        else if (_emulator.IsRomLoaded && _emulator.RomPath is { } rom)
        {
            var r = MessageBox.Show(this,
                "The disk is on the bus, but its driver loads during the startup scan." +
                "\n\nRestart now so it appears?",
                "Transfer Disk", MessageBoxButton.YesNo, MessageBoxImage.Question);
            if (r == MessageBoxResult.Yes) LoadRom(rom);
        }
        return true;
    }

    /// <summary>ISO/High Sierra volume descriptor or raw 2352-byte sync — the
    /// shapes that mean "CD" regardless of what the file is called.</summary>
    private static bool LooksLikeCdImage(string path)
    {
        try
        {
            using var fs = File.OpenRead(path);
            byte[] head = new byte[16];
            if (fs.Read(head, 0, 16) == 16 &&
                head[0] == 0x00 && head[1] == 0xFF && head[11] == 0x00 &&
                head[2] == 0xFF && head[10] == 0xFF)
                return true;   // raw 2352 sync pattern
            if (fs.Length >= 17L * 2048)
            {
                byte[] pvd = new byte[16];
                fs.Seek(16L * 2048, SeekOrigin.Begin);
                if (fs.Read(pvd, 0, 16) == 16)
                {
                    if (pvd[1] == 'C' && pvd[2] == 'D' && pvd[3] == '0' &&
                        pvd[4] == '0' && pvd[5] == '1') return true;   // ISO 9660
                    if (pvd[9] == 'C' && pvd[10] == 'D' && pvd[11] == 'R' &&
                        pvd[12] == 'O' && pvd[13] == 'M') return true; // High Sierra
                }
            }
        }
        catch { /* unreadable = not a CD */ }
        return false;
    }

    /// <summary>An Apple partition map ('ER') or a bare HFS volume ('BD' at
    /// 1024) that isn't floppy-sized — the shapes the SCSI disk mounts.</summary>
    private static bool LooksLikeHardDisk(string path)
    {
        try
        {
            using var fs = File.OpenRead(path);
            byte[] b = new byte[2];
            if (fs.Read(b, 0, 2) == 2 && b[0] == 0x45 && b[1] == 0x52) return true;
            if (fs.Length >= 1536)
            {
                fs.Seek(1024, SeekOrigin.Begin);
                if (fs.Read(b, 0, 2) == 2 && b[0] == 0x42 && b[1] == 0x44) return true;
            }
        }
        catch { /* unreadable = not a disk */ }
        return false;
    }

    // ---- view ----
    private void Scale_Click(object sender, RoutedEventArgs e)
    {
        _settings.Scale = int.Parse((string)((MenuItem)sender).Tag);
        _settings.Save();
        ApplyScale();
        UpdateUi();
    }

    private void ApplyScale()
    {
        if (_settings.Scale <= 0 || _fullscreen) return;
        if (ScreenHost.ActualWidth <= 0) return;
        double chromeW = ActualWidth - ScreenHost.ActualWidth;
        double chromeH = ActualHeight - ScreenHost.ActualHeight;
        Width = _emulator.ScreenWidth * _settings.Scale + chromeW;
        Height = _emulator.ScreenHeight * _settings.Scale + chromeH;
    }

    private void Fullscreen_Click(object sender, RoutedEventArgs e) => ToggleFullscreen();

    private void ToggleFullscreen()
    {
        if (!_fullscreen)
        {
            _savedStyle = WindowStyle;
            _savedState = WindowState;
            WindowState = WindowState.Normal;
            WindowStyle = WindowStyle.None;
            ResizeMode = ResizeMode.NoResize;
            WindowState = WindowState.Maximized;
            _fullscreen = true;
        }
        else
        {
            WindowStyle = _savedStyle;
            ResizeMode = ResizeMode.CanResize;
            WindowState = _savedState;
            _fullscreen = false;
            ApplyScale();
        }
    }

    private void Debugger_Click(object sender, RoutedEventArgs e) =>
        MessageBox.Show(this,
            "A live register/disassembly/backtrace panel will dock here once the native core is "
            + "linked into the GUI.\n\nToday the headless monitor (openmac_trace) already provides "
            + "step-over/step-out, conditional breakpoints, branch tracing, and struct dumps.",
            "Debugger", MessageBoxButton.OK, MessageBoxImage.Information);

    private void About_Click(object sender, RoutedEventArgs e) =>
        MessageBox.Show(this,
            "OpenMac\nA from-scratch Macintosh Classic emulator.\n\n"
            + "Custom 68000 core, VIA 6522 / RTC / ADB / IWM, and a high-level .Sony disk driver.",
            "About OpenMac", MessageBoxButton.OK, MessageBoxImage.Information);

    private void Exit_Click(object sender, RoutedEventArgs e) => Close();

    // ---- ui state ----
    private void UpdateUi()
    {
        Overlay.Visibility = _emulator.IsRomLoaded ? Visibility.Collapsed : Visibility.Visible;

        StatusState.Text = _emulator.IsRomLoaded ? "Running" : "Stopped";
        string machine = _emulator.IsRomLoaded
            ? (_settings.IsQuadra ? "Quadra 650  •  " : "") + $"{_settings.ModelRamMB} MB"
              + (_emulator.FloppyPath is { } f ? $"  •  Floppy: {Path.GetFileName(f)}" : "")
              + (_emulator.ExternalFloppyPath is { } f2
                  ? $"  •  External: {Path.GetFileName(f2)}" : "")
              + (_emulator.HardDiskAttached && _emulator.HardDiskPath is { } hd
                  ? $"  •  HD: {Path.GetFileName(hd)}" : "")
              + (_emulator.CdPath is { } cd ? $"  •  CD: {Path.GetFileName(cd)}" : "")
              + (_emulator.FolderDiskPath is { } fd
                  ? $"  •  Folder: {Path.GetFileName(fd.TrimEnd('\\', '/'))}" : "")
              + (_emulator.TransferDiskLabel is { } td ? $"  •  Drop: {td}" : "")
            : "No ROM loaded";
        StatusMachine.Text = machine;
        Title = _emulator.IsRomLoaded && _emulator.RomPath is { } r
            ? $"OpenMac — {Path.GetFileName(r)}" : "OpenMac";

        ModelClassicItem.IsChecked = !_settings.IsQuadra;
        ModelQuadraItem.IsChecked = _settings.IsQuadra;
        Mem1Item.IsChecked = !_settings.IsQuadra && _settings.RamMB == 1;
        Mem2Item.IsChecked = !_settings.IsQuadra && _settings.RamMB == 2;
        Mem4Item.IsChecked = !_settings.IsQuadra && _settings.RamMB == 4;
        Mem1Item.IsEnabled = Mem2Item.IsEnabled = Mem4Item.IsEnabled = !_settings.IsQuadra;
        BootRomDiskItem.IsChecked = _settings.BootRomDisk;
        BootRomDiskItem.IsEnabled = !_settings.IsQuadra;
        WriteProtectItem.IsChecked = _settings.WriteProtectFloppies;

        ScaleFitItem.IsChecked = _settings.Scale == 0;
        Scale1Item.IsChecked = _settings.Scale == 1;
        Scale2Item.IsChecked = _settings.Scale == 2;
        Scale3Item.IsChecked = _settings.Scale == 3;

        // Name the disk each eject would actually throw out. With two drives, "Eject
        // Floppy" alone leaves the user to remember which disk went where.
        static string EjectLabel(string drive, string? path) =>
            path is null ? $"Eject Floppy from {drive} Drive"
                         : $"Eject “{Path.GetFileNameWithoutExtension(path)}” from {drive} Drive";

        EjectFloppyItem.IsEnabled = _emulator.FloppyPath is not null;
        EjectFloppyItem.Header = EjectLabel("Internal", _emulator.FloppyPath);
        ExternalDriveItem.IsChecked = _emulator.ExternalDriveAttached;
        // Inserting into the external drive connects one if there isn't one, so
        // the item stays reachable rather than making the checkbox a prerequisite.
        EjectFloppy2Item.IsEnabled = _emulator.ExternalFloppyPath is not null;
        EjectFloppy2Item.Header = EjectLabel("External", _emulator.ExternalFloppyPath);
        DetachHdItem.IsEnabled = _emulator.HardDiskAttached;
        CdDriveItem.IsChecked = _emulator.CdRomAttached;
        EjectCdItem.IsEnabled = _emulator.CdPath is not null;
        EjectCdItem.Header = _emulator.CdPath is { } cdPath
            ? $"Eject “{Path.GetFileNameWithoutExtension(cdPath)}”" : "Eject CD";
        CloseFolderDiskItem.IsEnabled = _emulator.FolderDiskPath is not null;
        NetworkingItem.IsChecked = _emulator.NetworkingEnabled;
    }
}
