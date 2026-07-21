using System.IO;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using Microsoft.Win32;
using OpenMac.Gui.Dialogs;
using OpenMac.Gui.Emulation;

namespace OpenMac.Gui;

public partial class MainWindow : Window
{
    private readonly Settings _settings;
    private IEmulator _emulator;
    private readonly WriteableBitmap _bitmap;
    private readonly byte[] _bgra;
    private readonly DispatcherTimer _timer;

    private bool _mouseLocked;
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
        _emulator = CreateBackend();
        int w = _emulator.ScreenWidth, h = _emulator.ScreenHeight;
        _bgra = new byte[w * h * 4];
        _bitmap = new WriteableBitmap(w, h, 96, 96, PixelFormats.Bgra32, null);
        ScreenImage.Source = _bitmap;

        StatusBackend.Text = _emulator.IsRealCore ? "core: native" : "core: stub (not linked)";
        Log.Line($"GUI ready — {_emulator.BackendName} backend, screen {w}x{h}");

        _timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(1000.0 / 60.0) };
        _timer.Tick += (_, _) => Tick();
        _timer.Start();

        WireInput();
        BuildRecentMenu();
        UpdateUi();

        Loaded += (_, _) =>
        {
            ApplyScale();
            if (!string.IsNullOrEmpty(_settings.LastRom) && File.Exists(_settings.LastRom))
                LoadRom(_settings.LastRom!);
        };
        Closing += (_, _) => _settings.Save();
    }

    /// <summary>Real core if openmac_c.dll loads; otherwise the stub preview.</summary>
    private static IEmulator CreateBackend()
    {
        try
        {
            Native.omac_version();   // probes the native DLL; throws if it's missing
            Log.Line("backend: native core (openmac_c.dll)");
            return new NativeEmulator();
        }
        catch (Exception ex)
        {
            Log.Line("backend: native core unavailable, using stub — " + ex.Message);
            return new StubEmulator();
        }
    }

    private void Tick()
    {
        _emulator.RunFrame();
        _emulator.RenderTo(_bgra);
        _bitmap.WritePixels(new Int32Rect(0, 0, _emulator.ScreenWidth, _emulator.ScreenHeight),
                            _bgra, _emulator.ScreenWidth * 4, 0);
    }

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
            int code = AdbKeys.Map(e.SystemKey != Key.None ? e.SystemKey : e.Key);
            if (code >= 0) _emulator.KeyEvent(code, true);
        };
        KeyUp += (_, e) =>
        {
            int code = AdbKeys.Map(e.SystemKey != Key.None ? e.SystemKey : e.Key);
            if (code >= 0) _emulator.KeyEvent(code, false);
        };
    }

    // ---- relative mouse capture ----
    private void LockMouse()
    {
        if (_mouseLocked) return;
        _mouseLocked = true;
        Mouse.Capture(ScreenImage);
        ScreenImage.Cursor = Cursors.None;
        RecenterCursor();
        Title = _baseTitle + "   —   mouse captured (middle-click to release)";
    }

    private void UnlockMouse()
    {
        if (!_mouseLocked) return;
        _mouseLocked = false;
        _ignoreUpAfterLock = false;
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
        Log.Line($"load ROM: {path}  (RAM={_settings.RamMB} MB, bootRomDisk={_settings.BootRomDisk})");
        try
        {
            _emulator.LoadRom(path, _settings.RamMB, _settings.BootRomDisk);
            if (!string.IsNullOrEmpty(_settings.LastFloppy) && File.Exists(_settings.LastFloppy))
                _emulator.InsertFloppy(_settings.LastFloppy!);
            if (!string.IsNullOrEmpty(_settings.LastHardDisk) && File.Exists(_settings.LastHardDisk))
                _emulator.AttachHardDisk(_settings.LastHardDisk!);
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
        var dlg = new OpenFileDialog
        {
            Title = "Open Macintosh ROM",
            Filter = "Macintosh ROM (*.rom;*.bin)|*.rom;*.bin|All files (*.*)|*.*",
        };
        if (dlg.ShowDialog(this) == true) LoadRom(dlg.FileName);
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

    private void Reset_Click(object sender, RoutedEventArgs e) => _emulator.Reset();

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
        UpdateUi();
    }

    // ---- disks ----
    private void InsertFloppy_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Title = "Insert Floppy",
            Filter = "Disk image (*.img;*.image;*.dsk;*.dc42)|*.img;*.image;*.dsk;*.dc42|All files (*.*)|*.*",
        };
        if (dlg.ShowDialog(this) == true)
        {
            _emulator.InsertFloppy(dlg.FileName);
            _settings.LastFloppy = dlg.FileName;
            _settings.Save();
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

    private void AttachHardDisk_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Title = "Attach Hard Disk",
            Filter = "Disk image (*.img;*.dsk;*.hda)|*.img;*.dsk;*.hda|All files (*.*)|*.*",
        };
        if (dlg.ShowDialog(this) == true) AttachHardDisk(dlg.FileName);
    }

    private void CreateHardDisk_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new CreateHardDiskDialog { Owner = this };
        if (dlg.ShowDialog() == true && dlg.CreatedPath is { } path)
            AttachHardDisk(path);
    }

    private void AttachHardDisk(string path)
    {
        Log.Line($"attach hard disk: {path}");
        _emulator.AttachHardDisk(path);
        _settings.LastHardDisk = path;
        _settings.Save();
        UpdateUi();
    }

    private void DetachHardDisk_Click(object sender, RoutedEventArgs e)
    {
        _emulator.DetachHardDisk();
        _settings.LastHardDisk = null;
        _settings.Save();
        UpdateUi();
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
            ? $"{_settings.RamMB} MB"
              + (_emulator.FloppyPath is { } f ? $"  •  Floppy: {Path.GetFileName(f)}" : "")
              + (_emulator.HardDiskAttached && _emulator.HardDiskPath is { } hd
                  ? $"  •  HD: {Path.GetFileName(hd)}" : "")
            : "No ROM loaded";
        StatusMachine.Text = machine;
        Title = _emulator.IsRomLoaded && _emulator.RomPath is { } r
            ? $"OpenMac — {Path.GetFileName(r)}" : "OpenMac";

        Mem1Item.IsChecked = _settings.RamMB == 1;
        Mem2Item.IsChecked = _settings.RamMB == 2;
        Mem4Item.IsChecked = _settings.RamMB == 4;
        BootRomDiskItem.IsChecked = _settings.BootRomDisk;

        ScaleFitItem.IsChecked = _settings.Scale == 0;
        Scale1Item.IsChecked = _settings.Scale == 1;
        Scale2Item.IsChecked = _settings.Scale == 2;
        Scale3Item.IsChecked = _settings.Scale == 3;

        EjectFloppyItem.IsEnabled = _emulator.FloppyPath is not null;
        DetachHdItem.IsEnabled = _emulator.HardDiskAttached;
    }
}
