using System.IO;
using System.Text;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Interop;
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
    // The click which captures the host pointer is not a Macintosh click. Keep
    // the guest button as an explicit state instead of deriving it from WPF's
    // physical LeftButton flag during motion: the latter remains pressed while
    // the capture click is being released and used to leave the Mac button
    // stuck down after that release was intentionally ignored.
    private bool _guestMouseDown;
    private bool _captureMotionLogged;
    private HwndSource? _windowSource;
    private bool _rawMouseRegistered;
    private long _pendingMouseDx, _pendingMouseDy;
    private ulong _capturedMotionPackets;
    private long _capturedMouseDx, _capturedMouseDy;
    private KeyboardHook? _keyHook;        // swallows host combos while input is captured
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
        // WPF coalesces legacy WM_MOUSEMOVE messages around SetCursorPos. That
        // can reduce a captured mouse to one synthetic pixel of motion. Raw
        // Input is attached once the HWND exists and supplies the HID's actual
        // relative deltas; the WPF button events remain enabled.
        SourceInitialized += (_, _) => InitializeRawMouse();
        // The captured-input keyboard hook lives for the window's lifetime and
        // does nothing until capture switches it on. It reads _emulator through
        // this, so a backend swap never leaves it pointing at a dead machine.
        _keyHook = new KeyboardHook((code, down) => _emulator.KeyEvent(code, down), ToggleFullscreen);
        BuildRecentMenu();
        BuildMonitorMenu();
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
            UnlockMouse();
            ShutdownRawMouse();
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
            if (settings.IsIifx)
            {
                Log.Line("backend: native core (openmac_c.dll), Macintosh IIfx");
                return new IifxEmulator { VideoRomPath = settings.VideoRomIifx };
            }
            if (settings.IsQuadra)
            {
                Log.Line("backend: native core (openmac_c.dll), Quadra 650");
                return new QuadraEmulator { Monitor = settings.MonitorQuadra };
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
    private void ModelIifx_Click(object sender, RoutedEventArgs e) => SwitchModel("iifx");
    private void ModelQuadra_Click(object sender, RoutedEventArgs e) => SwitchModel("quadra650");

    /// <summary>Fill the Monitor menu from the displays the machine can drive.
    /// Built from the core's own list so the menu cannot drift from what the
    /// machine supports.</summary>
    private void BuildMonitorMenu()
    {
        // Nothing built from the native core may be allowed to stop the window
        // from opening. This runs in the constructor, and an older DLL without
        // the display list took the whole application down with it.
        try
        {
            MonitorMenu.Items.Clear();
            string current = _settings.MonitorQuadra ?? "13-inch RGB";
            foreach (var (name, w, h) in QuadraEmulator.Displays())
            {
                var item = new MenuItem
                {
                    Header = $"Apple {name} — {w}×{h}",
                    Tag = name,
                    IsCheckable = true,
                    IsChecked = name == current,
                };
                item.Click += Monitor_Click;
                MonitorMenu.Items.Add(item);
            }
        }
        catch (Exception ex)
        {
            Log.Line("monitor list unavailable: " + ex.Message);
            MonitorMenu.IsEnabled = false;
        }
    }

    private void Monitor_Click(object sender, RoutedEventArgs e)
    {
        if (sender is not MenuItem item || item.Tag is not string name) return;
        if (_settings.MonitorQuadra == name) { BuildMonitorMenu(); return; }
        _settings.MonitorQuadra = name;
        _settings.Save();
        Log.Line($"monitor: {name}");
        BuildMonitorMenu();
        // The ROM reads the video port's sense lines once, while it starts, so
        // a different monitor means starting the machine again -- which is what
        // unplugging one and plugging in another would mean too.
        if (_emulator is QuadraEmulator q) q.Monitor = name;
        if (!string.IsNullOrEmpty(_settings.ModelLastRom) && File.Exists(_settings.ModelLastRom))
        {
            LoadRom(_settings.ModelLastRom!);
            RebuildScreen();   // the new display is a different size
            ApplyScale();
        }
        UpdateUi();
    }

    private void Tick()
    {
        // Coalesce high-poll-rate host mice to the display cadence. This keeps
        // WM_INPUT non-blocking while still delivering every relative count to
        // the emulated ADB mouse before the next displayed guest frame.
        FlushMouseMotion();
        // Display only. The emulator produces frames (and audio) on its own thread;
        // copy the most recent one and blit it. TryGetFrame returns false when
        // nothing new has been produced since the previous refresh.
        int w = _emulator.ScreenWidth, h = _emulator.ScreenHeight;
        // A machine can come up on a different display than the window was
        // built for. Re-fit rather than write past the bitmap: this runs from
        // the render handler, where an exception takes the application down.
        if (_bitmap.PixelWidth != w || _bitmap.PixelHeight != h)
        {
            RebuildScreen();
            ApplyScale();
            return;
        }
        if (_emulator.TryGetFrame(_bgra))
            _bitmap.WritePixels(new Int32Rect(0, 0, w, h), _bgra, w * 4, 0);
        // The machine ejects disks on its own; keep the menus and status honest.
        if (_emulator.ConsumeDiskStateChanged())
        {
            SeatWaitingFloppy();   // the drive is empty and the old disk is saved
            UpdateUi();
        }
        // The speed readout: once a second, from the run loop's own count.
        long tick = Environment.TickCount64;
        if (tick - _speedShownAt >= 1000)
        {
            _speedShownAt = tick;
            StatusSpeed.Text = _emulator.IsRomLoaded
                ? $"speed {_emulator.SpeedPercent:0}%   •   " : "";
        }
    }
    private long _speedShownAt;

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
        ScreenImage.MouseMove += (_, _) =>
        {
            // Raw Input does not include SetCursorPos's synthetic movement and
            // is the normal path. Retain the old calculation only as a fallback
            // for a Windows configuration that refuses device registration.
            if (_rawMouseRegistered) return;
            if (!_mouseLocked || !_emulator.IsRomLoaded) return;
            if (!GetCursorPos(out POINT pt)) return;
            int dx = pt.X - _lockCx, dy = pt.Y - _lockCy;
            if (dx == 0 && dy == 0) return;                   // the warp-back itself
            QueueMouseMotion(dx, dy, "legacy");
            SetCursorPos(_lockCx, _lockCy);                   // warp back to centre
        };
        ScreenImage.MouseLeftButtonDown += (_, e) =>
        {
            ScreenImage.Focus();
            if (!_mouseLocked) { LockMouse(); e.Handled = true; return; }
            if (_guestMouseDown) return;
            FlushMouseMotion();
            _guestMouseDown = true;
            _emulator.MouseButton(true);
        };
        ScreenImage.MouseLeftButtonUp += (_, e) =>
        {
            // A release belonging to the capture gesture has no matching guest
            // press. A real guest click is forwarded exactly once.
            if (!_guestMouseDown) { e.Handled = true; return; }
            FlushMouseMotion();
            _guestMouseDown = false;
            _emulator.MouseButton(false);
        };
        ScreenImage.MouseDown += (_, e) =>
        {
            if (e.ChangedButton == MouseButton.Middle)
            {
                UnlockMouse();
                e.Handled = true;
            }
        };
        ScreenImage.LostMouseCapture += (_, _) =>
        {
            if (_mouseLocked && !ReferenceEquals(Mouse.Captured, ScreenImage))
                UnlockMouse();
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
    private void InitializeRawMouse()
    {
        IntPtr hwnd = new WindowInteropHelper(this).Handle;
        _windowSource = HwndSource.FromHwnd(hwnd);
        _windowSource?.AddHook(WindowProc);

        var devices = new[]
        {
            new RAWINPUTDEVICE
            {
                UsagePage = 0x01,             // HID generic desktop controls
                Usage = 0x02,                 // mouse
                Flags = 0,                    // foreground only; keep legacy buttons
                Target = hwnd,
            },
        };
        _rawMouseRegistered = RegisterRawInputDevices(
            devices, (uint)devices.Length, (uint)Marshal.SizeOf<RAWINPUTDEVICE>());
        if (_rawMouseRegistered)
            Log.Line($"input: raw relative mouse registered (packet={Marshal.SizeOf<RAWINPUT>()} bytes)");
        else
            Log.Line($"input: raw mouse registration failed ({Marshal.GetLastWin32Error()}); using legacy motion");
    }

    private void ShutdownRawMouse()
    {
        if (_rawMouseRegistered)
        {
            var devices = new[]
            {
                new RAWINPUTDEVICE
                {
                    UsagePage = 0x01,
                    Usage = 0x02,
                    Flags = RIDEV_REMOVE,
                    Target = IntPtr.Zero,
                },
            };
            RegisterRawInputDevices(
                devices, (uint)devices.Length, (uint)Marshal.SizeOf<RAWINPUTDEVICE>());
            _rawMouseRegistered = false;
        }
        _windowSource?.RemoveHook(WindowProc);
        _windowSource = null;
    }

    private IntPtr WindowProc(IntPtr hwnd, int message, IntPtr wParam,
                              IntPtr lParam, ref bool handled)
    {
        if (message != WM_INPUT || !_mouseLocked || !_emulator.IsRomLoaded)
            return IntPtr.Zero;

        uint size = (uint)Marshal.SizeOf<RAWINPUT>();
        RAWINPUT input = default;
        uint read = GetRawInputData(lParam, RID_INPUT, ref input, ref size,
                                    (uint)Marshal.SizeOf<RAWINPUTHEADER>());
        if (read == uint.MaxValue || input.Header.Type != RIM_TYPEMOUSE)
            return IntPtr.Zero;

        int dx = input.Mouse.LastX;
        int dy = input.Mouse.LastY;
        if ((input.Mouse.Flags & MOUSE_MOVE_ABSOLUTE) != 0)
        {
            // Absolute pointing devices report a normalized position instead
            // of counts. The Windows cursor has already resolved that position,
            // so turn it back into a relative delta from our lock point.
            if (!GetCursorPos(out POINT point)) return IntPtr.Zero;
            dx = point.X - _lockCx;
            dy = point.Y - _lockCy;
        }
        if (dx == 0 && dy == 0) return IntPtr.Zero;

        QueueMouseMotion(dx, dy, "raw");
        // Warping the legacy pointer does not manufacture Raw Input packets.
        // Keeping it centred prevents an invisible host cursor reaching another
        // monitor while the raw HID deltas continue without an artificial edge.
        SetCursorPos(_lockCx, _lockCy);
        return IntPtr.Zero;                  // WPF must still perform WM_INPUT cleanup
    }

    private void QueueMouseMotion(int dx, int dy, string source)
    {
        _pendingMouseDx += dx;
        _pendingMouseDy += dy;
        _capturedMouseDx += dx;
        _capturedMouseDy += dy;
        ++_capturedMotionPackets;
        if (_captureMotionLogged) return;
        _captureMotionLogged = true;
        Log.Line($"input: first {source} motion dx={dx} dy={dy} backend={_emulator.BackendName}");
    }

    private void FlushMouseMotion()
    {
        long dx = _pendingMouseDx;
        long dy = _pendingMouseDy;
        _pendingMouseDx = _pendingMouseDy = 0;
        if (!_mouseLocked || !_emulator.IsRomLoaded || (dx == 0 && dy == 0)) return;

        // A physically impossible multi-billion-count packet is the only way
        // these casts can split; retaining the remainder makes even that lossless.
        while (dx != 0 || dy != 0)
        {
            int partX = (int)Math.Clamp(dx, int.MinValue, int.MaxValue);
            int partY = (int)Math.Clamp(dy, int.MinValue, int.MaxValue);
            _emulator.MouseMove(partX, partY, _guestMouseDown);
            dx -= partX;
            dy -= partY;
        }
    }

    private void LockMouse()
    {
        if (_mouseLocked) return;
        if (!Mouse.Capture(ScreenImage, CaptureMode.Element))
        {
            Log.Line("input: pointer capture refused by WPF");
            return;
        }
        _mouseLocked = true;
        _guestMouseDown = false;
        _captureMotionLogged = false;
        _pendingMouseDx = _pendingMouseDy = 0;
        _capturedMotionPackets = 0;
        _capturedMouseDx = _capturedMouseDy = 0;
        ScreenImage.Cursor = Cursors.None;
        RecenterCursor();
        if (_keyHook != null) _keyHook.Enabled = true;   // Cmd(Win)+Q, Alt+Tab etc. now reach the Mac
        Title = _baseTitle + "   —   input captured: keys go to the Mac (middle-click to release)";
        Log.Line($"input: pointer captured backend={_emulator.BackendName}");
    }

    private void UnlockMouse()
    {
        if (!_mouseLocked) return;
        // Losing focus or middle-clicking while dragging must never strand the
        // Macintosh button in its active-low pressed state.
        FlushMouseMotion();
        if (_guestMouseDown && _emulator.IsRomLoaded)
            _emulator.MouseButton(false);
        _guestMouseDown = false;
        _mouseLocked = false;
        if (_keyHook != null) { _keyHook.Enabled = false; _keyHook.ReleaseAll(); }
        if (ScreenImage.IsMouseCaptured) ScreenImage.ReleaseMouseCapture();
        ScreenImage.Cursor = null;
        Title = _baseTitle;
        Log.Line($"input: pointer released packets={_capturedMotionPackets} "
                 + $"delta={_capturedMouseDx}/{_capturedMouseDy} "
                 + $"source={(_rawMouseRegistered ? "raw" : "legacy")}");
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
    [StructLayout(LayoutKind.Sequential)]
    private struct RAWINPUTDEVICE
    {
        public ushort UsagePage;
        public ushort Usage;
        public uint Flags;
        public IntPtr Target;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct RAWINPUTHEADER
    {
        public uint Type;
        public uint Size;
        public IntPtr Device;
        public IntPtr WParam;
    }
    [StructLayout(LayoutKind.Explicit, Size = 24)]
    private struct RAWMOUSE
    {
        [FieldOffset(0)] public ushort Flags;
        [FieldOffset(4)] public uint Buttons;
        [FieldOffset(8)] public uint RawButtons;
        [FieldOffset(12)] public int LastX;
        [FieldOffset(16)] public int LastY;
        [FieldOffset(20)] public uint ExtraInformation;
    }
    [StructLayout(LayoutKind.Sequential)]
    private struct RAWINPUT
    {
        public RAWINPUTHEADER Header;
        public RAWMOUSE Mouse;
    }
    private const int WM_INPUT = 0x00FF;
    private const uint RID_INPUT = 0x10000003;
    private const uint RIM_TYPEMOUSE = 0;
    private const ushort MOUSE_MOVE_ABSOLUTE = 0x0001;
    private const uint RIDEV_REMOVE = 0x00000001;
    [DllImport("user32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool RegisterRawInputDevices(
        [In] RAWINPUTDEVICE[] devices, uint count, uint size);
    [DllImport("user32.dll", SetLastError = true)]
    private static extern uint GetRawInputData(
        IntPtr rawInput, uint command, ref RAWINPUT data, ref uint size, uint headerSize);
    [DllImport("user32.dll")] private static extern bool SetCursorPos(int x, int y);
    [DllImport("user32.dll")] private static extern bool GetCursorPos(out POINT p);
    [DllImport("user32.dll")] private static extern bool GetWindowRect(IntPtr hWnd, out RECT r);

    // ---- machine lifecycle ----
    private void LoadRom(string path)
    {
        // The machines take distinct ROMs (Classic and IIfx: 512 KB, Quadra
        // family: 1 MB). Feeding the wrong one wedges the CPU into a white
        // screen with no diagnostics -- refuse it with directions instead.
        long romLen = new FileInfo(path).Length;
        bool looksQuadra = romLen >= 1024 * 1024;
        uint header = 0;
        if (romLen >= 4)
        {
            using FileStream stream = File.OpenRead(path);
            Span<byte> bytes = stackalloc byte[4];
            if (stream.Read(bytes) == 4)
                header = ((uint)bytes[0] << 24) | ((uint)bytes[1] << 16)
                         | ((uint)bytes[2] << 8) | bytes[3];
        }
        bool looksIifx = romLen == 512 * 1024 && header == 0x4147DD77u;
        bool rightModel = _settings.IsQuadra ? looksQuadra
                        : _settings.IsIifx ? looksIifx
                        : !looksQuadra && !looksIifx;
        if (!rightModel)
        {
            string detected = looksQuadra ? "a Quadra-family ROM"
                            : looksIifx ? "the Macintosh IIfx ROM (checksum 4147DD77)"
                            : "a Classic-family 512 KB ROM";
            string wanted = _settings.IsQuadra ? "Macintosh Quadra 650"
                          : _settings.IsIifx ? "Macintosh IIfx"
                          : "Macintosh Classic";
            string msg = $"This file looks like {detected}, but the current model is the {wanted}.\n\n"
                       + "Choose the matching machine under Machine > Model, then open its ROM there.";
            Log.Line($"ROM refused (wrong model): {path} ({romLen} bytes, model={_settings.Model})");
            MessageBox.Show(this, msg, "OpenMac", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        Log.Line($"load ROM: {path}  (RAM={_settings.ModelRamMB} MB, bootRomDisk={_settings.BootRomDisk}, "
                 + $"floppies {(_settings.WriteProtectFloppies ? "write-protected" : "writable")})");
        try
        {
            _emulator.LoadRom(path, _settings.ModelRamMB, _settings.BootRomDisk);
            // The machine's screen is only known once it exists -- the monitor
            // on its video port decides it. Match the framebuffer to it here,
            // or a display larger than the one this window was built for gets
            // written past the end of the bitmap on the very next frame.
            if (_bitmap is null || _bitmap.PixelWidth != _emulator.ScreenWidth ||
                _bitmap.PixelHeight != _emulator.ScreenHeight)
            {
                RebuildScreen();
                ApplyScale();
            }
            // Before any disk goes in: the tab is read at insertion.
            _emulator.WriteProtectFloppies = _settings.WriteProtectFloppies;
            // Only the Classic has an external floppy port in this frontend.
            // The internal-drive restore below is shared by every model.
            if (!_settings.IsQuadra && !_settings.IsIifx)
            {
                if (_settings.ExternalDrive) _emulator.SetExternalDrive(true);
                if (!string.IsNullOrEmpty(_settings.LastExternalFloppy) &&
                    File.Exists(_settings.LastExternalFloppy) &&
                    !_emulator.InsertExternalFloppy(_settings.LastExternalFloppy!))
                {
                    Log.Line($"floppy refused: {_settings.LastExternalFloppy} -- {_emulator.MediumNote(1)}");
                    _settings.LastExternalFloppy = null;
                }
            }
            // A remembered path the core now refuses is forgotten rather than
            // retried on every boot; the refusal is in the log.
            if (!string.IsNullOrEmpty(_settings.ModelLastFloppy) &&
                File.Exists(_settings.ModelLastFloppy) &&
                !_emulator.InsertFloppy(_settings.ModelLastFloppy!))
            {
                Log.Line($"floppy refused: {_settings.ModelLastFloppy} -- {_emulator.MediumNote(0)}");
                _settings.ModelLastFloppy = null;
            }
            if (!string.IsNullOrEmpty(_settings.ModelLastHardDisk) && File.Exists(_settings.ModelLastHardDisk))
                _emulator.AttachHardDisk(_settings.ModelLastHardDisk!);
            if (_settings.IsQuadra || _settings.IsIifx)
            {
                // Mount a remembered SCSI CD so it is present for the scan (the
                // Quadra) or as soon as the System is up (the IIfx installs its
                // own driver). A remembered attached-but-empty drive rides too.
                if (_settings.CdRomAttached) _emulator.SetCdRomAttached(true);
                if (!string.IsNullOrEmpty(_settings.LastCd) && File.Exists(_settings.LastCd) &&
                    !_emulator.InsertCd(_settings.LastCd!))
                {
                    Log.Line($"cd refused: {_settings.LastCd} -- {_emulator.CdMediumNote()}");
                    _settings.LastCd = null;
                }
            }
            if (!_settings.IsQuadra && !_settings.IsIifx)
            {
                if (_settings.CdRomAttached) _emulator.SetCdRomAttached(true);
                if (!string.IsNullOrEmpty(_settings.LastCd) && File.Exists(_settings.LastCd) &&
                    !_emulator.InsertCd(_settings.LastCd!))
                {
                    Log.Line($"cd refused: {_settings.LastCd} -- {_emulator.CdMediumNote()}");
                    _settings.LastCd = null;
                }
                if (_settings.Networking) _emulator.SetNetworking(true);
            }
            // The folder goes on the seat before the machine runs a frame, so
            // its driver is there for the ROM's startup bus scan to load --
            // which is the difference between the volume appearing on its own
            // and needing a restart. All three models have the seat.
            //
            // Two ways to want it: the drop box is on (use the chosen folder,
            // or make the default one), or a folder disk was simply left open
            // last session (restore exactly that, and create nothing).
            string? seatFolder = _settings.DropBox ? DropBoxFolder() : ChosenFolder();
            if (seatFolder is not null &&
                !_emulator.AttachFolderDisk(seatFolder, out string dbErr))
            {
                Log.Line($"folder disk refused: {seatFolder} -- {dbErr}");
                if (_settings.DropBox)
                {
                    MessageBox.Show(this,
                        "The drop box folder did not become a disk, so it is off for now."
                        + "\n\n" + dbErr,
                        "Drop Box", MessageBoxButton.OK, MessageBoxImage.Warning);
                    _settings.DropBox = false;
                }
                else
                {
                    _settings.LastFolderDisk = null;
                }
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
        // Every boot invalidates any pending Shift release from the last one.
        _bootShiftGen++;
        if (_settings.BootExtensionsOff) HoldBootShift();
        _settings.PushRecentRom(path);
        _settings.Save();
        BuildRecentMenu();
        BuildMonitorMenu();
        UpdateUi();
    }

    private void OpenRom_Click(object sender, RoutedEventArgs e)
    {
        if (FilePicker.Open(this, _settings, FilePicker.Rom, "Open Macintosh ROM",
                            "Macintosh ROM (*.rom;*.bin)|*.rom;*.bin|All files (*.*)|*.*",
                            _settings.ModelLastRom) is { } path)
            LoadRom(path);
    }

    private void IifxVideoRom_Click(object sender, RoutedEventArgs e)
    {
        if (!_settings.IsIifx) return;
        if (FilePicker.Open(this, _settings, FilePicker.VideoRom,
                            "Open Macintosh Display Card 8•24 GC ROM",
                            "8•24 GC ROM (*.bin;*.rom)|*.bin;*.rom|All files (*.*)|*.*",
                            _settings.VideoRomIifx) is not { } path)
            return;
        long bytes = new FileInfo(path).Length;
        if (bytes != 32 * 1024 && bytes != 64 * 1024)
        {
            MessageBox.Show(this,
                "An 8•24 GC declaration ROM must be 32 KiB or 64 KiB.",
                "OpenMac", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }
        _settings.VideoRomIifx = path;
        _settings.Save();
        if (_emulator is IifxEmulator iifx) iifx.VideoRomPath = path;
        if (_emulator.RomPath is { } rom) LoadRom(rom);
        UpdateUi();
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

    private void MemoryLarge_Click(object sender, RoutedEventArgs e)
    {
        int ram = int.Parse((string)((MenuItem)sender).Tag);
        if (_settings.IsIifx) _settings.RamMBIifx = ram;
        else _settings.RamMBQuadra = ram;
        _settings.Save();
        if (_emulator.IsRomLoaded && _emulator.RomPath is { } rom) LoadRom(rom);
        UpdateUi();
    }

    private void MemoryIifx_Click(object sender, RoutedEventArgs e)
    {
        _settings.RamMBIifx = int.Parse((string)((MenuItem)sender).Tag);
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

    private void BootExtensionsOff_Click(object sender, RoutedEventArgs e)
    {
        _settings.BootExtensionsOff = !_settings.BootExtensionsOff;
        _settings.Save();
        // Reboot immediately so the toggle takes effect now (mirrors the RAM menu).
        if (_emulator.IsRomLoaded && _emulator.RomPath is { } rom) LoadRom(rom);
        UpdateUi();
    }

    // ---- extensions-off boot: the virtual held Shift ----
    // System 6/7 samples Shift early in the boot (the "Extensions off" welcome);
    // holding it that long on the real keyboard trips Windows' sticky/filter-keys
    // accessibility hooks. So the hold happens inside the machine instead: one
    // Shift-down as the boot begins, one Shift-up 25 s later — past the check on
    // the slowest machine here (IIfx, 128 MB RAM test), before desktop typing.
    // The generation counter keeps a restart mid-hold from releasing the new
    // boot's Shift: only the newest hold owns the release.
    private int _bootShiftGen;

    private async void HoldBootShift()
    {
        int gen = ++_bootShiftGen;
        _emulator.KeyEvent(0x38 /* ADB Shift */, true);
        Log.Line("extensions-off boot: virtual Shift held");
        await Task.Delay(TimeSpan.FromSeconds(25));
        if (gen != _bootShiftGen || !_emulator.IsRomLoaded) return;
        _emulator.KeyEvent(0x38, false);
        Log.Line("extensions-off boot: virtual Shift released");
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

    // A disk waiting for the drive to empty. Picking a new disk while one is
    // already in there has to take the old one OUT first -- the guest is holding
    // that volume mounted, and swapping the bytes underneath it loses everything
    // it wrote and leaves it reading a catalog that now belongs to another disk.
    // The eject is carried out by the machine a frame later, so the new disk
    // waits here until the drive reports empty (which is also when the outgoing
    // disk is written back to its file).
    private string? _floppyWaiting;

    private void InsertFloppy_Click(object sender, RoutedEventArgs e)
    {
        if (FilePicker.Open(this, _settings, FilePicker.Floppy, "Insert Floppy",
                            DiskImageFilter, _settings.ModelLastFloppy) is { } path)
        {
            if (_emulator.FloppyPath is not null)
            {
                _floppyWaiting = path;
                _emulator.EjectFloppy();
                Log.Line($"[disk] taking out {Path.GetFileName(_emulator.FloppyPath)} "
                         + $"before {Path.GetFileName(path)} goes in");
                UpdateUi();
                return;
            }
            if (TryInsert(path, _emulator.InsertFloppy, 0))
            {
                _settings.ModelLastFloppy = path;
                _settings.Save();
            }
            UpdateUi();
        }
    }

    // Called from the frame tick once the drive state has changed. The outgoing
    // disk has been saved by then, so the one that was waiting can go in.
    private void SeatWaitingFloppy()
    {
        if (_floppyWaiting is not { } path || _emulator.FloppyPath is not null) return;
        _floppyWaiting = null;
        if (TryInsert(path, _emulator.InsertFloppy, 0))
        {
            _settings.ModelLastFloppy = path;
            _settings.Save();
        }
    }

    private void EjectFloppy_Click(object sender, RoutedEventArgs e)
    {
        _emulator.EjectFloppy();
        _settings.ModelLastFloppy = null;
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
                            _settings.LastExternalFloppy ?? _settings.ModelLastFloppy) is { } path)
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
                            _settings.ModelLastHardDisk) is { } path)
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
                "CD image (*.iso;*.cdr;*.toast;*.dmg;*.bin;*.cue;*.mdf;*.nrg;*.img;*.dsk)|*.iso;*.cdr;*.toast;*.dmg;*.bin;*.cue;*.mdf;*.nrg;*.img;*.dsk|"
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

    // ---- drop box ----
    //
    // The drop box IS the folder disk, kept in place for the session instead of
    // being opened and closed around a task. There is one second-disk seat, so
    // this shares it with "Open Host Folder as Disk" and the transfer disk; the
    // occupant is named rather than clobbered.

    /// <summary>The folder the user has chosen, or null. No side effects: a
    /// folder is not created for somebody who never asked for one.</summary>
    private string? ChosenFolder() =>
        !string.IsNullOrEmpty(_settings.LastFolderDisk) &&
        Directory.Exists(_settings.LastFolderDisk) ? _settings.LastFolderDisk : null;

    /// <summary>The folder the drop box uses: the one chosen last, else a
    /// plainly named folder under Documents, made on demand.</summary>
    private string? DropBoxFolder() => ChosenFolder() ?? DropBoxSeat.EnsureDefaultFolder();

    private void DropBoxEnabled_Click(object sender, RoutedEventArgs e)
    {
        bool want = DropBoxEnabledItem.IsChecked;
        if (!want)
        {
            _emulator.DetachFolderDisk();
            _settings.DropBox = false;
            _settings.Save();
            UpdateUi();
            return;
        }
        // Remember the choice even with no machine yet; it takes effect on load.
        if (!_emulator.IsRomLoaded)
        {
            _settings.DropBox = true;
            _settings.Save();
            UpdateUi();
            return;
        }
        if (_emulator.TransferDiskLabel is { } tdl)
        {
            MessageBox.Show(this,
                $"The transfer disk “{tdl}” is using the second SCSI seat. Restart the " +
                "machine to release it, then turn the drop box on.",
                "Drop Box", MessageBoxButton.OK, MessageBoxImage.Information);
            DropBoxEnabledItem.IsChecked = false;
            return;
        }
        string? folder = DropBoxFolder();
        if (folder is null)
        {
            MessageBox.Show(this,
                "The drop box folder could not be created. Choose one with " +
                "File ▸ Drop Box ▸ Choose Folder…",
                "Drop Box", MessageBoxButton.OK, MessageBoxImage.Warning);
            DropBoxEnabledItem.IsChecked = false;
            return;
        }
        AttachDropBox(folder, offerRestart: true);
    }

    /// <summary>Put the folder on the seat and remember it. When the seat's
    /// driver was not loaded by this Mac's startup scan there is nothing to read
    /// the disk with, so offer the restart that loads one.</summary>
    private void AttachDropBox(string folder, bool offerRestart)
    {
        if (!_emulator.AttachFolderDisk(folder, out string error))
        {
            Log.Line($"drop box refused: {folder} -- {error}");
            MessageBox.Show(this, "The drop box folder did not become a disk.\n\n" + error,
                            "Drop Box", MessageBoxButton.OK, MessageBoxImage.Warning);
            DropBoxEnabledItem.IsChecked = false;
            return;
        }
        _settings.DropBox = true;
        _settings.LastFolderDisk = folder;
        _settings.Save();
        UpdateUi();
        if (offerRestart && !_emulator.TransferDiskResident &&
            _emulator.RomPath is { } rom)
        {
            var r = MessageBox.Show(this,
                "The drop box is on the bus, but its driver is loaded by the Mac's " +
                "startup scan — and this Mac started without one.\n\nRestart now so it " +
                "appears on the desktop?",
                "Drop Box", MessageBoxButton.YesNo, MessageBoxImage.Question);
            if (r == MessageBoxResult.Yes) LoadRom(rom);
        }
    }

    private void DropBoxChooseFolder_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new Microsoft.Win32.OpenFolderDialog
        {
            Title = "Choose the Drop Box Folder",
        };
        string? current = DropBoxFolder();
        if (current is not null && Directory.Exists(current)) dlg.InitialDirectory = current;
        if (dlg.ShowDialog(this) != true) return;

        _settings.LastFolderDisk = dlg.FolderName;
        _settings.Save();
        if (!_emulator.IsRomLoaded || !_settings.DropBox) { UpdateUi(); return; }
        if (_emulator.FolderDiskPath is null)
        {
            // Nothing on the seat yet: a plain attach is safe.
            AttachDropBox(dlg.FolderName, offerRestart: true);
            return;
        }
        // A volume is already mounted from the old folder, so the move has to go
        // through the swap. Putting the new folder's bytes on the seat while the
        // Mac still holds the old volume would leave it reading the old catalog
        // against the new disk's blocks -- and the outgoing folder would never
        // get back what the guest saved into it.
        if (!_emulator.RetargetFolderDisk(dlg.FolderName, out string error))
        {
            Log.Line($"drop box move refused: {dlg.FolderName} -- {error}");
            MessageBox.Show(this, "The drop box did not move.\n\n" + error,
                            "Drop Box", MessageBoxButton.OK, MessageBoxImage.Warning);
        }
        UpdateUi();
    }

    private void DropBoxOpenFolder_Click(object sender, RoutedEventArgs e)
    {
        string? folder = DropBoxFolder();
        if (folder is null) return;
        try
        {
            System.Diagnostics.Process.Start(
                new System.Diagnostics.ProcessStartInfo(folder) { UseShellExecute = true });
        }
        catch (Exception ex) { Log.Line("could not open the drop box folder: " + ex.Message); }
    }

    private void DropBoxRepublish_Click(object sender, RoutedEventArgs e)
    {
        if (!_emulator.RepublishFolderDisk(null, out string error))
            MessageBox.Show(this, "Nothing to refresh.\n\n" + error,
                            "Drop Box", MessageBoxButton.OK, MessageBoxImage.Information);
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
    private static readonly string[] CdOnlyExtensions = { ".iso", ".toast", ".cue", ".cdr", ".mdf", ".nrg" };

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

        // With a drop box on the desktop, a loose Mac file simply goes in it --
        // no disk of its own, nothing to throw in the Trash afterwards. Without
        // one, archives and encodings ride in on a transfer floppy instead.
        if (TransferExtensions.Contains(ext) && DropBoxDrop(path)) return;
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
                else _settings.ModelLastFloppy = path;
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
            // A disk image dropped while one is ALREADY the startup disk is
            // almost never meant to replace it -- it is a volume somebody
            // downloaded to get one application out of. Replacing the startup
            // disk swaps the machine's whole world for a 3 MB utility volume,
            // and the swap has to persist the outgoing disk, which is a write
            // nobody asked for. Put it on the second seat instead, read-only,
            // where it mounts beside the startup disk the way a second drive
            // on the bus would.
            if (_emulator.HardDiskAttached && SecondSeatDrop(path)) return;
            Log.Line($"drop: {Path.GetFileName(path)} -> hard disk");
            AttachHardDisk(path);
            return;
        }

        // Nothing recognised it as media. With a drop box open that is not a
        // failure -- it is a file, and a file is what the drop box is for.
        if (DropBoxDrop(path)) return;

        string why = triedFloppy ? _emulator.MediumNote(0)
                                 : "not floppy media, a CD image, or a hard-disk image";
        Log.Line($"drop refused: {path} -- {why}");
        MessageBox.Show(this,
            Path.GetFileName(path) + " did not go in any drive.\n\n" + why +
            "\n\nTo copy it into the Mac as a file, turn on File ▸ Drop Box ▸ " +
            "Keep Drop Box on the Desktop and drop it again.",
            "OpenMac", MessageBoxButton.OK, MessageBoxImage.Warning);
    }

    /// <summary>Mount a dropped disk image on the second SCSI seat, read-only,
    /// beside the startup disk. False when the seat is not free, which leaves
    /// the caller to fall back on replacing the hard disk.</summary>
    private bool SecondSeatDrop(string path)
    {
        string name = Path.GetFileName(path);
        if (_emulator.FolderDiskPath is { } fd)
        {
            var r = MessageBox.Show(this,
                $"“{name}” can mount beside your startup disk, but the drop box " +
                $"“{Path.GetFileName(fd.TrimEnd('\\', '/'))}” is using that seat.\n\n" +
                "Close the drop box and mount this disk instead?",
                "Second Disk", MessageBoxButton.YesNo, MessageBoxImage.Question);
            if (r != MessageBoxResult.Yes) return true;   // handled: they said no
            _emulator.DetachFolderDisk();
            _settings.DropBox = false;
            _settings.Save();
        }
        if (!_emulator.AttachSecondDisk(path, out string error))
        {
            Log.Line($"second disk refused: {path} -- {error}");
            return false;         // let the caller try the old route
        }
        Log.Line($"drop: {name} -> second disk (read-only)");
        UpdateUi();
        if (!_emulator.TransferDiskResident && _emulator.RomPath is { } rom)
        {
            var r = MessageBox.Show(this,
                $"“{name}” is on the SCSI bus beside your startup disk, but the " +
                "driver for that seat is loaded by the Mac's startup scan — and " +
                "this Mac started without one.\n\nRestart now so it appears?" +
                "\n\n(Your startup disk is not being replaced.)",
                "Second Disk", MessageBoxButton.YesNo, MessageBoxImage.Question);
            if (r == MessageBoxResult.Yes) LoadRom(rom);
        }
        return true;
    }

    /// <summary>Put a loose file in the drop box and republish, so it turns up
    /// on the Mac's desktop. False when no drop box is attached, which leaves
    /// the caller to try the older routes.</summary>
    private bool DropBoxDrop(string path)
    {
        if (_emulator.FolderDiskPath is null) return false;
        if (!_emulator.RepublishFolderDisk(path, out string error))
        {
            Log.Line($"drop box refused {path}: {error}");
            MessageBox.Show(this,
                Path.GetFileName(path) + " did not go in the drop box.\n\n" + error,
                "Drop Box", MessageBoxButton.OK, MessageBoxImage.Warning);
            return true;      // handled: told the user what happened
        }
        Log.Line($"drop: {Path.GetFileName(path)} -> drop box");
        UpdateUi();
        return true;
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
            var r = MessageBox.Show(this,
                Path.GetFileName(path) + " needs the second SCSI disk, but the folder disk " +
                $"“{Path.GetFileName(fd.TrimEnd('\\', '/'))}” is using it.\n\n" +
                "Close the folder disk and put this file on that seat instead? " +
                "(The folder's changes are written back first. Alternatively, copy the " +
                "file into that folder and restart; it rides in with the folder.)",
                "Transfer Disk", MessageBoxButton.YesNo, MessageBoxImage.Question);
            if (r != MessageBoxResult.Yes) return true;   // handled: they said no
            _emulator.DetachFolderDisk();
            _settings.DropBox = false;
            _settings.LastFolderDisk = null;
            _settings.Save();
        }
        // A transfer disk already on the seat is swapped out by the backend:
        // one the Mac has mounted is flushed and unmounted first (the guest
        // must forget its catalog before the blocks change under it), and one
        // it never got to see is simply replaced. Only a volume with a file
        // still open in the Mac refuses, and says so.
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
                return true;   // raw 2352/2448 sync pattern (MODE1 or MODE2)
            // Disk Utility's UDIF (.dmg): a 'koly' trailer in the last 512 bytes.
            // The CD drive is the one place that decodes it (chunked, zlib).
            if (fs.Length >= 512)
            {
                byte[] koly = new byte[4];
                fs.Seek(fs.Length - 512, SeekOrigin.Begin);
                if (fs.Read(koly, 0, 4) == 4 &&
                    koly[0] == 'k' && koly[1] == 'o' && koly[2] == 'l' && koly[3] == 'y')
                    return true;
            }
            // An ISO 9660 / High Sierra descriptor at sector 16 -- in plain 2048
            // framing, or behind the 8-byte subheader of a 2336-byte MODE2 image.
            foreach (var (stride, dataOff) in new[] { (2048L, 0L), (2336L, 8L) })
            {
                if (fs.Length < 17L * stride) continue;
                byte[] pvd = new byte[16];
                fs.Seek(16L * stride + dataOff, SeekOrigin.Begin);
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
            "OpenMac\nA from-scratch Macintosh emulator for Windows.\n\n"
            + "Macintosh Classic · Macintosh IIfx · Quadra 650\n"
            + "68000, 68030 and 68040 machines with model-specific video, storage, input, audio and PRAM.",
            "About OpenMac", MessageBoxButton.OK, MessageBoxImage.Information);

    // Capture what the machine is doing right now, next to the app log. Taken
    // while the guest is misbehaving this is the whole picture: where the CPU
    // is looping, and which device it is waiting on. It reads model state
    // only, so it is safe at any moment -- including while the guest is
    // wedged, which is exactly when it is wanted.
    private void CaptureDiagnostics_Click(object sender, RoutedEventArgs e)
    {
        string report;
        try { report = _emulator.DiagnosticReport(); }
        catch (Exception ex) { report = "diagnostic capture failed: " + ex; }

        if (string.IsNullOrEmpty(report))
        {
            MessageBox.Show(this,
                _emulator.IsRomLoaded
                    ? "This machine does not provide a diagnostic snapshot yet."
                    : "Load a ROM first — there is no machine to report on.",
                "Capture Diagnostics", MessageBoxButton.OK, MessageBoxImage.Information);
            return;
        }

        string dir = System.IO.Path.GetDirectoryName(Log.Path)!;
        string file = System.IO.Path.Combine(
            dir, "openmac-diagnostics-" + DateTime.Now.ToString("yyyyMMdd-HHmmss") + ".txt");
        try
        {
            Directory.CreateDirectory(dir);
            // The log rides along: the snapshot says what the machine is
            // doing, the log says how it got there.
            var text = new StringBuilder();
            text.AppendLine(report);
            text.AppendLine();
            text.AppendLine("---- app log (tail) ----");
            text.AppendLine(ReadLogTail(200));
            File.WriteAllText(file, text.ToString());
        }
        catch (Exception ex)
        {
            MessageBox.Show(this, "Could not write the report:\n" + ex.Message,
                "Capture Diagnostics", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        Log.Line("diagnostics captured: " + file);
        if (MessageBox.Show(this, "Saved:\n" + file + "\n\nShow it in Explorer?",
                "Capture Diagnostics", MessageBoxButton.YesNo,
                MessageBoxImage.Information) == MessageBoxResult.Yes)
        {
            try
            {
                System.Diagnostics.Process.Start(
                    new System.Diagnostics.ProcessStartInfo(
                        "explorer.exe", "/select,\"" + file + "\"")
                    { UseShellExecute = true });
            }
            catch (Exception ex) { Log.Line("open diagnostics folder failed: " + ex.Message); }
        }
    }

    private static string ReadLogTail(int lines)
    {
        try
        {
            if (!File.Exists(Log.Path)) return "(no log)";
            // Share the handle -- the logger keeps the file open.
            using var fs = new FileStream(Log.Path, FileMode.Open, FileAccess.Read,
                                          FileShare.ReadWrite);
            using var sr = new StreamReader(fs);
            var tail = new Queue<string>(lines);
            string? line;
            while ((line = sr.ReadLine()) != null)
            {
                if (tail.Count == lines) tail.Dequeue();
                tail.Enqueue(line);
            }
            return string.Join(Environment.NewLine, tail);
        }
        catch (Exception ex) { return "(log unavailable: " + ex.Message + ")"; }
    }

    private void Exit_Click(object sender, RoutedEventArgs e) => Close();

    // ---- ui state ----
    private void UpdateUi()
    {
        Overlay.Visibility = _emulator.IsRomLoaded ? Visibility.Collapsed : Visibility.Visible;

        StatusState.Text = _emulator.IsRomLoaded ? "Running" : "Stopped";
        string machine = _emulator.IsRomLoaded
            ? (_settings.IsQuadra ? "Quadra 650  •  "
               : _settings.IsIifx ? "Macintosh IIfx  •  " : "")
              + $"{_settings.ModelRamMB} MB"
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

        bool q = _settings.IsQuadra;
        bool fx = _settings.IsIifx;
        bool classic = !q && !fx;
        ModelClassicItem.IsChecked = classic;
        ModelIifxItem.IsChecked = fx;
        ModelQuadraItem.IsChecked = q;
        IifxVideoRomItem.Visibility = fx ? Visibility.Visible : Visibility.Collapsed;
        IifxVideoRomItem.IsEnabled = fx;
        IifxVideoRomItem.Header = fx && !string.IsNullOrEmpty(_settings.VideoRomIifx)
            ? $"8•24 GC Card ROM…  ({Path.GetFileName(_settings.VideoRomIifx)})"
            : "8•24 GC Card ROM…";
        Mem1Item.IsChecked = classic && _settings.RamMB == 1;
        Mem2Item.IsChecked = classic && _settings.RamMB == 2;
        Mem4Item.IsChecked = classic && _settings.RamMB == 4;
        Mem1Item.IsEnabled = Mem2Item.IsEnabled = Mem4Item.IsEnabled = classic;
        Mem1Item.Visibility = Mem2Item.Visibility = Mem4Item.Visibility =
            classic ? Visibility.Visible : Visibility.Collapsed;
        MemFx4Item.Visibility = fx ? Visibility.Visible : Visibility.Collapsed;
        MemFx4Item.IsEnabled = fx;
        MemFx4Item.IsChecked = fx && _settings.RamMBIifx == 4;
        // Each of the IIfx's two four-SIMM banks is empty or holds four equal
        // 1, 4, or 16 MB 64-pin SIMMs. The mixed-bank totals are therefore 20,
        // 68 and 80 MB; values such as 12, 48 and 96 MB are not real layouts.
        var fxOnlyMems = new[] { MemFx20Item, MemFx68Item, MemFx80Item };
        foreach (var item in fxOnlyMems)
        {
            item.Visibility = fx ? Visibility.Visible : Visibility.Collapsed;
            item.IsEnabled = fx;
            item.IsChecked = fx && _settings.RamMBIifx == int.Parse((string)item.Tag);
        }
        var largeMems = new[] { MemQ8Item, MemQ16Item, MemQ32Item, MemQ64Item, MemQ128Item };
        foreach (var item in largeMems)
        {
            item.Visibility = q || fx ? Visibility.Visible : Visibility.Collapsed;
            item.IsEnabled = q || fx;
            item.IsChecked = (q || fx) && _settings.ModelRamMB == int.Parse((string)item.Tag);
        }
        MemQ136Item.Visibility = q ? Visibility.Visible : Visibility.Collapsed;
        MemQ136Item.IsEnabled = q;
        MemQ136Item.IsChecked = q && _settings.RamMBQuadra == 136;
        MonitorMenu.IsEnabled = q;
        BootRomDiskItem.IsChecked = _settings.BootRomDisk;
        BootRomDiskItem.IsEnabled = classic;
        BootExtensionsOffItem.IsChecked = _settings.BootExtensionsOff;
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
        ExternalDriveItem.IsEnabled = classic;
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
        NetworkingItem.IsEnabled = classic;
        CdDriveItem.IsEnabled = !fx;

        DropBoxEnabledItem.IsChecked = _settings.DropBox;
        DropBoxEnabledItem.IsEnabled = true;
        DropBoxFolderItem.IsEnabled = true;
        // Name the folder in the menu: a drop box whose folder you cannot see
        // is one you have to go looking for.
        string? dbFolder = ChosenFolder() ?? (_settings.DropBox ? DropBoxSeat.DefaultFolder : null);
        DropBoxFolderItem.Header = dbFolder is null
            ? "Choose Folder…"
            : $"Choose Folder…   ({Path.GetFileName(dbFolder.TrimEnd('\\', '/'))})";
        DropBoxOpenItem.IsEnabled = dbFolder is not null;
        DropBoxRepublishItem.IsEnabled =
            _emulator.FolderDiskPath is not null && !_emulator.RepublishPending;
    }
}
