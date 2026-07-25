using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Windows.Input;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// Low-level keyboard hook for captured-input mode. While enabled, every key the
/// ADB map knows goes to the Mac and is swallowed before Windows can act on it --
/// that is the only way combos like Cmd(Win)+Q, Alt+Tab or Win+Tab reach the guest
/// instead of the shell. Disabled, it passes everything through untouched.
/// Ctrl+Alt+Del is reserved by Windows and never arrives here.
/// </summary>
internal sealed class KeyboardHook : IDisposable
{
    private const int WH_KEYBOARD_LL = 13;
    private const int WM_KEYDOWN = 0x0100, WM_KEYUP = 0x0101;
    private const int WM_SYSKEYDOWN = 0x0104, WM_SYSKEYUP = 0x0105;

    private delegate IntPtr HookProc(int nCode, IntPtr wParam, IntPtr lParam);

    [DllImport("user32.dll", SetLastError = true)]
    private static extern IntPtr SetWindowsHookExW(int idHook, HookProc lpfn, IntPtr hMod, uint dwThreadId);
    [DllImport("user32.dll", SetLastError = true)]
    private static extern bool UnhookWindowsHookEx(IntPtr hhk);
    [DllImport("user32.dll")]
    private static extern IntPtr CallNextHookEx(IntPtr hhk, int nCode, IntPtr wParam, IntPtr lParam);

    [StructLayout(LayoutKind.Sequential)]
    private struct KBDLLHOOKSTRUCT
    {
        public uint vkCode;
        public uint scanCode;
        public uint flags;
        public uint time;
        public IntPtr dwExtraInfo;
    }

    private readonly HookProc _proc;          // strong ref: the GC must never collect it
    private readonly Action<int, bool> _sendKey;
    private readonly Action _toggleFullscreen;
    private readonly HashSet<int> _down = new();
    private IntPtr _hook = IntPtr.Zero;

    /// <summary>Capture is on: forward + swallow. Off: pure pass-through.</summary>
    public bool Enabled { get; set; }

    public KeyboardHook(Action<int, bool> sendKey, Action toggleFullscreen)
    {
        _sendKey = sendKey;
        _toggleFullscreen = toggleFullscreen;
        _proc = Callback;
        _hook = SetWindowsHookExW(WH_KEYBOARD_LL, _proc, IntPtr.Zero, 0);
    }

    private IntPtr Callback(int nCode, IntPtr wParam, IntPtr lParam)
    {
        if (nCode < 0 || !Enabled)
            return CallNextHookEx(_hook, nCode, wParam, lParam);

        var info = Marshal.PtrToStructure<KBDLLHOOKSTRUCT>(lParam);
        int msg = (int)wParam;
        bool downMsg = msg == WM_KEYDOWN || msg == WM_SYSKEYDOWN;
        bool upMsg = msg == WM_KEYUP || msg == WM_SYSKEYUP;
        if (!downMsg && !upMsg)
            return CallNextHookEx(_hook, nCode, wParam, lParam);

        Key key = KeyInterop.KeyFromVirtualKey((int)info.vkCode);

        // F11 stays the host's fullscreen toggle even while captured, or there
        // would be no keyboard way back out of fullscreen capture.
        if (key == Key.F11)
        {
            if (downMsg) _toggleFullscreen();
            return (IntPtr)1;
        }

        int code = AdbKeys.Map(key);
        if (code < 0)
            return CallNextHookEx(_hook, nCode, wParam, lParam);

        if (downMsg)
        {
            if (_down.Add(code)) _sendKey(code, true);   // auto-repeat stays host-side
        }
        else
        {
            _down.Remove(code);
            _sendKey(code, false);
        }
        return (IntPtr)1;                                // Windows never sees it
    }

    /// <summary>Lift every key still held, so releasing capture (or losing focus
    /// mid-combo) cannot leave the Mac with a stuck modifier.</summary>
    public void ReleaseAll()
    {
        foreach (int code in _down) _sendKey(code, false);
        _down.Clear();
    }

    public void Dispose()
    {
        if (_hook != IntPtr.Zero) { UnhookWindowsHookEx(_hook); _hook = IntPtr.Zero; }
    }
}
