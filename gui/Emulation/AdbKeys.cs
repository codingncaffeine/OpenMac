using System.Windows.Input;

namespace OpenMac.Gui.Emulation;

/// <summary>WPF Key -> Apple ADB keycode. Partial; extend as the core needs.
/// These are the RAW codes the keyboard puts on the ADB bus, which the System's
/// KMAP turns into the virtual key codes applications see -- the two differ for
/// Control (raw $36, virtual $3B) and the arrows (raw $3B-$3E, virtual $7B-$7E).
/// Sending the virtual codes pressed the Extended keyboard's raw $7B/$7D/$7E,
/// its right-hand Shift/Option/Control, so the arrows acted as modifiers.</summary>
internal static class AdbKeys
{
    public static int Map(Key k) => k switch
    {
        Key.A => 0x00, Key.S => 0x01, Key.D => 0x02, Key.F => 0x03, Key.H => 0x04,
        Key.G => 0x05, Key.Z => 0x06, Key.X => 0x07, Key.C => 0x08, Key.V => 0x09,
        Key.B => 0x0B, Key.Q => 0x0C, Key.W => 0x0D, Key.E => 0x0E, Key.R => 0x0F,
        Key.Y => 0x10, Key.T => 0x11, Key.O => 0x1F, Key.U => 0x20, Key.I => 0x22,
        Key.P => 0x23, Key.L => 0x25, Key.J => 0x26, Key.K => 0x28, Key.N => 0x2D, Key.M => 0x2E,
        Key.D1 => 0x12, Key.D2 => 0x13, Key.D3 => 0x14, Key.D4 => 0x15, Key.D5 => 0x17,
        Key.D6 => 0x16, Key.D7 => 0x1A, Key.D8 => 0x1C, Key.D9 => 0x19, Key.D0 => 0x1D,
        Key.Return => 0x24, Key.Tab => 0x30, Key.Space => 0x31, Key.Back => 0x33, Key.Escape => 0x35,
        Key.OemMinus => 0x1B, Key.OemPlus => 0x18, Key.OemComma => 0x2B, Key.OemPeriod => 0x2F,
        Key.OemQuestion => 0x2C, Key.OemSemicolon => 0x29, Key.OemQuotes => 0x27,
        Key.OemOpenBrackets => 0x21, Key.OemCloseBrackets => 0x1E, Key.OemBackslash => 0x2A,
        Key.OemTilde => 0x32, Key.Delete => 0x75,
        Key.Left => 0x3B, Key.Right => 0x3C, Key.Down => 0x3D, Key.Up => 0x3E,
        Key.LeftShift or Key.RightShift => 0x38, Key.CapsLock => 0x39,
        Key.LeftCtrl or Key.RightCtrl => 0x36, Key.LeftAlt or Key.RightAlt => 0x3A,
        Key.LWin or Key.RWin => 0x37,
        // Keypad (the emulated keyboard reports itself as an extended ADB
        // keyboard, so the full keypad is fair game; 0x5A is unused on Apple)
        Key.NumPad0 => 0x52, Key.NumPad1 => 0x53, Key.NumPad2 => 0x54,
        Key.NumPad3 => 0x55, Key.NumPad4 => 0x56, Key.NumPad5 => 0x57,
        Key.NumPad6 => 0x58, Key.NumPad7 => 0x59, Key.NumPad8 => 0x5B,
        Key.NumPad9 => 0x5C, Key.Decimal => 0x41, Key.Multiply => 0x43,
        Key.Add => 0x45, Key.Divide => 0x4B, Key.Subtract => 0x4E,
        Key.NumLock => 0x47,   // Clear
        // Function and navigation keys (F11 is the host's fullscreen toggle)
        Key.F1 => 0x7A, Key.F2 => 0x78, Key.F3 => 0x63, Key.F4 => 0x76,
        Key.F5 => 0x60, Key.F6 => 0x61, Key.F7 => 0x62, Key.F8 => 0x64,
        Key.F9 => 0x65, Key.F10 => 0x6D, Key.F12 => 0x6F,
        Key.Home => 0x73, Key.End => 0x77, Key.PageUp => 0x74, Key.PageDown => 0x79,
        Key.Insert => 0x72,    // Help
        _ => -1,
    };
}
