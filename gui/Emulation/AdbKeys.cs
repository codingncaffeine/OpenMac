using System.Windows.Input;

namespace OpenMac.Gui.Emulation;

/// <summary>WPF Key -> Apple ADB keycode. Partial; extend as the core needs.</summary>
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
        Key.Left => 0x7B, Key.Right => 0x7C, Key.Down => 0x7D, Key.Up => 0x7E,
        Key.LeftShift or Key.RightShift => 0x38, Key.CapsLock => 0x39,
        Key.LeftCtrl or Key.RightCtrl => 0x3B, Key.LeftAlt or Key.RightAlt => 0x3A,
        Key.LWin or Key.RWin => 0x37,
        _ => -1,
    };
}
