using System.Runtime.InteropServices;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// Minimal streaming audio sink over Win32 waveOut — no external dependency.
/// The emulated Mac produces 8-bit unsigned mono PCM at ~22254 Hz; each frame's
/// drained samples are queued into a small pool of buffers. If the pool is full
/// (host can't keep up) the excess is dropped rather than blocking the UI thread.
/// </summary>
internal sealed class WaveAudio : IDisposable
{
    [StructLayout(LayoutKind.Sequential)]
    private struct WAVEFORMATEX
    {
        public ushort wFormatTag, nChannels;
        public uint nSamplesPerSec, nAvgBytesPerSec;
        public ushort nBlockAlign, wBitsPerSample, cbSize;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct WAVEHDR
    {
        public IntPtr lpData;
        public uint dwBufferLength, dwBytesRecorded;
        public IntPtr dwUser;
        public uint dwFlags, dwLoops;
        public IntPtr lpNext, reserved;
    }

    [DllImport("winmm.dll")] private static extern int waveOutOpen(out IntPtr h, uint dev, in WAVEFORMATEX fmt, IntPtr cb, IntPtr inst, uint flags);
    [DllImport("winmm.dll")] private static extern int waveOutClose(IntPtr h);
    [DllImport("winmm.dll")] private static extern int waveOutPrepareHeader(IntPtr h, IntPtr hdr, int size);
    [DllImport("winmm.dll")] private static extern int waveOutUnprepareHeader(IntPtr h, IntPtr hdr, int size);
    [DllImport("winmm.dll")] private static extern int waveOutWrite(IntPtr h, IntPtr hdr, int size);
    [DllImport("winmm.dll")] private static extern int waveOutReset(IntPtr h);

    private const uint WHDR_DONE = 0x01;
    private const uint WAVE_MAPPER = 0xFFFFFFFF;
    private const int NBUF = 8, BUFSZ = 4096;
    private static readonly int HdrSize = Marshal.SizeOf<WAVEHDR>();

    private IntPtr _h;
    private readonly IntPtr[] _hdr = new IntPtr[NBUF];
    private readonly IntPtr[] _data = new IntPtr[NBUF];
    private readonly bool[] _used = new bool[NBUF];
    private int _next;

    public WaveAudio(int sampleRate)
    {
        var fmt = new WAVEFORMATEX
        {
            wFormatTag = 1,                 // WAVE_FORMAT_PCM
            nChannels = 1,
            nSamplesPerSec = (uint)sampleRate,
            wBitsPerSample = 8,
            nBlockAlign = 1,
            nAvgBytesPerSec = (uint)sampleRate,
            cbSize = 0
        };
        if (waveOutOpen(out _h, WAVE_MAPPER, in fmt, IntPtr.Zero, IntPtr.Zero, 0) != 0)
        {
            _h = IntPtr.Zero;               // no audio device — stay silent, never throw
            return;
        }
        for (int i = 0; i < NBUF; i++)
        {
            _data[i] = Marshal.AllocHGlobal(BUFSZ);
            _hdr[i] = Marshal.AllocHGlobal(HdrSize);
            Marshal.StructureToPtr(new WAVEHDR { lpData = _data[i], dwBufferLength = BUFSZ }, _hdr[i], false);
            waveOutPrepareHeader(_h, _hdr[i], HdrSize);
        }
    }

    public bool Ok => _h != IntPtr.Zero;

    /// <summary>Queue up to `count` samples; drops overflow instead of blocking.</summary>
    public void Queue(byte[] samples, int count)
    {
        if (_h == IntPtr.Zero || count <= 0) return;
        int offset = 0;
        while (offset < count)
        {
            int slot = FindFree();
            if (slot < 0) return;           // pool full — drop the remainder
            int n = Math.Min(count - offset, BUFSZ);
            Marshal.Copy(samples, offset, _data[slot], n);
            var h = Marshal.PtrToStructure<WAVEHDR>(_hdr[slot]);
            h.dwBufferLength = (uint)n;
            h.dwFlags &= ~WHDR_DONE;
            Marshal.StructureToPtr(h, _hdr[slot], false);
            waveOutWrite(_h, _hdr[slot], HdrSize);
            _used[slot] = true;
            offset += n;
        }
    }

    private int FindFree()
    {
        for (int t = 0; t < NBUF; t++)
        {
            int i = _next;
            _next = (i + 1) % NBUF;
            if (!_used[i]) return i;
            var h = Marshal.PtrToStructure<WAVEHDR>(_hdr[i]);
            if ((h.dwFlags & WHDR_DONE) != 0) { _used[i] = false; return i; }
        }
        return -1;
    }

    public void Dispose()
    {
        if (_h == IntPtr.Zero) return;
        waveOutReset(_h);
        for (int i = 0; i < NBUF; i++)
        {
            if (_hdr[i] != IntPtr.Zero)
            {
                waveOutUnprepareHeader(_h, _hdr[i], HdrSize);
                Marshal.FreeHGlobal(_hdr[i]);
            }
            if (_data[i] != IntPtr.Zero) Marshal.FreeHGlobal(_data[i]);
        }
        waveOutClose(_h);
        _h = IntPtr.Zero;
    }
}
