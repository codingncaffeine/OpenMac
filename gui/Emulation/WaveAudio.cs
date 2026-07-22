using System.Runtime.InteropServices;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// Streaming audio sink over Win32 waveOut — no external dependency. The emulated
/// Mac produces 8-bit unsigned mono PCM at ~22254 Hz. The per-frame sample drain
/// is jittery (it rides the WPF dispatcher timer), so samples are accumulated in a
/// ring and handed to waveOut as full, fixed-size buffers kept a few deep. That
/// decouples the uneven producer from the steady consumer and avoids the underruns
/// and dropped samples that make naive per-frame feeding sound rough.
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
    private const int NBUF = 5;          // buffers kept in flight
    private const int BUFSZ = 1024;      // ~46 ms each at 22254 Hz
    private static readonly int HdrSize = Marshal.SizeOf<WAVEHDR>();

    private IntPtr _h;
    private readonly IntPtr[] _hdr = new IntPtr[NBUF];
    private readonly IntPtr[] _data = new IntPtr[NBUF];
    private readonly bool[] _used = new bool[NBUF];

    // Accumulation ring (decouples the jittery drain from steady playback).
    private readonly byte[] _ring = new byte[BUFSZ * (NBUF + 4)];
    private int _rHead, _rCount;

    // Health counters, reset each Stats() read (diagnostics for choppy audio).
    private int _underruns, _drops;
    private long _fed;
    private bool _started;
    private bool _primed;

    // 8-bit UNSIGNED PCM silence is 0x80, not 0x00. Priming the device with a
    // couple of these buffers starts playback during silence, so the first real
    // samples don't trigger a start-of-stream click. (The SDL shell stayed clean
    // because SDL keeps its audio device continuously fed from the moment it opens.)
    private static readonly byte[] Silence = MakeSilence();
    private static byte[] MakeSilence() { var s = new byte[BUFSZ]; for (int i = 0; i < BUFSZ; i++) s[i] = 0x80; return s; }

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

    /// <summary>Accumulate this frame's samples and push any full buffers to the device.</summary>
    public void Feed(byte[] samples, int count)
    {
        if (_h == IntPtr.Zero || count <= 0) return;
        if (!_primed) { _primed = true; Prime(); }

        // 1. Append to the ring; on overflow drop the oldest so latency stays bounded.
        for (int i = 0; i < count; i++)
        {
            if (_rCount == _ring.Length) { _rHead = (_rHead + 1) % _ring.Length; _rCount--; _drops++; }
            _ring[(_rHead + _rCount) % _ring.Length] = samples[i];
            _rCount++;
        }
        _fed += count;

        // 2. Emit full, fixed-size buffers into any free slots.
        while (_rCount >= BUFSZ)
        {
            if (_started && InFlight() == 0) _underruns++;   // waveOut had run dry
            int slot = FindFree();
            if (slot < 0) break;            // all buffers busy — leave the rest in the ring
            int first = Math.Min(BUFSZ, _ring.Length - _rHead);
            Marshal.Copy(_ring, _rHead, _data[slot], first);
            if (first < BUFSZ)
                Marshal.Copy(_ring, 0, IntPtr.Add(_data[slot], first), BUFSZ - first);
            _rHead = (_rHead + BUFSZ) % _ring.Length;
            _rCount -= BUFSZ;

            var h = Marshal.PtrToStructure<WAVEHDR>(_hdr[slot]);
            h.dwBufferLength = BUFSZ;
            h.dwFlags &= ~WHDR_DONE;
            Marshal.StructureToPtr(h, _hdr[slot], false);
            waveOutWrite(_h, _hdr[slot], HdrSize);
            _used[slot] = true;
            _started = true;
        }
    }

    // Start the device playing silence so the first real samples don't click.
    private void Prime()
    {
        for (int k = 0; k < 2; k++)
        {
            int slot = FindFree();
            if (slot < 0) break;
            Marshal.Copy(Silence, 0, _data[slot], BUFSZ);
            var h = Marshal.PtrToStructure<WAVEHDR>(_hdr[slot]);
            h.dwBufferLength = BUFSZ;
            h.dwFlags &= ~WHDR_DONE;
            Marshal.StructureToPtr(h, _hdr[slot], false);
            waveOutWrite(_h, _hdr[slot], HdrSize);
            _used[slot] = true;
            _started = true;
        }
    }

    // Buffers written to the device but not yet finished playing.
    private int InFlight()
    {
        int n = 0;
        for (int i = 0; i < NBUF; i++)
        {
            if (!_used[i]) continue;
            var h = Marshal.PtrToStructure<WAVEHDR>(_hdr[i]);
            if ((h.dwFlags & WHDR_DONE) == 0) n++;
        }
        return n;
    }

    /// <summary>One-line health snapshot; resets the per-interval counters.</summary>
    public string Stats()
    {
        string s = $"audio: inflight={InFlight()}/{NBUF} ring={_rCount} fed={_fed} underruns={_underruns} drops={_drops}";
        _fed = 0; _underruns = 0; _drops = 0;
        return s;
    }

    private int FindFree()
    {
        for (int i = 0; i < NBUF; i++)
        {
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
