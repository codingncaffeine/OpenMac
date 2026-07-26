using System.Net;
using System.Net.Sockets;

namespace OpenMac.Gui.Emulation;

/// <summary>
/// User-mode NAT for the DaynaPORT adapter — the slirp idea in managed code:
/// no drivers, no admin, nothing listening on the host's behalf. The guest
/// lives on 10.0.2.0/24 (gateway .2, DNS .3, guest .15, BOOTP-assigned).
/// ARP/BOOTP/ICMP-echo are answered locally; DNS resolves through the host;
/// UDP relays socket-per-flow; TCP terminates here and relays bytes over
/// ordinary outbound host sockets. Frames toward the guest go through the
/// injector callback, which may be called from socket-pool threads.
/// </summary>
internal sealed class SlirpNat : IDisposable
{
    private static readonly byte[] GwMac = { 0x52, 0x54, 0x00, 0x12, 0x35, 0x02 };
    private static readonly byte[] GwIp = { 10, 0, 2, 2 };
    private static readonly byte[] DnsIp = { 10, 0, 2, 3 };
    private static readonly byte[] GuestIp = { 10, 0, 2, 15 };
    private static readonly byte[] Mask = { 255, 255, 255, 0 };

    private readonly Action<byte[]> _inject;
    private readonly Action<string> _log;
    private readonly object _lock = new();
    private byte[] _guestMac = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    private bool _disposed;
    private uint _isnSeed = 0x0C1A0000;

    private readonly Dictionary<(ushort, uint, ushort), UdpFlow> _udp = new();
    private readonly Dictionary<(ushort, uint, ushort), TcpFlow> _tcp = new();

    public SlirpNat(Action<byte[]> injectToGuest, Action<string> log)
    {
        _inject = injectToGuest;
        _log = log;
    }

    public void Dispose()
    {
        lock (_lock)
        {
            _disposed = true;
            foreach (var f in _udp.Values) f.Dispose();
            foreach (var f in _tcp.Values) f.Dispose();
            _udp.Clear();
            _tcp.Clear();
        }
    }

    // ---- guest -> world --------------------------------------------------

    public void OnGuestFrame(byte[] f)
    {
        if (f.Length < 14) return;
        Array.Copy(f, 6, _guestMac, 0, 6);
        int ethertype = (f[12] << 8) | f[13];
        if (ethertype == 0x0806) { OnArp(f); return; }
        if (ethertype != 0x0800 || f.Length < 34) return;   // EtherTalk etc: not ours

        int ip = 14;
        if ((f[ip] >> 4) != 4) return;
        int ihl = (f[ip] & 0x0F) * 4;
        int total = (f[ip + 2] << 8) | f[ip + 3];
        if (total < ihl || ip + total > f.Length) return;
        byte proto = f[ip + 9];
        uint dstIp = Be32(f, ip + 16);
        int l4 = ip + ihl;
        int l4len = total - ihl;

        switch (proto)
        {
            case 17: OnUdp(f, ip, l4, l4len, dstIp); break;
            case 6: OnTcp(f, ip, l4, l4len, dstIp); break;
            case 1: OnIcmp(f, ip, l4, l4len, dstIp); break;
        }
    }

    private void OnArp(byte[] f)
    {
        // Answer "who has 10.0.2.x" for everything that isn't the guest itself.
        if (f.Length < 42 || f[20] != 0 || f[21] != 1) return;   // ARP request only
        byte[] target = { f[38], f[39], f[40], f[41] };
        if (target.AsSpan().SequenceEqual(GuestIp)) return;
        byte[] reply = new byte[42];
        Array.Copy(_guestMac, 0, reply, 0, 6);
        Array.Copy(GwMac, 0, reply, 6, 6);
        reply[12] = 0x08; reply[13] = 0x06;
        reply[14] = 0; reply[15] = 1;          // htype ethernet
        reply[16] = 0x08; reply[17] = 0x00;    // ptype IPv4
        reply[18] = 6; reply[19] = 4;
        reply[20] = 0; reply[21] = 2;          // ARP reply
        Array.Copy(GwMac, 0, reply, 22, 6);
        Array.Copy(target, 0, reply, 28, 4);   // sender = the asked-for address
        Array.Copy(f, 22, reply, 32, 6);       // target = the guest
        Array.Copy(f, 28, reply, 38, 4);
        _inject(reply);
    }

    private void OnIcmp(byte[] f, int ip, int l4, int len, uint dstIp)
    {
        if (len < 8 || f[l4] != 8) return;     // echo request only
        if (dstIp != Be32(GwIp, 0) && dstIp != Be32(DnsIp, 0)) return;
        byte[] icmp = new byte[len];
        Array.Copy(f, l4, icmp, 0, len);
        icmp[0] = 0;                            // echo reply
        icmp[2] = 0; icmp[3] = 0;
        PutSum(icmp, 2, Checksum(icmp, 0, len));
        SendIp(f, ip + 12, 1, icmp);
    }

    // ---- UDP -------------------------------------------------------------

    private void OnUdp(byte[] f, int ip, int l4, int len, uint dstIp)
    {
        if (len < 8) return;
        ushort srcPort = (ushort)((f[l4] << 8) | f[l4 + 1]);
        ushort dstPort = (ushort)((f[l4 + 2] << 8) | f[l4 + 3]);
        int dlen = Math.Min(((f[l4 + 4] << 8) | f[l4 + 5]) - 8, len - 8);
        if (dlen < 0) return;
        byte[] payload = new byte[dlen];
        Array.Copy(f, l4 + 8, payload, 0, dlen);

        if (dstPort == 67) { BootpReply(payload, srcPort); return; }
        if (dstIp == Be32(DnsIp, 0) && dstPort == 53) { DnsQuery(payload, srcPort); return; }

        lock (_lock)
        {
            if (_disposed) return;
            var key = (srcPort, dstIp, dstPort);
            if (!_udp.TryGetValue(key, out var flow))
            {
                flow = new UdpFlow(this, srcPort, dstIp, dstPort);
                _udp[key] = flow;
                if (_udp.Count > 64) ReapUdp();
            }
            flow.Send(payload);
        }
    }

    private void ReapUdp()
    {
        var dead = new List<(ushort, uint, ushort)>();
        foreach (var kv in _udp)
            if ((DateTime.UtcNow - kv.Value.LastUsed).TotalSeconds > 60)
                dead.Add(kv.Key);
        foreach (var k in dead) { _udp[k].Dispose(); _udp.Remove(k); }
    }

    private sealed class UdpFlow : IDisposable
    {
        private readonly SlirpNat _nat;
        private readonly UdpClient _sock = new();
        private readonly ushort _guestPort;
        private readonly uint _remoteIp;
        private readonly ushort _remotePort;
        public DateTime LastUsed = DateTime.UtcNow;

        public UdpFlow(SlirpNat nat, ushort guestPort, uint remoteIp, ushort remotePort)
        {
            _nat = nat;
            _guestPort = guestPort;
            _remoteIp = remoteIp;
            _remotePort = remotePort;
            _ = ReceiveLoop();
        }

        public void Send(byte[] payload)
        {
            LastUsed = DateTime.UtcNow;
            try
            {
                _sock.Send(payload, payload.Length,
                           new IPEndPoint(new IPAddress(HostOrder(_remoteIp)), _remotePort));
            }
            catch { }
        }

        private async Task ReceiveLoop()
        {
            try
            {
                while (true)
                {
                    var r = await _sock.ReceiveAsync();
                    LastUsed = DateTime.UtcNow;
                    _nat.SendUdpToGuest(_remoteIp, _remotePort, _guestPort, r.Buffer);
                }
            }
            catch { /* socket closed */ }
        }

        public void Dispose() => _sock.Dispose();
    }

    // ---- BOOTP (MacTCP's zero-config path) -------------------------------

    private void BootpReply(byte[] req, ushort guestPort)
    {
        if (req.Length < 236 || req[0] != 1) return;
        byte[] r = new byte[300];
        r[0] = 2;                              // BOOTREPLY
        r[1] = 1; r[2] = 6;                    // ethernet, hlen 6
        Array.Copy(req, 4, r, 4, 4);           // xid
        Array.Copy(GuestIp, 0, r, 16, 4);      // yiaddr
        Array.Copy(GwIp, 0, r, 20, 4);         // siaddr
        Array.Copy(req, 28, r, 28, 16);        // chaddr
        // RFC 1497 vendor extensions: magic, subnet mask, router, DNS, end.
        r[236] = 99; r[237] = 130; r[238] = 83; r[239] = 99;
        r[240] = 1; r[241] = 4; Array.Copy(Mask, 0, r, 242, 4);
        r[246] = 3; r[247] = 4; Array.Copy(GwIp, 0, r, 248, 4);
        r[252] = 6; r[253] = 4; Array.Copy(DnsIp, 0, r, 254, 4);
        r[258] = 255;
        SendUdpToGuest(Be32(GwIp, 0), 67, guestPort == 0 ? (ushort)68 : guestPort, r);
        _log("net: BOOTP answered — guest is 10.0.2.15, router 10.0.2.2, DNS 10.0.2.3");
    }

    // ---- DNS -------------------------------------------------------------

    private void DnsQuery(byte[] q, ushort guestPort)
    {
        if (q.Length < 17) return;
        // Parse the single question's name.
        int p = 12;
        var name = new System.Text.StringBuilder();
        while (p < q.Length && q[p] != 0)
        {
            int n = q[p++];
            if (n > 63 || p + n > q.Length) return;
            if (name.Length > 0) name.Append('.');
            name.Append(System.Text.Encoding.ASCII.GetString(q, p, n));
            p += n;
        }
        if (p + 5 > q.Length) return;
        int qtype = (q[p + 1] << 8) | q[p + 2];
        int qend = p + 5;
        string host = name.ToString();

        _ = Task.Run(() =>
        {
            byte[] r;
            try
            {
                var addrs = qtype == 1
                    ? Array.FindAll(Dns.GetHostAddresses(host),
                                    a => a.AddressFamily == AddressFamily.InterNetwork)
                    : Array.Empty<IPAddress>();
                int count = Math.Min(addrs.Length, 8);
                r = new byte[qend + count * 16];
                Array.Copy(q, r, qend);
                r[2] = 0x81; r[3] = 0x80;                 // response, RD+RA, NOERROR
                r[6] = 0; r[7] = (byte)count;             // answer count
                int w = qend;
                for (int i = 0; i < count; ++i)
                {
                    r[w++] = 0xC0; r[w++] = 0x0C;         // pointer to the question name
                    r[w++] = 0; r[w++] = 1;               // A
                    r[w++] = 0; r[w++] = 1;               // IN
                    r[w++] = 0; r[w++] = 0; r[w++] = 0; r[w++] = 60;   // TTL
                    r[w++] = 0; r[w++] = 4;
                    Array.Copy(addrs[i].GetAddressBytes(), 0, r, w, 4);
                    w += 4;
                }
            }
            catch
            {
                r = new byte[qend];
                Array.Copy(q, r, qend);
                r[2] = 0x81; r[3] = 0x82;                 // SERVFAIL
            }
            SendUdpToGuest(Be32(DnsIp, 0), 53, guestPort, r);
        });
    }

    // ---- TCP -------------------------------------------------------------

    private void OnTcp(byte[] f, int ip, int l4, int len, uint dstIp)
    {
        if (len < 20) return;
        ushort srcPort = (ushort)((f[l4] << 8) | f[l4 + 1]);
        ushort dstPort = (ushort)((f[l4 + 2] << 8) | f[l4 + 3]);
        uint seq = Be32(f, l4 + 4);
        uint ack = Be32(f, l4 + 8);
        int off = (f[l4 + 12] >> 4) * 4;
        byte flags = f[l4 + 13];
        int dlen = len - off;
        if (dlen < 0) return;

        var key = (srcPort, dstIp, dstPort);
        lock (_lock)
        {
            if (_disposed) return;
            _tcp.TryGetValue(key, out var flow);
            const byte SYN = 0x02, RST = 0x04;

            if (flow is null)
            {
                if ((flags & SYN) == 0 || (flags & 0x10) != 0) return;   // only fresh SYNs
                flow = new TcpFlow(this, srcPort, dstIp, dstPort, seq, _isnSeed);
                _isnSeed += 0x10000;
                _tcp[key] = flow;
                flow.Connect();
                return;
            }
            if ((flags & RST) != 0) { flow.Dispose(); _tcp.Remove(key); return; }
            flow.OnGuestSegment(seq, ack, flags, f, l4 + off, dlen);
            if (flow.Closed) { flow.Dispose(); _tcp.Remove(key); }
        }
    }

    private sealed class TcpFlow : IDisposable
    {
        private readonly SlirpNat _nat;
        private readonly ushort _guestPort;
        private readonly uint _remoteIp;
        private readonly ushort _remotePort;
        private readonly TcpClient _sock = new();
        private NetworkStream? _stream;
        private uint _ourSeq;          // next byte we will send
        private uint _guestSeq;        // next byte we expect from the guest
        private uint _guestAcked;      // highest ack the guest has sent for our data
        private bool _finSent, _guestFinSeen;
        public bool Closed { get; private set; }

        public TcpFlow(SlirpNat nat, ushort guestPort, uint remoteIp, ushort remotePort,
                       uint guestIsn, uint ourIsn)
        {
            _nat = nat;
            _guestPort = guestPort;
            _remoteIp = remoteIp;
            _remotePort = remotePort;
            _guestSeq = guestIsn + 1;
            _ourSeq = ourIsn;
            _guestAcked = ourIsn;
        }

        public void Connect()
        {
            _ = Task.Run(async () =>
            {
                try
                {
                    await _sock.ConnectAsync(new IPAddress(HostOrder(_remoteIp)), _remotePort);
                    _stream = _sock.GetStream();
                    // SYN-ACK with an MSS the Mac can live with.
                    byte[] opts = { 2, 4, 0x05, 0xB4 };
                    _nat.SendTcpToGuest(_remoteIp, _remotePort, _guestPort,
                                        _ourSeq, _guestSeq, 0x12, opts,
                                        Array.Empty<byte>(), 0, 0);
                    _ourSeq += 1;
                    _ = ReceiveLoop();
                }
                catch
                {
                    _nat.SendTcpToGuest(_remoteIp, _remotePort, _guestPort,
                                        0, _guestSeq, 0x14,             // RST+ACK
                                        Array.Empty<byte>(), Array.Empty<byte>(), 0, 0);
                    Closed = true;
                }
            });
        }

        public void OnGuestSegment(uint seq, uint ack, byte flags, byte[] buf, int off, int len)
        {
            if ((flags & 0x10) != 0) _guestAcked = ack;
            if (len > 0 && seq == _guestSeq && _stream is not null)
            {
                try { _stream.Write(buf, off, len); } catch { }
                _guestSeq += (uint)len;
                Ack();
            }
            else if (len > 0 && seq != _guestSeq)
            {
                Ack();   // duplicate or out of order: restate where we are
            }
            if ((flags & 0x01) != 0 && !_guestFinSeen && seq + (uint)len == _guestSeq)
            {
                _guestFinSeen = true;
                _guestSeq += 1;
                try { _sock.Client.Shutdown(SocketShutdown.Send); } catch { }
                Ack();
                if (_finSent) Closed = true;
            }
            if (_finSent && _guestFinSeen && ack == _ourSeq) Closed = true;
        }

        private void Ack() =>
            _nat.SendTcpToGuest(_remoteIp, _remotePort, _guestPort, _ourSeq, _guestSeq,
                                0x10, Array.Empty<byte>(), Array.Empty<byte>(), 0, 0);

        private async Task ReceiveLoop()
        {
            byte[] buf = new byte[1460];
            try
            {
                while (true)
                {
                    // Pace by the guest's acks: at most 8 KB in flight.
                    while (unchecked(_ourSeq - _guestAcked) > 8192)
                        await Task.Delay(10);
                    int n = await _stream!.ReadAsync(buf);
                    if (n == 0) break;
                    byte[] seg = new byte[n];
                    Array.Copy(buf, seg, n);
                    _nat.SendTcpToGuest(_remoteIp, _remotePort, _guestPort, _ourSeq,
                                        _guestSeq, 0x18, Array.Empty<byte>(), seg, 0, n);
                    _ourSeq += (uint)n;
                }
            }
            catch { }
            // Host side is done: FIN toward the guest.
            _nat.SendTcpToGuest(_remoteIp, _remotePort, _guestPort, _ourSeq, _guestSeq,
                                0x11, Array.Empty<byte>(), Array.Empty<byte>(), 0, 0);
            _ourSeq += 1;
            _finSent = true;
            if (_guestFinSeen) Closed = true;
        }

        public void Dispose()
        {
            try { _sock.Dispose(); } catch { }
        }
    }

    // ---- frame building --------------------------------------------------

    private void SendUdpToGuest(uint srcIp, ushort srcPort, ushort dstPort, byte[] payload)
    {
        byte[] udp = new byte[8 + payload.Length];
        udp[0] = (byte)(srcPort >> 8); udp[1] = (byte)srcPort;
        udp[2] = (byte)(dstPort >> 8); udp[3] = (byte)dstPort;
        udp[4] = (byte)((udp.Length) >> 8); udp[5] = (byte)udp.Length;
        Array.Copy(payload, 0, udp, 8, payload.Length);
        PutSum(udp, 6, PseudoSum(srcIp, Be32(GuestIp, 0), 17, udp, udp.Length));
        InjectIp(srcIp, 17, udp);
    }

    private void SendTcpToGuest(uint srcIp, ushort srcPort, ushort dstPort, uint seq,
                                uint ack, byte flags, byte[] opts, byte[] data,
                                int off, int len)
    {
        int hdr = 20 + opts.Length;
        byte[] tcp = new byte[hdr + len];
        tcp[0] = (byte)(srcPort >> 8); tcp[1] = (byte)srcPort;
        tcp[2] = (byte)(dstPort >> 8); tcp[3] = (byte)dstPort;
        PutBe32(tcp, 4, seq);
        PutBe32(tcp, 8, ack);
        tcp[12] = (byte)((hdr / 4) << 4);
        tcp[13] = flags;
        tcp[14] = 0x20; tcp[15] = 0x00;   // window 8192
        Array.Copy(opts, 0, tcp, 20, opts.Length);
        if (len > 0) Array.Copy(data, off, tcp, hdr, len);
        PutSum(tcp, 16, PseudoSum(srcIp, Be32(GuestIp, 0), 6, tcp, tcp.Length));
        InjectIp(srcIp, 6, tcp);
    }

    /// <summary>Reply using the request's own source/dest IPs (ICMP echo).</summary>
    private void SendIp(byte[] reqFrame, int reqSrcIpOff, byte proto, byte[] payload)
    {
        InjectIp(Be32(reqFrame, reqSrcIpOff + 4), proto, payload);
    }

    private ushort _ipId = 1;

    private void InjectIp(uint srcIp, byte proto, byte[] payload)
    {
        byte[] frame = new byte[14 + 20 + payload.Length];
        byte[] guestMac;
        lock (_lock) guestMac = (byte[])_guestMac.Clone();
        Array.Copy(guestMac, 0, frame, 0, 6);
        Array.Copy(GwMac, 0, frame, 6, 6);
        frame[12] = 0x08; frame[13] = 0x00;
        int ip = 14;
        frame[ip] = 0x45;
        int total = 20 + payload.Length;
        frame[ip + 2] = (byte)(total >> 8); frame[ip + 3] = (byte)total;
        ushort id;
        lock (_lock) id = _ipId++;
        frame[ip + 4] = (byte)(id >> 8); frame[ip + 5] = (byte)id;
        frame[ip + 8] = 64;               // TTL
        frame[ip + 9] = proto;
        PutBe32(frame, ip + 12, srcIp);
        Array.Copy(GuestIp, 0, frame, ip + 16, 4);
        PutSum(frame, ip + 10, Checksum(frame, ip, 20));
        Array.Copy(payload, 0, frame, ip + 20, payload.Length);
        _inject(frame);
    }

    // ---- byte helpers ----------------------------------------------------

    private static uint Be32(byte[] b, int p) =>
        ((uint)b[p] << 24) | ((uint)b[p + 1] << 16) | ((uint)b[p + 2] << 8) | b[p + 3];

    private static void PutBe32(byte[] b, int p, uint v)
    {
        b[p] = (byte)(v >> 24); b[p + 1] = (byte)(v >> 16);
        b[p + 2] = (byte)(v >> 8); b[p + 3] = (byte)v;
    }

    private static long HostOrder(uint beIp) =>
        (uint)IPAddress.HostToNetworkOrder((int)beIp);

    private static ushort Checksum(byte[] b, int off, int len)
    {
        uint sum = 0;
        for (int i = 0; i + 1 < len; i += 2) sum += (uint)((b[off + i] << 8) | b[off + i + 1]);
        if ((len & 1) != 0) sum += (uint)(b[off + len - 1] << 8);
        while ((sum >> 16) != 0) sum = (sum & 0xFFFF) + (sum >> 16);
        return (ushort)~sum;
    }

    private static ushort PseudoSum(uint src, uint dst, byte proto, byte[] seg, int len)
    {
        uint sum = 0;
        sum += (src >> 16) + (src & 0xFFFF);
        sum += (dst >> 16) + (dst & 0xFFFF);
        sum += proto;
        sum += (uint)len;
        for (int i = 0; i + 1 < len; i += 2) sum += (uint)((seg[i] << 8) | seg[i + 1]);
        if ((len & 1) != 0) sum += (uint)(seg[len - 1] << 8);
        while ((sum >> 16) != 0) sum = (sum & 0xFFFF) + (sum >> 16);
        return (ushort)~sum;
    }

    private static void PutSum(byte[] b, int p, ushort sum)
    {
        b[p] = (byte)(sum >> 8);
        b[p + 1] = (byte)sum;
    }
}
