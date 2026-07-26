#pragma once

// A Dayna DaynaPORT SCSI/Link Ethernet adapter -- the period-correct network
// device for a slotless compact Mac, and the one the retro community's SCSI
// emulators serve to real machines, so drivers for System 6/7 exist and are
// obtainable. The device moves raw Ethernet frames; the host front-end owns
// what the frames mean (its user-mode NAT, a bridge, a UDP tunnel...).
//
// The vendor command set here is the shape documented publicly by the
// reimplementation projects (BlueSCSI / PiSCSI / scuznet -- behavior
// references only) and is deliberately permissive: anything unrecognized is
// logged through onDiag and answered GOOD, so a real driver's expectations
// can be read out of a trace and pinned during bring-up (plan C-E1) instead
// of guessed at.
//
//   $08  read packet(s): Data-In = 6-byte header (length BE16, flags BE32 --
//        bit 4 set when more packets wait) + the frame; all zeros when idle.
//   $0A  send packet: length in CDB[3..4], Data-Out = the frame.
//   $09  retrieve statistics: MAC(6) + three BE32 error counters.
//   $0C/$0D  filter/register setup: accepted.
//   $0E  enable/disable the interface (CDB[5] bit 7).
//
// Frames cross to the host through bounded queues: injectFrame (host->guest,
// dropped with a count when the guest reads too slowly) and drainFrame.

#include "scsi.hpp"

#include <cstring>
#include <deque>
#include <functional>
#include <vector>

namespace openmac {

class ScsiEthernet : public ScsiTarget {
public:
    std::function<void(const char*)> onDiag;

    void setAttached(bool on, int busId) { attached_ = on; id_ = busId & 7; }
    bool attachedState() const { return attached_; }
    void setMac(const u8 mac[6]) { std::memcpy(mac_, mac, 6); }
    bool linkEnabled() const { return enabled_; }

    // Host -> guest. Returns false (and counts the drop) when the ring is full.
    bool injectFrame(const u8* frame, u32 len) {
        if (!attached_ || len < 14 || len > 1600) return false;
        if (rx_.size() >= kRxDepth) { ++rxDropped_; return false; }
        rx_.emplace_back(frame, frame + len);
        return true;
    }

    // Guest -> host. Returns false when nothing is waiting.
    bool drainFrame(std::vector<u8>& out) {
        if (tx_.empty()) return false;
        out = std::move(tx_.front());
        tx_.pop_front();
        return true;
    }

    u32 rxDropped() const { return rxDropped_; }

    // ---- ScsiTarget ----
    bool present() const override { return attached_; }
    int  id() const override { return id_; }

    u8 execute(const u8* cdb, std::vector<u8>& out, u32& writeBytes) override {
        out.clear();
        writeBytes = 0;
        switch (cdb[0]) {
            case 0x00:   // TEST UNIT READY
                return good();
            case 0x03: { // REQUEST SENSE
                u8 d[18] = {};
                d[0] = 0x70;
                d[2] = senseKey_;
                d[7] = 10;
                emit(out, d, sizeof d, cdb[4] ? cdb[4] : 18);
                senseKey_ = 0;
                return 0x00;
            }
            case 0x12: { // INQUIRY
                u8 d[36] = {};
                d[0] = 0x03;   // processor device, the identity the driver probes
                d[2] = 0x02;
                d[3] = 0x02;
                d[4] = 31;
                std::memcpy(d + 8,  "Dayna   ", 8);
                std::memcpy(d + 16, "SCSI/Link       ", 16);
                std::memcpy(d + 32, "1.4a", 4);
                emit(out, d, sizeof d, cdb[4]);
                return 0x00;
            }
            case 0x08: { // read packet(s)
                const u32 alloc = (static_cast<u32>(cdb[3]) << 8) | cdb[4];
                u8 hdr[6] = {};
                if (rx_.empty() || !enabled_) {
                    emit(out, hdr, sizeof hdr, alloc ? alloc : 6);
                    return good();
                }
                std::vector<u8> frame = std::move(rx_.front());
                rx_.pop_front();
                const u32 len = static_cast<u32>(frame.size());
                hdr[0] = static_cast<u8>(len >> 8);
                hdr[1] = static_cast<u8>(len);
                hdr[5] = rx_.empty() ? 0x00 : 0x10;   // more packets waiting
                out.assign(hdr, hdr + 6);
                out.insert(out.end(), frame.begin(), frame.end());
                if (alloc && out.size() > alloc) out.resize(alloc);
                return good();
            }
            case 0x0A: { // send packet: the frame arrives as Data-Out
                writeBytes = (static_cast<u32>(cdb[3]) << 8) | cdb[4];
                sendTrim_ = (cdb[5] & 0x80) != 0;   // mode with a trailing pad
                return good();
            }
            case 0x09: { // retrieve statistics
                u8 d[18] = {};
                std::memcpy(d, mac_, 6);
                emit(out, d, sizeof d, cdb[4] ? cdb[4] : 18);
                return good();
            }
            case 0x0C:   // multicast / filter registers: accepted
            case 0x0D:
                writeBytes = cdb[4];   // parameters (if any) arrive and are kept
                return good();
            case 0x0E:   // enable/disable the interface
                enabled_ = (cdb[5] & 0x80) != 0;
                if (onDiag)
                    onDiag(enabled_ ? "net: interface enabled by the guest"
                                    : "net: interface disabled by the guest");
                return good();
            default:
                if (onDiag) {
                    char b[64];
                    std::snprintf(b, sizeof b,
                                  "net: unhandled CDB %02X %02X %02X %02X %02X %02X",
                                  cdb[0], cdb[1], cdb[2], cdb[3], cdb[4], cdb[5]);
                    onDiag(b);
                }
                return good();   // permissive during bring-up; the trace names it
        }
    }

    void acceptWrite(const std::vector<u8>& data) override {
        if (data.size() < 14) return;   // filter-register writes land here too
        std::vector<u8> frame = data;
        if (sendTrim_ && frame.size() > 18) frame.resize(frame.size() - 4);
        sendTrim_ = false;
        if (tx_.size() < kTxDepth) tx_.push_back(std::move(frame));
    }

private:
    u8 good() { senseKey_ = 0; return 0x00; }

    static void emit(std::vector<u8>& out, const u8* d, std::size_t n, u32 alloc) {
        const std::size_t len = (alloc && alloc < n) ? alloc : n;
        out.insert(out.end(), d, d + len);
    }

    static constexpr std::size_t kRxDepth = 64;
    static constexpr std::size_t kTxDepth = 64;

    bool attached_ = false;
    int  id_ = 4;   // a free ID beside disk 0, disk 1, CD 3
    bool enabled_ = false;
    bool sendTrim_ = false;
    u8   mac_[6] = {0x52, 0x54, 0x00, 0x0C, 0x1A, 0x51};   // locally administered
    u8   senseKey_ = 0;
    std::deque<std::vector<u8>> rx_, tx_;
    u32  rxDropped_ = 0;
};

} // namespace openmac
