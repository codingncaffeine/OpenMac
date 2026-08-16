#pragma once

// A SCSI-2 CD-ROM target of the AppleCD SC family. The drive identity is the
// AppleCD 300's mechanism (Sony's CDU-8003A) because that is a drive the stock
// Apple CD-ROM software shipped for; the strings get refined against the
// user's actual driver if it turns its nose up (plan C-B1). Read-only by
// construction: WRITE reports DATA PROTECT and nothing ever copies back out.
//
// What the Apple driver actually leans on:
//   - TEST UNIT READY polling is how discs appear: NOT READY (ASC $3A) with an
//     empty tray, one UNIT ATTENTION ($28, medium changed) after an insertion,
//     then GOOD. REQUEST SENSE carries the details and clears them.
//   - MODE SELECT(6) switches the logical block size; period Apple drivers run
//     HFS discs at 512 bytes. The image is a flat byte run, so any block size
//     addresses it cleanly (offset = LBA x size).
//   - READ TOC in MSF and LBA forms; one data track starting at 00:02:00.
//   - START STOP UNIT with LoEj is the Finder's drag-to-Trash eject; the host
//     polls takeEjectRequest() to hear that the guest let the disc go.
//
// Reference: SCSI-2 (X3.131) §8-§14 command set; AppleCD SC Developers Guide
// holds the vendor audio commands for the audio phase. Clean-room from specs.

#include "scsi.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <functional>
#include <vector>

namespace openmac {

class ScsiCdRom : public ScsiTarget {
public:
    // The drive on the bus (a device), separate from a disc in it (media).
    void setAttached(bool on, int busId) { attached_ = on; id_ = busId & 7; }
    bool attachedState() const { return attached_; }

    void insert(std::vector<u8> data) {
        media_ = std::move(data);
        blockSize_ = 2048;
        unitAttention_ = true;   // the next command hears "medium changed"
    }
    void eject() { media_.clear(); unitAttention_ = true; }
    bool discPresent() const { return !media_.empty(); }
    // The flat byte run the drive serves. A driver installed by the machine
    // reads the disc from here rather than through the bus, the way the hard
    // disk's Prime hook reads its image -- the guest's driver, DCE and
    // completion path stay its own; only the transfer is the machine's.
    const std::vector<u8>& media() const { return media_; }
    // One-shot: the guest ejected the disc (START STOP UNIT + LoEj).
    bool takeEjectRequest() {
        const bool r = ejectRequest_;
        ejectRequest_ = false;
        return r;
    }
    u32 blockSize() const { return blockSize_; }

    // ---- ScsiTarget ----
    bool present() const override { return attached_; }
    int  id() const override { return id_; }

    // Every command the drive is asked, with what it answered. A CD that will
    // not mount is a conversation between the Apple CD-ROM driver and this
    // drive, and until it is written down the failure is a silence.
    std::function<void(const char*)> onDiag;
    int diagBudget = 400;

    u8 execute(const u8* cdb, std::vector<u8>& out, u32& writeBytes) override {
        const u8 status = executeImpl(cdb, out, writeBytes);
        if (onDiag && diagBudget > 0) {
            --diagBudget;
            char b[160];
            const int n = cdbLen(cdb[0]);
            int p = std::snprintf(b, sizeof b, "cd: cdb");
            for (int i = 0; i < n && p < 60; ++i)
                p += std::snprintf(b + p, sizeof b - std::size_t(p), " %02X", cdb[0 + i]);
            std::snprintf(b + p, sizeof b - std::size_t(p),
                          " -> %s%s, %zu bytes in%s",
                          status == 0 ? "GOOD" : "CHECK",
                          status == 0 ? "" : status == 0x02 ? "" : " ?",
                          out.size(),
                          status == 0 ? "" : (senseKey_ == 0x05 && asc_ == 0x20)
                                                 ? "  <-- COMMAND NOT IMPLEMENTED"
                                                 : "");
            onDiag(b);
        }
        return status;
    }

private:
    u8 executeImpl(const u8* cdb, std::vector<u8>& out, u32& writeBytes) {
        out.clear();
        writeBytes = 0;
        const u8 op = cdb[0];
        // A latched unit-attention answers everything except INQUIRY and
        // REQUEST SENSE once, then clears (SCSI-2 rules).
        if (unitAttention_ && op != 0x12 && op != 0x03) {
            unitAttention_ = false;
            return check(0x06, 0x28);   // UNIT ATTENTION: medium may have changed
        }
        switch (op) {
            case 0x00:   // TEST UNIT READY
                return discPresent() ? good() : check(0x02, 0x3A);
            case 0x03:   // REQUEST SENSE
                appendSense(out, cdb[4]);
                return 0x00;
            case 0x12:   // INQUIRY
                appendInquiry(out, cdb[4]);
                return 0x00;
            case 0x15:   // MODE SELECT(6): the parameter list arrives as Data-Out
                writeBytes = cdb[4];
                modeSelectPending_ = writeBytes != 0;
                return 0x00;
            case 0x1A:   // MODE SENSE(6)
                return appendModeSense(out, cdb[2] & 0x3F, cdb[4]);
            case 0x1B:   // START STOP UNIT: LoEj without Start = eject
                if ((cdb[4] & 0x02) && !(cdb[4] & 0x01)) {
                    if (prevented_) return check(0x05, 0x53);   // removal prevented
                    if (discPresent()) {
                        media_.clear();
                        ejectRequest_ = true;
                        unitAttention_ = true;
                    }
                }
                return good();
            case 0x1E:   // PREVENT/ALLOW MEDIUM REMOVAL
                prevented_ = (cdb[4] & 0x01) != 0;
                return good();
            case 0x25: { // READ CAPACITY
                if (!discPresent()) return check(0x02, 0x3A);
                const u32 last = totalBlocks() ? totalBlocks() - 1u : 0u;
                const u8 d[8] = {static_cast<u8>(last >> 24), static_cast<u8>(last >> 16),
                                 static_cast<u8>(last >> 8),  static_cast<u8>(last),
                                 static_cast<u8>(blockSize_ >> 24),
                                 static_cast<u8>(blockSize_ >> 16),
                                 static_cast<u8>(blockSize_ >> 8),
                                 static_cast<u8>(blockSize_)};
                out.assign(d, d + 8);
                return good();
            }
            case 0x08: { // READ(6)
                const u32 lba = ((cdb[1] & 0x1Fu) << 16) | (cdb[2] << 8) | cdb[3];
                return readBlocks(lba, cdb[4] ? cdb[4] : 256u, out);
            }
            case 0x28: { // READ(10)
                const u32 lba = (static_cast<u32>(cdb[2]) << 24) |
                                (static_cast<u32>(cdb[3]) << 16) |
                                (static_cast<u32>(cdb[4]) << 8) | cdb[5];
                return readBlocks(lba, (static_cast<u32>(cdb[7]) << 8) | cdb[8], out);
            }
            case 0x0B:   // SEEK(6) / SEEK(10): position means nothing here
            case 0x2B:
                return discPresent() ? good() : check(0x02, 0x3A);
            case 0x43:   // READ TOC
                if (!discPresent()) return check(0x02, 0x3A);
                appendToc(out, (cdb[1] & 0x02) != 0, cdb[6],
                          (static_cast<u32>(cdb[7]) << 8) | cdb[8]);
                return good();
            case 0x0A:   // WRITE: it is a CD-ROM
            case 0x2A:
                return check(0x07, 0x27);   // DATA PROTECT, write protected
            default:
                return check(0x05, 0x20);   // invalid command operation code
        }
    }

public:
    void acceptWrite(const std::vector<u8>& data) override {
        if (!modeSelectPending_) return;
        modeSelectPending_ = false;
        // MODE SELECT(6) parameter list: 4-byte header (block-descriptor length
        // at [3]), then a block descriptor whose last three bytes are the
        // requested logical block size. Anything that doesn't divide the flat
        // image sensibly is ignored rather than argued with.
        if (data.size() >= 12 && data[3] >= 8) {
            const u32 bs = (static_cast<u32>(data[9]) << 16) |
                           (static_cast<u32>(data[10]) << 8) | data[11];
            if (bs == 512 || bs == 1024 || bs == 2048) blockSize_ = bs;
        }
    }

private:
    u8 good() { senseKey_ = 0; asc_ = 0; return 0x00; }
    u8 check(u8 key, u8 asc) { senseKey_ = key; asc_ = asc; return 0x02; }

    u32 totalBlocks() const {
        return static_cast<u32>((media_.size() + blockSize_ - 1) / blockSize_);
    }
    // TOC addressing counts 2048-byte data sectors regardless of the logical
    // block size a MODE SELECT picked.
    u32 dataSectors2048() const {
        return static_cast<u32>((media_.size() + 2047u) / 2048u);
    }

    u8 readBlocks(u32 lba, u32 n, std::vector<u8>& out) {
        if (!discPresent()) return check(0x02, 0x3A);
        const u32 total = totalBlocks();
        if (lba >= total) return check(0x05, 0x21);   // LBA out of range
        if (n > total - lba) n = total - lba;
        out.assign(static_cast<std::size_t>(n) * blockSize_, 0);
        const std::size_t off = static_cast<std::size_t>(lba) * blockSize_;
        const std::size_t avail = media_.size() > off ? media_.size() - off : 0;
        std::memcpy(out.data(), media_.data() + off, std::min(out.size(), avail));
        return good();
    }

    void appendInquiry(std::vector<u8>& out, u8 allocLen) {
        u8 d[36] = {};
        d[0] = 0x05;   // peripheral device type: CD-ROM
        d[1] = 0x80;   // removable medium
        d[2] = 0x02;   // SCSI-2
        d[3] = 0x02;   // response data format
        d[4] = 31;     // additional length (total 36)
        std::memcpy(d + 8,  "SONY    ", 8);          // vendor (bytes 8-15)
        std::memcpy(d + 16, "CD-ROM CDU-8003A", 16); // product (bytes 16-31)
        std::memcpy(d + 32, "1.9a", 4);              // revision (bytes 32-35)
        emitClamped(out, d, sizeof d, allocLen);
    }

    // Header + block descriptor + the page that was ASKED FOR. Answering every
    // page code with page 01 is the same lie an echoing register stub tells:
    // the initiator gets a well-formed reply that describes something else, and
    // nothing is logged because the command "succeeded". A drive reports
    // ILLEGAL REQUEST for a page it does not have, and that is an answer a
    // driver knows how to handle. Page 3F means "all of them".
    u8 appendModeSense(std::vector<u8>& out, u8 page, u8 allocLen) {
        u8 d[64] = {};
        std::size_t n = 4;
        d[3] = 8;      // block descriptor length
        d[4 + 5] = static_cast<u8>(blockSize_ >> 16);  // density 0, 0 blocks
        d[4 + 6] = static_cast<u8>(blockSize_ >> 8);   // (the whole medium),
        d[4 + 7] = static_cast<u8>(blockSize_);        // then the block size
        n += 8;
        const bool all = page == 0x3F;
        if (all || page == 0x01) {          // read error recovery
            d[n++] = 0x01; d[n++] = 10; n += 10;
        }
        if (all || page == 0x0D) {          // CD-ROM device parameters
            d[n++] = 0x0D; d[n++] = 6;
            d[n + 1] = 5;                   // inactivity timer: 125 ms
            d[n + 2] = 0; d[n + 3] = 60;    // seconds per minute
            d[n + 4] = 0; d[n + 5] = 75;    // frames per second
            n += 6;
        }
        if (all || page == 0x2A) {          // CD-ROM capabilities
            d[n++] = 0x2A; d[n++] = 12;
            d[n + 0] = 0x00;                // no CD-R/CD-RW reading claimed
            d[n + 2] = 0x01;                // audio play supported
            d[n + 6] = 0x02; d[n + 7] = 0x58;   // 600 KB/s
            n += 12;
        }
        if (n == 12 && !all) return check(0x05, 0x24);   // invalid field in CDB
        d[0] = static_cast<u8>(n - 1);      // mode data length excludes itself
        emitClamped(out, d, n, allocLen);
        return good();
    }

    void appendSense(std::vector<u8>& out, u8 allocLen) {
        u8 d[18] = {};
        d[0] = 0x70;         // current error, fixed format
        d[2] = senseKey_;
        d[7] = 10;           // additional sense length
        d[12] = asc_;
        emitClamped(out, d, sizeof d, allocLen ? allocLen : 18);
        senseKey_ = 0;       // sense reports once, then clears
        asc_ = 0;
    }

    void appendToc(std::vector<u8>& out, bool msf, u8 startTrack, u32 allocLen) {
        u8 d[20] = {};
        std::size_t n = 4;
        d[2] = 1;   // first track
        d[3] = 1;   // last track
        const auto putAddr = [&](u32 lba, u8* p) {
            if (msf) {
                const u32 f = lba + 150;   // 2-second pregap
                p[0] = 0;
                p[1] = static_cast<u8>(f / 4500);        // minutes
                p[2] = static_cast<u8>((f / 75) % 60);   // seconds
                p[3] = static_cast<u8>(f % 75);          // frames
            } else {
                p[0] = static_cast<u8>(lba >> 24); p[1] = static_cast<u8>(lba >> 16);
                p[2] = static_cast<u8>(lba >> 8);  p[3] = static_cast<u8>(lba);
            }
        };
        if (startTrack <= 1) {   // track 1: the data track
            d[n++] = 0; d[n++] = 0x14; d[n++] = 1; d[n++] = 0;
            putAddr(0, d + n); n += 4;
        }
        d[n++] = 0; d[n++] = 0x14; d[n++] = 0xAA; d[n++] = 0;   // lead-out
        putAddr(dataSectors2048(), d + n); n += 4;
        d[0] = static_cast<u8>((n - 2) >> 8);
        d[1] = static_cast<u8>(n - 2);
        emitClamped(out, d, n, allocLen);
    }

    static void emitClamped(std::vector<u8>& out, const u8* d, std::size_t n,
                            u32 allocLen) {
        const std::size_t len = (allocLen && allocLen < n) ? allocLen : n;
        out.insert(out.end(), d, d + len);
    }

    friend class IifxStateCodec;
    bool attached_ = false;
    int  id_ = 3;              // Apple's factory default CD-ROM bus ID
    std::vector<u8> media_;
    u32 blockSize_ = 2048;
    u8  senseKey_ = 0, asc_ = 0;
    bool unitAttention_ = false;
    bool prevented_ = false;
    bool ejectRequest_ = false;
    bool modeSelectPending_ = false;
};

} // namespace openmac
