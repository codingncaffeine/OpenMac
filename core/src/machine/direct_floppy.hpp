#pragma once

// The internal SuperDrive of a machine whose floppy chip is wired straight to
// the processor and worked by the ROM's own driver (Macintosh SE/30): the
// SWIM, one Sony mechanism on its first drive connector, nothing on the
// second, and the media in the drive.
//
// The driver does everything: in the chip's IWM mode it reads and writes
// 400K and 800K disks a nibble at a time and frames the sectors itself; for
// 1.4 MB disks it switches the chip to its second register set, where the
// chip finds the marks and computes the CRC. This class keeps the chip's
// inputs true to the mechanism before every access (the status line the
// phase lines address, the nibble under the head), moves bytes between the
// chip and the track, strobes the driver's commands into the mechanism, and
// puts what was written back into the disk's sectors.
//
// The order of work around an access is the Macintosh Classic's
// (machine.cpp), where it was worked out against that machine's ROM driver
// and Systems 6 and 7. Wiring: the drive's fourth address bit and head
// select is the machine's SEL line (VIA1 port A bit 5), which the board
// passes in; /ENBL1 is the internal drive.
//
// Reference: SWIM Chip User's Reference Rev 1.5 (Apple, 1988); Inside
// Macintosh vol. III; Guide to the Macintosh Family Hardware 2nd ed. ch. 9.
// Clean-room.

#include "openmac/types.hpp"

#include "dc42.hpp"
#include "gcr.hpp"
#include "iwm.hpp"
#include "macbinary.hpp"
#include "mfm.hpp"
#include "sony.hpp"

#include <cstdio>
#include <functional>
#include <utility>
#include <vector>

namespace openmac {

class DirectFloppy {
public:
    // `clockHz` is the rate of the `now` values every call takes: the
    // machine's own cycle counter.
    explicit DirectFloppy(u64 clockHz) {
        drive_.installed = true;
        drive_.doubleSided = true;
        drive_.superDrive = true;
        drive_.setClockHz(clockHz);
        absent_.installed = false;
        absent_.setClockHz(clockHz);
        reset();
    }

    // The machine's reset: chip and mechanism start over, a disk stays in.
    void reset() {
        swim_.reset();
        drive_.reset();
        absent_.reset();
        if (!media_.empty()) drive_.insert(&media_, readOnly_, highDensity(), false);
        lstrbPrev_ = actionPrev_ = false;
        writeMarkPrev_ = readMarkPrev_ = false;
        crcOut_ = 0xFFFF;
    }

    // The machine's SEL line.
    void setHeadSelect(bool high) { sel_ = high; }

    // ---- media ----
    // 400K, 800K or 1.44 MB of sectors, bare or inside a DiskCopy 4.2 or
    // MacBinary file. False = not a floppy image; the disk in the drive stays.
    bool insert(std::vector<u8> image, bool readOnly) {
        const char* container = "raw image";
        std::vector<u8> mbHeader, mbResource, dcHeader, dcTags;
        if (macbinary::isMacBinary(image)) {
            macbinary::Parts parts = macbinary::split(image);
            mbHeader = std::move(parts.header);
            mbResource = std::move(parts.resource);
            container = "MacBinary";
        }
        if (dc42::isDiskCopy(image)) {
            dc42::Parts parts = dc42::split(image);
            dcHeader = std::move(parts.header);
            dcTags = std::move(parts.tags);
            container = "DiskCopy 4.2";
        }
        const std::size_t bytes = image.size();
        if (bytes != 409600 && bytes != 819200 && bytes != 1474560) {
            say("floppy refused: %zu bytes (%s); need 400K, 800K or 1.44 MB", bytes, container);
            return false;
        }
        flushTrack(drive_);
        mbHeader_ = std::move(mbHeader);
        mbResource_ = std::move(mbResource);
        dcHeader_ = std::move(dcHeader);
        dcTags_ = std::move(dcTags);
        media_ = std::move(image);
        ejected_.clear();
        readOnly_ = readOnly;
        drive_.insert(&media_, readOnly_, highDensity(), true);
        say("floppy: %s disk in the internal SuperDrive (%s%s)",
            bytes == 1474560 ? "1.44 MB" : bytes == 819200 ? "800K" : "400K", container,
            readOnly ? ", write-protected" : "");
        return true;
    }

    // The disk leaves the drive now: the mechanism's own eject once the
    // driver has commanded it, and the host's Eject.
    void eject() {
        if (media_.empty()) return;
        flushTrack(drive_);
        ejected_ = std::move(media_);
        media_.clear();
        drive_.removeDisk();
        say("floppy: disk ejected from the internal SuperDrive");
        if (onEject) onEject();
    }

    bool present() const { return !media_.empty(); }
    bool readOnly() const { return readOnly_; }
    // The sectors as they are now (what is still in the track buffer written
    // back first); after an eject, the disk that came out.
    const std::vector<u8>& sectors() {
        flushTrack(drive_);
        return media_.empty() ? ejected_ : media_;
    }
    // The same in the file format the disk came in: what goes back to the
    // host file.
    std::vector<u8> fileImage() {
        std::vector<u8> result = sectors();
        if (result.empty()) return result;
        if (dcHeader_.size() == dc42::kHeaderSize) {
            dc42::Parts parts;
            parts.header = dcHeader_;
            parts.tags = dcTags_;
            result = dc42::rewrap(parts, result);
        }
        if (mbHeader_.size() == macbinary::kHeaderSize) {
            macbinary::Parts parts;
            parts.header = mbHeader_;
            parts.resource = mbResource_;
            result = macbinary::rewrap(parts, result);
        }
        return result;
    }

    u64 bytesRead() const { return dataBytes_; }
    u64 bytesWritten() const { return dataWrites_; }
    u32 tracksWritten() const { return trackWrites_; }
    u32 ejects() const { return ejects_; }
    Iwm& chip() { return swim_; }
    const Iwm& chip() const { return swim_; }
    SonyDrive& mechanism() { return drive_; }
    const SonyDrive& mechanism() const { return drive_; }

    // ---- the chip, as the processor sees it ----
    // One access to one of the chip's sixteen addresses.
    u8 access(int reg, bool write, u8 data, u64 now) {
        drive_.readOnly = readOnly_;
        drive_.hdMedia = highDensity();
        // ENABLE powers the mechanism the current register set addresses: in
        // IWM mode the enable latch, in the second register set Mode bit 7
        // together with a drive's own bit (SWIM reference p. 23).
        ++selected().selections;
        const bool enable = swim_.ismSelected()
            ? ((swim_.ismMode() & 0x80) != 0 && (swim_.ismMode() & 0x06) != 0)
            : swim_.motorOn();
        selected().setEnabled(enable, now);
        swim_.senseHigh = selected().sense(driveRegister(), now);
        updateTrack(now);
        SonyDrive& d = selected();
        if (swim_.ismSelected()) {
            serviceIsm(d, now);
        } else {
            // With Q7 high the chip is driving the write head, and reading
            // the surface then would consume the bytes being laid down. A
            // 1.4 MB disk gives the software reader nothing: its track is
            // legible only through the second register set.
            if (!(swim_.lines() & Iwm::kQ7) && !d.hdMedia) {
                swim_.dataByte = d.readNibble(now);
                if (swim_.dataByte) ++dataBytes_;
            }
            swim_.writeReady = d.writeReady(now);
        }
        const u8 result = swim_.access(reg & 15, write, data);
        if (swim_.ismSelected()) {
            serviceIsm(d, now);
        } else if (swim_.writeLatched) {
            swim_.writeLatched = false;
            d.writeNibble(now, swim_.writeData);
            ++dataWrites_;
        }
        strobe(now);
        return result;
    }

    std::function<void(const char* message)> onDiag;
    // After a disk has left the drive, by the guest's command or the host's.
    std::function<void()> onEject;
    // The driver strobed a command into a mechanism (register, data bit,
    // whether it was the internal drive).
    std::function<void(int reg, bool bit, bool internal)> onCommand;

private:
    bool highDensity() const { return media_.size() == 1474560u; }

    void say(const char* text) {
        if (onDiag) onDiag(text);
    }
    template <typename First, typename... Rest>
    void say(const char* format, First first, Rest... rest) {
        if (!onDiag) return;
        char message[160];
        std::snprintf(message, sizeof message, format, first, rest...);
        onDiag(message);
    }

    // The mechanism's register the phase lines and SEL address: CA2 CA1 CA0 SEL.
    int driveRegister() const { return swim_.driveRegister(sel_); }

    // Which connector the chip addresses: in IWM mode the drive-select latch,
    // in the second register set Mode bit 2. Nothing is on the second.
    SonyDrive& selected() {
        if (swim_.ismSelected()) return (swim_.ismMode() & 0x04) ? absent_ : drive_;
        return swim_.externalDrive() ? absent_ : drive_;
    }

    // Decode the track buffer back into the sector image if it was written.
    void flushTrack(SonyDrive& d) {
        if (!d.trackDirty() || !d.hasDisk() || d.readOnly) {
            d.clearTrackDirty();
            return;
        }
        std::vector<u8>& image = *d.image;
        u32 good = 0;
        const int sectors = d.hdMedia
            ? mfm::decodeTrack(d.trackData(), d.trackMarks(), image, d.cacheTrack, d.cacheSide, &good)
            : gcr::decodeTrack(d.trackData(), image, d.cacheTrack, image.size() > 440u * 1024u ? 2 : 1);
        d.clearTrackDirty();
        ++trackWrites_;
        if (d.hdMedia)
            say("floppy: track %d side %d written back, %d sectors (mask %08X)", d.cacheTrack, d.cacheSide, sectors, good);
        else
            say("floppy: track %d side %d written back, %d sectors", d.cacheTrack, d.cacheSide, sectors);
    }

    // Keep the track under the head in step with the head's position, the
    // side SEL selects and the disk in the drive.
    void updateTrack(u64) {
        SonyDrive& d = selected();
        // SEL both addresses the status registers and selects the head. The
        // drive latches the head when the phase lines address a read-data
        // line (Inside Macintosh III-35), not on every change of SEL.
        const int addr = driveRegister();
        if ((addr & 0xE) == 0x8) d.headUpper = (addr & 1) != 0;
        if (!d.hasDisk()) return;
        const std::vector<u8>& image = *d.image;
        const int side = d.headUpper ? 1 : 0;
        if (d.trackLoaded() && d.cacheTrack == d.track && d.cacheSide == side && d.cacheGen == d.mediaGen) return;
        // The head is about to leave a track the driver may have written.
        flushTrack(d);
        if (d.hdMedia) {
            if (d.track < 0 || d.track >= mfm::kTracks) {
                d.invalidateTrack();
                return;
            }
            std::vector<u8> bytes, marks;
            mfm::buildTrack(image, d.track, side, bytes, marks);
            d.setTrackData(std::move(bytes), std::move(marks));
        } else {
            const int sides = image.size() > 440u * 1024u ? 2 : 1;
            if (d.track < 0 || d.track > 79 || side >= sides) {
                d.invalidateTrack();
                return;
            }
            d.setTrackData(gcr::buildTrack(image, d.track, side, sides, sides == 2 ? 0x22 : 0x02));
        }
        d.cacheTrack = d.track;
        d.cacheSide = side;
        d.cacheGen = d.mediaGen;
    }

    // Move bytes between the second register set's FIFO and the surface.
    // Setting ACTION on a read parks the head on the next mark; from there
    // the FIFO follows the surface. On a write a byte the processor hands
    // over goes down at the head, and the Mark and CRC registers put a mark
    // byte or the two CRC bytes on the disk instead.
    void serviceIsm(SonyDrive& d, u64 now) {
        const bool action = swim_.ismAction();
        if (action && !actionPrev_) {
            if (!swim_.ismWriting()) d.syncToMark(now);
            crcOut_ = 0xFFFF;
            writeMarkPrev_ = false;
            readMarkPrev_ = false;
        }
        actionPrev_ = action;
        if (!action) return;

        if (!swim_.ismWriting()) {
            while (swim_.ismFifoOccupancy() < 2u) {
                u8 byte = 0;
                bool mark = false;
                if (!d.nextByte(now, &byte, &mark)) break;
                // The first mark of a run opens a field and restarts the CRC.
                if (mark && !readMarkPrev_) swim_.ismCrcReset();
                readMarkPrev_ = mark;
                swim_.ismCrcAdd(byte);
                if (!swim_.ismPushReadByte(byte, mark)) break;
                ++dataBytes_;
            }
            return;
        }
        swim_.ismDataReady = d.writeReady(now);
        if (swim_.ismWroteData || swim_.ismWroteMark) {
            const bool mark = swim_.ismWroteMark;
            // The sync bytes before an address mark precede the field and
            // are not in its CRC; only the first mark of a run restarts it.
            if (mark && !writeMarkPrev_) crcOut_ = 0xFFFF;
            writeMarkPrev_ = mark;
            d.writeByte(now, swim_.ismWritten, mark);
            crcOut_ = mfm::crc16Update(crcOut_, swim_.ismWritten);
            swim_.ismWroteData = swim_.ismWroteMark = false;
            ++dataWrites_;
        }
        if (swim_.ismWroteCrc) {
            swim_.ismWroteCrc = false;
            d.writeCrc(now, static_cast<u8>(crcOut_ >> 8), static_cast<u8>(crcOut_));
            dataWrites_ += 2;
        }
    }

    // A drive command is latched on the rising edge of LSTRB, with the
    // register in CA1 CA0 SEL and the data bit on CA2. An eject runs on the
    // mechanism's own clock once commanded, so it is advanced on every access.
    void strobe(u64 now) {
        const bool lstrb = swim_.lstrb();
        if (lstrb && !lstrbPrev_) {
            const int addr = driveRegister();
            SonyDrive& d = selected();
            d.command(addr & 7, (addr & 8) != 0, now);
            if (onCommand) onCommand(addr & 7, (addr & 8) != 0, &d == &drive_);
        }
        lstrbPrev_ = lstrb;
        SonyDrive& d = selected();
        d.tickEject(now);
        if (d.takeEjectRequest() && &d == &drive_) {
            ++ejects_;
            eject();
        }
    }

    Iwm swim_;
    SonyDrive drive_;              // the internal SuperDrive
    SonyDrive absent_;             // the second connector: nothing plugged in
    std::vector<u8> media_;        // the sectors of the disk in the drive
    std::vector<u8> ejected_;      // and of the one that last came out
    std::vector<u8> mbHeader_, mbResource_, dcHeader_, dcTags_;   // the file the sectors came in
    bool readOnly_ = false;
    bool sel_ = false;
    bool lstrbPrev_ = false;
    bool actionPrev_ = false;
    bool writeMarkPrev_ = false;
    bool readMarkPrev_ = false;
    u16 crcOut_ = 0xFFFF;
    u64 dataBytes_ = 0;
    u64 dataWrites_ = 0;
    u32 trackWrites_ = 0;
    u32 ejects_ = 0;
};

} // namespace openmac
