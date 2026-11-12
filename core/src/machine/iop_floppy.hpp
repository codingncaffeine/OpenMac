#pragma once

// The internal SuperDrive of a machine whose floppy chip sits behind an I/O
// processor (Macintosh IIfx, Quadra 900 and 950): the SWIM in its ISM mode
// as that processor's peripheral, one Sony mechanism on its first drive
// connector, and the media in it.
//
// The processor's firmware does everything a floppy driver does on other
// Macintoshes -- it steps the head, searches for address marks, moves sector
// bytes through the chip's two-byte FIFO under its data-request line -- so
// this class has no knowledge of sectors in flight. It keeps the chip's
// inputs true to the mechanism (SENSE, the read-data line), feeds the FIFO
// from the track under the head while the chip's ACTION bit is on, takes
// what the chip writes back to the track, and raises the processor's first
// data-request line when a byte can move.
//
// Wiring (Guide to the Macintosh Family Hardware, figure 9-14, and what the
// IIfx's downloaded firmware makes observable): /ENBL1 is the internal
// drive, mode bit 5 is the dedicated HDSEL output that both selects the head
// and forms the fourth bit of the drive's register address, DAT1BYTE goes to
// the processor's request line A. Clean-room.

#include "openmac/types.hpp"

#include "dc42.hpp"
#include "gcr.hpp"
#include "iop.hpp"
#include "iwm.hpp"
#include "macbinary.hpp"
#include "mfm.hpp"
#include "sony.hpp"

#include <algorithm>
#include <cstdio>
#include <functional>
#include <utility>
#include <vector>

namespace openmac {

class IifxStateCodec;

class IopFloppy {
public:
    // `clockHz` is the rate of the `now` values every call takes: the
    // machine's own cycle counter.
    IopFloppy(IoProcessor& iop, u64 clockHz) : iop_(iop), clockHz_(clockHz) {
        drive_.installed = true;
        drive_.doubleSided = true;
        drive_.superDrive = true;
        drive_.setClockHz(clockHz);
        reset();
    }

    // The machine's reset: chip and mechanism start over, a disk stays in.
    void reset() {
        swim_.reset();
        swim_.forceIsm();
        drive_.reset();
        if (!media_.empty()) drive_.insert(&media_, readOnly_, highDensity(), false);
        lstrbPrev_ = actionPrev_ = false;
        dmaIdle_ = false;
        writeMarkPrev_ = readMarkPrev_ = readSynced_ = false;
        crcOut_ = 0xFFFF;
        firmwareKnowsDisk_ = false;     // it starts over and finds the disk again
    }

    // ---- media ----
    // 400K, 800K or 1.44 MB of sectors, bare or inside a DiskCopy 4.2 or
    // MacBinary file. False = not a floppy image; the disk in the drive stays,
    // with the file format it came in. `announce` raises the mechanism's
    // disk-changed latch; a machine whose firmware has not made its first
    // mechanism poll yet passes false and lets that poll find the disk.
    bool insert(std::vector<u8> image, bool readOnly, bool announce = true) {
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
        flushTrack();
        mbHeader_ = std::move(mbHeader);
        mbResource_ = std::move(mbResource);
        dcHeader_ = std::move(dcHeader);
        dcTags_ = std::move(dcTags);
        media_ = std::move(image);
        ejected_.clear();
        ejectAsked_ = false;
        firmwareKnowsDisk_ = false;
        readOnly_ = readOnly;
        drive_.insert(&media_, readOnly_, highDensity(), announce);
        say("floppy: %s disk in the internal SuperDrive (%s%s)",
            bytes == 1474560 ? "1.44 MB" : bytes == 819200 ? "800K" : "400K", container,
            readOnly ? ", write-protected" : "");
        return true;
    }

    // The disk leaves the drive now, whatever the Macintosh thinks: the
    // mechanism's own eject, and the host's last resort.
    void eject() {
        ejectAsked_ = false;
        firmwareKnowsDisk_ = false;
        if (media_.empty()) return;
        flushTrack();
        ejected_ = std::move(media_);
        media_.clear();
        drive_.removeDisk();
        say("floppy: disk ejected from the internal SuperDrive");
        if (onEject) onEject();
    }

    // The host wants the disk back (the application's Eject command).
    //
    // This drive has no way to lose a disk except its own eject motor, and the
    // processor's firmware relies on that: once it has taken notice of a disk
    // it polls the disk-changed latch, not the disk-in-place line, until it
    // has ejected the disk itself. Pulling the disk out from under it leaves
    // it believing a disk is still there, and it then has the System eject
    // the next disk that goes in. So the request goes through the Macintosh:
    // the latch is raised with the disk still in place, the firmware reports
    // a changed disk, the System flushes the volume and tells the firmware to
    // eject, and the mechanism throws the disk out a second or two later.
    //
    // A disk the firmware has not touched yet is simply taken out. So is one
    // the Macintosh has not ejected four seconds after the request or after
    // the last byte moved to or from the disk, whichever is later (a System
    // that is stopped, or one that never answers the firmware).
    void requestEject(u64 now) {
        if (media_.empty()) return;
        if (!firmwareKnowsDisk_) {
            eject();
            return;
        }
        drive_.signalMediaChange();
        ejectAsked_ = true;
        ejectAskedAt_ = now;
        say("floppy: the Macintosh was asked to eject the disk");
    }
    bool ejectRequested() const { return ejectAsked_; }

    bool present() const { return !media_.empty(); }
    bool readOnly() const { return readOnly_; }
    // The sectors as they are now (what is still in the track buffer written
    // back first); after an eject, the disk that came out.
    const std::vector<u8>& sectors() {
        flushTrack();
        return media_.empty() ? ejected_ : media_;
    }
    // The same in the file format the disk came in (DiskCopy 4.2 and MacBinary
    // wrappers put back around the sectors): what goes back to the host file.
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
    // The disk in the drive as it stands, the track buffer left alone: for a
    // machine that also serves sector requests at driver level.
    std::vector<u8>& media() { return media_; }
    const std::vector<u8>& media() const { return media_; }
    std::size_t ejectedBytes() const { return ejected_.size(); }

    u64 bytesRead() const { return dataBytes_; }
    u64 bytesWritten() const { return dataWrites_; }
    u32 tracksWritten() const { return trackWrites_; }
    void clearCounters() { dataBytes_ = dataWrites_ = 0; trackWrites_ = 0; }
    int track() const { return drive_.track; }
    Iwm& chip() { return swim_; }
    const Iwm& chip() const { return swim_; }
    SonyDrive& mechanism() { return drive_; }
    const SonyDrive& mechanism() const { return drive_; }
    bool readSynced() const { return readSynced_; }

    // ---- the chip, as the I/O processor's peripheral ----
    u8 access(u8 offset, bool write, u8 value, u64 now) {
        drive_.readOnly = readOnly_;
        drive_.hdMedia = highDensity();
        const bool drive1Selected = (swim_.ismMode() & 0x02u) != 0;
        const bool drive2Selected = (swim_.ismMode() & 0x04u) != 0;
        // A mechanism in connector 1 only: connector 2 floats SENSE.
        const bool enabled = swim_.ismSelected() ? ((swim_.ismMode() & 0x80u) != 0 && drive1Selected)
                                                 : swim_.motorOn();
        drive_.setEnabled(enabled, now);

        const bool driveSel = (swim_.ismMode() & 0x20u) != 0;
        const int driveAddress = swim_.driveRegister(driveSel);
        swim_.senseHigh = drive_.sense(driveAddress, now);
        if (!drive1Selected || drive2Selected) swim_.senseHigh = true;
        // The mechanism's multiplexed status also reaches the chip on the
        // read-data line, which the firmware samples to tell drive kinds apart.
        swim_.readDataHigh = swim_.senseHigh;
        drive_.headUpper = driveSel;
        service(now);
        const u8 fifoBefore = swim_.ismFifoOccupancy();
        const u8 fifoHeadBefore = swim_.ismFifoHeadData();
        const u8 result = swim_.access(offset & 15u, write, value);
        if (onFifoChange && swim_.ismFifoOccupancy() != fifoBefore)
            onFifoChange(static_cast<u8>(offset & 15u), write, fifoHeadBefore, fifoBefore, swim_.ismFifoOccupancy());
        const bool newDriveSel = (swim_.ismMode() & 0x20u) != 0;
        drive_.headUpper = newDriveSel;
        service(now);
        if (onAccess)
            onAccess(Access{static_cast<u8>(offset & 15u), write, value, result, driveAddress, drive1Selected,
                            drive2Selected, enabled});

        const bool strobe = swim_.lstrb();
        if (strobe && !lstrbPrev_ && drive1Selected && !drive2Selected) {
            const int address = swim_.driveRegister(newDriveSel);
            const int trackBefore = drive_.track;
            const bool stepInBefore = drive_.stepIn;
            drive_.command(address & 7, (address & 8) != 0, now);
            if (drive_.hasDisk()) firmwareKnowsDisk_ = true;
            if (onCommand) onCommand(address, trackBefore, stepInBefore);
        }
        lstrbPrev_ = strobe;
        drive_.tickEject(now);
        if (drive_.takeEjectRequest()) eject();
        return result;
    }

    // Keeps the track under the head loaded and moves bytes between it and
    // the chip. Called around every chip access and from the machine's clock.
    void service(u64 now) {
        if (ejectAsked_ && now >= std::max(ejectAskedAt_, lastTransferAt_) + 4u * clockHz_) {
            say("floppy: the Macintosh did not eject the disk; taken out");
            eject();
        }
        updateTrack();
        serviceSurface(now);
    }

    // The byte-moving half alone, on the track already loaded.
    void serviceSurface(u64 now) {
        // Mode bits 1 and 2 are independent output enables, not a two-bit
        // drive number; the internal connector is /ENBL1.
        const bool drive1Enabled = (swim_.ismMode() & 0x82u) == 0x82u;
        const bool action = drive1Enabled && swim_.ismAction();
        if (action && !actionPrev_) {
            // ACTION starts the chip's mark search. The disk keeps turning
            // while ordinary gap bytes are ignored: the search does not jump
            // to the next mark, which leaves the firmware the time it has on
            // the real machine to arm its transfer.
            if (!swim_.ismWriting()) readSynced_ = false;
            crcOut_ = 0xFFFF;
            writeMarkPrev_ = readMarkPrev_ = false;
        }
        if (!action) readSynced_ = false;
        actionPrev_ = action;

        if (!action || !drive_.hasDisk()) {
            if (!dmaIdle_) {
                iop_.setDmaRequest(0, false);
                iop_.setDmaRequest(1, false);
                dmaIdle_ = true;
            }
            return;
        }
        dmaIdle_ = false;

        if (!swim_.ismWriting()) {
            while (swim_.ismFifoOccupancy() < 2u) {
                u8 byte = 0;
                bool mark = false;
                if (!drive_.nextByte(now, &byte, &mark)) break;
                if (!readSynced_) {
                    if (!mark) continue;
                    readSynced_ = true;
                    readMarkPrev_ = false;
                }
                if (mark && !readMarkPrev_) swim_.ismCrcReset();
                readMarkPrev_ = mark;
                swim_.ismCrcAdd(byte);
                if (!swim_.ismPushReadByte(byte, mark)) break;
                ++dataBytes_;
                lastTransferAt_ = now;
                if (onFifoPush) onFifoPush(byte, mark);
            }
        } else {
            swim_.ismDataReady = drive_.writeReady(now);
            if (swim_.ismWroteData || swim_.ismWroteMark) {
                const bool mark = swim_.ismWroteMark;
                if (mark && !writeMarkPrev_) crcOut_ = 0xFFFF;
                writeMarkPrev_ = mark;
                // The chip's write FIFO is two bytes deep and its handshake
                // says so: firmware that finds both slots free hands over two
                // bytes at once (the IIfx's does; the Quadra 950's writes one
                // at a time). The first goes into the cell under the head,
                // the second into the cell behind it. A third has no slot and
                // is lost, counted by the mechanism.
                const u64 cell = drive_.nextCellAt();
                const bool waits = now < cell && cell <= now + drive_.cyclesPerByte();
                drive_.writeByte(waits ? cell : now, swim_.ismWritten, mark);
                crcOut_ = mfm::crc16Update(crcOut_, swim_.ismWritten);
                swim_.ismWroteData = swim_.ismWroteMark = false;
                ++dataWrites_;
                lastTransferAt_ = now;
            }
            if (swim_.ismWroteCrc) {
                swim_.ismWroteCrc = false;
                // The two CRC bytes follow whatever is still waiting, cell
                // after cell.
                drive_.writeByte(std::max(now, drive_.nextCellAt()), static_cast<u8>(crcOut_ >> 8), false);
                drive_.writeByte(drive_.nextCellAt(), static_cast<u8>(crcOut_), false);
                dataWrites_ += 2;
            }
        }

        // The chip's "one byte" line is the processor's request line A; line
        // B is not connected.
        const bool ready = swim_.ismWriting() ? swim_.ismDataReady : swim_.ismFifoOccupancy() != 0;
        iop_.setDmaRequest(0, ready);
        iop_.setDmaRequest(1, false);
    }

    std::function<void(const char* message)> onDiag;
    // After a disk has left the drive, by the guest's command or the host's.
    std::function<void()> onEject;

    // Observation points for a machine's hardware trace; none changes what
    // the drive does. One chip access, after the surface has caught up with it:
    struct Access {
        u8 offset;
        bool write;
        u8 value, result;
        int driveAddress;                       // the mechanism register the chip's lines selected
        bool drive1Selected, drive2Selected;    // as they were before the access
        bool enabled;
    };
    std::function<void(const Access&)> onAccess;
    // The access itself changed how many bytes the chip's FIFO holds.
    std::function<void(u8 offset, bool write, u8 headBefore, u8 before, u8 after)> onFifoChange;
    // A byte from the disk went into the FIFO.
    std::function<void(u8 byte, bool mark)> onFifoPush;
    // The chip strobed a command into the mechanism.
    std::function<void(int address, int trackBefore, bool stepInBefore)> onCommand;

private:
    friend class IifxStateCodec;

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

    // Decode the track buffer back into the sector image if it was written.
    void flushTrack() {
        if (!drive_.trackDirty() || !drive_.hasDisk() || drive_.readOnly) {
            drive_.clearTrackDirty();
            return;
        }
        std::vector<u8>& image = *drive_.image;
        u32 good = 0;
        const int sectors = drive_.hdMedia
            ? mfm::decodeTrack(drive_.trackData(), drive_.trackMarks(), image, drive_.cacheTrack, drive_.cacheSide, &good)
            : gcr::decodeTrack(drive_.trackData(), image, drive_.cacheTrack, image.size() > 440u * 1024u ? 2 : 1);
        drive_.clearTrackDirty();
        ++trackWrites_;
        if (drive_.hdMedia)
            say("floppy: track %d side %d written back, %d sectors (mask %08X)", drive_.cacheTrack, drive_.cacheSide,
                sectors, good);
        else
            say("floppy: track %d side %d written back, %d sectors", drive_.cacheTrack, drive_.cacheSide, sectors);
    }

    // Encode the track the head is on, if it is not the one already loaded.
    void updateTrack() {
        if (!drive_.hasDisk()) return;
        const int side = drive_.headUpper ? 1 : 0;
        if (drive_.trackLoaded() && drive_.cacheTrack == drive_.track && drive_.cacheSide == side &&
            drive_.cacheGen == drive_.mediaGen)
            return;
        flushTrack();
        const std::vector<u8>& image = *drive_.image;
        if (drive_.track < 0 || drive_.track >= 80) {
            drive_.invalidateTrack();
            return;
        }
        if (drive_.hdMedia) {
            std::vector<u8> bytes, marks;
            mfm::buildTrack(image, drive_.track, side, bytes, marks);
            drive_.setTrackData(std::move(bytes), std::move(marks));
        } else {
            const int sides = image.size() > 440u * 1024u ? 2 : 1;
            if (side >= sides) {
                drive_.invalidateTrack();
                return;
            }
            drive_.setTrackData(gcr::buildTrack(image, drive_.track, side, sides, sides == 2 ? 0x22 : 0x02));
        }
        drive_.cacheTrack = drive_.track;
        drive_.cacheSide = side;
        drive_.cacheGen = drive_.mediaGen;
    }

    IoProcessor& iop_;
    u64 clockHz_;
    Iwm swim_;
    SonyDrive drive_;
    std::vector<u8> media_;        // the sectors of the disk in the drive
    std::vector<u8> ejected_;      // and of the one that last came out
    std::vector<u8> mbHeader_, mbResource_, dcHeader_, dcTags_;   // the file the sectors came in
    bool readOnly_ = false;
    bool lstrbPrev_ = false;
    bool actionPrev_ = false;
    bool dmaIdle_ = false;
    bool writeMarkPrev_ = false;
    bool readMarkPrev_ = false;
    bool readSynced_ = false;
    u16 crcOut_ = 0xFFFF;
    u64 dataBytes_ = 0;
    u64 dataWrites_ = 0;
    u32 trackWrites_ = 0;
    // Host-side bookkeeping for requestEject(), not machine state.
    bool firmwareKnowsDisk_ = false;   // it has commanded the mechanism with this disk in
    bool ejectAsked_ = false;
    u64 ejectAskedAt_ = 0;
    u64 lastTransferAt_ = 0;           // the last byte to or from the disk
};

} // namespace openmac
