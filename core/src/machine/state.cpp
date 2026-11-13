// The Macintosh Classic's saved state, in the format of the Macintosh IIfx's
// checkpoints (iifx/state_codec.hpp): a 64-byte header naming the ROM and the
// RAM size, then one tagged section per part. The chips the machines share
// are written by the IIfx codec's visitors; the 68000 and the Classic's own
// board state are this file's.

#include "openmac/machine.hpp"

#include "iifx/state_codec.hpp"
#include "scsinet.hpp"

namespace openmac {

namespace {

constexpr u32 kClassicStateVersion = 1;
constexpr char kClassicStateMagic[8] = {'O', 'M', 'C', 'L', 'A', 'S', 'S', 'T'};
constexpr u64 kClassicMaxRamBytes = 4ull * 1024ull * 1024ull;
// The shared visitors take the IIfx's format number: the current one.
constexpr u32 kSharedFormat = kStateVersion;

} // namespace

class ClassicStateCodec {
public:
    static std::vector<u8> save(const Machine& source) {
        Machine& machine = const_cast<Machine&>(source);
        StateWriter payload;
        visit(payload, machine);

        u32 version = kClassicStateVersion;
        u32 headerBytes = 64;
        u32 endian = kEndianMarker;
        u32 flags = configuration(machine);
        u64 romBytes = static_cast<u64>(machine.rom_.size());
        u64 romHash = fnv1a(machine.rom_.data(), machine.rom_.size());
        u64 ramBytes = static_cast<u64>(machine.ram_.size());
        u64 payloadBytes = static_cast<u64>(payload.data().size());
        u64 payloadHash = fnv1a(payload.data().data(), payload.data().size());

        StateWriter output;
        output.bytes(reinterpret_cast<const u8*>(kClassicStateMagic), sizeof kClassicStateMagic);
        output.u32v(version);
        output.u32v(headerBytes);
        output.u32v(endian);
        output.u32v(flags);
        output.u64v(romBytes);
        output.u64v(romHash);
        output.u64v(ramBytes);
        output.u64v(payloadBytes);
        output.u64v(payloadHash);
        std::vector<u8> result = output.take();
        const std::vector<u8>& body = payload.data();
        result.insert(result.end(), body.begin(), body.end());
        return result;
    }

    static bool load(Machine& machine, const u8* data, std::size_t size, std::string* error) {
        const auto fail = [error](const std::string& text) {
            if (error) *error = text;
            return false;
        };
        if (!data || size < 64) return fail("saved state is too small");
        if (std::memcmp(data, kClassicStateMagic, sizeof kClassicStateMagic) != 0)
            return fail("not an OpenMac Macintosh Classic saved state");

        StateReader header(data + sizeof kClassicStateMagic, size - sizeof kClassicStateMagic,
                           true);
        u32 version = 0, headerBytes = 0, endian = 0, flags = 0;
        u64 romBytes = 0, romHash = 0, ramBytes = 0, payloadBytes = 0, payloadHash = 0;
        header.u32v(version);
        header.u32v(headerBytes);
        header.u32v(endian);
        header.u32v(flags);
        header.u64v(romBytes);
        header.u64v(romHash);
        header.u64v(ramBytes);
        header.u64v(payloadBytes);
        header.u64v(payloadHash);
        if (!header.ok()) return fail(header.error());
        if (version != kClassicStateVersion)
            return fail("unsupported Macintosh Classic saved-state version");
        if (headerBytes != 64 || endian != kEndianMarker)
            return fail("invalid Macintosh Classic saved-state header");
        if (payloadBytes != size - headerBytes)
            return fail("saved-state payload length mismatch");
        if (romBytes != machine.rom_.size() ||
            romHash != fnv1a(machine.rom_.data(), machine.rom_.size()))
            return fail("the state was saved with a different Macintosh Classic ROM");
        if (ramBytes != machine.ram_.size())
            return fail("the state was saved with a different amount of RAM");
        if (flags != configuration(machine))
            return fail("the state was saved with the external floppy drive, the CD-ROM "
                        "drive or the network adapter set up differently");
        const u8* payload = data + headerBytes;
        if (payloadHash != fnv1a(payload, static_cast<std::size_t>(payloadBytes)))
            return fail("saved-state payload checksum mismatch");

        // Every section, length and sparse page is read once without touching
        // the machine, so a damaged file changes nothing.
        StateReader validator(payload, static_cast<std::size_t>(payloadBytes), false);
        visit(validator, machine);
        if (!validator.ok()) return fail(validator.error());
        if (!validator.atEnd()) return fail("saved state has trailing payload data");

        StateReader reader(payload, static_cast<std::size_t>(payloadBytes), true);
        visit(reader, machine);
        if (!reader.ok() || !reader.atEnd())
            return fail(reader.ok() ? "saved state has trailing payload data" : reader.error());

        // What is derived from the restored bytes rather than stored.
        if (!rebindDisk(machine.scsi_->disk, machine.hd_, machine.scsiImage_,
                        machine.hfsImageOffset_))
            return fail("saved hard disk lies outside its SCSI image");
        if (!rebindDisk(*machine.disk2_, machine.hd2_, machine.scsiImage2_,
                        machine.hfsImageOffset2_))
            return fail("saved second disk lies outside its SCSI image");
        machine.cpu_.setIrqLevel(machine.via_->irqAsserted() ? 1 : 0);
        if (error) error->clear();
        return true;
    }

private:
    // What a state has to be loaded back into: the drives and adapters on
    // the machine. RAM is in the header.
    static u32 configuration(Machine& machine) {
        u32 word = machine.drive1_->installed ? 1u : 0u;
        if (machine.cdrom_->attachedState()) word |= 2u | (static_cast<u32>(machine.cdrom_->id()) << 4);
        if (machine.netdev_->attachedState()) word |= 4u | (static_cast<u32>(machine.netdev_->id()) << 8);
        return word;
    }

    // The volume the front end sees is a copy of its partition in the SCSI
    // image; the SCSI target reads the image itself.
    static bool rebindDisk(ScsiDisk& disk, std::vector<u8>& volume, std::vector<u8>& image,
                           u32 offset) {
        if (volume.empty()) {
            disk.image_ = nullptr;
            return true;
        }
        const u64 end = static_cast<u64>(offset) + volume.size();
        if (image.empty() || end > image.size()) return false;
        volume.assign(image.begin() + offset, image.begin() + static_cast<std::ptrdiff_t>(end));
        disk.image_ = &image;
        return true;
    }

    template <typename Io>
    static void visitCpu(Io& io, M68000& cpu) {
        section(io, fourcc('C', 'P', 'U', '0'));
        for (u32& value : cpu.d) io.u32v(value);
        for (u32& value : cpu.a) io.u32v(value);
        io.u32v(cpu.usp); io.u32v(cpu.ssp); io.u32v(cpu.pc);
        io.u16v(cpu.sr_);
        io.boolv(cpu.stopped); io.boolv(cpu.halted);
        io.intv(cpu.irqLevel_); io.u32v(cpu.instrStart_); io.u16v(cpu.ir_);
        for (u32& value : cpu.pcRing_) io.u32v(value);
        io.intv(cpu.pcRingPos_);
        io.intv(cpu.eaUndoReg_); io.u32v(cpu.eaUndoVal_);
        io.intv(cpu.eaUndoKind_); io.intv(cpu.eaFaultCycles_);
    }

    template <typename Io>
    static void visitWrapper(Io& io, Machine::MediumWrapper& wrap) {
        io.sparse(wrap.mbHeader, kMaxSmallBlob);
        io.sparse(wrap.mbResource, kMaxSmallBlob);
        io.sparse(wrap.dcHeader, kMaxSmallBlob);
        io.sparse(wrap.dcTags, kMaxSmallBlob);
    }

    // A drive's medium is a pointer to one of the machine's two floppy
    // buffers; whether it points anywhere is what is stored.
    template <typename Io>
    static void visitDrive(Io& io, SonyDrive& drive, std::vector<u8>& medium) {
        bool loaded = drive.image != nullptr;
        io.boolv(loaded);
        if (io.applying()) drive.image = loaded ? &medium : nullptr;
        IifxStateCodec::visitSony(io, drive);
    }

    template <typename Io>
    static void visit(Io& io, Machine& m) {
        section(io, fourcc('C', 'L', 'A', 'S'));
        io.sparse(m.ram_, kClassicMaxRamBytes);
        io.boolv(m.overlay_); io.boolv(m.screenAlt_); io.boolv(m.soundAlt_);
        io.u32v(m.bootTraceFrame_);
        io.u64v(m.totalCycles_); io.u64v(m.frameCounter_);
        io.u64v(m.lineTarget_); io.intv(m.viaRemainder_);
        io.u64v(m.secondAcc_); io.intv(m.ca2PulseLines_);
        io.intv(m.sccPtr_); io.bytes(m.sccRegs_, sizeof m.sccRegs_);
        io.boolv(m.romDiskKeymapHeld_);
        io.sparse(m.audioOut_, kMaxSmallBlob);
        io.boolv(m.adbArmed_); io.boolv(m.adbArmedInput_);
        io.intv(m.adbPending_); io.boolv(m.adbPendingInput_);
        io.intv(m.adbWakeStreak_); io.u32v(m.adbLastPollTotal_);

        // The floppies: the disk in each drive, one staged behind a disk that
        // is still in, one the drive threw out, and the containers the media
        // arrived in.
        section(io, fourcc('F', 'L', 'O', 'P'));
        io.sparse(m.floppy_, kMaxSmallBlob);
        io.sparse(m.floppy2_, kMaxSmallBlob);
        visitWrapper(io, m.floppyWrap_);
        visitWrapper(io, m.floppy2Wrap_);
        io.sparse(m.floppyStaged_, kMaxSmallBlob);
        visitWrapper(io, m.floppyStagedWrap_);
        io.boolv(m.floppyStagedRO_); io.boolv(m.floppyRO_);
        io.sparse(m.floppyEjected_, kMaxSmallBlob);
        io.sparse(m.mediumOut_, kMaxSmallBlob);
        io.u32v(m.floppyGen_); io.intv(m.floppyEjectSense_);
        io.intv(m.trackCacheTrack_); io.intv(m.trackCacheSide_);
        io.u32v(m.trackCacheGen_);
        io.boolv(m.ismActionPrev_); io.u16v(m.ismCrcOut_);
        io.boolv(m.ismWriteMarkPrev_); io.boolv(m.ismReadMarkPrev_);
        io.boolv(m.lstrbPrev_);
        io.u32v(m.floppyWrites_); io.u32v(m.floppyEjects_);
        io.u32v(m.iwmDataWrites_); io.u32v(m.iwmDataReads_); io.u32v(m.iwmDataBytes_);

        // The ROM's .Sony driver, as the machine found it in this startup.
        io.u32v(m.sonyOpenPc_); io.u32v(m.sonyPrimePc_);
        io.u32v(m.sonyControlPc_); io.u32v(m.sonyStatusPc_);
        io.u32v(m.sonyResultPc_); io.u32v(m.drvStatusAddr_);
        io.boolv(m.inSony_);
        u64 errSites = static_cast<u64>(m.gcrErrSites_.size());
        io.boundedU64(errSites, 256);
        if (io.applying()) m.gcrErrSites_.resize(static_cast<std::size_t>(errSites));
        for (std::size_t i = 0; i < static_cast<std::size_t>(errSites); ++i) {
            Machine::GcrErrSite site = i < m.gcrErrSites_.size() ? m.gcrErrSites_[i]
                                                               : Machine::GcrErrSite{};
            io.u32v(site.pc); io.u8v(site.code);
            if (io.applying()) m.gcrErrSites_[i] = site;
        }
        io.u32v(m.gcrErrLo_); io.u32v(m.gcrErrSpan_);

        // The two hard disks with their contents, and how far their mounts
        // have got.
        section(io, fourcc('D', 'I', 'S', 'K'));
        for (int seat = 0; seat < 2; ++seat) {
            std::vector<u8>& volume = seat ? m.hd2_ : m.hd_;
            std::vector<u8>& image = seat ? m.scsiImage2_ : m.scsiImage_;
            u64 volumeBytes = static_cast<u64>(volume.size());
            io.boundedU64(volumeBytes, kMaxMediaBytes);
            if (io.applying()) volume.assign(static_cast<std::size_t>(volumeBytes), 0);
            io.sparse(image, kMaxMediaBytes + 1024ull * 1024ull);
            io.u32v(seat ? m.hfsImageOffset2_ : m.hfsImageOffset_);
        }
        IifxStateCodec::visitDisk(io, *m.disk2_);
        io.boolv(m.hdRO_); io.boolv(m.hdInstalled_); io.intv(m.hdDriveNum_);
        io.boolv(m.hdMounted_); io.boolv(m.hdAutoMount_); io.boolv(m.scsiHandlesHd_);
        io.u32v(m.hdReads_); io.u32v(m.hdWrites_);
        io.u32v(m.diskEvtPosts_); io.u32v(m.diskEvtResult_);
        io.boolv(m.hd2Installed_); io.boolv(m.hd2Mounted_);
        io.boolv(m.hd2SeenOnLine_); io.intv(m.hd2OffSeconds_); io.boolv(m.hd2PutAway_);
        io.u32v(m.hd2MountTries_); io.boolv(m.bootHadHd2_);
        io.u8v(m.mountsWaiting_); io.intv(m.hd2UnmountAsked_);
        // The CD-ROM drive's own state; the disc is the loader's to put back.
        ScsiCdRom& cd = *m.cdrom_;
        io.u32v(cd.blockSize_); io.u8v(cd.senseKey_); io.u8v(cd.asc_);
        io.boolv(cd.unitAttention_); io.boolv(cd.prevented_);
        io.boolv(cd.ejectRequest_); io.boolv(cd.modeSelectPending_);
        io.boolv(cd.trayLoading_); io.boolv(cd.trayOpen_);

        visitCpu(io, m.cpu_);
        IifxStateCodec::visitVia(io, *m.via_);
        IifxStateCodec::visitRtc(io, *m.rtc_);
        IifxStateCodec::visitAdb(io, *m.adb_);
        IifxStateCodec::visitScsi(io, *m.scsi_);
        IifxStateCodec::visitSwim(io, *m.iwm_, kSharedFormat);
        visitDrive(io, *m.drive0_, m.floppy_);
        visitDrive(io, *m.drive1_, m.floppy2_);
        visitDrive(io, *m.driveBay2_, m.floppy2_);
        section(io, fourcc('E', 'N', 'D', '!'));
    }
};

std::vector<u8> Machine::saveState() const { return ClassicStateCodec::save(*this); }

bool Machine::loadState(const u8* data, std::size_t size, std::string* error) {
    return ClassicStateCodec::load(*this, data, size, error);
}

bool Machine::saveStateFile(const std::string& path, std::string* error) const {
    const std::vector<u8> state = saveState();
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        if (error) *error = "cannot open the saved state for writing: " + path;
        return false;
    }
    output.write(reinterpret_cast<const char*>(state.data()),
                 static_cast<std::streamsize>(state.size()));
    if (!output) {
        if (error) *error = "failed while writing the saved state: " + path;
        return false;
    }
    if (error) error->clear();
    return true;
}

bool Machine::loadStateFile(const std::string& path, std::string* error) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        if (error) *error = "cannot open the saved state for reading: " + path;
        return false;
    }
    std::vector<u8> state{std::istreambuf_iterator<char>(input),
                          std::istreambuf_iterator<char>()};
    return loadState(state.data(), state.size(), error);
}

} // namespace openmac
