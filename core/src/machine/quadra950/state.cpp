// The Quadra 950's saved state, in the format of the Macintosh IIfx's
// checkpoints (iifx/state_codec.hpp): a 64-byte header naming the ROM and the
// RAM size, then one tagged section per part. The chips the two machines
// share are written by the IIfx codec's visitors; what those visitors leave
// out, because the IIfx never used it, follows in a section of this file's
// own, so the IIfx's format does not move.

#include "openmac/quadra950.hpp"

#include "../iifx/state_codec.hpp"
#include "../quadra/dafb.hpp"
#include "../quadra/ncr53c96.hpp"
#include "../scsinet.hpp"
#include "egret.hpp"
#include "memctl.hpp"

namespace openmac {

namespace {

constexpr u32 k950StateVersion = 1;
constexpr char k950StateMagic[8] = {'O', 'M', 'Q', '9', '5', '0', 'S', 'T'};
constexpr u64 k950MaxRamBytes = 256ull * 1024ull * 1024ull;
// A 1 GB volume behind its driver partition.
constexpr u64 k950MaxMediaBytes = 1100ull * 1024ull * 1024ull;
constexpr u64 k950MaxVideoRam = 4ull * 1024ull * 1024ull;
// The shared visitors take the IIfx's format number: the current one.
constexpr u32 kSharedFormat = kStateVersion;

} // namespace

class Quadra950StateCodec {
public:
    static std::vector<u8> save(const Quadra950Machine& source) {
        Quadra950Machine& machine = const_cast<Quadra950Machine&>(source);
        StateWriter payload;
        visit(payload, machine);

        u32 version = k950StateVersion;
        u32 headerBytes = 64;
        u32 endian = kEndianMarker;
        u32 flags = configuration(machine);
        u64 romBytes = static_cast<u64>(machine.rom_.size());
        u64 romHash = fnv1a(machine.rom_.data(), machine.rom_.size());
        u64 ramBytes = static_cast<u64>(machine.ram_.size());
        u64 payloadBytes = static_cast<u64>(payload.data().size());
        u64 payloadHash = fnv1a(payload.data().data(), payload.data().size());

        StateWriter output;
        output.bytes(reinterpret_cast<const u8*>(k950StateMagic), sizeof k950StateMagic);
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

    static bool load(Quadra950Machine& machine, const u8* data, std::size_t size,
                     std::string* error) {
        const auto fail = [error](const std::string& text) {
            if (error) *error = text;
            return false;
        };
        if (!data || size < 64) return fail("saved state is too small");
        if (std::memcmp(data, k950StateMagic, sizeof k950StateMagic) != 0)
            return fail("not an OpenMac Quadra 950 saved state");

        StateReader header(data + sizeof k950StateMagic, size - sizeof k950StateMagic, true);
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
        if (version != k950StateVersion)
            return fail("unsupported Quadra 950 saved-state version");
        if (headerBytes != 64 || endian != kEndianMarker)
            return fail("invalid Quadra 950 saved-state header");
        if (payloadBytes != size - headerBytes)
            return fail("saved-state payload length mismatch");
        if (romBytes != machine.rom_.size() ||
            romHash != fnv1a(machine.rom_.data(), machine.rom_.size()))
            return fail("the state was saved with a different Quadra 950 ROM");
        if (ramBytes != machine.ram_.size())
            return fail("the state was saved with a different amount of RAM");
        if (flags != configuration(machine))
            return fail("the state was saved with different video RAM, a different "
                        "monitor or the second disk on the other SCSI bus");
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
        machine.memctl_->rebuild();
        if (!rebindDisk(*machine.disk_, machine.hd_, machine.scsiImage_, machine.hfsImageOffset_))
            return fail("saved hard disk lies outside its SCSI image");
        if (!rebindDisk(*machine.disk2_, machine.hd2_, machine.scsiImage2_,
                        machine.hfsImageOffset2_))
            return fail("saved second disk lies outside its SCSI image");
        IopFloppy& floppy = *machine.floppy_;
        floppy.drive_.image = floppy.media_.empty() ? nullptr : &floppy.media_;
        machine.updateIpl();
        if (error) error->clear();
        return true;
    }

private:
    // What a state has to be loaded back into: video RAM, the monitor's
    // wiring, and where each SCSI device sits.
    static u32 configuration(Quadra950Machine& machine) {
        const Dafb& dafb = *machine.dafb_;
        u32 word = (dafb.vramFitted_ >> 20) & 0xFu;
        word |= static_cast<u32>(dafb.monitor_.grounded & 0x7u) << 4;
        word |= static_cast<u32>(dafb.monitor_.pairs & 0x7u) << 7;
        // Three bits a device: the bus (bit 2) and its ID's low two bits are
        // not enough to tell every layout apart, so each device's place is
        // spelled out: bus * 8 + ID, in five bits, for the two disks.
        word |= place(machine, machine.disk_.get()) << 10;
        word |= place(machine, machine.disk2_.get()) << 15;
        word |= place(machine, machine.cdrom_.get()) << 20;
        return word;
    }

    static u32 place(Quadra950Machine& machine, const ScsiTarget* target) {
        for (u32 id = 0; id < 8; ++id) {
            if (machine.scsi0_->targets_[id] == target) return id;
            if (machine.scsi1_->targets_[id] == target) return 8u + id;
        }
        return 31u;
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
    static void visitMemoryController(Io& io, MemoryController950& memctl) {
        section(io, fourcc('M', 'C', 'T', 'L'));
        for (u32 units : memctl.bankUnits_) io.sameU32(units);
        io.bytes(memctl.bits_.data(), memctl.bits_.size());
        for (u32& value : memctl.other_) io.u32v(value);
    }

    template <typename Io>
    static void visitEgret(Io& io, EgretController& egret) {
        section(io, fourcc('E', 'G', 'R', 'T'));
        io.enum8(egret.state_);
        io.boolv(egret.sysSession_); io.boolv(egret.viaFull_);
        io.boolv(egret.xcvrSession_);
        io.sparse(egret.command_, kMaxSmallBlob);
        io.sparse(egret.reply_, kMaxSmallBlob);
        io.sizev(egret.replyPos_);
        io.boolv(egret.tickPending_); io.u8v(egret.oneSecondMode_);
        io.boolv(egret.autoPoll_); io.u32v(egret.seconds_);
        io.bytes(egret.pram_.data(), egret.pram_.size());
    }

    template <typename Io>
    static void visitDafb(Io& io, Dafb& dafb) {
        section(io, fourcc('D', 'A', 'F', 'B'));
        io.sparse(dafb.vram_, k950MaxVideoRam);
        io.bytes(dafb.clockRegs_, sizeof dafb.clockRegs_);
        for (u32& value : dafb.regs_) io.u32v(value);
        for (u32& value : dafb.swatch_) io.u32v(value);
        for (u32& value : dafb.clut_) io.u32v(value);
        io.u32v(dafb.clutAddr_); io.intv(dafb.clutPhase_);
        io.u32v(dafb.depthCtl_); io.u32v(dafb.ctl1_);
        io.boolv(dafb.vblEnabled_); io.boolv(dafb.vblPending_);
        io.boolv(dafb.cursorPending_); io.u32v(dafb.intMask_);
        io.u64v(dafb.clockShift_);
        // The log budgets: a loaded machine's log continues the saved one's.
        io.intv(dafb.senseDiag_); io.intv(dafb.clutDiag_);
        io.intv(dafb.ctlDiag_); io.intv(dafb.regDiag_);
    }

    template <typename Io>
    static void visitNcr(Io& io, Ncr53c96& chip, u32 tag) {
        section(io, tag);
        io.bytes(chip.fifo_, sizeof chip.fifo_);
        io.u32v(chip.fifoHead_); io.u32v(chip.fifoTail_);
        io.u8v(chip.tcountLo_); io.u8v(chip.tcountHi_);
        io.u32v(chip.counter_); io.boolv(chip.tcZero_);
        io.u8v(chip.command_); io.u8v(chip.destId_);
        io.u8v(chip.selTimeout_); io.u8v(chip.syncPer_); io.u8v(chip.syncOff_);
        io.u8v(chip.cfg1_); io.u8v(chip.cfg2_); io.u8v(chip.cfg3_);
        io.u8v(chip.clkFactor_); io.u8v(chip.test_); io.u8v(chip.align_);
        io.u8v(chip.phase_); io.u8v(chip.seqStep_); io.u8v(chip.intr_);
        io.boolv(chip.statusInt_); io.boolv(chip.dmaActive_);
        io.boolv(chip.cmdFeed_); io.boolv(chip.feedIsSelect_);
        io.boolv(chip.selAtn_); io.boolv(chip.feedDma_);
        io.sparse(chip.selCdb_, kMaxSmallBlob);
        io.u8v(chip.lastStatusDiag_); io.intv(chip.timeoutCountdown_);
        io.u8v(chip.queuedCmd_); io.boolv(chip.queuedValid_);
        io.sparse(chip.data_, kMaxSmallBlob); io.sizev(chip.dataPos_);
        io.u32v(chip.chunkLeft_);
        io.sparse(chip.writeBuf_, kMaxSmallBlob); io.u32v(chip.writeExpect_);
        io.u8v(chip.scsiStatus_);
        io.intv(chip.diagBudget_); io.intv(chip.eventBudget_);
        io.u32v(chip.diagWrites); io.u32v(chip.diagSelects); io.u32v(chip.diagCommands);
        io.bytes(chip.lastCdb, sizeof chip.lastCdb); io.intv(chip.lastCdbLen);
        // The selected target is a pointer; its place on the bus is what is stored.
        int selected = -1;
        if (!io.applying()) {
            for (int id = 0; id < 8; ++id)
                if (chip.selected_ && chip.targets_[id] == chip.selected_) selected = id;
        }
        io.intv(selected);
        if (io.applying())
            chip.selected_ = selected >= 0 && selected < 8 ? chip.targets_[selected] : nullptr;
    }

    // What the IIfx's visitors leave out of the chips both machines have.
    template <typename Io>
    static void visitSharedExtras(Io& io, Quadra950Machine& machine) {
        section(io, fourcc('X', 'T', 'R', 'A'));
        for (Via6522* via : {machine.via1_.get(), machine.via2_.get()}) {
            io.boolv(via->cb1_); io.boolv(via->cb2_);
        }
        for (auto& channel : machine.scc_->ch_) {
            io.u8v(channel.shiftByte); io.u8v(channel.bufByte);
            io.boolv(channel.rxAvail); io.u8v(channel.rxData);
            io.boolv(channel.zeroCount); io.u64v(channel.brgPhase);
        }
        Easc& sound = *machine.easc_;
        io.u8v(sound.playRecA_);
        io.bytes(sound.waveRam_, sizeof sound.waveRam_);
        IopFloppy& floppy = *machine.floppy_;
        io.boolv(floppy.firmwareKnowsDisk_); io.boolv(floppy.ejectAsked_);
        io.u64v(floppy.ejectAskedAt_); io.u64v(floppy.lastTransferAt_);
    }

    template <typename Io>
    static void visitFloppy(Io& io, IopFloppy& floppy) {
        section(io, fourcc('F', 'L', 'O', 'P'));
        io.sparse(floppy.media_, kMaxSmallBlob);
        io.sparse(floppy.ejected_, kMaxSmallBlob);
        io.sparse(floppy.mbHeader_, kMaxSmallBlob);
        io.sparse(floppy.mbResource_, kMaxSmallBlob);
        io.sparse(floppy.dcHeader_, kMaxSmallBlob);
        io.sparse(floppy.dcTags_, kMaxSmallBlob);
        io.boolv(floppy.readOnly_);
        io.boolv(floppy.lstrbPrev_); io.boolv(floppy.actionPrev_);
        // A host-side skip flag, not machine state: a loaded state may carry
        // live floppy DMA requests, so the next idle pass must clear them.
        if (io.applying()) floppy.dmaIdle_ = false;
        io.boolv(floppy.writeMarkPrev_); io.boolv(floppy.readMarkPrev_);
        io.boolv(floppy.readSynced_);
        io.u16v(floppy.crcOut_); io.u64v(floppy.dataBytes_);
        io.u64v(floppy.dataWrites_); io.u32v(floppy.trackWrites_);
    }

    // The storage the host serves: the two hard disks with their contents,
    // the drivers' hooked entry points, and the CD driver's bookkeeping. The
    // disc itself is the loader's to put back, as on the IIfx.
    template <typename Io>
    static void visitHostServices(Io& io, Quadra950Machine& m) {
        section(io, fourcc('H', 'O', 'S', 'T'));
        io.boolv(m.unitTableFresh_); io.boolv(m.sawNoApp_); io.boolv(m.inDriver_);
        for (u32& value : m.diskPrimePc_) io.u32v(value);
        for (u32& value : m.diskCtlPc_) io.u32v(value);
        for (u32& value : m.diskStatusPc_) io.u32v(value);
        io.u32v(m.cdDrvr_); io.u32v(m.cdPrimePc_); io.u32v(m.cdCtlPc_);
        io.u32v(m.cdStatusPc_); io.u32v(m.cdStatus_);
        u16 cdRefNum = std::bit_cast<u16>(m.cdRefNum_);
        io.u16v(cdRefNum);
        if (io.applying()) m.cdRefNum_ = std::bit_cast<s16>(cdRefNum);
        io.u16v(m.cdDrive_); io.u32v(m.cdHfsOffset_);
        io.boolv(m.cdMountPending_); io.intv(m.cdMountTries_);
        io.intv(m.cdInstallTries_); io.boolv(m.cdEjectPending_);
        io.intv(m.cdEjectTries_);
        ScsiCdRom& cd = *m.cdrom_;
        io.u32v(cd.blockSize_); io.u8v(cd.senseKey_); io.u8v(cd.asc_);
        io.boolv(cd.unitAttention_); io.boolv(cd.prevented_);
        io.boolv(cd.ejectRequest_); io.boolv(cd.modeSelectPending_);
        io.boolv(cd.trayLoading_); io.boolv(cd.trayOpen_);

        for (int seat = 0; seat < 2; ++seat) {
            std::vector<u8>& volume = seat ? m.hd2_ : m.hd_;
            std::vector<u8>& image = seat ? m.scsiImage2_ : m.scsiImage_;
            u64 volumeBytes = static_cast<u64>(volume.size());
            io.boundedU64(volumeBytes, k950MaxMediaBytes);
            if (io.applying()) volume.assign(static_cast<std::size_t>(volumeBytes), 0);
            io.sparse(image, k950MaxMediaBytes);
            io.u32v(seat ? m.hfsImageOffset2_ : m.hfsImageOffset_);
            io.boolv(seat ? m.hd2RO_ : m.hdRO_);
            IifxStateCodec::visitDisk(io, seat ? *m.disk2_ : *m.disk_);
        }
        io.boolv(m.hdAnnouncePending_); io.boolv(m.hd2AnnouncePending_);
        io.boolv(m.hd2SeenMounted_); io.intv(m.hd2OffSeconds_); io.boolv(m.hd2PutAway_);
        io.intv(m.hdAnnounceDelay_); io.boolv(m.announceInFlight_);
        io.u32v(m.hdMountPb_); io.intv(m.hdMountTries_);
        io.u32v(m.hdReadCount_); io.u32v(m.hdWriteCount_);
        io.intv(m.hdTraceBudget_); io.intv(m.hdErrBudget_);
    }

    template <typename Io>
    static void visit(Io& io, Quadra950Machine& m) {
        section(io, fourcc('Q', '9', '5', '0'));
        io.sparse(m.ram_, k950MaxRamBytes);
        io.boolv(m.overlay_);
        for (u16& value : m.sonicRegs_) io.u16v(value);
        io.bytes(m.macProm_, sizeof m.macProm_);
        for (u32& value : m.nubusCtlRegs_) io.u32v(value);
        io.boolv(m.dafbIrq_);
        for (bool& value : m.scsiIrq_) io.boolv(value);
        io.u32v(m.scsiBlindMisses_); io.intv(m.scsiPortDiag_);
        io.boolv(m.via2Pb7_); io.boolv(m.resetPending_);
        io.boolv(m.poweredOff_); io.boolv(m.powerDownRequested);
        io.u64v(m.totalCycles_); io.u64v(m.frameCounter_);
        io.intv(m.viaRemainder_); io.u64v(m.secondAcc_);
        io.intv(m.audioAcc_); io.intv(m.sliceInFrame_); io.u32v(m.vblAcc_);
        io.sparse(m.audioOut_, kMaxSmallBlob);

        IifxStateCodec::visitCpu(io, m.cpu_);
        IifxStateCodec::visitVia(io, *m.via1_);
        IifxStateCodec::visitVia(io, *m.via2_);
        visitMemoryController(io, *m.memctl_);
        visitEgret(io, *m.egret_);
        IifxStateCodec::visitIop(io, *m.sccIop_, fourcc('I', 'O', 'P', 'S'));
        IifxStateCodec::visitIop(io, *m.swimIop_, fourcc('I', 'O', 'P', 'M'));
        IifxStateCodec::visitScc(io, *m.scc_);
        IifxStateCodec::visitEasc(io, *m.easc_);
        visitNcr(io, *m.scsi0_, fourcc('S', 'C', 'S', '0'));
        visitNcr(io, *m.scsi1_, fourcc('S', 'C', 'S', '1'));
        IifxStateCodec::visitAdb(io, *m.adb_);
        IifxStateCodec::visitAdbBus(io, *m.adbBus_);
        visitFloppy(io, *m.floppy_);
        IifxStateCodec::visitSwim(io, m.floppy_->swim_, kSharedFormat);
        IifxStateCodec::visitSony(io, m.floppy_->drive_);
        visitDafb(io, *m.dafb_);
        visitHostServices(io, m);
        visitSharedExtras(io, m);
        section(io, fourcc('E', 'N', 'D', '!'));
    }
};

std::vector<u8> Quadra950Machine::saveState() const {
    return Quadra950StateCodec::save(*this);
}

bool Quadra950Machine::loadState(const u8* data, std::size_t size, std::string* error) {
    return Quadra950StateCodec::load(*this, data, size, error);
}

bool Quadra950Machine::saveStateFile(const std::string& path, std::string* error) const {
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

bool Quadra950Machine::loadStateFile(const std::string& path, std::string* error) {
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
