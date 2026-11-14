// The saved states of the machines built on MacHostServices (the two Quadras
// and the Macintosh SE/30), in the format of the Macintosh IIfx's checkpoints
// (iifx/state_codec.hpp): a 64-byte header naming the ROM and the RAM size,
// then one tagged section per part. The chips the machines share are written
// by the IIfx codec's visitors; what those visitors leave out, because the
// IIfx never used it, follows in a section of this file's own, so the IIfx's
// format does not move. The Quadra 950's file comes first, the Quadra 650's
// after it, then the SE/30's; the storage all three get from MacHostServices
// is one visitor.

#include "openmac/quadra.hpp"
#include "openmac/quadra950.hpp"
#include "openmac/se30.hpp"

#include "../adb_via.hpp"
#include "../direct_floppy.hpp"
#include "../iifx/state_codec.hpp"
#include "../se30/video.hpp"
#include "../quadra950/egret.hpp"
#include "../quadra950/memctl.hpp"
#include "../scsinet.hpp"
#include "dafb.hpp"
#include "ncr53c96.hpp"
#include "pseudovia.hpp"

namespace openmac {

namespace {

// Format 2 adds what is waiting for the guest's next call (CALL section).
constexpr u32 k950StateVersion = 2;
constexpr char k950StateMagic[8] = {'O', 'M', 'Q', '9', '5', '0', 'S', 'T'};
constexpr u64 k950MaxRamBytes = 256ull * 1024ull * 1024ull;
// A 1 GB volume behind its driver partition.
constexpr u64 k950MaxMediaBytes = 1100ull * 1024ull * 1024ull;
constexpr u64 k950MaxVideoRam = 4ull * 1024ull * 1024ull;
// The shared visitors take the IIfx's format number: the current one.
constexpr u32 kSharedFormat = kStateVersion;

constexpr u32 k650StateVersion = 1;
constexpr char k650StateMagic[8] = {'O', 'M', 'Q', '6', '5', '0', 'S', 'T'};
constexpr u64 k650MaxRamBytes = 136ull * 1024ull * 1024ull;

constexpr u32 kSe30StateVersion = 1;
constexpr char kSe30StateMagic[8] = {'O', 'M', 'S', 'E', '3', '0', 'S', 'T'};
constexpr u64 kSe30MaxRamBytes = 128ull * 1024ull * 1024ull;

} // namespace

class QuadraStateCodec {
public:
    static std::vector<u8> save(const Quadra950Machine& source) {
        Quadra950Machine& machine = const_cast<Quadra950Machine&>(source);
        StateWriter payload;
        visit(payload, machine, k950StateVersion);

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
        if (version < 1 || version > k950StateVersion)
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
        visit(validator, machine, version);
        if (!validator.ok()) return fail(validator.error());
        if (!validator.atEnd()) return fail("saved state has trailing payload data");

        StateReader reader(payload, static_cast<std::size_t>(payloadBytes), true);
        visit(reader, machine, version);
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
    static void visitHostServices(Io& io, MacHostServices& m) {
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

    // What is waiting for the guest's next call. A format 1 file was written
    // by a machine that ran its guest calls where the clock tick fell:
    // nothing was waiting in it.
    template <typename Io>
    static void visitCalls(Io& io, MacHostServices& m, bool present) {
        if (present) {
            section(io, fourcc('C', 'A', 'L', 'L'));
            io.boolv(m.storageDue_); io.intv(m.hd2UnmountAsked_);
            io.boolv(m.beepAsked_);
            io.u8v(m.hcpu_.holdAtCalls);
        } else if (io.applying()) {
            m.storageDue_ = false;
            m.hd2UnmountAsked_ = 0;
            m.beepAsked_ = false;
            m.hcpu_.holdAtCalls = 0;
        }
        if (io.applying()) {
            m.hcpu_.heldAtCall = false;
            m.atCall_ = m.inGuestCall_ = m.askedInCall_ = false;
        }
    }

    template <typename Io>
    static void visit(Io& io, Quadra950Machine& m, u32 version) {
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
        visitCalls(io, m, version >= 2);
        section(io, fourcc('E', 'N', 'D', '!'));
    }

    // ---- the Quadra 650 -------------------------------------------------
public:
    static std::vector<u8> save650(const QuadraMachine& source) {
        QuadraMachine& machine = const_cast<QuadraMachine&>(source);
        StateWriter payload;
        visit650(payload, machine);

        u32 version = k650StateVersion;
        u32 headerBytes = 64;
        u32 endian = kEndianMarker;
        u32 flags = configuration650(machine);
        u64 romBytes = static_cast<u64>(machine.rom_.size());
        u64 romHash = fnv1a(machine.rom_.data(), machine.rom_.size());
        u64 ramBytes = static_cast<u64>(machine.ram_.size());
        u64 payloadBytes = static_cast<u64>(payload.data().size());
        u64 payloadHash = fnv1a(payload.data().data(), payload.data().size());

        StateWriter output;
        output.bytes(reinterpret_cast<const u8*>(k650StateMagic), sizeof k650StateMagic);
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

    static bool load650(QuadraMachine& machine, const u8* data, std::size_t size,
                        std::string* error) {
        const auto fail = [error](const std::string& text) {
            if (error) *error = text;
            return false;
        };
        if (!data || size < 64) return fail("saved state is too small");
        if (std::memcmp(data, k650StateMagic, sizeof k650StateMagic) != 0)
            return fail("not an OpenMac Quadra 650 saved state");

        StateReader header(data + sizeof k650StateMagic, size - sizeof k650StateMagic, true);
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
        if (version != k650StateVersion)
            return fail("unsupported Quadra 650 saved-state version");
        if (headerBytes != 64 || endian != kEndianMarker)
            return fail("invalid Quadra 650 saved-state header");
        if (payloadBytes != size - headerBytes)
            return fail("saved-state payload length mismatch");
        if (romBytes != machine.rom_.size() ||
            romHash != fnv1a(machine.rom_.data(), machine.rom_.size()))
            return fail("the state was saved with a different Quadra 650 ROM");
        if (ramBytes != machine.ram_.size())
            return fail("the state was saved with a different amount of RAM");
        if (flags != configuration650(machine))
            return fail("the state was saved with a different monitor");
        const u8* payload = data + headerBytes;
        if (payloadHash != fnv1a(payload, static_cast<std::size_t>(payloadBytes)))
            return fail("saved-state payload checksum mismatch");

        // Every section, length and sparse page is read once without touching
        // the machine, so a damaged file changes nothing.
        StateReader validator(payload, static_cast<std::size_t>(payloadBytes), false);
        visit650(validator, machine);
        if (!validator.ok()) return fail(validator.error());
        if (!validator.atEnd()) return fail("saved state has trailing payload data");

        StateReader reader(payload, static_cast<std::size_t>(payloadBytes), true);
        visit650(reader, machine);
        if (!reader.ok() || !reader.atEnd())
            return fail(reader.ok() ? "saved state has trailing payload data" : reader.error());

        // What is derived from the restored bytes rather than stored.
        if (!rebindDisk(*machine.disk_, machine.hd_, machine.scsiImage_, machine.hfsImageOffset_))
            return fail("saved hard disk lies outside its SCSI image");
        if (!rebindDisk(*machine.disk2_, machine.hd2_, machine.scsiImage2_,
                        machine.hfsImageOffset2_))
            return fail("saved second disk lies outside its SCSI image");
        machine.fd_->image = machine.floppy_.empty() ? nullptr : &machine.floppy_;
        machine.updateIpl();
        if (error) error->clear();
        return true;
    }

private:
    // What a state has to be loaded back into: the monitor's wiring. RAM is
    // in the header; the SCSI devices sit where the board put them.
    static u32 configuration650(QuadraMachine& machine) {
        const Dafb& dafb = *machine.dafb_;
        u32 word = (dafb.vramFitted_ >> 20) & 0xFu;
        word |= static_cast<u32>(dafb.monitor_.grounded & 0x7u) << 4;
        word |= static_cast<u32>(dafb.monitor_.pairs & 0x7u) << 7;
        return word;
    }

    template <typename Io>
    static void visitPseudoVia(Io& io, PseudoVia& via) {
        section(io, fourcc('P', 'V', 'I', 'A'));
        io.u8v(via.ifr_); io.u8v(via.ier_);
        io.u8v(via.ora_); io.u8v(via.orb_); io.u8v(via.ddra_); io.u8v(via.ddrb_);
        io.boolv(via.irqOut_);
        io.boolv(via.scsiDrqPrev_); io.boolv(via.slotPrev_);
        io.boolv(via.scsiIrqPrev_); io.boolv(via.ascPrev_);
        io.intv(via.rdBudget_); io.intv(via.drqDiagBudget_);
    }

    // What the IIfx's visitors leave out of the chips this machine has.
    template <typename Io>
    static void visitSharedExtras650(Io& io, QuadraMachine& machine) {
        section(io, fourcc('X', 'T', 'R', 'A'));
        io.boolv(machine.via1_->cb1_); io.boolv(machine.via1_->cb2_);
        for (auto& channel : machine.scc_->ch_) {
            io.u8v(channel.shiftByte); io.u8v(channel.bufByte);
            io.boolv(channel.rxAvail); io.u8v(channel.rxData);
            io.boolv(channel.zeroCount); io.u64v(channel.brgPhase);
        }
        Easc& sound = *machine.easc_;
        io.u8v(sound.playRecA_);
        io.bytes(sound.waveRam_, sizeof sound.waveRam_);
    }

    template <typename Io>
    static void visit650(Io& io, QuadraMachine& m) {
        section(io, fourcc('Q', '6', '5', '0'));
        io.sparse(m.ram_, k650MaxRamBytes);
        io.boolv(m.overlay_);
        for (u32& value : m.djmemcRegs_) io.u32v(value);
        for (u16& value : m.iosbRegs_) io.u16v(value);
        io.u8v(m.swimMode_); io.u8v(m.swimSetup_); io.u8v(m.swimPhases_);
        io.bytes(m.swimParams_, sizeof m.swimParams_);
        io.intv(m.swimParamPtr_);
        for (u16& value : m.sonicRegs_) io.u16v(value);
        io.bytes(m.macProm_, sizeof m.macProm_);

        // The floppy: the disk in the drive, one that left it and waits for
        // the host to write it back, one waiting for the drive to empty, and
        // the containers the medium arrived in.
        io.sparse(m.floppy_, kMaxSmallBlob);
        io.sparse(m.floppyEjected_, kMaxSmallBlob);
        io.boolv(m.floppyEjectPending_); io.intv(m.floppyEjectWait_);
        io.sparse(m.floppyPending_, kMaxSmallBlob);
        io.boolv(m.floppyPendingRO_); io.boolv(m.floppyPendingSet_);
        io.intv(m.floppyPendingDelay_);
        io.sparse(m.fdMbHeader_, kMaxSmallBlob);
        io.sparse(m.fdMbResource_, kMaxSmallBlob);
        io.sparse(m.fdDcHeader_, kMaxSmallBlob);
        io.sparse(m.fdDcTags_, kMaxSmallBlob);
        io.boolv(m.floppySel_); io.u8v(m.swimPhasePrev_);
        io.u32v(m.fdReadCount_); io.u32v(m.fdWriteCount_);

        io.boolv(m.adbArmed_); io.boolv(m.adbArmedInput_);
        io.intv(m.adbPending_); io.boolv(m.adbPendingInput_);
        io.intv(m.adbWakeStreak_); io.u32v(m.adbLastPollTotal_);

        io.u64v(m.totalCycles_); io.u64v(m.frameCounter_);
        io.intv(m.via1Remainder_); io.u64v(m.secondAcc_);
        io.intv(m.audioAcc_); io.intv(m.sliceInFrame_); io.u32v(m.vblAcc_);
        io.intv(m.ca2PulseSlices_); io.u32v(m.bootTraceFrame_);
        io.boolv(m.dafbIrq_);
        io.sparse(m.audioOut_, kMaxSmallBlob);
        // The log budgets: a loaded machine's log continues the saved one's.
        io.intv(m.eascDiagBudget_); io.intv(m.swimDiagBudget_);
        io.intv(m.cmdDiagBudget_); io.intv(m.primeDiagBudget_);
        io.intv(m.scsiDiagBudget_); io.intv(m.iplDiagBudget_);
        io.intv(m.via2DiagBudget_); io.intv(m.cdbDiagBudget_);
        io.intv(m.cdbListBudget_); io.intv(m.cdbSeen_);

        IifxStateCodec::visitCpu(io, m.cpu_);
        IifxStateCodec::visitVia(io, *m.via1_);
        visitPseudoVia(io, *m.via2_);
        IifxStateCodec::visitRtc(io, *m.rtc_);
        IifxStateCodec::visitAdb(io, *m.adb_);
        IifxStateCodec::visitScc(io, *m.scc_);
        IifxStateCodec::visitEasc(io, *m.easc_);
        visitNcr(io, *m.scsi_, fourcc('S', 'C', 'S', '0'));
        IifxStateCodec::visitSony(io, *m.fd_);
        visitDafb(io, *m.dafb_);
        visitHostServices(io, m);
        visitSharedExtras650(io, m);
        visitCalls(io, m, true);
        section(io, fourcc('E', 'N', 'D', '!'));
    }

    // ---- the Macintosh SE/30 --------------------------------------------
public:
    static std::vector<u8> saveSe30(const Se30Machine& source) {
        Se30Machine& machine = const_cast<Se30Machine&>(source);
        StateWriter payload;
        visitSe30(payload, machine);

        u32 version = kSe30StateVersion;
        u32 headerBytes = 64;
        u32 endian = kEndianMarker;
        u32 flags = configurationSe30(machine);
        u64 romBytes = static_cast<u64>(machine.rom_.size());
        u64 romHash = fnv1a(machine.rom_.data(), machine.rom_.size());
        u64 ramBytes = static_cast<u64>(machine.ram_.size());
        u64 payloadBytes = static_cast<u64>(payload.data().size());
        u64 payloadHash = fnv1a(payload.data().data(), payload.data().size());

        StateWriter output;
        output.bytes(reinterpret_cast<const u8*>(kSe30StateMagic), sizeof kSe30StateMagic);
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

    static bool loadSe30(Se30Machine& machine, const u8* data, std::size_t size,
                         std::string* error) {
        const auto fail = [error](const std::string& text) {
            if (error) *error = text;
            return false;
        };
        if (!data || size < 64) return fail("saved state is too small");
        if (std::memcmp(data, kSe30StateMagic, sizeof kSe30StateMagic) != 0)
            return fail("not an OpenMac Macintosh SE/30 saved state");

        StateReader header(data + sizeof kSe30StateMagic, size - sizeof kSe30StateMagic, true);
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
        if (version != kSe30StateVersion)
            return fail("unsupported Macintosh SE/30 saved-state version");
        if (headerBytes != 64 || endian != kEndianMarker)
            return fail("invalid Macintosh SE/30 saved-state header");
        if (payloadBytes != size - headerBytes)
            return fail("saved-state payload length mismatch");
        if (romBytes != machine.rom_.size() ||
            romHash != fnv1a(machine.rom_.data(), machine.rom_.size()))
            return fail("the state was saved with a different Macintosh SE/30 ROM");
        if (ramBytes != machine.ram_.size())
            return fail("the state was saved with a different amount of RAM");
        if (flags != configurationSe30(machine))
            return fail("the state was saved with a different video ROM");
        const u8* payload = data + headerBytes;
        if (payloadHash != fnv1a(payload, static_cast<std::size_t>(payloadBytes)))
            return fail("saved-state payload checksum mismatch");

        // Every section, length and sparse page is read once without touching
        // the machine, so a damaged file changes nothing.
        StateReader validator(payload, static_cast<std::size_t>(payloadBytes), false);
        visitSe30(validator, machine);
        if (!validator.ok()) return fail(validator.error());
        if (!validator.atEnd()) return fail("saved state has trailing payload data");

        StateReader reader(payload, static_cast<std::size_t>(payloadBytes), true);
        visitSe30(reader, machine);
        if (!reader.ok() || !reader.atEnd())
            return fail(reader.ok() ? "saved state has trailing payload data" : reader.error());

        // What is derived from the restored bytes rather than stored.
        machine.busWaitClocks_ = 0;
        machine.updateRamDecode();
        machine.systemHeapSeen_ = 0;
        machine.watchSystemRebuilt();     // the heap low memory names now is the one known
        if (!rebindDisk(*machine.disk_, machine.hd_, machine.scsiImage_, machine.hfsImageOffset_))
            return fail("saved hard disk lies outside its SCSI image");
        if (!rebindDisk(*machine.disk2_, machine.hd2_, machine.scsiImage2_,
                        machine.hfsImageOffset2_))
            return fail("saved second disk lies outside its SCSI image");
        DirectFloppy& floppy = *machine.floppy_;
        floppy.drive_.image = floppy.media_.empty() ? nullptr : &floppy.media_;
        machine.updateIpl();
        if (error) error->clear();
        return true;
    }

private:
    // What a state has to be loaded back into beyond ROM and RAM: the
    // screen's declaration ROM, by its bytes, and whether it is OpenMac's
    // own (its driver is in the System's heap).
    static u32 configurationSe30(Se30Machine& machine) {
        u32 word = static_cast<u32>(fnv1a(machine.videoRom_.data(), machine.videoRom_.size())) & ~1u;
        if (machine.ownVideoRom_) word |= 1u;
        return word;
    }

    template <typename Io>
    static void visitAdbViaPort(Io& io, AdbViaPort& port) {
        section(io, fourcc('A', 'D', 'B', 'V'));
        io.boolv(port.armed_); io.boolv(port.pendingInput_); io.intv(port.pending_);
        io.u32v(port.lastPollTotal_); io.intv(port.wakeStreak_);
    }

    // The floppy chip's wiring to its one drive: the disk in the drive, the
    // one that last came out, and the file the sectors arrived in.
    template <typename Io>
    static void visitDirectFloppy(Io& io, DirectFloppy& floppy) {
        section(io, fourcc('D', 'F', 'L', 'P'));
        io.sparse(floppy.media_, kMaxSmallBlob);
        io.sparse(floppy.ejected_, kMaxSmallBlob);
        io.sparse(floppy.mbHeader_, kMaxSmallBlob);
        io.sparse(floppy.mbResource_, kMaxSmallBlob);
        io.sparse(floppy.dcHeader_, kMaxSmallBlob);
        io.sparse(floppy.dcTags_, kMaxSmallBlob);
        io.boolv(floppy.readOnly_); io.boolv(floppy.sel_);
        io.boolv(floppy.lstrbPrev_); io.boolv(floppy.actionPrev_);
        io.boolv(floppy.writeMarkPrev_); io.boolv(floppy.readMarkPrev_);
        io.u16v(floppy.crcOut_); io.u64v(floppy.dataBytes_);
        io.u64v(floppy.dataWrites_); io.u32v(floppy.trackWrites_);
        io.u32v(floppy.ejects_);
    }

    template <typename Io>
    static void visitSe30Video(Io& io, Se30Video& video) {
        section(io, fourcc('S', 'V', 'I', 'D'));
        io.bytes(video.vram_.data(), video.vram_.size());
        io.boolv(video.mainBuffer_); io.boolv(video.enabled_); io.boolv(video.request_);
    }

    // What the IIfx's visitors leave out of the chips this machine has.
    template <typename Io>
    static void visitSharedExtrasSe30(Io& io, Se30Machine& machine) {
        section(io, fourcc('X', 'T', 'R', 'A'));
        for (Via6522* via : {machine.via1_.get(), machine.via2_.get()}) {
            io.boolv(via->cb1_); io.boolv(via->cb2_);
        }
        for (auto& channel : machine.scc_->ch_) {
            io.u8v(channel.shiftByte); io.u8v(channel.bufByte);
            io.boolv(channel.rxAvail); io.u8v(channel.rxData);
            io.boolv(channel.zeroCount); io.u64v(channel.brgPhase);
        }
        Ncr5380& scsi = *machine.scsi_;
        io.intv(scsi.settleLeft_);
        for (u32& value : scsi.diagCommandsById) io.u32v(value);
    }

    template <typename Io>
    static void visitSe30(Io& io, Se30Machine& m) {
        section(io, fourcc('S', 'E', '3', '0'));
        io.sameU32(m.bankA_); io.sameU32(m.bankB_);
        io.sparse(m.ram_, kSe30MaxRamBytes);
        io.boolv(m.overlay_); io.u8v(m.sizeBits_);
        io.boolv(m.via2Pb7_); io.u64v(m.screenInterrupts_);
        io.u64v(m.totalCycles_); io.u64v(m.frameCounter_);
        io.intv(m.viaRemainder_); io.intv(m.lineAcc_); io.intv(m.line_);
        io.u64v(m.secondAcc_); io.intv(m.secondPulseLines_);
        io.u32v(m.secondsUnanswered_);
        for (u32& value : m.unansweredAtFrame_) io.u32v(value);
        io.u64v(m.serialFreeAt_); io.u64v(m.fetchesCounted_);
        io.u64v(m.scsiPlainCycles_); io.u64v(m.scsiHandshakeCycles_);
        io.u32v(m.scsiHandshakeTimeouts_);
        io.boolv(m.floppyEjectPending_); io.boolv(m.floppyEjectAsked_);
        io.intv(m.floppyEjectFrames_);
        io.sparse(m.audioOut_, kMaxSmallBlob);
        io.sparse(m.audioStereo_, kMaxSmallBlob);
        // What the last instruction's bus cycle left running under the next.
        int tail = m.cpu_.pendingTail030();
        io.intv(tail);
        if (io.applying()) m.cpu_.setPendingTail030(tail);

        IifxStateCodec::visitCpu(io, m.cpu_);
        IifxStateCodec::visitVia(io, *m.via1_);
        IifxStateCodec::visitVia(io, *m.via2_);
        IifxStateCodec::visitRtc(io, *m.rtc_);
        IifxStateCodec::visitAdb(io, *m.adb_);
        visitAdbViaPort(io, *m.adbPort_);
        IifxStateCodec::visitScc(io, *m.scc_);
        IifxStateCodec::visitScsi(io, *m.scsi_);
        IifxStateCodec::visitAsc(io, *m.asc_);
        visitDirectFloppy(io, *m.floppy_);
        IifxStateCodec::visitSwim(io, m.floppy_->swim_, kSharedFormat);
        IifxStateCodec::visitSony(io, m.floppy_->drive_);
        IifxStateCodec::visitSony(io, m.floppy_->absent_);
        visitSe30Video(io, *m.video_);
        visitHostServices(io, m);
        visitSharedExtrasSe30(io, m);
        visitCalls(io, m, true);
        section(io, fourcc('E', 'N', 'D', '!'));
    }
};

std::vector<u8> Quadra950Machine::saveState() const {
    return QuadraStateCodec::save(*this);
}

bool Quadra950Machine::loadState(const u8* data, std::size_t size, std::string* error) {
    return QuadraStateCodec::load(*this, data, size, error);
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

std::vector<u8> Se30Machine::saveState() const {
    return QuadraStateCodec::saveSe30(*this);
}

bool Se30Machine::loadState(const u8* data, std::size_t size, std::string* error) {
    return QuadraStateCodec::loadSe30(*this, data, size, error);
}

bool Se30Machine::saveStateFile(const std::string& path, std::string* error) const {
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

bool Se30Machine::loadStateFile(const std::string& path, std::string* error) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        if (error) *error = "cannot open the saved state for reading: " + path;
        return false;
    }
    std::vector<u8> state{std::istreambuf_iterator<char>(input),
                          std::istreambuf_iterator<char>()};
    return loadState(state.data(), state.size(), error);
}

std::vector<u8> QuadraMachine::saveState() const {
    return QuadraStateCodec::save650(*this);
}

bool QuadraMachine::loadState(const u8* data, std::size_t size, std::string* error) {
    return QuadraStateCodec::load650(*this, data, size, error);
}

bool QuadraMachine::saveStateFile(const std::string& path, std::string* error) const {
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

bool QuadraMachine::loadStateFile(const std::string& path, std::string* error) {
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
