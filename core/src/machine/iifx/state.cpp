#include "openmac/iifx.hpp"

#include "../adb.hpp"
#include "../iwm.hpp"
#include "../rtc.hpp"
#include "../scsi.hpp"
#include "../sony.hpp"
#include "../via.hpp"
#include "../quadra/easc.hpp"
#include "../quadra/scc.hpp"
#include "adb_bus.hpp"
#include "asc.hpp"
#include "iop.hpp"
#include "nubus_video.hpp"
#include "oss.hpp"
#include "scsi_dma.hpp"

#include <algorithm>
#include <bit>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

namespace openmac {

namespace {

constexpr u32 kStateVersion = 16;
constexpr u32 kEndianMarker = 0x12345678u;
constexpr u32 kPageBytes = 4096;
constexpr u64 kMaxMediaBytes = 1024ull * 1024ull * 1024ull;
constexpr u64 kMaxRamBytes = 128ull * 1024ull * 1024ull;
constexpr u64 kMaxSmallBlob = 16ull * 1024ull * 1024ull;
constexpr char kStateMagic[8] = {'O', 'M', 'I', 'I', 'F', 'X', 'S', 'T'};

constexpr u32 fourcc(char a, char b, char c, char d) {
    return (static_cast<u32>(static_cast<u8>(a)) << 24) |
           (static_cast<u32>(static_cast<u8>(b)) << 16) |
           (static_cast<u32>(static_cast<u8>(c)) << 8) |
           static_cast<u32>(static_cast<u8>(d));
}

u64 fnv1a(const u8* data, std::size_t size) {
    u64 hash = 1469598103934665603ull;
    for (std::size_t index = 0; index < size; ++index)
        hash = (hash ^ data[index]) * 1099511628211ull;
    return hash;
}

class StateWriter {
public:
    bool ok() const { return true; }
    bool applying() const { return false; }

    void byte(u8 value) { data_.push_back(value); }
    void u8v(u8& value) { byte(value); }
    void u16v(u16& value) {
        byte(static_cast<u8>(value));
        byte(static_cast<u8>(value >> 8));
    }
    void u32v(u32& value) {
        for (int shift = 0; shift < 32; shift += 8)
            byte(static_cast<u8>(value >> shift));
    }
    void u64v(u64& value) {
        for (int shift = 0; shift < 64; shift += 8)
            byte(static_cast<u8>(value >> shift));
    }
    void s64v(s64& value) {
        u64 bits = std::bit_cast<u64>(value);
        u64v(bits);
    }
    void intv(int& value) {
        s32 signedValue = static_cast<s32>(value);
        u32 bits = std::bit_cast<u32>(signedValue);
        u32v(bits);
    }
    void boolv(bool& value) { byte(value ? 1u : 0u); }
    void doublev(double& value) {
        u64 bits = std::bit_cast<u64>(value);
        u64v(bits);
    }
    template <typename Enum>
    void enum8(Enum& value) {
        static_assert(std::is_enum_v<Enum>);
        u8 encoded = static_cast<u8>(value);
        u8v(encoded);
    }
    void sizev(std::size_t& value) {
        u64 encoded = static_cast<u64>(value);
        u64v(encoded);
    }
    void bytes(const u8* data, std::size_t size) {
        data_.insert(data_.end(), data, data + size);
    }
    void tag(u32 expected) { u32v(expected); }
    void sameU32(u32 actual) { u32v(actual); }
    void boundedU64(u64& value, u64) { u64v(value); }
    void boundedSize(std::size_t& value, std::size_t) { sizev(value); }

    void sparse(std::vector<u8>& bytes, u64) {
        u64 size = static_cast<u64>(bytes.size());
        u64v(size);
        u32 pageBytes = kPageBytes;
        u32v(pageBytes);

        u32 records = 0;
        for (std::size_t at = 0; at < bytes.size(); at += kPageBytes) {
            const std::size_t count = std::min<std::size_t>(kPageBytes,
                                                            bytes.size() - at);
            if (std::any_of(bytes.begin() + static_cast<std::ptrdiff_t>(at),
                            bytes.begin() + static_cast<std::ptrdiff_t>(at + count),
                            [](u8 value) { return value != 0; }))
                ++records;
        }
        u32v(records);

        u32 page = 0;
        for (std::size_t at = 0; at < bytes.size(); at += kPageBytes, ++page) {
            const std::size_t count = std::min<std::size_t>(kPageBytes,
                                                            bytes.size() - at);
            const u8 first = bytes[at];
            const bool allZero = first == 0 &&
                std::all_of(bytes.begin() + static_cast<std::ptrdiff_t>(at),
                            bytes.begin() + static_cast<std::ptrdiff_t>(at + count),
                            [](u8 value) { return value == 0; });
            if (allZero) continue;
            const bool fill = std::all_of(
                bytes.begin() + static_cast<std::ptrdiff_t>(at),
                bytes.begin() + static_cast<std::ptrdiff_t>(at + count),
                [first](u8 value) { return value == first; });
            u32 pageIndex = page;
            u32 valid = static_cast<u32>(count);
            u8 encoding = fill ? 1u : 2u;
            u32v(pageIndex);
            u32v(valid);
            u8v(encoding);
            if (fill) {
                u8 value = first;
                u8v(value);
            } else {
                data_.insert(data_.end(), bytes.begin() +
                    static_cast<std::ptrdiff_t>(at), bytes.begin() +
                    static_cast<std::ptrdiff_t>(at + count));
            }
        }
    }

    const std::vector<u8>& data() const { return data_; }
    std::vector<u8> take() { return std::move(data_); }

private:
    std::vector<u8> data_;
};

class StateReader {
public:
    StateReader(const u8* data, std::size_t size, bool apply)
        : data_(data), size_(size), apply_(apply) {}

    bool ok() const { return error_.empty(); }
    bool applying() const { return apply_; }
    bool atEnd() const { return position_ == size_; }
    const std::string& error() const { return error_; }

    void u8v(u8& value) {
        const u8 decoded = readByte();
        if (apply_ && ok()) value = decoded;
    }
    void u16v(u16& value) {
        u16 decoded = 0;
        for (int shift = 0; shift < 16; shift += 8)
            decoded |= static_cast<u16>(readByte()) << shift;
        if (apply_ && ok()) value = decoded;
    }
    void u32v(u32& value) {
        const u32 decoded = readU32();
        if (apply_ && ok()) value = decoded;
    }
    void u64v(u64& value) {
        const u64 decoded = readU64();
        if (apply_ && ok()) value = decoded;
    }
    void s64v(s64& value) {
        const u64 bits = readU64();
        if (apply_ && ok()) value = std::bit_cast<s64>(bits);
    }
    void intv(int& value) {
        const s32 decoded = std::bit_cast<s32>(readU32());
        if (apply_ && ok()) value = static_cast<int>(decoded);
    }
    void boolv(bool& value) {
        const u8 decoded = readByte();
        if (decoded > 1) fail("invalid checkpoint boolean");
        if (apply_ && ok()) value = decoded != 0;
    }
    void doublev(double& value) {
        const u64 bits = readU64();
        if (apply_ && ok()) value = std::bit_cast<double>(bits);
    }
    template <typename Enum>
    void enum8(Enum& value) {
        static_assert(std::is_enum_v<Enum>);
        const u8 decoded = readByte();
        if (apply_ && ok()) value = static_cast<Enum>(decoded);
    }
    void sizev(std::size_t& value) {
        const u64 decoded = readU64();
        if (decoded > static_cast<u64>(std::numeric_limits<std::size_t>::max()))
            fail("checkpoint size does not fit this host");
        if (apply_ && ok()) value = static_cast<std::size_t>(decoded);
    }
    void bytes(u8* data, std::size_t count) {
        if (!require(count)) return;
        if (apply_) std::memcpy(data, data_ + position_, count);
        position_ += count;
    }
    void tag(u32 expected) {
        if (readU32() != expected) fail("checkpoint section tag mismatch");
    }
    void sameU32(u32 actual) {
        if (readU32() != actual) fail("checkpoint fixed topology mismatch");
    }
    void boundedU64(u64& value, u64 maximum) {
        const u64 decoded = readU64();
        if (decoded > maximum) fail("checkpoint count exceeds its component limit");
        if (apply_ && ok()) value = decoded;
    }
    void boundedSize(std::size_t& value, std::size_t maximum) {
        const u64 decoded = readU64();
        if (decoded > maximum ||
            decoded > static_cast<u64>(std::numeric_limits<std::size_t>::max()))
            fail("checkpoint count exceeds its component limit");
        // This operation is intended for local loop counts and must be
        // available to the validation pass as well as the applying pass.
        if (ok()) value = static_cast<std::size_t>(decoded);
    }

    void sparse(std::vector<u8>& bytes, u64 maximum) {
        const u64 decodedSize = readU64();
        const u32 pageBytes = readU32();
        const u32 records = readU32();
        if (!ok()) return;
        if (decodedSize > maximum) {
            fail("checkpoint blob exceeds its component limit");
            return;
        }
        if (pageBytes != kPageBytes) {
            fail("unsupported checkpoint sparse-page size");
            return;
        }
        const u64 pages = (decodedSize + kPageBytes - 1u) / kPageBytes;
        if (records > pages) {
            fail("invalid checkpoint sparse-page count");
            return;
        }

        std::vector<u8> decoded;
        if (apply_) decoded.assign(static_cast<std::size_t>(decodedSize), 0);
        u32 previousPage = 0;
        bool firstRecord = true;
        for (u32 record = 0; record < records && ok(); ++record) {
            const u32 page = readU32();
            const u32 valid = readU32();
            const u8 encoding = readByte();
            if (page >= pages || (!firstRecord && page <= previousPage)) {
                fail("invalid checkpoint sparse-page index");
                break;
            }
            const u64 at = static_cast<u64>(page) * kPageBytes;
            const u32 expected = static_cast<u32>(
                std::min<u64>(kPageBytes, decodedSize - at));
            if (valid != expected || (encoding != 1 && encoding != 2)) {
                fail("invalid checkpoint sparse-page record");
                break;
            }
            if (encoding == 1) {
                const u8 fill = readByte();
                if (apply_ && ok())
                    std::fill_n(decoded.begin() + static_cast<std::ptrdiff_t>(at),
                                valid, fill);
            } else {
                if (!require(valid)) break;
                if (apply_)
                    std::memcpy(decoded.data() + static_cast<std::size_t>(at),
                                data_ + position_, valid);
                position_ += valid;
            }
            previousPage = page;
            firstRecord = false;
        }
        if (apply_ && ok()) bytes = std::move(decoded);
    }

private:
    bool require(std::size_t count) {
        if (!ok()) return false;
        if (count > size_ - position_) {
            fail("truncated checkpoint payload");
            return false;
        }
        return true;
    }
    u8 readByte() {
        if (!require(1)) return 0;
        return data_[position_++];
    }
    u32 readU32() {
        u32 result = 0;
        for (int shift = 0; shift < 32; shift += 8)
            result |= static_cast<u32>(readByte()) << shift;
        return result;
    }
    u64 readU64() {
        u64 result = 0;
        for (int shift = 0; shift < 64; shift += 8)
            result |= static_cast<u64>(readByte()) << shift;
        return result;
    }
    void fail(const char* message) {
        if (error_.empty()) error_ = message;
    }

    const u8* data_ = nullptr;
    std::size_t size_ = 0;
    std::size_t position_ = 0;
    bool apply_ = false;
    std::string error_;
};

template <typename Io>
void section(Io& io, u32 tag) {
    io.tag(tag);
    io.sameU32(1);
}

} // namespace

class IifxStateCodec {
public:
    static std::vector<u8> save(const IifxMachine& source) {
        IifxMachine& machine = const_cast<IifxMachine&>(source);
        StateWriter payload;
        visit(payload, machine, kStateVersion);

        u64 romHash = fnv1a(machine.rom_.data(), machine.rom_.size());
        if (machine.video_->genuineGc_)
            romHash = fnv1a(machine.video_->rom_.data(),
                            machine.video_->rom_.size()) ^
                      ((romHash << 1) | (romHash >> 63));
        const u64 payloadHash = fnv1a(payload.data().data(), payload.data().size());
        u32 version = kStateVersion;
        u32 headerBytes = 64;
        u32 endian = kEndianMarker;
        u32 flags = (machine.videoEnabled_ ? 1u : 0u) |
                    (machine.nativeStorage_ ? 2u : 0u);
        u64 romBytes = static_cast<u64>(machine.rom_.size());
        u64 ramBytes = static_cast<u64>(machine.ram_.size());
        u64 payloadBytes = static_cast<u64>(payload.data().size());
        u64 mutableRomHash = romHash;
        u64 mutablePayloadHash = payloadHash;

        StateWriter output;
        output.bytes(reinterpret_cast<const u8*>(kStateMagic),
                     sizeof kStateMagic);
        output.u32v(version);
        output.u32v(headerBytes);
        output.u32v(endian);
        output.u32v(flags);
        output.u64v(romBytes);
        output.u64v(mutableRomHash);
        output.u64v(ramBytes);
        output.u64v(payloadBytes);
        output.u64v(mutablePayloadHash);
        std::vector<u8> result = output.take();
        const std::vector<u8>& body = payload.data();
        result.insert(result.end(), body.begin(), body.end());
        return result;
    }

    static bool load(IifxMachine& machine, const u8* data, std::size_t size,
                     std::string* error) {
        const auto fail = [error](const std::string& text) {
            if (error) *error = text;
            return false;
        };
        if (!data || size < 64) return fail("checkpoint is too small");
        if (std::memcmp(data, kStateMagic, sizeof kStateMagic) != 0)
            return fail("not an OpenMac Macintosh IIfx checkpoint");

        StateReader header(data + sizeof kStateMagic,
                           size - sizeof kStateMagic, true);
        u32 version = 0, headerBytes = 0, endian = 0, flags = 0;
        u64 romBytes = 0, romHash = 0, ramBytes = 0;
        u64 payloadBytes = 0, payloadHash = 0;
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
        if (version < 1 || version > kStateVersion)
            return fail("unsupported Macintosh IIfx checkpoint version");
        if (headerBytes != 64 || endian != kEndianMarker)
            return fail("invalid Macintosh IIfx checkpoint header");
        if (payloadBytes != size - headerBytes)
            return fail("checkpoint payload length mismatch");
        u64 expectedRomHash = fnv1a(machine.rom_.data(), machine.rom_.size());
        if (version >= 4 && machine.video_->genuineGc_)
            expectedRomHash = fnv1a(machine.video_->rom_.data(),
                                    machine.video_->rom_.size()) ^
                              ((expectedRomHash << 1) |
                               (expectedRomHash >> 63));
        if (romBytes != machine.rom_.size() || romHash != expectedRomHash)
            return fail("checkpoint was created with a different Macintosh IIfx ROM");
        if (ramBytes != machine.ram_.size())
            return fail("checkpoint RAM configuration does not match this machine");
        const u32 expectedFlags = (machine.videoEnabled_ ? 1u : 0u) |
                                  (machine.nativeStorage_ ? 2u : 0u);
        if (flags != expectedFlags)
            return fail("checkpoint machine configuration does not match");
        const u8* payload = data + headerBytes;
        if (payloadHash != fnv1a(payload, static_cast<std::size_t>(payloadBytes)))
            return fail("checkpoint payload checksum mismatch");

        // A non-applying pass validates every section, length and sparse-page
        // record before the live machine is touched.
        StateReader validator(payload, static_cast<std::size_t>(payloadBytes), false);
        visit(validator, machine, version);
        if (!validator.ok()) return fail(validator.error());
        if (!validator.atEnd()) return fail("checkpoint has trailing payload data");

        StateReader reader(payload, static_cast<std::size_t>(payloadBytes), true);
        visit(reader, machine, version);
        if (!reader.ok() || !reader.atEnd())
            return fail(reader.ok() ? "checkpoint has trailing payload data"
                                    : reader.error());
        if (machine.ram_.size() != ramBytes)
            return fail("checkpoint RAM payload has the wrong size");

        const std::size_t logicalDiskBytes = machine.hardDisk_.size();
        if (logicalDiskBytes != 0) {
            if (machine.scsiImage_.empty())
                return fail("checkpoint hard disk has no SCSI backing image");
            if (machine.hardDiskIsWholeScsi_) {
                if (logicalDiskBytes != machine.scsiImage_.size())
                    return fail("checkpoint whole-disk size mismatch");
                machine.hardDisk_ = machine.scsiImage_;
            } else {
                const u64 end = static_cast<u64>(machine.hfsImageOffset_) +
                                logicalDiskBytes;
                if (end > machine.scsiImage_.size())
                    return fail("checkpoint HFS partition lies outside its disk");
                machine.hardDisk_.assign(
                    machine.scsiImage_.begin() + machine.hfsImageOffset_,
                    machine.scsiImage_.begin() +
                        static_cast<std::ptrdiff_t>(end));
            }
            machine.disk_->image_ = &machine.scsiImage_;
        } else {
            machine.hardDisk_.clear();
            machine.disk_->image_ = nullptr;
        }
        machine.floppyDrive_->image = machine.floppy_.empty()
            ? nullptr : &machine.floppy_;

        machine.accessLog_.clear();
        machine.hardwareTrace_.reset();
        machine.traceBusDepth_ = 0;
        machine.traceLastPc_ = 0;
        machine.traceSamePcCount_ = 0;
        machine.traceLastScsiPhase_ = -1;
        machine.traceLastScsiCommands_ = 0;
        machine.traceLastSwimMode_ = 0xFF;
        machine.traceLastSwimHandshake_ = 0xFF;
        machine.traceLastSwimFifo_ = 0xFF;
        machine.traceProtocolScsiPhase_ = -1;
        machine.traceAssertionMask_ = 0;
        machine.traceCheckpointPending_ = false;
        machine.traceCheckpointReason_.clear();

        // Re-derive external interrupt pins from the restored device state.
        machine.oss_->setLevel(Oss::Via1, machine.via_->irqAsserted());
        machine.oss_->setLevel(Oss::Scsi, machine.scsiDma_->irqAsserted());
        machine.oss_->setLevel(Oss::Sound, machine.soundIrq_);
        machine.oss_->setLevel(Oss::SccIop, machine.sccIop_->irqAsserted());
        machine.oss_->setLevel(Oss::IsmIop, machine.ismIop_->irqAsserted());
        machine.cpu_.setIrqLevel(machine.oss_->ipl());
        if (error) error->clear();
        return true;
    }

private:
    template <typename Io>
    static void visitCpu(Io& io, M68040& cpu) {
        section(io, fourcc('C', 'P', 'U', '3'));
        for (u32& value : cpu.d) io.u32v(value);
        for (u32& value : cpu.a) io.u32v(value);
        io.u32v(cpu.usp); io.u32v(cpu.isp); io.u32v(cpu.msp); io.u32v(cpu.pc);
        io.u16v(cpu.sr_);
        io.u32v(cpu.vbr); io.u32v(cpu.sfc); io.u32v(cpu.dfc);
        io.u32v(cpu.cacr); io.u32v(cpu.tc);
        io.u32v(cpu.itt0); io.u32v(cpu.itt1);
        io.u32v(cpu.dtt0); io.u32v(cpu.dtt1);
        io.u32v(cpu.urp); io.u32v(cpu.srp); io.u32v(cpu.mmusr);
        io.u32v(cpu.caar); io.u32v(cpu.tt0); io.u32v(cpu.tt1);
        io.u64v(cpu.crp); io.u64v(cpu.srp030);
        for (double& value : cpu.fp) io.doublev(value);
        io.u32v(cpu.fpcr); io.u32v(cpu.fpsr); io.u32v(cpu.fpiar);
        io.boolv(cpu.stopped); io.boolv(cpu.halted);
        io.u32v(cpu.ext020Count); io.u32v(cpu.lastFaultAddr);
        io.intv(cpu.irqLevel_); io.u32v(cpu.instrStart_); io.u16v(cpu.ir_);
        for (u32& value : cpu.pcRing_) io.u32v(value);
        io.intv(cpu.pcRingPos_);
        for (u32& value : cpu.snapD_) io.u32v(value);
        for (u32& value : cpu.snapA_) io.u32v(value);
        io.u16v(cpu.snapSR_); io.intv(cpu.eaExtra_);
        io.boolv(cpu.lastEaFull030_); io.boolv(cpu.fpuUsed_);
        for (auto& cache : {&cpu.instructionCache030_, &cpu.dataCache030_}) {
            for (auto& line : *cache) {
                io.u32v(line.tag); io.u8v(line.fc); io.u8v(line.valid);
                for (u32& value : line.data) io.u32v(value);
            }
        }
        io.u64v(cpu.cacheStats030_.instructionHits);
        io.u64v(cpu.cacheStats030_.instructionMisses);
        io.u64v(cpu.cacheStats030_.dataHits);
        io.u64v(cpu.cacheStats030_.dataMisses);
        io.u64v(cpu.cacheStats030_.burstLongwords);
        for (auto& entry : cpu.tlb_) {
            io.u32v(entry.tag); io.u32v(entry.phys);
            io.boolv(entry.writable); io.boolv(entry.modified);
            io.boolv(entry.cacheInhibit); io.u8v(entry.pageShift);
            io.u8v(entry.fc);
        }
        io.intv(cpu.tlbNext030_);
    }

    template <typename Io>
    static void visitR65(Io& io, R65C02& cpu) {
        io.u8v(cpu.a); io.u8v(cpu.x); io.u8v(cpu.y); io.u8v(cpu.s);
        io.u8v(cpu.p); io.u16v(cpu.pc);
        io.boolv(cpu.waiting); io.boolv(cpu.stopped);
        io.u64v(cpu.instructions); io.u64v(cpu.cycles); io.u64v(cpu.brkCount);
        io.u8v(cpu.lastOpcode);
        io.boolv(cpu.irq_); io.boolv(cpu.nmiLine_); io.boolv(cpu.nmiPending_);
        for (u16& value : cpu.recentPc_) io.u16v(value);
        for (u16& value : cpu.preBrkPc_) io.u16v(value);
        u32 head = cpu.recentPcHead_;
        io.u32v(head);
        if (io.applying()) cpu.recentPcHead_ = head;
    }

    template <typename Io>
    static void visitVia(Io& io, Via6522& via) {
        section(io, fourcc('V', 'I', 'A', '1'));
        io.u8v(via.orb_); io.u8v(via.ora_); io.u8v(via.ddrb_); io.u8v(via.ddra_);
        io.u16v(via.t1c_); io.u16v(via.t1l_); io.u16v(via.t2c_);
        io.u8v(via.t2ll_); io.boolv(via.t1Running_); io.boolv(via.t2Running_);
        io.u8v(via.sr_); io.u8v(via.acr_); io.u8v(via.pcr_);
        io.u8v(via.ifr_); io.u8v(via.ier_);
        io.boolv(via.ca1_); io.boolv(via.ca2_); io.boolv(via.pb7_);
        io.intv(via.srTicks_); io.boolv(via.srInput_);
    }

    template <typename Io>
    static void visitRtc(Io& io, Rtc& rtc) {
        section(io, fourcc('R', 'T', 'C', ' '));
        io.u32v(rtc.seconds_); io.bytes(rtc.xpram_.data(), rtc.xpram_.size());
        io.enum8(rtc.state_); io.u8v(rtc.shift_); io.intv(rtc.bits_);
        io.u8v(rtc.cmd_); io.u8v(rtc.extAddr_);
        io.boolv(rtc.extended_); io.boolv(rtc.writeProtect_);
        io.boolv(rtc.lastClock_); io.boolv(rtc.enabled_); io.boolv(rtc.dataOut_);
        io.u8v(rtc.outShift_); io.intv(rtc.outBits_);
    }

    template <typename Io>
    static void visitOss(Io& io, Oss& oss) {
        section(io, fourcc('O', 'S', 'S', ' '));
        io.bytes(oss.priority_.data(), oss.priority_.size());
        io.u16v(oss.levelMask_); io.u16v(oss.latchMask_);
        io.u8v(oss.romControl_); io.boolv(oss.poweredOff_);
        oss.iplDirty_ = true;   // the cached level belongs to the old state
    }

    template <typename Io>
    static void visitIop(Io& io, IifxIop& iop, u32 tag) {
        section(io, tag);
        io.bytes(iop.ram_.data(), iop.ram_.size());
        visitR65(io, iop.cpu_);
        io.u16v(iop.address_); io.u8v(iop.status_); io.boolv(iop.resetHeld_);
        io.u16v(iop.timerLatch_); io.s64v(iop.timerTicks_);
        io.u64v(iop.timerSinceExpiry_);
        for (auto& dma : iop.dma_) {
            io.u8v(dma.control); io.u16v(dma.map); io.u16v(dma.count);
            io.boolv(dma.request);
        }
        io.u8v(iop.sccControl_); io.u8v(iop.ioControl_);
        io.u8v(iop.timerDpllControl_); io.u8v(iop.interruptMask_);
        io.u8v(iop.interruptFlags_); io.s64v(iop.cpuPhase_);
        io.u64v(iop.cpuCycles_); io.u64v(iop.asicPhase_);
        io.u64v(iop.dmaDivider_); io.boolv(iop.hostMemoryAccess_);
        io.u32v(iop.hostReads); io.u32v(iop.hostWrites);
        io.u32v(iop.ramReads); io.u32v(iop.ramWrites);
        io.u32v(iop.mailboxCommands); io.u64v(iop.dmaTransfers);
        io.u64v(iop.timerExpirations);
    }

    template <typename Io>
    static void visitScc(Io& io, Scc8530& scc) {
        section(io, fourcc('S', 'C', 'C', ' '));
        for (auto& channel : scc.ch_) {
            io.bytes(channel.wr, sizeof channel.wr);
            io.boolv(channel.txEmpty); io.boolv(channel.underrun);
            io.boolv(channel.underrunArmed); io.boolv(channel.bufFull);
            io.intv(channel.shiftTimer); io.intv(channel.feedTimer);
            io.boolv(channel.extIp); io.boolv(channel.txIp); io.boolv(channel.rxIp);
            io.u8v(channel.rr10); io.u32v(channel.bytesSent);
        }
        io.intv(scc.ptr_); io.u8v(scc.wr2_); io.u8v(scc.wr9_);
        io.intv(scc.cmdBudget_); io.intv(scc.regBudget_);
        io.intv(scc.dataBudget_); io.intv(scc.irqBudget_);
    }

    template <typename Io>
    static void visitDisk(Io& io, ScsiDisk& disk) {
        io.boolv(disk.readOnly); io.intv(disk.id_);
        io.u8v(disk.senseKey_); io.u32v(disk.writeLba_);
    }

    template <typename Io>
    static void visitScsi(Io& io, Ncr5380& scsi) {
        section(io, fourcc('S', 'C', 'S', 'I'));
        visitDisk(io, scsi.disk);
        io.u32v(scsi.diagReads); io.u32v(scsi.diagWrites);
        io.u32v(scsi.diagSelects); io.u32v(scsi.diagCommands);
        io.u32v(scsi.diagDataInBytes); io.u32v(scsi.diagDataOutBytes);
        io.bytes(scsi.diagLastCdb, sizeof scsi.diagLastCdb);
        io.intv(scsi.diagLastCdbLen);
        io.bytes(&scsi.diagCdbHist[0][0], sizeof scsi.diagCdbHist);
        io.intv(scsi.diagCdbHistLen);
        for (u16& value : scsi.diagWriteTrace) io.u16v(value);
        io.intv(scsi.diagWriteTraceLen);
        io.u8v(scsi.odr_); io.u8v(scsi.icr_); io.u8v(scsi.mr_);
        io.u8v(scsi.tcr_); io.u8v(scsi.ser_); io.enum8(scsi.phase_);
        io.bytes(scsi.cdb_, sizeof scsi.cdb_);
        io.intv(scsi.cdbPos_); io.intv(scsi.cdbLen_);
        io.sparse(scsi.xfer_, kMaxSmallBlob); io.sizev(scsi.xferPos_);
        io.u8v(scsi.status_); io.boolv(scsi.status_ready_);
        io.boolv(scsi.msg_ready_); io.boolv(scsi.writeMode_);
        io.boolv(scsi.irq_); io.enum8(scsi.dmaStart_);
        io.enum8(scsi.ackCompletion_);
        io.sameU32(static_cast<u32>(scsi.nTargets_));
        int selected = -1;
        if (!io.applying()) {
            for (int index = 0; index < scsi.nTargets_; ++index)
                if (scsi.targets_[index] == scsi.sel_) selected = index;
        }
        io.intv(selected);
        if (io.applying())
            scsi.sel_ = selected >= 0 && selected < scsi.nTargets_
                ? scsi.targets_[selected] : nullptr;
    }

    template <typename Io>
    static void visitScsiDma(Io& io, IifxScsiDma& dma) {
        section(io, fourcc('S', 'D', 'M', 'A'));
        io.u32v(dma.controlWrite_); io.u32v(dma.count_);
        io.u32v(dma.address_); io.u32v(dma.watchdogReload_);
        io.u32v(dma.watchdogCounter_); io.bytes(dma.fifo_.data(), dma.fifo_.size());
        io.u8v(dma.fifoValid_); io.boolv(dma.dmaActive_);
        io.enum8(dma.direction_); io.boolv(dma.watchdogPending_);
        io.boolv(dma.busError_); io.boolv(dma.wonArbitration_);
        io.boolv(dma.resetCommandHigh_); io.boolv(dma.lastInterrupt_);
        io.u64v(dma.transferPhase_); io.u64v(dma.watchdogPhase_);
    }

    template <typename Io>
    static void visitEasc(Io& io, Easc& sound) {
        section(io, fourcc('E', 'A', 'S', 'C'));
        io.bytes(sound.bufA_, sizeof sound.bufA_);
        io.bytes(sound.bufB_, sizeof sound.bufB_);
        io.u32v(sound.headA_); io.u32v(sound.tailA_);
        io.u32v(sound.headB_); io.u32v(sound.tailB_);
        io.u8v(sound.lastA_); io.u8v(sound.lastB_);
        io.u8v(sound.mode_); io.u8v(sound.control_); io.u8v(sound.fifoMode_);
        io.u8v(sound.irqStatus_); io.u8v(sound.volume_);
        io.bytes(sound.ext_, sizeof sound.ext_);
        io.u8v(sound.dfacSettings_); io.u8v(sound.dfacShift_);
        io.boolv(sound.dfacPrevClock_); io.boolv(sound.dfacPrevLatch_);
        io.u32v(sound.pushesA_); io.u32v(sound.pushesB_);
        io.intv(sound.diagBudget_); io.boolv(sound.linePending_);
        io.bytes(sound.xaParam_, sizeof sound.xaParam_);
        for (int& value : sound.xaPos_) io.intv(value);
        for (int& value : sound.xaSubpos_) io.intv(value);
        io.bytes(sound.xaByte_, sizeof sound.xaByte_);
        for (int& value : sound.xaS0_) io.intv(value);
        for (int& value : sound.xaS1_) io.intv(value);
    }

    template <typename Io>
    static void visitAsc(Io& io, Asc& sound) {
        section(io, fourcc('A', 'S', 'C', ' '));
        io.bytes(sound.ram_, sizeof sound.ram_);
        io.bytes(sound.regs_, sizeof sound.regs_);
        for (u32& value : sound.phase_) io.u32v(value);
        for (u32& value : sound.increment_) io.u32v(value);
        for (u16& value : sound.fifoRead_) io.u16v(value);
        for (u16& value : sound.fifoWrite_) io.u16v(value);
        for (u16& value : sound.fifoLevel_) io.u16v(value);
        io.bytes(sound.lastFifo_, sizeof sound.lastFifo_);
        io.bytes(sound.lastOutput_, sizeof sound.lastOutput_);
        io.boolv(sound.linePending_);

        io.u64v(sound.ramWrites_);
        for (u64& value : sound.fifoWrites_) io.u64v(value);
        io.u64v(sound.irqTransitions_);
        io.u64v(sound.producedSamples_);
        io.u64v(sound.nonSilentSamples_);
        io.u64v(sound.mode2Selections_);
        io.u8v(sound.outputMin_); io.u8v(sound.outputMax_);

        for (u16& value : sound.historyOffset_) io.u16v(value);
        io.bytes(sound.historyValue_, sizeof sound.historyValue_);
        for (u32& value : sound.historyOrder_) io.u32v(value);
        io.u8v(sound.historyHead_); io.u8v(sound.historyCount_);
        io.u32v(sound.historySequence_); io.intv(sound.diagBudget_);
    }

    template <typename Io>
    static void visitAdb(Io& io, AdbTransceiver& adb) {
        section(io, fourcc('A', 'D', 'B', ' '));
        io.intv(adb.state_); io.u8v(adb.cmd_); io.bytes(adb.buf_, sizeof adb.buf_);
        io.intv(adb.len_); io.intv(adb.idx_); io.boolv(adb.int_);
        io.boolv(adb.open_); io.boolv(adb.stagedByWake_);
        io.intv(adb.kbdAddr_); io.intv(adb.mouseAddr_);
        for (bool& value : adb.keyState_) io.boolv(value);
        io.bytes(adb.kbdQ_, sizeof adb.kbdQ_);
        io.intv(adb.kbdHead_); io.intv(adb.kbdTail_);
        io.intv(adb.mouseDx_); io.intv(adb.mouseDy_);
        io.boolv(adb.mouseButton_); io.boolv(adb.mousePending_);
        io.intv(adb.listenReg_); io.intv(adb.listenAddr_); io.intv(adb.listenPos_);
        io.bytes(adb.listenBuf_, sizeof adb.listenBuf_);
        io.u32v(adb.mousePolls_); io.u32v(adb.kbdPolls_);
        io.u32v(adb.mouseReports_); io.u32v(adb.mouseBytesRead_);
        io.boolv(adb.lastEmitMouse_);
        io.sparse(adb.mouseBytesLog_, kMaxSmallBlob);
        io.u32v(adb.kbdReg2_); io.u32v(adb.kbdReg3_); io.u32v(adb.mouseReg3_);
        io.sparse(adb.cmdTrace_, kMaxSmallBlob);
        io.sparse(adb.respTrace_, kMaxSmallBlob);
    }

    template <typename Io>
    static void visitAdbBus(Io& io, IifxAdbBus& bus) {
        section(io, fourcc('A', 'B', 'U', 'S'));
        io.enum8(bus.receiveState_); io.boolv(bus.hostReleased_);
        io.boolv(bus.deviceReleased_); io.boolv(bus.wireHigh_);
        io.boolv(bus.deviceSending_); io.u64v(bus.now_); io.u64v(bus.lastEdge_);
        io.u8v(bus.shift_); io.intv(bus.bit_); io.intv(bus.listenBytes_);
        io.bytes(bus.lastListen_.data(), bus.lastListen_.size());
        std::size_t eventCount = bus.events_.size();
        io.boundedSize(eventCount, 1024u * 1024u);
        if (io.applying()) bus.events_.resize(eventCount);
        for (std::size_t index = 0; index < eventCount; ++index) {
            IifxAdbBus::LineEvent scratch;
            auto& event = io.applying() ? bus.events_[index] : scratch;
            io.u64v(event.tick); io.boolv(event.released); io.boolv(event.final);
        }
        io.sizev(bus.nextEvent_);
        io.u64v(bus.commands_); io.u64v(bus.replies_); io.u64v(bus.replyBytes_);
        io.u64v(bus.resets_); io.u64v(bus.replySamples_);
        io.u64v(bus.replyLowSamples_); io.u64v(bus.abortedReplies_);
        io.u64v(bus.listenTransactions_); io.u8v(bus.lastCommand_);
    }

    template <typename Io>
    static void visitSwim(Io& io, Iwm& swim, u32 stateVersion) {
        section(io, fourcc('S', 'W', 'I', 'M'));
        io.boolv(swim.writeReady); io.boolv(swim.writeLatched);
        io.u8v(swim.writeData); io.boolv(swim.senseHigh);
        io.boolv(swim.readDataHigh); io.boolv(swim.swimEnabled);
        io.u8v(swim.ismData); io.boolv(swim.ismDataReady);
        io.boolv(swim.ismMarkNext); io.boolv(swim.ismCrcOk);
        io.boolv(swim.ismMarkLatched); io.boolv(swim.ismDataTaken);
        io.boolv(swim.ismWroteData); io.u8v(swim.ismWritten);
        io.boolv(swim.ismWroteMark); io.boolv(swim.ismWroteCrc);
        io.u8v(swim.lines_); io.u8v(swim.mode_); io.boolv(swim.ism_);
        io.u8v(swim.unlock_); io.intv(swim.unlockN_);
        io.u8v(swim.ismMode_); io.u8v(swim.ismPhase_);
        io.u8v(swim.ismSetup_); io.u8v(swim.ismError_);
        io.u8v(swim.ismIwmCfg_); io.u16v(swim.ismCrc_);
        io.boolv(swim.ismFifoArmed_);
        if (stateVersion >= 2) {
            io.bytes(swim.ismFifoData_, sizeof swim.ismFifoData_);
            io.bytes(swim.ismFifoMark_, sizeof swim.ismFifoMark_);
            for (u16& crc : swim.ismFifoCrc_) io.u16v(crc);
            io.u8v(swim.ismFifoHead_); io.u8v(swim.ismFifoCount_);
        } else if (io.applying()) {
            // Version 1 represented the SWIM read FIFO as one public latch.
            // Promote that latch into slot zero so old pre-trigger captures
            // remain deterministic under the two-byte FIFO model.
            swim.ismFifoHead_ = 0;
            swim.ismFifoCount_ = swim.ismDataReady ? 1u : 0u;
            swim.ismFifoData_[0] = swim.ismData;
            swim.ismFifoMark_[0] = swim.ismMarkNext ? 1u : 0u;
            swim.ismFifoCrc_[0] = swim.ismCrc_;
            swim.ismFifoData_[1] = 0;
            swim.ismFifoMark_[1] = 0;
            swim.ismFifoCrc_[1] = 0xFFFF;
            swim.refreshIsmReadFifoView();
        }
        io.bytes(swim.ismParam_, sizeof swim.ismParam_);
        io.intv(swim.ismParamIdx_);
    }

    template <typename Io>
    static void visitSony(Io& io, SonyDrive& drive) {
        section(io, fourcc('S', 'O', 'N', 'Y'));
        io.boolv(drive.installed); io.boolv(drive.doubleSided);
        io.boolv(drive.superDrive); io.u32v(drive.surfaceReads);
        io.u32v(drive.selections); io.u32v(drive.writesDropped);
        io.boolv(drive.readOnly); io.boolv(drive.hdMedia);
        io.intv(drive.track); io.boolv(drive.headUpper); io.boolv(drive.stepIn);
        io.intv(drive.cacheTrack); io.intv(drive.cacheSide);
        io.u32v(drive.cacheGen); io.u32v(drive.mediaGen);
        io.u16v(drive.overrideMask); io.u16v(drive.overrideValue);
        io.boolv(drive.motorWanted_); io.boolv(drive.diskSwitched_);
        io.boolv(drive.mfmMode_); io.boolv(drive.enabled_);
        io.boolv(drive.ejectPending_); io.boolv(drive.ejectRequested_);
        io.u64v(drive.stepDoneAt_); io.u64v(drive.motorUpAt_);
        io.u64v(drive.ejectAt_);
        io.sparse(drive.trackData_, kMaxSmallBlob);
        io.sparse(drive.trackMark_, kMaxSmallBlob);
        io.sizev(drive.bytePos_); io.u64v(drive.lastByteAt_);
        io.boolv(drive.trackDirty_); io.u64v(drive.clockHz_);
    }

    template <typename Io>
    static void visitAm29000(Io& io, Am29000& cpu) {
        for (u32& value : cpu.m_r) io.u32v(value);
        for (u32& value : cpu.m_tlb) io.u32v(value);
        cpu.fetchTranslationValid_ = false; // hint derived from m_tlb
        cpu.clearFetchWindows();            // host pointers; re-registered lazily
        cpu.loopReset();                    // learned loops belong to the old state
        io.u32v(cpu.m_pc); io.u32v(cpu.m_vab); io.u32v(cpu.m_ops);
        io.u32v(cpu.m_cps); io.u32v(cpu.m_cfg); io.u32v(cpu.m_cha);
        io.u32v(cpu.m_chd); io.u32v(cpu.m_chc); io.u32v(cpu.m_rbp);
        io.u32v(cpu.m_tmc); io.u32v(cpu.m_tmr); io.u32v(cpu.m_pc0);
        io.u32v(cpu.m_pc1); io.u32v(cpu.m_pc2); io.u32v(cpu.m_mmu);
        io.u32v(cpu.m_lru); io.u32v(cpu.m_ipc); io.u32v(cpu.m_ipa);
        io.u32v(cpu.m_ipb); io.u32v(cpu.m_q); io.u32v(cpu.m_alu);
        io.u32v(cpu.m_fpe); io.u32v(cpu.m_inte); io.u32v(cpu.m_fps);
        io.u32v(cpu.m_exceptions);
        for (u32& value : cpu.m_exception_queue) io.u32v(value);
        io.u8v(cpu.m_irq_active); io.u8v(cpu.m_irq_lines);
        io.u32v(cpu.m_exec_ir); io.u32v(cpu.m_next_ir);
        io.u32v(cpu.m_pl_flags); io.u32v(cpu.m_next_pl_flags);
        io.u32v(cpu.m_iret_pc); io.u32v(cpu.m_exec_pc);
        io.u32v(cpu.m_next_pc); io.u64v(cpu.instructions_);
        io.boolv(cpu.faulted_);
        if (io.applying()) {
            cpu.faultReason_ = cpu.faulted_
                ? "faulted Am29000 state restored from checkpoint" : "";
        }
    }

    template <typename Io>
    static void visitVideo(Io& io, IifxNuBusVideo& video, u32 stateVersion) {
        section(io, fourcc('V', 'I', 'D', '9'));
        io.sparse(video.vram_, IifxNuBusVideo::kGcVramBytes);
        io.u16v(video.mode_); io.u16v(video.page_);
        io.u64v(video.reads); io.u64v(video.romReads);
        io.u64v(video.writes); io.u64v(video.vramWrites);
        for (u32& value : video.firstReadOffsets) io.u32v(value);
        io.sizev(video.firstReadCount);
        if (stateVersion >= 4) {
            io.sameU32(video.genuineGc_ ? 1u : 0u);
            io.u64v(video.superReads);
            io.u16v(video.control_); io.u16v(video.preload_);
            io.u32v(video.base_); io.u32v(video.stride_);
            for (u32& value : video.jmfbShadow_) io.u32v(value);
            for (u32& value : video.crtcShadow_) io.u32v(value);
            for (u32& value : video.palette_) io.u32v(value);
            for (u8& value : video.clutColor_) io.u8v(value);
            io.u8v(video.clutAddress_); io.u8v(video.clutComponent_);
            io.u8v(video.ramdacMode_); io.boolv(video.ramdacConvolution_);
            io.u16v(video.serialShift_); io.u8v(video.serialBits_);
            io.boolv(video.gcInterruptGate_); io.boolv(video.vblankDisable_);
            io.boolv(video.vblankEnabled_); io.boolv(video.vblankIrq_);
            io.u64v(video.vblankCount); io.u64v(video.vblankAssertions);
            io.u64v(video.vblankAcks); io.u64v(video.frameVramWrites_);
            io.u64v(video.lastFrameVramWrites);
            if (stateVersion >= 5) {
                io.u64v(video.serialCommands_);
                io.u16v(video.lastSerialCommand_);
                io.u64v(video.timingPhase_);
            } else if (io.applying()) {
                video.serialCommands_ = 0;
                video.lastSerialCommand_ = 0;
                video.timingPhase_ = 0;
            }
            if (stateVersion >= 6) {
                io.u64v(video.timingEchoWrites_);
                io.boolv(video.timingEchoPending_);
                io.u32v(video.timingEchoValue_);
                io.u8v(video.timingEchoLane_);
            } else if (io.applying()) {
                video.timingEchoWrites_ = 0;
                video.timingEchoPending_ = false;
                video.timingEchoValue_ = 0;
                video.timingEchoLane_ = 0;
            }
            if (stateVersion >= 7) {
                io.u8v(video.ramdacControl_);
                io.u32v(video.mfbBaseShift_); io.u8v(video.mfbBaseBits_);
                io.u16v(video.mfbStrideShift_); io.u8v(video.mfbStrideBits_);
            } else if (io.applying()) {
                video.ramdacControl_ = static_cast<u8>(
                    0x80u | (video.ramdacMode_ << 1) |
                    (video.ramdacConvolution_ ? 1u : 0u));
                if (video.ramdacMode_ == 0x0Eu) video.mode_ = 0x84u;
                video.mfbBaseShift_ = 0;
                video.mfbBaseBits_ = 0;
                video.mfbStrideShift_ = 0;
                video.mfbStrideBits_ = 0;
            }
            if (stateVersion >= 9) {
                io.bytes(video.gcSram_.data(), video.gcSram_.size());
            } else if (io.applying()) {
                video.gcSram_.fill(0);
            }
            if (stateVersion >= 10) {
                io.u64v(video.gcCpuPhase_);
                io.u32v(video.gcResetControl_);
                io.u32v(video.gcInterruptControl_);
                io.boolv(video.gcCpuReleased_);
                io.u64v(video.gcInstructionReads_);
                io.u64v(video.gcDataReads_); io.u64v(video.gcDataWrites_);
                io.u64v(video.gcUnknownDataReads_);
                io.u64v(video.gcUnknownDataWrites_);
                io.u32v(video.gcFirstUnknownDataAddress_);
                visitAm29000(io, video.gcCpu_);
            } else if (io.applying()) {
                video.gcCpuPhase_ = 0;
                video.gcResetControl_ = 0;
                video.gcInterruptControl_ = 0;
                video.gcCpuReleased_ = false;
                video.gcInstructionReads_ = 0;
                video.gcDataReads_ = video.gcDataWrites_ = 0;
                video.gcUnknownDataReads_ = video.gcUnknownDataWrites_ = 0;
                video.gcFirstUnknownDataAddress_ = 0;
                video.gcCpu_.reset();
            }
            if (stateVersion >= 11) {
                io.bytes(video.gcVectorRam_.data(), video.gcVectorRam_.size());
            } else if (io.applying()) {
                video.resetGcVectorStore();
            }
            if (stateVersion >= 12) {
                for (u32& value : video.gcMfbShadow_) io.u32v(value);
                io.u32v(video.gcCacheControl_);
                io.boolv(video.gcHostRequest_);
                if (io.applying()) {
                    video.gcCpu_.setInput(0, false);
                    video.gcMfbShadow_[0x64u >> 2u] =
                        video.gcHostRequest_ ? 0xFFFFFFFFu : 0u;
                    video.gcCpu_.setInput(2, video.gcHostRequest_);
                    // The $44000088 interrupt hold stays disarmed (see
                    // nubus_video.hpp); keep the derived CPU state in the
                    // matching disarmed position on load.
                    video.gcCpu_.setInterruptHold(false);
                }
            } else if (io.applying()) {
                video.gcMfbShadow_.fill(0);
                video.gcCacheControl_ = 0xFFFF0000u;
                video.gcHostRequest_ = false;
                video.gcCpu_.setInput(0, false);
                video.gcCpu_.setInput(2, false);
            }
            if (stateVersion >= 13) {
                io.sparse(video.gcDram_, IifxNuBusVideo::kGcDramBytes);
            } else if (io.applying()) {
                std::fill(video.gcDram_.begin(), video.gcDram_.end(), u8{0});
            }
            if (stateVersion >= 14) {
                io.sparse(video.gcExpansionDram_,
                          IifxNuBusVideo::kGcExpansionDramBytes);
            } else if (io.applying()) {
                std::fill(video.gcExpansionDram_.begin(),
                          video.gcExpansionDram_.end(), u8{0});
            }
            if (stateVersion >= 15) {
                io.u32v(video.gcMacInterruptControl_);
                io.boolv(video.gcMacRequest_);
            } else if (io.applying()) {
                video.gcMacInterruptControl_ = 0;
                video.gcMacRequest_ = false;
            }
            if (stateVersion >= 16) {
                io.u32v(video.gcCacheFillAddress_);
                io.u32v(video.gcCacheFillWordsRemaining_);
                io.boolv(video.gcCacheFillActive_);
            } else if (io.applying()) {
                video.gcCacheFillAddress_ = 0;
                video.gcCacheFillWordsRemaining_ = 0;
                video.gcCacheFillActive_ = false;

                // Versions through 15 accidentally overlaid RDNC's shared
                // data aperture on MFB's instruction-cache SRAM.  The two
                // memories cannot be reconstructed from that single image:
                // values such as $FFFF73FF are cache-page tags, not IPC
                // headers.  Preserve the separately serialized GC DRAM; only
                // a compact IPC block with its unambiguous runtime signature
                // can be recovered without corrupting resident GCOS code.
                const u32 ipcHeader = video.gcReadSramWord(0x8C00u);
                if (ipcHeader == 0x01010000u) {
                    constexpr std::size_t kLegacyIpcBytes = 0x80u;
                    std::copy_n(video.gcSram_.begin() + 0x8C00u,
                                kLegacyIpcBytes,
                                video.gcDram_.begin() + 0x8C00u);
                }
            }
        }
    }

    template <typename Io>
    static void visit(Io& io, IifxMachine& machine, u32 stateVersion) {
        section(io, fourcc('M', 'A', 'C', 'H'));
        io.sparse(machine.ram_, kMaxRamBytes);
        io.boolv(machine.overlay_);

        u64 logicalDiskBytes = static_cast<u64>(machine.hardDisk_.size());
        io.boundedU64(logicalDiskBytes, kMaxMediaBytes);
        if (io.applying())
            machine.hardDisk_.assign(static_cast<std::size_t>(logicalDiskBytes), 0);
        io.sparse(machine.scsiImage_, kMaxMediaBytes);
        io.u32v(machine.hfsImageOffset_); io.u32v(machine.hfsVolumeBytes_);
        io.boolv(machine.hardDiskIsWholeScsi_); io.boolv(machine.inDiskDriver_);
        io.u32v(machine.diskPrimePc_); io.u32v(machine.diskCtlPc_);
        io.u32v(machine.diskStatusPc_); io.u32v(machine.hardDiskReadCount_);
        io.u32v(machine.hardDiskWriteCount_); io.intv(machine.hardDiskTraceBudget_);
        io.intv(machine.hardDiskErrorBudget_);
        io.boolv(machine.hardDiskMountPending_); io.boolv(machine.hardDiskMounted_);
        io.boolv(machine.hardDiskMountInFlight_); io.u32v(machine.hardDiskMountPb_);
        io.u32v(machine.hardDiskMountTries_); io.u32v(machine.hardDiskMountDelay_);
        u16 mountResult = std::bit_cast<u16>(machine.hardDiskMountResult_);
        io.u16v(mountResult);
        if (io.applying()) machine.hardDiskMountResult_ = std::bit_cast<s16>(mountResult);
        io.u32v(machine.injectedTrapTimeouts_);

        io.u64v(machine.totalCycles_); io.u64v(machine.frameCounter_);
        io.u64v(machine.viaPhase_); io.u64v(machine.framePhase_);
        io.u64v(machine.secondPhase_); io.u64v(machine.audioPhase_);
        for (u32& value : machine.biuRegs_) io.u32v(value);
        io.bytes(&machine.ossExpansionRegs_[0][0], sizeof machine.ossExpansionRegs_);
        io.boolv(machine.soundIrq_);

        for (u32 index = 0; index < IifxMachine::kFmcLines; ++index) {
            auto& line = machine.fmcCache_[index];
            io.u32v(line.tag); io.boolv(line.valid);
            for (u32& value : line.data) io.u32v(value);
        }
        io.intv(machine.fmcCyclePenalty_);
        io.u64v(machine.fmcHits_); io.u64v(machine.fmcMisses_);
        io.u64v(machine.fmcFills_); io.u64v(machine.fmcCacheInhibited_);

        io.sparse(machine.floppy_, kMaxSmallBlob);
        io.sparse(machine.floppyEjected_, kMaxSmallBlob);
        io.sparse(machine.fdMbHeader_, kMaxSmallBlob);
        io.sparse(machine.fdMbResource_, kMaxSmallBlob);
        io.sparse(machine.fdDcHeader_, kMaxSmallBlob);
        io.sparse(machine.fdDcTags_, kMaxSmallBlob);
        io.boolv(machine.floppyReadOnly_); io.boolv(machine.floppyServiceReady_);
        io.s64v(machine.floppyEventRetryCycles_);
        io.boolv(machine.swimLstrbPrev_); io.boolv(machine.swimActionPrev_);
        // A host-side skip flag, not machine state: a loaded checkpoint may
        // carry live SWIM DMA requests, so the next idle pass must clear them.
        if (io.applying()) machine.swimDmaIdle_ = false;
        io.boolv(machine.swimWriteMarkPrev_); io.boolv(machine.swimReadMarkPrev_);
        if (stateVersion >= 3) {
            io.boolv(machine.swimReadSynced_);
        } else if (io.applying()) {
            machine.swimReadSynced_ = false;
        }
        io.u16v(machine.swimCrcOut_); io.u64v(machine.swimDataBytes_);
        io.u64v(machine.swimDataWrites_); io.u32v(machine.floppyTrackWrites_);
        io.intv(machine.swimDiagBudget_); io.intv(machine.sonyPrimeDiagBudget_);
        if (stateVersion >= 2) {
            io.u64v(machine.mediaEventId_); io.u64v(machine.activeMediaEventId_);
        } else if (io.applying()) {
            machine.mediaEventId_ = 0;
            machine.activeMediaEventId_ = 0;
        }
        io.sparse(machine.audioOut_, kMaxSmallBlob);
        io.boolv(machine.legacyAccessLogEnabled_);

        visitCpu(io, machine.cpu_);
        visitVia(io, *machine.via_);
        visitRtc(io, *machine.rtc_);
        visitOss(io, *machine.oss_);
        visitIop(io, *machine.sccIop_, fourcc('I', 'O', 'P', 'S'));
        visitIop(io, *machine.ismIop_, fourcc('I', 'O', 'P', 'M'));
        visitScc(io, *machine.scc_);
        visitScsi(io, *machine.scsi_);
        section(io, fourcc('D', 'I', 'S', 'K'));
        visitDisk(io, *machine.disk_);
        visitScsiDma(io, *machine.scsiDma_);
        if (stateVersion >= 8) {
            visitAsc(io, *machine.sound_);
        } else {
            // Versions 1-7 stored the mistakenly attached Quadra EASC.  Read
            // every field in its original order so the following ADB/SWIM/
            // video sections remain aligned, then start the real ASC from a
            // clean power-on state.  Useful old Finder/color checkpoints are
            // therefore retained without pretending their silent EASC state
            // can be meaningfully translated into first-generation ASC RAM.
            Easc discardedLegacySound;
            visitEasc(io, discardedLegacySound);
            if (io.applying()) {
                machine.sound_->reset();
                machine.soundIrq_ = false;
            }
        }
        visitAdb(io, *machine.adb_);
        visitAdbBus(io, *machine.adbBus_);
        visitSwim(io, *machine.swim_, stateVersion);
        visitSony(io, *machine.floppyDrive_);
        visitVideo(io, *machine.video_, stateVersion);
        if (stateVersion < 5 && io.applying() && machine.video_->genuineGc_) {
            const u64 reduced = machine.totalCycles_ %
                IifxNuBusVideo::kGcTimingFrameUnits;
            machine.video_->timingPhase_ =
                (reduced * IifxNuBusVideo::kGcTimingUnitsPerCpuCycle) %
                IifxNuBusVideo::kGcTimingFrameUnits;
        }
        section(io, fourcc('E', 'N', 'D', '!'));
    }
};

std::vector<u8> IifxMachine::saveState() const {
    return IifxStateCodec::save(*this);
}

bool IifxMachine::loadState(const u8* data, std::size_t size,
                            std::string* error) {
    return IifxStateCodec::load(*this, data, size, error);
}

bool IifxMachine::saveStateFile(const std::string& path,
                                std::string* error) const {
    const std::vector<u8> checkpoint = saveState();
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output) {
        if (error) *error = "cannot open checkpoint for writing: " + path;
        return false;
    }
    output.write(reinterpret_cast<const char*>(checkpoint.data()),
                 static_cast<std::streamsize>(checkpoint.size()));
    if (!output) {
        if (error) *error = "failed while writing checkpoint: " + path;
        return false;
    }
    if (error) error->clear();
    return true;
}

bool IifxMachine::loadStateFile(const std::string& path, std::string* error) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        if (error) *error = "cannot open checkpoint for reading: " + path;
        return false;
    }
    std::vector<u8> checkpoint{std::istreambuf_iterator<char>(input),
                               std::istreambuf_iterator<char>()};
    return loadState(checkpoint.data(), checkpoint.size(), error);
}

} // namespace openmac
