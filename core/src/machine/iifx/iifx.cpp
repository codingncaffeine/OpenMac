#include "openmac/iifx.hpp"

#include "../rtc.hpp"
#include "../scsi.hpp"
#include "../scsiimage.hpp"
#include "../via.hpp"
#include "../adb.hpp"
#include "../dc42.hpp"
#include "../iwm.hpp"
#include "../sony.hpp"
#include "../gcr.hpp"
#include "../mfm.hpp"
#include "../macbinary.hpp"
#include "../quadra/scc.hpp"
#include "asc.hpp"
#include "iop.hpp"
#include "adb_bus.hpp"
#include "nubus_video.hpp"
#include "oss.hpp"
#include "scsi_dma.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace openmac {

namespace {

constexpr u64 kCpuHz = 40000000;
constexpr u64 kViaHz = 783360;
constexpr u64 kFrameNumerator = 6015;     // 60.15 Hz exactly as a rational
constexpr u64 kFrameDenominator = 100;
constexpr u32 kPramBlobBytes = 268;
constexpr u32 kMiB = 1024u * 1024u;

u64 hashByte(u64 hash, u8 value) {
    return (hash ^ value) * 1099511628211ull;
}

u64 hashWord(u64 hash, u64 value, int bytes = 8) {
    for (int index = 0; index < bytes; ++index)
        hash = hashByte(hash, static_cast<u8>(value >> (index * 8)));
    return hash;
}

HardwareTraceSource traceSourceFor(const char* device) {
    if (std::strcmp(device, "VIA1") == 0) return HardwareTraceSource::Via;
    if (std::strcmp(device, "RTC") == 0) return HardwareTraceSource::Rtc;
    if (std::strcmp(device, "OSS") == 0 ||
        std::strncmp(device, "OSS-", 4) == 0)
        return HardwareTraceSource::Oss;
    if (std::strcmp(device, "SCC-IOP") == 0)
        return HardwareTraceSource::SccIop;
    if (std::strcmp(device, "ISM-IOP") == 0)
        return HardwareTraceSource::IsmIop;
    if (std::strcmp(device, "NCR5380") == 0)
        return HardwareTraceSource::Ncr5380;
    if (std::strncmp(device, "SCSI-DMA", 8) == 0)
        return HardwareTraceSource::ScsiDma;
    if (std::strcmp(device, "ASC") == 0) return HardwareTraceSource::Asc;
    if (std::strncmp(device, "NUBUS9", 6) == 0)
        return HardwareTraceSource::NuBus;
    if (std::strcmp(device, "BIU30") == 0) return HardwareTraceSource::Biu;
    return HardwareTraceSource::Other;
}

constexpr u32 kIoBase = 0x50000000u;
constexpr u32 kIoMirrorMask = 0x0003FFFFu;
constexpr u32 kSonyPrime = 0x4086C406u;
constexpr u32 kSonyPrimeAlias = 0x0086C406u;
constexpr u32 kSonyPrimeHandler = 0x4086C5ACu;
constexpr u32 kSonyPrimeHandlerAlias = 0x0086C5ACu;
constexpr u32 kSonyControl = 0x4086C414u;
constexpr u32 kSonyControlAlias = 0x0086C414u;
constexpr u32 kSonyControlHandler = 0x4086C6D8u;
constexpr u32 kSonyControlHandlerAlias = 0x0086C6D8u;
constexpr u32 kSonyStatus = 0x4086C422u;
constexpr u32 kSonyStatusAlias = 0x0086C422u;
constexpr u32 kScsiDiskUnit = 32u;            // canonical .ScsiHD refnum -33

bool scsiDmaLongRegister(u32 address, u32& reg) {
    if (address < kIoBase || address >= 0x54000000u) return false;
    const u32 off = address & kIoMirrorMask;
    const u32 block = off & 0x3E000u;
    if (block != 0x08000u && block != 0x0C000u) return false;
    reg = off & 0x1FFu;
    return reg == IifxScsiDma::Control || reg == IifxScsiDma::Count ||
           reg == IifxScsiDma::Address || reg == IifxScsiDma::Watchdog ||
           reg == IifxScsiDma::Fifo;
}

u32 nextPowerOfTwo(u32 v) {
    u32 result = 1;
    while (result < v) result <<= 1;
    return result;
}

u32 validatedIifxRamSize(u32 bytes) {
    if ((bytes % kMiB) == 0) {
        switch (bytes / kMiB) {
        case 4: case 8: case 16: case 20: case 32:
        case 64: case 68: case 80: case 128:
            return bytes;
        default:
            break;
        }
    }
    throw std::invalid_argument(
        "IIfx RAM must be a legal two-bank layout: 4, 8, 16, 20, 32, 64, 68, 80, or 128 MB");
}

u32 readBe32(const std::vector<u8>& bytes, std::size_t at) {
    return (static_cast<u32>(bytes[at]) << 24) |
           (static_cast<u32>(bytes[at + 1]) << 16) |
           (static_cast<u32>(bytes[at + 2]) << 8) |
           static_cast<u32>(bytes[at + 3]);
}

// Return the byte range occupied by the first Apple_HFS partition in a
// complete physical disk image. Apple partition-map entries live in 512-byte
// blocks even when the device reports another logical block size.
bool locateHfsPartition(const std::vector<u8>& disk, u32& offset, u32& bytes) {
    for (std::size_t block = 1; block < 64; ++block) {
        const std::size_t at = block * 512u;
        if (at + 512u > disk.size()) break;
        if (disk[at] != 0x50 || disk[at + 1] != 0x4D) continue;  // 'PM'
        if (std::strncmp(reinterpret_cast<const char*>(disk.data() + at + 48),
                         "Apple_HFS", 32) != 0)
            continue;
        const u64 start = static_cast<u64>(readBe32(disk, at + 8)) * 512u;
        const u64 length = static_cast<u64>(readBe32(disk, at + 12)) * 512u;
        if (start >= disk.size() || length == 0 || start + length > disk.size())
            return false;
        offset = static_cast<u32>(start);
        bytes = static_cast<u32>(length);
        return true;
    }
    return false;
}

} // namespace

IifxMachine::IifxMachine(std::vector<u8> rom, const Config& config)
    : ram_(validatedIifxRamSize(config.ramSize), 0),
      rom_(std::move(rom)),
      via_(std::make_unique<Via6522>()),
      rtc_(std::make_unique<Rtc>()),
      oss_(std::make_unique<Oss>()),
      sccIop_(std::make_unique<IifxIop>()),
      ismIop_(std::make_unique<IifxIop>()),
      scc_(std::make_unique<Scc8530>()),
      scsi_(std::make_unique<Ncr5380>()),
      scsiDma_(std::make_unique<IifxScsiDma>(*scsi_)),
      disk_(std::make_unique<ScsiDisk>()),
      sound_(std::make_unique<Asc>()),
      adb_(std::make_unique<AdbTransceiver>()),
      adbBus_(std::make_unique<IifxAdbBus>(*adb_)),
      swim_(std::make_unique<Iwm>()),
      floppyDrive_(std::make_unique<SonyDrive>()),
      video_(std::make_unique<IifxNuBusVideo>(config.videoDeclarationRom)),
      cpu_(*this),
      videoEnabled_(config.videoCard),
      nativeStorage_(config.nativeStorage),
      fmcCache_(std::make_unique<FmcLine[]>(kFmcLines)) {
    if (rom_.empty()) throw std::invalid_argument("IIfx ROM is empty");
    const u32 romSize = nextPowerOfTwo(static_cast<u32>(rom_.size()));
    rom_.resize(romSize, 0xFF);
    romMask_ = romSize - 1;

    // A reset/default Macintosh PRAM selects the Macintosh Operating System.
    // This ROM's GetOSDefault reads the two-byte DefOSRec at XPRAM $76; byte
    // $77 is sdOSType, whose documented Mac OS value is 1.
    rtc_->xpram()[0x77] = 1;
    floppyDrive_->installed = true;
    floppyDrive_->doubleSided = true;
    floppyDrive_->superDrive = true;
    floppyDrive_->setClockHz(kCpuHz);

    scsi_->addTarget(disk_.get());
    wireDevices();
    reset();
}

IifxMachine::IifxMachine(std::vector<u8> rom)
    : IifxMachine(std::move(rom), Config{}) {}

IifxMachine::~IifxMachine() = default;

void IifxMachine::wireDevices() {
    video_->setGcBusMasterCallbacks(
        [this](u32 address) {
            if (address <= ram_.size() - 4u)
                return (u32(ram_[address]) << 24) |
                       (u32(ram_[address + 1u]) << 16) |
                       (u32(ram_[address + 2u]) << 8) |
                       u32(ram_[address + 3u]);
            // NuBus slot $0 belongs to the Macintosh logic board.  Apple's
            // published mapping aliases $F0800000-$F0FFFFFF to system ROM.
            if (address >= 0xF0800000u && address <= 0xF0FFFFFCu) {
                return (u32(rom_[address & romMask_]) << 24) |
                       (u32(rom_[(address + 1u) & romMask_]) << 16) |
                       (u32(rom_[(address + 2u) & romMask_]) << 8) |
                       u32(rom_[(address + 3u) & romMask_]);
            }
            return u32{0};
        },
        [this](u32 address, u32 value) {
            if (address > ram_.size() - 4u) return;
            // NuBus masters are snooped by the IIfx FMC just like CPU writes.
            write32(address, value);
        });

    const auto traceIopMailbox = [this](IifxIop& iop, const char* name,
                                        bool isIsm) {
        return [this, &iop, name, isIsm](bool host, u16 address, u8 value) {
            const bool send = address >= IifxIop::SendState &&
                              address < IifxIop::SendState + IifxIop::ChannelCount;
            const bool receive = address >= IifxIop::RecvState &&
                                 address < IifxIop::RecvState + IifxIop::ChannelCount;
            if (!send && !receive) return;
            if (value != IifxIop::MsgNew && value != IifxIop::MsgComplete)
                return;
            const int channel = static_cast<int>(address -
                (send ? IifxIop::SendState : IifxIop::RecvState));
            const u16 message = static_cast<u16>(
                (send ? IifxIop::SendMsg : IifxIop::RecvMsg) +
                channel * IifxIop::MessageSize);

            std::array<u8, IifxIop::MessageSize> payload{};
            for (int index = 0; index < IifxIop::MessageSize; ++index)
                payload[static_cast<std::size_t>(index)] = iop.ram(
                    static_cast<u16>(message + index));
            const auto be32 = [&payload](int offset) {
                return (static_cast<u32>(payload[static_cast<std::size_t>(offset)]) << 24) |
                       (static_cast<u32>(payload[static_cast<std::size_t>(offset + 1)]) << 16) |
                       (static_cast<u32>(payload[static_cast<std::size_t>(offset + 2)]) << 8) |
                       payload[static_cast<std::size_t>(offset + 3)];
            };
            const u8 operation = payload[0];
            const s16 operationResult = static_cast<s16>(
                (static_cast<u16>(payload[2]) << 8) | payload[3]);
            const bool blockOperation = operation >= 0x0Au && operation <= 0x0Cu;
            const u32 ramAddress = blockOperation ? be32(4) : 0;
            const u32 block = blockOperation ? be32(8) : 0;
            const u32 blockCount = blockOperation ? be32(12) : 0;
            const u8 expectedTrack = blockOperation
                ? static_cast<u8>(block / (mfm::kSides * mfm::kSectorsPerTrack))
                : 0xFFu;
            const u32 trackBlock = blockOperation
                ? block % (mfm::kSides * mfm::kSectorsPerTrack) : 0;
            const u8 expectedSide = blockOperation
                ? static_cast<u8>(trackBlock / mfm::kSectorsPerTrack) : 0xFFu;
            const u8 expectedSector = blockOperation
                ? static_cast<u8>(trackBlock % mfm::kSectorsPerTrack + 1u) : 0xFFu;

            // Keep the active request's geometry in every subsequent SWIM
            // snapshot. A command edge can then be read without finding the
            // earlier mailbox record in a very busy trace.
            if (isIsm && host && send && channel == 1 &&
                value == IifxIop::MsgNew && blockOperation) {
                traceFloppyBlock_ = block;
                traceFloppyTrack_ = expectedTrack;
                traceFloppySide_ = expectedSide;
                traceFloppySector_ = expectedSector;
            }

            if (hardwareTrace_.enabled() && !hardwareTrace_.frozen()) {
                HardwareTraceEvent event = makeTraceEvent(
                    HardwareTraceKind::Mailbox,
                    isIsm ? HardwareTraceSource::IsmIop
                          : HardwareTraceSource::SccIop,
                    TraceIop | (isIsm ? TraceSwim : 0u));
                event.address = address;
                event.value = (static_cast<u64>(host ? 1u : 0u) << 63) |
                              (static_cast<u64>(send ? 1u : 0u) << 62) |
                              (static_cast<u64>(channel & 0xFF) << 48) |
                              (static_cast<u64>(value) << 40);
                for (int index = 0; index < 5; ++index)
                    event.value |= static_cast<u64>(payload[
                        static_cast<std::size_t>(index)]) << ((4 - index) * 8);
                event.mailbox.length = IifxIop::MessageSize;
                event.mailbox.operation = operation;
                event.mailbox.channel = static_cast<u8>(channel);
                if (host) event.mailbox.flags |= 1u << 0;
                if (send) event.mailbox.flags |= 1u << 1;
                if (value == IifxIop::MsgNew) event.mailbox.flags |= 1u << 2;
                if (value == IifxIop::MsgComplete) event.mailbox.flags |= 1u << 3;
                if (blockOperation) event.mailbox.flags |= 1u << 4;
                event.mailbox.result = operationResult;
                event.mailbox.ramAddress = ramAddress;
                event.mailbox.block = block;
                event.mailbox.blockCount = blockCount;
                event.mailbox.expectedTrack = expectedTrack;
                event.mailbox.expectedSide = expectedSide;
                event.mailbox.expectedSector = expectedSector;
                event.mailbox.payload = payload;
                std::snprintf(event.detail.data(), event.detail.size(),
                              "%s %s ch%d %s by %s", name,
                              send ? "send" : "recv", channel,
                              value == IifxIop::MsgNew ? "new" : "complete",
                              host ? "host" : "firmware");
                hardwareTrace_.record(std::move(event));
            }

            const HardwareTraceConfig& traceConfig = hardwareTrace_.config();
            if (isIsm && host && send && channel == 1 &&
                value == IifxIop::MsgNew && !hardwareTrace_.triggered() &&
                (traceConfig.triggerIopOperation != 0xFFFFFFFFu ||
                 traceConfig.triggerIopBlock != 0xFFFFFFFFu) &&
                (traceConfig.triggerIopOperation == 0xFFFFFFFFu ||
                 operation == traceConfig.triggerIopOperation) &&
                (traceConfig.triggerIopBlock == 0xFFFFFFFFu ||
                 (blockOperation && block == traceConfig.triggerIopBlock)) &&
                ++traceIopOperationHits_ >= traceConfig.triggerIopHits) {
                char reason[96];
                std::snprintf(reason, sizeof reason,
                    "IOP SWIM request op=$%02X block=%u count=%u track=%u side=%u sector=%u",
                    static_cast<unsigned>(operation), block, blockCount,
                    static_cast<unsigned>(expectedTrack),
                    static_cast<unsigned>(expectedSide),
                    static_cast<unsigned>(expectedSector));
                triggerTrace(reason);
            }

            // A DiskInserted notification is a statement about the physical
            // mechanism, not merely a hint to the Event Manager.  The SWIM
            // driver ERS requires it to carry the same DriveStatus record as
            // request $06, including a nonzero DiskInPlace byte at offset 7.
            // Freezing here preserves the phase/handshake accesses that made
            // the downloaded firmware reach that impossible conclusion.
            if (isIsm && !host && receive && channel == 1 &&
                value == IifxIop::MsgNew && iop.ram(message) == 1u) {
                if (!floppyPresent())
                    protocolAssertion(1ull << 23,
                        HardwareTraceSource::IsmIop, TraceSwim | TraceIop,
                        "IOP SWIM reported DiskInserted with no medium",
                        iop.ram(static_cast<u16>(message + 1)),
                        iop.ram(static_cast<u16>(message + 7)));
                else if (iop.ram(static_cast<u16>(message + 7)) == 0u)
                    protocolAssertion(1ull << 24,
                        HardwareTraceSource::IsmIop, TraceSwim | TraceIop,
                        "IOP SWIM DiskInserted carried an empty DriveStatus",
                        iop.ram(static_cast<u16>(message + 1)), 0);
            }

            if (isIsm && host && receive && channel == 1 &&
                value == IifxIop::MsgComplete) {
                const u8 event = iop.ram(message);
                const u16 replyWord = static_cast<u16>(
                    (static_cast<u16>(iop.ram(message + 2)) << 8) |
                    iop.ram(message + 3));
                // The downloaded firmware retains an unacknowledged media
                // change and republishes it on its own polling cadence.  Do
                // not manufacture another physical SWITCHED edge here: that
                // is a different event, and can race the firmware's retry.
                floppyEventRetryCycles_ = 0;
                // The receive reply is opaque to the SWIM protocol. In
                // particular, bytes 2..3 are the event message's Error Code
                // on the outbound leg, but IOP Manager permits the host reply
                // to have a different layout. Treating a nonzero reply word
                // as a failed PostEvent stopped a valid insertion before the
                // firmware could observe completion and begin disk I/O.
                if (nativeStorage_ && onDiag) {
                    char diagnostic[224];
                    std::snprintf(diagnostic, sizeof diagnostic,
                        "IOP IIfx Sony event=%u drive=%u result=%04X "
                        "SysEvtMask=%04X EvtQ=%08X/%08X DrvQ=%08X "
                        "ticks=%08X retry=%lld switched=%d",
                        event, iop.ram(message + 1), replyWord,
                        read16(0x0144u), read32(0x014Cu), read32(0x0150u),
                        read32(0x030Au), read32(0x016Au),
                        static_cast<long long>(floppyEventRetryCycles_),
                        floppyDrive_->diskSwitched() ? 1 : 0);
                    onDiag(diagnostic);
                }
            }

            // The downloaded firmware owns mailbox scheduling and completion.
            // Track when its initial mechanism poll has completed so later
            // host insertions can assert the physical SWITCHED latch.
            if (isIsm && !host && send && channel == 1 &&
                value == IifxIop::MsgComplete) {
                // An explicitly requested cycle/PC trigger owns the trace;
                // the opportunistic floppy-error trigger must not freeze a
                // capture aimed at something else.
                if (operation == 0x0Bu && operationResult != 0 &&
                    hardwareTrace_.enabled() && !hardwareTrace_.triggered() &&
                    hardwareTrace_.config().triggerCycle == 0 &&
                    hardwareTrace_.config().triggerPc == 0) {
                    char reason[96];
                    std::snprintf(reason, sizeof reason,
                                  "IOP SWIM Read completed with error=%d ($%04X)",
                                  static_cast<int>(operationResult),
                                  static_cast<unsigned>(
                                      static_cast<u16>(operationResult)));
                    triggerTrace(reason);
                }
                if (operation == 1 && onDiag) {
                    // Initialize returns a two-byte result followed by one
                    // drive-kind byte for each of the 28 possible mechanisms.
                    // Keep the complete reply in one deterministic trace line:
                    // the first nonzero byte identifies exactly which physical
                    // enable/SEL/status pattern the downloaded firmware found.
                    char diagnosticPayload[192];
                    int used = std::snprintf(diagnosticPayload, sizeof diagnosticPayload,
                                             "IOP ISM initialize reply:");
                    for (int index = 0; index < IifxIop::MessageSize &&
                         used > 0 && used < static_cast<int>(sizeof diagnosticPayload); ++index) {
                        used += std::snprintf(diagnosticPayload + used,
                            sizeof diagnosticPayload - static_cast<std::size_t>(used),
                            " %02X", iop.ram(static_cast<u16>(message + index)));
                    }
                    onDiag(diagnosticPayload);
                }
                if (operation == 3) {
                    floppyServiceReady_ = true;
                    // The running firmware's mechanism poll publishes the
                    // initial media status after this command completes.
                }
            }
            if (onDiag) {
                char text[192];
                std::snprintf(text, sizeof text,
                    "IOP %s %s ch%d %s by %s: %02X %02X %02X %02X %02X %02X %02X %02X",
                    name, send ? "send" : "recv", channel,
                    value == IifxIop::MsgNew ? "new" : "complete",
                    host ? "host" : "firmware", iop.ram(message + 0),
                    iop.ram(message + 1), iop.ram(message + 2),
                    iop.ram(message + 3), iop.ram(message + 4),
                    iop.ram(message + 5), iop.ram(message + 6),
                    iop.ram(message + 7));
                onDiag(text);
                if (isIsm && channel == 1 &&
                    ((receive && !host && value == IifxIop::MsgNew) ||
                     (receive && host && value == IifxIop::MsgComplete))) {
                    char diagnosticPayload[224];
                    int used = std::snprintf(diagnosticPayload, sizeof diagnosticPayload,
                                             "IOP ISM channel-1 payload:");
                    for (int index = 0; index < IifxIop::MessageSize &&
                         used > 0 && used < static_cast<int>(sizeof diagnosticPayload); ++index) {
                        used += std::snprintf(diagnosticPayload + used,
                            sizeof diagnosticPayload - static_cast<std::size_t>(used),
                            " %02X", iop.ram(static_cast<u16>(message + index)));
                    }
                    onDiag(diagnosticPayload);
                }
            }
        };
    };
    sccIop_->onSharedWrite = traceIopMailbox(*sccIop_, "SCC", false);
    ismIop_->onSharedWrite = traceIopMailbox(*ismIop_, "ISM", true);
    const auto traceIopInstruction = [this](HardwareTraceSource source,
                                            u16 pc, u8 opcode,
                                            const R65C02& cpu) {
        if (!hardwareTrace_.acceptsIopFlight()) return;
        HardwareTraceEvent event = makeTraceEvent(
            HardwareTraceKind::Instruction, source, TraceIop);
        event.pc = pc;
        event.address = pc;
        event.value = opcode;
        event.iop.pc = pc;
        event.iop.opcode = opcode;
        event.auxiliary = (static_cast<u64>(cpu.a) << 40) |
                          (static_cast<u64>(cpu.x) << 32) |
                          (static_cast<u64>(cpu.y) << 24) |
                          (static_cast<u64>(cpu.s) << 16) |
                          (static_cast<u64>(cpu.p) << 8);
        std::snprintf(event.detail.data(), event.detail.size(),
                      "%s instruction",
                      source == HardwareTraceSource::IsmIop ? "ISM IOP"
                                                            : "SCC IOP");
        hardwareTrace_.recordIopFlight(std::move(event));
    };
    sccIop_->onInstruction = [traceIopInstruction](u16 pc, u8 opcode,
                                                   const R65C02& cpu) {
        traceIopInstruction(HardwareTraceSource::SccIop, pc, opcode, cpu);
    };
    ismIop_->onInstruction = [traceIopInstruction](u16 pc, u8 opcode,
                                                   const R65C02& cpu) {
        traceIopInstruction(HardwareTraceSource::IsmIop, pc, opcode, cpu);
    };
    ismIop_->onDmaRequestEdge = [this](int channel, bool level) {
        if (!hardwareTrace_.accepts(TraceIop | TraceSwim)) return;
        HardwareTraceEvent event = makeTraceEvent(
            HardwareTraceKind::Dma, HardwareTraceSource::IsmIop,
            TraceIop | TraceSwim);
        event.address = static_cast<u32>(channel);
        event.value = level ? 1u : 0u;
        std::snprintf(event.detail.data(), event.detail.size(),
                      "ISM DMA%d request %s", channel,
                      level ? "assert" : "clear");
        hardwareTrace_.record(std::move(event));
    };
    ismIop_->onDmaTransfer = [this](int channel, bool peripheralToRam,
                                    u8 value, u16 map, u16 count) {
        if (!hardwareTrace_.accepts(TraceIop | TraceSwim)) return;
        HardwareTraceEvent event = makeTraceEvent(
            HardwareTraceKind::Dma, HardwareTraceSource::IsmIop,
            TraceIop | TraceSwim);
        event.address = map;
        event.value = value;
        event.auxiliary = (static_cast<u64>(channel & 1) << 63) |
                          (static_cast<u64>(peripheralToRam ? 1u : 0u) << 62) |
                          count;
        event.width = 1;
        if (!peripheralToRam) event.flags = 1;
        std::snprintf(event.detail.data(), event.detail.size(),
                      "ISM DMA%d transfer %s", channel,
                      peripheralToRam ? "SWIM-to-RAM" : "RAM-to-SWIM");
        hardwareTrace_.record(std::move(event));
    };
    ismIop_->onDmaWrite = [this](int channel, u8 reg, u8 value) {
        if (hardwareTrace_.enabled() && !hardwareTrace_.frozen()) {
            HardwareTraceEvent event = makeTraceEvent(
                HardwareTraceKind::Dma, HardwareTraceSource::IsmIop,
                TraceIop | TraceSwim);
            event.address = static_cast<u32>(channel * 8 + reg);
            event.value = value;
            event.width = 1;
            event.flags = 1;
            std::snprintf(event.detail.data(), event.detail.size(),
                          "ISM DMA%d register %u", channel,
                          static_cast<unsigned>(reg));
            hardwareTrace_.record(std::move(event));
        }
        if (!nativeStorage_ || !onDiag) return;
        char message[112];
        std::snprintf(message, sizeof message,
                      "IOP ISM DMA%d reg%u=%02X -> ctrl=%02X map=%04X count=%04X req=%d",
                      channel, reg, value, ismIop_->dmaControl(channel),
                      ismIop_->dmaMap(channel), ismIop_->dmaCount(channel),
                      ismIop_->dmaRequest(channel) ? 1 : 0);
        onDiag(message);
    };
    rtc_->onByte = [this](const char* what, u8 value) {
        if (!onDiag) return;
        char message[48];
        std::snprintf(message, sizeof message, "RTC %s=%02X pc=%08X",
                      what, value, cpu_.pc);
        onDiag(message);
    };
    via_->inA = [] {
        // CPU.ID3..0 = 1101 on PA6,4,2,1.  The undriven PA7 and PA0 inputs
        // are pulled high on the IIfx logic board; this is the pin state
        // returned by a physical-board-compatible IIfx implementation.
        return static_cast<u8>(0xD3);
    };
    via_->inB = [this] {
        // Only PB0 is an external input on a normal, non-parity IIfx: the
        // RTC serial-data line.  PB3..5 are unused because ADB belongs to the
        // ISM IOP, PB6 is the optional RPU's active-low enable output, and
        // PB7 is retained only as the legacy sound-enable output.  In
        // particular, do not make unused pins look like pull-ups.  System 7
        // temporarily makes PB6 an input while probing for the special-order
        // RPU; returning it high falsely identifies a half-present parity
        // subsystem and produces the "parity circuitry is not functioning"
        // startup alert.  This also agrees with the board-level input value
        // used by MAME's IIfx implementation.
        return static_cast<u8>(rtc_->dataOut() ? 0x01u : 0x00u);
    };
    via_->outB = [this](u8 value, u8 ddr) {
        const u8 pins = static_cast<u8>(value | ~ddr);
        rtc_->setLines((pins & 0x01u) != 0,
                       (pins & 0x02u) != 0,
                       (pins & 0x04u) != 0);
    };

    oss_->onInterruptChange = [this] { updateIpl(); };
    scsiDma_->readMemory = [this](u32 address) {
        return rawRead8(address);
    };
    scsiDma_->writeMemory = [this](u32 address, u8 value) {
        // FMC snoops writes from every bus master just as it snoops processor
        // writes. The 68030's own data cache remains software-coherent, which
        // is the contract used by the native IIfx SCSI driver.
        fmcMergeWrite(address, value);
        rawWrite8(address, value);
    };
    scsiDma_->onInterruptChange = [this] {
        oss_->setLevel(Oss::Scsi, scsiDma_->irqAsserted());
    };
    sccIop_->onInterruptChange = [this] {
        oss_->setLevel(Oss::SccIop, sccIop_->irqAsserted());
    };
    ismIop_->onInterruptChange = [this] {
        oss_->setLevel(Oss::IsmIop, ismIop_->irqAsserted());
    };
    // Compatibility mode bypasses the SCC IOP and exposes the Z8530 directly.
    sccIop_->bypassRead = [this](u32 offset) {
        return scc_->read(offset & 6u);
    };
    sccIop_->bypassWrite = [this](u32 offset, u8 value) {
        scc_->write(offset & 6u, value);
    };
    sccIop_->peripheralRead = [this](u8 offset) {
        return scc_->read(static_cast<u32>(offset & 3u) * 2u);
    };
    sccIop_->peripheralWrite = [this](u8 offset, u8 value) {
        scc_->write(static_cast<u32>(offset & 3u) * 2u, value);
    };
    scc_->onDmaRequestChange = [this] { updateSccDmaRequests(); };
    sccIop_->onDmaWrite = [this](int, u8, u8) {
        updateSccDmaRequests();
    };
    ismIop_->peripheralRead = [this](u8 offset) {
        return swimAccess(offset, false);
    };
    ismIop_->peripheralWrite = [this](u8 offset, u8 value) {
        (void)swimAccess(offset, true, value);
    };
    // GPIO0 is the physical ADB connection on the IIfx. GPOut is inverted on
    // the logic board; GPIn samples the resulting open-collector wire.
    ismIop_->gpioRead = [this] {
        return static_cast<u8>(adbBus_->readLine(ismIop_->cpuCycles()) ? 1 : 0);
    };
    ismIop_->gpioWrite = [this](int pin, bool level) {
        if (pin == 0)
            adbBus_->hostDrive(!level, ismIop_->cpuCycles());
    };

    sound_->onIrq = [this](bool level) {
        soundIrq_ = level;
        oss_->setLevel(Oss::Sound, level);
    };
    sound_->onDiag = [this](const char* message) {
        if (onDiag) onDiag(message);
    };
    scc_->onDiag = [this](const char* message) {
        if (onDiag) onDiag(message);
    };
    disk_->onBlockIo = [this](bool write, u32 lba, const u8*, u32 length) {
        if (!onDiag) return;
        char message[96];
        std::snprintf(message, sizeof message,
                      "SCSI1 block %c lba=%u count=%u",
                      write ? 'W' : 'R', lba, length / 512u);
        onDiag(message);
    };

    cpu_.onResetInstruction = [this] {
        via_->reset();
        rtc_->reset();
        oss_->reset();
        sccIop_->reset();
        ismIop_->reset();
        scc_->reset();
        scsiDma_->reset();
        sound_->reset();
        adb_->reset();
        adbBus_->reset();
        swim_->reset();
        swim_->forceIsm();
        floppyDrive_->reset();
        if (!floppy_.empty())
            floppyDrive_->insert(&floppy_, floppyReadOnly_,
                                 floppy_.size() == 1474560u, false);
        swimLstrbPrev_ = false;
        swimActionPrev_ = false;
        swimWriteMarkPrev_ = swimReadMarkPrev_ = false;
        swimReadSynced_ = false;
        swimCrcOut_ = 0xFFFF;
        video_->reset();
        clearFmcCache();
        floppyServiceReady_ = false;
        diskPrimePc_ = 0;
        diskCtlPc_ = 0;
        diskStatusPc_ = 0;
        inDiskDriver_ = false;
        updateIpl();
    };
}

void IifxMachine::reset() {
    overlay_ = true;
    totalCycles_ = 0;
    frameCounter_ = 0;
    viaPhase_ = 0;
    framePhase_ = 0;
    secondPhase_ = 0;
    audioPhase_ = 0;
    audioOut_.clear();
    floppyServiceReady_ = false;
    floppyEventRetryCycles_ = 0;
    hardDiskMountPending_ = !hardDisk_.empty();
    hardDiskMounted_ = false;
    hardDiskMountInFlight_ = false;
    hardDiskMountPb_ = 0;
    hardDiskMountTries_ = 0;
    hardDiskMountDelay_ = 0;
    hardDiskMountResult_ = static_cast<s16>(-32768);
    injectedTrapTimeouts_ = 0;
    diskPrimePc_ = 0;
    diskCtlPc_ = 0;
    diskStatusPc_ = 0;
    inDiskDriver_ = false;
    hardDiskReadCount_ = 0;
    hardDiskWriteCount_ = 0;
    hardDiskTraceBudget_ = 48;
    hardDiskErrorBudget_ = 20;
    soundIrq_ = false;
    std::fill(std::begin(biuRegs_), std::end(biuRegs_), u32{0});
    std::memset(ossExpansionRegs_, 0, sizeof ossExpansionRegs_);
    clearFmcCache();
    fmcCyclePenalty_ = 0;
    fmcHits_ = fmcMisses_ = fmcFills_ = fmcCacheInhibited_ = 0;
    accessLog_.clear();
    hardwareTrace_.reset();
    traceLastPc_ = 0;
    traceSamePcCount_ = 0;
    traceLastScsiPhase_ = -1;
    traceLastScsiCommands_ = 0;
    traceLastSwimMode_ = traceLastSwimHandshake_ = 0xFF;
    traceLastSwimFifo_ = 0xFF;
    traceProtocolScsiPhase_ = -1;
    traceAssertionMask_ = 0;
    traceLastVideoAssertions_ = 0;
    traceLastVideoAcks_ = 0;
    traceLastVideoVblank_ = 0;
    traceVideoIrqFrames_ = 0;
    traceVideoIrqEligibleSince_ = 0;
    traceLastMouseReports_ = 0;
    traceLastMouseV_ = traceLastMouseH_ = 0;
    traceMouseMoveFrame_ = 0;
    traceMouseVramWrites_ = 0;
    traceVideoFrameSamples_ = 0;
    traceLastVideoSerialCommands_ = 0;
    traceLastVideoPriorityFrame_ = 0;
    traceIopOperationHits_ = 0;
    traceFloppyBlock_ = 0;
    traceFloppyTrack_ = traceFloppySide_ = traceFloppySector_ = 0xFF;
    traceCheckpointPending_ = false;
    traceCheckpointReason_.clear();
    via_->reset();
    rtc_->reset();
    oss_->reset();
    sccIop_->reset();
    ismIop_->reset();
    scc_->reset();
    scsiDma_->reset();
    sound_->reset();
    adb_->reset();
    adbBus_->reset();
    swim_->reset();
    swim_->forceIsm();
    floppyDrive_->reset();
    if (!floppy_.empty())
        floppyDrive_->insert(&floppy_, floppyReadOnly_,
                             floppy_.size() == 1474560u, false);
    swimLstrbPrev_ = false;
    swimActionPrev_ = false;
    swimWriteMarkPrev_ = swimReadMarkPrev_ = false;
    swimReadSynced_ = false;
    swimCrcOut_ = 0xFFFF;
    swimDataBytes_ = swimDataWrites_ = 0;
    floppyTrackWrites_ = 0;
    swimDiagBudget_ = 128;
    lowMemoryDiagBudget_ = 96;
    video_->reset();
    cpu_.reset();
    updateIpl();
}

void IifxMachine::updateIpl() {
    oss_->setLevel(Oss::Slot9, videoEnabled_ && video_->irqAsserted());
    oss_->setLevel(Oss::Via1, via_->irqAsserted());
    oss_->setLevel(Oss::Scsi, scsiDma_->irqAsserted());
    oss_->setLevel(Oss::Sound, soundIrq_);
    cpu_.setIrqLevel(oss_->ipl());
}

void IifxMachine::updateSccDmaRequests() {
    for (int channel = 0; channel < 2; ++channel) {
        const u8 control = sccIop_->dmaControl(channel);
        const bool enabled = (control & 0x01u) != 0;
        const bool peripheralToRam = (control & 0x04u) != 0;
        const u8 peripheral = static_cast<u8>(control >> 4);
        sccIop_->setDmaRequest(channel,
            enabled && scc_->dmaRequest(peripheral, peripheralToRam));
    }
}

void IifxMachine::tickDevices(int cpuCycles) {
    if (cpuCycles <= 0) return;
    totalCycles_ += static_cast<u64>(cpuCycles);
    scsiDma_->tick(cpuCycles);
    if (videoEnabled_ && video_->genuineGc())
        video_->tick(static_cast<u64>(cpuCycles));

    viaPhase_ += static_cast<u64>(cpuCycles) * kViaHz;
    while (viaPhase_ >= kCpuHz) {
        viaPhase_ -= kCpuHz;
        via_->tick(1);
    }

    updateFloppyTrack();
    serviceSwimSurface();
    updateSccDmaRequests();
    sccIop_->tick(cpuCycles);
    ismIop_->tick(cpuCycles);
    serviceSwimSurface();
    scc_->tick(cpuCycles);
    sccIop_->setPeripheralInterrupt(scc_->irqAsserted());

    if (floppyEventRetryCycles_ > 0) {
        floppyEventRetryCycles_ -= cpuCycles;
        if (floppyEventRetryCycles_ <= 0) {
            floppyEventRetryCycles_ = 0;
            floppyDrive_->signalMediaChange();
            if (nativeStorage_ && onDiag) {
                char message[128];
                std::snprintf(message, sizeof message,
                              "IOP IIfx Sony event retry asserted SWITCHED "
                              "ticks=%08X SysEvtMask=%04X",
                              read32(0x016Au), read16(0x0144u));
                onDiag(message);
            }
        }
    }

    // ASC consumes samples continuously, independently of how often the front
    // end asks for them.  The guest-selected sample clock is part of emulated
    // device time, so stepping and frame-running have identical phase/FIFO/IRQ
    // behaviour even when the front end does not drain audio promptly.
    audioPhase_ += static_cast<u64>(cpuCycles) * sound_->sampleRate();
    while (audioPhase_ >= kCpuHz) {
        audioPhase_ -= kCpuHz;
        const u8 sample = sound_->pullSample();
        if (audioOut_.size() < 8192) audioOut_.push_back(sample);
    }

    secondPhase_ += static_cast<u64>(cpuCycles);
    while (secondPhase_ >= kCpuHz) {
        secondPhase_ -= kCpuHz;
        rtc_->tickSecond();
        via_->setCA2(true);
        via_->setCA2(false);
        announceHardDisk();
    }

    framePhase_ += static_cast<u64>(cpuCycles) * kFrameNumerator;
    const u64 frameThreshold = kCpuHz * kFrameDenominator;
    while (framePhase_ >= frameThreshold) {
        framePhase_ -= frameThreshold;
        ++frameCounter_;
        if (videoEnabled_ && !video_->genuineGc()) video_->onVblank();
        oss_->pulse(Oss::Clock60Hz);
        if (!hardDisk_.empty() && !diskPrimePc_) findDiskDriverPrime();
    }

    updateIpl();

    if (hardwareTrace_.enabled() && !hardwareTrace_.frozen()) {
        checkHardwareProtocols();
        const int phase = scsi_->phase();
        if (phase != traceLastScsiPhase_ ||
            scsi_->diagCommands != traceLastScsiCommands_) {
            traceState(HardwareTraceKind::State, HardwareTraceSource::Ncr5380,
                       TraceScsi, phase != traceLastScsiPhase_
                           ? "SCSI phase transition" : "SCSI command complete");
            traceLastScsiPhase_ = phase;
            traceLastScsiCommands_ = scsi_->diagCommands;
        }
        const u8 handshake = swim_->diagnosticHandshake();
        if (swim_->ismMode() != traceLastSwimMode_ ||
            handshake != traceLastSwimHandshake_ ||
            swim_->ismFifoOccupancy() != traceLastSwimFifo_) {
            traceState(HardwareTraceKind::State, HardwareTraceSource::Swim,
                       TraceSwim | TraceIop, "SWIM state transition");
            traceLastSwimMode_ = swim_->ismMode();
            traceLastSwimHandshake_ = handshake;
            traceLastSwimFifo_ = swim_->ismFifoOccupancy();
        }
    }
}

// The ROM loads the disk's driver from its Apple partition map and installs a
// handle in unit-table slot 32. Cache the three Device Manager entry points once
// that has happened. Prime still enters the guest DRVR normally; the machine
// only substitutes the byte transfer that the IIfx old-SCSI API otherwise
// drains without copying into the caller's buffer.
void IifxMachine::findDiskDriverPrime() {
    const u32 unitTable = guestPtr(read32(0x011Cu));
    if (!unitTable || unitTable + (kScsiDiskUnit + 1u) * 4u > ram_.size()) return;
    const u32 handle = guestPtr(read32(unitTable + kScsiDiskUnit * 4u));
    if (!handle || handle + 4u > ram_.size()) return;
    const u32 dce = guestPtr(read32(handle));
    if (!dce || dce + 4u > ram_.size()) return;
    const u32 driver = guestPtr(read32(dce));             // dCtlDriver
    if (!driver || driver + 0x20u > ram_.size()) return;

    // Do not capture some unrelated unit-one driver. Both OpenMac's portable
    // disk driver and Apple's period SCSI disk driver use the canonical name.
    if (read8(driver + 0x12u) != 7 || read8(driver + 0x13u) != '.' ||
        read8(driver + 0x14u) != 'S' || read8(driver + 0x15u) != 'c')
        return;

    diskPrimePc_ = driver + read16(driver + 0x0Au);
    diskCtlPc_ = driver + read16(driver + 0x0Cu);
    diskStatusPc_ = driver + read16(driver + 0x0Eu);
    if (onDiag) {
        char message[112];
        std::snprintf(message, sizeof message,
                      "IIfx .ScsiHD unit 32 Prime=%08X Control=%08X Status=%08X",
                      diskPrimePc_, diskCtlPc_, diskStatusPc_);
        onDiag(message);
    }
}

void IifxMachine::serveDiskPrime() {
    const u32 pb = guestPtr(cpu_.a[0]);
    const u32 dce = guestPtr(cpu_.a[1]);
    const u16 trap = read16(pb + 0x06u);
    const u32 buffer = guestPtr(read32(pb + 0x20u));
    const u32 requested = read32(pb + 0x24u);
    const u16 posMode = read16(pb + 0x2Cu);
    const u32 posOffset = read32(pb + 0x2Eu);
    const u32 mark = read32(dce + 0x10u);
    const u32 volumeBytes = hfsVolumeBytes_;

    u32 position = mark;
    switch (posMode & 3u) {
    case 1: position = posOffset; break;                  // fsFromStart
    case 2: position = volumeBytes + posOffset; break;    // fsFromLEOF
    case 3: position = mark + posOffset; break;           // fsFromMark
    default: break;                                       // fsAtMark
    }

    s16 result = 0;
    u32 completed = 0;
    if (volumeBytes == 0 || scsiImage_.empty() ||
        static_cast<u64>(hfsImageOffset_) + volumeBytes > scsiImage_.size()) {
        result = -65;                                     // offLinErr
    } else if ((trap & 1u) && disk_->readOnly) {
        result = -44;                                     // wPrErr
    } else if (position >= volumeBytes) {
        result = -39;                                     // eofErr
    } else {
        u32 count = requested;
        if (count > volumeBytes - position) {
            count = volumeBytes - position;
            result = -39;
        }
        const std::size_t at = static_cast<std::size_t>(hfsImageOffset_) + position;
        if (trap & 1u) {
            for (u32 i = 0; i < count; ++i)
                scsiImage_[at + i] = read8(buffer + i);
        } else {
            for (u32 i = 0; i < count; ++i)
                write8(buffer + i, scsiImage_[at + i]);
            cpu_.invalidateDataCache030(buffer, count);
        }
        completed = count;
    }

    if (trap & 1u) ++hardDiskWriteCount_;
    else ++hardDiskReadCount_;
    if (onDiag && hardDiskTraceBudget_ > 0) {
        --hardDiskTraceBudget_;
        char message[144];
        std::snprintf(message, sizeof message,
                      "IIfx HDIO %s pos=%u block=%u req=%u done=%u result=%d",
                      (trap & 1u) ? "write" : "read", position,
                      position / 512u, requested, completed, result);
        onDiag(message);
    }
    if (result != 0 && onDiag && hardDiskErrorBudget_ > 0) {
        --hardDiskErrorBudget_;
        char message[144];
        std::snprintf(message, sizeof message,
                      "IIfx hard disk %s failed result=%d pos=%u req=%u size=%u",
                      (trap & 1u) ? "write" : "read", result, position,
                      requested, volumeBytes);
        onDiag(message);
    }

    write32(pb + 0x28u, completed);                     // ioActCount
    write32(dce + 0x10u, position + completed);         // dCtlPosition
    write16(pb + 0x10u, static_cast<u16>(result));      // ioResult
    cpu_.invalidateDataCache030(pb, 0x40u);
    cpu_.invalidateDataCache030(dce, 0x20u);
    cpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const u32 target = read32(0x08FCu);                 // jIODone
    cpu_.a[0] = target;
    cpu_.pc = target;
}

void IifxMachine::serveDiskCtlStatus(bool status) {
    const u32 pb = guestPtr(cpu_.a[0]);
    const u16 csCode = read16(pb + 0x1Au);
    s16 result = 0;
    if (status) {
        if (csCode == 6) {                              // return format list
            write16(pb + 0x1Cu, 1);
            write32(pb + 0x1Eu, hfsVolumeBytes_ / 512u);
        }
    } else if (csCode == 1 || csCode == 21 || csCode == 22) {
        // Fixed disks cannot be killed/ejected, and refusing pointer-valued
        // icon requests makes callers use their built-in hard-disk artwork.
        result = -17;                                   // controlErr
    }
    write16(pb + 0x10u, static_cast<u16>(result));
    cpu_.invalidateDataCache030(pb, 0x40u);
    cpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const u32 target = read32(0x08FCu);
    cpu_.a[0] = target;
    cpu_.pc = target;
}

int IifxMachine::stepInstruction() {
    const bool recordCpu = hardwareTrace_.accepts(
        TraceCpu, HardwareTraceSource::Cpu);
    const bool watchPc = hardwareTrace_.enabled() &&
        !hardwareTrace_.frozen() && !hardwareTrace_.triggered() &&
        hardwareTrace_.config().triggerPcHits != 0;
    if (recordCpu || watchPc) {
        if (recordCpu) checkTraceStall(cpu_.pc);

        // PC triggers must remain usable with a bus-only trace.  Feeding a
        // minimal, filtered instruction record lets HardwareTrace count the
        // hit without filling the ring (or paying to snapshot every device)
        // on every 68030 instruction.
        HardwareTraceEvent event;
        if (recordCpu) {
            event = makeTraceEvent(HardwareTraceKind::Instruction,
                                   HardwareTraceSource::Cpu, TraceCpu);
            event.value = cpu_.getSR();
        } else {
            event.cycle = totalCycles_;
            event.correlation = activeMediaEventId_;
            event.kind = HardwareTraceKind::Instruction;
            event.source = HardwareTraceSource::Cpu;
            event.categories = TraceCpu;
        }
        event.pc = cpu_.pc;
        hardwareTrace_.record(std::move(event));
    }
    if (onStep) onStep(cpu_.pc);
    if (!nativeStorage_ && diskPrimePc_ && cpu_.pc == diskPrimePc_) {
        inDiskDriver_ = true;
        try {
            serveDiskPrime();
        } catch (const BusFault& fault) {
            if (onDiag) {
                char message[128];
                std::snprintf(message, sizeof message,
                              "IIfx disk Prime fault pb=%08X dce=%08X addr=%08X",
                              cpu_.a[0], cpu_.a[1], fault.addr);
                onDiag(message);
            }
            cpu_.d[0] = static_cast<u32>(static_cast<s32>(-36));
            cpu_.pc = read32(0x08FCu);
            cpu_.a[0] = cpu_.pc;
        }
        inDiskDriver_ = false;
        tickDevices(40);
        return 40;
    }
    if (!nativeStorage_ &&
        ((diskCtlPc_ && cpu_.pc == diskCtlPc_) ||
         (diskStatusPc_ && cpu_.pc == diskStatusPc_))) {
        const bool status = diskStatusPc_ && cpu_.pc == diskStatusPc_;
        inDiskDriver_ = true;
        try {
            serveDiskCtlStatus(status);
        } catch (const BusFault& fault) {
            if (onDiag) {
                char message[120];
                std::snprintf(message, sizeof message,
                              "IIfx disk Control/Status fault pb=%08X addr=%08X",
                              cpu_.a[0], fault.addr);
                onDiag(message);
            }
            cpu_.d[0] = static_cast<u32>(static_cast<s32>(-36));
            cpu_.pc = read32(0x08FCu);
            cpu_.a[0] = cpu_.pc;
        }
        inDiskDriver_ = false;
        tickDevices(40);
        return 40;
    }
    if (!nativeStorage_ &&
        (cpu_.pc == kSonyPrime || cpu_.pc == kSonyPrimeAlias ||
         cpu_.pc == kSonyPrimeHandler || cpu_.pc == kSonyPrimeHandlerAlias)) {
        try {
            serveSonyPrime();
        } catch (const BusFault& fault) {
            if (onDiag) {
                char message[112];
                std::snprintf(message, sizeof message,
                              "sony prime fault pb=%08X dce=%08X addr=%08X",
                              cpu_.a[0], cpu_.a[1], fault.addr);
                onDiag(message);
            }
            cpu_.d[0] = static_cast<u32>(static_cast<s32>(-36));
            cpu_.pc = read32(0x08FC);
            cpu_.a[0] = cpu_.pc;
        }
        tickDevices(40);
        return 40;
    }
    if (!nativeStorage_ &&
        (cpu_.pc == kSonyControl || cpu_.pc == kSonyControlAlias ||
         cpu_.pc == kSonyControlHandler ||
         cpu_.pc == kSonyControlHandlerAlias)) {
        serveSonyCtlStatus(false);
        tickDevices(40);
        return 40;
    }
    if (!nativeStorage_ &&
        (cpu_.pc == kSonyStatus || cpu_.pc == kSonyStatusAlias)) {
        serveSonyCtlStatus(true);
        tickDevices(40);
        return 40;
    }
    const int cycles = cpu_.step();
    tickDevices(cycles);
    return cycles;
}

void IifxMachine::runFrame() {
    const u64 target = frameCounter_ + 1;
    while (frameCounter_ < target && !cpu_.halted && !poweredOff())
        stepInstruction();
}

bool IifxMachine::poweredOff() const { return oss_->poweredOff(); }

bool IifxMachine::diagnosticIsmFirmwareAlive() const {
    return ismIop_->firmwareAlive();
}

bool IifxMachine::diagnosticFloppyMounted() {
    return volumeMountedFor(1);
}

void IifxMachine::flushFloppyTrack() {
    SonyDrive& drive = *floppyDrive_;
    if (!drive.trackDirty() || !drive.hasDisk() || drive.readOnly) {
        drive.clearTrackDirty();
        return;
    }
    std::vector<u8>& image = *drive.image;
    u32 good = 0;
    const int sectors = drive.hdMedia
        ? mfm::decodeTrack(drive.trackData(), drive.trackMarks(), image,
                           drive.cacheTrack, drive.cacheSide, &good)
        : gcr::decodeTrack(drive.trackData(), image, drive.cacheTrack,
                           image.size() > 440u * 1024u ? 2 : 1);
    drive.clearTrackDirty();
    ++floppyTrackWrites_;
    if (onDiag) {
        char message[128];
        std::snprintf(message, sizeof message,
                      "IOP SWIM track %d side %d write-back: %d sectors%s%08X",
                      drive.cacheTrack, drive.cacheSide, sectors,
                      drive.hdMedia ? " mask=" : "", drive.hdMedia ? good : 0u);
        onDiag(message);
    }
}

void IifxMachine::updateFloppyTrack() {
    SonyDrive& drive = *floppyDrive_;
    if (!drive.hasDisk()) return;
    const int side = drive.headUpper ? 1 : 0;
    if (drive.trackLoaded() && drive.cacheTrack == drive.track &&
        drive.cacheSide == side && drive.cacheGen == drive.mediaGen)
        return;

    flushFloppyTrack();
    const std::vector<u8>& image = *drive.image;
    if (drive.track < 0 || drive.track >= 80) {
        drive.invalidateTrack();
        return;
    }
    if (drive.hdMedia) {
        std::vector<u8> bytes, marks;
        mfm::buildTrack(image, drive.track, side, bytes, marks);
        drive.setTrackData(std::move(bytes), std::move(marks));
    } else {
        const int sides = image.size() > 440u * 1024u ? 2 : 1;
        if (side >= sides) {
            drive.invalidateTrack();
            return;
        }
        drive.setTrackData(gcr::buildTrack(image, drive.track, side, sides,
                                            sides == 2 ? 0x22 : 0x02));
    }
    drive.cacheTrack = drive.track;
    drive.cacheSide = side;
    drive.cacheGen = drive.mediaGen;
}

void IifxMachine::serviceSwimSurface() {
    // Mode bits 1 and 2 are independent active-low output enables, not a
    // two-bit drive selector. The first IIfx internal connector is /ENBL1.
    const bool drive1Enabled = (swim_->ismMode() & 0x82u) == 0x82u;
    const bool action = drive1Enabled && swim_->ismAction();
    if (action && !swimActionPrev_) {
        // ACTION starts the SWIM mark-search state machine.  The platter keeps
        // rotating while the chip ignores ordinary gap bytes; it does not jump
        // instantly to the next mark.  Preserving that elapsed gap gives the
        // downloaded IOP firmware the real hardware's window to arm DMA before
        // the data field reaches the head.
        if (!swim_->ismWriting()) swimReadSynced_ = false;
        swimCrcOut_ = 0xFFFF;
        swimWriteMarkPrev_ = swimReadMarkPrev_ = false;
    }
    if (!action) swimReadSynced_ = false;
    swimActionPrev_ = action;

    if (!action || !floppyDrive_->hasDisk()) {
        ismIop_->setDmaRequest(0, false);
        ismIop_->setDmaRequest(1, false);
        return;
    }

    if (!swim_->ismWriting()) {
        while (swim_->ismFifoOccupancy() < 2u) {
            u8 byte = 0;
            bool mark = false;
            if (!floppyDrive_->nextByte(totalCycles_, &byte, &mark)) break;
            if (!swimReadSynced_) {
                if (!mark) continue;
                swimReadSynced_ = true;
                swimReadMarkPrev_ = false;
            }
            if (mark && !swimReadMarkPrev_) swim_->ismCrcReset();
            swimReadMarkPrev_ = mark;
            swim_->ismCrcAdd(byte);
            if (!swim_->ismPushReadByte(byte, mark)) break;
            ++swimDataBytes_;
            if (hardwareTrace_.accepts(TraceSwim | TraceIop)) {
                HardwareTraceEvent event = makeTraceEvent(
                    HardwareTraceKind::State, HardwareTraceSource::Swim,
                    TraceSwim | TraceIop);
                event.address = static_cast<u32>(floppyDrive_->bytePos());
                event.value = byte;
                event.auxiliary = (static_cast<u64>(mark ? 1u : 0u) << 32) |
                                  swim_->ismFifoOccupancy();
                std::snprintf(event.detail.data(), event.detail.size(),
                              "SWIM FIFO push%s occupancy=%u",
                              mark ? " mark" : "",
                              static_cast<unsigned>(swim_->ismFifoOccupancy()));
                hardwareTrace_.record(std::move(event));
            }
        }
    } else {
        swim_->ismDataReady = floppyDrive_->writeReady(totalCycles_);
        if (swim_->ismWroteData || swim_->ismWroteMark) {
            const bool mark = swim_->ismWroteMark;
            if (mark && !swimWriteMarkPrev_) swimCrcOut_ = 0xFFFF;
            swimWriteMarkPrev_ = mark;
            floppyDrive_->writeByte(totalCycles_, swim_->ismWritten, mark);
            swimCrcOut_ = mfm::crc16Update(swimCrcOut_, swim_->ismWritten);
            swim_->ismWroteData = swim_->ismWroteMark = false;
            ++swimDataWrites_;
        }
        if (swim_->ismWroteCrc) {
            swim_->ismWroteCrc = false;
            floppyDrive_->writeCrc(totalCycles_,
                                   static_cast<u8>(swimCrcOut_ >> 8),
                                   static_cast<u8>(swimCrcOut_));
            swimDataWrites_ += 2;
        }
    }

    // DAT1BYTE is physically wired to REQA on the IIfx SWIM PIC. Channel B is
    // independent and must remain deasserted.
    const bool dmaReady = swim_->ismWriting()
        ? swim_->ismDataReady : swim_->ismFifoOccupancy() != 0;
    ismIop_->setDmaRequest(0, dmaReady);
    ismIop_->setDmaRequest(1, false);
}

u8 IifxMachine::swimAccess(u8 offset, bool write, u8 value) {
    floppyDrive_->readOnly = floppyReadOnly_;
    floppyDrive_->hdMedia = floppy_.size() == 1474560u;
    const bool drive1Selected = (swim_->ismMode() & 0x02u) != 0;
    const bool drive2Selected = (swim_->ismMode() & 0x04u) != 0;
    // Figure 9-14 of Guide to the Macintosh Family Hardware shows /ENBL1
    // and /ENBL2 wired to the two internal IIfx connectors. This model has a
    // mechanism in connector 1 only; connector 2 therefore floats SENSE.
    const bool enabled = swim_->ismSelected()
        ? ((swim_->ismMode() & 0x80u) != 0 && drive1Selected)
        : swim_->motorOn();
    floppyDrive_->setEnabled(enabled, totalCycles_);

    // The internal drive occupies SWIM device-select A.  Its fourth Sony
    // status/command address bit is the dedicated HDSEL output on the 44-pin
    // SWIM used here, driven by Mode bit 5.  Setup bit 0 controls only the
    // alternate Q3/HDSEL pin used by smaller packages; gating the dedicated
    // output on that bit shifts every firmware query to the neighbouring Sony
    // register.  The downloaded IIfx firmware makes the wiring observable:
    // Mode.5 + Phase $F0 reads CSTIN ($1), $F4/$FC strobes SWITCHED-reset
    // ($9), $F7 reads high-density media ($F), and $F1 reads WRTPRT ($3).
    const bool driveSel = (swim_->ismMode() & 0x20u) != 0;
    const int driveAddress = swim_->driveRegister(driveSel);
    swim_->senseHigh = floppyDrive_->sense(driveAddress, totalCycles_);
    if (!drive1Selected || drive2Selected)
        swim_->senseHigh = true;
    // The IIfx mechanism returns its multiplexed status on the drive RD line,
    // which enters SWIM as RDDATA. Its downloaded firmware samples handshake
    // bit 2 while probing addresses $F/$E/$C/$A and indexes the resulting
    // four-bit signature to distinguish 400K, 800K, FDHD and HD20 devices.
    // SENSE is a separate SWIM input; mirror the physical line there as well
    // for the controller's ordinary status-register view.
    swim_->readDataHigh = swim_->senseHigh;
    floppyDrive_->headUpper = driveSel;
    updateFloppyTrack();
    serviceSwimSurface();
    const u8 fifoBefore = swim_->ismFifoOccupancy();
    const u8 fifoHeadBefore = swim_->ismFifoHeadData();
    const u8 result = swim_->access(offset & 15u, write, value);
    const u8 fifoAfterAccess = swim_->ismFifoOccupancy();
    if (fifoAfterAccess != fifoBefore &&
        hardwareTrace_.accepts(TraceSwim | TraceIop)) {
        HardwareTraceEvent event = makeTraceEvent(
            HardwareTraceKind::State, HardwareTraceSource::Swim,
            TraceSwim | TraceIop);
        event.address = offset & 15u;
        event.value = fifoHeadBefore;
        event.auxiliary = (static_cast<u64>(fifoBefore) << 32) |
                          fifoAfterAccess;
        std::snprintf(event.detail.data(), event.detail.size(),
                      "SWIM FIFO %s occupancy=%u->%u",
                      !write && ((offset & 15u) == 8u ||
                                 (offset & 15u) == 9u) ? "pop" : "transition",
                      static_cast<unsigned>(fifoBefore),
                      static_cast<unsigned>(fifoAfterAccess));
        hardwareTrace_.record(std::move(event));
    }
    const bool newDriveSel = (swim_->ismMode() & 0x20u) != 0;
    floppyDrive_->headUpper = newDriveSel;
    updateFloppyTrack();
    serviceSwimSurface();

    traceAccess(HardwareTraceSource::Swim, write, offset & 15u,
                write ? value : result, 1, "ISM peripheral register");

    if (onDiag && swimDiagBudget_-- > 0) {
        char message[192];
        std::snprintf(message, sizeof message,
                      "IOP SWIM %c port=%X value=%02X result=%02X phase=%02X "
                      "mode=%02X setup=%02X drive=%X d1=%d d2=%d sense=%d "
                      "enabled=%d present=%d bytes=%zu",
                      write ? 'W' : 'R', offset & 15u, value, result,
                      static_cast<unsigned>(swim_->lines()), swim_->ismMode(),
                      swim_->ismSetup(), driveAddress,
                      drive1Selected ? 1 : 0, drive2Selected ? 1 : 0,
                      swim_->senseHigh ? 1 : 0,
                      enabled ? 1 : 0, floppyDrive_->hasDisk() ? 1 : 0,
                      floppy_.size());
        onDiag(message);
    }

    const bool strobe = swim_->lstrb();
    if (strobe && !swimLstrbPrev_ && drive1Selected && !drive2Selected) {
        const int address = swim_->driveRegister(newDriveSel);
        const int command = address & 7;
        const bool data = (address & 8) != 0;
        const int trackBefore = floppyDrive_->track;
        const bool directionBefore = floppyDrive_->stepIn;
        floppyDrive_->command(address & 7, (address & 8) != 0,
                              totalCycles_);
        if (hardwareTrace_.accepts(TraceSwim | TraceIop)) {
            HardwareTraceEvent event = makeTraceEvent(
                HardwareTraceKind::State, HardwareTraceSource::Swim,
                TraceSwim | TraceIop);
            event.address = static_cast<u32>(address);
            event.value = static_cast<u64>(command) |
                          (static_cast<u64>(data ? 1u : 0u) << 8) |
                          (static_cast<u64>(trackBefore & 0xFF) << 16) |
                          (static_cast<u64>(floppyDrive_->track & 0xFF) << 24);
            event.auxiliary = (static_cast<u64>(directionBefore ? 1u : 0u) << 63) |
                              (static_cast<u64>(floppyDrive_->stepIn ? 1u : 0u) << 62) |
                              floppyDrive_->stepRemaining(totalCycles_);
            static constexpr const char* kCommandName[8] = {
                "DIRTN", "SWITCHED-ACK", "STEP", "FORMAT", "MOTOR",
                "COMMAND5", "EJECT", "COMMAND7"};
            std::snprintf(event.detail.data(), event.detail.size(),
                "SuperDrive %s data=%u track=%d->%d direction=%s busy=%u",
                kCommandName[command], data ? 1u : 0u, trackBefore,
                floppyDrive_->track, floppyDrive_->stepIn ? "in" : "out",
                floppyDrive_->stepping(totalCycles_) ? 1u : 0u);
            hardwareTrace_.record(std::move(event));
        }
    }
    swimLstrbPrev_ = strobe;
    floppyDrive_->tickEject(totalCycles_);
    if (floppyDrive_->takeEjectRequest()) ejectFloppy();
    return result;
}

std::vector<u8> IifxMachine::iopRamImage(bool ism) const {
    std::vector<u8> image(0x8000u);
    const IifxIop& iop = ism ? *ismIop_ : *sccIop_;
    for (u32 address = 0; address < image.size(); ++address)
        image[address] = iop.ram(static_cast<u16>(address));
    return image;
}

void IifxMachine::insertHardDisk(std::vector<u8> image, bool readOnly) {
    hardDisk_ = std::move(image);
    disk_->readOnly = readOnly;
    scsiImage_.clear();
    hfsImageOffset_ = 0;
    hfsVolumeBytes_ = 0;
    hardDiskIsWholeScsi_ = false;
    diskPrimePc_ = 0;
    diskCtlPc_ = 0;
    diskStatusPc_ = 0;
    hardDiskReadCount_ = 0;
    hardDiskWriteCount_ = 0;
    hardDiskTraceBudget_ = 48;
    hardDiskErrorBudget_ = 20;
    hardDiskMountPending_ = !hardDisk_.empty();
    hardDiskMounted_ = false;
    hardDiskMountInFlight_ = false;
    hardDiskMountPb_ = 0;
    hardDiskMountTries_ = 0;
    hardDiskMountDelay_ = 0;
    hardDiskMountResult_ = static_cast<s16>(-32768);
    if (hardDisk_.empty()) {
        disk_->detach();
        return;
    }

    if (hardDisk_.size() >= 512 && hardDisk_[0] == 0x45 && hardDisk_[1] == 0x52) {
        // A complete physical SCSI image: preserve its partitioning and driver.
        scsiImage_ = hardDisk_;
        hardDiskIsWholeScsi_ = true;
        if (!locateHfsPartition(scsiImage_, hfsImageOffset_, hfsVolumeBytes_) &&
            onDiag)
            onDiag("IIfx physical SCSI image has no valid Apple_HFS partition");
    } else {
        // OpenMac normally stores a bare HFS volume on the host. Present it as
        // a period SCSI disk with DDM, APM, and a ROM-independent 68030 driver.
        // The IIfx ROM's startup-device wait derives refnum -33 from the OSS
        // SCSI slot identity and explicitly searches the drive queue for that
        // unit.  Using the Classic-era -2 refnum leaves a perfectly valid
        // drive linked but makes the ROM wait twenty seconds and skip its
        // startup mount path.
        scsiImage_ = scsi::buildAppleScsiDisk(
            hardDisk_, scsi::buildScsiDriverPortable(
                           0, 4, static_cast<u8>(kScsiDiskUnit)));
        hfsImageOffset_ = static_cast<u32>(scsiImage_.size() - hardDisk_.size());
        hfsVolumeBytes_ = static_cast<u32>(hardDisk_.size());
    }
    disk_->attach(&scsiImage_, 0);
}

const std::vector<u8>& IifxMachine::hardDiskImage() const {
    if (scsiImage_.empty()) return hardDisk_;
    if (hardDiskIsWholeScsi_) {
        hardDisk_ = scsiImage_;
    } else if (static_cast<std::size_t>(hfsImageOffset_) + hardDisk_.size() <=
               scsiImage_.size()) {
        hardDisk_.assign(scsiImage_.begin() + hfsImageOffset_,
                         scsiImage_.begin() + hfsImageOffset_ + hardDisk_.size());
    }
    return hardDisk_;
}

bool IifxMachine::floppyGeometry(std::vector<u8>& image) {
    if (macbinary::isMacBinary(image)) macbinary::split(image);
    if (dc42::isDiskCopy(image)) dc42::split(image);
    const std::size_t bytes = image.size();
    return bytes == 409600 || bytes == 819200 || bytes == 1474560;
}

int IifxMachine::insertFloppy(std::vector<u8> image, bool readOnly) {
    const char* container = "raw image";
    // Peel a candidate into locals.  A drive does not throw out the disk that
    // is already in it merely because the next host file is not valid media;
    // keeping the live wrapper state untouched until validation succeeds also
    // means a refused swap cannot turn a DiskCopy/MacBinary disk into a raw
    // file when it is written back later.
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
        if (onDiag) {
            char message[144];
            std::snprintf(message, sizeof message,
                          "floppy refused: %zu bytes (%s); need 400K, 800K or 1.44 MB",
                          bytes, container);
            onDiag(message);
        }
        return 0;
    }

    flushFloppyTrack();
    fdMbHeader_ = std::move(mbHeader);
    fdMbResource_ = std::move(mbResource);
    fdDcHeader_ = std::move(dcHeader);
    fdDcTags_ = std::move(dcTags);
    floppy_ = std::move(image);
    floppyEjected_.clear();
    floppyReadOnly_ = readOnly;
    floppyDrive_->insert(&floppy_, floppyReadOnly_,
                         floppy_.size() == 1474560u,
                         floppyServiceReady_);
    // Media present at construction is still the first physical media event.
    // reset() deliberately preserves the host-mounted image, so assign its ID
    // here where the image actually enters the mechanism.
    activeMediaEventId_ = ++mediaEventId_;
    if (hardwareTrace_.enabled() && !hardwareTrace_.frozen()) {
        HardwareTraceEvent event = makeTraceEvent(
            HardwareTraceKind::Milestone, HardwareTraceSource::Swim,
            TraceSwim | TraceIop | TraceMilestone);
        event.address = 1; // insert
        event.value = bytes;
        event.auxiliary = (static_cast<u64>(readOnly ? 1u : 0u) << 1) |
                          (floppyServiceReady_ ? 1u : 0u);
        std::snprintf(event.detail.data(), event.detail.size(),
                      "floppy media inserted event=%llu",
                      static_cast<unsigned long long>(activeMediaEventId_));
        hardwareTrace_.record(std::move(event));
    }
    if (onDiag) {
        char message[128];
        std::snprintf(message, sizeof message,
                      "floppy: %s disk in IIfx internal SuperDrive (%s%s)",
                      bytes == 1474560 ? "1.44 MB" : bytes == 819200 ? "800K" : "400K",
                      container, readOnly ? ", write-protected" : "");
        onDiag(message);
    }
    return 1;
}

void IifxMachine::ejectFloppy() {
    if (floppy_.empty()) return;
    if (!diagnosticFloppyMounted() && hardwareTrace_.enabled() &&
        !hardwareTrace_.triggered())
        triggerTrace("SuperDrive ejected media before mount");
    activeMediaEventId_ = ++mediaEventId_;
    flushFloppyTrack();
    floppyEjected_ = std::move(floppy_);
    floppy_.clear();
    floppyDrive_->removeDisk();
    if (hardwareTrace_.enabled() && !hardwareTrace_.frozen()) {
        HardwareTraceEvent event = makeTraceEvent(
            HardwareTraceKind::Milestone, HardwareTraceSource::Swim,
            TraceSwim | TraceIop | TraceMilestone);
        event.address = 2; // eject
        event.value = floppyEjected_.size();
        std::snprintf(event.detail.data(), event.detail.size(),
                      "floppy media ejected event=%llu",
                      static_cast<unsigned long long>(activeMediaEventId_));
        hardwareTrace_.record(std::move(event));
    }
    if (onDiag) onDiag("floppy: disk ejected from IIfx internal SuperDrive");
}

std::vector<u8> IifxMachine::floppyFileImage(
    const std::vector<u8>& sectors) const {
    std::vector<u8> result = sectors;
    if (result.empty()) return result;
    if (fdDcHeader_.size() == dc42::kHeaderSize) {
        dc42::Parts parts;
        parts.header = fdDcHeader_;
        parts.tags = fdDcTags_;
        result = dc42::rewrap(parts, result);
    }
    if (fdMbHeader_.size() == macbinary::kHeaderSize) {
        macbinary::Parts parts;
        parts.header = fdMbHeader_;
        parts.resource = fdMbResource_;
        result = macbinary::rewrap(parts, result);
    }
    return result;
}

std::vector<u8> IifxMachine::floppyForWriteBack() {
    flushFloppyTrack();
    return floppyFileImage(floppy_.empty() ? floppyEjected_ : floppy_);
}

u32 IifxMachine::guestPtr(u32 address) const {
    if (address < ram_.size()) return address;
    const u32 low24 = address & 0x00FFFFFFu;
    return low24 < ram_.size() ? low24 : address;
}

bool IifxMachine::volumeMountedFor(u16 drive) {
    u32 vcb = read32(0x0358u);
    for (int count = 0; count < 16 && vcb && vcb != 0xFFFFFFFFu; ++count) {
        vcb = guestPtr(vcb);
        if (vcb + 80u > ram_.size()) return false;
        if (read16(vcb + 72u) == drive) return true;
        vcb = read32(vcb);
    }
    return false;
}

bool IifxMachine::markHardDiskClean() {
    if (disk_->readOnly || hfsVolumeBytes_ < 2048u || scsiImage_.empty())
        return false;
    bool changed = false;
    for (u32 volumeOffset : {1024u, hfsVolumeBytes_ - 1024u}) {
        const std::size_t at = static_cast<std::size_t>(hfsImageOffset_) +
                               volumeOffset;
        if (at + 12u > scsiImage_.size()) continue;
        if (scsiImage_[at] != 0x42 || scsiImage_[at + 1u] != 0x44) continue;
        // HFS drAtrb bit 8 records a clean unmount. A successful FlushVol has
        // made that statement true even when the startup volume itself remains
        // busy and cannot be removed from the VCB queue.
        if ((scsiImage_[at + 0x0Au] & 1u) == 0) {
            scsiImage_[at + 0x0Au] |= 1u;
            changed = true;
        }
    }
    return changed;
}

bool IifxMachine::shutdownHardDisk() {
    if (hardDisk_.empty() || inDiskDriver_ || hardDiskMountInFlight_)
        return false;
    if (!volumeMountedFor(4)) return false;
    if (read32(0x0360u) == 0xFFFFFFFFu) return false;  // FS queue not built

    hardDiskMountInFlight_ = true;
    u32 savedD[8], savedA[8];
    for (int i = 0; i < 8; ++i) {
        savedD[i] = cpu_.d[i];
        savedA[i] = cpu_.a[i];
    }

    if (!hardDiskMountPb_) {
        cpu_.d[0] = 80;
        if (execute68kTrap(0xA71Eu))                  // _NewPtr,Sys,Clear
            hardDiskMountPb_ = guestPtr(cpu_.a[0]);
    }

    s16 flushed = -108, unmounted = -108;
    bool marked = false;
    if (hardDiskMountPb_ && hardDiskMountPb_ + 80u <= ram_.size()) {
        auto call = [this](u16 trap) -> s16 {
            for (u32 i = 0; i < 80; ++i) write8(hardDiskMountPb_ + i, 0);
            write16(hardDiskMountPb_ + 22u, 4);       // ioVRefNum = drive 4
            cpu_.a[0] = hardDiskMountPb_;
            if (!execute68kTrap(trap)) return -36;    // ioErr
            return static_cast<s16>(cpu_.d[0] & 0xFFFFu);
        };
        flushed = call(0xA013u);                      // _FlushVol
        unmounted = call(0xA00Eu);                    // _UnmountVol
        if (flushed == 0 && unmounted != 0) marked = markHardDiskClean();
    }

    if (unmounted == 0) hardDiskMounted_ = false;
    hardDiskMountPending_ = false;
    if (onDiag) {
        char message[128];
        std::snprintf(message, sizeof message,
                      "IIfx drive 4 shutdown flush=%d unmount=%d%s",
                      flushed, unmounted,
                      marked ? ", marked cleanly unmounted" : "");
        onDiag(message);
    }

    for (int i = 0; i < 8; ++i) {
        cpu_.d[i] = savedD[i];
        cpu_.a[i] = savedA[i];
    }
    hardDiskMountInFlight_ = false;
    return flushed == 0 && (unmounted == 0 || marked || disk_->readOnly);
}

bool IifxMachine::execute68kTrap(u16 trap) {
    // ApplScratch is identity-mapped after the IIfx System enables its PMMU.
    // Preserve it: Finder is allowed to retain application state here across
    // Toolbox calls, so a host-injected trap must leave no footprint.
    constexpr u32 scratch = 0x0A78u;
    const u8 saved0 = read8(scratch);
    const u8 saved1 = read8(scratch + 1u);
    write16(scratch, trap);
    const u32 savedPc = cpu_.pc;
    const u16 savedSr = cpu_.getSR();
    cpu_.pc = scratch;

    int guard = 64000000;
    while (guard-- > 0 && cpu_.pc != scratch + 2u && !cpu_.halted &&
           !poweredOff()) {
        stepInstruction();
    }
    const bool completed = cpu_.pc == scratch + 2u;
    if (!completed) {
        ++injectedTrapTimeouts_;
        if (onDiag) {
            char message[96];
            std::snprintf(message, sizeof message,
                          "IIfx injected trap %04X did not complete pc=%08X",
                          trap, cpu_.pc);
            onDiag(message);
        }
    }
    cpu_.pc = savedPc;
    cpu_.setSR(savedSr);
    write8(scratch, saved0);
    write8(scratch + 1u, saved1);
    return completed;
}

void IifxMachine::announceHardDisk() {
    if (!hardDiskMountPending_ || hardDisk_.empty() ||
        hardDiskMountInFlight_ || inDiskDriver_) return;

    const u32 vcbHead = read32(0x0358u);
    if (vcbHead == 0 || vcbHead == 0xFFFFFFFFu) return;  // System not up
    if (volumeMountedFor(4)) {
        hardDiskMountPending_ = false;
        hardDiskMounted_ = true;
        return;
    }
    if ((read8(0x0360u) & 1u) != 0) return;             // File Manager busy
    if (read32(0x0360u) == 0xFFFFFFFFu) return;         // queue not built
    if ((cpu_.getSR() & 0x0700u) != 0) return;          // mid-interrupt

    // Give startup one full second to mount the fixed disk itself. If it was
    // present before the Event Manager came online, the insertion announce is
    // otherwise lost; a deferred _MountVol is the same recovery used by the
    // established Classic and Quadra backends.
    if (++hardDiskMountDelay_ < 2) return;

    bool drivePresent = false;
    u32 entry = read32(0x030Au);
    for (int count = 0; count < 16 && entry && entry != 0xFFFFFFFFu; ++count) {
        entry = guestPtr(entry);
        if (entry + 12u > ram_.size()) break;
        if (read16(entry + 6u) == 4) {
            drivePresent = true;
            break;
        }
        entry = read32(entry);
    }
    if (!drivePresent) return;

    hardDiskMountInFlight_ = true;
    u32 savedD[8], savedA[8];
    for (int i = 0; i < 8; ++i) {
        savedD[i] = cpu_.d[i];
        savedA[i] = cpu_.a[i];
    }

    if (!hardDiskMountPb_) {
        cpu_.d[0] = 80;
        if (execute68kTrap(0xA71Eu))                   // _NewPtr,Sys,Clear
            hardDiskMountPb_ = guestPtr(cpu_.a[0]);
    }

    s16 result = -108;                                 // memFullErr fallback
    if (hardDiskMountPb_ && hardDiskMountPb_ + 80u <= ram_.size()) {
        for (u32 i = 0; i < 80; ++i) write8(hardDiskMountPb_ + i, 0);
        write16(hardDiskMountPb_ + 22u, 4);             // ioVRefNum = drive 4
        cpu_.a[0] = hardDiskMountPb_;
        if (execute68kTrap(0xA00Fu))                    // _MountVol
            result = static_cast<s16>(cpu_.d[0] & 0xFFFFu);
    }
    hardDiskMountResult_ = result;
    ++hardDiskMountTries_;
    if (result == 0 || result == -55) {                 // mounted/already online
        hardDiskMountPending_ = false;
        hardDiskMounted_ = true;
    } else if (hardDiskMountTries_ >= 15) {
        hardDiskMountPending_ = false;
    }
    if (onDiag) {
        char message[112];
        std::snprintf(message, sizeof message,
                      "IIfx hard disk drive 4 mount result=%d try=%u%s",
                      result, hardDiskMountTries_, hardDiskMounted_ ? " mounted" : "");
        onDiag(message);
    }

    for (int i = 0; i < 8; ++i) {
        cpu_.d[i] = savedD[i];
        cpu_.a[i] = savedA[i];
    }
    hardDiskMountInFlight_ = false;
}

void IifxMachine::serveSonyPrime() {
    const u32 pb = guestPtr(cpu_.a[0]);
    const u32 dce = guestPtr(cpu_.a[1]);
    const u16 trap = read16(pb + 0x06);
    const u32 buffer = guestPtr(read32(pb + 0x20));
    const u32 requested = read32(pb + 0x24);
    const u16 posMode = read16(pb + 0x2C);
    const u32 posOffset = read32(pb + 0x2E);
    const u32 mark = read32(dce + 0x10);

    u32 position = mark;
    switch (posMode & 3) {
    case 1: position = posOffset; break;                         // fsFromStart
    case 2: position = static_cast<u32>(floppy_.size()) + posOffset; break;
    case 3: position = mark + posOffset; break;                  // fsFromMark
    default: break;                                              // fsAtMark
    }

    s16 result = 0;
    u32 actual = 0;
    const bool write = (trap & 1) != 0;
    if (sonyPrimeDiagBudget_ > 0 && onDiag) {
        --sonyPrimeDiagBudget_;
        char message[112];
        std::snprintf(message, sizeof message,
                      "IIfx .Sony Prime %s pos=%u req=%u buf=%08X",
                      write ? "write" : "read", position, requested, buffer);
        onDiag(message);
    }
    if (floppy_.empty()) {
        result = -65;                                           // offLinErr
    } else if (write && floppyReadOnly_) {
        result = -44;                                           // wPrErr
    } else if (position >= floppy_.size()) {
        result = -39;                                           // eofErr
    } else {
        u32 count = requested;
        if (position + count > floppy_.size()) {
            count = static_cast<u32>(floppy_.size()) - position;
            result = -39;
        }
        if (write) {
            for (u32 i = 0; i < count; ++i)
                floppy_[position + i] = read8(buffer + i);
        } else {
            for (u32 i = 0; i < count; ++i)
                write8(buffer + i, floppy_[position + i]);
            cpu_.invalidateDataCache030(buffer, count);
        }
        actual = count;
    }
    write32(pb + 0x28, actual);                                 // ioActCount
    write32(dce + 0x10, position + actual);                     // dCtlPosition
    write16(pb + 0x10, static_cast<u16>(result));               // ioResult
    cpu_.invalidateDataCache030(pb, 0x40u);
    cpu_.invalidateDataCache030(dce, 0x20u);
    cpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const u32 done = read32(0x08FC);                             // jIODone
    cpu_.a[0] = done;
    cpu_.pc = done;
}

void IifxMachine::serveSonyCtlStatus(bool status) {
    const u32 pb = guestPtr(cpu_.a[0]);
    const u16 code = read16(pb + 0x1A);                          // csCode
    if (onDiag) {
        char message[128];
        std::snprintf(message, sizeof message,
                      "IOP .Sony bridge %s code=%u pb=%08X present=%d bytes=%zu",
                      status ? "status" : "control", code, pb,
                      floppy_.empty() ? 0 : 1, floppy_.size());
        onDiag(message);
    }
    s16 result = 0;
    if (status) {
        if (code == 6) {                                        // format list
            write16(pb + 0x1C, 1);
            write32(pb + 0x1E, static_cast<u32>(floppy_.size() / 512));
        } else if (code == 8) {                                 // DriveStatus
            for (u32 i = 0; i < 30; ++i) write8(pb + 0x1C + i, 0);
            write8(pb + 0x1E, floppyReadOnly_ ? 0x80 : 0);      // dsWriteProt
            write8(pb + 0x1F, floppy_.empty() ? 0 : 8);         // dsDiskInPlace
            write8(pb + 0x20, 1);                               // dsInstalled
            write8(pb + 0x21, floppy_.size() == 409600 ? 0 : 1);// dsSides
            write8(pb + 0x32, 0xFF);                            // dsMFMDrive
            write8(pb + 0x33, floppy_.size() == 1474560 ? 0xFF : 0);
            write8(pb + 0x34, 0xFF);                            // dsTwoMegFmt
        }
    } else {
        if (code == 1 || code == 21 || code == 22) result = -17; // controlErr
        if (code == 7) ejectFloppy();                            // Eject
    }

    write16(pb + 0x10, static_cast<u16>(result));
    cpu_.invalidateDataCache030(pb, 0x40u);
    cpu_.d[0] = static_cast<u32>(static_cast<s32>(result));
    const bool noQueue = (read16(pb + 0x06) & 0x0200u) != 0;
    const u32 done = read32(0x08FC);
    if (!noQueue && done) {
        cpu_.a[0] = done;
        cpu_.pc = done;
    } else {
        const u32 sp = cpu_.a[7];
        cpu_.pc = read32(sp);
        cpu_.a[7] = sp + 4;
    }
}

void IifxMachine::drainAudio(std::vector<u8>& out) {
    out = std::move(audioOut_);
    audioOut_.clear();
}

u32 IifxMachine::audioSampleRate() const { return sound_->sampleRate(); }

void IifxMachine::mouseMove(int dx, int dy, bool button) {
    adb_->injectMouse(dx, dy, button);
}

void IifxMachine::keyEvent(u8 adbCode, bool down) {
    adb_->injectKey(adbCode, down);
}

u32 IifxMachine::savePram(u8* out, u32 cap) const {
    if (!out || cap < kPramBlobBytes) return kPramBlobBytes;
    const auto& p = rtc_->xpram();
    std::memcpy(out, "PRAM", 4);
    out[4] = 1;
    out[5] = out[6] = out[7] = 0;
    std::copy(p.begin(), p.end(), out + 8);
    const u32 seconds = rtc_->seconds();
    out[264] = static_cast<u8>(seconds >> 24);
    out[265] = static_cast<u8>(seconds >> 16);
    out[266] = static_cast<u8>(seconds >> 8);
    out[267] = static_cast<u8>(seconds);
    return kPramBlobBytes;
}

bool IifxMachine::loadPram(const u8* data, u32 len, u32 addSeconds) {
    if (!data || len < kPramBlobBytes) return false;
    if (std::memcmp(data, "PRAM", 4) != 0 || data[4] != 1) return false;
    auto& p = rtc_->xpram();
    std::copy(data + 8, data + 8 + 256, p.begin());
    const u32 seconds = (u32(data[264]) << 24) | (u32(data[265]) << 16) |
                        (u32(data[266]) << 8) | u32(data[267]);
    rtc_->setSeconds(seconds + addSeconds);
    return true;
}

void IifxMachine::logAccess(const char* device, bool write, u32 addr, u32 value,
                            u8 width) {
    // Retain a rolling window. Bring-up often contains tens of thousands of
    // identical timer/audio polls before the one access that explains a stall;
    // a stop-when-full log preserves precisely the wrong end of that story.
    if (legacyAccessLogEnabled_) {
        if (accessLog_.size() >= 4096)
            accessLog_.erase(accessLog_.begin(), accessLog_.begin() + 2048);
        char line[112];
        std::snprintf(line, sizeof line, "%10llu %-8s %c %08X = %08X pc=%08X",
                      static_cast<unsigned long long>(totalCycles_), device,
                      write ? 'W' : 'R', addr, value, cpu_.pc);
        accessLog_.emplace_back(line);
    }

    const HardwareTraceSource source = traceSourceFor(device);
    u32 categories = TraceBus | TraceIo;
    if (source == HardwareTraceSource::Ncr5380 ||
        source == HardwareTraceSource::ScsiDma)
        categories |= TraceScsi;
    if (source == HardwareTraceSource::NuBus) categories |= TraceVideo;
    // Host-window reads are ordinary bus traffic. Mailbox state changes and
    // IOP DMA/firmware transitions have their own typed TraceIop events; not
    // tagging every SRAM polling read keeps a protocol-only ring compact.
    if (!hardwareTrace_.acceptsAccess(categories, source, addr)) return;
    HardwareTraceEvent event = makeTraceEvent(
        HardwareTraceKind::Access, source, categories);
    event.address = addr;
    event.value = value;
    event.width = width;
    if (write) event.flags |= 1u;
    const std::size_t count = std::min<std::size_t>(
        std::strlen(device), event.detail.size() - 1u);
    std::copy_n(device, count, event.detail.data());
    event.detail[count] = '\0';
    hardwareTrace_.record(std::move(event));
}

HardwareTraceEvent IifxMachine::makeTraceEvent(
    HardwareTraceKind kind, HardwareTraceSource source, u32 categories) const {
    HardwareTraceEvent event;
    event.cycle = totalCycles_;
    event.pc = cpu_.instructionAddress();
    event.correlation = activeMediaEventId_;
    event.kind = kind;
    event.source = source;
    event.categories = categories;

    event.scsi.phase = static_cast<u8>(scsi_->phase());
    if (scsi_->requestAsserted()) event.scsi.flags |= 1u << 0;
    if (scsi_->acknowledgeAsserted()) event.scsi.flags |= 1u << 1;
    if (scsi_->irqAsserted()) event.scsi.flags |= 1u << 2;
    if (scsiDma_->drqAsserted()) event.scsi.flags |= 1u << 3;
    if (scsi_->phaseMatches()) event.scsi.flags |= 1u << 4;
    if (scsiDma_->dmaActive()) event.scsi.flags |= 1u << 5;
    event.scsi.transferPosition = scsi_->xferPos();
    event.scsi.transferLength = scsi_->xferLen();
    event.scsi.dmaControl = scsiDma_->control();
    event.scsi.dmaAddress = scsiDma_->address();
    event.scsi.dmaCount = scsiDma_->count();
    event.scsi.commandCount = scsi_->diagCommands;
    event.scsi.cdbLength = static_cast<u8>(
        std::clamp(scsi_->diagLastCdbLen, 0, 12));
    std::copy_n(scsi_->diagLastCdb, event.scsi.cdb.size(),
                event.scsi.cdb.begin());

    event.swim.mode = swim_->ismMode();
    event.swim.setup = swim_->ismSetup();
    event.swim.handshake = swim_->diagnosticHandshake();
    event.swim.error = swim_->diagnosticError();
    event.swim.data = swim_->ismData;
    event.swim.lines = swim_->diagnosticLines();
    const bool driveSel = (swim_->ismMode() & 0x20u) != 0;
    event.swim.driveAddress = static_cast<u8>(swim_->driveRegister(driveSel));
    if (swim_->ismAction()) event.swim.flags |= 1u << 0;
    if (swim_->ismWriting()) event.swim.flags |= 1u << 1;
    if (swim_->ismDataReady) event.swim.flags |= 1u << 2;
    if (swim_->ismMarkNext) event.swim.flags |= 1u << 3;
    if (swim_->ismCrcOk) event.swim.flags |= 1u << 4;
    if (floppyDrive_->hasDisk()) event.swim.flags |= 1u << 5;
    if (floppyDrive_->diskSwitched()) event.swim.flags |= 1u << 6;
    if (floppyDrive_->motorRunning(totalCycles_)) event.swim.flags |= 1u << 7;
    event.swim.crc = swim_->diagnosticCrc();
    event.swim.track = static_cast<u8>(std::clamp(floppyDrive_->track, 0, 255));
    event.swim.side = floppyDrive_->headUpper ? 1 : 0;
    event.swim.fifoOccupancy = swim_->ismFifoOccupancy();
    event.swim.fifoHead = swim_->ismFifoHeadData();
    event.swim.fifoTail = swim_->ismFifoTailData();
    event.swim.readSynced = swimReadSynced_ ? 1u : 0u;
    event.swim.fifoHeadCrc = swim_->diagnosticFifoHeadCrc();
    if (floppyDrive_->stepIn) event.swim.mechanismFlags |= 1u << 0;
    if (floppyDrive_->stepping(totalCycles_))
        event.swim.mechanismFlags |= 1u << 1;
    if (floppyDrive_->motorCommanded())
        event.swim.mechanismFlags |= 1u << 2;
    if (floppyDrive_->enabled()) event.swim.mechanismFlags |= 1u << 3;
    if (floppyDrive_->mfmMode()) event.swim.mechanismFlags |= 1u << 4;
    event.swim.stepRemaining = floppyDrive_->stepRemaining(totalCycles_);
    event.swim.requestedBlock = traceFloppyBlock_;
    event.swim.expectedTrack = traceFloppyTrack_;
    event.swim.expectedSide = traceFloppySide_;
    event.swim.expectedSector = traceFloppySector_;

    // Diagnostic request selectors are intentionally not part of the machine
    // checkpoint. Recover them from the architected channel-1 send mailbox so
    // a trace started after loading a checkpoint remains self-describing.
    // The firmware leaves the active/completed request in this slot until the
    // next request arrives.
    if (ismIop_) {
        constexpr u16 message = static_cast<u16>(
            IifxIop::SendMsg + IifxIop::MessageSize);
        const u8 operation = ismIop_->ram(message);
        if (operation >= 0x0Au && operation <= 0x0Cu) {
            const u32 block =
                (static_cast<u32>(ismIop_->ram(message + 8u)) << 24) |
                (static_cast<u32>(ismIop_->ram(message + 9u)) << 16) |
                (static_cast<u32>(ismIop_->ram(message + 10u)) << 8) |
                ismIop_->ram(message + 11u);
            const u32 cylinder = mfm::kSides * mfm::kSectorsPerTrack;
            const u32 withinTrack = block % cylinder;
            event.swim.requestedBlock = block;
            event.swim.expectedTrack = static_cast<u8>(block / cylinder);
            event.swim.expectedSide = static_cast<u8>(
                withinTrack / mfm::kSectorsPerTrack);
            event.swim.expectedSector = static_cast<u8>(
                withinTrack % mfm::kSectorsPerTrack + 1u);
        }
    }

    const IifxIop& iop = source == HardwareTraceSource::SccIop
        ? *sccIop_ : *ismIop_;
    event.iop.pc = iop.cpu().pc;
    event.iop.opcode = iop.cpu().lastOpcode;
    event.iop.a = iop.cpu().a;
    event.iop.x = iop.cpu().x;
    event.iop.y = iop.cpu().y;
    event.iop.s = iop.cpu().s;
    event.iop.p = iop.cpu().p;
    event.iop.status = iop.status();
    event.iop.interruptMask = iop.interruptMask();
    event.iop.interruptFlags = iop.interruptFlags();
    for (int channel = 0; channel < 2; ++channel) {
        event.iop.dmaControl[static_cast<std::size_t>(channel)] =
            iop.dmaControl(channel);
        event.iop.dmaRequest[static_cast<std::size_t>(channel)] =
            iop.dmaRequest(channel) ? 1 : 0;
        event.iop.dmaMap[static_cast<std::size_t>(channel)] =
            iop.dmaMap(channel);
        event.iop.dmaCount[static_cast<std::size_t>(channel)] =
            iop.dmaCount(channel);
    }
    event.iop.dmaTransfers = iop.dmaTransfers;
    event.iop.instructions = iop.cpu().instructions;
    if (&iop == ismIop_.get()) {
        event.iop.pollEnable = iop.ram(0x4ED7u);
        event.iop.currentDrive = iop.ram(0x74u);
        const u8 drive = event.iop.currentDrive < 9u
            ? event.iop.currentDrive : 0u;
        event.iop.driveKind = iop.ram(static_cast<u16>(0x4EF4u + drive));
        event.iop.driveState = iop.ram(static_cast<u16>(0x4EFDu + drive));
        event.iop.driveFormat = iop.ram(static_cast<u16>(0x4F06u + drive));
        event.iop.receiveState = iop.ram(0x0302u);
        event.iop.continuation = static_cast<u16>(
            (static_cast<u16>(iop.ram(0x4EDAu)) << 8) |
            iop.ram(0x4ED9u));
    }

    event.asc.mode = sound_->mode();
    event.asc.control = sound_->control();
    event.asc.volume = sound_->volume();
    event.asc.clock = sound_->clock();
    event.asc.status = sound_->fifoStatus();
    if (sound_->stereo()) event.asc.flags |= 1u << 0;
    if (sound_->irqAsserted()) event.asc.flags |= 1u << 1;
    event.asc.fifoLevelA = sound_->fifoLevel(0);
    event.asc.fifoLevelB = sound_->fifoLevel(1);
    for (int voice = 0; voice < 4; ++voice) {
        event.asc.phase[static_cast<std::size_t>(voice)] =
            sound_->phase(voice);
        event.asc.increment[static_cast<std::size_t>(voice)] =
            sound_->increment(voice);
    }
    event.asc.ramWrites = sound_->ramWrites();
    event.asc.producedSamples = sound_->producedSamples();
    event.asc.nonSilentSamples = sound_->nonSilentSamples();
    event.asc.irqTransitions = sound_->irqTransitions();

    event.video.mode = video_->mode();
    event.video.control = video_->control();
    event.video.base = video_->baseRegister();
    event.video.stride = video_->strideRegister();
    event.video.vramWrites = video_->vramWrites;
    event.video.frameVramWrites = video_->lastFrameVramWrites;
    event.video.vblankCount = video_->vblankCount;
    event.video.vblankAssertions = video_->vblankAssertions;
    event.video.vblankAcks = video_->vblankAcks;
    if (video_->genuineGc()) event.video.flags |= 1u << 0;
    if (video_->vblankEnabled()) event.video.flags |= 1u << 1;
    if (video_->irqAsserted()) event.video.flags |= 1u << 2;
    if ((oss_->pending() & (1u << Oss::Slot9)) != 0)
        event.video.flags |= 1u << 3;
    event.video.flags |= static_cast<u8>((oss_->priority(Oss::Slot9) & 7u) << 4);
    event.video.bitsPerPixel = static_cast<u8>(video_->bitsPerPixel());
    event.video.ossIpl = static_cast<u8>(oss_->ipl());
    event.video.ramdacMode = video_->ramdacMode();
    event.video.serialCommand = video_->lastSerialCommand();
    event.video.serialCommands = video_->serialCommands();
    event.video.timingEchoWrites = video_->timingEchoWrites();
    return event;
}

void IifxMachine::traceAccess(HardwareTraceSource source, bool write,
                              u32 address, u64 value, u8 width,
                              const char* detail) {
    if (!hardwareTrace_.enabled() || hardwareTrace_.frozen()) return;
    u32 categories = TraceBus;
    if (source == HardwareTraceSource::Ncr5380 ||
        source == HardwareTraceSource::ScsiDma)
        categories |= TraceScsi;
    if (source == HardwareTraceSource::Swim) categories |= TraceSwim | TraceIop;
    if (source == HardwareTraceSource::NuBus) categories |= TraceVideo | TraceIo;
    HardwareTraceEvent event = makeTraceEvent(
        HardwareTraceKind::Access, source, categories);
    event.address = address;
    event.value = value;
    event.width = width;
    if (write) event.flags |= 1u;
    if (detail) {
        const std::size_t count = std::min<std::size_t>(
            std::strlen(detail), event.detail.size() - 1u);
        std::copy_n(detail, count, event.detail.data());
        event.detail[count] = '\0';
    }
    hardwareTrace_.record(std::move(event));
}

void IifxMachine::traceState(HardwareTraceKind kind,
                             HardwareTraceSource source, u32 categories,
                             const char* detail) {
    if (!hardwareTrace_.enabled() || hardwareTrace_.frozen()) return;
    HardwareTraceEvent event = makeTraceEvent(kind, source, categories);
    if (detail) {
        const std::size_t count = std::min<std::size_t>(
            std::strlen(detail), event.detail.size() - 1u);
        std::copy_n(detail, count, event.detail.data());
        event.detail[count] = '\0';
    }
    hardwareTrace_.record(std::move(event));
}

void IifxMachine::checkTraceStall(u32 pc) {
    if (!hardwareTrace_.enabled() || hardwareTrace_.triggered()) return;
    if (pc == traceLastPc_) ++traceSamePcCount_;
    else {
        traceLastPc_ = pc;
        traceSamePcCount_ = 1;
    }
    // A tight single-PC loop this long is never normal driver progress. Keep
    // the threshold high enough that intentional hardware polling receives
    // hundreds of bus snapshots before the ring freezes.
    if (traceSamePcCount_ == 200000u)
        triggerTrace("repeated-PC stall");
}

void IifxMachine::checkScsiProtocol(u8 reg, u8 value) {
    if (!hardwareTrace_.enabled() || hardwareTrace_.frozen()) return;
    // MR.Arbitrate may rise only on a free bus. Starting a second arbitration
    // while the target still owns Status/Message-In discards the command's
    // completion handshake and is an architecturally impossible initiator
    // transition unless RST is being asserted concurrently.
    if (reg == 2 && (value & 1u) != 0 &&
        (scsi_->modeRegister() & 1u) == 0 &&
        scsi_->phase() != Ncr5380::BusFree &&
        (scsi_->initiatorCommand() & 0x80u) == 0) {
        protocolAssertion(1ull << 0, HardwareTraceSource::Ncr5380,
            TraceScsi,
            "SCSI arbitration attempted before Status/Message-In completed",
            reg, value);
    }

    if (reg == 1 && (value & 0x10u) != 0 &&
        (scsi_->initiatorCommand() & 0x10u) == 0 &&
        !scsi_->requestAsserted() && (value & 0x80u) == 0) {
        protocolAssertion(1ull << 1, HardwareTraceSource::Ncr5380,
            TraceScsi, "SCSI ACK asserted without target REQ", reg, value);
    }

    if (reg == 1 && (value & 0x04u) != 0 &&
        (scsi_->initiatorCommand() & 0x04u) == 0 &&
        scsi_->phase() != Ncr5380::BusFree &&
        scsi_->phase() != Ncr5380::Arbitration &&
        scsi_->phase() != Ncr5380::Selection && (value & 0x80u) == 0) {
        protocolAssertion(1ull << 2, HardwareTraceSource::Ncr5380,
            TraceScsi, "SCSI selection started before the active nexus completed",
            reg, value);
    }
}

void IifxMachine::protocolAssertion(u64 identity,
                                    HardwareTraceSource source,
                                    u32 categories, const char* reason,
                                    u32 address, u64 value) {
    if (!hardwareTrace_.enabled() || hardwareTrace_.frozen() ||
        (traceAssertionMask_ & identity) != 0)
        return;
    traceAssertionMask_ |= identity;
    HardwareTraceEvent event = makeTraceEvent(
        HardwareTraceKind::Assertion, source, categories | TraceAssertion);
    event.address = address;
    event.value = value;
    hardwareTrace_.assertion(std::move(event), reason);
    if (traceCheckpointOnTrigger_) {
        traceCheckpointPending_ = true;
        traceCheckpointReason_ = reason;
    }
}

void IifxMachine::checkHardwareProtocols() {
    const int phase = scsi_->phase();
    if (traceProtocolScsiPhase_ >= 0 && phase != traceProtocolScsiPhase_) {
        const auto legal = [](int from, int to) {
            if (to == Ncr5380::BusFree) return true; // reset, abort, or Msg-In ACK
            switch (from) {
            case Ncr5380::BusFree:
                return to == Ncr5380::Arbitration ||
                       to == Ncr5380::Selection || to == Ncr5380::Command;
            case Ncr5380::Arbitration:
            case Ncr5380::Selection:
                return to == Ncr5380::Command;
            case Ncr5380::Command:
                return to == Ncr5380::DataOut || to == Ncr5380::DataIn ||
                       to == Ncr5380::Status;
            case Ncr5380::DataOut:
            case Ncr5380::DataIn:
                return to == Ncr5380::Status;
            case Ncr5380::Status:
                return to == Ncr5380::MsgIn;
            case Ncr5380::MsgIn:
                return false;
            default:
                return false;
            }
        };
        if (!legal(traceProtocolScsiPhase_, phase)) {
            const u64 transition =
                (static_cast<u64>(static_cast<u8>(traceProtocolScsiPhase_)) << 8) |
                static_cast<u8>(phase);
            protocolAssertion(1ull << 3, HardwareTraceSource::Ncr5380,
                TraceScsi, "illegal SCSI bus phase transition", 0, transition);
        }
    }
    traceProtocolScsiPhase_ = phase;

    if (scsi_->xferPos() > scsi_->xferLen())
        protocolAssertion(1ull << 4, HardwareTraceSource::Ncr5380,
            TraceScsi, "SCSI transfer position exceeded transfer length",
            scsi_->xferPos(), scsi_->xferLen());
    if (scsi_->cdbPos() > scsi_->cdbLen() || scsi_->cdbLen() > 12)
        protocolAssertion(1ull << 5, HardwareTraceSource::Ncr5380,
            TraceScsi, "SCSI CDB position/length is out of range",
            static_cast<u32>(scsi_->cdbPos()),
            static_cast<u64>(static_cast<u32>(scsi_->cdbLen())));
    if (scsi_->requestAsserted() && scsi_->acknowledgeAsserted())
        protocolAssertion(1ull << 6, HardwareTraceSource::Ncr5380,
            TraceScsi, "SCSI target REQ remained asserted during initiator ACK");
    if (phase == Ncr5380::BusFree && scsi_->requestAsserted())
        protocolAssertion(1ull << 7, HardwareTraceSource::Ncr5380,
            TraceScsi, "SCSI target REQ asserted while the bus is free");

    if (scsiDma_->dmaActive() && scsiDma_->count() == 0)
        protocolAssertion(1ull << 8, HardwareTraceSource::ScsiDma,
            TraceScsi, "IIfx SCSI DMA active with a zero byte count");
    if (scsiDma_->dmaActive() && (scsiDma_->control() & 1u) == 0)
        protocolAssertion(1ull << 9, HardwareTraceSource::ScsiDma,
            TraceScsi, "IIfx SCSI DMA active while DMA enable is clear");
    if (scsiDma_->dmaActive() && (scsi_->modeRegister() & 0x02u) == 0)
        protocolAssertion(1ull << 10, HardwareTraceSource::ScsiDma,
            TraceScsi, "IIfx SCSI DMA active while NCR5380 DMA mode is clear");

    const u8 swimMode = swim_->ismMode();
    const bool swimAction = swim_->ismAction();
    const unsigned selectedDrives = ((swimMode & 0x02u) ? 1u : 0u) +
                                    ((swimMode & 0x04u) ? 1u : 0u);
    if (swimAction && selectedDrives == 0)
        protocolAssertion(1ull << 11, HardwareTraceSource::Swim,
            TraceSwim | TraceIop, "SWIM ACTION asserted with no selected drive");
    if (swimAction && selectedDrives > 1)
        protocolAssertion(1ull << 12, HardwareTraceSource::Swim,
            TraceSwim | TraceIop, "SWIM ACTION asserted with both drives selected");
    if (!swimAction && (ismIop_->dmaRequest(0) || ismIop_->dmaRequest(1)))
        protocolAssertion(1ull << 13, HardwareTraceSource::Swim,
            TraceSwim | TraceIop, "SWIM DMA request asserted while ACTION is clear");
    if ((floppyDrive_->track < 0 || floppyDrive_->track > 79) ||
        (floppyDrive_->headUpper && !floppyDrive_->doubleSided))
        protocolAssertion(1ull << 14, HardwareTraceSource::Swim,
            TraceSwim, "SuperDrive head is outside the installed mechanism geometry",
            static_cast<u32>(floppyDrive_->track),
            floppyDrive_->headUpper ? 1u : 0u);

    // Video interrupt/cursor assertions are deliberately frame-bounded. A
    // normal slot handler acknowledges every VBL quickly; holding the level
    // for four complete display periods means either the handler was never
    // installed or it is polling the wrong card register.
    if (video_->vblankCount != traceLastVideoVblank_) {
        traceLastVideoVblank_ = video_->vblankCount;
        if (videoEnabled_ && video_->irqAsserted()) ++traceVideoIrqFrames_;
        else traceVideoIrqFrames_ = 0;
        if (video_->lastFrameVramWrites != 0 || traceVideoFrameSamples_ < 16) {
            traceState(HardwareTraceKind::State, HardwareTraceSource::NuBus,
                       TraceVideo,
                       video_->lastFrameVramWrites != 0
                           ? "video frame write summary"
                           : "video frame idle summary");
            ++traceVideoFrameSamples_;
        }
    }
    const u8 videoPriority = oss_->priority(Oss::Slot9);
    const u8 cpuMask = static_cast<u8>((cpu_.getSR() >> 8) & 7u);
    const bool videoIrqEligible = videoEnabled_ && video_->irqAsserted() &&
                                  videoPriority != 0 && cpuMask < videoPriority;
    if (videoIrqEligible && traceVideoIrqEligibleSince_ == 0)
        traceVideoIrqEligibleSince_ = totalCycles_;
    else if (!video_->irqAsserted() || videoPriority == 0)
        traceVideoIrqEligibleSince_ = 0;
    // Once it has been unmasked, a level-2 slot interrupt is sampled at the
    // following 68030 instruction boundary. Keep the deadline latched while
    // its handler raises the SR mask: a handler that enters but never clears
    // the card is just as broken as a disconnected OSS path.
    if (traceVideoIrqEligibleSince_ != 0 &&
        totalCycles_ - traceVideoIrqEligibleSince_ > 600038u)
        protocolAssertion(1ull << 25, HardwareTraceSource::NuBus,
            TraceVideo | TraceIo,
            "eligible video VBL slot interrupt remained unacknowledged for one frame",
            IifxNuBusVideo::kSlotBase, video_->vblankAcks);

    if (videoPriority != 0 && traceLastVideoPriorityFrame_ == 0) {
        traceLastVideoPriorityFrame_ = frameCounter_;
        traceState(HardwareTraceKind::State, HardwareTraceSource::Oss,
                   TraceVideo | TraceIo, "OSS enabled NuBus slot 9");
    }

    if (video_->serialCommands() != traceLastVideoSerialCommands_) {
        const u64 commands = video_->serialCommands();
        if (commands <= 32u || (commands & (commands - 1u)) == 0)
            traceState(HardwareTraceKind::State, HardwareTraceSource::NuBus,
                       TraceVideo | TraceIo, "8*24 GC serial command");
        traceLastVideoSerialCommands_ = video_->serialCommands();
    }

    if (video_->vblankAssertions != traceLastVideoAssertions_ ||
        video_->vblankAcks != traceLastVideoAcks_) {
        traceState(HardwareTraceKind::State, HardwareTraceSource::NuBus,
                   TraceVideo | TraceIo,
                   video_->irqAsserted() ? "video VBL asserted"
                                         : "video VBL acknowledged");
        traceLastVideoAssertions_ = video_->vblankAssertions;
        traceLastVideoAcks_ = video_->vblankAcks;
    }

    // ADB reports moving the system cursor through the Event/VBL managers.
    // Remember the video write count when that becomes visible in low memory;
    // if no card memory changes in the next four frames, capture the exact
    // interrupt/register lead-in instead of producing a minutes-long log.
    const auto traceRam16 = [this](u32 address) {
        return address + 1u < ram_.size()
            ? static_cast<u16>((static_cast<u16>(ram_[address]) << 8) |
                               ram_[address + 1u])
            : u16{0};
    };
    const u16 mouseV = traceRam16(0x0828u);
    const u16 mouseH = traceRam16(0x082Au);
    const u32 mouseReports = adb_->mouseReports();
    if ((mouseV != traceLastMouseV_ || mouseH != traceLastMouseH_) &&
        mouseReports != traceLastMouseReports_) {
        traceMouseMoveFrame_ = frameCounter_;
        traceMouseVramWrites_ = video_->vramWrites;
        traceState(HardwareTraceKind::State, HardwareTraceSource::NuBus,
                   TraceVideo | TraceIo, "guest logical cursor moved");
    }
    traceLastMouseV_ = mouseV;
    traceLastMouseH_ = mouseH;
    traceLastMouseReports_ = mouseReports;
    if (traceMouseMoveFrame_ != 0 &&
        frameCounter_ >= traceMouseMoveFrame_ + 4u) {
        if (video_->vramWrites == traceMouseVramWrites_)
            protocolAssertion(1ull << 26, HardwareTraceSource::NuBus,
                TraceVideo | TraceIo,
                "logical cursor moved without a framebuffer update",
                (u32(mouseV) << 16) | mouseH, video_->vramWrites);
        traceMouseMoveFrame_ = 0;
    }

    for (int iopIndex = 0; iopIndex < 2; ++iopIndex) {
        IifxIop& iop = iopIndex ? *ismIop_ : *sccIop_;
        if (iop.firmwareAlive()) {
            for (int channel = 0; channel < IifxIop::ChannelCount; ++channel) {
                const u8 send = iop.ram(static_cast<u16>(
                    IifxIop::SendState + channel));
                const u8 receive = iop.ram(static_cast<u16>(
                    IifxIop::RecvState + channel));
                // The ROM's power-on IOP SRAM test leaves unused message
                // state bytes at $FF.  The IOP Manager ERS defines 0..3 for
                // configured messages, but $FF is therefore a legitimate
                // unconfigured sentinel and must not be diagnosed as a live
                // protocol transition.
                const auto invalidMailboxState = [](u8 state) {
                    return state != 0xFFu && state > IifxIop::MsgComplete;
                };
                if (invalidMailboxState(send) ||
                    invalidMailboxState(receive)) {
                    protocolAssertion(1ull << (15 + iopIndex),
                        iopIndex ? HardwareTraceSource::IsmIop
                                 : HardwareTraceSource::SccIop,
                        TraceIop,
                        "IOP mailbox state contains an undefined protocol value",
                        static_cast<u32>(channel),
                        (static_cast<u64>(send) << 8) | receive);
                    break;
                }
            }
        }
        for (int channel = 0; channel < 2; ++channel) {
            if ((iop.dmaControl(channel) & 1u) != 0 &&
                iop.dmaRequest(channel) && iop.dmaCount(channel) == 0) {
                protocolAssertion(1ull << (17 + iopIndex * 2 + channel),
                    iopIndex ? HardwareTraceSource::IsmIop
                             : HardwareTraceSource::SccIop,
                    TraceIop,
                    "IOP DMA channel enabled and requested with zero count",
                    static_cast<u32>(channel), iop.dmaControl(channel));
            }
        }
        if (iop.running() && iop.cpu().stopped)
            protocolAssertion(1ull << (21 + iopIndex),
                iopIndex ? HardwareTraceSource::IsmIop
                         : HardwareTraceSource::SccIop,
                TraceIop, "running IOP firmware executed the STP instruction");
    }
}

void IifxMachine::configureHardwareTrace(const HardwareTraceConfig& config) {
    hardwareTrace_.configure(config);
    traceLastPc_ = 0;
    traceSamePcCount_ = 0;
    traceLastScsiPhase_ = -1;
    traceLastScsiCommands_ = 0;
    traceLastSwimMode_ = traceLastSwimHandshake_ = 0xFF;
    traceLastSwimFifo_ = 0xFF;
    traceProtocolScsiPhase_ = -1;
    traceAssertionMask_ = 0;
    traceIopOperationHits_ = 0;
    traceLastVideoAssertions_ = video_->vblankAssertions;
    traceLastVideoAcks_ = video_->vblankAcks;
    traceLastVideoVblank_ = video_->vblankCount;
    traceVideoIrqFrames_ = 0;
    traceVideoIrqEligibleSince_ = 0;
    traceLastMouseReports_ = adb_->mouseReports();
    traceLastMouseV_ = ram_.size() > 0x0829u
        ? static_cast<u16>((static_cast<u16>(ram_[0x0828u]) << 8) |
                           ram_[0x0829u]) : 0;
    traceLastMouseH_ = ram_.size() > 0x082Bu
        ? static_cast<u16>((static_cast<u16>(ram_[0x082Au]) << 8) |
                           ram_[0x082Bu]) : 0;
    traceMouseMoveFrame_ = 0;
    traceMouseVramWrites_ = video_->vramWrites;
    traceVideoFrameSamples_ = 0;
    traceLastVideoSerialCommands_ = video_->serialCommands();
    traceLastVideoPriorityFrame_ = oss_->priority(Oss::Slot9) != 0
        ? frameCounter_ : 0;
}

void IifxMachine::triggerHardwareTrace(const std::string& reason) {
    triggerTrace(reason.c_str());
}

void IifxMachine::triggerTrace(const char* reason) {
    if (!hardwareTrace_.enabled() || hardwareTrace_.triggered()) return;
    hardwareTrace_.trigger(totalCycles_, cpu_.instructionAddress(), reason,
                           activeMediaEventId_);
    if (traceCheckpointOnTrigger_) {
        traceCheckpointPending_ = true;
        traceCheckpointReason_ = reason;
    }
}

bool IifxMachine::takeTraceCheckpoint(std::vector<u8>& state,
                                      std::string* reason) {
    if (!traceCheckpointPending_) return false;
    state = saveState();
    if (reason) *reason = traceCheckpointReason_;
    traceCheckpointPending_ = false;
    return true;
}

void IifxMachine::recordHardwareMilestone(const std::string& name) {
    if (!hardwareTrace_.enabled() || hardwareTrace_.frozen()) return;
    HardwareTraceEvent event = makeTraceEvent(
        HardwareTraceKind::Milestone, HardwareTraceSource::Other,
        TraceMilestone);
    event.pc = cpu_.pc;
    event.value = deterministicStateHash();
    event.auxiliary = framebufferHash();
    const std::size_t count = std::min(name.size(), event.detail.size() - 1u);
    std::copy_n(name.data(), count, event.detail.data());
    event.detail[count] = '\0';
    hardwareTrace_.record(std::move(event));
}

bool IifxMachine::writeHardwareTraceJsonl(const std::string& path) const {
    std::ofstream output(path, std::ios::binary);
    return output && hardwareTrace_.writeJsonl(output);
}

u64 IifxMachine::deterministicStateHash() const {
    u64 hash = 1469598103934665603ull;
    hash = hashWord(hash, totalCycles_);
    hash = hashWord(hash, frameCounter_);
    hash = hashWord(hash, cpu_.pc, 4);
    hash = hashWord(hash, cpu_.getSR(), 2);
    for (u32 value : cpu_.d) hash = hashWord(hash, value, 4);
    for (u32 value : cpu_.a) hash = hashWord(hash, value, 4);
    hash = hashWord(hash, scsi_->phase(), 1);
    hash = hashWord(hash, scsi_->diagCommands, 4);
    hash = hashWord(hash, scsiDma_->control(), 4);
    hash = hashWord(hash, scsiDma_->address(), 4);
    hash = hashWord(hash, scsiDma_->count(), 4);
    hash = hashWord(hash, swim_->ismMode(), 1);
    hash = hashWord(hash, swim_->diagnosticHandshake(), 1);
    hash = hashWord(hash, swim_->diagnosticCrc(), 2);
    hash = hashWord(hash, ismIop_->cpu().pc, 2);
    hash = hashWord(hash, ismIop_->dmaTransfers);
    return hash;
}

u64 IifxMachine::framebufferHash() const {
    const int width = screenWidth();
    const int height = screenHeight();
    std::vector<u32> pixels(static_cast<std::size_t>(width) * height);
    renderScreen(pixels.data());
    u64 hash = 1469598103934665603ull;
    hash = hashWord(hash, static_cast<u32>(width), 4);
    hash = hashWord(hash, static_cast<u32>(height), 4);
    for (u32 pixel : pixels) hash = hashWord(hash, pixel, 4);
    return hash;
}

void IifxMachine::synchronizeViaAccess() {
    // The 40 MHz processor cannot complete a motherboard-VIA transaction at
    // CPU-bus speed.  The access starts on the next 783.36 kHz VIA clock and
    // finishes half a VIA clock later.  Besides being board-visible timing,
    // this is how the IIfx ROM paces its wavetable decay: BOOTBEEP repeatedly
    // reads VIA register 14 at $50F01C00 between ASC RAM updates.
    const u64 cycle = totalCycles_ + static_cast<u64>(fmcCyclePenalty_);
    const u64 seconds = cycle / kCpuHz;
    const u64 remainder = cycle % kCpuHz;
    const u64 viaCycle = seconds * kViaHz +
        (remainder * kViaHz) / kCpuHz;

    // Express (viaCycle + 1.5) in CPU clocks without a potentially
    // overflowing multiply of the complete uptime by kCpuHz.
    const u64 halfViaCycle = viaCycle * 2u + 3u;
    const u64 halfViaHz = kViaHz * 2u;
    const u64 target = (halfViaCycle / halfViaHz) * kCpuHz +
        ((halfViaCycle % halfViaHz) * kCpuHz) / halfViaHz + 1u;
    if (target > cycle)
        fmcCyclePenalty_ += static_cast<int>(target - cycle);
}

u8 IifxMachine::ioRead8(u32 addr) {
    const u32 off = addr & kIoMirrorMask;
    const u32 block = off & 0x3E000u;
    u8 value = 0xFF;
    const char* name = "I/O";

    switch (block) {
    case 0x00000:
        name = "VIA1";
        synchronizeViaAccess();
        value = via_->read(static_cast<int>((off >> 9) & 15u));
        updateIpl();
        break;
    case 0x02000:
        // RTC has no parallel registers; this select is present for board
        // diagnostics while normal software reaches it through VIA PB0..2.
        name = "RTC";
        value = rtc_->dataOut() ? 0xFF : 0xFE;
        break;
    case 0x04000:
        name = "SCC-IOP";
        value = sccIop_->read(off & 0x3Fu);
        break;
    case 0x06000:
        name = "SCC";
        if (!sccIop_->bypassed()) throw BusFault{addr, true, 1};
        value = scc_->read(off & 6u);
        break;
    case 0x08000: {
        name = "SCSI-DMA";
        value = scsiDma_->read8(off & 0x1FFu);
        break;
    }
    case 0x0C000:
        name = "SCSI-DMA-ALT";
        value = scsiDma_->read8(off & 0x1FFu);
        break;
    case 0x0A000:
        throw BusFault{addr, true, 1};
    case 0x0E000:
        name = "NCR5380";
        value = scsi_->read(static_cast<int>((off >> 4) & 7u));
        break;
    case 0x10000:
        name = "ASC";
        value = sound_->read(off & 0x1FFFu);
        break;
    case 0x12000:
        name = "ISM-IOP";
        value = ismIop_->read(off & 0x3Fu);
        break;
    case 0x14000:
        throw BusFault{addr, true, 1};
    case 0x16000:
        // Direct ADB access is intentionally absent on the IIfx; the ISM IOP
        // owns it and third-party direct probes must fail.
        throw BusFault{addr, true, 1};
    case 0x18000: {
        name = "BIU30";
        const u32 word = biuRegs_[(off >> 2) & 15u];
        value = static_cast<u8>(word >> (8 * (3 - (off & 3u))));
        break;
    }
    case 0x1A000:
        name = "OSS";
        value = oss_->read(off & 0x1FFFu);
        break;
    case 0x1C000:
    case 0x1E000:
    case 0x20000: {
        // The board decodes these optional expansion selects and terminates
        // the cycle, but an unpopulated socket does not drive the data bus.
        // Reads consequently see pull-ups and writes disappear.  Do not back
        // these windows with writable storage: the ROM deliberately writes a
        // walking pattern and treats readback as installed hardware (notably
        // the special-order RAM Parity Unit).
        const u32 select = (block - 0x1C000u) >> 13;
        name = select == 0 ? "OSS-EXP0" : select == 1 ? "OSS-EXP1" : "OSS-EXP2";
        value = 0xFF;
        break;
    }
    default:
        // Expansion selects 3..5 and $28000-$3FFFF are outside the OSS DSACK
        // bracket in Apple's Figure 3-9. An absent device times out.
        throw BusFault{addr, true, 1};
    }
    logAccess(name, false, addr, value);
    if (onDiag && scsi_->diagCommands >= 8 &&
        scsi_->phase() != Ncr5380::DataIn &&
        (block == 0x08000u || block == 0x0C000u || block == 0x0E000u)) {
        char message[112];
        std::snprintf(message, sizeof message,
                      "SCSI1 R %08X=%02X phase=%d pos=%u/%u pc=%08X",
                      addr, value, scsi_->phase(), scsi_->xferPos(),
                      scsi_->xferLen(), cpu_.pc);
        onDiag(message);
    }
    return value;
}

void IifxMachine::ioWrite8(u32 addr, u8 value) {
    const u32 off = addr & kIoMirrorMask;
    const u32 block = off & 0x3E000u;
    const char* name = "I/O";

    switch (block) {
    case 0x00000:
        name = "VIA1";
        synchronizeViaAccess();
        via_->write(static_cast<int>((off >> 9) & 15u), value);
        updateIpl();
        break;
    case 0x02000:
        name = "RTC";
        break;
    case 0x04000:
        name = "SCC-IOP";
        sccIop_->write(off & 0x3Fu, value);
        break;
    case 0x06000:
        name = "SCC";
        if (!sccIop_->bypassed()) throw BusFault{addr, false, 1};
        scc_->write(off & 6u, value);
        break;
    case 0x08000: {
        name = "SCSI-DMA";
        scsiDma_->write8(off & 0x1FFu, value);
        break;
    }
    case 0x0C000:
        name = "SCSI-DMA-ALT";
        scsiDma_->write8(off & 0x1FFu, value);
        break;
    case 0x0A000:
        throw BusFault{addr, false, 1};
    case 0x0E000:
        name = "NCR5380";
        checkScsiProtocol(static_cast<u8>((off >> 4) & 7u), value);
        scsi_->write(static_cast<int>((off >> 4) & 7u), value);
        break;
    case 0x10000:
        name = "ASC";
        sound_->write(off & 0x1FFFu, value);
        break;
    case 0x12000:
        name = "ISM-IOP";
        ismIop_->write(off & 0x3Fu, value);
        break;
    case 0x14000:
    case 0x16000:
        throw BusFault{addr, false, 1};
    case 0x18000: {
        name = "BIU30";
        u32& word = biuRegs_[(off >> 2) & 15u];
        const int shift = 8 * (3 - static_cast<int>(off & 3u));
        word = (word & ~(0xFFu << shift)) | (static_cast<u32>(value) << shift);
        break;
    }
    case 0x1A000:
        name = "OSS";
        oss_->write(off & 0x1FFFu, value);
        break;
    case 0x1C000:
    case 0x1E000:
    case 0x20000: {
        const u32 select = (block - 0x1C000u) >> 13;
        name = select == 0 ? "OSS-EXP0" : select == 1 ? "OSS-EXP1" : "OSS-EXP2";
        // Unpopulated decoded socket: cycle terminates, no device latches it.
        break;
    }
    default:
        throw BusFault{addr, false, 1};
    }
    logAccess(name, true, addr, value);
    if (onDiag && scsi_->diagCommands >= 8 &&
        scsi_->phase() != Ncr5380::DataIn &&
        (block == 0x08000u || block == 0x0C000u || block == 0x0E000u)) {
        char message[112];
        std::snprintf(message, sizeof message,
                      "SCSI1 W %08X=%02X phase=%d pos=%u/%u pc=%08X",
                      addr, value, scsi_->phase(), scsi_->xferPos(),
                      scsi_->xferLen(), cpu_.pc);
        onDiag(message);
    }
}

u8 IifxMachine::rawRead8(u32 addr) {
    // With TC.IS=8 the 68030 table deliberately emits the low 24 bits.  The
    // Macintosh II bus then expands $9x_xxxx back into slot-9 standard space.
    // Do this ahead of DRAM decode; in 32-bit mode the same physical addresses
    // are ordinary RAM, which matters on 16-128 MB IIfx configurations.
    const bool address24 = cpu_.mmuEnabled() && ((cpu_.tc >> 16) & 0x0Fu) == 8u;
    if (videoEnabled_ && address24 &&
        (addr & 0x00F00000u) == 0x00900000u) {
        const u32 slotOffset = addr & 0x000FFFFFu;
        const u8 value = video_->readStandard(slotOffset);
        logAccess("NUBUS9-24", false, addr, value);
        return value;
    }
    // In 24-bit-compatible mode the Slot Manager constructs $Fssx_xxxx.
    // Macintosh II bus hardware aliases that form to slot s standard space;
    // decode it before the ordinary physical map so it cannot fall through as
    // ROM or RAM after the 68030 discards the logical high byte.
    if (videoEnabled_ &&
        (addr & 0xFFF00000u) == 0xF9900000u) {
        const u32 slotOffset = addr & 0x000FFFFFu;
        const u8 value = video_->readStandard(slotOffset);
        logAccess("NUBUS9-24", false, addr, value);
        return value;
    }
    if (addr < 0x40000000u) {
        if (overlay_) return rom_[addr & romMask_];
        if (addr < ram_.size()) return ram_[addr];
        return 0xFF;                         // unpopulated DRAM sizing probe
    }
    if (addr < 0x50000000u) {
        if (overlay_) {
            overlay_ = false;                // native-ROM access removes overlay
            clearFmcCache();                 // low tags referred to ROM, now RAM
        }
        return rom_[addr & romMask_];
    }
    if (addr < 0x54000000u) return ioRead8(addr);
    if (addr < 0x60000000u) throw BusFault{addr, true, 1};
    if (videoEnabled_ &&
        (addr & 0xFF000000u) == IifxNuBusVideo::kSlotBase) {
        const u8 value = video_->readStandard(addr - IifxNuBusVideo::kSlotBase);
        logAccess("NUBUS9", false, addr, value);
        if (video_->reads <= 64 && onDiag) {
            char message[96];
            std::snprintf(message, sizeof message, "NuBus9 R %08X=%02X pc=%08X",
                          addr, value, cpu_.pc);
            onDiag(message);
        }
        return value;
    }
    if (videoEnabled_ &&
        (addr & 0xF0000000u) == IifxNuBusVideo::kSuperSlotBase) {
        const std::size_t before = video_->firstGcReadCount;
        const u8 value = video_->readSuper(
            addr - IifxNuBusVideo::kSuperSlotBase, totalCycles_);
        logAccess("NUBUS9", false, addr, value);
        if (onDiag && before < 128u) {
            char message[112];
            std::snprintf(message, sizeof message,
                          "NuBus9 GC R %08X=%02X pc=%08X",
                          addr, value, cpu_.pc);
            onDiag(message);
        }
        return value;
    }
    // PDS, fast PDS, NuBus super/standard spaces and reserved $F0 are
    // unacknowledged until a card is explicitly installed.
    throw BusFault{addr, true, 1};
}

u8 IifxMachine::rawRead8Traced(u32 addr) {
    ++traceBusDepth_;
    try {
        const u8 value = rawRead8(addr);
        --traceBusDepth_;
        if (traceBusDepth_ == 0)
            traceAccess(addr < 0x40000000u ? HardwareTraceSource::Memory
                                          : addr < 0x50000000u
                                                ? HardwareTraceSource::Rom
                                                : HardwareTraceSource::Other,
                        false, addr, value, 1, "physical bus");
        return value;
    } catch (...) {
        --traceBusDepth_;
        throw;
    }
}

u32 IifxMachine::rawRead32(u32 addr) {
    u32 scsiReg;
    if ((addr & 3u) == 0 && scsiDmaLongRegister(addr, scsiReg)) {
        const u32 value = scsiDma_->read32(scsiReg);
        logAccess("SCSI-DMA", false, addr, value, 4);
        return value;
    }
    return (static_cast<u32>(rawRead8(addr)) << 24) |
           (static_cast<u32>(rawRead8(addr + 1)) << 16) |
           (static_cast<u32>(rawRead8(addr + 2)) << 8) |
           static_cast<u32>(rawRead8(addr + 3));
}

bool IifxMachine::fmcLookup(u32 addr, u32& word) const {
    if (!cacheable(addr) || !fmcCache_) return false;
    const FmcLine& line = fmcCache_[(addr >> 4) & (kFmcLines - 1u)];
    if (!line.valid || line.tag != (addr >> 15)) return false;
    word = line.data[(addr >> 2) & 3u];
    return true;
}

void IifxMachine::clearFmcCache() {
    if (!fmcCache_) return;
    for (u32 i = 0; i < kFmcLines; ++i) fmcCache_[i] = FmcLine{};
}

u8 IifxMachine::read8(u32 addr) {
    u32 word;
    if (fmcLookup(addr, word)) {
        ++fmcHits_;
        return static_cast<u8>(word >> ((3u - (addr & 3u)) * 8u));
    }
    if (cacheable(addr)) {
        ++fmcMisses_;
        fmcCyclePenalty_ += 4;                // documented FMC retry on a miss
    }
    return rawRead8Traced(addr);
}

u16 IifxMachine::read16(u32 addr) {
    if ((addr & 1u) == 0) {
        u32 word;
        if (fmcLookup(addr, word)) {
            ++fmcHits_;
            return static_cast<u16>(word >> ((addr & 2u) ? 0 : 16));
        }
        if (cacheable(addr)) {
            ++fmcMisses_;
            fmcCyclePenalty_ += 4;
            ++traceBusDepth_;
            const u16 value = static_cast<u16>((rawRead8(addr) << 8) |
                                               rawRead8(addr + 1));
            --traceBusDepth_;
            if (traceBusDepth_ == 0)
                traceAccess(addr < 0x40000000u ? HardwareTraceSource::Memory
                                              : HardwareTraceSource::Rom,
                            false, addr, value, 2, "physical bus");
            return value;
        }
    }
    return static_cast<u16>((static_cast<u16>(read8(addr)) << 8) |
                            read8(addr + 1));
}

u32 IifxMachine::read32(u32 addr) {
    u32 scsiReg;
    if ((addr & 3u) == 0 && scsiDmaLongRegister(addr, scsiReg))
        return rawRead32(addr);
    if ((addr & 3u) == 0) {
        u32 word;
        if (fmcLookup(addr, word)) {
            ++fmcHits_;
            return word;
        }
        if (cacheable(addr)) {
            ++fmcMisses_;
            fmcCyclePenalty_ += 4;
            return rawRead32(addr);
        }
    }
    return (static_cast<u32>(read16(addr)) << 16) | read16(addr + 2);
}

u8 IifxMachine::read8CacheInhibited(u32 addr) {
    ++fmcCacheInhibited_;
    return rawRead8(addr);
}

u16 IifxMachine::read16CacheInhibited(u32 addr) {
    ++fmcCacheInhibited_;
    return static_cast<u16>((static_cast<u16>(rawRead8(addr)) << 8) |
                            rawRead8(addr + 1));
}

u32 IifxMachine::read32CacheInhibited(u32 addr) {
    ++fmcCacheInhibited_;
    return rawRead32(addr);
}

void IifxMachine::rawWrite8(u32 addr, u8 value) {
    const bool gcSemaphore =
        (addr >= 0xF9008DD4u && addr <= 0xF9008DD7u) ||
        (addr >= 0x9C008DD4u && addr <= 0x9C008DD7u) ||
        (addr >= 0x92008DD4u && addr <= 0x92008DD7u);
    if (gcSemaphore && onDiag) {
        char message[96];
        std::snprintf(message, sizeof message,
                      "IOP GC shared semaphore W %08X=%02X pc=%08X cycle=%llu",
                      addr, value, cpu_.pc,
                      static_cast<unsigned long long>(totalCycles_));
        onDiag(message);
    }
    const bool address24 = cpu_.mmuEnabled() && ((cpu_.tc >> 16) & 0x0Fu) == 8u;
    if (videoEnabled_ && address24 &&
        (addr & 0x00F00000u) == 0x00900000u) {
        const u64 before = video_->vramWrites;
        video_->writeStandard(addr & 0x000FFFFFu, value);
        logAccess(video_->vramWrites != before ? "NUBUS9-VRAM-24" : "NUBUS9-24",
                  true, addr, value);
        updateIpl();
        return;
    }
    if (videoEnabled_ &&
        (addr & 0xFFF00000u) == 0xF9900000u) {
        const u64 before = video_->vramWrites;
        video_->writeStandard(addr & 0x000FFFFFu, value);
        logAccess(video_->vramWrites != before ? "NUBUS9-VRAM-24" : "NUBUS9-24",
                  true, addr, value);
        updateIpl();
        return;
    }
    if (addr < 0x40000000u) {
        if (!overlay_ && addr < ram_.size()) {
            ram_[addr] = value;
            const bool ramTest = cpu_.pc >= 0x40841400u &&
                                 cpu_.pc < 0x40841600u;
            if (addr >= 0x00000360u && addr < 0x0000036Cu &&
                !ramTest && onDiag && lowMemoryDiagBudget_ > 0) {
                --lowMemoryDiagBudget_;
                char message[112];
                std::snprintf(message, sizeof message,
                              "LOWMEM W %08X=%02X pc=%08X",
                              addr, value, cpu_.pc);
                onDiag(message);
            }
        }
        return;
    }
    if (addr < 0x50000000u) return;           // ROM is read-only
    if (addr < 0x54000000u) { ioWrite8(addr, value); return; }
    if (videoEnabled_ &&
        (addr & 0xFF000000u) == IifxNuBusVideo::kSlotBase) {
        const u64 before = video_->vramWrites;
        video_->writeStandard(addr - IifxNuBusVideo::kSlotBase, value);
        const bool vram = video_->vramWrites != before;
        if (!vram || video_->vramWrites <= 32u ||
            (video_->vramWrites & 0x0FFFu) == 0)
            logAccess(vram ? "NUBUS9-VRAM" : "NUBUS9", true, addr, value);
        updateIpl();
        return;
    }
    if (videoEnabled_ &&
        (addr & 0xF0000000u) == IifxNuBusVideo::kSuperSlotBase) {
        const std::size_t transactionIndex = video_->firstGcWriteCount;
        const u64 vramWritesBefore = video_->vramWrites;
        video_->writeSuper(addr - IifxNuBusVideo::kSuperSlotBase, value);
        const bool vram = video_->vramWrites != vramWritesBefore;
        if (!vram || video_->vramWrites <= 32u ||
            (video_->vramWrites & 0x0FFFu) == 0)
            logAccess(vram ? "NUBUS9-VRAM" : "NUBUS9", true, addr, value);
        if (onDiag && !vram && transactionIndex < 128u) {
            char message[112];
            std::snprintf(message, sizeof message,
                          "NuBus9 GC W %08X=%02X pc=%08X",
                          addr, value, cpu_.pc);
            onDiag(message);
        }
        const u32 superOffset = addr - IifxNuBusVideo::kSuperSlotBase;
        if (onDiag && superOffset >= 0x04000050u &&
            superOffset < 0x04000058u) {
            char message[112];
            std::snprintf(message, sizeof message,
                          "GC DBELL W %08X=%02X pc=%08X",
                          addr, value, cpu_.pc);
            onDiag(message);
        }
        updateIpl();
        return;
    }
    throw BusFault{addr, false, 1};
}

void IifxMachine::rawWrite8Traced(u32 addr, u8 value) {
    ++traceBusDepth_;
    try {
        rawWrite8(addr, value);
        --traceBusDepth_;
        if (traceBusDepth_ == 0)
            traceAccess(addr < 0x40000000u ? HardwareTraceSource::Memory
                                          : addr < 0x50000000u
                                                ? HardwareTraceSource::Rom
                                                : HardwareTraceSource::Other,
                        true, addr, value, 1, "physical bus");
    } catch (...) {
        --traceBusDepth_;
        throw;
    }
}

void IifxMachine::fmcMergeWrite(u32 addr, u8 value) {
    if (!cacheable(addr) || !fmcCache_) return;
    FmcLine& line = fmcCache_[(addr >> 4) & (kFmcLines - 1u)];
    if (!line.valid || line.tag != (addr >> 15)) return;
    u32& word = line.data[(addr >> 2) & 3u];
    const u32 shift = (3u - (addr & 3u)) * 8u;
    word = (word & ~(0xFFu << shift)) | (static_cast<u32>(value) << shift);
}

void IifxMachine::write8(u32 addr, u8 value) {
    // FMC snoops every write bus master and updates a resident block even if
    // the processor's own caches are disabled or frozen.
    fmcMergeWrite(addr, value);
    rawWrite8Traced(addr, value);
}

void IifxMachine::write16(u32 addr, u16 value) {
    write8(addr, static_cast<u8>(value >> 8));
    write8(addr + 1, static_cast<u8>(value));
}

void IifxMachine::write32(u32 addr, u32 value) {
    u32 scsiReg;
    if ((addr & 3u) == 0 && scsiDmaLongRegister(addr, scsiReg)) {
        scsiDma_->write32(scsiReg, value);
        logAccess("SCSI-DMA", true, addr, value, 4);
        return;
    }
    write16(addr, static_cast<u16>(value >> 16));
    write16(addr + 2, static_cast<u16>(value));
}

bool IifxMachine::cacheable(u32 addr) const {
    if (videoEnabled_ && cpu_.mmuEnabled() &&
        ((cpu_.tc >> 16) & 0x0Fu) == 8u &&
        (addr & 0x00F00000u) == 0x00900000u)
        return false;
    // The FMC asserts cache inhibit for motherboard I/O and expansion cycles.
    // Only populated DRAM (or the reset ROM overlay) and the native ROM take
    // part in the 68030/FMC cache hierarchy.
    if (addr < 0x40000000u) return overlay_ || addr < ram_.size();
    return addr < 0x50000000u;
}

bool IifxMachine::readBurst32(u32 firstAddr, u32 out[4]) {
    firstAddr &= ~3u;
    u32 word;
    if (fmcLookup(firstAddr, word)) {
        ++fmcHits_;
        out[0] = word;
        // A cache hit is a two-clock single transfer. FMC deliberately does
        // not assert CBACK, so the 68030 must not fill its remaining entries.
        return false;
    }
    if (!cacheable(firstAddr)) {
        out[0] = rawRead32(firstAddr);
        return false;
    }

    ++fmcMisses_;
    fmcCyclePenalty_ += 4;
    const u32 lineBase = firstAddr & ~15u;
    const u32 firstEntry = (firstAddr >> 2) & 3u;
    u32 filled[4]{};
    for (u32 n = 0; n < 4; ++n) {
        const u32 entry = (firstEntry + n) & 3u;
        filled[entry] = rawRead32(lineBase + entry * 4u);
        out[n] = filled[entry];
    }
    FmcLine& line = fmcCache_[(firstAddr >> 4) & (kFmcLines - 1u)];
    line.tag = firstAddr >> 15;
    line.valid = true;
    for (u32 i = 0; i < 4; ++i) line.data[i] = filled[i];
    ++fmcFills_;
    return true;
}

int IifxMachine::takeCyclePenalty() {
    const int result = fmcCyclePenalty_;
    fmcCyclePenalty_ = 0;
    return result;
}

int IifxMachine::screenWidth() const { return video_->width(); }
int IifxMachine::screenHeight() const { return video_->height(); }
void IifxMachine::renderScreen(u32* argbOut) const { video_->render(argbOut); }

int IifxMachine::diagnosticScsiPhase() const { return scsi_->phase(); }
u32 IifxMachine::diagnosticScsiCommandCount() const { return scsi_->diagCommands; }
u32 IifxMachine::diagnosticScsiTransferPosition() const { return scsi_->xferPos(); }
u32 IifxMachine::diagnosticScsiTransferLength() const { return scsi_->xferLen(); }
bool IifxMachine::diagnosticScsiIrq() const { return scsi_->irqAsserted(); }
u32 IifxMachine::diagnosticScsiDmaControl() const { return scsiDma_->control(); }
u32 IifxMachine::diagnosticScsiDmaCount() const { return scsiDma_->count(); }
u32 IifxMachine::diagnosticScsiDmaAddress() const { return scsiDma_->address(); }
bool IifxMachine::diagnosticScsiDmaActive() const { return scsiDma_->dmaActive(); }
bool IifxMachine::diagnosticScsiDrq() const { return scsiDma_->drqAsserted(); }
u32 IifxMachine::diagnosticGcProcessorPc() const {
    return video_->gcProcessorPc();
}
u64 IifxMachine::diagnosticGcProcessorInstructions() const {
    return video_->gcProcessorInstructions();
}
bool IifxMachine::diagnosticGcProcessorRecentlyExecuted(u32 pc) const {
    const std::size_t count = video_->gcProcessorRecentInstructionCount();
    for (std::size_t back = 0; back < count; ++back)
        if (video_->gcProcessorRecentPc(back) == pc) return true;
    return false;
}
void IifxMachine::diagnosticSetGcProcessorPcWatch(u32 pc) {
    video_->setGcProcessorPcWatch(pc);
}
u64 IifxMachine::diagnosticGcProcessorPcWatchHits() const {
    return video_->gcProcessorPcWatchHits();
}

void IifxMachine::diagnosticSetGcProcessorProfileRange(u32 first, u32 last) {
    video_->setGcProcessorProfileRange(first, last);
}

void IifxMachine::diagnosticSetGcPcTapRange(u32 first, u32 last) {
    video_->setGcPcTapRange(first, last);
}

void IifxMachine::diagnosticSetGcAddrTapRange(u32 first, u32 last,
                                              bool writesOnly) {
    video_->setGcAddrTapRange(first, last, writesOnly);
}

void IifxMachine::diagnosticSetGcDoorbellWatch(u32 address) {
    video_->setGcDoorbellWatchAddress(address);
}

void IifxMachine::diagnosticSetGcVramWatch(u32 offset) {
    video_->setGcVramWatch(offset);
}

void IifxMachine::diagnosticSetGcRegisterWatch(u16 physicalIndex) {
    video_->gcSetProcessorRegisterWatch(physicalIndex);
}

void IifxMachine::diagnosticSetGcPcSnap(u32 pc, u16 architecturalIndex) {
    video_->gcSetProcessorPcSnap(pc, architecturalIndex);
}

void IifxMachine::diagnosticSetGcFlightWindow(u64 start, u64 count,
                                              u16 physicalIndex) {
    video_->gcSetProcessorFlightWindow(start, count, physicalIndex);
}

bool IifxMachine::diagnosticWriteGcFlight(const char* path) const {
    std::ofstream file(path);
    if (!file) return false;
    char line[96];
    for (const Am29000::FlightEntry& entry : video_->gcProcessorFlight()) {
        std::snprintf(line, sizeof line, "%llu %08X %08X %08X %08X %08X\n",
                      static_cast<unsigned long long>(entry.instr), entry.pc,
                      entry.gr1, entry.rfb, entry.rab, entry.watched);
        file << line;
    }
    return file.good();
}

std::vector<std::pair<u64, u32>> IifxMachine::diagnosticGcProcessorProfile(
    std::size_t limit) const {
    std::vector<std::pair<u64, u32>> sites;
    const std::vector<u64>& counts = video_->gcProcessorProfileCounts();
    const u32 first = video_->gcProcessorProfileFirst();
    for (std::size_t index = 0; index < counts.size(); ++index) {
        if (counts[index] == 0) continue;
        sites.emplace_back(counts[index],
                           first + static_cast<u32>(index << 2u));
    }
    std::sort(sites.begin(), sites.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
    if (sites.size() > limit) sites.resize(limit);
    return sites;
}

std::vector<u8> IifxMachine::diagnosticGcDataMemory(
    u32 offset, u32 size) const {
    std::vector<u8> result;
    if (size == 0 || offset >= IifxNuBusVideo::kGcDramBytes ||
        size > IifxNuBusVideo::kGcDramBytes - offset)
        return result;
    result.resize(size);
    for (u32 byte = 0; byte < size; ++byte) {
        const u32 address = offset + byte;
        const u32 word = video_->gcDiagnosticDramWord(address & ~3u);
        result[byte] = static_cast<u8>(word >> ((3u - (address & 3u)) * 8u));
    }
    return result;
}

std::vector<u8> IifxMachine::diagnosticGcExpansionMemory(
    u32 offset, u32 size) const {
    std::vector<u8> result;
    if (size == 0 || offset >= IifxNuBusVideo::kGcExpansionDramBytes ||
        size > IifxNuBusVideo::kGcExpansionDramBytes - offset)
        return result;
    result.resize(size);
    for (u32 byte = 0; byte < size; ++byte) {
        const u32 address = offset + byte;
        const u32 word = video_->gcDiagnosticExpansionDramWord(address & ~3u);
        result[byte] = static_cast<u8>(word >> ((3u - (address & 3u)) * 8u));
    }
    return result;
}

std::vector<u8> IifxMachine::diagnosticGcInstructionSram(
    u32 offset, u32 size) const {
    std::vector<u8> result;
    if (size == 0 || offset >= IifxNuBusVideo::kGcSramBytes ||
        size > IifxNuBusVideo::kGcSramBytes - offset)
        return result;
    result.resize(size);
    for (u32 byte = 0; byte < size; ++byte) {
        const u32 address = offset + byte;
        const u32 word = video_->gcDiagnosticSramWord(address & ~3u);
        result[byte] = static_cast<u8>(word >> ((3u - (address & 3u)) * 8u));
    }
    return result;
}

std::string IifxMachine::diagnosticReport() const {
    const auto ram16 = [&](u32 address) -> u16 {
        return address + 1u < ram_.size()
            ? static_cast<u16>((static_cast<u16>(ram_[address]) << 8) |
                               ram_[address + 1u])
            : 0;
    };
    const auto ram32 = [&](u32 address) -> u32 {
        return address + 3u < ram_.size()
            ? (static_cast<u32>(ram_[address]) << 24) |
                  (static_cast<u32>(ram_[address + 1u]) << 16) |
                  (static_cast<u32>(ram_[address + 2u]) << 8) |
                  ram_[address + 3u]
            : 0;
    };
    const auto ramPointer = [&](u32 pointer) -> u32 {
        if (pointer < ram_.size()) return pointer;
        const u32 low24 = pointer & 0x00FFFFFFu;
        return low24 < ram_.size() ? low24 : pointer;
    };
    const u32 mainDeviceHandle = ramPointer(ram32(0x08A4u));
    const u32 mainDevice = ramPointer(ram32(mainDeviceHandle));
    const u32 pixMapHandle = ramPointer(ram32(mainDevice + 0x16u));
    const u32 pixMap = ramPointer(ram32(pixMapHandle));
    char text[4096];
    std::snprintf(text, sizeof text,
                  "Macintosh IIfx\n"
                  "  cycles=%llu frame=%llu overlay=%d power=%d\n"
                  "  CPU pc=%08X sr=%04X sp=%08X halted=%d stopped=%d "
                  "tc=%08X cacr=%08X\n"
                  "  D0-7 %08X %08X %08X %08X %08X %08X %08X %08X\n"
                  "  A0-6 %08X %08X %08X %08X %08X %08X %08X\n"
                  "  OSS pending=%04X ipl=%d VIA ifr=%02X ier=%02X\n"
                  "  SCC IOP status=%02X addr=%04X host=%u/%u ram=%u/%u cmd=%u\n"
                  "    send max=%02X state=%02X %02X %02X %02X %02X %02X %02X "
                  "recv max=%02X state=%02X %02X %02X %02X %02X %02X %02X alive=%02X\n"
                  "  ISM IOP status=%02X addr=%04X host=%u/%u ram=%u/%u cmd=%u\n"
                  "    send max=%02X state=%02X %02X %02X %02X %02X %02X %02X "
                  "recv max=%02X state=%02X %02X %02X %02X %02X %02X %02X alive=%02X\n"
                  "  NuBus video card=%s mode=%04X bpp=%d reads=%llu "
                  "rom=%llu super=%llu writes=%llu vram=%llu\n"
                   "    VBL enabled=%d irq=%d frames=%llu assertions=%llu "
                   "acks=%llu last-frame-writes=%llu base=%08X stride=%u\n"
                   "    guest ScreenRow=%u ScrnBase=%08X CrsrBase=%08X "
                   "MainDevice=%08X DeviceList=%08X\n"
                   "    guest gd=%08X pm=%08X base=%08X rowBytes=%u "
                   "bounds=%d,%d,%d,%d pixel=%u/%u/%u\n"
                   "  SCSI reg reads=%u writes=%u selects=%u commands=%u\n",
                  static_cast<unsigned long long>(totalCycles_),
                  static_cast<unsigned long long>(frameCounter_), overlay_ ? 1 : 0,
                  poweredOff() ? 1 : 0, cpu_.pc, cpu_.getSR(), cpu_.a[7],
                  cpu_.halted ? 1 : 0, cpu_.stopped ? 1 : 0, cpu_.tc, cpu_.cacr,
                  cpu_.d[0], cpu_.d[1], cpu_.d[2], cpu_.d[3], cpu_.d[4],
                  cpu_.d[5], cpu_.d[6], cpu_.d[7], cpu_.a[0], cpu_.a[1],
                  cpu_.a[2], cpu_.a[3], cpu_.a[4], cpu_.a[5], cpu_.a[6],
                  oss_->pending(), oss_->ipl(), via_->ifr(), via_->ier(),
                  sccIop_->status(), sccIop_->address(), sccIop_->hostReads,
                  sccIop_->hostWrites, sccIop_->ramReads, sccIop_->ramWrites,
                  sccIop_->mailboxCommands,
                  sccIop_->ram(IifxIop::SendMax),
                  sccIop_->ram(IifxIop::SendState + 0),
                  sccIop_->ram(IifxIop::SendState + 1),
                  sccIop_->ram(IifxIop::SendState + 2),
                  sccIop_->ram(IifxIop::SendState + 3),
                  sccIop_->ram(IifxIop::SendState + 4),
                  sccIop_->ram(IifxIop::SendState + 5),
                  sccIop_->ram(IifxIop::SendState + 6),
                  sccIop_->ram(IifxIop::RecvMax),
                  sccIop_->ram(IifxIop::RecvState + 0),
                  sccIop_->ram(IifxIop::RecvState + 1),
                  sccIop_->ram(IifxIop::RecvState + 2),
                  sccIop_->ram(IifxIop::RecvState + 3),
                  sccIop_->ram(IifxIop::RecvState + 4),
                  sccIop_->ram(IifxIop::RecvState + 5),
                  sccIop_->ram(IifxIop::RecvState + 6),
                  sccIop_->ram(IifxIop::Alive),
                  ismIop_->status(), ismIop_->address(), ismIop_->hostReads,
                  ismIop_->hostWrites, ismIop_->ramReads, ismIop_->ramWrites,
                  ismIop_->mailboxCommands,
                  ismIop_->ram(IifxIop::SendMax),
                  ismIop_->ram(IifxIop::SendState + 0),
                  ismIop_->ram(IifxIop::SendState + 1),
                  ismIop_->ram(IifxIop::SendState + 2),
                  ismIop_->ram(IifxIop::SendState + 3),
                  ismIop_->ram(IifxIop::SendState + 4),
                  ismIop_->ram(IifxIop::SendState + 5),
                  ismIop_->ram(IifxIop::SendState + 6),
                  ismIop_->ram(IifxIop::RecvMax),
                  ismIop_->ram(IifxIop::RecvState + 0),
                  ismIop_->ram(IifxIop::RecvState + 1),
                  ismIop_->ram(IifxIop::RecvState + 2),
                  ismIop_->ram(IifxIop::RecvState + 3),
                  ismIop_->ram(IifxIop::RecvState + 4),
                  ismIop_->ram(IifxIop::RecvState + 5),
                  ismIop_->ram(IifxIop::RecvState + 6),
                  ismIop_->ram(IifxIop::Alive),
                  video_->genuineGc() ? "8*24 GC" : "synthetic",
                  video_->mode(), video_->bitsPerPixel(),
                  static_cast<unsigned long long>(video_->reads),
                  static_cast<unsigned long long>(video_->romReads),
                  static_cast<unsigned long long>(video_->superReads),
                  static_cast<unsigned long long>(video_->writes),
                  static_cast<unsigned long long>(video_->vramWrites),
                  video_->vblankEnabled() ? 1 : 0,
                  video_->irqAsserted() ? 1 : 0,
                  static_cast<unsigned long long>(video_->vblankCount),
                  static_cast<unsigned long long>(video_->vblankAssertions),
                  static_cast<unsigned long long>(video_->vblankAcks),
                   static_cast<unsigned long long>(video_->lastFrameVramWrites),
                   video_->displayBase(), video_->displayStride(),
                   ram16(0x0106u), ram32(0x0824u), ram32(0x0898u),
                   ram32(0x08A4u), ram32(0x08A8u),
                   mainDevice, pixMap, ram32(pixMap),
                   ram16(pixMap + 4u) & 0x3FFFu,
                   static_cast<s16>(ram16(pixMap + 6u)),
                   static_cast<s16>(ram16(pixMap + 8u)),
                   static_cast<s16>(ram16(pixMap + 10u)),
                   static_cast<s16>(ram16(pixMap + 12u)),
                   ram16(pixMap + 30u), ram16(pixMap + 32u),
                   ram16(pixMap + 36u),
                   scsi_->diagReads, scsi_->diagWrites, scsi_->diagSelects,
                  scsi_->diagCommands);
    std::string result(text);
    if (video_->genuineGc()) {
        char item[320];
        std::snprintf(item, sizeof item,
                       "    GC Am29000 released=%d pc=%08X cps=%08X cfg=%08X mmu=%08X "
                       "tmc/tmr=%06X/%08X instructions=%llu "
                       "gc-vram=%llu fault=%d unknown-data=%llu "
                       "first-unknown=%08X%s%s\n",
                      video_->gcProcessorReleased() ? 1 : 0,
                      video_->gcProcessorPc(),
                      video_->gcProcessorStatus(),
                      video_->gcProcessorConfiguration(),
                      video_->gcProcessorMmu(),
                      video_->gcProcessorTimerCounter(),
                      video_->gcProcessorTimerReload(),
                       static_cast<unsigned long long>(
                           video_->gcProcessorInstructions()),
                       static_cast<unsigned long long>(
                           video_->gcProcessorVramWrites()),
                      video_->gcProcessorFaulted() ? 1 : 0,
                      static_cast<unsigned long long>(
                          video_->gcUnknownDataAccesses()),
                      video_->gcFirstUnknownDataAddress(),
                      video_->gcProcessorFaulted() ? " reason=" : "",
                      video_->gcProcessorFaulted()
                          ? video_->gcProcessorFaultReason().c_str() : "");
        result += item;
        std::snprintf(item, sizeof item,
                      "      GC interrupt facility: timer-expiries=%llu "
                      "timer-interrupts=%llu timer-writes=%llu "
                      "external=%llu/%llu/%llu/%llu\n",
                      static_cast<unsigned long long>(
                          video_->gcProcessorTimerExpiries()),
                      static_cast<unsigned long long>(
                          video_->gcProcessorTimerInterrupts()),
                      static_cast<unsigned long long>(
                          video_->gcProcessorTimerWrites()),
                      static_cast<unsigned long long>(
                          video_->gcProcessorExternalInterrupts(0)),
                      static_cast<unsigned long long>(
                          video_->gcProcessorExternalInterrupts(1)),
                      static_cast<unsigned long long>(
                          video_->gcProcessorExternalInterrupts(2)),
                      static_cast<unsigned long long>(
                          video_->gcProcessorExternalInterrupts(3)));
        result += item;
        std::snprintf(item, sizeof item,
                      "      GC external lines asserted=%llu/%llu/%llu/%llu "
                      "dropped-standard-writes=%llu\n",
                      static_cast<unsigned long long>(
                          video_->gcProcessorExternalAssertions(0)),
                      static_cast<unsigned long long>(
                          video_->gcProcessorExternalAssertions(1)),
                      static_cast<unsigned long long>(
                          video_->gcProcessorExternalAssertions(2)),
                      static_cast<unsigned long long>(
                          video_->gcProcessorExternalAssertions(3)),
                      static_cast<unsigned long long>(
                          video_->gcDroppedStandardWrites));
        result += item;
        if (video_->gcDoorbellEdgeCount != 0) {
            std::snprintf(item, sizeof item,
                          "      GC doorbell INTR2 edges (%zu):",
                          video_->gcDoorbellEdgeCount);
            result += item;
            for (std::size_t index = 0;
                 index < video_->gcDoorbellEdgeCount; ++index) {
                std::snprintf(item, sizeof item, " %s#%llu cps=%08X ctl=%08X",
                              video_->gcDoorbellEdgeStates[index] ? "SET"
                                                                  : "CLR",
                              static_cast<unsigned long long>(
                                  video_->gcDoorbellEdgeInstructions[index]),
                              video_->gcDoorbellEdgeCps[index],
                              video_->gcDoorbellEdgeControl[index]);
                result += item;
            }
            result += '\n';
        }
        if (video_->gcDroppedStandardWriteCount != 0) {
            std::snprintf(item, sizeof item,
                          "      first dropped standard writes (%zu):",
                          video_->gcDroppedStandardWriteCount);
            result += item;
            for (std::size_t index = 0;
                 index < video_->gcDroppedStandardWriteCount; ++index) {
                std::snprintf(item, sizeof item, " %06X:%02X",
                              video_->gcDroppedStandardWriteOffsets[index],
                              video_->gcDroppedStandardWriteValues[index]);
                result += item;
            }
            result += '\n';
        }
        if (video_->gcPcTapCount != 0) {
            std::snprintf(item, sizeof item,
                          "      GC pc-tap ring (%zu):",
                          video_->gcPcTapCount);
            result += item;
            for (std::size_t index = 0;
                 index < video_->gcPcTapCount; ++index) {
                std::snprintf(item, sizeof item, " %s@%08X %08X=%08X#%llu",
                              video_->gcPcTapWrites[index] ? "W" : "R",
                              video_->gcPcTapPcs[index],
                              video_->gcPcTapAddresses[index],
                              video_->gcPcTapValues[index],
                              static_cast<unsigned long long>(
                                  video_->gcPcTapInstructions[index]));
                result += item;
            }
            result += '\n';
        }
        std::snprintf(item, sizeof item,
                      "      GC window rsp=%08X msp(gr125)=%08X\n",
                      video_->gcProcessorRegister(1),
                      video_->gcProcessorRegister(125));
        result += item;
        if (video_->gcProcessorPcWatchHitCount() != 0) {
            std::snprintf(item, sizeof item,
                          "      GC pc-watch hits (first %zu, instr#):",
                          video_->gcProcessorPcWatchHitCount());
            result += item;
            for (std::size_t index = 0;
                 index < video_->gcProcessorPcWatchHitCount(); ++index) {
                std::snprintf(item, sizeof item, " %llu",
                              static_cast<unsigned long long>(
                                  video_->gcProcessorPcWatchHitInstruction(
                                      index)));
                result += item;
            }
            result += '\n';
        }
        if (video_->gcProcessorRegisterWatchCount() != 0) {
            std::snprintf(item, sizeof item,
                          "      GC register watch (last %zu, newest first):",
                          video_->gcProcessorRegisterWatchCount());
            result += item;
            for (std::size_t back = 0;
                 back < video_->gcProcessorRegisterWatchCount(); ++back) {
                std::snprintf(item, sizeof item, " %08X@%08X#%llu",
                              video_->gcProcessorRegisterWatchValue(back),
                              video_->gcProcessorRegisterWatchPc(back),
                              static_cast<unsigned long long>(
                                  video_->gcProcessorRegisterWatchInstruction(
                                      back)));
                result += item;
            }
            result += '\n';
        }
        if (video_->gcProcessorPcSnapCount() != 0) {
            std::snprintf(item, sizeof item,
                          "      GC pc snap (last %zu, newest first, "
                          "gr1/value/rfb/rab#instr):",
                          video_->gcProcessorPcSnapCount());
            result += item;
            for (std::size_t back = 0;
                 back < video_->gcProcessorPcSnapCount(); ++back) {
                std::snprintf(item, sizeof item, " %08X/%08X/%08X/%08X#%llu",
                              video_->gcProcessorPcSnapWindowBase(back),
                              video_->gcProcessorPcSnapValue(back),
                              video_->gcProcessorPcSnapSpillBound(back),
                              video_->gcProcessorPcSnapFillBound(back),
                              static_cast<unsigned long long>(
                                  video_->gcProcessorPcSnapInstruction(back)));
                result += item;
            }
            result += '\n';
        }
        if (video_->gcProcessorTimerWriteRingCount() != 0) {
            std::snprintf(item, sizeof item,
                          "      GC timer writes (last %zu, newest first):",
                          video_->gcProcessorTimerWriteRingCount());
            result += item;
            for (std::size_t back = 0;
                 back < video_->gcProcessorTimerWriteRingCount(); ++back) {
                std::snprintf(item, sizeof item, " %s=%08X@%08X#%llu",
                              video_->gcProcessorTimerWriteIsReload(back)
                                  ? "TMR" : "TMC",
                              video_->gcProcessorTimerWriteValue(back),
                              video_->gcProcessorTimerWritePc(back),
                              static_cast<unsigned long long>(
                                  video_->gcProcessorTimerWriteInstruction(
                                      back)));
                result += item;
            }
            result += '\n';
        }
        if (video_->gcProcessorCallTraceCount() != 0) {
            std::snprintf(item, sizeof item,
                          "      GC call trace (last %zu, newest first):",
                          video_->gcProcessorCallTraceCount());
            result += item;
            for (std::size_t back = 0;
                 back < video_->gcProcessorCallTraceCount(); ++back) {
                std::snprintf(item, sizeof item, " %08X>%08X#%llu",
                              video_->gcProcessorCallTracePc(back),
                              video_->gcProcessorCallTraceTarget(back),
                              static_cast<unsigned long long>(
                                  video_->gcProcessorCallTraceInstruction(
                                      back)));
                result += item;
            }
            result += '\n';
        }
        if (video_->gcVramWatchCount != 0) {
            std::snprintf(item, sizeof item,
                          "      GC vram watch @%06X (%zu writes):\n",
                          video_->gcVramWatchOffset_,
                          video_->gcVramWatchCount);
            result += item;
            for (std::size_t index = 0; index < video_->gcVramWatchCount;
                 ++index) {
                std::snprintf(item, sizeof item,
                              "        #%llu pc=%08X value=%08X trail:",
                              static_cast<unsigned long long>(
                                  video_->gcVramWatchInstructions[index]),
                              video_->gcVramWatchPcs[index],
                              video_->gcVramWatchValues[index]);
                result += item;
                for (std::size_t back = 0;
                     back < IifxNuBusVideo::kGcVramWatchTrail; ++back) {
                    const std::size_t slot =
                        index * IifxNuBusVideo::kGcVramWatchTrail + back;
                    if (video_->gcVramWatchTrailPcs[slot] == 0) break;
                    std::snprintf(item, sizeof item, " %08X>%08X",
                                  video_->gcVramWatchTrailPcs[slot],
                                  video_->gcVramWatchTrailTargets[slot]);
                    result += item;
                }
                result += "\n";
            }
        }
        if (video_->gcPhantomCaptured) {
            std::snprintf(item, sizeof item,
                          "      GC first phantom access: %s %08X=%08X pc=%08X"
                          " #%llu\n        trail (newest first):",
                          video_->gcPhantomWrite ? "W" : "R",
                          video_->gcPhantomAddress, video_->gcPhantomValue,
                          video_->gcPhantomPc,
                          static_cast<unsigned long long>(
                              video_->gcPhantomInstruction));
            result += item;
            for (std::size_t back = 0; back < video_->gcPhantomTraceCount;
                 ++back) {
                std::snprintf(item, sizeof item, " %08X>%08X#%llu",
                              video_->gcPhantomTracePcs[back],
                              video_->gcPhantomTraceTargets[back],
                              static_cast<unsigned long long>(
                                  video_->gcPhantomTraceInstructions[back]));
                result += item;
            }
            result += "\n        regs gr64-127:";
            for (u32 index = 64; index < 128u; ++index) {
                std::snprintf(item, sizeof item, "%s%08X",
                              (index % 16u) == 0 ? "\n          " : " ",
                              video_->gcPhantomRegisters[index]);
                result += item;
            }
            result += "\n        regs lr0-63:";
            for (u32 index = 128; index < 192u; ++index) {
                std::snprintf(item, sizeof item, "%s%08X",
                              (index % 16u) == 0 ? "\n          " : " ",
                              video_->gcPhantomRegisters[index]);
                result += item;
            }
            result += '\n';
        }
        if (video_->gcDoorbellTraceCount != 0) {
            std::snprintf(item, sizeof item,
                          "      GC doorbell bus-master ring (%zu):",
                          video_->gcDoorbellTraceCount);
            result += item;
            for (std::size_t index = 0;
                 index < video_->gcDoorbellTraceCount; ++index) {
                std::snprintf(item, sizeof item, " %s%08X@%08X#%llu",
                              video_->gcDoorbellTraceWrites[index] ? "W" : "R",
                              video_->gcDoorbellTraceValues[index],
                              video_->gcDoorbellTracePcs[index],
                              static_cast<unsigned long long>(
                                  video_->gcDoorbellTraceInstructions[index]));
                result += item;
            }
            result += '\n';
        }
        result += "      data read regions:";
        for (std::size_t region = 0;
             region < video_->gcReadRegionCounts.size(); ++region) {
            if (video_->gcReadRegionCounts[region] == 0) continue;
            std::snprintf(item, sizeof item, " %02zX:%llu[%08X@%08X]",
                          region,
                          static_cast<unsigned long long>(
                              video_->gcReadRegionCounts[region]),
                          video_->gcReadRegionFirstAddresses[region],
                          video_->gcReadRegionFirstPcs[region]);
            result += item;
        }
        result += '\n';
        std::snprintf(item, sizeof item,
                      "      data reads/writes=%llu/%llu store regions:",
                      static_cast<unsigned long long>(
                          video_->gcProcessorDataReads()),
                      static_cast<unsigned long long>(
                          video_->gcProcessorDataWrites()));
        result += item;
        for (std::size_t region = 0;
             region < video_->gcWriteRegionCounts.size(); ++region) {
            if (video_->gcWriteRegionCounts[region] == 0) continue;
            std::snprintf(item, sizeof item,
                          " %02zX:%llu[%08X=%08X@%08X]", region,
                          static_cast<unsigned long long>(
                              video_->gcWriteRegionCounts[region]),
                          video_->gcWriteRegionFirstAddresses[region],
                          video_->gcWriteRegionFirstValues[region],
                          video_->gcWriteRegionFirstPcs[region]);
            result += item;
        }
        result += '\n';
        result += "      local-memory store sites:";
        for (std::size_t site = 0;
             site < video_->gcLocalWriteSiteCount; ++site) {
            std::snprintf(item, sizeof item,
                          " %02X@%08X:%llu[%08X-%08X %08X>%08X]",
                          video_->gcLocalWriteSiteRegions[site],
                          video_->gcLocalWriteSitePcs[site],
                          static_cast<unsigned long long>(
                              video_->gcLocalWriteSiteCounts[site]),
                          video_->gcLocalWriteSiteMinAddresses[site],
                          video_->gcLocalWriteSiteMaxAddresses[site],
                          video_->gcLocalWriteSiteFirstValues[site],
                          video_->gcLocalWriteSiteLastValues[site]);
            result += item;
        }
        result += '\n';
        result += "      MFB write census:";
        for (std::size_t reg = 0;
             reg < video_->gcMfbWriteCounts.size(); ++reg) {
            if (video_->gcMfbWriteCounts[reg] == 0) continue;
            std::snprintf(item, sizeof item,
                          " +%03zX:%llu[%08X>%08X@%08X]", reg * 4u,
                          static_cast<unsigned long long>(
                              video_->gcMfbWriteCounts[reg]),
                          video_->gcMfbWriteFirstValues[reg],
                          video_->gcMfbWriteLastValues[reg],
                          video_->gcMfbWriteLastPcs[reg]);
            result += item;
        }
        result += '\n';
        result += "      MFB write history:";
        for (std::size_t index = 0;
             index < video_->gcMfbWriteTraceCount; ++index) {
            std::snprintf(item, sizeof item, " +%03X=%08X@%08X",
                          video_->gcMfbWriteTraceAddresses[index] -
                              0x44000000u,
                          video_->gcMfbWriteTraceValues[index],
                          video_->gcMfbWriteTracePcs[index]);
            result += item;
        }
        result += '\n';
        result += "      host GCQD selector census:";
        for (std::size_t selector = 0;
             selector < video_->gcHostSelectorCounts.size(); ++selector) {
            if (video_->gcHostSelectorCounts[selector] == 0) continue;
            std::snprintf(item, sizeof item, " %04zX:%llu", selector,
                          static_cast<unsigned long long>(
                              video_->gcHostSelectorCounts[selector]));
            result += item;
        }
        result += '\n';
        result += "      host GCQD commands:";
        for (std::size_t index = 0;
             index < video_->gcHostCommandCount; ++index) {
            std::snprintf(item, sizeof item, " S%u:%08X",
                          video_->gcHostCommandSources[index],
                          video_->gcHostCommandValues[index]);
            result += item;
        }
        result += '\n';
        result += "      GC completion writes:";
        for (std::size_t index = 0;
             index < video_->gcCompletionTraceCount; ++index) {
            std::snprintf(item, sizeof item, " %08X=%08X@%08X[r128-143",
                          video_->gcCompletionTraceAddresses[index],
                          video_->gcCompletionTraceValues[index],
                          video_->gcCompletionTracePcs[index]);
            result += item;
            for (u32 value : video_->gcCompletionTraceRegisters[index]) {
                std::snprintf(item, sizeof item, ":%08X", value);
                result += item;
            }
            result += ']';
        }
        result += '\n';
        if (video_->gcAbnormalCompletionCaptured) {
            std::snprintf(item, sizeof item,
                          "      abnormal completion snapshot "
                          "%08X=%08X@%08X instructions=%llu "
                          "cps/ops=%08X/%08X pc0/1/2=%08X/%08X/%08X "
                          "iret=%08X\n",
                          video_->gcAbnormalCompletionAddress,
                          video_->gcAbnormalCompletionValue,
                          video_->gcAbnormalCompletionPc,
                          static_cast<unsigned long long>(
                              video_->gcAbnormalCompletionInstructions),
                          video_->gcAbnormalCompletionCps,
                          video_->gcAbnormalCompletionOps,
                          video_->gcAbnormalCompletionPc0,
                          video_->gcAbnormalCompletionPc1,
                          video_->gcAbnormalCompletionPc2,
                          video_->gcAbnormalCompletionIretPc);
            result += item;
            for (std::size_t first = 0;
                 first < video_->gcAbnormalCompletionRegisters.size();
                 first += 16u) {
                std::snprintf(item, sizeof item,
                              "        snapshot r%zu-%zu:",
                              first + 64u, first + 79u);
                result += item;
                for (std::size_t index = first; index < first + 16u;
                     ++index) {
                    std::snprintf(item, sizeof item, " %08X",
                                  video_->gcAbnormalCompletionRegisters[index]);
                    result += item;
                }
                result += '\n';
            }
            result += "        snapshot recent:";
            for (std::size_t back =
                     video_->gcAbnormalCompletionRecentCount;
                 back != 0; --back) {
                const std::size_t index = back - 1u;
                std::snprintf(item, sizeof item, " %08X:%08X",
                              video_->gcAbnormalCompletionRecentPcs[index],
                              video_->gcAbnormalCompletionRecentInstructions[index]);
                result += item;
            }
            result += '\n';
        }
        std::snprintf(item, sizeof item,
                      "      pipeline exec=%08X ir=%08X pc0/1/2="
                      "%08X/%08X/%08X next=%08X prefetched=%08X "
                      "flags=%08X/%08X iret=%08X ops=%08X vab=%08X\n",
                      video_->gcProcessorInstructionPc(),
                      video_->gcProcessorInstruction(),
                      video_->gcProcessorPc0(), video_->gcProcessorPc1(),
                      video_->gcProcessorPc2(), video_->gcProcessorNextPc(),
                      video_->gcProcessorPrefetchedInstruction(),
                      video_->gcProcessorPipelineFlags(),
                      video_->gcProcessorNextPipelineFlags(),
                      video_->gcProcessorIretPc(),
                      video_->gcProcessorOldStatus(),
                      video_->gcProcessorVectorBase());
        result += item;
        std::snprintf(item, sizeof item,
                      "      globals r64-79=%08X %08X %08X %08X %08X %08X %08X %08X "
                      "%08X %08X %08X %08X %08X %08X %08X %08X\n",
                      video_->gcProcessorRegister(64),
                      video_->gcProcessorRegister(65),
                      video_->gcProcessorRegister(66),
                      video_->gcProcessorRegister(67),
                      video_->gcProcessorRegister(68),
                      video_->gcProcessorRegister(69),
                      video_->gcProcessorRegister(70),
                      video_->gcProcessorRegister(71),
                      video_->gcProcessorRegister(72),
                      video_->gcProcessorRegister(73),
                      video_->gcProcessorRegister(74),
                      video_->gcProcessorRegister(75),
                      video_->gcProcessorRegister(76),
                      video_->gcProcessorRegister(77),
                      video_->gcProcessorRegister(78),
                      video_->gcProcessorRegister(79));
        result += item;
        std::snprintf(item, sizeof item,
                      "      globals r80-95=%08X %08X %08X %08X %08X %08X %08X %08X "
                      "%08X %08X %08X %08X %08X %08X %08X %08X\n",
                      video_->gcProcessorRegister(80),
                      video_->gcProcessorRegister(81),
                      video_->gcProcessorRegister(82),
                      video_->gcProcessorRegister(83),
                      video_->gcProcessorRegister(84),
                      video_->gcProcessorRegister(85),
                      video_->gcProcessorRegister(86),
                      video_->gcProcessorRegister(87),
                      video_->gcProcessorRegister(88),
                      video_->gcProcessorRegister(89),
                      video_->gcProcessorRegister(90),
                      video_->gcProcessorRegister(91),
                      video_->gcProcessorRegister(92),
                      video_->gcProcessorRegister(93),
                      video_->gcProcessorRegister(94),
                      video_->gcProcessorRegister(95));
        result += item;
        std::snprintf(item, sizeof item,
                      "      globals r96-111=%08X %08X %08X %08X %08X %08X %08X %08X "
                      "%08X %08X %08X %08X %08X %08X %08X %08X\n",
                      video_->gcProcessorRegister(96),
                      video_->gcProcessorRegister(97),
                      video_->gcProcessorRegister(98),
                      video_->gcProcessorRegister(99),
                      video_->gcProcessorRegister(100),
                      video_->gcProcessorRegister(101),
                      video_->gcProcessorRegister(102),
                      video_->gcProcessorRegister(103),
                      video_->gcProcessorRegister(104),
                      video_->gcProcessorRegister(105),
                      video_->gcProcessorRegister(106),
                      video_->gcProcessorRegister(107),
                      video_->gcProcessorRegister(108),
                      video_->gcProcessorRegister(109),
                      video_->gcProcessorRegister(110),
                      video_->gcProcessorRegister(111));
        result += item;
        std::snprintf(item, sizeof item,
                      "      locals r128-139 (gr1=%08X)=%08X %08X %08X %08X %08X %08X %08X %08X "
                      "%08X %08X %08X %08X\n",
                      video_->gcProcessorRegisterWindowBase(),
                      video_->gcProcessorRegister(128),
                      video_->gcProcessorRegister(129),
                      video_->gcProcessorRegister(130),
                      video_->gcProcessorRegister(131),
                      video_->gcProcessorRegister(132),
                      video_->gcProcessorRegister(133),
                      video_->gcProcessorRegister(134),
                      video_->gcProcessorRegister(135),
                      video_->gcProcessorRegister(136),
                      video_->gcProcessorRegister(137),
                      video_->gcProcessorRegister(138),
                      video_->gcProcessorRegister(139));
        result += item;
        std::snprintf(item, sizeof item,
                      "      locals r140-151=%08X %08X %08X %08X %08X %08X %08X %08X "
                      "%08X %08X %08X %08X\n",
                      video_->gcProcessorRegister(140),
                      video_->gcProcessorRegister(141),
                      video_->gcProcessorRegister(142),
                      video_->gcProcessorRegister(143),
                      video_->gcProcessorRegister(144),
                      video_->gcProcessorRegister(145),
                      video_->gcProcessorRegister(146),
                      video_->gcProcessorRegister(147),
                      video_->gcProcessorRegister(148),
                      video_->gcProcessorRegister(149),
                      video_->gcProcessorRegister(150),
                      video_->gcProcessorRegister(151));
        result += item;
        result += "      recent instructions:";
        const std::size_t recent = std::min<std::size_t>(
            video_->gcProcessorRecentInstructionCount(), 32u);
        for (std::size_t back = recent; back != 0; --back) {
            std::snprintf(item, sizeof item, " %08X:%08X",
                          video_->gcProcessorRecentPc(back - 1u),
                          video_->gcProcessorRecentInstruction(back - 1u));
            result += item;
        }
        result += '\n';
        result += "      r64 changes:";
        const std::size_t changes = std::min<std::size_t>(
            video_->gcProcessorRegister64ChangeCount(), 32u);
        for (std::size_t back = changes; back != 0; --back) {
            std::snprintf(item, sizeof item, " %08X:%08X:%08X>%08X",
                          video_->gcProcessorRegister64ChangePc(back - 1u),
                          video_->gcProcessorRegister64ChangeInstruction(back - 1u),
                          video_->gcProcessorRegister64ChangeOldValue(back - 1u),
                          video_->gcProcessorRegister64ChangeNewValue(back - 1u));
            result += item;
        }
        result += '\n';
    }
    for (const auto& iop : {std::pair{"SCC", sccIop_.get()},
                            std::pair{"ISM", ismIop_.get()}}) {
        const R65C02& pic = iop.second->cpu();
        char item[256];
        std::snprintf(item, sizeof item,
                      "    %s firmware pc=%04X op=%02X A/X/Y/S/P="
                      "%02X/%02X/%02X/%02X/%02X instructions=%llu cycles=%llu "
                      "timer=%llu dma=%llu brk=%llu irq=%02X/%02X ctrl=%02X/%02X "
                      "dma0=%02X/%04X/%04X/%d dma1=%02X/%04X/%04X/%d\n",
                      iop.first, pic.pc, pic.lastOpcode, pic.a, pic.x, pic.y,
                      pic.s, pic.p,
                      static_cast<unsigned long long>(pic.instructions),
                      static_cast<unsigned long long>(pic.cycles),
                      static_cast<unsigned long long>(iop.second->timerExpirations),
                      static_cast<unsigned long long>(iop.second->dmaTransfers),
                      static_cast<unsigned long long>(pic.brkCount),
                      iop.second->interruptFlags(), iop.second->interruptMask(),
                      iop.second->timerDpllControl(), iop.second->ioControl(),
                      iop.second->dmaControl(0), iop.second->dmaMap(0),
                      iop.second->dmaCount(0), iop.second->dmaRequest(0) ? 1 : 0,
                      iop.second->dmaControl(1), iop.second->dmaMap(1),
                      iop.second->dmaCount(1), iop.second->dmaRequest(1) ? 1 : 0);
        result += item;
        result += "      recent:";
        for (int back = 15; back >= 0; --back) {
            std::snprintf(item, sizeof item, " %04X", pic.recentPc(back));
            result += item;
        }
        result += '\n';
        if (pic.brkCount) {
            result += "      pre-BRK:";
            for (int back = 15; back >= 0; --back) {
                std::snprintf(item, sizeof item, " %04X", pic.preBrkPc(back));
                result += item;
            }
            result += '\n';
        }
    }
    {
        char item[1024];
        sound_->debugState(item, sizeof item);
        result += "  ASC ";
        result += item;
        result += '\n';
    }
    {
        const auto& cache = cpu_.cacheStats030();
        char item[192];
        std::snprintf(item, sizeof item,
                      "  68030 cache I hit/miss=%llu/%llu D hit/miss=%llu/%llu "
                      "burst-longs=%llu\n",
                      static_cast<unsigned long long>(cache.instructionHits),
                      static_cast<unsigned long long>(cache.instructionMisses),
                      static_cast<unsigned long long>(cache.dataHits),
                      static_cast<unsigned long long>(cache.dataMisses),
                      static_cast<unsigned long long>(cache.burstLongwords));
        result += item;
    }
    {
        char item[320];
        std::snprintf(item, sizeof item,
                      "  ADB wire commands=%llu replies=%llu bytes=%llu resets=%llu "
                      "samples=%llu/%llu aborts=%llu listens=%llu pending=%d last=%02X "
                      "listen=%02X/%02X dev=%X/%X\n"
                      "    device mouse polls=%u reports=%u bytes-read=%u "
                      "keyboard-polls=%u\n",
                      static_cast<unsigned long long>(adbBus_->commands()),
                      static_cast<unsigned long long>(adbBus_->replies()),
                      static_cast<unsigned long long>(adbBus_->replyBytes()),
                      static_cast<unsigned long long>(adbBus_->resets()),
                      static_cast<unsigned long long>(adbBus_->replySamples()),
                      static_cast<unsigned long long>(adbBus_->replyLowSamples()),
                      static_cast<unsigned long long>(adbBus_->abortedReplies()),
                      static_cast<unsigned long long>(adbBus_->listenTransactions()),
                      adb_->hasPendingEvent() ? 1 : 0, adbBus_->lastCommand(),
                      adbBus_->lastListenByte(0),
                      adbBus_->lastListenByte(1), adb_->keyboardAddress(),
                      adb_->mouseAddress(), adb_->mousePolls(),
                      adb_->mouseReports(), adb_->mouseBytesRead(),
                      adb_->kbdPolls());
        result += item;
    }
    {
        // A display card can be discovered and adopted as the main GDevice
        // even when its ROM driver was never installed or opened.  Keep the
        // Device Manager state beside the video counters so a frozen cursor
        // can be assigned to Slot Manager, DriverInstall/Open, or VBL setup
        // from one bounded diagnostic snapshot.
        const auto readable = [&](u32 address, u32 bytes) {
            return address < ram_.size() && bytes <= ram_.size() - address;
        };
        const u32 unitTable = ramPointer(ram32(0x011Cu));
        const u16 unitCount = ram16(0x01D2u);
        char item[320];
        std::snprintf(item, sizeof item,
                      "  Device Manager units=%u table=%08X\n",
                      unitCount, unitTable);
        result += item;
        const u32 boundedCount = std::min<u32>(unitCount, 64u);
        for (u32 unit = 0; unit < boundedCount; ++unit) {
            const u32 entry = unitTable + unit * 4u;
            if (!readable(entry, 4u)) break;
            const u32 dceHandle = ramPointer(ram32(entry));
            if (!dceHandle || !readable(dceHandle, 4u)) continue;
            const u32 dce = ramPointer(ram32(dceHandle));
            if (!dce || !readable(dce, 0x2Eu)) continue;
            const u16 flags = ram16(dce + 4u);
            u32 driver = ramPointer(ram32(dce));
            if ((flags & 0x0040u) != 0 && readable(driver, 4u))
                driver = ramPointer(ram32(driver));
            std::string name = "?";
            if (readable(driver + 18u, 1u)) {
                const u8 length = ram_[driver + 18u];
                if (length && length <= 63u && readable(driver + 19u, length))
                    name.assign(reinterpret_cast<const char*>(ram_.data() + driver + 19u),
                                length);
            }
            std::snprintf(item, sizeof item,
                          "    unit=%u ref=%d flags=%04X opened=%d dce=%08X "
                          "driver=%08X slot=%02X id=%02X base=%08X name=%s\n",
                          unit, static_cast<int>(static_cast<s16>(~u16(unit))),
                          flags, (flags & 0x0020u) != 0 ? 1 : 0, dce, driver,
                          ram_[dce + 0x28u], ram_[dce + 0x29u],
                          ram32(dce + 0x2Au), name.c_str());
            result += item;
        }
    }
    {
        char item[192];
        std::snprintf(item, sizeof item,
                      "  FMC cache hit/miss=%llu/%llu fills=%llu CI-bypass=%llu "
                      "pending-wait=%d\n",
                      static_cast<unsigned long long>(fmcHits_),
                      static_cast<unsigned long long>(fmcMisses_),
                      static_cast<unsigned long long>(fmcFills_),
                      static_cast<unsigned long long>(fmcCacheInhibited_),
                      fmcCyclePenalty_);
        result += item;
    }
    {
        char item[256];
        std::snprintf(item, sizeof item,
                      "  hard disk bytes=%zu pending=%d mounted=%d pb=%08X "
                      "tries=%u result=%d trap-timeouts=%u\n"
                      "    HFS offset=%u bytes=%u driver=%08X reads=%u writes=%u\n",
                      hardDisk_.size(), hardDiskMountPending_ ? 1 : 0,
                      hardDiskMounted_ ? 1 : 0, hardDiskMountPb_,
                      hardDiskMountTries_, hardDiskMountResult_,
                      injectedTrapTimeouts_, hfsImageOffset_, hfsVolumeBytes_,
                      diskPrimePc_, hardDiskReadCount_, hardDiskWriteCount_);
        result += item;
    }
    {
        char item[224];
        const auto& p = rtc_->xpram();
        std::snprintf(item, sizeof item,
                      "  RTC seconds=%08X PRAM[0..15]="
                      "%02X %02X %02X %02X %02X %02X %02X %02X "
                      "%02X %02X %02X %02X %02X %02X %02X %02X "
                      "PRAM[10..13]=%02X %02X %02X %02X PRAM[8A]=%02X\n",
                      rtc_->seconds(), p[0], p[1], p[2], p[3], p[4], p[5],
                      p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13],
                      p[14], p[15], p[0x10], p[0x11], p[0x12], p[0x13],
                      p[0x8A]);
        result += item;
    }
    if (video_->firstReadCount) {
        result += "    first slot-ROM offsets:";
        char item[16];
        for (std::size_t i = 0; i < video_->firstReadCount; ++i) {
            std::snprintf(item, sizeof item, " %06X", video_->firstReadOffsets[i]);
            result += item;
        }
        result += '\n';
    }
    if (video_->firstGcReadCount) {
        result += "    first GC register reads:";
        char item[24];
        for (std::size_t i = 0; i < video_->firstGcReadCount; ++i) {
            std::snprintf(item, sizeof item, " %08X:%02X",
                          video_->firstGcReadOffsets[i],
                          video_->firstGcReadValues[i]);
            result += item;
        }
        result += '\n';
    }
    if (video_->firstGcWriteCount) {
        result += "    first GC register writes:";
        char item[24];
        for (std::size_t i = 0; i < video_->firstGcWriteCount; ++i) {
            std::snprintf(item, sizeof item, " %08X:%02X",
                          video_->firstGcWriteOffsets[i],
                          video_->firstGcWriteValues[i]);
            result += item;
        }
        result += '\n';
    }
    if (video_->gcUnknownTraceCount) {
        result += "    first GC Am29000 unmapped accesses:";
        char item[64];
        for (std::size_t i = 0; i < video_->gcUnknownTraceCount; ++i) {
            std::snprintf(item, sizeof item, " %c%08X=%08X@%08X",
                          video_->gcUnknownTraceWrites[i] ? 'W' : 'R',
                          video_->gcUnknownTraceAddresses[i],
                          video_->gcUnknownTraceValues[i],
                          video_->gcUnknownTracePcs[i]);
            result += item;
        }
        result += '\n';
    }
    if (video_->gcQuickDrawWriteTraceCount) {
        result += "    first GC QuickDraw physical writes:";
        char item[64];
        for (std::size_t i = 0;
             i < video_->gcQuickDrawWriteTraceCount; ++i) {
            std::snprintf(item, sizeof item, " W%08X=%08X@%08X",
                          video_->gcQuickDrawWriteTraceAddresses[i],
                          video_->gcQuickDrawWriteTraceValues[i],
                          video_->gcQuickDrawWriteTracePcs[i]);
            result += item;
        }
        result += '\n';
    }
    if (video_->gcFramebufferReadTraceCount) {
        result += "    first GC framebuffer reads:";
        char item[64];
        for (std::size_t i = 0;
             i < video_->gcFramebufferReadTraceCount; ++i) {
            std::snprintf(item, sizeof item, " R%08X=%08X@%08X",
                          video_->gcFramebufferReadTraceAddresses[i],
                          video_->gcFramebufferReadTraceValues[i],
                          video_->gcFramebufferReadTracePcs[i]);
            result += item;
        }
        result += '\n';
    }
    if (video_->gcWorkBufferReadTraceCount) {
        result += "    GC post-raster work-buffer reads:";
        char item[64];
        for (std::size_t i = 0;
             i < video_->gcWorkBufferReadTraceCount; ++i) {
            std::snprintf(item, sizeof item, " R%08X=%08X@%08X",
                          video_->gcWorkBufferReadTraceAddresses[i],
                          video_->gcWorkBufferReadTraceValues[i],
                          video_->gcWorkBufferReadTracePcs[i]);
            result += item;
        }
        result += '\n';
    }
    const auto appendGcRegisterSnapshot = [&](const char* label, u32 address,
                                               u32 pc,
                                               const std::array<u32, 128>& regs) {
        result += std::string("    ") + label;
        char item[64];
        std::snprintf(item, sizeof item, " address=%08X pc=%08X r64-191:",
                      address, pc);
        result += item;
        for (u32 value : regs) {
            std::snprintf(item, sizeof item, " %08X", value);
            result += item;
        }
        result += '\n';
    };
    if (video_->gcRasterStoreSnapshotCaptured)
        appendGcRegisterSnapshot("first GC raster store",
            video_->gcRasterStoreSnapshotAddress,
            video_->gcRasterStoreSnapshotPc,
            video_->gcRasterStoreRegisterSnapshot);
    if (video_->gcUnknownSnapshotCaptured)
        appendGcRegisterSnapshot("first GC unknown access",
            video_->gcUnknownSnapshotAddress, video_->gcUnknownSnapshotPc,
            video_->gcUnknownRegisterSnapshot);
    if (video_->gcLowReadTraceCount) {
        result += "    first GC low-aperture reads:";
        char item[128];
        for (std::size_t i = 0; i < video_->gcLowReadTraceCount; ++i) {
            std::snprintf(item, sizeof item,
                          " R%08X=%08X@%08X[cps=%X ops=%X ir=%08X]",
                          video_->gcLowReadTraceAddresses[i],
                          video_->gcLowReadTraceValues[i],
                          video_->gcLowReadTracePcs[i],
                          video_->gcLowReadTraceCps[i],
                          video_->gcLowReadTraceOps[i],
                          video_->gcLowReadTraceInstructions[i]);
            result += item;
        }
        result += '\n';
    }
    if (video_->gcPointerReadTraceCount) {
        result += "    GC pointer-path reads:";
        char item[64];
        for (std::size_t i = 0; i < video_->gcPointerReadTraceCount; ++i) {
            std::snprintf(item, sizeof item, " R%08X=%08X@%08X",
                          video_->gcPointerReadTraceAddresses[i],
                          video_->gcPointerReadTraceValues[i],
                          video_->gcPointerReadTracePcs[i]);
            result += item;
        }
        result += '\n';
    }
    if (video_->gcMapTraceCount) {
        result += "    GC mapping-table writes:";
        char item[96];
        for (std::size_t i = 0; i < video_->gcMapTraceCount; ++i) {
            std::snprintf(item, sizeof item, " [%08X+%08X->%08X@%08X]",
                          video_->gcMapTraceBases[i],
                          video_->gcMapTraceLengths[i],
                          video_->gcMapTracePhysicals[i],
                          video_->gcMapTracePcs[i]);
            result += item;
        }
        result += '\n';
    }
    if (video_->gcSyncTraceCount) {
        result += "    GC sync-word $8DD4 accesses:";
        char item[72];
        for (std::size_t i = 0; i < video_->gcSyncTraceCount; ++i) {
            std::snprintf(item, sizeof item, " %c%08X=%08X@%08X",
                          video_->gcSyncTraceWrites[i] ? 'W' : 'R',
                          video_->gcSyncTraceAddresses[i],
                          video_->gcSyncTraceValues[i],
                          video_->gcSyncTracePcs[i]);
            result += item;
        }
        result += '\n';
    }
    if (video_->gcSyncSnapshotCaptured) {
        result += "    GC sync-read registers r64-191:";
        char item[24];
        for (u32 value : video_->gcSyncRegisterSnapshot) {
            std::snprintf(item, sizeof item, " %08X", value);
            result += item;
        }
        result += '\n';
    }
    if (video_->gcSemaphoreMutationCount) {
        result += "    GC semaphore $8DD4 mutations:";
        char item[64];
        for (std::size_t i = 0; i < video_->gcSemaphoreMutationCount; ++i) {
            std::snprintf(item, sizeof item, " S%u:%08X@%08X",
                          video_->gcSemaphoreMutationSources[i],
                          video_->gcSemaphoreMutationValues[i],
                          video_->gcSemaphoreMutationPcs[i]);
            result += item;
        }
        result += '\n';
    }
    const auto appendGcSnapshot = [&](const char* label,
                                      const std::array<u32, 128>& mfb,
                                      const std::array<u32, 128>& regs) {
        result += std::string("    ") + label + " registers r64-191:";
        char item[64];
        for (u32 value : regs) {
            std::snprintf(item, sizeof item, " %08X", value);
            result += item;
        }
        result += '\n';
        result += std::string("    ") + label + " nonzero MFB:";
        for (std::size_t index = 0; index < mfb.size(); ++index) {
            if (mfb[index] != 0) {
                std::snprintf(item, sizeof item, " +%03X=%08X",
                              static_cast<u32>(index * 4u), mfb[index]);
                result += item;
            }
        }
        result += '\n';
    };
    if (video_->gcPointerMfbSnapshotCaptured)
        appendGcSnapshot("GC pointer-read", video_->gcPointerMfbSnapshot,
                         video_->gcPointerRegisterSnapshot);
    if (video_->gcRasterMfbSnapshotCaptured)
        appendGcSnapshot("GC raster-read", video_->gcRasterMfbSnapshot,
                         video_->gcRasterRegisterSnapshot);
    if (video_->gcLowWriteSnapshotCaptured) {
        char item[128];
        std::snprintf(item, sizeof item,
                      "    GC first low write W%08X=%08X@%08X\n",
                      video_->gcLowWriteSnapshotAddress,
                      video_->gcLowWriteSnapshotValue,
                      video_->gcLowWriteSnapshotPc);
        result += item;
        appendGcSnapshot("GC low-write", video_->gcLowWriteMfbSnapshot,
                         video_->gcLowWriteRegisterSnapshot);
        result += "    GC low-write map table $316C:";
        for (std::size_t index = 0;
             index < video_->gcLowWriteMapSnapshot.size(); ++index) {
            const u32 value = video_->gcLowWriteMapSnapshot[index];
            if (value == 0) continue;
            std::snprintf(item, sizeof item, " +%03zX=%08X",
                          index * 4u, value);
            result += item;
        }
        result += '\n';
    }
    if (video_->gcHeapOverwriteCaptured) {
        char item[256];
        std::snprintf(item, sizeof item,
                      "    GC heap overwrite $B184: value=%08X pc=%08X\n"
                      "      r128-r151:",
                      video_->gcHeapOverwriteValue,
                      video_->gcHeapOverwritePc);
        result += item;
        result += "      r96-r127:";
        for (std::size_t index = 0;
             index < video_->gcHeapOverwriteGlobals.size(); ++index) {
            std::snprintf(item, sizeof item, " r%zu=%08X", 96u + index,
                          video_->gcHeapOverwriteGlobals[index]);
            result += item;
        }
        result += '\n';
        for (std::size_t index = 0;
             index < video_->gcHeapOverwriteRegisters.size(); ++index) {
            std::snprintf(item, sizeof item, " r%zu=%08X", 128u + index,
                          video_->gcHeapOverwriteRegisters[index]);
            result += item;
        }
        result += '\n';
        result += "      MFB:";
        for (std::size_t index = 0;
             index < video_->gcHeapOverwriteMfb.size(); ++index) {
            const u32 value = video_->gcHeapOverwriteMfb[index];
            if (value == 0) continue;
            std::snprintf(item, sizeof item, " +%03zX=%08X", index * 4u,
                          value);
            result += item;
        }
        result += '\n';
    }
    if (video_->genuineGc()) {
        result += "    GC Am29000 nonzero TLB registers:";
        char item[256];
        for (std::size_t i = 0; i < 128; ++i) {
            const u32 value = video_->gcProcessorTlbRegister(i);
            if (!value) continue;
            std::snprintf(item, sizeof item, " %02zX:%08X", i, value);
            result += item;
        }
        result += '\n';
        std::snprintf(item, sizeof item,
                      "    GC vectors: unaligned=%08X intr0=%08X intr1=%08X intr2=%08X "
                      "intr3=%08X timer=%08X trap64=%08X trap65=%08X "
                      "trap66=%08X trap69=%08X\n",
                      video_->gcDiagnosticVectorWord(0x4C000000u + 1u * 4u),
                      video_->gcDiagnosticVectorWord(0x4C000000u + 16u * 4u),
                      video_->gcDiagnosticVectorWord(0x4C000000u + 17u * 4u),
                      video_->gcDiagnosticVectorWord(0x4C000000u + 18u * 4u),
                      video_->gcDiagnosticVectorWord(0x4C000000u + 19u * 4u),
                      video_->gcDiagnosticVectorWord(0x4C000000u + 14u * 4u),
                      video_->gcDiagnosticVectorWord(0x4C000000u + 64u * 4u),
                      video_->gcDiagnosticVectorWord(0x4C000000u + 65u * 4u),
                      video_->gcDiagnosticVectorWord(0x4C000000u + 66u * 4u),
                      video_->gcDiagnosticVectorWord(0x4C000000u + 69u * 4u));
        result += item;
        result += "    GC raw DRAM configuration $6400:";
        for (u32 offset = 0x6400u; offset < 0x6480u; offset += 4u) {
            std::snprintf(item, sizeof item, " %04X:%08X", offset,
                          video_->gcDiagnosticDramWord(offset));
            result += item;
        }
        result += '\n';
        result += "    GC page-map buckets $2818:";
        for (u32 offset = 0x2818u; offset < 0x2898u; offset += 4u) {
            std::snprintf(item, sizeof item, " %04X:%08X", offset,
                          video_->gcDiagnosticDramWord(offset));
            result += item;
        }
        result += '\n';
        std::snprintf(item, sizeof item,
                      "    GC raw DRAM high: 008C00:%08X 1FA000:%08X "
                      "1FD51C:%08X SRAM 8C00:%08X F000:%08X\n",
                      video_->gcDiagnosticDramWord(0x00008C00u),
                      video_->gcDiagnosticDramWord(0x001FA000u),
                      video_->gcDiagnosticDramWord(0x001FD51Cu),
                      video_->gcDiagnosticSramWord(0x00008C00u),
                      video_->gcDiagnosticSramWord(0x0000F000u));
        result += item;
        result += "    GC IPC $8C00:";
        for (u32 offset = 0x8C00u; offset < 0x8C60u; offset += 4u) {
            std::snprintf(item, sizeof item, " +%02X:%08X", offset - 0x8C00u,
                          video_->gcDiagnosticDramWord(offset));
            result += item;
        }
        result += '\n';
        std::snprintf(item, sizeof item,
                      "    GC handshakes: host=%d mac=%d MFB50=%08X "
                      "MFB54=%08X MFB64=%08X MFB68=%08X\n",
                      video_->gcHostRequestPending() ? 1 : 0,
                      video_->gcMacRequestPending() ? 1 : 0,
                      video_->gcMfbDiagnosticRegister(0x50u),
                      video_->gcMfbDiagnosticRegister(0x54u),
                      video_->gcMfbDiagnosticRegister(0x64u),
                      video_->gcMfbDiagnosticRegister(0x68u));
        result += item;
        std::snprintf(item, sizeof item,
                      "    GC expansion anchors: 7AD3F8:%08X 7BD400:%08X "
                      "7EC500:%08X 7FD51C:%08X\n",
                      video_->gcDiagnosticExpansionDramWord(0x007AD3F8u),
                      video_->gcDiagnosticExpansionDramWord(0x007BD400u),
                      video_->gcDiagnosticExpansionDramWord(0x007EC500u),
                      video_->gcDiagnosticExpansionDramWord(0x007FD51Cu));
        result += item;
    }
    {
        char item[64];
        std::snprintf(item, sizeof item,
                      "    SCSI live phase=%d cdb=%d/%d xfer=%u/%u data=%u/%u writes:",
                      scsi_->phase(), scsi_->cdbPos(), scsi_->cdbLen(),
                      scsi_->xferPos(), scsi_->xferLen(),
                      scsi_->diagDataInBytes, scsi_->diagDataOutBytes);
        result += item;
        for (int i = 0; i < scsi_->diagWriteTraceLen; ++i) {
            std::snprintf(item, sizeof item, " %X:%02X",
                          scsi_->diagWriteTrace[i] >> 8,
                          scsi_->diagWriteTrace[i] & 0xFF);
            result += item;
        }
        result += '\n';
        result += "    SCSI CDB history:";
        for (int i = 0; i < scsi_->diagCdbHistLen; ++i) {
            result += " [";
            const int len = ScsiTarget::cdbLen(scsi_->diagCdbHist[i][0]);
            for (int j = 0; j < len; ++j) {
                std::snprintf(item, sizeof item, "%s%02X", j ? " " : "",
                              scsi_->diagCdbHist[i][j]);
                result += item;
            }
            result += ']';
        }
        result += '\n';
    }
    return result;
}

} // namespace openmac
