#pragma once

// NCR 5380 SCSI Interface Controller + a direct-access disk target, as the
// Macintosh Classic wires it.
//
// Address decode: the 5380 lives in a 512 KB block at $580000-$5FFFFF (the Classic
// ROM uses base $5FF000). register = (addr >> 4) & 7; a read comes through an even
// Mac address (Machine::read8) and a write through an odd one (Machine::write8).
//
// This models the register file, the SCSI bus phase state machine (Bus Free ->
// Arbitration -> Selection -> Command -> Data In/Out -> Status -> Message In), and
// one selectable disk target that answers the boot command set. Transfers complete
// instantly, so we keep the two lines the ROM's transfer loops actually watch --
// CSR /REQ and BSR DRQ/Phase-Match -- honest, and never leave a byte "pending" past
// the end of a phase (no /BERR timeout is modelled).
//
// Reference: NCR 5380 Design Manual (1985) §6; Guide to the Macintosh Family
// Hardware 2nd ed. Ch.1/2/11; SCSI-1/2 command set. Clean-room from the specs.

#include "openmac/types.hpp"

#include <cstring>
#include <string>
#include <vector>

namespace openmac {

// A direct-access (type 0) SCSI disk backed by a raw 512-byte-sector image.
class ScsiDisk {
public:
    void attach(std::vector<u8>* image, int id) { image_ = image; id_ = id & 7; }
    void detach() { image_ = nullptr; }
    bool present() const {
        // Expose the disk to the SCSI bus only when block 0 carries an Apple Driver
        // Descriptor Map ('ER', 0x4552) -- a disk the ROM's boot can read a partition
        // map and driver from. A bare volume with no map is left to the .Sony shim so
        // the boot scan skips it instead of engaging an unusable disk and hanging.
        return image_ != nullptr && image_->size() >= 512 &&
               (*image_)[0] == 0x45 && (*image_)[1] == 0x52;
    }
    int  id() const { return id_; }
    bool readOnly = false;

    u32 blockCount() const {
        return image_ ? static_cast<u32>(image_->size() / 512u) : 0u;
    }

    // Number of CDB bytes for an opcode, from its group code (bits 7-5).
    static int cdbLen(u8 opcode) {
        switch ((opcode >> 5) & 7) {
            case 0: return 6;    // group 0
            case 1: case 2: return 10;   // group 1/2
            case 5: return 12;   // group 5
            default: return 6;
        }
    }

    // Execute a complete CDB. `out` receives any Data-In bytes to hand back to the
    // initiator. Returns the SCSI status byte (0x00 GOOD / 0x02 CHECK CONDITION).
    // For WRITE, out is left empty and `outIsWrite`/`writeLBA`/`writeBlocks` describe
    // the Data-Out the initiator will push next.
    u8 execute(const u8* cdb, std::vector<u8>& out,
               bool& outIsWrite, u32& writeLBA, u32& writeBlocks) {
        out.clear();
        outIsWrite = false;
        writeLBA = writeBlocks = 0;
        senseKey_ = 0;
        const u8 op = cdb[0];
        switch (op) {
            case 0x00:  // TEST UNIT READY
                return present() ? 0x00 : checkCondition(0x02);   // 0x02 = not ready
            case 0x03:  // REQUEST SENSE
                appendSense(out, cdb[4]);
                return 0x00;
            case 0x12:  // INQUIRY
                appendInquiry(out, cdb[4]);
                return 0x00;
            case 0x1A:  // MODE SENSE(6)
                appendModeSense(out, cdb[4]);
                return 0x00;
            case 0x25:  // READ CAPACITY(10)
                appendReadCapacity(out);
                return 0x00;
            case 0x08: {  // READ(6)
                const u32 lba = ((cdb[1] & 0x1Fu) << 16) | (cdb[2] << 8) | cdb[3];
                const u32 n = cdb[4] ? cdb[4] : 256u;
                return readBlocks(lba, n, out);
            }
            case 0x28: {  // READ(10)
                const u32 lba = be32(cdb + 2);
                const u32 n = (cdb[7] << 8) | cdb[8];
                return readBlocks(lba, n, out);
            }
            case 0x0A: {  // WRITE(6)
                outIsWrite = true;
                writeLBA = ((cdb[1] & 0x1Fu) << 16) | (cdb[2] << 8) | cdb[3];
                writeBlocks = cdb[4] ? cdb[4] : 256u;
                return 0x00;
            }
            case 0x2A: {  // WRITE(10)
                outIsWrite = true;
                writeLBA = be32(cdb + 2);
                writeBlocks = (cdb[7] << 8) | cdb[8];
                return 0x00;
            }
            default:
                return checkCondition(0x05);   // 0x05 = illegal request
        }
    }

    // Commit a completed Data-Out (WRITE) into the image.
    void writeData(u32 lba, const std::vector<u8>& data) {
        if (!image_ || readOnly) return;
        const std::size_t off = static_cast<std::size_t>(lba) * 512u;
        for (std::size_t i = 0; i < data.size() && off + i < image_->size(); ++i)
            (*image_)[off + i] = data[i];
    }

private:
    static u32 be32(const u8* p) {
        return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | p[3];
    }

    u8 checkCondition(u8 key) { senseKey_ = key; return 0x02; }

    u8 readBlocks(u32 lba, u32 n, std::vector<u8>& out) {
        if (!image_) return checkCondition(0x02);
        if (lba >= blockCount()) return checkCondition(0x05);   // LBA out of range
        if (n > blockCount() - lba) n = blockCount() - lba;     // clamp to media
        out.resize(static_cast<std::size_t>(n) * 512u, 0);
        const std::size_t off = static_cast<std::size_t>(lba) * 512u;
        for (std::size_t i = 0; i < out.size(); ++i)
            out[i] = (off + i < image_->size()) ? (*image_)[off + i] : 0;
        return 0x00;
    }

    void appendInquiry(std::vector<u8>& out, u8 allocLen) {
        u8 d[36] = {};
        d[0] = 0x00;   // peripheral device type 0 = direct access
        d[1] = 0x00;   // not removable
        d[2] = 0x02;   // SCSI-2
        d[3] = 0x02;   // response data format
        d[4] = 31;     // additional length (total 36)
        std::memcpy(d + 8,  "OpenMac ", 8);           // vendor id (8)
        std::memcpy(d + 16, "Hard Disk       ", 16);  // product id (16)
        std::memcpy(d + 24, "0001", 4);               // revision (4)
        emit(out, d, sizeof d, allocLen);
    }

    void appendReadCapacity(std::vector<u8>& out) {
        const u32 lastLba = blockCount() ? blockCount() - 1u : 0u;
        u8 d[8];
        d[0] = u8(lastLba >> 24); d[1] = u8(lastLba >> 16);
        d[2] = u8(lastLba >> 8);  d[3] = u8(lastLba);
        d[4] = 0; d[5] = 0; d[6] = 0x02; d[7] = 0x00;   // block size 512
        emit(out, d, sizeof d, 8);
    }

    void appendModeSense(std::vector<u8>& out, u8 allocLen) {
        // Minimal 4-byte mode parameter header, no block descriptors, no pages.
        u8 d[4] = {3, 0, 0, 0};
        emit(out, d, sizeof d, allocLen);
    }

    void appendSense(std::vector<u8>& out, u8 allocLen) {
        u8 d[18] = {};
        d[0] = 0x70;         // current error, fixed format
        d[2] = senseKey_;    // sense key
        d[7] = 10;           // additional sense length
        emit(out, d, sizeof d, allocLen ? allocLen : 18);
    }

    static void emit(std::vector<u8>& out, const u8* d, std::size_t n, u8 allocLen) {
        const std::size_t len = (allocLen && allocLen < n) ? allocLen : n;
        out.insert(out.end(), d, d + len);
    }

    std::vector<u8>* image_ = nullptr;
    int id_ = 0;
    u8  senseKey_ = 0;
};

class Ncr5380 {
public:
    enum Phase { BusFree, Arbitration, Selection, Command, DataOut, DataIn, Status, MsgIn };

    ScsiDisk disk;

    // Diagnostics (persistent; not subject to the machine's rolling stub log).
    u32 diagReads = 0;         // total register reads
    u32 diagWrites = 0;        // total register writes
    u32 diagSelects = 0;       // successful target selections
    u32 diagCommands = 0;      // CDBs executed
    u32 diagDataInBytes = 0;   // bytes handed to the initiator
    u32 diagDataOutBytes = 0;  // bytes taken from the initiator
    u8  diagLastCdb[12] = {};
    int diagLastCdbLen = 0;
    u8  diagCdbHist[16][12] = {};   // first 16 CDBs, to read the probe sequence
    int diagCdbHistLen = 0;
    u16 diagWriteTrace[320] = {};   // packed (reg<<8)|val for the first register writes
    int diagWriteTraceLen = 0;

    void reset() {
        odr_ = icr_ = mr_ = tcr_ = ser_ = 0;
        phase_ = BusFree;
        xfer_.clear();
        xferPos_ = 0;
        cdbPos_ = cdbLen_ = 0;
        status_ = 0;
        writeMode_ = false;
        writeLBA_ = writeBlocks_ = 0;
    }

    u8 read(int reg) {
        ++diagReads;
        switch (reg & 7) {
            case 0: return dataIn();        // Current SCSI Data (CSD)
            case 1: return readIcr();       // Initiator Command (ICR)
            case 2: return mr_;             // Mode (MR)
            case 3: return tcr_;            // Target Command (TCR)
            case 4: return readCsr();       // Current SCSI Bus Status (CSR)
            case 5: return readBsr();       // Bus and Status (BSR)
            case 6: return dataIn();        // Input Data (IDR) -- pseudo-DMA reads here
            case 7: return 0;               // Reset Parity/Interrupts (read clears)
        }
        return 0;
    }

    void write(int reg, u8 v) {
        ++diagWrites;
        if (diagWriteTraceLen < 320)
            diagWriteTrace[diagWriteTraceLen++] = static_cast<u16>(((reg & 7) << 8) | v);
        switch (reg & 7) {
            case 0: dataOut(v); break;      // Output Data (ODR)
            case 1: writeIcr(v); break;     // Initiator Command (ICR)
            case 2: writeMr(v); break;      // Mode (MR)
            case 3: tcr_ = v; break;        // Target Command (TCR)
            case 4: ser_ = v; break;        // Select Enable (SER)
            case 6: dataOut(v); break;      // pseudo-DMA send: initiator -> target data (symmetric to read reg 6)
            case 5: case 7: break;          // Start DMA Send / Reset (mode start; data flows via reg 0/6)
        }
    }

private:
    // ---- registers -------------------------------------------------------
    void writeMr(u8 v) {
        const u8 prev = mr_;
        mr_ = v;
        // Arbitration: setting Arbitrate (b0) while the bus is free begins
        // arbitration. With a single initiator we win it immediately (AIP set in the
        // ICR read below, LA never). Clearing it before selection abandons the bus.
        if ((v & 0x01) && !(prev & 0x01) && phase_ == BusFree)
            phase_ = Arbitration;
        else if (!(v & 0x01) && phase_ == Arbitration)
            phase_ = BusFree;
    }

    void writeIcr(u8 v) {
        icr_ = v;
        // Selection: the initiator asserts /SEL (b2) with the target ID on the data
        // bus (ODR) and releases its own /BSY (b3) to hand the bus to the target. The
        // ID lands on ODR *after* /SEL is first asserted, so we test the whole
        // condition on every write, not just the /SEL edge: /SEL asserted, /BSY
        // released, and our disk's ID bit on the bus -> the target responds and the
        // bus enters Command phase.
        const bool selecting = (v & 0x04) && !(v & 0x08);
        if (selecting &&
            (phase_ == BusFree || phase_ == Arbitration || phase_ == Selection) &&
            disk.present() && (odr_ & (1u << disk.id())))
            enterCommand();
        // A bus reset (/RST, b7) drops everything back to Bus Free.
        if (v & 0x80) { phase_ = BusFree; xfer_.clear(); xferPos_ = 0; cdbPos_ = 0; }
    }

    u8 readIcr() const {
        // Read bits mirror the initiator-asserted lines (b4-b0); b7 = /RST. AIP (b6)
        // is set while we hold the bus arbitrating; LA (b5) stays 0 (we always win).
        u8 r = static_cast<u8>(icr_ & 0x9F);
        if (phase_ == Arbitration) r |= 0x40;
        return r;
    }

    // Current SCSI Bus Status: the live control lines. /BSY(b6) /REQ(b5) /MSG(b4)
    // /C-D(b3) /I-O(b2) /SEL(b1) mirror the phase; /RST(b7) if the initiator drives it.
    u8 readCsr() const {
        u8 s = 0;
        if (icr_ & 0x80) s |= 0x80;                 // /RST
        if (phase_ != BusFree) s |= 0x40;           // /BSY (a target holds the bus)
        if (reqAsserted()) s |= 0x20;               // /REQ
        const u8 pl = phaseLines();                 // MSG,C-D,I-O for the current phase
        if (pl & 0x04) s |= 0x10;                   // /MSG
        if (pl & 0x02) s |= 0x08;                   // /C-D
        if (pl & 0x01) s |= 0x04;                   // /I-O
        return s;
    }

    // Bus and Status: b6 DRQ, b4 IRQ, b3 Phase Match, b1 /ATN, b0 /ACK.
    u8 readBsr() const {
        u8 s = 0;
        if (reqAsserted() && (mr_ & 0x02)) s |= 0x40;   // DRQ (in DMA mode a byte is ready)
        if (phaseMatch()) s |= 0x08;                    // Phase Match
        if (icr_ & 0x02) s |= 0x02;                     // /ATN
        if (icr_ & 0x10) s |= 0x01;                     // /ACK
        return s;
    }

    // ---- bus phase machine ----------------------------------------------
    void enterCommand() {
        phase_ = Command;
        cdbPos_ = 0;
        cdbLen_ = 6;   // provisional; refined once the opcode byte arrives
        ++diagSelects;
    }

    // The three phase lines (MSG,C-D,I-O) the target drives for the current phase.
    u8 phaseLines() const {
        switch (phase_) {
            case Command: return 0x02;   // C-D
            case DataOut: return 0x00;
            case DataIn:  return 0x01;   // I-O
            case Status:  return 0x03;   // C-D | I-O
            case MsgIn:   return 0x07;   // MSG | C-D | I-O
            default:      return 0x00;
        }
    }

    bool reqAsserted() const {
        // Handshake: the target drops /REQ while the initiator holds /ACK, and
        // re-asserts it once /ACK is released for the next byte. The ROM's per-byte
        // send/receive loop waits on that /REQ-low edge after asserting /ACK; without
        // it the loop spins, and interrupts firing during the spin corrupt the
        // in-flight CDB in low memory.
        if (icr_ & 0x10) return false;   // /ACK asserted
        switch (phase_) {
            case Command: return cdbPos_ < cdbLen_;
            case DataOut: return xferPos_ < xfer_.size();
            case DataIn:  return xferPos_ < xfer_.size();
            case Status:  return status_ready_;
            case MsgIn:   return msg_ready_;
            default:      return false;
        }
    }

    bool phaseMatch() const {
        if (phase_ == BusFree || phase_ == Arbitration || phase_ == Selection)
            return false;
        return (tcr_ & 0x07) == phaseLines();
    }

    // Initiator -> target byte (Command / Data Out), or the pseudo-DMA send path.
    void dataOut(u8 v) {
        odr_ = v;
        if (phase_ == Command) {
            if (cdbPos_ < 12) cdb_[cdbPos_] = v;
            if (cdbPos_ == 0) cdbLen_ = ScsiDisk::cdbLen(v);
            if (++cdbPos_ >= cdbLen_) executeCdb();
        } else if (phase_ == DataOut) {
            if (xferPos_ < xfer_.size()) { xfer_[xferPos_++] = v; ++diagDataOutBytes; }
            if (xferPos_ >= xfer_.size()) finishDataOut();
        }
    }

    // Target -> initiator byte (Data In / Status / Message In), or pseudo-DMA recv.
    u8 dataIn() {
        switch (phase_) {
            case DataIn:
                if (xferPos_ < xfer_.size()) {
                    const u8 b = xfer_[xferPos_++];
                    ++diagDataInBytes;
                    if (xferPos_ >= xfer_.size()) toStatus();
                    return b;
                }
                return 0;
            case Status:
                status_ready_ = false;
                toMsgIn();
                return status_;
            case MsgIn:
                msg_ready_ = false;
                phase_ = BusFree;   // Command Complete -> release the bus
                return 0x00;        // Command Complete message
            default:
                return odr_;        // selection: data bus reflects ODR
        }
    }

    void executeCdb() {
        ++diagCommands;
        std::memcpy(diagLastCdb, cdb_, 12);
        diagLastCdbLen = cdbLen_;
        if (diagCdbHistLen < 16) std::memcpy(diagCdbHist[diagCdbHistLen++], cdb_, 12);
        bool isWrite = false;
        status_ = disk.execute(cdb_, xfer_, isWrite, writeLBA_, writeBlocks_);
        xferPos_ = 0;
        if (status_ != 0x00) {           // CHECK CONDITION -> straight to Status
            toStatus();
        } else if (isWrite) {            // WRITE -> Data Out from the initiator
            writeMode_ = true;
            xfer_.assign(static_cast<std::size_t>(writeBlocks_) * 512u, 0);
            xferPos_ = 0;
            phase_ = DataOut;
        } else if (!xfer_.empty()) {     // READ/INQUIRY/... -> Data In
            phase_ = DataIn;
        } else {                         // no data -> Status
            toStatus();
        }
    }

    void finishDataOut() {
        if (writeMode_) disk.writeData(writeLBA_, xfer_);
        writeMode_ = false;
        toStatus();
    }

    void toStatus() { phase_ = Status; status_ready_ = true; xfer_.clear(); xferPos_ = 0; }
    void toMsgIn()  { phase_ = MsgIn; msg_ready_ = true; }

    // ---- state -----------------------------------------------------------
    u8 odr_ = 0, icr_ = 0, mr_ = 0, tcr_ = 0, ser_ = 0;
    Phase phase_ = BusFree;
    u8  cdb_[12] = {};
    int cdbPos_ = 0, cdbLen_ = 0;
    std::vector<u8> xfer_;
    std::size_t xferPos_ = 0;
    u8  status_ = 0;
    bool status_ready_ = false, msg_ready_ = false;
    bool writeMode_ = false;
    u32 writeLBA_ = 0, writeBlocks_ = 0;
};

} // namespace openmac
