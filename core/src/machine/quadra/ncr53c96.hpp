#pragma once

// NCR 53C96 ("ESP" family, 53C94-level register map) as the Quadra 650's
// SCSI controller, driving the same ScsiTarget devices the Classic's 5380
// does. Initiator-only: the Mac is always the initiator. Transfers complete
// instantly at the target layer (targets answer whole CDBs), so this models
// the register file, the selection/phase sequencing, the FIFO, the transfer
// counter, and the IRQ/DRQ lines the ROM's driver actually watches.
//
// Register map (offset: read / write):
//   0 counter lo / count lo      6 sequence step / sync period
//   1 counter hi / count hi      7 FIFO flags / sync offset
//   2 FIFO      / FIFO           8 config 1
//   3 command   / command        9 - / clock factor   A - / test
//   4 status    / dest bus ID    B config 2   C config 3   F - / alignment
//   5 interrupt / select timeout
//
// Commands used by the Mac driver: NOP 00, FLUSH 01, RESET CHIP 02, RESET
// BUS 03, TRANSFER INFO 10, ICCS 11, MSG ACCEPT 12, PAD 18, SET/RESET ATN
// 1A/1B, SELECT 41 / SELECT-ATN 42 (+80 = DMA). Reading the interrupt
// register clears it (and the 53C90A+ error status bits).
//
// Reference: Quadra 650 hardware dossier (53C96 section, from the NCR
// 53C90A/B data book facts). Clean-room.

#include "../scsi.hpp"
#include "openmac/types.hpp"

#include <functional>
#include <vector>

namespace openmac {

class Ncr53c96 {
public:
    enum Phase : u8 {
        kDataOut = 0, kDataIn = 1, kCommand = 2, kStatus = 3,
        kMsgOut = 6, kMsgIn = 7,
    };

    void addTarget(ScsiTarget* t) {
        for (auto& slot : targets_) {
            if (!slot) { slot = t; return; }
        }
    }

    void reset() {
        resetChip();
        intr_ = 0;
        statusInt_ = false;
        updateLines();
    }

    // ---- register file ($50010000 + reg) ----
    u8 read(int reg) {
        switch (reg & 0xF) {
        case 0x0: return static_cast<u8>(counter_ & 0xFF);
        case 0x1: return static_cast<u8>(counter_ >> 8);
        case 0x2: return fifoPop();
        case 0x3: return command_;
        case 0x4: {
            u8 v = phase_;
            if (tcZero_) v |= 0x10;
            if (statusInt_) v |= 0x80;
            return v;
        }
        case 0x5: {
            // Reading the interrupt register clears it and (53C90A+) the
            // sticky status error bits. The sequence step SURVIVES -- it is
            // cleared by the next command, and the manager reads it after
            // the interrupt to learn where a select sequence stopped.
            const u8 v = intr_;
            intr_ = 0;
            statusInt_ = false;
            tcZero_ = false;
            updateLines();
            return v;
        }
        case 0x6: return seqStep_;
        case 0x7: return static_cast<u8>(fifoCount() & 0x1F);
        case 0x8: return cfg1_;
        case 0xB: return cfg2_;
        case 0xC: return cfg3_;
        default:  return 0;
        }
    }

    void write(int reg, u8 v) {
        ++diagWrites;
        switch (reg & 0xF) {
        case 0x0: tcountLo_ = v; break;
        case 0x1: tcountHi_ = v; break;
        case 0x2: fifoPush(v); break;
        case 0x3: command_ = v; execute(v); break;
        case 0x4: destId_ = v & 7; break;
        case 0x5: selTimeout_ = v; break;
        case 0x6: syncPer_ = v; break;
        case 0x7: syncOff_ = v; break;
        case 0x8: cfg1_ = v; break;
        case 0x9: clkFactor_ = v; break;
        case 0xA: test_ = v; break;
        case 0xB: cfg2_ = v; break;
        case 0xC: cfg3_ = v; break;
        case 0xF: align_ = v; break;
        default: break;
        }
    }

    // ---- pseudo-DMA data path (IOSB pulls 16-bit halves) ----
    bool drq() const {
        if (dmaSelect_) return counter_ != 0;   // the select pulls its CDB by DMA
        if (!dmaActive_) return false;
        if (phase_ == kDataIn) return dataPos_ < data_.size() && counter_ != 0;
        if (phase_ == kDataOut) return counter_ != 0;
        return false;
    }

    u16 dma16Read() {
        u16 v = 0xFFFF;
        if (phase_ == kDataIn && dataPos_ < data_.size()) {
            const u8 hi = data_[dataPos_++];
            const u8 lo = dataPos_ < data_.size() ? data_[dataPos_++] : 0xFF;
            v = static_cast<u16>((hi << 8) | lo);
            counterConsume(2);
        }
        finishDataIfDone();
        return v;
    }

    void dma16Write(u16 v) {
        if (dmaSelect_) {
            // The select sequence's message/CDB bytes arrive by DMA.
            selBuf_.push_back(static_cast<u8>(v >> 8));
            counterConsume(1);
            if (counter_ != 0) {
                selBuf_.push_back(static_cast<u8>(v & 0xFF));
                counterConsume(1);
            }
            if (counter_ == 0) completeDmaSelect();
            if (onDrq) onDrq(drq());
            return;
        }
        if (phase_ == kDataOut) {
            writeBuf_.push_back(static_cast<u8>(v >> 8));
            if (writeBuf_.size() < writeExpect_)
                writeBuf_.push_back(static_cast<u8>(v & 0xFF));
            counterConsume(2);
        }
        finishDataIfDone();
    }

    bool irqAsserted() const { return statusInt_; }
    std::function<void(bool level)> onIrq;
    std::function<void(bool level)> onDrq;

    // Diagnostics for the bring-up trace.
    u32 diagWrites = 0, diagSelects = 0, diagCommands = 0;
    u8  lastCdb[12]{};
    int lastCdbLen = 0;
    std::function<void(int id, const u8* cdb, int len)> onCdb;
    std::function<void(u8 cmd, u32 fifoLevel, u8 phase)> onCmd;

private:
    static constexpr u32 kFifoSize = 16;

    void resetChip() {
        fifoHead_ = fifoTail_ = 0;
        tcountLo_ = tcountHi_ = 0;
        counter_ = 0;
        phase_ = kDataOut;
        seqStep_ = 0;
        command_ = 0;
        dmaActive_ = false;
        tcZero_ = false;
        data_.clear();
        dataPos_ = 0;
        writeBuf_.clear();
        writeExpect_ = 0;
        selected_ = nullptr;
        dmaSelect_ = false;
        selBuf_.clear();
    }

    ScsiTarget* findTarget(int id) {
        for (auto* t : targets_) {
            if (t && t->present() && t->id() == id) return t;
        }
        return nullptr;
    }

    u32 fifoCount() const { return (fifoTail_ - fifoHead_) & 31; }
    void fifoPush(u8 v) {
        if (fifoCount() < kFifoSize) fifo_[fifoTail_++ & 31] = v;
    }
    u8 fifoPop() {
        if (fifoCount() == 0) {
            // An empty FIFO refills from an in-progress polled data-in.
            if (phase_ == kDataIn && dataPos_ < data_.size()) {
                fillFifoFromData();
                finishDataIfDone();
            }
            if (fifoCount() == 0) return 0xFF;
        }
        return fifo_[fifoHead_++ & 31];
    }
    void fifoClear() { fifoHead_ = fifoTail_ = 0; }

    void fillFifoFromData() {
        // Non-DMA transfer info moves bytes by REQ/ACK through the FIFO --
        // the transfer counter only governs DMA transfers.
        while (fifoCount() < kFifoSize && dataPos_ < data_.size() &&
               (!dmaActive_ || counter_ != 0)) {
            fifoPush(data_[dataPos_++]);
            if (dmaActive_) counterConsume(1);
        }
    }

    void counterConsume(u32 n) {
        if (counter_ >= n) counter_ -= n;
        else counter_ = 0;
        if (counter_ == 0) tcZero_ = true;
    }

    void loadCounter() {
        counter_ = static_cast<u16>(tcountLo_ | (tcountHi_ << 8));
        if (counter_ == 0) counter_ = 0; // 0 means 65536 on real silicon for
                                         // DMA; the Mac driver never uses it.
        tcZero_ = false;
    }

    void raise(u8 bits) {
        intr_ |= bits;
        statusInt_ = true;
        updateLines();
    }

    void updateLines() {
        if (onIrq) onIrq(statusInt_);
        if (onDrq) onDrq(drq());
    }

    // A data phase drains; move to status once it has.
    void finishDataIfDone() {
        if (phase_ == kDataIn && dataPos_ >= data_.size()) {
            dmaActive_ = false;
            phase_ = kStatus;
            raise(0x10);   // bus service: phase change
        } else if (phase_ == kDataOut && writeBuf_.size() >= writeExpect_ && writeExpect_ > 0) {
            if (selected_) selected_->acceptWrite(writeBuf_);
            writeBuf_.clear();
            writeExpect_ = 0;
            dmaActive_ = false;
            phase_ = kStatus;
            raise(0x10);
        } else {
            if (onDrq) onDrq(drq());
        }
    }

    void completeDmaSelect() {
        dmaSelect_ = false;
        size_t idx = 0;
        if (dmaSelectAtn_ && idx < selBuf_.size()) ++idx;   // IDENTIFY message
        std::vector<u8> cdb(selBuf_.begin() + static_cast<long>(idx), selBuf_.end());
        if (!cdb.empty() && selected_) {
            runCdb(cdb.data(), static_cast<int>(cdb.size()));
            seqStep_ = 4;
            raise(0x18);
        } else {
            phase_ = kCommand;
            seqStep_ = 3;
            raise(0x10);
        }
    }

    void beginSelect(bool withAtn, bool dma) {
        ++diagSelects;
        selected_ = findTarget(destId_);
        if (!selected_) {
            // Selection timeout: disconnected.
            raise(0x20);
            seqStep_ = 0;
            return;
        }
        if (dma) {
            // This manager's DMA-flagged select is a bare selection stage:
            // it reads the sequence step afterwards to confirm the target
            // answered, then sends the CDB itself with a transfer-info in
            // the command phase. Report "stopped before command bytes".
            loadCounter();
            phase_ = kCommand;
            seqStep_ = 3;
            raise(0x10);
            return;
        }
        fifoBytes_.clear();
        while (fifoCount() > 0) fifoBytes_.push_back(fifoPop());
        size_t idx = 0;
        if (withAtn && idx < fifoBytes_.size()) ++idx;   // IDENTIFY message
        // The rest of the FIFO is the CDB the select sequence sends.
        lastCdbLen = 0;
        std::vector<u8> cdb(fifoBytes_.begin() + static_cast<long>(idx), fifoBytes_.end());
        if (!cdb.empty()) {
            runCdb(cdb.data(), static_cast<int>(cdb.size()));
            seqStep_ = 4;   // completed through the command phase
            raise(0x18);    // function complete + bus service
        } else {
            // Selection succeeded but the sequence stopped before any
            // command bytes went out (the old-API manager's manual flow):
            // bus service only, step 3.
            phase_ = kCommand;
            seqStep_ = 3;
            raise(0x10);
        }
    }

    void runCdb(const u8* cdb, int len) {
        ++diagCommands;
        lastCdbLen = len < 12 ? len : 12;
        for (int i = 0; i < lastCdbLen; ++i) lastCdb[i] = cdb[i];
        if (onCdb) onCdb(destId_, cdb, lastCdbLen);
        data_.clear();
        dataPos_ = 0;
        writeExpect_ = 0;
        scsiStatus_ = selected_->execute(cdb, data_, writeExpect_);
        writeBuf_.clear();
        if (writeExpect_ > 0) phase_ = kDataOut;
        else if (!data_.empty()) phase_ = kDataIn;
        else phase_ = kStatus;
    }

    void execute(u8 cmd) {
        if (onCmd) onCmd(cmd, fifoCount(), phase_);
        const bool dma = (cmd & 0x80) != 0;
        switch (cmd & 0x7F) {
        case 0x00:   // NOP
            if (dma) loadCounter();
            break;
        case 0x01:   // flush FIFO
            fifoClear();
            break;
        case 0x02:   // reset chip
            resetChip();
            break;
        case 0x03:   // reset SCSI bus
            resetChip();
            if (!(cfg1_ & 0x40)) raise(0x80);
            break;
        case 0x10: { // transfer information
            if (!selected_) {
                // Initiator commands are only legal while connected; the
                // chip answers with an illegal-command interrupt (the ROM's
                // startup test deliberately provokes exactly this).
                raise(0x40);
                break;
            }
            if (dma) loadCounter();
            dmaActive_ = dma;
            if (phase_ == kCommand) {
                // CDB arriving via transfer info rather than the select
                // sequence: consume the FIFO as the command.
                std::vector<u8> cdb;
                while (fifoCount() > 0) cdb.push_back(fifoPop());
                if (selected_ && !cdb.empty()) {
                    runCdb(cdb.data(), static_cast<int>(cdb.size()));
                    raise(0x10);
                }
            } else if (phase_ == kDataIn) {
                if (!dma) fillFifoFromData();
                finishDataIfDone();
            } else if (phase_ == kDataOut) {
                if (!dma) {
                    while (fifoCount() > 0 && writeBuf_.size() < writeExpect_)
                        writeBuf_.push_back(fifoPop());
                }
                finishDataIfDone();
            } else if (phase_ == kStatus) {
                raise(0x10);
            } else if (phase_ == kMsgIn) {
                fifoPush(0x00);   // COMMAND COMPLETE
                raise(0x10);
            }
            updateLines();
            break;
        }
        case 0x11:   // initiator command complete sequence
            if (!selected_) { raise(0x40); break; }
            fifoClear();
            fifoPush(scsiStatus_);
            fifoPush(0x00);       // COMMAND COMPLETE message
            phase_ = kMsgIn;
            raise(0x08);          // function complete
            break;
        case 0x12:   // message accepted
            if (!selected_) { raise(0x40); break; }
            phase_ = kDataOut;
            selected_ = nullptr;
            raise(0x20);          // disconnected
            break;
        case 0x18:   // transfer pad
            if (!selected_) { raise(0x40); break; }
            if (dma) loadCounter();
            if (phase_ == kDataIn) { dataPos_ = data_.size(); }
            finishDataIfDone();
            break;
        case 0x1A: case 0x1B:     // set/reset ATN: nothing observable here
            break;
        case 0x41: beginSelect(false, dma); break;
        case 0x42: beginSelect(true, dma); break;
        case 0x44: case 0x45:     // enable/disable selection (target role)
            break;
        case 0x46: beginSelect(true, dma); break;   // select with ATN3
        default:
            raise(0x40);          // illegal command
            break;
        }
    }

    ScsiTarget* targets_[8]{};
    ScsiTarget* selected_ = nullptr;

    u8 fifo_[32]{};
    u32 fifoHead_ = 0, fifoTail_ = 0;
    std::vector<u8> fifoBytes_;

    u8 tcountLo_ = 0, tcountHi_ = 0;
    u32 counter_ = 0;
    bool tcZero_ = false;
    u8 command_ = 0;
    u8 destId_ = 0, selTimeout_ = 0, syncPer_ = 0, syncOff_ = 0;
    u8 cfg1_ = 0, cfg2_ = 0, cfg3_ = 0, clkFactor_ = 0, test_ = 0, align_ = 0;
    u8 phase_ = kDataOut;
    u8 seqStep_ = 0;
    u8 intr_ = 0;
    bool statusInt_ = false;
    bool dmaActive_ = false;
    bool dmaSelect_ = false;
    bool dmaSelectAtn_ = false;
    std::vector<u8> selBuf_;

    std::vector<u8> data_;
    size_t dataPos_ = 0;
    std::vector<u8> writeBuf_;
    u32 writeExpect_ = 0;
    u8 scsiStatus_ = 0;
};

} // namespace openmac
