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

#include <cstdio>
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

    // A selection of an absent target times out ~250ms later on real
    // silicon -- long after the manager has finished its issue path and
    // parked in its interrupt-wait. An instant timeout lands mid-issue and
    // the manager's connection bookkeeping records a half-open transaction
    // (its completion path then drains a phantom connection forever). The
    // machine ticks this countdown with CPU time.
    void tick(int cycles) {
        if (timeoutCountdown_ > 0) {
            timeoutCountdown_ -= cycles;
            if (timeoutCountdown_ <= 0) {
                timeoutCountdown_ = 0;
                seqStep_ = 0;
                raise(0x20);   // disconnected: selection timed out
                // A command written while the select was running sat in
                // the command FIFO; it falls to the bottom now (and an
                // initiator command meets a disconnected chip: one clean
                // illegal-command, not a storm).
                if (queuedValid_) {
                    queuedValid_ = false;
                    command_ = queuedCmd_;
                    execute(queuedCmd_);
                }
            }
        }
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
            if (v != lastStatusDiag_) { lastStatusDiag_ = v; diag("SRD status=%02X", v); }
            return v;
        }
        case 0x5: {
            // Reading the interrupt register clears it and (53C90A+) the
            // sticky status error bits. The sequence step SURVIVES -- it is
            // cleared by the next command, and the manager reads it after
            // the interrupt to learn where a select sequence stopped. The
            // terminal-count flag is a counter condition, not an interrupt
            // latch: only reloading the counter clears it.
            const u8 v = intr_;
            intr_ = 0;
            statusInt_ = false;
            updateLines();
            diag("SRD intr=%02X", v);
            return v;
        }
        case 0x6: diag("SRD seq=%02X", seqStep_); return seqStep_;
        case 0x7: {
            // Upper three bits DUPLICATE the sequence step (datasheet
            // Fig. 11) -- the manager judges a select by these mirrored
            // bits, not by reading register 6. During a DMA data-in the
            // count presents the staged chunk as the FIFO's fill level.
            const u8 ss = static_cast<u8>((seqStep_ & 7) << 5);
            if (dmaActive_ && phase_ == kDataIn) {
                u32 avail = static_cast<u32>(data_.size() - dataPos_);
                if (avail > chunkLeft_) avail = chunkLeft_;
                if (avail > kFifoSize) avail = kFifoSize;
                return static_cast<u8>(ss | (avail & 0x1F));
            }
            return static_cast<u8>(ss | (fifoCount() & 0x1F));
        }
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
        case 0x2:
            if (cmdFeed_) {
                // While a command feed is live, a FIFO byte goes straight
                // out to the target -- the manager polls the FIFO flags for
                // empty as its "byte accepted" handshake, so bytes must not
                // linger here.
                selCdb_.push_back(v);
                tryFeedCdb();
            } else {
                fifoPush(v);
            }
            if (onDrq) onDrq(drq());
            break;
        case 0x3:
            // While a select awaits its timeout, non-miscellaneous
            // commands stack in the command FIFO behind it (datasheet:
            // a command is only accepted "when it falls to the bottom of
            // the command FIFO"). Miscellaneous group ($00-$03) runs any
            // time.
            if (timeoutCountdown_ > 0 && (v & 0x70) != 0) {
                queuedCmd_ = v;
                queuedValid_ = true;
                break;
            }
            command_ = v;
            execute(v);
            break;
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
        // A DMA-flagged command feed requests bytes while the counter
        // stands and the FIFO has room (memory-to-device direction).
        if (cmdFeed_ && feedDma_) return counter_ != 0 && fifoCount() < kFifoSize;
        if (!dmaActive_) return false;
        if (phase_ == kDataIn) return dataPos_ < data_.size() && chunkLeft_ != 0;
        if (phase_ == kDataOut) return counter_ != 0;
        return false;
    }

    u8 dma8Read() {
        u8 v = 0xFF;
        if (phase_ == kDataIn && chunkLeft_ > 0 && dataPos_ < data_.size()) {
            v = data_[dataPos_++];
            --chunkLeft_;
            if (chunkLeft_ == 0 && dataPos_ < data_.size()) raise(0x10);
        }
        finishDataIfDone();
        return v;
    }

    u16 dma16Read() {
        u16 v = 0xFFFF;
        if (phase_ == kDataIn && chunkLeft_ > 0 && dataPos_ < data_.size()) {
            const u8 hi = data_[dataPos_++];
            --chunkLeft_;
            u8 lo = 0xFF;
            if (chunkLeft_ > 0 && dataPos_ < data_.size()) {
                lo = data_[dataPos_++];
                --chunkLeft_;
            }
            v = static_cast<u16>((hi << 8) | lo);
            // A chunk boundary mid-transfer interrupts (bus service) so the
            // manager starts the next chunk; the LAST chunk's interrupt
            // comes from the status-phase transition below instead.
            if (chunkLeft_ == 0 && dataPos_ < data_.size()) raise(0x10);
        }
        finishDataIfDone();
        return v;
    }

    void dma16Write(u16 v) {
        if (cmdFeed_) {
            // CDB bytes through the pseudo-DMA port feed the same live
            // command sequence the FIFO path does; the counter frames them
            // (the manager's 5+1 trick puts exactly the last CDB byte
            // here with tcount=1).
            selCdb_.push_back(static_cast<u8>(v >> 8));
            counterConsume(1);
            if (counter_ != 0) {
                selCdb_.push_back(static_cast<u8>(v & 0xFF));
                counterConsume(1);
            }
            tryFeedCdb();
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
    std::function<void(const char* msg)> onDiag;

private:
    static constexpr u32 kFifoSize = 16;

    void diag(const char* fmt, u32 a) {
        if (!onDiag || diagBudget_ <= 0) return;
        --diagBudget_;
        char line[64];
        std::snprintf(line, sizeof(line), fmt, a);
        onDiag(line);
    }

    // Structural events (selects, CDB completion) keep their own budget so
    // a polling storm cannot drown them out of the log.
    void diagEvent(const char* fmt, u32 a) {
        if (!onDiag || eventBudget_ <= 0) return;
        --eventBudget_;
        char line[64];
        std::snprintf(line, sizeof(line), fmt, a);
        onDiag(line);
    }

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
        chunkLeft_ = 0;
        writeBuf_.clear();
        writeExpect_ = 0;
        selected_ = nullptr;
        cmdFeed_ = false;
        feedIsSelect_ = false;
        selAtn_ = false;
        feedDma_ = false;
        selCdb_.clear();
        timeoutCountdown_ = 0;
        queuedValid_ = false;
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
            // A DMA data-in chunk can be drained through the FIFO register
            // just as through the pseudo-DMA port -- the old-API manager
            // polls bytes out of reg 2. Serve from the staged chunk with
            // the same boundary interrupt the port path raises.
            if (phase_ == kDataIn && dmaActive_ && chunkLeft_ > 0 &&
                dataPos_ < data_.size()) {
                const u8 v = data_[dataPos_++];
                --chunkLeft_;
                if (chunkLeft_ == 0 && dataPos_ < data_.size()) raise(0x10);
                finishDataIfDone();
                return v;
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

    // A data phase drains; move to status once it has. The phase change
    // waits for the FIFO to empty -- the last received byte must be read
    // out while the phase still says data-in, and only the FOLLOWING
    // transfer-info sees the target's move to status.
    void finishDataIfDone() {
        if (phase_ == kDataIn && dataPos_ >= data_.size() && fifoCount() == 0) {
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

    // The 53C9x select sequence, as the Quadra ROM family drives it (and as
    // MAME's ncr53c90 models it -- the undocumented empty-FIFO wait is the
    // step that "makes macqd700 happy"):
    //   - selection of an absent target times out: disconnected interrupt,
    //     step 0, NO other activity;
    //   - selection of a present target raises NO interrupt: the chip
    //     parks at step 1 in the command phase (DRQ up if DMA-flagged) and
    //     sends command bytes to the target as the manager supplies them.
    //     This manager bulk-writes the leading CDB bytes into the FIFO --
    //     polling the FIFO flags down to zero as its "bytes went out"
    //     handshake -- and delivers exactly the LAST byte through the
    //     pseudo-DMA port, framed by the select's tcount=1 (the 5+1 trick);
    //   - once the target has a whole CDB it leaves the command phase, and
    //     only THEN does the chip interrupt: function complete + bus
    //     service, step 4 when the counter drained and the FIFO emptied,
    //     step 2 otherwise (tcount=1 is still standing here: step 2).
    // The same feed serves a transfer-info issued in the command phase (the
    // manager's non-select command path uses the identical bulk+DMA-tail
    // shape), completing with bus service alone.
    void beginSelect(bool withAtn, bool dma) {
        ++diagSelects;
        seqStep_ = 0;
        cmdFeed_ = false;
        selCdb_.clear();
        // A DMA-flagged command loads the transfer count the moment it
        // issues -- present target or not. The manager reads TC0 after a
        // timed-out select, and a stale terminal count from the previous
        // transaction reads as a dirty bus (it pad-flushes forever).
        if (dma) loadCounter();
        selected_ = findTarget(destId_);
        if (!selected_) {
            // Selection timeout: the disconnected interrupt arrives after
            // the real chip's ~250ms wait (modeled shorter), never
            // synchronously inside the command write.
            timeoutCountdown_ = 50000;
            diagEvent("SEL id=%02X timeout pending", destId_);
            return;
        }
        timeoutCountdown_ = 0;
        cmdFeed_ = true;
        feedIsSelect_ = true;
        selAtn_ = withAtn;
        feedDma_ = dma;
        phase_ = kCommand;
        seqStep_ = 1;
        diagEvent("SEL id=%02X pending", destId_);
        // The documented flow pre-loads the CDB before the command; treat
        // anything already in the FIFO as the first bytes sent.
        while (fifoCount() > 0) selCdb_.push_back(fifoPop());
        tryFeedCdb();
        updateLines();
    }

    static int cdbLenFor(u8 op) {
        switch (op >> 5) {
        case 0: return 6;
        case 1: case 2: return 10;
        case 5: return 12;
        default: return 1;   // vendor/reserved: our targets reject the
                             // opcode byte itself (ends a pad-byte abort)
        }
    }

    void tryFeedCdb() {
        if (!cmdFeed_) return;
        const u32 skip = (feedIsSelect_ && selAtn_) ? 1u : 0u;   // IDENTIFY
        if (selCdb_.size() <= skip) return;
        const u32 need = skip + static_cast<u32>(cdbLenFor(selCdb_[skip]));
        if (selCdb_.size() < need) {
            if (feedIsSelect_ && seqStep_ < 3) seqStep_ = 3;   // bytes flowing
            return;
        }
        cmdFeed_ = false;
        runCdb(selCdb_.data() + skip, static_cast<int>(need - skip));
        if (feedIsSelect_) {
            seqStep_ = ((!feedDma_ || tcZero_) && fifoCount() == 0) ? 4 : 2;
            diagEvent("SEL cdb done seq=%02X", seqStep_);
            raise(0x18);                      // function complete + bus service
        } else {
            diagEvent("XFER cdb done phase=%02X", phase_);
            raise(0x10);                      // bus service: phase changed
        }
        selCdb_.clear();
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
                // Initiator commands are only legal while connected: the
                // command is ignored, an illegal-command interrupt is
                // generated and the command register CLEARS (datasheet
                // p.32 -- the manager's cleanup reads it back).
                command_ = 0;
                raise(0x40);
                break;
            }
            if (dma) loadCounter();
            dmaActive_ = dma;
            if (phase_ == kCommand) {
                // CDB arriving via transfer info rather than the select
                // sequence: same live feed, FIFO first, DMA tail.
                dmaActive_ = false;
                cmdFeed_ = true;
                feedIsSelect_ = false;
                feedDma_ = dma;
                selCdb_.clear();
                while (fifoCount() > 0) selCdb_.push_back(fifoPop());
                tryFeedCdb();
            } else if (phase_ == kDataIn) {
                if (dma) {
                    // The chip leads the target: the whole chunk is fetched
                    // as soon as the command issues, so terminal count
                    // reports done BEFORE the host drains the bytes -- the
                    // manager polls TC, then bursts the FIFO's worth out
                    // through the pseudo-DMA port.
                    chunkLeft_ = counter_;
                    counter_ = 0;
                    tcZero_ = true;
                } else if (dataPos_ < data_.size()) {
                    // Non-DMA receive: exactly ONE byte lands in the FIFO
                    // and every byte interrupts (bus service). The phase
                    // flips to status only once the last byte is consumed,
                    // so the next transfer-info reports the change.
                    fifoPush(data_[dataPos_++]);
                    raise(0x10);
                }
                finishDataIfDone();
            } else if (phase_ == kDataOut) {
                if (!dma) {
                    while (fifoCount() > 0 && writeBuf_.size() < writeExpect_)
                        writeBuf_.push_back(fifoPop());
                    // FIFO emptied with more expected: the target REQs the
                    // next byte -- transfer complete, bus service.
                    if (writeBuf_.size() < writeExpect_) raise(0x10);
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
            if (!selected_) { command_ = 0; raise(0x40); break; }
            cmdFeed_ = false;
            fifoClear();
            fifoPush(scsiStatus_);
            fifoPush(0x00);       // COMMAND COMPLETE message
            phase_ = kMsgIn;
            raise(0x08);          // function complete
            break;
        case 0x12:   // message accepted
            if (!selected_) { command_ = 0; raise(0x40); break; }
            cmdFeed_ = false;
            phase_ = kDataOut;
            selected_ = nullptr;
            raise(0x20);          // disconnected
            break;
        case 0x18:   // transfer pad
            if (!selected_) { command_ = 0; raise(0x40); break; }
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
    bool cmdFeed_ = false;
    bool feedIsSelect_ = false;
    bool selAtn_ = false;
    bool feedDma_ = false;
    std::vector<u8> selCdb_;
    u8 lastStatusDiag_ = 0xFF;
    int timeoutCountdown_ = 0;
    u8 queuedCmd_ = 0;
    bool queuedValid_ = false;

    std::vector<u8> data_;
    size_t dataPos_ = 0;
    u32 chunkLeft_ = 0;
    std::vector<u8> writeBuf_;
    u32 writeExpect_ = 0;
    u8 scsiStatus_ = 0;
    int diagBudget_ = 800;
    int eventBudget_ = 4000;
};

} // namespace openmac
