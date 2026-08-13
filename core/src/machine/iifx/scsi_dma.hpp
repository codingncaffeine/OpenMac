#pragma once

// Apple 343S0064-A custom SCSI DMA controller used by the Macintosh IIfx.
//
// The ASIC contains an enhanced 53C80 plus a 32-bit, four-byte FIFO and DMA
// address/count/watchdog registers.  Its programmer-visible map is documented
// in Apple drawing 343S0064-A, pp. 24-31:
//
//   $000-$070  eight byte-wide 53C80 registers, 16-byte spaced
//   $080       DMA control (32 bit)
//   $0C0       byte count (32 bit)
//   $100       DMA address (32 bit)
//   $140       watchdog reload value (32 bit)
//   $180       FIFO (32 bit; writable only in loopback test mode)
//
// The device is clocked from the 40 MHz processor clock.  Its documented
// asynchronous SCSI ceiling is 3 MB/s, represented here as three byte slots
// per forty CPU clocks; the watchdog decrements at CPUCLK/2.

#include "../scsi.hpp"

#include <array>
#include <functional>

namespace openmac {

class IifxStateCodec;

class IifxScsiDma {
public:
    static constexpr u32 Control = 0x080;
    static constexpr u32 Count = 0x0C0;
    static constexpr u32 Address = 0x100;
    static constexpr u32 Watchdog = 0x140;
    static constexpr u32 Fifo = 0x180;

    explicit IifxScsiDma(Ncr5380& ncr) : ncr_(ncr) {}

    // Physical-memory callbacks are supplied by the IIfx bus.  A failed
    // callback is represented by an exception and becomes the ASIC's latched
    // DMA-bus-error status instead of escaping into the emulated CPU.
    std::function<u8(u32)> readMemory;
    std::function<void(u32, u8)> writeMemory;
    std::function<void()> onInterruptChange;

    void reset() {
        ncr_.reset();
        controlWrite_ = 0;
        count_ = 0;
        address_ = 0;
        watchdogReload_ = 0;
        watchdogCounter_ = 0;
        fifo_.fill(0);
        fifoValid_ = 0;
        dmaActive_ = false;
        direction_ = Direction::None;
        watchdogPending_ = false;
        busError_ = false;
        wonArbitration_ = false;
        resetCommandHigh_ = false;
        transferPhase_ = 0;
        watchdogPhase_ = 0;
        lastInterrupt_ = false;
        notifyInterrupt();
    }

    u8 read8(u32 offset) {
        offset &= 0x1FFu;
        u8 value = 0xFF;
        if (offset < Control) {
            value = ncr_.read(static_cast<int>(offset >> 4));
        } else if (inLong(offset, Control)) {
            value = byteOf(controlValue(), offset - Control);
        } else if (inLong(offset, Count)) {
            value = byteOf(count_, offset - Count);
        } else if (inLong(offset, Address)) {
            value = byteOf(address_, offset - Address);
        } else if (inLong(offset, Watchdog)) {
            value = byteOf(watchdogReload_, offset - Watchdog);
            clearWatchdogInterrupt();
        } else if (inLong(offset, Fifo)) {
            value = fifo_[offset - Fifo];
        }
        notifyInterrupt();
        return value;
    }

    void write8(u32 offset, u8 value) {
        offset &= 0x1FFu;
        if (offset < Control) {
            const int reg = static_cast<int>(offset >> 4);
            ncr_.write(reg, value);
            if (reg >= 5) beginTransfer(reg);
        } else if (inLong(offset, Control)) {
            setByte(controlWrite_, offset - Control, value);
            writeControl(controlWrite_);
        } else if (inLong(offset, Count)) {
            setByte(count_, offset - Count, value);
        } else if (inLong(offset, Address)) {
            setByte(address_, offset - Address, value);
        } else if (inLong(offset, Watchdog)) {
            setByte(watchdogReload_, offset - Watchdog, value);
            loadWatchdog();
        } else if (inLong(offset, Fifo) && testMode()) {
            fifo_[offset - Fifo] = value;
            fifoValid_ |= static_cast<u8>(1u << (offset - Fifo));
        }
        notifyInterrupt();
    }

    u32 read32(u32 offset) {
        offset &= 0x1FFu;
        u32 value = 0xFFFFFFFFu;
        switch (offset) {
        case Control: value = controlValue(); break;
        case Count: value = count_; break;
        case Address: value = address_; break;
        case Watchdog:
            value = watchdogReload_;
            clearWatchdogInterrupt();
            break;
        case Fifo:
            value = fifoWord();
            break;
        default:
            value = (static_cast<u32>(read8(offset)) << 24) |
                    (static_cast<u32>(read8(offset + 1)) << 16) |
                    (static_cast<u32>(read8(offset + 2)) << 8) |
                    read8(offset + 3);
            break;
        }
        notifyInterrupt();
        return value;
    }

    void write32(u32 offset, u32 value) {
        offset &= 0x1FFu;
        switch (offset) {
        case Control:
            controlWrite_ = value;
            writeControl(value);
            break;
        case Count:
            count_ = value;
            break;
        case Address:
            address_ = value;
            break;
        case Watchdog:
            watchdogReload_ = value;
            loadWatchdog();
            break;
        case Fifo:
            if (testMode()) {
                fifo_[0] = static_cast<u8>(value >> 24);
                fifo_[1] = static_cast<u8>(value >> 16);
                fifo_[2] = static_cast<u8>(value >> 8);
                fifo_[3] = static_cast<u8>(value);
                fifoValid_ = 0x0F;
            }
            break;
        default:
            write8(offset, static_cast<u8>(value >> 24));
            write8(offset + 1, static_cast<u8>(value >> 16));
            write8(offset + 2, static_cast<u8>(value >> 8));
            write8(offset + 3, static_cast<u8>(value));
            break;
        }
        notifyInterrupt();
    }

    void tick(int cpuCycles) {
        if (cpuCycles <= 0) return;

        // The watchdog is a free-running CPUCLK/2 down-counter once loaded.
        if (watchdogCounter_ != 0) {
            const u64 clocks = watchdogPhase_ + static_cast<u64>(cpuCycles);
            const u64 decrements = clocks / 2u;
            watchdogPhase_ = clocks & 1u;
            if (decrements >= watchdogCounter_) {
                watchdogCounter_ = 0;
                watchdogPending_ = true;
                dmaActive_ = false;
            } else {
                watchdogCounter_ -= static_cast<u32>(decrements);
            }
        }

        if (dmaActive_ && dmaEnabled()) {
            transferPhase_ += static_cast<u64>(cpuCycles) * 3u;
            while (dmaActive_ && transferPhase_ >= 40u) {
                transferPhase_ -= 40u;
                transferByte();
            }
        }
        notifyInterrupt();
    }

    bool irqAsserted() const {
        return ((controlWrite_ & ScsiInterruptEnable) && ncr_.irqAsserted()) ||
               ((controlWrite_ & WatchdogInterruptEnable) && watchdogPending_);
    }

    bool drqAsserted() const { return ncr_.dmaRequestAsserted(); }
    bool dmaActive() const { return dmaActive_; }
    u32 control() const { return controlValue(); }
    u32 count() const { return count_; }
    u32 address() const { return address_; }
    u32 watchdogCounter() const { return watchdogCounter_; }

private:
    friend class IifxStateCodec;

    enum class Direction { None, ToScsi, FromScsi };

    static constexpr u32 DmaEnable = 1u << 0;
    static constexpr u32 ScsiInterruptEnable = 1u << 1;
    static constexpr u32 WatchdogInterruptEnable = 1u << 2;
    static constexpr u32 HardwareHandshake = 1u << 3;
    static constexpr u32 Reset53c80 = 1u << 4;
    static constexpr u32 TestMode = 1u << 5;
    static constexpr u32 ScsiInterruptPending = 1u << 6;
    static constexpr u32 WatchdogInterruptPending = 1u << 7;
    static constexpr u32 DmaBusError = 1u << 8;
    static constexpr u32 ArbitrationEnable = 1u << 12;
    static constexpr u32 WonArbitration = 1u << 13;
    static constexpr u32 WritableMask = 0x00001E3Fu;

    static bool inLong(u32 offset, u32 base) {
        return offset >= base && offset < base + 4u;
    }

    static u8 byteOf(u32 value, u32 lane) {
        return static_cast<u8>(value >> ((3u - lane) * 8u));
    }

    static void setByte(u32& word, u32 lane, u8 value) {
        const u32 shift = (3u - lane) * 8u;
        word = (word & ~(0xFFu << shift)) | (static_cast<u32>(value) << shift);
    }

    bool dmaEnabled() const { return (controlWrite_ & DmaEnable) != 0; }
    bool hardwareHandshake() const {
        return (controlWrite_ & HardwareHandshake) != 0;
    }
    bool testMode() const { return (controlWrite_ & TestMode) != 0; }

    u32 fifoWord() const {
        return (static_cast<u32>(fifo_[0]) << 24) |
               (static_cast<u32>(fifo_[1]) << 16) |
               (static_cast<u32>(fifo_[2]) << 8) |
               fifo_[3];
    }

    u32 controlValue() const {
        u32 value = controlWrite_ & WritableMask;
        value &= ~Reset53c80;       // bit 4 is bytes-left when read
        if (fifoValid_) value |= Reset53c80;
        if (ncr_.irqAsserted()) value |= ScsiInterruptPending;
        if (watchdogPending_) value |= WatchdogInterruptPending;
        if (busError_) value |= DmaBusError;
        if (wonArbitration_) value |= WonArbitration;
        return value;
    }

    void writeControl(u32 value) {
        const bool resetHigh = (value & Reset53c80) != 0;
        controlWrite_ = value & WritableMask;
        if (resetHigh && !resetCommandHigh_) {
            ncr_.reset();
            dmaActive_ = false;
            direction_ = Direction::None;
            fifoValid_ = 0;
        }
        resetCommandHigh_ = resetHigh;

        if (!dmaEnabled()) dmaActive_ = false;
        if (!(controlWrite_ & ArbitrationEnable)) wonArbitration_ = false;
    }

    void beginTransfer(int startRegister) {
        if (!dmaEnabled() || hardwareHandshake() || count_ == 0) return;
        if (startRegister == 5)
            direction_ = Direction::ToScsi;
        else if (startRegister == 6 || startRegister == 7)
            direction_ = Direction::FromScsi;
        else
            return;
        dmaActive_ = true;
        busError_ = false;
        fifoValid_ = 0;
        transferPhase_ = 0;
    }

    void transferByte() {
        if (count_ == 0) {
            finishDma();
            return;
        }
        if (!ncr_.dmaRequestAsserted() || !ncr_.phaseMatches()) {
            if (ncr_.irqAsserted()) dmaActive_ = false;
            return;
        }

        try {
            const u32 current = address_;
            const unsigned lane = current & 3u;
            if (direction_ == Direction::ToScsi) {
                if (!readMemory) { busError_ = true; dmaActive_ = false; return; }
                fifo_[lane] = readMemory(current);
                fifoValid_ |= static_cast<u8>(1u << lane);
                ncr_.write(0, fifo_[lane]);
                fifoValid_ &= static_cast<u8>(~(1u << lane));
            } else if (direction_ == Direction::FromScsi) {
                fifo_[lane] = ncr_.read(6);
                fifoValid_ |= static_cast<u8>(1u << lane);
                // A filled high lane, or the final byte, commits the assembled
                // FIFO lanes to memory as one emulated DMA bus transaction.
                if (lane == 3u || count_ == 1u) flushReceiveFifo(current);
            }
            ++address_;
            --count_;
            watchdogCounter_ = watchdogReload_;
            watchdogPhase_ = 0;
            if (count_ == 0) finishDma();
        } catch (...) {
            busError_ = true;
            dmaActive_ = false;
            direction_ = Direction::None;
        }
    }

    void flushReceiveFifo(u32 currentAddress) {
        if (!writeMemory) { busError_ = true; dmaActive_ = false; return; }
        const u32 base = currentAddress & ~3u;
        for (unsigned lane = 0; lane < 4; ++lane) {
            if (fifoValid_ & (1u << lane))
                writeMemory(base + lane, fifo_[lane]);
        }
        fifoValid_ = 0;
    }

    void finishDma() {
        dmaActive_ = false;
        direction_ = Direction::None;
        ncr_.signalEndOfProcess();
    }

    void loadWatchdog() {
        watchdogCounter_ = watchdogReload_;
        watchdogPhase_ = 0;
        clearWatchdogInterrupt();
    }

    void clearWatchdogInterrupt() {
        watchdogPending_ = false;
    }

    void notifyInterrupt() {
        const bool level = irqAsserted();
        if (level == lastInterrupt_) return;
        lastInterrupt_ = level;
        if (onInterruptChange) onInterruptChange();
    }

    Ncr5380& ncr_;
    u32 controlWrite_ = 0;
    u32 count_ = 0;
    u32 address_ = 0;
    u32 watchdogReload_ = 0;
    u32 watchdogCounter_ = 0;
    std::array<u8, 4> fifo_{};
    u8 fifoValid_ = 0;
    bool dmaActive_ = false;
    Direction direction_ = Direction::None;
    bool watchdogPending_ = false;
    bool busError_ = false;
    bool wonArbitration_ = false;
    bool resetCommandHigh_ = false;
    bool lastInterrupt_ = false;
    u64 transferPhase_ = 0;
    u64 watchdogPhase_ = 0;
};

} // namespace openmac
