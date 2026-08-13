#pragma once

// Apple 343S1021 Peripheral Interface Controller used twice on the Macintosh
// IIfx.  Its NCR 65CX02 core executes firmware downloaded by the 68030 into
// 32 KiB of shared SRAM.  The ASIC adds a host window, a 16-bit timer, two
// byte-DMA channels, interrupt latches, two GPIO outputs and a sixteen-byte
// peripheral window (SCC in one PIC, SWIM in the other).

#include "openmac/r65c02.hpp"
#include "openmac/types.hpp"

#include <array>
#include <functional>

namespace openmac {

class IifxStateCodec;

class IifxIop {
public:
    static constexpr int ChannelCount = 8;
    static constexpr int MessageSize = 32;

    static constexpr u8 Bypass      = 0x01;
    static constexpr u8 AutoInc     = 0x02;
    static constexpr u8 Run         = 0x04;
    static constexpr u8 IrqToIop    = 0x08;
    static constexpr u8 Int0        = 0x10;
    static constexpr u8 Int1        = 0x20;
    static constexpr u8 HardwareInt = 0x40;
    static constexpr u8 DmaInactive = 0x80;

    static constexpr u16 SendMax = 0x0200;
    static constexpr u16 SendState = 0x0201;
    static constexpr u16 SendPatch = 0x021F;
    static constexpr u16 SendMsg = 0x0220;
    static constexpr u16 RecvMax = 0x0300;
    static constexpr u16 RecvState = 0x0301;
    static constexpr u16 RecvPatch = 0x031F;
    static constexpr u16 RecvMsg = 0x0320;
    static constexpr u16 Alive = 0x031F;

    static constexpr u8 MsgIdle = 0;
    static constexpr u8 MsgNew = 1;
    static constexpr u8 MsgReceived = 2;
    static constexpr u8 MsgComplete = 3;

    IifxIop();

    std::function<u8(u32)> bypassRead;
    std::function<void(u32, u8)> bypassWrite;
    std::function<u8(u8)> peripheralRead;
    std::function<void(u8, u8)> peripheralWrite;
    std::function<u8()> gpioRead;
    std::function<void(int, bool)> gpioWrite;
    std::function<void()> onInterruptChange;
    std::function<void(bool, u16, u8)> onSharedWrite;
    std::function<void(int, u8, u8)> onDmaWrite;
    // Bounded tracing hooks. They observe completed instruction and DMA
    // boundaries only and are absent from the normal fast path unless a
    // caller explicitly installs them.
    std::function<void(u16, u8, const R65C02&)> onInstruction;
    std::function<void(int, bool, u8, u16, u16)> onDmaTransfer;
    std::function<void(int, bool)> onDmaRequestEdge;

    void reset();
    u8 read(u32 offset);
    void write(u32 offset, u8 value);
    void tick(int mainCpuCycles);

    void setPeripheralInterrupt(bool level);
    void setDmaRequest(int channel, bool level);

    bool irqAsserted() const { return (status_ & (Int0 | Int1)) != 0; }
    bool bypassed() const { return (sccControl_ & Bypass) != 0; }
    bool running() const { return !resetHeld_; }
    bool firmwareAlive() const { return ram_[Alive] == 0xFF; }
    u8 status() const;
    u16 address() const { return address_; }
    u8 interruptMask() const { return interruptMask_; }
    u8 interruptFlags() const { return interruptFlags_; }
    u8 timerDpllControl() const { return timerDpllControl_; }
    u8 ioControl() const { return ioControl_; }
    u8 dmaControl(int channel) const {
        return channel >= 0 && channel < 2
            ? dma_[static_cast<std::size_t>(channel)].control : 0;
    }
    u16 dmaMap(int channel) const {
        return channel >= 0 && channel < 2
            ? dma_[static_cast<std::size_t>(channel)].map : 0;
    }
    u16 dmaCount(int channel) const {
        return channel >= 0 && channel < 2
            ? dma_[static_cast<std::size_t>(channel)].count : 0;
    }
    bool dmaRequest(int channel) const {
        return channel >= 0 && channel < 2 &&
            dma_[static_cast<std::size_t>(channel)].request;
    }
    u8 ram(u16 address) const { return ram_[address & 0x7FFFu]; }
    void setRam(u16 address, u8 value) { ram_[address & 0x7FFFu] = value; }
    const R65C02& cpu() const { return cpu_; }
    u64 cpuCycles() const { return cpuCycles_; }

    u32 hostReads = 0;
    u32 hostWrites = 0;
    u32 ramReads = 0;
    u32 ramWrites = 0;
    u32 mailboxCommands = 0;
    u64 dmaTransfers = 0;
    u64 timerExpirations = 0;

private:
    friend class IifxStateCodec;

    enum Interrupt : int {
        Dma1Interrupt = 1,
        Dma2Interrupt = 2,
        PeripheralInterrupt = 3,
        HostInterrupt = 4,
        TimerInterrupt = 5,
    };

    struct DmaChannel {
        u8 control = 0;
        u16 map = 0;
        u16 count = 0;
        bool request = false;
    };

    void writeStatus(u8 value);
    u8 cpuRead(u16 address);
    void cpuWrite(u16 address, u8 value);
    u8 timerRead(u8 offset);
    void timerWrite(u8 offset, u8 value);
    u8 dmaRead(u8 offset) const;
    void dmaWrite(u8 offset, u8 value);
    void advanceAsic(u64 ticks);
    void dmaBeat();
    void setInterrupt(int which);
    void clearInterrupt(int which);
    void updateCpuIrq();
    void changed();

    std::array<u8, 0x8000> ram_{};
    R65C02 cpu_;
    u16 address_ = 0;
    u8 status_ = DmaInactive;
    bool resetHeld_ = true;

    u16 timerLatch_ = 0;
    s64 timerTicks_ = -1;
    u64 timerSinceExpiry_ = 0;
    std::array<DmaChannel, 2> dma_{};
    u8 sccControl_ = 0;
    u8 ioControl_ = 0;
    u8 timerDpllControl_ = 0;
    u8 interruptMask_ = 0;
    u8 interruptFlags_ = 0;

    s64 cpuPhase_ = 0;
    u64 cpuCycles_ = 0;
    u64 asicPhase_ = 0;
    u64 dmaDivider_ = 0;
    bool hostMemoryAccess_ = false;
};

} // namespace openmac
