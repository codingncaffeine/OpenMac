#include "iop.hpp"

#include <algorithm>

namespace openmac {

namespace {

constexpr u64 kMainCpuHz = 40000000;
constexpr u64 kPicClockHz = 15667200;
constexpr u64 kIopCpuHz = kPicClockHz / 8;
constexpr u8 kDmaEnable = 0x01;
constexpr u8 kDmaRequest = 0x02;
constexpr u8 kDmaDirection = 0x04;
constexpr u8 kEnableOther = 0x08;

} // namespace

IifxIop::IifxIop() {
    cpu_.read = [this](u16 address) { return cpuRead(address); };
    cpu_.write = [this](u16 address, u8 value) { cpuWrite(address, value); };
    reset();
}

void IifxIop::reset() {
    ram_.fill(0);
    address_ = 0;
    status_ = DmaInactive;
    resetHeld_ = true;
    timerLatch_ = 0;
    timerTicks_ = -1;
    timerSinceExpiry_ = 0;
    dma_ = {};
    sccControl_ = ioControl_ = timerDpllControl_ = 0;
    interruptMask_ = interruptFlags_ = 0;
    cpuPhase_ = 0;
    cpuCycles_ = 0;
    asicPhase_ = dmaDivider_ = 0;
    hostReads = hostWrites = ramReads = ramWrites = mailboxCommands = 0;
    dmaTransfers = timerExpirations = 0;
    changed();
}

u8 IifxIop::status() const {
    // PINT (D6) and /REQ (D7) are meaningful to the host only while the PIC
    // has selected peripheral bypass. In normal operation D7 reads high.
    return bypassed() ? static_cast<u8>(status_ | Bypass)
                      : static_cast<u8>((status_ & 0x3Fu) | DmaInactive);
}

u8 IifxIop::read(u32 offset) {
    ++hostReads;
    const u32 reg = offset & 0x3Fu;
    if (reg < 2) return static_cast<u8>(address_ >> 8);
    if (reg < 4) return static_cast<u8>(address_);
    if (reg < 8) return status();
    if (reg < 12) {
        ++ramReads;
        const u8 value = cpuRead(address_);
        if (status_ & AutoInc) ++address_;
        return value;
    }
    if (reg >= 0x20 && reg < 0x24)
        return bypassed() && bypassRead ? bypassRead(offset) : 0xFF;
    return 0xFF;
}

void IifxIop::write(u32 offset, u8 value) {
    ++hostWrites;
    const u32 reg = offset & 0x3Fu;
    if (reg < 2) {
        address_ = static_cast<u16>((address_ & 0x00FFu) |
                                    (static_cast<u16>(value) << 8));
    } else if (reg < 4) {
        address_ = static_cast<u16>((address_ & 0xFF00u) | value);
    } else if (reg < 8) {
        writeStatus(value);
    } else if (reg < 12) {
        ++ramWrites;
        hostMemoryAccess_ = true;
        cpuWrite(address_, value);
        hostMemoryAccess_ = false;
        if (status_ & AutoInc) ++address_;
    } else if (reg >= 0x20 && reg < 0x24 && bypassed() && bypassWrite) {
        bypassWrite(offset, value);
    }
}

void IifxIop::writeStatus(u8 value) {
    const bool release = (value & Run) != 0;
    if ((value & Int0) != 0) status_ &= static_cast<u8>(~Int0);
    if ((value & Int1) != 0) status_ &= static_cast<u8>(~Int1);
    status_ = static_cast<u8>((status_ & (Int0 | Int1 | HardwareInt |
                                          DmaInactive)) |
                              (value & (AutoInc | Run)));

    if (release != !resetHeld_) {
        resetHeld_ = !release;
        cpuPhase_ = 0;
        if (release) {
            cpu_.reset();
            updateCpuIrq();
        }
    }
    if ((value & IrqToIop) != 0 && !resetHeld_)
        setInterrupt(HostInterrupt);
    changed();
}

u8 IifxIop::cpuRead(u16 address) {
    if (address < 0x8000u) return ram_[address];
    if (address < 0xF000u) return ram_[address - 0x8000u];
    if (address >= 0xF800u) return ram_[address - 0x8000u];

    if (address >= 0xF010u && address <= 0xF013u)
        return timerRead(static_cast<u8>(address - 0xF010u));
    if (address >= 0xF020u && address <= 0xF02Fu)
        return dmaRead(static_cast<u8>(address - 0xF020u));
    switch (address) {
    case 0xF030: return sccControl_;
    case 0xF031: return ioControl_;
    case 0xF032:
        return static_cast<u8>(timerDpllControl_ |
            ((gpioRead ? gpioRead() : 0u) & 3u) << 2);
    case 0xF033: return interruptMask_;
    case 0xF034: return static_cast<u8>(interruptFlags_ & interruptMask_);
    case 0xF035: return static_cast<u8>((status_ & (Int0 | Int1)) >> 2);
    default:
        if (address >= 0xF040u && address <= 0xF04Fu)
            return bypassed() ? 0 : (peripheralRead
                ? peripheralRead(static_cast<u8>(address & 0x0Fu)) : 0);
        return 0xFF;
    }
}

void IifxIop::cpuWrite(u16 address, u8 value) {
    if (address < 0x8000u) {
        ram_[address] = value;
        if (onSharedWrite) onSharedWrite(hostMemoryAccess_, address, value);
        return;
    }
    if (address < 0xF000u) {
        const u16 shared = static_cast<u16>(address - 0x8000u);
        ram_[shared] = value;
        if (onSharedWrite) onSharedWrite(hostMemoryAccess_, shared, value);
        return;
    }
    if (address >= 0xF800u) {
        const u16 shared = static_cast<u16>(address - 0x8000u);
        ram_[shared] = value;
        if (onSharedWrite) onSharedWrite(hostMemoryAccess_, shared, value);
        return;
    }

    if (address >= 0xF010u && address <= 0xF013u) {
        timerWrite(static_cast<u8>(address - 0xF010u), value);
        return;
    }
    if (address >= 0xF020u && address <= 0xF02Fu) {
        dmaWrite(static_cast<u8>(address - 0xF020u), value);
        return;
    }
    switch (address) {
    case 0xF030:
        sccControl_ = value;
        if (!bypassed() && (status_ & HardwareInt))
            setInterrupt(PeripheralInterrupt);
        else
            clearInterrupt(PeripheralInterrupt);
        if (gpioWrite) gpioWrite(1, (value & 0x80u) != 0);
        break;
    case 0xF031:
        ioControl_ = value;
        break;
    case 0xF032:
        timerDpllControl_ = static_cast<u8>((timerDpllControl_ & 0xA0u) |
                                            (value & 0x53u));
        if (gpioWrite) gpioWrite(0, (value & 0x02u) != 0);
        break;
    case 0xF033:
        interruptMask_ = static_cast<u8>(value & 0x3Eu);
        updateCpuIrq();
        break;
    case 0xF034:
        interruptFlags_ &= static_cast<u8>(~value);
        updateCpuIrq();
        break;
    case 0xF035:
        status_ |= static_cast<u8>((value & 0x0Cu) << 2);
        changed();
        break;
    default:
        if (address >= 0xF040u && address <= 0xF04Fu &&
            !bypassed() && peripheralWrite)
            peripheralWrite(static_cast<u8>(address & 0x0Fu), value);
        break;
    }
}

u8 IifxIop::timerRead(u8 offset) {
    u16 count = timerLatch_;
    if ((offset & 2u) == 0) {
        if (timerTicks_ >= 0)
            count = static_cast<u16>(std::max<s64>(0, (timerTicks_ - 4) / 8));
        else
            count = static_cast<u16>(0xFFFFu -
                static_cast<u16>((timerSinceExpiry_ + 4u) / 8u));
        if (offset == 0) clearInterrupt(TimerInterrupt);
    }
    return (offset & 1u) ? static_cast<u8>(count >> 8)
                         : static_cast<u8>(count);
}

void IifxIop::timerWrite(u8 offset, u8 value) {
    if (offset & 1u) {
        timerLatch_ = static_cast<u16>((timerLatch_ & 0x00FFu) |
                                       (static_cast<u16>(value) << 8));
        if (offset == 1) {
            clearInterrupt(TimerInterrupt);
            timerTicks_ = static_cast<s64>(timerLatch_) * 8 + 12;
            timerSinceExpiry_ = 0;
        }
    } else {
        timerLatch_ = static_cast<u16>((timerLatch_ & 0xFF00u) | value);
    }
}

u8 IifxIop::dmaRead(u8 offset) const {
    const DmaChannel& channel = dma_[(offset >> 3) & 1u];
    switch (offset & 7u) {
    case 0: return channel.control;
    case 1: return static_cast<u8>(channel.map);
    case 2: return static_cast<u8>(channel.map >> 8);
    case 3: return static_cast<u8>(channel.count);
    case 4: return static_cast<u8>((channel.count >> 8) & 7u);
    default: return 0;
    }
}

void IifxIop::dmaWrite(u8 offset, u8 value) {
    const int index = (offset >> 3) & 1u;
    DmaChannel& channel = dma_[static_cast<std::size_t>(index)];
    switch (offset & 7u) {
    case 0:
        channel.control = static_cast<u8>((value & ~kDmaRequest) |
                                           (channel.control & kDmaRequest));
        break;
    case 1:
        channel.map = static_cast<u16>((channel.map & 0xFF00u) | value);
        break;
    case 2:
        channel.map = static_cast<u16>((channel.map & 0x00FFu) |
                                       (static_cast<u16>(value) << 8));
        break;
    case 3:
        channel.count = static_cast<u16>((channel.count & 0x0700u) | value);
        break;
    case 4:
        channel.count = static_cast<u16>((channel.count & 0x00FFu) |
                                         ((value & 7u) << 8));
        break;
    default:
        break;
    }
    if (onDmaWrite) onDmaWrite(index, static_cast<u8>(offset & 7u), value);
}

void IifxIop::setPeripheralInterrupt(bool level) {
    if (level) {
        status_ |= HardwareInt;
        if (!bypassed()) setInterrupt(PeripheralInterrupt);
    } else {
        status_ &= static_cast<u8>(~HardwareInt);
        if (!bypassed()) clearInterrupt(PeripheralInterrupt);
    }
    changed();
}

void IifxIop::setDmaRequest(int channel, bool level) {
    if (channel < 0 || channel >= 2) return;
    DmaChannel& dma = dma_[static_cast<std::size_t>(channel)];
    const bool changedLevel = dma.request != level;
    dma.request = level;
    if (level) dma.control |= kDmaRequest;
    else dma.control &= static_cast<u8>(~kDmaRequest);
    if (changedLevel && onDmaRequestEdge) onDmaRequestEdge(channel, level);
}

void IifxIop::dmaBeat() {
    for (int index = 0; index < 2; ++index) {
        DmaChannel& channel = dma_[static_cast<std::size_t>(index)];
        if (!(channel.control & kDmaEnable) || !channel.request ||
            channel.count == 0)
            continue;
        const u8 peripheral = static_cast<u8>(channel.control >> 4);
        u8 value = 0;
        if (channel.control & kDmaDirection) {
            value = peripheralRead ? peripheralRead(peripheral) : 0;
            cpuWrite(channel.map, value);
        } else {
            value = cpuRead(channel.map);
            if (peripheralWrite) peripheralWrite(peripheral, value);
        }
        if (onDmaTransfer)
            onDmaTransfer(index, (channel.control & kDmaDirection) != 0,
                          value, channel.map, channel.count);
        ++channel.map;
        --channel.count;
        ++dmaTransfers;
        if (channel.count == 0) {
            channel.control &= static_cast<u8>(~kDmaEnable);
            DmaChannel& other = dma_[static_cast<std::size_t>(index ^ 1)];
            if (other.control & kEnableOther) other.control |= kDmaEnable;
            setInterrupt(Dma1Interrupt + index);
        }
    }
}

void IifxIop::advanceAsic(u64 ticks) {
    if (timerTicks_ >= 0) {
        timerTicks_ -= static_cast<s64>(ticks);
        while (timerTicks_ <= 0) {
            ++timerExpirations;
            setInterrupt(TimerInterrupt);
            if (timerDpllControl_ & 1u) {
                timerTicks_ += static_cast<s64>(timerLatch_ + 2u) * 8;
            } else {
                timerSinceExpiry_ = static_cast<u64>(-timerTicks_);
                timerTicks_ = -1;
                break;
            }
        }
    } else {
        timerSinceExpiry_ += ticks;
    }

    dmaDivider_ += ticks;
    while (dmaDivider_ >= 8u) {
        dmaDivider_ -= 8u;
        dmaBeat();
    }
}

void IifxIop::tick(int mainCpuCycles) {
    if (mainCpuCycles <= 0) return;
    asicPhase_ += static_cast<u64>(mainCpuCycles) * kPicClockHz;
    const u64 asicTicks = asicPhase_ / kMainCpuHz;
    asicPhase_ %= kMainCpuHz;
    if (asicTicks) advanceAsic(asicTicks);

    if (!resetHeld_) {
        cpuPhase_ += static_cast<s64>(mainCpuCycles) *
                     static_cast<s64>(kIopCpuHz);
        while (cpuPhase_ >= static_cast<s64>(kMainCpuHz)) {
            const u16 instructionPc = cpu_.pc;
            const int used = cpu_.step();
            cpuCycles_ += static_cast<u64>(used);
            cpuPhase_ -= static_cast<s64>(used) *
                         static_cast<s64>(kMainCpuHz);
            if (onInstruction)
                onInstruction(instructionPc, cpu_.lastOpcode, cpu_);
        }
    }
}

void IifxIop::setInterrupt(int which) {
    interruptFlags_ |= static_cast<u8>(1u << which);
    updateCpuIrq();
}

void IifxIop::clearInterrupt(int which) {
    interruptFlags_ &= static_cast<u8>(~(1u << which));
    updateCpuIrq();
}

void IifxIop::updateCpuIrq() {
    cpu_.setIrq((interruptMask_ & interruptFlags_) != 0);
}

void IifxIop::changed() {
    if (onInterruptChange) onInterruptChange();
}

} // namespace openmac
