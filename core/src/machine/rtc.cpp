#include "rtc.hpp"

namespace openmac {

void Rtc::reset() {
    state_ = State::Command;
    shift_ = 0;
    bits_ = 0;
    extended_ = false;
    lastClock_ = false;
    enabled_ = false;
    dataOut_ = true;
}

void Rtc::setLines(bool data, bool clock, bool enable) {
    const bool nowEnabled = !enable;   // /enable is active low
    if (!nowEnabled) {
        if (enabled_) {                // deselect resets the protocol
            state_ = State::Command;
            bits_ = 0;
            extended_ = false;
        }
        enabled_ = false;
        lastClock_ = clock;
        return;
    }
    enabled_ = true;

    if (!lastClock_ && clock) {        // rising edge
        if (state_ == State::ReadData) {
            dataOut_ = (outShift_ & 0x80) != 0;
            outShift_ = static_cast<u8>(outShift_ << 1);
            if (++outBits_ >= 8) {
                state_ = State::Command;
                bits_ = 0;
            }
        } else {
            clockedInBit(data);
        }
    }
    lastClock_ = clock;
}

void Rtc::clockedInBit(bool bit) {
    shift_ = static_cast<u8>((shift_ << 1) | (bit ? 1 : 0));
    if (++bits_ < 8) return;
    bits_ = 0;

    switch (state_) {
    case State::Command:
        cmd_ = shift_;
        if (onByte) onByte("cmd", cmd_);
        if ((cmd_ & 0x78) == 0x38) {   // %x0111xxx: extended command prefix
            extended_ = true;
            state_ = State::ExtendedCommand;
        } else if (cmd_ & 0x80) {
            outShift_ = readRegister();
            if (onByte) onByte("out", outShift_);
            outBits_ = 0;
            state_ = State::ReadData;
        } else {
            state_ = State::WriteData;
        }
        break;
    case State::ExtendedCommand:
        extAddr_ = shift_;
        if (onByte) onByte("adr", extAddr_);
        if (cmd_ & 0x80) {
            // The command contributes A7..A5 and the second byte contributes
            // A4..A0 in bits 6..2. Bit 7 of the second byte is reserved and is
            // ignored by the RTC; allowing it through aliases (for example)
            // XPRAM $57 onto $77 and corrupts the Start Manager OS type.
            const u8 addr = static_cast<u8>(((cmd_ & 0x07) << 5) |
                                            ((extAddr_ >> 2) & 0x1F));
            if (onByte) onByte("xra", addr);
            outShift_ = xpram_[addr];
            if (onByte) onByte("out", outShift_);
            outBits_ = 0;
            state_ = State::ReadData;
        } else {
            state_ = State::WriteData;
        }
        break;
    case State::WriteData:
        if (onByte) onByte("dat", shift_);
        if (extended_) {
            const u8 addr = static_cast<u8>(((cmd_ & 0x07) << 5) |
                                            ((extAddr_ >> 2) & 0x1F));
            if (onByte) onByte(writeProtect_ ? "xwp" : "xwa", addr);
            if (!writeProtect_) xpram_[addr] = shift_;
        } else {
            writeRegister(shift_);
        }
        state_ = State::Command;
        extended_ = false;
        break;
    default:
        break;
    }
    shift_ = 0;
}

u8 Rtc::readRegister() {
    const int reg = (cmd_ >> 2) & 0x1F;
    if (onByte) onByte("rrg", static_cast<u8>(reg));
    switch (reg) {
    case 0: case 4: return static_cast<u8>(seconds_ & 0xFF);
    case 1: case 5: return static_cast<u8>((seconds_ >> 8) & 0xFF);
    case 2: case 6: return static_cast<u8>((seconds_ >> 16) & 0xFF);
    case 3: case 7: return static_cast<u8>((seconds_ >> 24) & 0xFF);
    default:
        // The original 20-byte system-parameter block is split in XPRAM:
        // legacy registers $10..$1F expose XPRAM $10..$1F (the first 16
        // bytes), while registers $08..$0B expose XPRAM $08..$0B (the final
        // four bytes).  Keeping the physical XPRAM offsets here is important:
        // the validity byte written through register $10 must appear at
        // XPRAM $10 or the ROM discards the block and installs defaults.
        if (reg >= 0x10) return xpram_[static_cast<u8>(reg)];
        if (reg >= 8 && reg <= 0x0B)
            return xpram_[static_cast<u8>(reg)];
        return 0xFF;
    }
}

void Rtc::writeRegister(u8 value) {
    const int reg = (cmd_ >> 2) & 0x1F;
    if (onByte) onByte(writeProtect_ && reg != 0x0D ? "wrp" : "wrg",
                       static_cast<u8>(reg));
    if (writeProtect_ && reg != 0x0D) return;
    switch (reg) {
    case 0: case 4: seconds_ = (seconds_ & 0xFFFFFF00u) | value; break;
    case 1: case 5: seconds_ = (seconds_ & 0xFFFF00FFu) | (u32(value) << 8); break;
    case 2: case 6: seconds_ = (seconds_ & 0xFF00FFFFu) | (u32(value) << 16); break;
    case 3: case 7: seconds_ = (seconds_ & 0x00FFFFFFu) | (u32(value) << 24); break;
    case 0x0C: break;                                   // test register
    case 0x0D: writeProtect_ = (value & 0x80) != 0; break;
    default:
        if (reg >= 0x10) xpram_[static_cast<u8>(reg)] = value;
        else if (reg >= 8 && reg <= 0x0B)
            xpram_[static_cast<u8>(reg)] = value;
        break;
    }
}

} // namespace openmac
