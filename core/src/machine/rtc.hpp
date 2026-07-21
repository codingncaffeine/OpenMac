#pragma once

// Macintosh real-time clock / PRAM chip, bit-banged over VIA port B:
//   PB0 = serial data (bidirectional), PB1 = clock, PB2 = /enable.
// Commands are 8 bits (MSB first): bit 7 = 1 for read, bits 6-2 address.
// Writes are followed by 8 data bits; reads clock 8 bits back out.
// The extended (XPRAM) protocol uses a second command byte for 256 bytes.

#include "openmac/types.hpp"

#include <array>
#include <functional>

namespace openmac {

class Rtc {
public:
    void reset();

    // VIA port B lines as driven by the CPU.
    void setLines(bool data, bool clock, bool enable);
    // What the data line reads as when the chip is driving it.
    bool dataOut() const { return dataOut_; }

    void tickSecond() { ++seconds_; }

    u32 seconds() const { return seconds_; }
    std::array<u8, 256>& xpram() { return xpram_; }

    // Diagnostic: observe completed command/data bytes on the wire.
    std::function<void(const char* what, u8 value)> onByte;

private:
    void clockedInBit(bool bit);
    void executeCommand();
    u8   readRegister();
    void writeRegister(u8 value);

    u32 seconds_ = 0;
    std::array<u8, 256> xpram_{};

    enum class State { Command, ExtendedCommand, WriteData, ReadData };
    State state_ = State::Command;
    u8 shift_ = 0;
    int bits_ = 0;
    u8 cmd_ = 0;
    u8 extAddr_ = 0;
    bool extended_ = false;
    bool writeProtect_ = false;
    bool lastClock_ = false;
    bool enabled_ = false;
    bool dataOut_ = true;
    u8 outShift_ = 0;
    int outBits_ = 0;
};

} // namespace openmac
