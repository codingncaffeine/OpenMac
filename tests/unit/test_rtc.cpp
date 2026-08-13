#include <doctest/doctest.h>

#include "../../core/src/machine/rtc.hpp"

using namespace openmac;

namespace {

void clockBit(Rtc& rtc, bool bit) {
    rtc.setLines(bit, false, false);
    rtc.setLines(bit, true, false);
}

void sendByte(Rtc& rtc, u8 value) {
    for (int bit = 7; bit >= 0; --bit)
        clockBit(rtc, (value & (1u << bit)) != 0);
}

void writeExtended(Rtc& rtc, u8 command, u8 extension, u8 value) {
    rtc.setLines(true, false, true);   // deselect
    rtc.setLines(true, false, false);  // select
    sendByte(rtc, command);
    sendByte(rtc, extension);
    sendByte(rtc, value);
    rtc.setLines(true, false, true);
}

void writeRegister(Rtc& rtc, u8 command, u8 value) {
    rtc.setLines(true, false, true);   // deselect
    rtc.setLines(true, false, false);  // select
    sendByte(rtc, command);
    sendByte(rtc, value);
    rtc.setLines(true, false, true);
}

} // namespace

TEST_CASE("RTC extended address ignores the reserved extension high bit") {
    Rtc rtc;
    rtc.reset();

    // Command $3A supplies A7..A5 = 010 and extension $DC supplies
    // A4..A0 = 10111, so this is XPRAM $57. Extension bit 7 must not
    // turn it into $77, which is the Start Manager's OS-type byte.
    rtc.xpram()[0x77] = 1;
    writeExtended(rtc, 0x3A, 0xDC, 0x5A);
    CHECK(rtc.xpram()[0x57] == 0x5A);
    CHECK(rtc.xpram()[0x77] == 1);
}

TEST_CASE("RTC legacy commands address all 20 system parameter bytes") {
    Rtc rtc;
    rtc.reset();

    // Per Inside Macintosh, the first sixteen parameter bytes occupy XPRAM
    // $10..$1F and the last four occupy XPRAM $08..$0B.  The command-register
    // numbers therefore match their physical XPRAM offsets even though the
    // logical 20-byte parameter block wraps between the two ranges.
    writeRegister(rtc, 0x41, 0xA8); // register $10 -> XPRAM $10 (validity)
    writeRegister(rtc, 0x61, 0xA5); // register $18 -> XPRAM $18
    writeRegister(rtc, 0x7D, 0x5A); // register $1F -> XPRAM $1F
    writeRegister(rtc, 0x21, 0x13); // register $08 -> XPRAM $08
    writeRegister(rtc, 0x25, 0x88); // register $09 -> XPRAM $09
    writeRegister(rtc, 0x29, 0x01); // register $0A -> XPRAM $0A
    writeRegister(rtc, 0x2D, 0x6C); // register $0B -> XPRAM $0B

    CHECK(rtc.xpram()[0x10] == 0xA8);
    CHECK(rtc.xpram()[0x18] == 0xA5);
    CHECK(rtc.xpram()[0x1F] == 0x5A);
    CHECK(rtc.xpram()[0x08] == 0x13);
    CHECK(rtc.xpram()[0x09] == 0x88);
    CHECK(rtc.xpram()[0x0A] == 0x01);
    CHECK(rtc.xpram()[0x0B] == 0x6C);

    // These are the locations used by the old, incorrect aliases.
    CHECK(rtc.xpram()[0x00] == 0x00);
    CHECK(rtc.xpram()[0x13] == 0x00);
}
