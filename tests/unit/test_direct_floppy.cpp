#include <doctest/doctest.h>

#include "../../core/src/machine/direct_floppy.hpp"

#include <vector>

using namespace openmac;

namespace {

constexpr u64 kClockHz = 15667200;

// A disk's worth of bytes that are not all alike.
std::vector<u8> sectors(std::size_t n, u32 seed = 0x13579BDFu) {
    std::vector<u8> d(n);
    u32 x = seed;
    for (auto& b : d) { x ^= x << 13; x ^= x >> 17; x ^= x << 5; b = static_cast<u8>(x); }
    return d;
}

// A DiskCopy 4.2 file around a disk: the 84-byte header, then the sectors.
std::vector<u8> diskCopyFile(const std::vector<u8>& data) {
    std::vector<u8> f(dc42::kHeaderSize, 0);
    f[0] = 6;
    const char* name = "Disk 1";
    for (int i = 0; i < 6; ++i) f[1 + static_cast<std::size_t>(i)] = static_cast<u8>(name[i]);
    const u32 n = static_cast<u32>(data.size());
    f[64] = static_cast<u8>(n >> 24); f[65] = static_cast<u8>(n >> 16);
    f[66] = static_cast<u8>(n >> 8);  f[67] = static_cast<u8>(n);
    const u32 sum = dc42::checksum(data.data(), data.size());
    f[72] = static_cast<u8>(sum >> 24); f[73] = static_cast<u8>(sum >> 16);
    f[74] = static_cast<u8>(sum >> 8);  f[75] = static_cast<u8>(sum);
    f[82] = 0x01; f[83] = 0x00;
    f.insert(f.end(), data.begin(), data.end());
    return f;
}

// The chip's sixteen addresses in IWM mode: an even one clears a line, the
// next odd one sets it.
enum : int { kCa0 = 0, kCa1 = 2, kCa2 = 4, kLstrb = 6, kEnable = 8, kSelect = 10, kQ6 = 12, kQ7 = 14 };
void line(DirectFloppy& f, int which, bool high, u64 now) { (void)f.access(which + (high ? 1 : 0), false, 0, now); }

// Address one of the mechanism's sixteen status lines (CA2 CA1 CA0 SEL) and
// read it through the chip's status register, as the ROM's driver does.
bool sense(DirectFloppy& f, int address, u64 now) {
    line(f, kCa2, (address & 8) != 0, now);
    line(f, kCa1, (address & 4) != 0, now);
    line(f, kCa0, (address & 2) != 0, now);
    f.setHeadSelect((address & 1) != 0);
    line(f, kQ7, false, now);
    return (f.access(kQ6 + 1, false, 0, now) & 0x80) != 0;   // Q6 high: the status register
}

constexpr int kCstin = 0x1, kWrtPrt = 0x3, kSuperDrive = 0xA, kInstalled = 0xE, kMediaIsLowDensity = 0xF;

} // namespace

TEST_CASE("direct floppy: only whole floppy sizes go in, and a refused file leaves the disk that is there") {
    DirectFloppy f(kClockHz);
    CHECK_FALSE(f.present());
    CHECK_FALSE(f.insert(sectors(12345), false));
    CHECK_FALSE(f.insert(sectors(737280), false));          // a 720K PC disk is not one of the three
    CHECK_FALSE(f.present());
    const std::vector<u8> disk = sectors(819200);
    REQUIRE(f.insert(disk, false));
    CHECK(f.present());
    CHECK_FALSE(f.insert(sectors(1000), false));
    CHECK(f.present());
    CHECK(f.sectors() == disk);
    CHECK(f.insert(sectors(409600), true));
    CHECK(f.readOnly());
    CHECK(f.insert(sectors(1474560), false));
    CHECK(f.sectors().size() == 1474560);
}

TEST_CASE("direct floppy: a disk comes back in the file format it came in") {
    DirectFloppy f(kClockHz);
    const std::vector<u8> disk = sectors(1474560);
    const std::vector<u8> file = diskCopyFile(disk);
    REQUIRE(file.size() == 1474644);
    REQUIRE(f.insert(file, false));
    CHECK(f.sectors() == disk);                  // the drive holds the sectors alone
    CHECK(f.fileImage() == file);                // and gives the file back around them
    // A bare image stays bare.
    REQUIRE(f.insert(disk, false));
    CHECK(f.fileImage() == disk);
}

TEST_CASE("direct floppy: the status lines the driver reads come from the drive on the first connector") {
    DirectFloppy f(kClockHz);
    u64 now = 1000;
    CHECK_FALSE(sense(f, kInstalled, now));      // low: a drive is there
    CHECK(sense(f, kSuperDrive, now));
    CHECK(sense(f, kCstin, now));                // high: no disk
    REQUIRE(f.insert(sectors(819200), true));
    CHECK_FALSE(sense(f, kCstin, now));          // low: disk in place
    CHECK_FALSE(sense(f, kWrtPrt, now));         // low: write-protected
    CHECK(sense(f, kMediaIsLowDensity, now));
    REQUIRE(f.insert(sectors(1474560), false));
    CHECK(sense(f, kWrtPrt, now));
    CHECK_FALSE(sense(f, kMediaIsLowDensity, now));
    // Nothing is plugged into the second connector: every line floats high.
    line(f, kSelect, true, now);
    CHECK(sense(f, kInstalled, now));
    CHECK(sense(f, kCstin, now));
    line(f, kSelect, false, now);
    CHECK_FALSE(sense(f, kInstalled, now));
    CHECK_FALSE(sense(f, kCstin, now));
}

TEST_CASE("direct floppy: the driver's eject runs on the mechanism's clock; the host's takes the disk at once") {
    DirectFloppy f(kClockHz);
    int left = 0;
    f.onEject = [&] { ++left; };
    const std::vector<u8> disk = sectors(819200);
    REQUIRE(f.insert(disk, false));
    u64 now = 1000;
    // The eject command: register 6 (CA1 and CA0 high, SEL low), data bit
    // high on CA2, latched by the rise of LSTRB.
    line(f, kCa2, true, now);
    line(f, kCa1, true, now);
    line(f, kCa0, true, now);
    f.setHeadSelect(false);
    line(f, kLstrb, true, now);
    CHECK(f.present());                          // the motor has only just started
    now += kClockHz / 2;                         // half a second on
    (void)sense(f, kCstin, now);
    line(f, kCa2, true, now); line(f, kCa1, true, now); line(f, kCa0, true, now); f.setHeadSelect(false);
    CHECK(f.present());
    now += kClockHz / 2;                         // a second after the command
    (void)f.access(kLstrb + 1, false, 0, now);
    CHECK_FALSE(f.present());
    CHECK(f.ejects() == 1);
    CHECK(left == 1);
    CHECK(f.sectors() == disk);                  // the disk that came out is kept for the host
    line(f, kLstrb, false, now);

    REQUIRE(f.insert(disk, false));
    f.eject();
    CHECK_FALSE(f.present());
    CHECK(left == 2);
    CHECK(f.ejects() == 1);                      // the host's eject is not the mechanism's
    f.eject();                                   // an empty drive: nothing happens
    CHECK(left == 2);
}

TEST_CASE("direct floppy: an 800K disk gives nibbles in IWM mode and a 1.4 MB disk gives none") {
    const auto nibblesRead = [](std::size_t diskBytes) {
        DirectFloppy f(kClockHz);
        REQUIRE(f.insert(sectors(diskBytes), true));
        u64 now = 1000;
        line(f, kSelect, false, now);
        line(f, kEnable, true, now);             // the spindle starts
        line(f, kQ7, false, now);
        now += kClockHz;                         // a second later it is at speed
        int valid = 0;
        for (int i = 0; i < 40000; ++i) {
            now += 64;                           // several looks per nibble time
            if (f.access(kQ6, false, 0, now) & 0x80) ++valid;   // Q6 low, enabled: the data register
        }
        return std::pair<int, u64>(valid, f.bytesRead());
    };
    const auto gcr = nibblesRead(819200);
    CHECK(gcr.first > 5000);                     // a valid nibble has its top bit set
    CHECK(gcr.second > 5000);
    // High-density media is framed by the chip's second register set; the
    // software reader gets nothing off it.
    const auto mfm = nibblesRead(1474560);
    CHECK(mfm.first == 0);
    CHECK(mfm.second == 0);
}

TEST_CASE("direct floppy: a machine reset keeps the disk in the drive") {
    DirectFloppy f(kClockHz);
    const std::vector<u8> disk = sectors(1474560);
    REQUIRE(f.insert(diskCopyFile(disk), true));
    f.reset();
    CHECK(f.present());
    CHECK(f.readOnly());
    CHECK(f.sectors() == disk);
    CHECK(f.fileImage().size() == 1474644);
    u64 now = 1000;
    CHECK_FALSE(sense(f, kCstin, now));
    CHECK_FALSE(f.chip().ismSelected());         // the chip is back in IWM mode
}
