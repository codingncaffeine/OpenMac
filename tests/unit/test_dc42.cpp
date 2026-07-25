#include <doctest/doctest.h>

#include "../../core/src/machine/dc42.hpp"

#include <vector>

using namespace openmac;

namespace {

// A DiskCopy 4.2 file: 84-byte header, then the disk.
std::vector<u8> wrap(const std::vector<u8>& data, u32 tagSize = 0) {
    std::vector<u8> f(dc42::kHeaderSize, 0);
    f[0] = 6;
    const char* nm = "Disk 1";
    for (int i = 0; i < 6; ++i) f[1 + static_cast<std::size_t>(i)] = static_cast<u8>(nm[i]);
    const u32 n = static_cast<u32>(data.size());
    f[64] = static_cast<u8>(n >> 24); f[65] = static_cast<u8>(n >> 16);
    f[66] = static_cast<u8>(n >> 8);  f[67] = static_cast<u8>(n);
    f[68] = static_cast<u8>(tagSize >> 24); f[69] = static_cast<u8>(tagSize >> 16);
    f[70] = static_cast<u8>(tagSize >> 8);  f[71] = static_cast<u8>(tagSize);
    f[82] = 0x01; f[83] = 0x00;                    // the format's private word
    f.insert(f.end(), data.begin(), data.end());
    f.insert(f.end(), tagSize, 0);
    return f;
}

std::vector<u8> disk(std::size_t n) {
    std::vector<u8> d(n);
    u32 x = 0x13579BDFu;
    for (auto& b : d) { x ^= x << 13; x ^= x >> 17; x ^= x << 5; b = static_cast<u8>(x); }
    return d;
}

} // namespace

TEST_CASE("dc42: a wrapped 1.4MB image is recognised and unwrapped") {
    const std::vector<u8> data = disk(1474560);
    std::vector<u8> file = wrap(data);
    CHECK(file.size() == 1474644);            // the size that gives it away
    CHECK(dc42::isDiskCopy(file));

    std::vector<u8> header = dc42::takeHeader(file);
    CHECK(header.size() == dc42::kHeaderSize);
    CHECK(file.size() == 1474560);            // what the drive sees
    CHECK(file == data);                      // ...byte for byte
}

TEST_CASE("dc42: an 800K image with tag bytes unwraps to its data alone") {
    // 800K GCR disks were archived with twelve tag bytes a sector. The drive
    // synthesises those, so they are dropped.
    const std::vector<u8> data = disk(819200);
    std::vector<u8> file = wrap(data, 1600 * 12);
    CHECK(dc42::isDiskCopy(file));
    dc42::takeHeader(file);
    CHECK(file.size() == 819200);
    CHECK(file == data);
}

TEST_CASE("dc42: a raw image is left alone") {
    const std::vector<u8> data = disk(1474560);
    std::vector<u8> file = data;
    CHECK(dc42::isDiskCopy(file) == false);
    CHECK(dc42::takeHeader(file).empty());
    CHECK(file == data);                      // untouched
}

TEST_CASE("dc42: the magic alone is not enough") {
    // A raw disk whose bytes happen to carry $0100 at offset 82 must not be
    // mistaken for a wrapped one; the length has to account for itself too.
    std::vector<u8> file = disk(1474560);
    file[82] = 0x01; file[83] = 0x00;
    CHECK(dc42::isDiskCopy(file) == false);

    // Nor may a plausible header with a nonsensical data size pass.
    std::vector<u8> bad = wrap(disk(1024));
    bad[64] = 0xFF;                            // data size no longer matches the file
    CHECK(dc42::isDiskCopy(bad) == false);
}

TEST_CASE("dc42: rewrapping produces a file that unwraps to the same disk") {
    // What a host writes back after a session must still be a DiskCopy image, so
    // the file keeps the format it arrived in.
    const std::vector<u8> data = disk(819200);
    std::vector<u8> file = wrap(data);
    const std::vector<u8> header = dc42::takeHeader(file);

    std::vector<u8> written = file;            // pretend the guest changed a sector
    written[4096] ^= 0xFF;
    std::vector<u8> out = dc42::rewrap(header, written);

    CHECK(out.size() == dc42::kHeaderSize + written.size());
    CHECK(dc42::isDiskCopy(out));
    std::vector<u8> again = out;
    dc42::takeHeader(again);
    CHECK(again == written);
    // The name survives, and the checksum describes what is actually there.
    CHECK(out[0] == 6);
    CHECK(dc42::be32(out, 72) == dc42::checksum(written.data(), written.size()));
}
