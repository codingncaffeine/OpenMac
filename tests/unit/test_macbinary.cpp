#include <doctest/doctest.h>

#include "../../core/src/machine/dc42.hpp"
#include "../../core/src/machine/macbinary.hpp"

#include <string>
#include <vector>

using namespace openmac;

namespace {

// Build a MacBinary II file around a data fork, the way an archiver would.
std::vector<u8> wrapMb(const std::vector<u8>& dataFork, const std::vector<u8>& rsrcFork,
                       const char* type, const char* creator, const char* name,
                       bool withCrc = true) {
    std::vector<u8> f(macbinary::kHeaderSize, 0);
    const std::size_t nameLen = std::string(name).size();
    f[1] = static_cast<u8>(nameLen);
    for (std::size_t i = 0; i < nameLen && i < 63; ++i) f[2 + i] = static_cast<u8>(name[i]);
    for (int i = 0; i < 4; ++i) { f[65 + i] = static_cast<u8>(type[i]); f[69 + i] = static_cast<u8>(creator[i]); }
    const u32 dn = static_cast<u32>(dataFork.size()), rn = static_cast<u32>(rsrcFork.size());
    f[83] = static_cast<u8>(dn >> 24); f[84] = static_cast<u8>(dn >> 16);
    f[85] = static_cast<u8>(dn >> 8);  f[86] = static_cast<u8>(dn);
    f[87] = static_cast<u8>(rn >> 24); f[88] = static_cast<u8>(rn >> 16);
    f[89] = static_cast<u8>(rn >> 8);  f[90] = static_cast<u8>(rn);
    if (withCrc) {
        f[122] = 129; f[123] = 129;                       // MacBinary II versions
        const u16 c = macbinary::crc(f.data(), 124);
        f[124] = static_cast<u8>(c >> 8); f[125] = static_cast<u8>(c);
    }
    f.insert(f.end(), dataFork.begin(), dataFork.end());
    f.resize(macbinary::kHeaderSize + macbinary::pad128(dataFork.size()), 0);
    f.insert(f.end(), rsrcFork.begin(), rsrcFork.end());
    f.resize(f.size() + (macbinary::pad128(rsrcFork.size()) - rsrcFork.size()), 0);
    return f;
}

std::vector<u8> pattern(std::size_t n, u32 seed) {
    std::vector<u8> d(n);
    for (auto& b : d) { seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5; b = static_cast<u8>(seed); }
    return d;
}

} // namespace

TEST_CASE("macbinary: a wrapped raw floppy is recognised, split and reassembled") {
    const std::vector<u8> data = pattern(1474560, 0x1234567u);
    const std::vector<u8> rsrc = pattern(600, 0xFEEDu);
    std::vector<u8> file = wrapMb(data, rsrc, "dimg", "Wrap", "Dark Castle");
    const std::vector<u8> original = file;
    CHECK(macbinary::isMacBinary(file));

    macbinary::Parts p = macbinary::split(file);
    CHECK(p.wrapped());
    CHECK(file == data);                        // what the drive sees
    CHECK(p.type() == "dimg");
    CHECK(p.creator() == "Wrap");
    CHECK(p.name() == "Dark Castle");
    CHECK(p.resource == rsrc);

    // An unchanged disk reassembles to the identical file.
    CHECK(macbinary::rewrap(p, file) == original);

    // A changed one reassembles to a valid MacBinary file holding the change.
    std::vector<u8> written = file;
    written[2048] ^= 0xFF;
    std::vector<u8> out = macbinary::rewrap(p, written);
    CHECK(macbinary::isMacBinary(out));
    macbinary::split(out);
    CHECK(out == written);
}

TEST_CASE("macbinary: a MacBinary I file (no CRC) still passes on structure") {
    const std::vector<u8> data = pattern(409600, 0xABCDEFu);
    std::vector<u8> file = wrapMb(data, {}, "dImg", "dCpy", "System Tools", false);
    CHECK(macbinary::isMacBinary(file));
    macbinary::Parts p = macbinary::split(file);
    CHECK(file == data);
    CHECK(macbinary::rewrap(p, file).size() ==
          macbinary::kHeaderSize + macbinary::pad128(data.size()));
}

TEST_CASE("macbinary: raw images and empty files are left alone") {
    std::vector<u8> raw = pattern(819200, 0x777u);
    raw[0] = 0;                                  // a zero first byte is not enough
    raw[1] = 0;                                  // (a name cannot be empty)
    const std::vector<u8> before = raw;
    CHECK(macbinary::isMacBinary(raw) == false);
    CHECK(macbinary::split(raw).wrapped() == false);
    CHECK(raw == before);

    std::vector<u8> tiny(64, 0);
    CHECK(macbinary::isMacBinary(tiny) == false);
}

TEST_CASE("macbinary: a corrupt CRC is rejected") {
    std::vector<u8> file = wrapMb(pattern(1024, 1), {}, "APPL", "oneb", "Setup");
    file[50] ^= 0x5A;                            // damage inside the CRC'd span
    CHECK(macbinary::isMacBinary(file) == false);
}

TEST_CASE("macbinary around dc42: the nested wrappers strip and reassemble") {
    // The common case in the wild: an .img.bin -- MacBinary around DiskCopy 4.2
    // around the sectors. Both come off on the way in and go back on the way
    // out, and an unchanged disk round-trips to the identical file.
    const std::vector<u8> sectors = pattern(819200, 0xC0FFEEu);
    std::vector<u8> dcFile(dc42::kHeaderSize, 0);
    dcFile[0] = 4;
    dcFile[1] = 'D'; dcFile[2] = 'i'; dcFile[3] = 's'; dcFile[4] = 'k';
    dcFile[64] = 0x00; dcFile[65] = 0x0C; dcFile[66] = 0x80; dcFile[67] = 0x00;
    const std::vector<u8> tags = pattern(1600 * 12, 0x7A65u);
    dcFile[68] = 0; dcFile[69] = 0;
    dcFile[70] = static_cast<u8>((tags.size() >> 8) & 0xFF);
    dcFile[71] = static_cast<u8>(tags.size() & 0xFF);
    const u32 ck = dc42::checksum(sectors.data(), sectors.size());
    dcFile[72] = static_cast<u8>(ck >> 24); dcFile[73] = static_cast<u8>(ck >> 16);
    dcFile[74] = static_cast<u8>(ck >> 8);  dcFile[75] = static_cast<u8>(ck);
    dcFile[82] = 0x01; dcFile[83] = 0x00;
    dcFile.insert(dcFile.end(), sectors.begin(), sectors.end());
    dcFile.insert(dcFile.end(), tags.begin(), tags.end());
    CHECK(dc42::isDiskCopy(dcFile));

    std::vector<u8> file = wrapMb(dcFile, pattern(300, 9), "dImg", "dCpy", "Games Disk");
    const std::vector<u8> original = file;

    macbinary::Parts mb = macbinary::split(file);
    CHECK(mb.wrapped());
    dc42::Parts dc = dc42::split(file);
    CHECK(dc.wrapped());
    CHECK(file == sectors);
    CHECK(dc.tags == tags);                      // tags kept, not dropped

    std::vector<u8> out = macbinary::rewrap(mb, dc42::rewrap(dc, file));
    CHECK(out == original);                      // byte for byte
}
