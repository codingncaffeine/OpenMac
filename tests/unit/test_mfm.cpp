#include <doctest/doctest.h>

#include "../../core/src/machine/mfm.hpp"

#include <vector>

using namespace openmac;

namespace {

std::vector<u8> makeVolume() {
    std::vector<u8> img(mfm::kImageBytes);
    u32 x = 0x2468ACE1u;
    for (auto& b : img) {
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;   // xorshift32
        b = static_cast<u8>(x);
    }
    return img;
}

} // namespace

TEST_CASE("mfm: the CRC over a field and its own CRC bytes is zero") {
    // That property is the whole point of the generator: the driver checks the
    // running CRC as the second CRC byte comes out of the FIFO and expects zero.
    const u8 id[8] = {0xA1, 0xA1, 0xA1, 0xFE, 12, 1, 7, 0x02};
    const u16 crc = mfm::crc16(id, 8);
    u8 whole[10];
    for (int i = 0; i < 8; ++i) whole[i] = id[i];
    whole[8] = static_cast<u8>(crc >> 8);
    whole[9] = static_cast<u8>(crc);
    CHECK(mfm::crc16(whole, 10) == 0);

    // A single flipped bit anywhere must break it.
    whole[5] ^= 0x01;
    CHECK(mfm::crc16(whole, 10) != 0);
}

TEST_CASE("mfm: a built track is a revolution of legal frames") {
    const std::vector<u8> src = makeVolume();
    std::vector<u8> trk, marks;
    mfm::buildTrack(src, 3, 1, trk, marks);

    CHECK(trk.size() == mfm::kTrackBytes);
    CHECK(marks.size() == trk.size());

    int idMarks = 0, dataMarks = 0, markBytes = 0;
    for (std::size_t i = 0; i + 3 < trk.size(); ++i) {
        if (marks[i]) ++markBytes;
        if (!(marks[i] && marks[i + 1] && marks[i + 2])) continue;
        if (trk[i] != 0xA1 || trk[i + 1] != 0xA1 || trk[i + 2] != 0xA1) continue;
        if (trk[i + 3] == 0xFE) ++idMarks;
        if (trk[i + 3] == 0xFB) ++dataMarks;
    }
    CHECK(idMarks == mfm::kSectorsPerTrack);
    CHECK(dataMarks == mfm::kSectorsPerTrack);
    // Three per address mark, three per data mark, plus the index mark.
    CHECK(markBytes == mfm::kSectorsPerTrack * 6 + 3);
}

TEST_CASE("mfm: every sector of a 1.4MB volume survives framing and reading back") {
    const std::vector<u8> src = makeVolume();
    std::vector<u8> dst(src.size(), 0);
    std::vector<u8> trk, marks;

    int total = 0;
    for (int track = 0; track < mfm::kTracks; ++track)
        for (int side = 0; side < mfm::kSides; ++side) {
            mfm::buildTrack(src, track, side, trk, marks);
            total += mfm::decodeTrack(trk, marks, dst, track, side);
        }
    CHECK(total == mfm::kTracks * mfm::kSides * mfm::kSectorsPerTrack);   // 2880
    CHECK(dst == src);
}

TEST_CASE("mfm: a sector with a broken CRC is refused, not written back") {
    const std::vector<u8> src = makeVolume();
    std::vector<u8> dst = src;
    std::vector<u8> trk, marks;
    mfm::buildTrack(src, 0, 0, trk, marks);

    // Corrupt one data byte without touching its CRC.
    std::size_t j = 0;
    for (; j + 4 < trk.size(); ++j)
        if (marks[j] && marks[j + 1] && marks[j + 2] && trk[j + 3] == 0xFB) break;
    REQUIRE(j + 4 < trk.size());
    trk[j + 40] ^= 0xFF;

    CHECK(mfm::decodeTrack(trk, marks, dst, 0, 0) == mfm::kSectorsPerTrack - 1);
    CHECK(dst == src);
}

TEST_CASE("mfm: a mark byte is not just its value") {
    // $A1 occurs in ordinary data too; only the flag distinguishes a real
    // address mark, which is why the flags travel with the bytes.
    std::vector<u8> src(mfm::kImageBytes, 0xA1);
    std::vector<u8> dst(src.size(), 0);
    std::vector<u8> trk, marks;
    mfm::buildTrack(src, 5, 0, trk, marks);
    CHECK(mfm::decodeTrack(trk, marks, dst, 5, 0) == mfm::kSectorsPerTrack);
    for (int s = 1; s <= mfm::kSectorsPerTrack; ++s) {
        const std::size_t off = mfm::blockOf(5, 0, s) * mfm::kSectorBytes;
        for (std::size_t k = 0; k < mfm::kSectorBytes; ++k) CHECK(dst[off + k] == 0xA1);
    }
}
