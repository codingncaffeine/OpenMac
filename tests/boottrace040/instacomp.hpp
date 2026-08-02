#pragma once

// InstaCompOne: the compression Apple's Installer SDK put on script resources.
//
// A "System 7.5 Installation" script keeps its packages, rule frameworks and
// disk lists compressed with it, so a host-side dissection of what an installer
// INTENDED to do cannot read the interesting half of the script without this.
// The guest decompresses these through the 'dcmp' 3 code resource; this is the
// same algorithm on our side of the glass, so the two can be compared.
//
// Like Deflate it is LZ77 plus Huffman, with its own bitstream: a copy-length
// code decides literal-run versus back-reference, and the back-reference
// DISTANCE is coded against the magnitude of the current output position --
// early output can only reach back a little way, so fewer bits are spent there.
//
// Ported from the Python of Max Poliakovski and Elliot Nunn (MIT licensed),
// https://github.com/elliotnunn/macresources -- their reverse engineering of
// Apple's decompressor, not Apple code.

#include "openmac/types.hpp"

#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

namespace openmac::instacomp {

// One Huffman entry. `bits`/`code` are the codeword; a plain entry yields
// `value`, a compact one reads `valBits` more bits and adds `offset` -- the
// table's way of covering a long tail without listing every codeword.
struct HuffEntry {
    int bits;
    u32 code;
    bool compact;
    int value;     // plain
    int valBits;   // compact
    int offset;    // compact
};

inline const std::vector<HuffEntry>& lenTable() {
    static const std::vector<HuffEntry> t = {
        {2, 0b00, false, 0, 0, 0},
        {2, 0b01, false, 1, 0, 0},
        {3, 0b100, false, 2, 0, 0},
        {4, 0b1010, false, 3, 0, 0},
        {4, 0b1011, false, 4, 0, 0},
        {5, 0b11000, false, 5, 0, 0},
        {5, 0b11001, false, 6, 0, 0},
        {6, 0b110100, false, 7, 0, 0},
        {6, 0b110101, false, 8, 0, 0},
        {6, 0b110110, false, 9, 0, 0},
        {6, 0b110111, false, 10, 0, 0},
        {4, 0b1110, true, 0, 3, 11},
        {5, 0b11110, true, 0, 3, 19},
        {6, 0b111110, true, 0, 5, 27},
        {7, 0b1111110, true, 0, 6, 59},
        {8, 0b11111110, true, 0, 7, 123},
        {9, 0b111111110, true, 0, 8, 251},
        {10, 0b1111111110, true, 0, 9, 507},
        {11, 0b11111111110, true, 0, 10, 1019},
    };
    return t;
}

inline const std::vector<HuffEntry>& litTable() {
    static const std::vector<HuffEntry> t = {
        {1, 0b0, false, 1, 0, 0},
        {3, 0b100, false, 2, 0, 0},
        {3, 0b101, false, 3, 0, 0},
        {5, 0b11000, false, 4, 0, 0},
        {5, 0b11001, false, 5, 0, 0},
        {5, 0b11010, false, 6, 0, 0},
        {5, 0b11011, false, 7, 0, 0},
        {7, 0b1110000, false, 8, 0, 0},
        {7, 0b1110001, false, 9, 0, 0},
        {7, 0b1110010, false, 10, 0, 0},
        {7, 0b1110011, false, 11, 0, 0},
        {7, 0b1110100, false, 12, 0, 0},
        {7, 0b1110101, false, 13, 0, 0},
        {7, 0b1110110, false, 14, 0, 0},
        {7, 0b1110111, false, 15, 0, 0},
        {5, 0b11110, true, 0, 4, 16},
        {5, 0b11111, true, 0, 5, 32},
    };
    return t;
}

// ceil(log2(x)) for x >= 2, and 1 below that -- the width the coder spends on
// a sub-range of x values.
inline int nextPow2(int x) {
    if (x < 2) return 1;
    int b = 0;
    while ((1 << b) < x) ++b;
    return b;
}

class BitReader {
public:
    BitReader(const std::vector<u8>& in, std::size_t pos) : in_(in), pos_(pos) {}

    u32 show(int nb) {
        if (nb <= 0) return 0;
        while (nb > bits_) {
            const u8 byte = pos_ < in_.size() ? in_[pos_] : 0;
            if (pos_ >= in_.size()) ++overrun_;
            ++pos_;
            pool_ = (pool_ << 8) | byte;
            bits_ += 8;
        }
        return static_cast<u32>((pool_ >> (bits_ - nb)) &
                                (0xFFFFFFFFull >> (32 - nb)));
    }
    void flush(int nb) { bits_ = nb <= bits_ ? bits_ - nb : 0; }
    u32 get(int nb) {
        const u32 v = show(nb);
        flush(nb);
        return v;
    }
    bool overran() const { return overrun_ != 0; }
    std::size_t pos() const { return pos_; }

    struct State { std::size_t pos; u64 pool; int bits; int overrun; };
    State save() const { return {pos_, pool_, bits_, overrun_}; }
    void restore(const State& s) {
        pos_ = s.pos; pool_ = s.pool; bits_ = s.bits; overrun_ = s.overrun;
    }

private:
    const std::vector<u8>& in_;
    std::size_t pos_ = 0;
    u64 pool_ = 0;
    int bits_ = 0;
    int overrun_ = 0;
};

// Walk widths shortest-first: the code is prefix-free, so the first width at
// which the shown bits match a codeword is the right one.
inline bool decodeHuff(BitReader& bs, const std::vector<HuffEntry>& tab,
                       int minLen, int maxLen, int& out) {
    for (int w = minLen; w <= maxLen; ++w) {
        const u32 cw = bs.show(w);
        for (const auto& e : tab) {
            if (e.bits != w || e.code != cw) continue;
            bs.flush(w);
            out = e.compact ? static_cast<int>(bs.get(e.valBits)) + e.offset : e.value;
            return true;
        }
    }
    return false;
}

// The distance decoder. The current output length picks a tier k, and one or
// two bits then choose among three sub-ranges within it:
//
//   0   -> 1 .. 2^k                      (k bits)
//   10  -> 2^k+1 .. 5*2^k                (k+2 bits)
//   11  -> 5*2^k+1 upward                (width from the magnitude, so a wide
//                                         tier still spends only what it must)
//
// The tier boundaries are a table, not a formula -- they double up to 160 and
// then widen, the top sub-range absorbing the slack. The published reverse
// engineering left the two smallest tiers undecoded ("Anon9"/"Anon10"); they
// continue the doubling, and the derivation is checked rather than assumed:
// `unpack` requires the bitstream to be consumed to its last byte, which a
// mis-chosen tier desynchronises long before.
inline bool decodeDistanceAt(BitReader& bs, int mag, int k, int& out) {
    const int base = 1 << k;
    if (!bs.get(1)) {
        out = static_cast<int>(bs.get(k)) + 1;
    } else if (!bs.get(1)) {
        out = static_cast<int>(bs.get(k + 2)) + base + 1;
    } else {
        const int hiBase = 5 * base + 1;
        if (mag < hiBase) return false;
        out = static_cast<int>(bs.get(nextPow2(mag - (hiBase - 1)))) + hiBase;
    }
    return out >= 1 && out <= mag;
}

// Set to have every tier correction reported: the boundaries came partly from
// other people's guesses, and this is how they get checked against real data.
inline bool& traceTiers() { static bool on = false; return on; }

// The tiers worth trying at a magnitude, best guess first. Two of the table's
// boundaries were never pinned down by the published work, so where it is wrong
// the true tier is a neighbour; which one is settled not here but by whether
// the remainder of the stream still decodes.
inline std::vector<int> tierCandidates(int mag) {
    static const long kTierUpTo[] = {10, 20, 40, 80, 160, 672, 1344, 2688, 5376, 10752};
    int k0 = 24;
    for (int k = 0; k <= 24; ++k) {
        const long span = k < 10 ? kTierUpTo[k] : (10752L << (k - 9));
        if (mag <= span) { k0 = k; break; }
    }
    std::vector<int> c;
    for (int d = 0; d <= 3; ++d)
        for (int s = 0; s < (d ? 2 : 1); ++s) {
            const int k = d == 0 ? k0 : (s ? k0 - d : k0 + d);
            if (k >= 0 && k <= 24) c.push_back(k);
        }
    return c;
}

constexpr u32 kMagic = 0xA89F6572u;
constexpr int kLitMaxLen = 63;

inline bool isCompressed(const std::vector<u8>& r) {
    return r.size() >= 14 && r[0] == 0xA8 && r[1] == 0x9F && r[2] == 0x65 &&
           r[3] == 0x72;
}

// Decompress a resource that carries the compressed-resource header. Returns
// false with `why` set when the header is not the shape we know or the
// bitstream runs into the undecoded corner.
inline bool unpack(const std::vector<u8>& src, std::vector<u8>& out, std::string& why) {
    if (!isCompressed(src)) { why = "not a compressed resource"; return false; }
    const u32 hdrLen = static_cast<u32>((src[4] << 8) | src[5]);
    const u32 vers = src[6], iscmp = src[7];
    const u32 unpackSize = (static_cast<u32>(src[8]) << 24) |
                           (static_cast<u32>(src[9]) << 16) |
                           (static_cast<u32>(src[10]) << 8) | src[11];
    const u32 dcmp = static_cast<u32>((src[12] << 8) | src[13]);
    if (hdrLen != 18 || vers != 9 || iscmp != 1 || dcmp != 3) {
        why = "unsupported compressed-resource header";
        return false;
    }
    // 14..17 are algorithm-specific words the decoder does not use.
    BitReader bs(src, 18);
    out.clear();
    out.reserve(unpackSize);
    int mode = 1;   // 1 = expect a literal run, 0 = expect a back-reference

    // Where the tier table is uncertain the stream admits more than one
    // reading, and only one of them carries on decoding to the end. Each
    // distance is a choice point: take the table's tier, and if the walk later
    // contradicts itself, come back and take the next candidate. A correct walk
    // finishes on the last byte of the packed stream; that is the acceptance
    // test, and it is what turns a guessed boundary into a checked one.
    struct Choice {
        BitReader::State st;
        std::size_t outLen;
        int mode, copyCount, mag;
        std::vector<int> cands;
        std::size_t next;
    };
    std::vector<Choice> stack;
    std::vector<u8> bestTail;
    int budget = 200000;

    auto applyCopy = [&](Choice& c) -> bool {
        while (c.next < c.cands.size()) {
            const int k = c.cands[c.next++];
            bs.restore(c.st);
            out.resize(c.outLen);
            int distance = 0;
            if (!decodeDistanceAt(bs, c.mag, k, distance)) continue;
            if (bs.overran()) continue;
            const std::size_t refPos = static_cast<std::size_t>(c.mag - distance);
            for (int i = 0; i < c.copyCount; ++i) out.push_back(out[refPos + static_cast<std::size_t>(i)]);
            mode = 1;
            if (traceTiers() && c.next - 1 != 0)
                std::printf("tier: mag=%d used candidate %zu (k=%d) dist=%d\n",
                            c.mag, c.next - 1, k, distance);
            return true;
        }
        return false;
    };
    auto backtrack = [&]() -> bool {
        while (!stack.empty()) {
            if (--budget <= 0) return false;
            if (applyCopy(stack.back())) return true;
            stack.pop_back();
        }
        return false;
    };

    for (;;) {
        bool bad = false;
        while (out.size() < unpackSize) {
            int copyCount = 0;
            if (!decodeHuff(bs, lenTable(), 2, 11, copyCount)) { bad = true; break; }
            if (copyCount > 0 || mode == 0) {
                copyCount += 2;
                if (mode == 0) copyCount += 1;
                Choice c{bs.save(), out.size(), mode, copyCount,
                         static_cast<int>(out.size()), tierCandidates(static_cast<int>(out.size())), 0};
                if (!applyCopy(c)) { bad = true; break; }
                stack.push_back(std::move(c));
            } else {
                int litLen = 0;
                if (!decodeHuff(bs, litTable(), 1, 7, litLen)) { bad = true; break; }
                for (int i = 0; i < litLen; ++i) out.push_back(static_cast<u8>(bs.get(8)));
                mode = litLen < kLitMaxLen ? 0 : 1;
            }
            if (bs.overran()) { bad = true; break; }
        }
        // Consuming the stream to its last byte is what says the walk was the
        // intended one rather than merely a self-consistent one. Prefer such a
        // walk, but do not throw away a complete output that merely leaves a
        // tail unread -- report the shortfall and let the caller judge the
        // content, which is a stronger check than byte counting anyway.
        if (!bad && bs.pos() + 4 >= src.size()) { why.clear(); break; }
        if (!bad && out.size() >= unpackSize && why.empty()) {
            char msg[160];
            std::snprintf(msg, sizeof msg, "%zu of %zu packed bytes unread",
                          src.size() - bs.pos(), src.size());
            why = msg;
            bestTail = out;
        }
        if (!backtrack()) {
            if (!bestTail.empty()) { out = std::move(bestTail); out.resize(unpackSize); return true; }
            char msg[160];
            std::snprintf(msg, sizeof msg,
                          "no consistent decode (reached %zu of %u bytes, consumed "
                          "%zu of %zu)",
                          out.size(), unpackSize, bs.pos(), src.size());
            why = msg;
            return false;
        }
    }
    out.resize(unpackSize);
    return true;
}

} // namespace openmac::instacomp
