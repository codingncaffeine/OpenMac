#pragma once

// CD-ROM media containers. The drive serves a flat byte run; the image files
// people actually have wrap that run a few ways:
//
//   .iso           2048-byte user sectors, ISO 9660 ('CD001' at sector 16 + 1)
//                  or High Sierra ('CDROM' at sector 16 + 9)
//   .toast         usually the same 2048 framing, or a raw HFS master
//   .bin (+.cue)   raw 2352-byte sectors: 12-byte sync + MSF + mode + payload
//   HFS masters    'ER' Driver Descriptor Map at 0, or a bare volume with the
//                  'BD' MDB signature at byte 1024
//
// normalize() peels the 2352 framing (MODE1 data only for now) and passes
// everything else through flat, naming what it saw -- or why it refused -- in
// words meant for the person who picked the file. Audio tracks in a .cue are a
// later phase; the data track is what mounts software.

#include "openmac/types.hpp"

#include <cstdio>
#include <cstring>
#include <vector>

namespace openmac::cd {

struct Medium {
    std::vector<u8> data;      // flat bytes as the drive will serve them
    char desc[200] = {0};      // what it is (accepted) or why not (refused)
    bool ok = false;
};

inline bool hasRawSync(const std::vector<u8>& f, std::size_t off) {
    static const u8 sync[12] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    return f.size() >= off + 16 && std::memcmp(f.data() + off, sync, 12) == 0;
}

inline Medium normalize(std::vector<u8> file) {
    Medium m;
    const double mb = static_cast<double>(file.size()) / (1024.0 * 1024.0);

    // Raw 2352-byte sectors (a .bin): the sync pattern opens every sector.
    // MODE1 carries its 2048 user bytes at +16; MODE2/XA interleaves
    // subheaders this drive does not unpick yet.
    if (hasRawSync(file, 0)) {
        const u8 mode = file[15];
        if (mode != 1) {
            std::snprintf(m.desc, sizeof m.desc,
                          "a raw-sector CD image in MODE%u (XA?) — only MODE1 "
                          "data discs are supported so far", mode);
            return m;
        }
        const std::size_t sectors = file.size() / 2352u;
        m.data.resize(sectors * 2048u);
        for (std::size_t s = 0; s < sectors; ++s)
            std::memcpy(m.data.data() + s * 2048u,
                        file.data() + s * 2352u + 16, 2048u);
        m.ok = true;
        std::snprintf(m.desc, sizeof m.desc,
                      "raw MODE1 CD image (%.0f MB) — %zu data sectors extracted",
                      mb, sectors);
        return m;
    }

    // Apple-partitioned master: Driver Descriptor Map at block 0. Checked
    // before ISO because hybrid HFS/ISO discs carry both, and the partition
    // map is the truth about where things live.
    if (file.size() >= 512 && file[0] == 0x45 && file[1] == 0x52) {
        m.data = std::move(file);
        m.ok = true;
        std::snprintf(m.desc, sizeof m.desc, "Apple-partitioned CD (%.0f MB)", mb);
        return m;
    }

    // ISO 9660 / High Sierra: a volume descriptor at sector 16.
    if (file.size() >= 17u * 2048u &&
        (std::memcmp(file.data() + 16 * 2048 + 1, "CD001", 5) == 0 ||
         std::memcmp(file.data() + 16 * 2048 + 9, "CDROM", 5) == 0)) {
        m.data = std::move(file);
        m.ok = true;
        std::snprintf(m.desc, sizeof m.desc, "ISO 9660 CD image (%.0f MB)", mb);
        return m;
    }

    // A bare HFS volume master: the MDB signature two blocks in.
    if (file.size() >= 1536 && file[1024] == 0x42 && file[1025] == 0x44) {
        m.data = std::move(file);
        m.ok = true;
        std::snprintf(m.desc, sizeof m.desc, "HFS CD master (%.0f MB)", mb);
        return m;
    }

    std::snprintf(m.desc, sizeof m.desc,
                  "not a CD image this drive recognises — expected ISO 9660, raw "
                  "2352-byte MODE1 sectors, an Apple-partitioned master, or a "
                  "bare HFS master");
    return m;
}

} // namespace openmac::cd
