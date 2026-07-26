#include <doctest/doctest.h>
#include <openmac/hfs.hpp>

#include <map>
#include <string>
#include <vector>

using namespace openmac;

namespace {

std::vector<u8> bytesOf(const std::string& s) {
    return std::vector<u8>(s.begin(), s.end());
}

std::map<std::string, hfs::Item> byName(const std::vector<hfs::Item>& items) {
    std::map<std::string, hfs::Item> m;
    for (const auto& it : items) m[it.name] = it;
    return m;
}

} // namespace

TEST_CASE("a built volume lists back with its tree, metadata, and forks intact") {
    hfs::VolumeBuilder b("Folder Disk");
    const u32 games = b.addDir(2, "Games");
    REQUIRE(games != 0);
    const u32 saves = b.addDir(games, "Saves");
    REQUIRE(saves != 0);
    b.addFile(2, "ReadMe", 0x54455854 /* TEXT */, 0x74747874 /* ttxt */, 0,
              bytesOf("hello from the host"), {});
    b.addFile(games, "Deep App", 0x4150504C /* APPL */, 0x3F3F3F3F, 0x2000,
              bytesOf("data fork bytes"), bytesOf("resource fork bytes"));
    b.addFile(saves, "slot1", 0x3F3F3F3F, 0x3F3F3F3F, 0, bytesOf("s"), {});

    auto img = b.build();
    REQUIRE_MESSAGE(!img.empty(), b.why());

    std::vector<hfs::Item> items;
    REQUIRE(hfs::listVolume(img, items));
    REQUIRE(items.size() == 6);   // root + 2 dirs + 3 files
    CHECK(items[0].isDir);
    CHECK(items[0].id == 2);
    CHECK(items[0].name == "Folder Disk");

    auto m = byName(items);
    REQUIRE(m.count("Games"));
    REQUIRE(m.count("Saves"));
    REQUIRE(m.count("ReadMe"));
    REQUIRE(m.count("Deep App"));
    CHECK(m["Games"].parent == 2);
    CHECK(m["Saves"].parent == m["Games"].id);
    CHECK(m["ReadMe"].type == 0x54455854);
    CHECK(m["ReadMe"].creator == 0x74747874);
    CHECK(m["Deep App"].fdFlags == 0x2000);
    CHECK(m["Deep App"].parent == m["Games"].id);
    CHECK(m["slot1"].parent == m["Saves"].id);

    std::vector<u8> fork;
    REQUIRE(hfs::readFork(img, m["ReadMe"].id, false, fork));
    CHECK(fork == bytesOf("hello from the host"));
    REQUIRE(hfs::readFork(img, m["Deep App"].id, false, fork));
    CHECK(fork == bytesOf("data fork bytes"));
    REQUIRE(hfs::readFork(img, m["Deep App"].id, true, fork));
    CHECK(fork == bytesOf("resource fork bytes"));
    REQUIRE(hfs::readFork(img, m["slot1"].id, false, fork));
    CHECK(fork == bytesOf("s"));
}

TEST_CASE("hundreds of files survive the multi-leaf, indexed catalog") {
    hfs::VolumeBuilder b("Big");
    const u32 dir = b.addDir(2, "Library");
    for (int i = 0; i < 320; ++i) {
        std::string name = "file" + std::to_string(i);
        std::vector<u8> data(64 + (i % 128), u8(i));
        b.addFile(i % 2 ? dir : 2, name, 0x54455854, 0x74747874, 0,
                  std::move(data), {});
    }
    auto img = b.build();
    REQUIRE_MESSAGE(!img.empty(), b.why());

    std::vector<hfs::Item> items;
    REQUIRE(hfs::listVolume(img, items));
    CHECK(items.size() == 322);   // root + dir + 320 files

    // Spot-check contents across the range.
    auto m = byName(items);
    for (int i = 0; i < 320; i += 37) {
        std::string name = "file" + std::to_string(i);
        REQUIRE_MESSAGE(m.count(name), name);
        std::vector<u8> fork;
        REQUIRE(hfs::readFork(img, m[name].id, false, fork));
        REQUIRE(fork.size() == 64u + (i % 128));
        CHECK(fork[0] == u8(i));
    }
}

TEST_CASE("hostile names canonicalize and collide into distinct entries") {
    hfs::VolumeBuilder b("Names");
    b.addFile(2, "caf\xC3\xA9:menu", 0, 0, 0, bytesOf("a"), {});   // UTF-8 é + colon
    b.addFile(2, "caf__%menu", 0, 0, 0, bytesOf("b"), {});
    b.addFile(2, "CAF__%MENU", 0, 0, 0, bytesOf("c"), {});         // case-collides
    auto img = b.build();
    REQUIRE_MESSAGE(!img.empty(), b.why());
    std::vector<hfs::Item> items;
    REQUIRE(hfs::listVolume(img, items));
    REQUIRE(items.size() == 4);
    // All three files exist under distinct names, and every fork reads back.
    int files = 0;
    for (const auto& it : items)
        if (!it.isDir) {
            ++files;
            std::vector<u8> fork;
            REQUIRE(hfs::readFork(img, it.id, false, fork));
            REQUIRE(fork.size() == 1);
        }
    CHECK(files == 3);
}

TEST_CASE("an explicit floppy-sized build packs tight and round-trips") {
    hfs::VolumeBuilder b("Transfer");
    std::vector<u8> big(1200 * 1024);
    for (std::size_t i = 0; i < big.size(); ++i) big[i] = u8(i * 3);
    b.addFile(2, "Game.sit", 0x53495444, 0x53495421, 0, big, {});
    auto img = b.build(1474560);   // exactly a 1.44 MB floppy
    REQUIRE_MESSAGE(!img.empty(), b.why());
    CHECK(img.size() == 1474560);

    std::vector<hfs::Item> items;
    REQUIRE(hfs::listVolume(img, items));
    REQUIRE(items.size() == 2);
    std::vector<u8> fork;
    REQUIRE(hfs::readFork(img, items[1].isDir ? items[0].id : items[1].id, false, fork));
    REQUIRE(fork.size() == big.size());
    CHECK(fork[777] == big[777]);

    // And a file that cannot fit fails with the reason set, not a bad volume.
    hfs::VolumeBuilder b2("Transfer");
    b2.addFile(2, "huge", 0, 0, 0, std::vector<u8>(2 * 1024 * 1024), {});
    CHECK(b2.build(1474560).empty());
    CHECK_FALSE(b2.why().empty());
}

TEST_CASE("the formatter's empty volume still lists as just a root") {
    auto img = hfs::formatVolume(4u * 1024 * 1024, "Blank");
    std::vector<hfs::Item> items;
    REQUIRE(hfs::listVolume(img, items));
    REQUIRE(items.size() == 1);
    CHECK(items[0].isDir);
    CHECK(items[0].name == "Blank");
}
