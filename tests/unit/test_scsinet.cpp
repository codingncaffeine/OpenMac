#include <doctest/doctest.h>

#include "../../core/src/machine/scsinet.hpp"

#include <string>
#include <vector>

using namespace openmac;

namespace {

struct Cmd {
    u8 status;
    std::vector<u8> data;
    u32 writeBytes;
};

Cmd run(ScsiEthernet& net, std::initializer_list<u8> bytes) {
    u8 cdb[12] = {};
    std::size_t i = 0;
    for (u8 b : bytes) cdb[i++] = b;
    Cmd r{};
    r.status = net.execute(cdb, r.data, r.writeBytes);
    return r;
}

std::vector<u8> frameOf(std::size_t len, u8 fill) {
    std::vector<u8> f(len, fill);
    return f;
}

} // namespace

TEST_CASE("the adapter answers INQUIRY as the Dayna SCSI/Link") {
    ScsiEthernet net;
    net.setAttached(true, 4);
    auto r = run(net, {0x12, 0, 0, 0, 36, 0});
    REQUIRE(r.data.size() == 36);
    CHECK(r.data[0] == 0x03);   // processor device
    CHECK(std::string(r.data.begin() + 8, r.data.begin() + 16) == "Dayna   ");
    CHECK(std::string(r.data.begin() + 16, r.data.begin() + 32) == "SCSI/Link       ");
    CHECK(net.id() == 4);
    CHECK(net.present());
}

TEST_CASE("guest sends land in the host drain queue") {
    ScsiEthernet net;
    net.setAttached(true, 4);
    auto frame = frameOf(64, 0xAB);
    auto s = run(net, {0x0A, 0, 0, 0, 64, 0});
    CHECK(s.status == 0x00);
    REQUIRE(s.writeBytes == 64);
    net.acceptWrite(frame);
    std::vector<u8> out;
    REQUIRE(net.drainFrame(out));
    CHECK(out == frame);
    CHECK_FALSE(net.drainFrame(out));
}

TEST_CASE("host frames reach the guest through $08 reads with the DaynaPORT header") {
    ScsiEthernet net;
    net.setAttached(true, 4);
    run(net, {0x0E, 0, 0, 0, 0, 0x80});   // guest enables the interface

    auto f1 = frameOf(60, 0x11), f2 = frameOf(90, 0x22);
    REQUIRE(net.injectFrame(f1.data(), static_cast<u32>(f1.size())));
    REQUIRE(net.injectFrame(f2.data(), static_cast<u32>(f2.size())));

    auto r1 = run(net, {0x08, 0, 0, 0x06, 0x00, 0});   // alloc 1536
    REQUIRE(r1.data.size() == 6 + 60);
    CHECK(r1.data[0] == 0);
    CHECK(r1.data[1] == 60);
    CHECK(r1.data[5] == 0x10);   // another packet waits
    CHECK(r1.data[6] == 0x11);

    auto r2 = run(net, {0x08, 0, 0, 0x06, 0x00, 0});
    REQUIRE(r2.data.size() == 6 + 90);
    CHECK(r2.data[5] == 0x00);   // queue now empty
    CHECK(r2.data[6] == 0x22);

    auto r3 = run(net, {0x08, 0, 0, 0x06, 0x00, 0});
    REQUIRE(r3.data.size() == 6);   // idle: all-zero header
    CHECK(r3.data[0] == 0);
    CHECK(r3.data[1] == 0);
}

TEST_CASE("a disabled interface reads idle and statistics carry the MAC") {
    ScsiEthernet net;
    net.setAttached(true, 4);
    auto f = frameOf(60, 0x33);
    net.injectFrame(f.data(), static_cast<u32>(f.size()));
    // Interface never enabled: reads answer idle, the frame stays queued.
    auto r = run(net, {0x08, 0, 0, 0x06, 0x00, 0});
    CHECK(r.data.size() == 6);

    const u8 mac[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
    net.setMac(mac);
    auto st = run(net, {0x09, 0, 0, 0, 18, 0});
    REQUIRE(st.data.size() == 18);
    CHECK(st.data[0] == 0x02);
    CHECK(st.data[5] == 0x55);
}
