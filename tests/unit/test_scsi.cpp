#include <doctest/doctest.h>

#include "../../core/src/machine/scsi.hpp"
#include "../../core/src/machine/iifx/scsi_dma.hpp"

#include <vector>

using namespace openmac;

namespace {

// A raw disk image whose block 0 carries the Apple Driver Descriptor Map
// signature ('ER') -- the mark that puts a ScsiDisk on the bus.
std::vector<u8> ddmImage(std::size_t blocks) {
    std::vector<u8> img(blocks * 512u, 0);
    img[0] = 0x45;   // 'E'
    img[1] = 0x52;   // 'R'
    return img;
}

// Drive a selection the way the ROM does: target ID (plus the initiator's own
// ID 7) on the data bus, then /SEL with /BSY released. Returns true if a target
// answered (bus entered Command phase).
bool select(Ncr5380& c, int id) {
    c.write(0, static_cast<u8>(0x80u | (1u << id)));   // ODR: initiator 7 + target
    c.write(1, 0x04);                                  // ICR: /SEL, no /BSY
    const bool taken = c.phase() == Ncr5380::Command;
    c.write(1, 0x00);                                  // release /SEL
    return taken;
}

// Push a CDB into the Command phase, one byte per ODR write.
void sendByte(Ncr5380& c, u8 value) {
    c.write(0, value);
    c.write(1, 0x10);   // assert /ACK: target accepts the driven byte
    c.write(1, 0x00);   // release /ACK: target may advance the phase
}

u8 receiveByte(Ncr5380& c) {
    const u8 value = c.read(0);
    c.write(1, 0x10);   // accept the sampled target byte
    c.write(1, 0x00);
    return value;
}

template <std::size_t N>
void sendCdb(Ncr5380& c, const u8 (&cdb)[N]) {
    for (u8 b : cdb) sendByte(c, b);
}

// Read the whole Data-In transfer, then consume Status and Message In so the
// bus returns to Bus Free. Returns the data bytes.
std::vector<u8> drainDataIn(Ncr5380& c) {
    std::vector<u8> data;
    while (c.phase() == Ncr5380::DataIn) data.push_back(receiveByte(c));
    CHECK(c.phase() == Ncr5380::Status);
    CHECK(receiveByte(c) == 0x00);       // GOOD
    CHECK(c.phase() == Ncr5380::MsgIn);
    CHECK(receiveByte(c) == 0x00);       // Command Complete
    CHECK(c.phase() == Ncr5380::BusFree);
    return data;
}

// A second device on the bus with an unmistakable INQUIRY answer.
struct TestTarget : ScsiTarget {
    int busId = 5;
    bool here = true;
    std::vector<u8> accepted;

    bool present() const override { return here; }
    int  id() const override { return busId; }
    u8 execute(const u8* cdb, std::vector<u8>& out, u32& writeBytes) override {
        if (cdb[0] == 0x12) { out.assign(4, 0xA5); return 0x00; }   // INQUIRY
        if (cdb[0] == 0x0A) { writeBytes = 512; return 0x00; }      // WRITE(6)
        return 0x02;
    }
    void acceptWrite(const std::vector<u8>& data) override { accepted = data; }
};

} // namespace

TEST_CASE("selection picks the present target whose ID bit is on the bus") {
    Ncr5380 c;
    auto img = ddmImage(8);
    c.disk.attach(&img, 0);
    TestTarget extra;
    c.addTarget(&extra);

    SUBCASE("built-in disk at ID 0") {
        REQUIRE(select(c, 0));
        const u8 inquiry[6] = {0x12, 0, 0, 0, 36, 0};
        sendCdb(c, inquiry);
        auto data = drainDataIn(c);
        REQUIRE(data.size() == 36);
        CHECK(data[0] == 0x00);   // direct access
        // Apple's disk tools refuse a drive whose vendor is not APPLE.
        CHECK(std::string(data.begin() + 8, data.begin() + 16) == "APPLE   ");
    }

    SUBCASE("added target at ID 5") {
        REQUIRE(select(c, 5));
        const u8 inquiry[6] = {0x12, 0, 0, 0, 36, 0};
        sendCdb(c, inquiry);
        auto data = drainDataIn(c);
        REQUIRE(data.size() == 4);
        CHECK(data[0] == 0xA5);
    }

    SUBCASE("nobody home at an unclaimed ID") {
        CHECK_FALSE(select(c, 3));
        CHECK(c.phase() == Ncr5380::BusFree);
    }

    SUBCASE("an absent target stops answering selection") {
        extra.here = false;
        CHECK_FALSE(select(c, 5));
        extra.here = true;
        CHECK(select(c, 5));
    }
}

TEST_CASE("a bare volume with no driver map stays off the bus") {
    Ncr5380 c;
    std::vector<u8> img(8 * 512u, 0);   // no 'ER' in block 0
    c.disk.attach(&img, 0);
    CHECK_FALSE(select(c, 0));
}

TEST_CASE("write stages Data-Out and commits through the selected target") {
    Ncr5380 c;
    auto img = ddmImage(8);
    c.disk.attach(&img, 0);

    REQUIRE(select(c, 0));
    const u8 write6[6] = {0x0A, 0, 0, 1, 1, 0};   // one block at LBA 1
    sendCdb(c, write6);
    REQUIRE(c.phase() == Ncr5380::DataOut);
    for (int i = 0; i < 512; ++i) sendByte(c, static_cast<u8>(i & 0xFF));
    CHECK(c.phase() == Ncr5380::Status);
    CHECK(receiveByte(c) == 0x00);
    CHECK(receiveByte(c) == 0x00);
    CHECK(c.phase() == Ncr5380::BusFree);
    CHECK(img[512] == 0x00);
    CHECK(img[513] == 0x01);
    CHECK(img[512 + 255] == 0xFF);

    // And read it back through READ(6).
    REQUIRE(select(c, 0));
    const u8 read6[6] = {0x08, 0, 0, 1, 1, 0};
    sendCdb(c, read6);
    auto data = drainDataIn(c);
    REQUIRE(data.size() == 512);
    CHECK(data[37] == 37);
}

TEST_CASE("the write path reaches an added target's acceptWrite") {
    Ncr5380 c;
    TestTarget extra;
    c.addTarget(&extra);

    REQUIRE(select(c, 5));
    const u8 write6[6] = {0x0A, 0, 0, 0, 1, 0};
    sendCdb(c, write6);
    REQUIRE(c.phase() == Ncr5380::DataOut);
    for (int i = 0; i < 512; ++i) sendByte(c, 0x5A);
    CHECK(receiveByte(c) == 0x00);
    CHECK(receiveByte(c) == 0x00);
    REQUIRE(extra.accepted.size() == 512);
    CHECK(extra.accepted[0] == 0x5A);
}

TEST_CASE("5380 DMA phase mismatch latches IRQ until reset-interrupt is read") {
    Ncr5380 c;
    auto img = ddmImage(8);
    c.disk.attach(&img, 0);

    REQUIRE(select(c, 0));
    const u8 read6[6] = {0x08, 0, 0, 0, 1, 0};
    sendCdb(c, read6);
    REQUIRE(c.phase() == Ncr5380::DataIn);

    c.write(2, 0x02);   // MR: DMA mode
    c.write(3, 0x01);   // TCR: Data In phase
    c.write(7, 0x00);   // Start DMA Initiator Receive
    for (int i = 0; i < 512; ++i) (void)c.read(6);

    CHECK(c.phase() == Ncr5380::Status);
    CHECK(c.irqAsserted());
    CHECK((c.read(5) & 0x10) != 0);   // BSR IRQ active
    CHECK(c.read(7) == 0);            // Reset Parity/Interrupt
    CHECK_FALSE(c.irqAsserted());
    CHECK((c.read(5) & 0x10) == 0);
}

TEST_CASE("5380 start-DMA does not retroactively interrupt for an asserted REQ") {
    Ncr5380 c;
    auto img = ddmImage(8);
    c.disk.attach(&img, 0);

    REQUIRE(select(c, 0));
    const u8 read6[6] = {0x08, 0, 0, 0, 1, 0};
    sendCdb(c, read6);
    REQUIRE(c.phase() == Ncr5380::DataIn);
    REQUIRE((c.read(4) & 0x20) != 0);   // target already asserts /REQ

    c.write(2, 0x02);                  // MR: DMA mode
    c.write(3, 0x00);                  // deliberately program Data-Out
    c.write(5, 0x00);                  // Start DMA Send

    CHECK_FALSE(c.phaseMatches());
    CHECK_FALSE(c.irqAsserted());       // no new /REQ assertion occurred
    CHECK((c.read(5) & 0x10) == 0);
}

TEST_CASE("IIfx SCSI ASIC performs paced misaligned true DMA and exposes EOP") {
    Ncr5380 c;
    auto img = ddmImage(8);
    for (std::size_t i = 2; i < 512; ++i) img[i] = static_cast<u8>(i);
    c.disk.attach(&img, 0);
    REQUIRE(select(c, 0));
    const u8 read6[6] = {0x08, 0, 0, 0, 1, 0};
    sendCdb(c, read6);
    REQUIRE(c.phase() == Ncr5380::DataIn);

    IifxScsiDma dma(c);
    std::vector<u8> memory(1024, 0xCC);
    dma.readMemory = [&](u32 address) { return memory.at(address); };
    dma.writeMemory = [&](u32 address, u8 value) { memory.at(address) = value; };

    dma.write32(IifxScsiDma::Control, 0x00000003); // DMA + SCSI IRQ output
    dma.write32(IifxScsiDma::Address, 3);          // deliberately misaligned
    dma.write32(IifxScsiDma::Count, 512);
    dma.write8(0x20, 0x0A);                       // DMA mode + EOP IRQ
    dma.write8(0x30, 0x01);                       // expected Data-In phase
    dma.write8(0x70, 0);                          // Start DMA Initiator Receive

    CHECK(dma.dmaActive());
    dma.tick(6800);
    CHECK(dma.count() > 0);                       // 3 MB/s pacing, not instant
    dma.tick(100);

    CHECK_FALSE(dma.dmaActive());
    CHECK(dma.count() == 0);
    CHECK(dma.address() == 515);
    CHECK(memory[3] == 0x45);
    CHECK(memory[4] == 0x52);
    CHECK(memory[5] == 0x02);
    CHECK(memory[514] == 0xFF);
    CHECK(memory[2] == 0xCC);
    CHECK(memory[515] == 0xCC);
    CHECK((dma.control() & (1u << 6)) != 0);       // SCSI/EOP pending
    CHECK(dma.irqAsserted());

    CHECK(dma.read8(0x70) == 0);                  // Reset parity/interrupt
    CHECK((dma.control() & (1u << 6)) == 0);
    CHECK_FALSE(dma.irqAsserted());
}

TEST_CASE("IIfx SCSI ASIC watchdog and FIFO loopback follow the control register") {
    Ncr5380 c;
    IifxScsiDma dma(c);

    dma.write32(IifxScsiDma::Control, 1u << 2);   // watchdog IRQ enable
    dma.write32(IifxScsiDma::Watchdog, 2);
    dma.tick(3);
    CHECK((dma.control() & (1u << 7)) == 0);
    dma.tick(1);
    CHECK((dma.control() & (1u << 7)) != 0);
    CHECK(dma.irqAsserted());
    CHECK(dma.read32(IifxScsiDma::Watchdog) == 2);
    CHECK((dma.control() & (1u << 7)) == 0);

    dma.write32(IifxScsiDma::Control, 1u << 5);   // FIFO/counter test mode
    dma.write32(IifxScsiDma::Fifo, 0x12345678);
    CHECK(dma.read32(IifxScsiDma::Fifo) == 0x12345678);
    CHECK((dma.control() & (1u << 4)) != 0);       // bytes remain in FIFO
}
