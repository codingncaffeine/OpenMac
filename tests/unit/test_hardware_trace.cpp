#include <openmac/hardware_trace.hpp>

#include <doctest/doctest.h>

#include <sstream>

using namespace openmac;

namespace {

HardwareTraceEvent eventAt(u64 cycle, u32 pc = 0x1000) {
    HardwareTraceEvent event;
    event.cycle = cycle;
    event.pc = pc;
    event.categories = TraceCpu;
    event.kind = HardwareTraceKind::Instruction;
    event.source = HardwareTraceSource::Cpu;
    return event;
}

} // namespace

TEST_CASE("hardware trace preserves rolling lead-in and reserved post-trigger tail") {
    HardwareTrace trace;
    HardwareTraceConfig config;
    config.capacity = 8;
    config.postTriggerEvents = 2;
    trace.configure(config);

    for (u64 cycle = 1; cycle <= 9; ++cycle) trace.record(eventAt(cycle));
    trace.trigger(10, 0x2000, "test trigger");
    trace.record(eventAt(11));
    trace.record(eventAt(12));
    trace.record(eventAt(13));

    CHECK(trace.triggered());
    CHECK(trace.frozen());
    const auto events = trace.events();
    REQUIRE(events.size() == 8);
    CHECK(events.front().cycle == 5);
    CHECK(events[5].kind == HardwareTraceKind::Trigger);
    CHECK(events[6].cycle == 11);
    CHECK(events[7].cycle == 12);
}

TEST_CASE("hardware trace PC trigger works even when CPU events are filtered") {
    HardwareTrace trace;
    HardwareTraceConfig config;
    config.capacity = 16;
    config.postTriggerEvents = 1;
    config.categories = TraceScsi;
    config.triggerPc = 0xCAFE;
    config.triggerPcHits = 3;
    trace.configure(config);

    trace.record(eventAt(1, 0xCAFE));
    trace.record(eventAt(2, 0xCAFE));
    trace.record(eventAt(3, 0xCAFE));
    CHECK(trace.triggered());
    CHECK(trace.events().size() == 1); // trigger; filtered instructions do not spend tail
    CHECK_FALSE(trace.frozen());
    HardwareTraceEvent scsi = eventAt(4);
    scsi.categories = TraceScsi;
    trace.record(scsi);
    CHECK(trace.frozen());
}

TEST_CASE("hardware trace source and address filters preserve only relevant traffic") {
    HardwareTrace trace;
    HardwareTraceConfig config;
    config.capacity = 16;
    config.sources = hardwareTraceSourceBit(HardwareTraceSource::Via) |
                     hardwareTraceSourceBit(HardwareTraceSource::Oss) |
                     hardwareTraceSourceBit(HardwareTraceSource::Biu);
    config.addressFirst = 0x50000000u;
    config.addressLast = 0x50024000u;
    trace.configure(config);

    auto access = [](HardwareTraceSource source, u32 address) {
        HardwareTraceEvent event;
        event.kind = HardwareTraceKind::Access;
        event.source = source;
        event.categories = TraceBus | TraceIo;
        event.address = address;
        return event;
    };
    trace.record(access(HardwareTraceSource::Via, 0x50000200u));
    trace.record(access(HardwareTraceSource::NuBus, 0x50000200u));
    trace.record(access(HardwareTraceSource::Oss, 0x50026000u));
    trace.record(access(HardwareTraceSource::Biu, 0x50018000u));

    HardwareTraceEvent state;
    state.kind = HardwareTraceKind::State;
    state.source = HardwareTraceSource::Oss;
    state.categories = TraceIo;
    trace.record(state); // Address filtering is intentionally access-only.
    trace.freeze();

    const auto events = trace.events();
    REQUIRE(events.size() == 3);
    CHECK(events[0].source == HardwareTraceSource::Via);
    CHECK(events[1].source == HardwareTraceSource::Biu);
    CHECK(events[2].kind == HardwareTraceKind::State);
}

TEST_CASE("hardware trace JSONL is deterministic and includes device snapshots") {
    HardwareTrace trace;
    HardwareTraceConfig config;
    config.capacity = 8;
    trace.configure(config);
    HardwareTraceEvent event = eventAt(42);
    event.categories = TraceBus | TraceScsi;
    event.kind = HardwareTraceKind::Access;
    event.source = HardwareTraceSource::Ncr5380;
    event.width = 1;
    event.flags = 1;
    event.address = 0x5000E020;
    event.value = 0x12;
    event.correlation = 7;
    event.scsi.phase = 3;
    event.scsi.flags = 0x19;
    event.scsi.cdbLength = 6;
    event.scsi.cdb[0] = 0x08;
    event.swim.fifoOccupancy = 2;
    event.swim.fifoTail = 0x5A;
    event.swim.readSynced = 1;
    event.swim.requestedBlock = 606;
    event.swim.expectedTrack = 16;
    event.swim.expectedSide = 1;
    event.swim.expectedSector = 13;
    event.iop.opcode = 0xAD;
    event.video.mode = 0x83;
    event.video.vblankAssertions = 9;
    event.video.vblankAcks = 8;
    event.video.flags = 0x2F;
    trace.record(event);
    trace.freeze();

    std::ostringstream output;
    REQUIRE(trace.writeJsonl(output));
    CHECK(output.str().find("\"schema\":\"openmac.hardware-trace\"") !=
          std::string::npos);
    CHECK(output.str().find("\"source\":\"ncr5380\"") !=
          std::string::npos);
    CHECK(output.str().find("\"cdb\":[8,0,0,0,0,0]") !=
          std::string::npos);
    CHECK(output.str().find("\"correlation\":7") != std::string::npos);
    CHECK(output.str().find("\"fifo_occupancy\":2") != std::string::npos);
    CHECK(output.str().find("\"fifo_tail\":90") != std::string::npos);
    CHECK(output.str().find("\"read_synced\":1") != std::string::npos);
    CHECK(output.str().find("\"requested_block\":606") != std::string::npos);
    CHECK(output.str().find("\"expected_track\":16") != std::string::npos);
    CHECK(output.str().find("\"opcode\":173") != std::string::npos);
    CHECK(output.str().find("\"video\":{\"mode\":131") != std::string::npos);
    CHECK(output.str().find("\"vblank_assertions\":9") != std::string::npos);
    CHECK(output.str().find("\"serial_command\":0") != std::string::npos);
    CHECK(output.str().find("\"timing_echo_writes\":0") != std::string::npos);
    CHECK(output.str().find("\"mailbox\":{}") != std::string::npos);
}

TEST_CASE("hardware trace emits complete decoded mailbox payloads") {
    HardwareTrace trace;
    HardwareTraceConfig config;
    config.capacity = 8;
    trace.configure(config);
    HardwareTraceEvent event = eventAt(7);
    event.categories = TraceIop | TraceSwim;
    event.kind = HardwareTraceKind::Mailbox;
    event.source = HardwareTraceSource::IsmIop;
    event.mailbox.length = 32;
    event.mailbox.operation = 0x0B;
    event.mailbox.channel = 1;
    event.mailbox.flags = 0x17;
    event.mailbox.block = 606;
    event.mailbox.blockCount = 1;
    event.mailbox.expectedTrack = 16;
    event.mailbox.expectedSide = 1;
    event.mailbox.expectedSector = 13;
    event.mailbox.payload[0] = 0x0B;
    event.mailbox.payload[8] = 0x00;
    event.mailbox.payload[9] = 0x00;
    event.mailbox.payload[10] = 0x02;
    event.mailbox.payload[11] = 0x5E;
    trace.record(event);
    trace.freeze();

    std::ostringstream output;
    REQUIRE(trace.writeJsonl(output));
    CHECK(output.str().find("\"operation\":11") != std::string::npos);
    CHECK(output.str().find("\"block\":606") != std::string::npos);
    CHECK(output.str().find("\"expected_track\":16") != std::string::npos);
    CHECK(output.str().find("\"expected_side\":1") != std::string::npos);
    CHECK(output.str().find("\"expected_sector\":13") != std::string::npos);
    CHECK(output.str().find("\"payload\":[11,0,0,0,0,0,0,0,0,0,2,94") !=
          std::string::npos);
}

TEST_CASE("hardware trace IOP flight recorder merges bounded lead-in on trigger") {
    HardwareTrace trace;
    HardwareTraceConfig config;
    config.capacity = 12;
    config.postTriggerEvents = 2;
    config.iopFlightEvents = 3;
    trace.configure(config);

    HardwareTraceEvent protocol = eventAt(1);
    protocol.categories = TraceSwim;
    protocol.kind = HardwareTraceKind::State;
    protocol.source = HardwareTraceSource::Swim;
    trace.record(protocol);
    for (u64 cycle = 2; cycle <= 6; ++cycle) {
        HardwareTraceEvent instruction = eventAt(cycle, static_cast<u32>(0x7000 + cycle));
        instruction.categories = TraceIop;
        instruction.source = HardwareTraceSource::IsmIop;
        trace.recordIopFlight(instruction);
    }
    trace.trigger(7, 0x1234, "flight trigger");

    const auto events = trace.events();
    REQUIRE(events.size() == 5);
    CHECK(events[0].cycle == 1);
    CHECK(events[1].cycle == 4);
    CHECK(events[2].cycle == 5);
    CHECK(events[3].cycle == 6);
    CHECK(events[4].kind == HardwareTraceKind::Trigger);
}

TEST_CASE("hardware trace carries correlation onto the trigger boundary") {
    HardwareTrace trace;
    HardwareTraceConfig config;
    config.capacity = 8;
    config.postTriggerEvents = 0;
    trace.configure(config);
    trace.trigger(42, 0x1234, "media fault", 19);

    const auto events = trace.events();
    REQUIRE(events.size() == 1);
    CHECK(events[0].kind == HardwareTraceKind::Trigger);
    CHECK(events[0].correlation == 19);
}
