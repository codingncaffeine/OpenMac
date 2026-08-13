#include "../tracecmp/compare.hpp"

#include <doctest/doctest.h>

#include <sstream>

using namespace openmac::tracecmp;

namespace {

std::vector<Event> events(const char* text) {
    std::istringstream input(text);
    std::vector<Event> parsed;
    std::string error;
    REQUIRE(parse(input, parsed, error));
    return parsed;
}

} // namespace

TEST_CASE("trace comparator normalizes generic hardware capture fields") {
    const auto actual = events(
        "{\"schema\":\"openmac.hardware-trace\",\"version\":1}\n"
        "{\"seq\":4,\"cycle\":1000,\"pc\":4096,\"kind\":\"access\","
        "\"source\":\"ncr5380\",\"width\":1,\"write\":true,"
        "\"address\":80,\"value\":3,\"detail\":\"host text\"}\n"
        "{\"seq\":5,\"cycle\":1020,\"pc\":4098,\"kind\":\"state\","
        "\"source\":\"swim\",\"swim\":{\"mode\":72}}\n");
    const auto reference = events(
        "{\"cpu_cycle\":50,\"pc\":4096,\"source\":\"NCR-5380\","
        "\"access_width\":1,\"direction\":\"W\",\"addr\":80,\"data\":3}\n"
        "{\"cpu_cycle\":70,\"pc\":4098,\"kind\":\"state\","
        "\"source\":\"SWIM\",\"swim\":{\"mode\":72}}\n");
    const Result result = compare(actual, reference);
    CHECK_MESSAGE(result.equal, result.report);
    CHECK(result.matchedEvents == 2);
}

TEST_CASE("trace comparator reports exact field and surrounding events") {
    const auto actual = events(
        "{\"cycle\":1,\"kind\":\"state\",\"source\":\"swim\",\"value\":1}\n"
        "{\"cycle\":2,\"kind\":\"state\",\"source\":\"swim\",\"value\":9}\n"
        "{\"cycle\":3,\"kind\":\"state\",\"source\":\"swim\",\"value\":3}\n");
    const auto reference = events(
        "{\"cycle\":1,\"kind\":\"state\",\"source\":\"swim\",\"value\":1}\n"
        "{\"cycle\":2,\"kind\":\"state\",\"source\":\"swim\",\"value\":2}\n"
        "{\"cycle\":3,\"kind\":\"state\",\"source\":\"swim\",\"value\":3}\n");
    const Result result = compare(actual, reference);
    CHECK_FALSE(result.equal);
    CHECK(result.report.find("field value: reference=2, actual=9") != std::string::npos);
    CHECK(result.report.find("actual context") != std::string::npos);
}

TEST_CASE("trace comparator has opt-in bounded resynchronization") {
    const auto actual = events(
        "{\"cycle\":10,\"kind\":\"state\",\"source\":\"swim\",\"value\":1}\n"
        "{\"cycle\":11,\"kind\":\"state\",\"source\":\"other\",\"value\":99}\n"
        "{\"cycle\":20,\"kind\":\"state\",\"source\":\"swim\",\"value\":2}\n");
    const auto reference = events(
        "{\"cycle\":0,\"kind\":\"state\",\"source\":\"swim\",\"value\":1}\n"
        "{\"cycle\":10,\"kind\":\"state\",\"source\":\"swim\",\"value\":2}\n");
    Options options;
    options.resyncLookahead = 1;
    const Result result = compare(actual, reference, options);
    CHECK_MESSAGE(result.equal, result.report);
    CHECK(result.skippedActualEvents == 1);
}
