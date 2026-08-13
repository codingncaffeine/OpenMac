#include "compare.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace openmac::tracecmp;

namespace {

void usage() {
    std::cerr
        << "usage: openmac_tracecmp <actual.jsonl> <reference.jsonl> [options]\n"
        << "  --cycle-tolerance N  permit N emulated CPU cycles of timing drift\n"
        << "  --absolute-cycles     compare cycle counters without origin normalization\n"
        << "  --resync N            skip up to N actual-only events to regain alignment\n"
        << "  --source NAME         retain one source (repeatable)\n"
        << "  --kind NAME           retain one event kind (repeatable)\n"
        << "  --ignore FIELD        ignore a flattened JSON field (repeatable)\n"
        << "  --context N           actual events shown around a mismatch\n";
}

std::uint64_t parseNumber(const char* text) {
    char* end = nullptr;
    const unsigned long long value = std::strtoull(text, &end, 0);
    if (!end || *end != '\0') throw std::invalid_argument("invalid number");
    return static_cast<std::uint64_t>(value);
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        usage();
        return 2;
    }
    Options options;
    try {
        for (int index = 3; index < argc; ++index) {
            const std::string option = argv[index];
            if (option == "--absolute-cycles") options.absoluteCycles = true;
            else if (option == "--cycle-tolerance" && index + 1 < argc)
                options.cycleTolerance = parseNumber(argv[++index]);
            else if (option == "--resync" && index + 1 < argc)
                options.resyncLookahead = static_cast<std::size_t>(parseNumber(argv[++index]));
            else if (option == "--context" && index + 1 < argc)
                options.context = static_cast<std::size_t>(parseNumber(argv[++index]));
            else if (option == "--source" && index + 1 < argc)
                options.sources.insert(normalizeSource(argv[++index]));
            else if (option == "--kind" && index + 1 < argc)
                options.kinds.insert(lower(argv[++index]));
            else if (option == "--ignore" && index + 1 < argc)
                options.ignoredFields.insert(argv[++index]);
            else {
                std::cerr << "unknown or incomplete option: " << option << '\n';
                usage();
                return 2;
            }
        }
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 2;
    }

    std::ifstream actualFile(argv[1], std::ios::binary);
    std::ifstream referenceFile(argv[2], std::ios::binary);
    if (!actualFile || !referenceFile) {
        std::cerr << "cannot open " << (!actualFile ? argv[1] : argv[2]) << '\n';
        return 2;
    }
    std::vector<Event> actual, reference;
    std::string error;
    if (!parse(actualFile, actual, error)) {
        std::cerr << "actual trace: " << error << '\n';
        return 2;
    }
    if (!parse(referenceFile, reference, error)) {
        std::cerr << "reference trace: " << error << '\n';
        return 2;
    }
    const Result result = compare(std::move(actual), std::move(reference), options);
    std::cout << result.report;
    return result.equal ? 0 : 1;
}
