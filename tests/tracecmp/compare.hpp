#pragma once

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <istream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace openmac::tracecmp {

struct Event {
    std::size_t line = 0;
    std::unordered_map<std::string, std::string> fields;
};

struct Options {
    std::uint64_t cycleTolerance = 0;
    bool absoluteCycles = false;
    std::size_t resyncLookahead = 0;
    std::size_t context = 2;
    std::unordered_set<std::string> sources;
    std::unordered_set<std::string> kinds;
    std::unordered_set<std::string> ignoredFields{
        "seq", "sequence", "categories", "detail"};
};

struct Result {
    bool equal = false;
    std::size_t referenceEvents = 0;
    std::size_t actualEvents = 0;
    std::size_t matchedEvents = 0;
    std::size_t skippedActualEvents = 0;
    std::size_t referenceIndex = 0;
    std::size_t actualIndex = 0;
    std::string report;
};

inline std::string lower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        if (ch == '_' || ch == ' ') return '-';
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

inline std::string normalizeSource(std::string text) {
    text = lower(std::move(text));
    if (text == "ncr" || text == "ncr-5380" || text == "5380")
        return "ncr5380";
    if (text == "scsidma" || text == "scsi-dma-asic") return "scsi-dma";
    if (text == "ism" || text == "swim-iop" || text == "floppy-iop")
        return "ism-iop";
    if (text == "scc-iop" || text == "serial-iop") return "scc-iop";
    return text;
}

inline std::string scalar(const nlohmann::json& value) {
    if (value.is_boolean()) return value.get<bool>() ? "true" : "false";
    if (value.is_number_unsigned()) return std::to_string(value.get<std::uint64_t>());
    if (value.is_number_integer()) return std::to_string(value.get<std::int64_t>());
    if (value.is_number_float()) {
        std::ostringstream output;
        output << std::setprecision(17) << value.get<double>();
        return output.str();
    }
    if (value.is_string()) return value.get<std::string>();
    return value.dump();
}

inline std::optional<std::uint64_t> number(const std::string& text) {
    if (text.empty() || text.front() == '-') return std::nullopt;
    char* end = nullptr;
    const unsigned long long value = std::strtoull(text.c_str(), &end, 0);
    if (!end || *end != '\0') return std::nullopt;
    return static_cast<std::uint64_t>(value);
}

inline void flatten(const nlohmann::json& value, const std::string& prefix,
                    std::unordered_map<std::string, std::string>& fields) {
    if (value.is_object()) {
        for (auto item = value.begin(); item != value.end(); ++item) {
            const std::string name = prefix.empty()
                ? item.key() : prefix + "." + item.key();
            flatten(item.value(), name, fields);
        }
        return;
    }
    if (value.is_array()) {
        fields[prefix] = value.dump();
        return;
    }
    fields[prefix] = scalar(value);
}

inline void alias(std::unordered_map<std::string, std::string>& fields,
                  const char* from, const char* to) {
    const auto found = fields.find(from);
    if (found == fields.end() || fields.count(to)) return;
    fields[to] = found->second;
    fields.erase(found);
}

inline bool parse(std::istream& input, std::vector<Event>& events,
                  std::string& error) {
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (line.empty() || line[0] == '#') continue;
        nlohmann::json object;
        try {
            object = nlohmann::json::parse(line);
        } catch (const std::exception& exception) {
            error = "line " + std::to_string(lineNumber) + ": " + exception.what();
            return false;
        }
        if (!object.is_object()) {
            error = "line " + std::to_string(lineNumber) + ": expected a JSON object";
            return false;
        }
        if (object.contains("schema") && !object.contains("kind")) continue;

        Event event;
        event.line = lineNumber;
        flatten(object, "", event.fields);
        alias(event.fields, "cpu_cycle", "cycle");
        alias(event.fields, "cpu-cycle", "cycle");
        alias(event.fields, "addr", "address");
        alias(event.fields, "data", "value");
        alias(event.fields, "access_width", "width");
        alias(event.fields, "access-width", "width");
        alias(event.fields, "rw", "direction");
        if (const auto direction = event.fields.find("direction");
            direction != event.fields.end() && !event.fields.count("write")) {
            const std::string normalized = lower(direction->second);
            event.fields["write"] =
                normalized == "w" || normalized == "write" || normalized == "1"
                    ? "true" : "false";
            event.fields.erase(direction);
        }
        if (auto source = event.fields.find("source"); source != event.fields.end())
            source->second = normalizeSource(source->second);
        if (auto kind = event.fields.find("kind"); kind != event.fields.end())
            kind->second = lower(kind->second);
        else if (event.fields.count("address"))
            event.fields["kind"] = "access";
        events.push_back(std::move(event));
    }
    return true;
}

inline bool selected(const Event& event, const Options& options) {
    if (!options.sources.empty()) {
        const auto source = event.fields.find("source");
        if (source == event.fields.end() || !options.sources.count(source->second))
            return false;
    }
    if (!options.kinds.empty()) {
        const auto kind = event.fields.find("kind");
        if (kind == event.fields.end() || !options.kinds.count(kind->second))
            return false;
    }
    return true;
}

inline std::string describe(const Event& event) {
    const auto get = [&event](const char* name) {
        const auto found = event.fields.find(name);
        return found == event.fields.end() ? std::string("-") : found->second;
    };
    return "line " + std::to_string(event.line) + " cycle=" + get("cycle") +
           " pc=" + get("pc") + " " + get("kind") + "/" + get("source") +
           " address=" + get("address") + " value=" + get("value");
}

inline std::vector<std::string> differences(
    const Event& actual, const Event& reference, const Options& options,
    std::optional<std::uint64_t> actualOrigin,
    std::optional<std::uint64_t> referenceOrigin) {
    std::vector<std::string> result;
    for (const auto& [name, expected] : reference.fields) {
        if (options.ignoredFields.count(name)) continue;
        const auto found = actual.fields.find(name);
        if (found == actual.fields.end()) {
            result.push_back(name + ": reference=" + expected + ", actual=<missing>");
            continue;
        }
        if (name == "cycle") {
            const auto actualCycle = number(found->second);
            const auto referenceCycle = number(expected);
            if (actualCycle && referenceCycle) {
                std::uint64_t left = *actualCycle;
                std::uint64_t right = *referenceCycle;
                if (!options.absoluteCycles && actualOrigin && referenceOrigin) {
                    left -= *actualOrigin;
                    right -= *referenceOrigin;
                }
                const std::uint64_t delta = left > right ? left - right : right - left;
                if (delta <= options.cycleTolerance) continue;
                result.push_back("cycle: reference=" + std::to_string(right) +
                                 ", actual=" + std::to_string(left) +
                                 ", delta=" + std::to_string(delta));
                continue;
            }
        }
        if (found->second != expected)
            result.push_back(name + ": reference=" + expected +
                             ", actual=" + found->second);
    }
    std::sort(result.begin(), result.end());
    return result;
}

inline Result compare(std::vector<Event> actual, std::vector<Event> reference,
                      const Options& options = {}) {
    actual.erase(std::remove_if(actual.begin(), actual.end(),
        [&](const Event& event) { return !selected(event, options); }), actual.end());
    reference.erase(std::remove_if(reference.begin(), reference.end(),
        [&](const Event& event) { return !selected(event, options); }), reference.end());

    Result result;
    result.actualEvents = actual.size();
    result.referenceEvents = reference.size();
    const auto origin = [](const std::vector<Event>& events) -> std::optional<std::uint64_t> {
        if (events.empty()) return std::nullopt;
        const auto cycle = events.front().fields.find("cycle");
        return cycle == events.front().fields.end() ? std::nullopt
                                                    : number(cycle->second);
    };
    const auto actualOrigin = origin(actual);
    const auto referenceOrigin = origin(reference);

    std::size_t ai = 0;
    std::size_t ri = 0;
    while (ai < actual.size() && ri < reference.size()) {
        auto diff = differences(actual[ai], reference[ri], options,
                                actualOrigin, referenceOrigin);
        if (diff.empty()) {
            ++ai; ++ri; ++result.matchedEvents;
            continue;
        }

        bool resynced = false;
        for (std::size_t skip = 1;
             skip <= options.resyncLookahead && ai + skip < actual.size(); ++skip) {
            if (differences(actual[ai + skip], reference[ri], options,
                            actualOrigin, referenceOrigin).empty()) {
                result.skippedActualEvents += skip;
                ai += skip;
                resynced = true;
                break;
            }
        }
        if (resynced) continue;

        result.referenceIndex = ri;
        result.actualIndex = ai;
        std::ostringstream report;
        report << "trace mismatch at reference event " << ri
               << " and actual event " << ai << '\n'
               << "  reference: " << describe(reference[ri]) << '\n'
               << "  actual:    " << describe(actual[ai]) << '\n';
        for (const std::string& item : diff) report << "  field " << item << '\n';
        const std::size_t begin = ai > options.context ? ai - options.context : 0;
        const std::size_t end = std::min(actual.size(), ai + options.context + 1);
        report << "  actual context:\n";
        for (std::size_t index = begin; index < end; ++index)
            report << (index == ai ? "    > " : "      ") << index << ": "
                   << describe(actual[index]) << '\n';
        result.report = report.str();
        return result;
    }

    if (ri != reference.size() || ai != actual.size()) {
        result.referenceIndex = ri;
        result.actualIndex = ai;
        std::ostringstream report;
        report << "trace length mismatch after " << result.matchedEvents
               << " matched events: reference has " << (reference.size() - ri)
               << " remaining, actual has " << (actual.size() - ai) << " remaining\n";
        if (ri < reference.size())
            report << "  next reference: " << describe(reference[ri]) << '\n';
        if (ai < actual.size())
            report << "  next actual:    " << describe(actual[ai]) << '\n';
        result.report = report.str();
        return result;
    }

    result.equal = true;
    std::ostringstream report;
    report << "traces match: " << result.matchedEvents << " events";
    if (result.skippedActualEvents)
        report << ", " << result.skippedActualEvents
               << " actual-only events skipped during resynchronization";
    report << '\n';
    result.report = report.str();
    return result;
}

} // namespace openmac::tracecmp
