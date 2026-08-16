#include <openmac/iifx.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

using namespace openmac;

namespace {

constexpr std::size_t kMiB = 1024u * 1024u;

void put16(std::vector<u8>& bytes, std::size_t at, u16 value) {
    bytes[at] = static_cast<u8>(value >> 8);
    bytes[at + 1] = static_cast<u8>(value);
}

void put32(std::vector<u8>& bytes, std::size_t at, u32 value) {
    bytes[at] = static_cast<u8>(value >> 24);
    bytes[at + 1] = static_cast<u8>(value >> 16);
    bytes[at + 2] = static_cast<u8>(value >> 8);
    bytes[at + 3] = static_cast<u8>(value);
}

std::vector<u8> matrixRom() {
    std::vector<u8> rom(512u * 1024u, 0xFF);
    put32(rom, 0, 0x00100000u);
    put32(rom, 4, 0x40000100u);
    std::size_t at = 0x100;
    auto moveLong = [&](u32 value, u32 address) {
        put16(rom, at, 0x23FCu); at += 2;           // MOVE.L #imm,(abs).L
        put32(rom, at, value); at += 4;
        put32(rom, at, address); at += 4;
    };
    moveLong(0x4D415452u, 0x00001000u);             // 'MATR'
    moveLong(0x49580001u, 0x00001004u);             // 'IX' + schema 1
    put16(rom, at, 0x60FEu);                        // BRA.S *
    return rom;
}

std::string hex64(u64 value) {
    static constexpr char digits[] = "0123456789ABCDEF";
    std::string result(16, '0');
    for (int index = 15; index >= 0; --index) {
        result[static_cast<std::size_t>(index)] = digits[value & 15u];
        value >>= 4;
    }
    return result;
}

struct Profile {
    const char* name;
    std::size_t diskBytes;
    bool floppy;
};

using Baseline = std::map<std::string, nlohmann::json>;

bool loadBaseline(const char* path, Baseline& baseline, std::string& error) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        error = std::string("cannot open matrix baseline: ") + path;
        return false;
    }
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (line.empty() || line[0] == '#') continue;
        try {
            nlohmann::json record = nlohmann::json::parse(line);
            if (!record.is_object() || !record.contains("case")) {
                error = "invalid matrix baseline record at line " +
                        std::to_string(lineNumber);
                return false;
            }
            baseline.emplace(record.at("case").get<std::string>(),
                             std::move(record));
        } catch (const std::exception& exception) {
            error = "matrix baseline line " + std::to_string(lineNumber) +
                    ": " + exception.what();
            return false;
        }
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    const bool emitOnly = argc == 2 && std::string(argv[1]) == "--emit";
    if (argc != 2) {
        std::cerr << "usage: openmac_iifx_matrix <baseline.jsonl>|--emit\n";
        return 2;
    }

    Baseline baseline;
    std::string error;
    if (!emitOnly && !loadBaseline(argv[1], baseline, error)) {
        std::cerr << error << '\n';
        return 2;
    }

    constexpr std::array<int, 9> ramSizes{4, 8, 16, 20, 32, 64, 68, 80, 128};
    constexpr std::array<Profile, 6> profiles{{
        {"none", 0, false},
        {"scsi80", 80u * kMiB, false},
        {"scsi160", 160u * kMiB, false},
        {"superdrive", 0, true},
        {"scsi80-superdrive", 80u * kMiB, true},
        {"scsi160-superdrive", 160u * kMiB, true},
    }};

    std::size_t failures = 0;
    std::size_t cases = 0;
    for (int ramMb : ramSizes) {
        for (const Profile& profile : profiles) {
            ++cases;
            IifxMachine::Config config;
            config.ramSize = static_cast<u32>(ramMb) * static_cast<u32>(kMiB);
            config.nativeStorage = true;
            IifxMachine machine(matrixRom(), config);

            if (profile.diskBytes != 0) {
                std::vector<u8> disk(profile.diskBytes, 0);
                disk[0] = 0x45; disk[1] = 0x52;     // Driver Descriptor Map
                disk[2] = 0x02; disk[3] = 0x00;     // 512-byte blocks
                machine.insertHardDisk(std::move(disk));
            }
            if (profile.floppy) {
                std::vector<u8> floppy(1440u * 1024u, 0);
                floppy[0] = 0x4C;
                if (!machine.insertFloppy(std::move(floppy))) {
                    std::cerr << "matrix media insertion failed\n";
                    return 2;
                }
            }

            HardwareTraceConfig trace;
            trace.capacity = 128;
            trace.postTriggerEvents = 0;
            trace.categories = TraceAssertion | TraceMilestone;
            machine.configureHardwareTrace(trace);
            int instructions = 0;
            while (instructions < 32 && machine.read32(0x1004u) != 0x49580001u) {
                machine.stepInstruction();
                ++instructions;
            }
            const bool milestone = machine.read32(0x1000u) == 0x4D415452u &&
                                   machine.read32(0x1004u) == 0x49580001u;
            // Devices tick in batches (IifxMachine::kTickBatchCycles): after
            // single-stepping, the cycles of the last few instructions are
            // still banked. Catch the devices up so the hash below is of the
            // machine at this instant, as the baseline was recorded.
            machine.flushPendingTicks();
            machine.recordHardwareMilestone("matrix-boot");

            // The checkpoint must still serialize, but its bytes are not a
            // cross-compiler contract; the state hash is the machine's
            // deterministic CPU/device hash instead.
            const std::vector<u8> checkpoint = machine.saveState();
            if (checkpoint.empty()) {
                std::cerr << "matrix checkpoint failed\n";
                return 2;
            }
            const std::string caseName = std::string(profile.name) + "-ram" +
                                         std::to_string(ramMb);
            nlohmann::json record{
                {"case", caseName},
                {"ram_mb", ramMb},
                {"media", profile.name},
                {"disk_bytes", profile.diskBytes},
                {"floppy_bytes", profile.floppy ? 1440u * 1024u : 0u},
                {"milestone", milestone ? "matrix-boot" : "missing"},
                {"instructions", instructions},
                {"state_hash", hex64(machine.deterministicStateHash())},
                {"framebuffer_hash", hex64(machine.framebufferHash())},
                {"assertion", machine.hardwareTrace().triggerReason()},
            };

            if (emitOnly) {
                std::cout << record.dump() << '\n';
                continue;
            }
            const auto expected = baseline.find(caseName);
            if (expected == baseline.end()) {
                ++failures;
                std::cerr << caseName << ": missing from baseline\n";
            } else if (expected->second != record) {
                ++failures;
                std::cerr << caseName << ": baseline mismatch\n  expected "
                          << expected->second.dump() << "\n  actual   "
                          << record.dump() << '\n';
            }
        }
    }
    if (!emitOnly && baseline.size() != cases) {
        ++failures;
        std::cerr << "baseline contains " << baseline.size() << " cases; expected "
                  << cases << '\n';
    }
    if (emitOnly) return 0;
    if (failures != 0) {
        std::cerr << "IIfx public boot matrix failed: " << failures
                  << " of " << cases << " cases\n";
        return 1;
    }
    std::cout << "IIfx public boot matrix passed: " << cases
              << " RAM/media cases\n";
    return 0;
}
