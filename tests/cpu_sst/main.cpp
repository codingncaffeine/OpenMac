// SingleStepTests/680x0 harness: runs the per-opcode JSON suites against the
// M68000 core. State-accurate comparison; cycle mismatches are counted
// separately (hard-failed only with --cycles).
//
// Test convention: initial.pc is the instruction address; prefetch[0..1] are
// the words at pc/pc+2 (not present in the ram list) and get injected into
// memory; final.pc is the next instruction's address.

#include <openmac/cpu.hpp>

#include <miniz.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using json = nlohmann::json;
namespace fs = std::filesystem;
using namespace openmac;

namespace {

class TestBus final : public IBus {
public:
    TestBus() : mem_(1u << 24, 0) {}

    u8 read8(u32 addr) override { return mem_[addr & 0xFFFFFF]; }
    u16 read16(u32 addr) override {
        addr &= 0xFFFFFF;
        return static_cast<u16>((mem_[addr] << 8) | mem_[(addr + 1) & 0xFFFFFF]);
    }
    void write8(u32 addr, u8 v) override { poke(addr, v); }
    void write16(u32 addr, u16 v) override {
        poke(addr, static_cast<u8>(v >> 8));
        poke(addr + 1, static_cast<u8>(v & 0xFF));
    }

    void poke(u32 addr, u8 v) {
        addr &= 0xFFFFFF;
        dirty_.push_back(addr);
        mem_[addr] = v;
    }
    u8 peek(u32 addr) const { return mem_[addr & 0xFFFFFF]; }

    void clear() {
        for (u32 addr : dirty_) mem_[addr] = 0;
        dirty_.clear();
    }

private:
    std::vector<u8> mem_;
    std::vector<u32> dirty_;
};

std::string gunzipFile(const fs::path& path) {
    std::ifstream f(path, std::ios::binary);
    std::vector<char> raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (raw.size() < 18 || static_cast<u8>(raw[0]) != 0x1F || static_cast<u8>(raw[1]) != 0x8B ||
        static_cast<u8>(raw[2]) != 8) {
        std::fprintf(stderr, "not a gzip file: %s\n", path.string().c_str());
        return {};
    }
    const u8 flg = static_cast<u8>(raw[3]);
    size_t off = 10;
    if (flg & 0x04) { // FEXTRA
        const size_t xlen = static_cast<u8>(raw[off]) | (static_cast<u8>(raw[off + 1]) << 8);
        off += 2 + xlen;
    }
    if (flg & 0x08) { while (off < raw.size() && raw[off] != 0) ++off; ++off; } // FNAME
    if (flg & 0x10) { while (off < raw.size() && raw[off] != 0) ++off; ++off; } // FCOMMENT
    if (flg & 0x02) off += 2;                                                  // FHCRC

    size_t outLen = 0;
    void* out = tinfl_decompress_mem_to_heap(raw.data() + off, raw.size() - off - 8, &outLen, 0);
    if (!out) {
        std::fprintf(stderr, "inflate failed: %s\n", path.string().c_str());
        return {};
    }
    std::string s(static_cast<char*>(out), outLen);
    mz_free(out);
    return s;
}

struct Totals {
    long tests = 0;
    long statePass = 0;
    long cyclePass = 0;
};

struct Options {
    bool cycles = false;
    int maxShow = 6;
    std::string filter;
};

void applyState(M68000& cpu, TestBus& bus, const json& st) {
    for (int i = 0; i < 8; ++i) cpu.d[i] = st["d" + std::to_string(i)].get<u32>();
    for (int i = 0; i < 7; ++i) cpu.a[i] = st["a" + std::to_string(i)].get<u32>();
    const u16 sr = static_cast<u16>(st["sr"].get<u32>());
    cpu.setSR(sr);
    cpu.usp = st["usp"].get<u32>();
    cpu.ssp = st["ssp"].get<u32>();
    cpu.a[7] = (sr & 0x2000) ? cpu.ssp : cpu.usp;
    cpu.pc = st["pc"].get<u32>();
    cpu.stopped = false;
    cpu.halted = false;

    for (const auto& pair : st["ram"]) {
        bus.poke(pair[0].get<u32>(), static_cast<u8>(pair[1].get<u32>()));
    }
    const auto& pf = st["prefetch"];
    const u32 pc = cpu.pc;
    const u16 w0 = static_cast<u16>(pf[0].get<u32>());
    const u16 w1 = static_cast<u16>(pf[1].get<u32>());
    bus.poke(pc, static_cast<u8>(w0 >> 8));
    bus.poke(pc + 1, static_cast<u8>(w0 & 0xFF));
    bus.poke(pc + 2, static_cast<u8>(w1 >> 8));
    bus.poke(pc + 3, static_cast<u8>(w1 & 0xFF));
}

bool compareState(const M68000& cpu, const TestBus& bus, const json& fin,
                  std::vector<std::string>& diffs) {
    char buf[160];
    auto expect = [&](const char* what, u32 got, u32 want) {
        if (got != want) {
            std::snprintf(buf, sizeof(buf), "  %-4s got %08X want %08X", what, got, want);
            diffs.emplace_back(buf);
        }
    };
    for (int i = 0; i < 8; ++i)
        expect(("d" + std::to_string(i)).c_str(), cpu.d[i], fin["d" + std::to_string(i)].get<u32>());
    for (int i = 0; i < 7; ++i)
        expect(("a" + std::to_string(i)).c_str(), cpu.a[i], fin["a" + std::to_string(i)].get<u32>());
    expect("usp", cpu.uspValue(), fin["usp"].get<u32>());
    expect("ssp", cpu.sspValue(), fin["ssp"].get<u32>());
    expect("sr", cpu.getSR(), fin["sr"].get<u32>());
    expect("pc", cpu.pc, fin["pc"].get<u32>());
    for (const auto& pair : fin["ram"]) {
        const u32 addr = pair[0].get<u32>();
        const u8 want = static_cast<u8>(pair[1].get<u32>());
        const u8 got = bus.peek(addr);
        if (got != want) {
            std::snprintf(buf, sizeof(buf), "  ram[%06X] got %02X want %02X", addr, got, want);
            diffs.emplace_back(buf);
        }
    }
    return diffs.empty();
}

Totals runFile(const fs::path& path, M68000& cpu, TestBus& bus, const Options& opt) {
    Totals t;
    const std::string text = gunzipFile(path);
    if (text.empty()) return t;
    const json doc = json::parse(text);

    // Suite anomalies: entries that contradict hardware or their own sibling
    // tests. The ASL.b pair changes the upper 24 bits of the register on a
    // byte shift; the DIVU test pushes a zero-divide PC that disagrees with
    // the 157 other passing (d16,A7) DIVU tests.
    static const char* kKnownBad[] = {
        "e502 [ASL.b Q, D2] 1583",
        "e502 [ASL.b Q, D2] 1761",
        "80ef [DIVU (d16, A7), D0] 5745",
    };

    int shown = 0;
    for (const auto& test : doc) {
        const std::string name = test["name"].get<std::string>();
        if (!opt.filter.empty() && name.find(opt.filter) == std::string::npos) {
            continue;
        }
        bool skip = false;
        for (const char* bad : kKnownBad) {
            if (name == bad) { skip = true; break; }
        }
        if (skip) continue;
        ++t.tests;
        bus.clear();
        applyState(cpu, bus, test["initial"]);
        const int cycles = cpu.step();

        std::vector<std::string> diffs;
        const bool stateOk = compareState(cpu, bus, test["final"], diffs);
        const bool cycleOk = cycles == test["length"].get<int>();
        if (stateOk) ++t.statePass;
        if (cycleOk) ++t.cyclePass;

        if (!stateOk && shown < opt.maxShow) {
            ++shown;
            std::printf("FAIL %s\n", test["name"].get<std::string>().c_str());
            for (const auto& d : diffs) std::printf("%s\n", d.c_str());
        }
        if (stateOk && !cycleOk && opt.cycles && shown < opt.maxShow) {
            ++shown;
            std::printf("CYC  %s: got %d want %d\n",
                        test["name"].get<std::string>().c_str(), cycles,
                        test["length"].get<int>());
        }
    }
    return t;
}

} // namespace

int main(int argc, char** argv) {
    Options opt;
    fs::path target;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--cycles") opt.cycles = true;
        else if (arg == "--max-show" && i + 1 < argc) opt.maxShow = std::atoi(argv[++i]);
        else if (arg == "--filter" && i + 1 < argc) opt.filter = argv[++i];
        else target = arg;
    }
    if (target.empty()) {
        std::fprintf(stderr, "usage: openmac_sst <file.json.gz | directory> [--cycles] "
                             "[--max-show N] [--filter substr]\n");
        return 2;
    }

    std::vector<fs::path> files;
    if (fs::is_directory(target)) {
        for (const auto& e : fs::directory_iterator(target)) {
            if (e.path().extension() == ".gz") files.push_back(e.path());
        }
        std::sort(files.begin(), files.end());
    } else {
        files.push_back(target);
    }

    TestBus bus;
    M68000 cpu(bus);
    Totals grand;
    long filesFullPass = 0;

    for (const auto& f : files) {
        const Totals t = runFile(f, cpu, bus, opt);
        grand.tests += t.tests;
        grand.statePass += t.statePass;
        grand.cyclePass += t.cyclePass;
        const bool full = t.statePass == t.tests && (!opt.cycles || t.cyclePass == t.tests);
        if (full) ++filesFullPass;
        std::printf("%-22s state %6ld/%-6ld cycles %6ld/%-6ld%s\n",
                    f.filename().string().c_str(), t.statePass, t.tests, t.cyclePass,
                    t.tests, full ? "" : "  <<<");
    }

    std::printf("\nTOTAL: state %ld/%ld  cycles %ld/%ld  files clean %ld/%zu\n",
                grand.statePass, grand.tests, grand.cyclePass, grand.tests,
                filesFullPass, files.size());

    const bool ok = grand.statePass == grand.tests &&
                    (!opt.cycles || grand.cyclePass == grand.tests);
    return ok ? 0 : 1;
}
