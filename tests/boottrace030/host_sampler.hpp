// Poor-man's sampling profiler for the trace tool itself (Windows only).
// A background thread suspends the emulation thread every millisecond,
// reads its instruction pointer, and buckets it by symbol through dbghelp
// once at report time.  Diagnostic-only; not part of the emulator.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#if defined(_WIN32) && defined(_MSC_VER)
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <dbghelp.h>
#include <timeapi.h>
#include <atomic>
#include <thread>
#include <algorithm>
#include <mutex>
#pragma comment(lib, "dbghelp.lib")
#pragma comment(lib, "winmm.lib")

class HostSampler {
public:
    void start() {
        if (running_.exchange(true)) return;
        HANDLE process = GetCurrentProcess();
        DuplicateHandle(process, GetCurrentThread(), process, &target_, 0,
                        FALSE, DUPLICATE_SAME_ACCESS);
        timeBeginPeriod(1);
        worker_ = std::thread([this] { loop(); });
    }
    void stop() {
        if (!running_.exchange(false)) return;
        if (worker_.joinable()) worker_.join();
        if (target_) CloseHandle(target_);
        target_ = nullptr;
        timeEndPeriod(1);
    }
    std::string report(std::size_t top = 40) {
        std::lock_guard<std::mutex> lock(mutex_);
        HANDLE process = GetCurrentProcess();
        SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS |
                      SYMOPT_LOAD_LINES);
        SymInitialize(process, nullptr, TRUE);
        std::unordered_map<std::string, std::uint64_t> bySymbol;
        std::uint64_t total = 0;
        alignas(SYMBOL_INFO) char buffer[sizeof(SYMBOL_INFO) + 512];
        for (const auto& [address, count] : samples_) {
            total += count;
            auto* symbol = reinterpret_cast<SYMBOL_INFO*>(buffer);
            symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
            symbol->MaxNameLen = 511;
            DWORD64 displacement = 0;
            std::string name;
            if (SymFromAddr(process, address, &displacement, symbol))
                name = symbol->Name;
            else {
                char item[32];
                std::snprintf(item, sizeof item, "%016llX",
                              static_cast<unsigned long long>(address));
                name = item;
            }
            bySymbol[name] += count;
        }
        std::vector<std::pair<std::string, std::uint64_t>> sorted(
            bySymbol.begin(), bySymbol.end());
        std::sort(sorted.begin(), sorted.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });
        std::string result;
        char line[640];
        std::snprintf(line, sizeof line,
                      "host profile: %llu samples (%zu distinct pcs)\n",
                      static_cast<unsigned long long>(total), samples_.size());
        result += line;
        for (std::size_t index = 0; index < sorted.size() && index < top;
             ++index) {
            std::snprintf(line, sizeof line, "  %6.2f%%  %s\n",
                          total ? 100.0 * static_cast<double>(sorted[index].second) /
                                      static_cast<double>(total)
                                : 0.0,
                          sorted[index].first.c_str());
            result += line;
        }
        // The same samples by source line, so a hot function's cost can be
        // placed on the statements that carry it.
        std::unordered_map<std::string, std::uint64_t> byLine;
        for (const auto& [address, count] : samples_) {
            IMAGEHLP_LINE64 info{};
            info.SizeOfStruct = sizeof info;
            DWORD displacement = 0;
            if (SymGetLineFromAddr64(process, address, &displacement, &info)) {
                const char* file = info.FileName ? info.FileName : "?";
                const char* base = std::strrchr(file, '\\');
                char key[320];
                std::snprintf(key, sizeof key, "%s:%lu", base ? base + 1 : file,
                              static_cast<unsigned long>(info.LineNumber));
                byLine[key] += count;
            }
        }
        std::vector<std::pair<std::string, std::uint64_t>> lines(
            byLine.begin(), byLine.end());
        std::sort(lines.begin(), lines.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });
        result += "host profile by line:\n";
        for (std::size_t index = 0; index < lines.size() && index < top;
             ++index) {
            std::snprintf(line, sizeof line, "  %6.2f%%  %s\n",
                          total ? 100.0 * static_cast<double>(lines[index].second) /
                                      static_cast<double>(total)
                                : 0.0,
                          lines[index].first.c_str());
            result += line;
        }
        SymCleanup(process);
        return result;
    }

private:
    void loop() {
        while (running_.load()) {
            Sleep(1);
            if (SuspendThread(target_) == static_cast<DWORD>(-1)) continue;
            CONTEXT context{};
            context.ContextFlags = CONTEXT_CONTROL;
            DWORD64 pc = 0;
            if (GetThreadContext(target_, &context)) pc = context.Rip;
            ResumeThread(target_);
            if (pc != 0) {
                std::lock_guard<std::mutex> lock(mutex_);
                ++samples_[pc];
            }
        }
    }
    std::atomic<bool> running_{false};
    std::thread worker_;
    HANDLE target_ = nullptr;
    std::mutex mutex_;
    std::unordered_map<DWORD64, std::uint64_t> samples_;
};
#else
class HostSampler {
public:
    void start() {}
    void stop() {}
    std::string report(std::size_t = 40) { return "host profile: unavailable\n"; }
};
#endif
