#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include <openmac/machine.hpp>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace {

std::vector<openmac::u8> loadFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return {};
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

} // namespace

int main(int argc, char** argv) {
    std::string romPath;
    openmac::Machine::Config cfg;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--rom" && i + 1 < argc) romPath = argv[++i];
        else if (arg == "--ram-mb" && i + 1 < argc) {
            cfg.ramSize = static_cast<openmac::u32>(std::atoi(argv[++i])) * 1024u * 1024u;
        }
    }

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }
    SDL_Window* window = SDL_CreateWindow("OpenMac", 1024, 684, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    if (!window || !renderer) {
        SDL_Log("SDL window/renderer failed: %s", SDL_GetError());
        return 1;
    }
    SDL_SetRenderVSync(renderer, 1);
    SDL_SetRenderLogicalPresentation(renderer, openmac::Machine::kScreenW,
                                     openmac::Machine::kScreenH,
                                     SDL_LOGICAL_PRESENTATION_LETTERBOX);

    SDL_Texture* screenTex = SDL_CreateTexture(
        renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        openmac::Machine::kScreenW, openmac::Machine::kScreenH);
    SDL_SetTextureScaleMode(screenTex, SDL_SCALEMODE_NEAREST);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    std::unique_ptr<openmac::Machine> mac;
    std::string status = "No ROM loaded. Start with: openmac --rom <path>";
    if (!romPath.empty()) {
        auto rom = loadFile(romPath);
        if (rom.empty()) {
            status = "Failed to read ROM: " + romPath;
        } else {
            mac = std::make_unique<openmac::Machine>(std::move(rom), cfg);
            status = "ROM: " + romPath;
        }
    }

    std::vector<openmac::u32> pixels(
        static_cast<size_t>(openmac::Machine::kScreenW) * openmac::Machine::kScreenH,
        0xFF202020u);

    bool running = true;
    bool machineRunning = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT) running = false;
        }

        if (mac && machineRunning) mac->runFrame();
        if (mac) {
            mac->renderScreen(pixels.data());
            SDL_UpdateTexture(screenTex, nullptr, pixels.data(),
                              openmac::Machine::kScreenW * 4);
        }

        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Machine");
        ImGui::TextUnformatted(status.c_str());
        if (mac) {
            ImGui::Checkbox("Run", &machineRunning);
            ImGui::SameLine();
            if (ImGui::Button("Step Instr")) mac->stepInstruction();
            ImGui::SameLine();
            if (ImGui::Button("Step Frame")) mac->runFrame();
            ImGui::SameLine();
            if (ImGui::Button("Reset")) mac->reset();
            ImGui::Text("cycles: %llu   overlay: %s",
                        static_cast<unsigned long long>(mac->totalCycles()),
                        mac->overlayActive() ? "ON" : "off");

            auto& cpu = mac->cpu();
            ImGui::SeparatorText("CPU");
            for (int i = 0; i < 8; ++i) {
                ImGui::Text("D%d %08X   A%d %08X", i, cpu.d[i], i, cpu.a[i]);
            }
            ImGui::Text("PC %08X  SR %04X  USP %08X  SSP %08X", cpu.pc,
                        cpu.getSR(), cpu.uspValue(), cpu.sspValue());
            if (cpu.halted) ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "HALTED (double fault)");
            if (cpu.stopped) ImGui::TextUnformatted("STOPPED (awaiting interrupt)");

            ImGui::SeparatorText("Access log");
            if (ImGui::Button("Clear")) mac->clearAccessLog();
            ImGui::BeginChild("log", ImVec2(0, 160), ImGuiChildFlags_Borders);
            for (const auto& line : mac->accessLog()) {
                ImGui::TextUnformatted(line.c_str());
            }
            ImGui::EndChild();
        }
        ImGui::End();

        ImGui::Render();
        SDL_SetRenderDrawColor(renderer, 24, 24, 24, 255);
        SDL_RenderClear(renderer);
        if (mac) SDL_RenderTexture(renderer, screenTex, nullptr, nullptr);
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyTexture(screenTex);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
