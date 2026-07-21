#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include <openmac/machine.hpp>

#include <cstdio>
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

// SDL scancode -> ADB keycode (Apple keyboard layout); 0xFF = unmapped.
openmac::u8 sdlToAdb(SDL_Scancode sc) {
    switch (sc) {
    case SDL_SCANCODE_A: return 0x00; case SDL_SCANCODE_S: return 0x01;
    case SDL_SCANCODE_D: return 0x02; case SDL_SCANCODE_F: return 0x03;
    case SDL_SCANCODE_H: return 0x04; case SDL_SCANCODE_G: return 0x05;
    case SDL_SCANCODE_Z: return 0x06; case SDL_SCANCODE_X: return 0x07;
    case SDL_SCANCODE_C: return 0x08; case SDL_SCANCODE_V: return 0x09;
    case SDL_SCANCODE_B: return 0x0B; case SDL_SCANCODE_Q: return 0x0C;
    case SDL_SCANCODE_W: return 0x0D; case SDL_SCANCODE_E: return 0x0E;
    case SDL_SCANCODE_R: return 0x0F; case SDL_SCANCODE_Y: return 0x10;
    case SDL_SCANCODE_T: return 0x11; case SDL_SCANCODE_1: return 0x12;
    case SDL_SCANCODE_2: return 0x13; case SDL_SCANCODE_3: return 0x14;
    case SDL_SCANCODE_4: return 0x15; case SDL_SCANCODE_6: return 0x16;
    case SDL_SCANCODE_5: return 0x17; case SDL_SCANCODE_EQUALS: return 0x18;
    case SDL_SCANCODE_9: return 0x19; case SDL_SCANCODE_7: return 0x1A;
    case SDL_SCANCODE_MINUS: return 0x1B; case SDL_SCANCODE_8: return 0x1C;
    case SDL_SCANCODE_0: return 0x1D; case SDL_SCANCODE_RIGHTBRACKET: return 0x1E;
    case SDL_SCANCODE_O: return 0x1F; case SDL_SCANCODE_U: return 0x20;
    case SDL_SCANCODE_LEFTBRACKET: return 0x21; case SDL_SCANCODE_I: return 0x22;
    case SDL_SCANCODE_P: return 0x23; case SDL_SCANCODE_RETURN: return 0x24;
    case SDL_SCANCODE_L: return 0x25; case SDL_SCANCODE_J: return 0x26;
    case SDL_SCANCODE_APOSTROPHE: return 0x27; case SDL_SCANCODE_K: return 0x28;
    case SDL_SCANCODE_SEMICOLON: return 0x29; case SDL_SCANCODE_BACKSLASH: return 0x2A;
    case SDL_SCANCODE_COMMA: return 0x2B; case SDL_SCANCODE_SLASH: return 0x2C;
    case SDL_SCANCODE_N: return 0x2D; case SDL_SCANCODE_M: return 0x2E;
    case SDL_SCANCODE_PERIOD: return 0x2F; case SDL_SCANCODE_TAB: return 0x30;
    case SDL_SCANCODE_SPACE: return 0x31; case SDL_SCANCODE_GRAVE: return 0x32;
    case SDL_SCANCODE_BACKSPACE: return 0x33; case SDL_SCANCODE_ESCAPE: return 0x35;
    case SDL_SCANCODE_LGUI: case SDL_SCANCODE_RGUI: return 0x37;   // Command
    case SDL_SCANCODE_LSHIFT: case SDL_SCANCODE_RSHIFT: return 0x38;
    case SDL_SCANCODE_CAPSLOCK: return 0x39;
    case SDL_SCANCODE_LALT: case SDL_SCANCODE_RALT: return 0x3A;   // Option
    case SDL_SCANCODE_LCTRL: case SDL_SCANCODE_RCTRL: return 0x3B;
    case SDL_SCANCODE_LEFT: return 0x3B; case SDL_SCANCODE_RIGHT: return 0x3C;
    case SDL_SCANCODE_DOWN: return 0x3D; case SDL_SCANCODE_UP: return 0x3E;
    default: return 0xFF;
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string romPath;
    openmac::Machine::Config cfg;
    bool bootDisk = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--rom" && i + 1 < argc) romPath = argv[++i];
        else if (arg == "--ram-mb" && i + 1 < argc) {
            cfg.ramSize = static_cast<openmac::u32>(std::atoi(argv[++i])) * 1024u * 1024u;
        }
        else if (arg == "--boot-disk") bootDisk = true;
    }

    std::ofstream log("openmac.log", std::ios::trunc);
    auto logline = [&](const std::string& s) { log << s << "\n"; log.flush(); };
    logline(std::string("OpenMac start; rom=") + romPath +
            (bootDisk ? " (holding Cmd-Opt-X-O for ROM-disk boot)" : ""));

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
            mac->cpu().onException = [&](int vector, openmac::u32 pc) {
                if (vector == 2 || vector == 3 || vector == 4 || vector == 8 ||
                    vector == 11) {
                    char b[96];
                    std::snprintf(b, sizeof b, "EXC vec=%d at pc=%06X cyc=%llu",
                                  vector, pc,
                                  static_cast<unsigned long long>(mac->totalCycles()));
                    logline(b);
                }
            };
        }
    }

    // ROM-disk boot: hold Cmd-Option-X-O down through the early boot window.
    int bootHold = bootDisk ? 200 : 0;
    if (mac && bootDisk) {
        mac->keyEvent(0x37, true); mac->keyEvent(0x3A, true);   // Command, Option
        mac->keyEvent(0x07, true); mac->keyEvent(0x1F, true);   // X, O
    }

    std::vector<openmac::u32> pixels(
        static_cast<size_t>(openmac::Machine::kScreenW) * openmac::Machine::kScreenH,
        0xFF202020u);

    bool running = true;
    bool machineRunning = true;
    bool mouseDown = false;
    int frameNo = 0;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            ImGuiIO& io = ImGui::GetIO();
            if (event.type == SDL_EVENT_QUIT) running = false;
            else if (!mac) continue;
            else if (event.type == SDL_EVENT_MOUSE_MOTION && !io.WantCaptureMouse) {
                mac->mouseMove(static_cast<int>(event.motion.xrel),
                               static_cast<int>(event.motion.yrel), mouseDown);
            } else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN &&
                       event.button.button == SDL_BUTTON_LEFT && !io.WantCaptureMouse) {
                mouseDown = true;
                mac->mouseMove(0, 0, true);
            } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP &&
                       event.button.button == SDL_BUTTON_LEFT) {
                mouseDown = false;
                mac->mouseMove(0, 0, false);
            } else if (event.type == SDL_EVENT_KEY_DOWN && !io.WantCaptureKeyboard &&
                       !event.key.repeat) {
                const openmac::u8 c = sdlToAdb(event.key.scancode);
                if (c != 0xFF) mac->keyEvent(c, true);
            } else if (event.type == SDL_EVENT_KEY_UP && !io.WantCaptureKeyboard) {
                const openmac::u8 c = sdlToAdb(event.key.scancode);
                if (c != 0xFF) mac->keyEvent(c, false);
            }
        }

        if (mac && machineRunning) {
            mac->runFrame();
            if (bootHold > 0 && --bootHold == 0) {          // release the boot combo
                mac->keyEvent(0x37, false); mac->keyEvent(0x3A, false);
                mac->keyEvent(0x07, false); mac->keyEvent(0x1F, false);
                logline("released Cmd-Opt-X-O");
            }
            if (++frameNo % 60 == 0) {                       // ~once a second
                const auto s = mac->adbStats();
                char b[160];
                std::snprintf(b, sizeof b,
                    "f=%d pc=%06X cyc=%llu mousePolls=%u kbdPolls=%u mouseReports=%u%s",
                    frameNo, mac->cpu().pc,
                    static_cast<unsigned long long>(mac->totalCycles()),
                    s.mousePolls, s.kbdPolls, s.mouseReports,
                    mac->cpu().halted ? " HALTED" : "");
                logline(b);
            }
        }
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
