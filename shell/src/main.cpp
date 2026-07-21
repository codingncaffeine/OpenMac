#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include <openmac/machine.hpp>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mutex>
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

// -------- persisted settings (a small key=value file next to the exe) --------
struct Settings {
    int ramMB = 4;
    bool bootDisk = false;
    std::vector<std::string> recent;   // most-recent first
};

std::string configDir() {
    const char* base = SDL_GetBasePath();
    return base ? std::string(base) : std::string();
}

Settings loadSettings() {
    Settings s;
    std::ifstream f(configDir() + "openmac.cfg");
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string k = line.substr(0, eq), v = line.substr(eq + 1);
        if (k == "ram") s.ramMB = std::atoi(v.c_str());
        else if (k == "bootdisk") s.bootDisk = std::atoi(v.c_str()) != 0;
        else if (k == "recent" && !v.empty()) s.recent.push_back(v);
    }
    return s;
}

void saveSettings(const Settings& s) {
    std::ofstream f(configDir() + "openmac.cfg", std::ios::trunc);
    f << "ram=" << s.ramMB << "\n" << "bootdisk=" << (s.bootDisk ? 1 : 0) << "\n";
    for (size_t i = 0; i < s.recent.size() && i < 8; ++i) f << "recent=" << s.recent[i] << "\n";
}

void pushRecent(Settings& s, const std::string& path) {
    s.recent.erase(std::remove(s.recent.begin(), s.recent.end(), path), s.recent.end());
    s.recent.insert(s.recent.begin(), path);
    if (s.recent.size() > 8) s.recent.resize(8);
}

// -------- async file dialog handoff (callback runs on a dialog thread) -------
struct FilePick {
    std::mutex m;
    std::string path;
    bool ready = false;
};
FilePick g_pick;

void SDLCALL onFileChosen(void*, const char* const* filelist, int) {
    if (filelist && filelist[0]) {
        std::lock_guard<std::mutex> lock(g_pick.m);
        g_pick.path = filelist[0];
        g_pick.ready = true;
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string cliRom;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--rom" && i + 1 < argc) cliRom = argv[++i];
    }

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "OpenMac", SDL_GetError(), nullptr);
        return 1;
    }
    SDL_Window* window = SDL_CreateWindow("OpenMac", 1024, 720, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    if (!window || !renderer) {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "OpenMac", SDL_GetError(), window);
        return 1;
    }
    SDL_SetRenderVSync(renderer, 1);

    SDL_Texture* screenTex = SDL_CreateTexture(
        renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        openmac::Machine::kScreenW, openmac::Machine::kScreenH);
    SDL_SetTextureScaleMode(screenTex, SDL_SCALEMODE_NEAREST);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    std::ofstream log(configDir() + "openmac.log", std::ios::trunc);
    auto logline = [&](const std::string& s) { log << s << "\n"; log.flush(); };

    Settings settings = loadSettings();
    std::unique_ptr<openmac::Machine> mac;
    std::string loadedRom;
    std::string loadError;
    int bootHold = 0;
    int frameNo = 0;

    auto startMachine = [&](const std::string& romPath) {
        auto rom = loadFile(romPath);
        if (rom.empty()) { loadError = "Could not read ROM: " + romPath; return; }
        loadError.clear();
        openmac::Machine::Config cfg;
        cfg.ramSize = static_cast<openmac::u32>(settings.ramMB) * 1024u * 1024u;
        mac = std::make_unique<openmac::Machine>(std::move(rom), cfg);
        loadedRom = romPath;
        frameNo = 0;
        mac->cpu().onException = [&](int vector, openmac::u32 pc) {
            if (vector == 2 || vector == 3 || vector == 4 || vector == 8 || vector == 11) {
                char b[96];
                std::snprintf(b, sizeof b, "EXC vec=%d at pc=%06X cyc=%llu", vector, pc,
                              static_cast<unsigned long long>(mac->totalCycles()));
                logline(b);
            }
        };
        bootHold = 0;
        if (settings.bootDisk) {                        // hold Cmd-Opt-X-O
            mac->keyEvent(0x37, true); mac->keyEvent(0x3A, true);
            mac->keyEvent(0x07, true); mac->keyEvent(0x1F, true);
            bootHold = 200;
        }
        pushRecent(settings, romPath);
        saveSettings(settings);
        logline("loaded " + romPath + (settings.bootDisk ? " (ROM-disk boot)" : ""));
    };

    auto openDialog = [&] {
        static const SDL_DialogFileFilter filters[] = {
            {"Macintosh ROM", "rom;bin"}, {"All files", "*"}};
        SDL_ShowOpenFileDialog(onFileChosen, nullptr, window, filters, 2, nullptr, false);
    };

    if (!cliRom.empty()) startMachine(cliRom);

    bool running = true;
    bool machineRunning = true;
    bool showDebugger = true;
    bool mouseDown = false;
    std::vector<openmac::u32> pixels(
        static_cast<size_t>(openmac::Machine::kScreenW) * openmac::Machine::kScreenH, 0xFF101010u);

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            ImGuiIO& io = ImGui::GetIO();
            if (event.type == SDL_EVENT_QUIT) running = false;
            else if (!mac) continue;
            else if (event.type == SDL_EVENT_MOUSE_MOTION && !io.WantCaptureMouse)
                mac->mouseMove(static_cast<int>(event.motion.xrel),
                               static_cast<int>(event.motion.yrel), mouseDown);
            else if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN &&
                     event.button.button == SDL_BUTTON_LEFT && !io.WantCaptureMouse) {
                mouseDown = true; mac->mouseMove(0, 0, true);
            } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP &&
                       event.button.button == SDL_BUTTON_LEFT) {
                mouseDown = false; mac->mouseMove(0, 0, false);
            } else if (event.type == SDL_EVENT_KEY_DOWN && !io.WantCaptureKeyboard &&
                       !event.key.repeat) {
                const openmac::u8 c = sdlToAdb(event.key.scancode);
                if (c != 0xFF) mac->keyEvent(c, true);
            } else if (event.type == SDL_EVENT_KEY_UP && !io.WantCaptureKeyboard) {
                const openmac::u8 c = sdlToAdb(event.key.scancode);
                if (c != 0xFF) mac->keyEvent(c, false);
            }
        }

        // Consume a ROM chosen by the file dialog (set on another thread).
        {
            std::string picked;
            {
                std::lock_guard<std::mutex> lock(g_pick.m);
                if (g_pick.ready) { picked = g_pick.path; g_pick.ready = false; }
            }
            if (!picked.empty()) startMachine(picked);
        }

        if (mac && machineRunning) {
            mac->runFrame();
            if (bootHold > 0 && --bootHold == 0) {
                mac->keyEvent(0x37, false); mac->keyEvent(0x3A, false);
                mac->keyEvent(0x07, false); mac->keyEvent(0x1F, false);
            }
            if (++frameNo % 60 == 0) {
                const auto s = mac->adbStats();
                char b[160];
                std::snprintf(b, sizeof b,
                    "f=%d pc=%06X cyc=%llu mousePolls=%u kbdPolls=%u mouseReports=%u%s",
                    frameNo, mac->cpu().pc,
                    static_cast<unsigned long long>(mac->totalCycles()),
                    s.mousePolls, s.kbdPolls, s.mouseReports, mac->cpu().halted ? " HALTED" : "");
                logline(b);
            }
        }
        if (mac) {
            mac->renderScreen(pixels.data());
            SDL_UpdateTexture(screenTex, nullptr, pixels.data(), openmac::Machine::kScreenW * 4);
        }

        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        float menuH = 0.0f;
        if (ImGui::BeginMainMenuBar()) {
            menuH = ImGui::GetWindowSize().y;
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Open ROM...")) openDialog();
                if (ImGui::BeginMenu("Recent ROMs", !settings.recent.empty())) {
                    for (const auto& r : settings.recent)
                        if (ImGui::MenuItem(r.c_str())) startMachine(r);
                    ImGui::EndMenu();
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Quit")) running = false;
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Machine")) {
                if (ImGui::MenuItem("Reset", nullptr, false, mac != nullptr)) mac->reset();
                ImGui::MenuItem("Run", nullptr, &machineRunning, mac != nullptr);
                ImGui::Separator();
                if (ImGui::BeginMenu("Memory")) {
                    for (int mb : {1, 2, 4})
                        if (ImGui::MenuItem((std::to_string(mb) + " MB").c_str(), nullptr,
                                            settings.ramMB == mb)) {
                            settings.ramMB = mb; saveSettings(settings);
                        }
                    ImGui::EndMenu();
                }
                if (ImGui::MenuItem("Boot from ROM disk (Cmd-Opt-X-O)", nullptr, &settings.bootDisk))
                    saveSettings(settings);
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("View")) {
                ImGui::MenuItem("Debugger", nullptr, &showDebugger);
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        // Fit the 512x342 screen into the window below the menu bar.
        if (mac) {
            int winW = 0, winH = 0;
            SDL_GetRenderOutputSize(renderer, &winW, &winH);
            const float availH = static_cast<float>(winH) - menuH;
            const float sx = static_cast<float>(winW) / openmac::Machine::kScreenW;
            const float sy = availH / openmac::Machine::kScreenH;
            const float sc = sx < sy ? sx : sy;
            SDL_FRect dst;
            dst.w = openmac::Machine::kScreenW * sc;
            dst.h = openmac::Machine::kScreenH * sc;
            dst.x = (winW - dst.w) * 0.5f;
            dst.y = menuH + (availH - dst.h) * 0.5f;
            SDL_SetRenderDrawColor(renderer, 16, 16, 16, 255);
            SDL_RenderClear(renderer);
            SDL_RenderTexture(renderer, screenTex, nullptr, &dst);
        } else {
            SDL_SetRenderDrawColor(renderer, 24, 26, 32, 255);
            SDL_RenderClear(renderer);
            // Launcher.
            const ImGuiViewport* vp = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(ImVec2(vp->WorkSize.x * 0.5f, vp->WorkSize.y * 0.5f),
                                    ImGuiCond_Always, ImVec2(0.5f, 0.5f));
            ImGui::Begin("Welcome to OpenMac", nullptr,
                         ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                             ImGuiWindowFlags_NoMove);
            ImGui::TextUnformatted("Macintosh Classic emulator");
            ImGui::Spacing();
            if (ImGui::Button("Open ROM...", ImVec2(220, 0))) openDialog();
            if (!settings.recent.empty()) {
                ImGui::SeparatorText("Recent");
                for (const auto& r : settings.recent)
                    if (ImGui::Selectable(r.c_str())) startMachine(r);
            }
            ImGui::SeparatorText("Options");
            int ramIdx = settings.ramMB == 1 ? 0 : settings.ramMB == 2 ? 1 : 2;
            const char* rams[] = {"1 MB", "2 MB", "4 MB"};
            if (ImGui::Combo("Memory", &ramIdx, rams, 3)) {
                settings.ramMB = ramIdx == 0 ? 1 : ramIdx == 1 ? 2 : 4;
                saveSettings(settings);
            }
            if (ImGui::Checkbox("Boot System 6 from ROM disk (Cmd-Opt-X-O)", &settings.bootDisk))
                saveSettings(settings);
            if (!loadError.empty()) {
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "%s", loadError.c_str());
            }
            ImGui::End();
        }

        if (mac && showDebugger) {
            ImGui::SetNextWindowSize(ImVec2(340, 460), ImGuiCond_FirstUseEver);
            ImGui::Begin("Debugger", &showDebugger);
            auto& cpu = mac->cpu();
            ImGui::Text("cycles: %llu   overlay: %s",
                        static_cast<unsigned long long>(mac->totalCycles()),
                        mac->overlayActive() ? "ON" : "off");
            if (ImGui::Button("Step Instr")) mac->stepInstruction();
            ImGui::SameLine();
            if (ImGui::Button("Reset")) mac->reset();
            ImGui::SeparatorText("CPU");
            for (int i = 0; i < 8; ++i)
                ImGui::Text("D%d %08X   A%d %08X", i, cpu.d[i], i, cpu.a[i]);
            ImGui::Text("PC %08X  SR %04X", cpu.pc, cpu.getSR());
            if (cpu.halted) ImGui::TextColored(ImVec4(1, 0.3f, 0.3f, 1), "HALTED");
            const auto s = mac->adbStats();
            ImGui::SeparatorText("ADB");
            ImGui::Text("mouse polls %u  reports %u  kbd polls %u",
                        s.mousePolls, s.mouseReports, s.kbdPolls);
            ImGui::SeparatorText("Access log");
            ImGui::BeginChild("log", ImVec2(0, 150), ImGuiChildFlags_Borders);
            for (const auto& l : mac->accessLog()) ImGui::TextUnformatted(l.c_str());
            ImGui::EndChild();
            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    saveSettings(settings);
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyTexture(screenTex);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
