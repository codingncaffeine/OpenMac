// Headless bring-up tool: runs a ROM for N frames and reports how far the
// machine gets — per-frame PC samples, screen activity, and the stub/unmapped
// access log. The debugger you can read in a terminal.

#include <openmac/machine.hpp>
#include <openmac/debugger.hpp>
#include <openmac/hfs.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace openmac;

namespace {

// Single-step one frame's worth of cycles, collecting unique PCs, then print
// them collapsed into ranges: the shape of the active code path.
void profileFrame(Machine& mac) {
    std::set<u32> pcs;
    const u64 target = mac.totalCycles() +
                       u64(Machine::kLinesPerFrame) * Machine::kCyclesPerLine;
    while (mac.totalCycles() < target && !mac.cpu().halted) {
        pcs.insert(mac.cpu().pc);
        mac.stepInstruction();
    }
    std::printf("-- profile: %zu unique PCs --\n", pcs.size());
    u32 start = 0, prev = 0;
    bool open = false;
    for (u32 pc : pcs) {
        if (!open) { start = prev = pc; open = true; continue; }
        if (pc - prev <= 8) { prev = pc; continue; }
        std::printf("  %06X-%06X\n", start, prev);
        start = prev = pc;
    }
    if (open) std::printf("  %06X-%06X\n", start, prev);
}

// Single-step until PC first hits `stopPc` — or, when stopPc is 1, until
// execution leaves plausible regions (a runaway) — then dump the trail.
void traceUntil(Machine& mac, u32 stopPc, u64 maxCycles) {
    std::vector<u32> ring(96, 0);
    size_t head = 0;
    while (mac.totalCycles() < maxCycles && !mac.cpu().halted) {
        const u32 pc = mac.cpu().pc;
        const u32 p24 = pc & 0xFFFFFF;
        const bool runaway =
            stopPc == 1 && (pc > 0xFFFFFF || (p24 >= 0x480000 && p24 < 0x800000) ||
                            (p24 >= 0x100000 && p24 < 0x400000));
        if (pc == stopPc || runaway) {
            std::printf("-- stopped at pc=%08X after %llu cycles; trail: --\n", pc,
                        static_cast<unsigned long long>(mac.totalCycles()));
            auto rd32 = [&](u32 a) {
                return (u32(mac.read16(a)) << 16) | mac.read16(a + 2);
            };
            std::printf("sr=%04X a7=%08X vec1=%08X vec2=%08X vec3=%08X\n",
                        mac.cpu().getSR(), mac.cpu().a[7], rd32(0x64), rd32(0x68),
                        rd32(0x6C));
            std::printf("via IFR=%02X IER=%02X\n", mac.read8(0xEFFBFE),
                        mac.read8(0xEFFDFE));
            std::vector<u32> trail;
            for (size_t i = 0; i < ring.size(); ++i) {
                const u32 p = ring[(head + i) % ring.size()];
                if (p && (trail.empty() || trail.back() != p)) trail.push_back(p);
            }
            for (size_t i = 0; i < trail.size(); ++i) {
                std::printf("%06X%s", trail[i], (i % 8 == 7) ? "\n" : " ");
            }
            std::printf("\n");
            return;
        }
        ring[head] = pc;
        head = (head + 1) % ring.size();
        mac.stepInstruction();
    }
    std::printf("-- never reached %06X --\n", stopPc);
}

void dumpBmp(Machine& mac, const std::string& path) {
    const int w = Machine::kScreenW, h = Machine::kScreenH;
    std::vector<u32> pix(static_cast<size_t>(w) * h);
    mac.renderScreen(pix.data());
    const int rowBytes = w * 3;
    const u32 imgSize = static_cast<u32>(rowBytes * h);
    const u32 fileSize = 54 + imgSize;
    u8 hdr[54] = {};
    hdr[0] = 'B'; hdr[1] = 'M';
    hdr[2] = static_cast<u8>(fileSize); hdr[3] = static_cast<u8>(fileSize >> 8);
    hdr[4] = static_cast<u8>(fileSize >> 16); hdr[5] = static_cast<u8>(fileSize >> 24);
    hdr[10] = 54; hdr[14] = 40;
    hdr[18] = static_cast<u8>(w); hdr[19] = static_cast<u8>(w >> 8);
    hdr[22] = static_cast<u8>(h); hdr[23] = static_cast<u8>(h >> 8);
    hdr[26] = 1; hdr[28] = 24;
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(hdr), sizeof hdr);
    std::vector<u8> row(static_cast<size_t>(rowBytes));
    for (int y = h - 1; y >= 0; --y) {
        for (int x = 0; x < w; ++x) {
            const u32 p = pix[static_cast<size_t>(y) * w + x];
            row[x * 3 + 0] = static_cast<u8>(p);
            row[x * 3 + 1] = static_cast<u8>(p >> 8);
            row[x * 3 + 2] = static_cast<u8>(p >> 16);
        }
        f.write(reinterpret_cast<const char*>(row.data()), rowBytes);
    }
    std::printf("screen dumped to %s\n", path.c_str());
}

void dumpBmpAt(Machine& mac, const std::string& path, u32 base);   // fwd decl

// ---- drive the real 6.0.8 Installer headlessly (corruption repro) ---------
//
// Boots to the Finder, then drives the desktop UI with the ADB mouse to open the
// System Startup floppy, launch the Installer, run Easy Install onto the HD, and
// feed the System Additions disk when prompted. The whole point is to reproduce
// the resource-fork corruption under the real Installer's live heap layout, which
// no synthetic harness triggers. Everything is closed-loop off low memory so it
// is deterministic and needs no screen scraping to hit a target.
struct DriveCfg {
    std::string floppy1;      // System Startup image (source, re-inserted after Switch Disk)
    std::string floppy2;      // System Additions image (disk 2)
    std::string shotBase;     // screenshot base path (stage name appended)
    std::string hdOut;        // where to write the HD image afterward
    int t1x = 455, t1y = 22;  // System Startup floppy icon (image center)
    int t2x = 60,  t2y = 60;  // Installer icon inside the floppy window
    int t3x = -1,  t3y = -1;  // Install/OK button (default = press Return)
    int dbGap = 4;            // frames between the two button-downs of a dbl-click
    int downF = 3, upF = 2;   // settled frames to hold each button state
    int midGap = 0;           // extra idle frames between the two clicks
    bool singleOnly = false;  // stage 1: single-click instead of double
    int bootFrames = 6000;    // frames cap to reach a fully-drawn desktop
    int stage = 9;            // run up to this stage
};

int runDriveInstall(Machine& mac, const DriveCfg& cfg) {
    // Count posted mouse events, and log any oversized memory allocation request
    // (NewHandle A122 / NewPtr A11E / SetHandleSize A024 / ReallocHandle: D0 = size).
    // The corruption misreads a resource length as ~7.26 MB; such a request in a 4 MB
    // heap is exactly what raises the Installer's "Out of memory" -- catch it here.
    int pMouseDown = 0, pMouseUp = 0;
    int bigAllocs = 0;
    mac.cpu().onTrap = [&](u16 trap, u32 pc) {
        const u16 t = trap & 0x0FFF;
        if (t == 0x02F || t == 0x12F) {
            const u16 what = static_cast<u16>(mac.cpu().d[0] & 0xFFFF);
            if (what == 1) ++pMouseDown;
            else if (what == 2) ++pMouseUp;
        }
        // Any OS trap carrying a 1-16 MB size in D0: the Memory Manager sizing traps
        // (NewHandle/NewPtr/SetHandleSize/ReallocHandle) put the byte count there, so
        // a ~7.26 MB request (the corruption's misread length) shows up here.
        if ((trap & 0xF800) == 0xA000 && bigAllocs < 60) {
            const u32 sz = mac.cpu().d[0];
            if (sz >= 0x100000u && sz <= 0x1000000u) {
                std::printf("  BIG D0 trap=%04X D0=%u (0x%X) A0=%06X pc=%06X\n",
                            trap, sz, sz, mac.cpu().a[0] & 0xFFFFFF, pc);
                ++bigAllocs;
            }
        }
    };
    auto rd16s = [&](u32 a) { return static_cast<int>(static_cast<s16>(mac.read16(a))); };
    auto rd32  = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
    auto curX = [&] { return rd16s(0x0832); };   // low-mem Mouse.h
    auto curY = [&] { return rd16s(0x0830); };   // low-mem Mouse.v
    auto ticks = [&] { return rd32(0x016A); };
    auto curAp = [&] {
        const u8 n = mac.read8(0x0910);
        std::string s;
        for (int i = 0; i < n && i < 31; ++i) s += static_cast<char>(mac.read8(0x0911 + i));
        return s;
    };
    auto winList = [&] { return rd32(0x09D6); };
    // Decode a WindowRecord: portRect (GrafPort+16), visible (+110), title
    // (titleHandle +134 -> handle -> Str255).
    auto winDump = [&](u32 w) {
        if (!w || w == 0xFFFFFFFFu) { std::printf("  win=%06X (none)\n", w); return; }
        const int t = rd16s(w + 16), l = rd16s(w + 18), b = rd16s(w + 20), r = rd16s(w + 22);
        const u8 vis = mac.read8(w + 110);
        const u32 th = rd32(w + 134);
        std::string title;
        if (th) { const u32 tp = rd32(th); if (tp) { const u8 n = mac.read8(tp);
            for (int i = 0; i < n && i < 40; ++i) title += static_cast<char>(mac.read8(tp + 1 + i)); } }
        const u32 nextW = rd32(w + 144);   // nextWindow
        std::printf("  win=%06X portRect=(%d,%d,%d,%d) vis=%d kind=%d title='%s' next=%06X\n",
                    w, t, l, b, r, vis, rd16s(w + 108), title.c_str(), nextW);
    };
    auto clampd = [](int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; };
    // Inject one mouse report and run frames until the ROM actually consumes it
    // (mouseReports increments), so every button transition is delivered no matter
    // the poll cadence. Returns frames spent.
    auto injectReport = [&](int dx, int dy, bool btn, int maxf) {
        const u32 r0 = mac.adbStats().mouseReports;
        mac.mouseMove(dx, dy, btn);
        for (int i = 0; i < maxf && !mac.cpu().halted; ++i) {
            mac.runFrame();
            if (mac.adbStats().mouseReports > r0) return i + 1;
        }
        return maxf;
    };
    auto moveTo = [&](int tx, int ty) {
        for (int it = 0; it < 120 && !mac.cpu().halted; ++it) {
            const int dx = tx - curX(), dy = ty - curY();
            if (dx == 0 && dy == 0) break;
            injectReport(clampd(dx, -24, 24), clampd(dy, -24, 24), false, 8);
        }
    };
    auto mb = [&] { return mac.read8(0x0172); };   // MBState: bit7=1 up, 0 down
    (void)injectReport;
    // Watch the OS event queue tail ($0150): when an event posts, qTail advances to
    // the new element. Log its what/when/where so we can see the button events the
    // interrupt-level poster actually generates (they bypass the A-trap dispatcher).
    bool logEv = false;
    u32 lastTail = rd32(0x0150);
    auto tick1 = [&] {
        mac.runFrame();
        if (!logEv) return;
        const u32 t = rd32(0x0150);
        if (t && t != lastTail) {
            std::printf("    +EVT what=%u when=%u where=(%d,%d)\n", mac.read16(t + 6),
                        rd32(t + 12), rd16s(t + 18), rd16s(t + 16));
            lastTail = t;
        }
    };
    // Hold a button state, re-asserting every frame so every mouse poll sees it,
    // until low-mem MBState settles to it (the ROM updates MBState a frame or two
    // after the ADB report is consumed) plus a couple of guard frames so the Finder
    // event loop actually samples the level.
    auto holdBtn = [&](bool btn, int minFrames) {
        const u8 want = btn ? 0x00 : 0x80;
        int settled = 0;
        for (int i = 0; i < 30 && !mac.cpu().halted; ++i) {
            mac.mouseMove(0, 0, btn);
            tick1();
            if ((mb() & 0x80) == want) { if (++settled >= minFrames) return; }
        }
    };
    auto doubleClick = [&] {
        logEv = true; lastTail = rd32(0x0150);
        const u32 t0 = ticks();
        holdBtn(true, cfg.downF); holdBtn(false, cfg.upF);
        const u32 t2 = ticks();
        for (int i = 0; i < cfg.midGap && !mac.cpu().halted; ++i) tick1();
        holdBtn(true, cfg.downF); holdBtn(false, cfg.upF);
        std::printf("  dbl: down1@%u down2@%u gap=%u DoubleTime=%u\n",
                    t0, t2, t2 - t0, rd32(0x02F0));
        logEv = false;
    };
    auto singleClick = [&] {
        logEv = true; lastTail = rd32(0x0150);
        std::printf("  click mb=%02X", mb());
        holdBtn(true, cfg.downF);  std::printf(" down->%02X", mb());
        holdBtn(false, cfg.upF);   std::printf(" up->%02X\n", mb());
        logEv = false;
    };
    // Find the frontmost VISIBLE window (skips the always-present invisible Clipboard).
    auto visibleWin = [&]() -> u32 {
        for (u32 w = winList(); w && w != 0xFFFFFFFFu; w = rd32(w + 144))
            if (mac.read8(w + 110) != 0) return w;
        return 0;
    };
    const u32 ramTop = mac.ramSize();
    auto shot = [&](const char* stage) {
        if (cfg.shotBase.empty()) return;
        dumpBmp(mac, cfg.shotBase + "." + stage + ".render.bmp");   // renderScreen (screenBase)
        dumpBmpAt(mac, cfg.shotBase + "." + stage + ".main.bmp", ramTop - 0x5900u);
        dumpBmpAt(mac, cfg.shotBase + "." + stage + ".alt.bmp",  ramTop - 0xD900u);
    };
    auto report = [&](const char* tag) {
        std::printf("[%s] t=%u pc=%06X curAp='%s' winList=%06X cursor=(%d,%d) "
                    "mReports=%u hdW=%u\n", tag, ticks(), mac.cpu().pc, curAp().c_str(),
                    winList(), curX(), curY(), mac.adbStats().mouseReports,
                    mac.hdAccessCount());
    };

    // Is the menu bar drawn? The Finder paints the gray desktop first, then draws
    // the menu bar (mostly white) + icons once its event loop is being serviced.
    // A white top strip is a reliable "desktop fully drawn" signal on the live page.
    // Menu bar drawn? Check BOTH video pages (the Finder can present from either)
    // -- a mostly-white top strip means the desktop + menu bar are painted.
    auto menuBarOn = [&](u32 base) {
        int black = 0;
        for (int y = 2; y < 17; ++y)
            for (int x = 60; x < 452; ++x)
                if (mac.read8(base + static_cast<u32>(y) * 64 + (x >> 3)) & (0x80 >> (x & 7)))
                    ++black;
        return black * 100 < 18 * 15 * 392;
    };
    auto menuBarDrawn = [&] {
        return menuBarOn(ramTop - 0x5900u) || menuBarOn(ramTop - 0xD900u);
    };
    auto activePage = [&] {
        return menuBarOn(ramTop - 0x5900u) ? ramTop - 0x5900u : ramTop - 0xD900u;
    };
    // The floppy icon has white (mostly-clear) body rows that stand out from the 50%
    // desktop dither. Count rows in the top-right icon area (strictly BELOW the menu
    // bar) that are mostly white; the dither has none, a drawn icon has several.
    auto floppyIconDrawn = [&] {
        const u32 base = activePage();
        int whiteRows = 0;
        for (int y = 22; y < 52; ++y) {
            int black = 0;
            for (int x = 435; x < 475; ++x)
                if (mac.read8(base + static_cast<u32>(y) * 64 + (x >> 3)) & (0x80 >> (x & 7)))
                    ++black;
            if (black < 12) ++whiteRows;   // 40-wide row that is mostly clear
        }
        return whiteRows >= 4;
    };

    // Stage 0: boot to the desktop. Nudge the mouse early to clear the boot-time
    // mouse-wait spin ($401606); then keep the event loop ticking with periodic
    // net-zero jitter until the Finder finishes drawing the menu bar + icons.
    for (int i = 0; i < cfg.bootFrames && !mac.cpu().halted; ++i) {
        if (i >= 200 && i < 340 && (i & 3) == 0) mac.mouseMove(3, 2, false);
        if (i > 400 && (i & 7) == 0) mac.mouseMove((i & 8) ? 2 : -2, (i & 16) ? 1 : -1, false);
        mac.runFrame();
        if (i > 1500 && (i & 31) == 0 && menuBarDrawn() && floppyIconDrawn()) {
            std::printf("desktop+icons at frame %d\n", i); break;
        }
    }
    std::printf("desktop menuBar=%d floppyIcon=%d activePage=%06X\n", menuBarDrawn(),
                floppyIconDrawn(), activePage());
    std::printf("DoubleTime=%u\n", rd32(0x02F0));
    report("desktop");
    shot("0desktop");
    // Locate the desktop icons on the ACTIVE video page (screenBase() -- not a
    // fixed offset; the Finder can run from either page). Print black-pixel count
    // per row in the icon column so icon image rows (vs label rows) are visible.
    {
        // Locate icons by flagging pixels that deviate from the 50% desktop
        // checkerboard (black iff (x+y) even). Deviation clusters = icons/labels.
        const u32 base = activePage();
        auto pix = [&](int x, int y) {
            return (mac.read8(base + static_cast<u32>(y) * 64 + (x >> 3)) & (0x80 >> (x & 7))) != 0;
        };
        std::printf("icon deviation map (active page %06X, x=410..512):\n", base);
        for (int y = 20; y < 135; ++y) {
            int c = 0, minx = 999, maxx = -1;
            for (int x = 410; x < 512; ++x) {
                const bool exp = ((x + y) & 1) == 0;
                if (pix(x, y) != exp) { ++c; if (x < minx) minx = x; if (x > maxx) maxx = x; }
            }
            if (c > 2) std::printf("  dev y=%3d cnt=%2d x=[%d..%d]\n", y, c, minx, maxx);
        }
    }
    if (cfg.stage < 1) { report("stop"); return 0; }

    // Count black pixels in a screen rectangle (main video page) -- an icon that
    // inverts on selection changes this sharply.
    auto blackIn = [&](int x0, int y0, int x1, int y1) {
        const u32 base = ramTop - 0x5900u;
        int n = 0;
        for (int y = y0; y < y1; ++y)
            for (int x = x0; x < x1; ++x) {
                const u8 byte = mac.read8(base + static_cast<u32>(y) * 64 + (x >> 3));
                if (byte & (0x80 >> (x & 7))) ++n;
            }
        return n;
    };

    (void)blackIn; (void)pMouseDown; (void)pMouseUp;
    // Stage 1: open the System Startup floppy window (double-click its icon). Poll
    // for a visible window to appear over the following frames.
    moveTo(cfg.t1x, cfg.t1y);
    report("on-floppy");
    const u32 qh0 = rd32(0x014C);
    if (cfg.singleOnly) singleClick(); else doubleClick();
    u32 vw = 0;
    int openedAt = -1;
    for (int i = 0; i < 250 && !mac.cpu().halted; ++i) {
        tick1();
        if (!vw && (vw = visibleWin()) != 0) openedAt = i;
    }
    report("after-dbl1");
    std::printf("  qHead %06X -> %06X (Finder %s events)\n", qh0, rd32(0x014C),
                rd32(0x014C) != qh0 ? "CONSUMED" : "did NOT consume");
    winDump(vw ? vw : winList());
    shot("1floppywin");
    std::printf("STAGE1 %s: visibleWin=%06X openedAt=%d\n",
                vw ? "OPENED" : "NO-WINDOW", vw, openedAt);
    if (cfg.stage < 2) { report("stop"); return 0; }

    // Stage 2: launch the Installer (double-click its icon inside the window).
    moveTo(cfg.t2x, cfg.t2y);
    report("on-installer");
    doubleClick();
    for (int i = 0; i < 400 && !mac.cpu().halted; ++i) {
        mac.runFrame();
        if (curAp() == "Installer") break;
    }
    report("after-dbl2");
    shot("2installer");
    std::printf("STAGE2 %s: curAp='%s'\n",
                curAp() == "Installer" ? "LAUNCHED" : "not-launched", curAp().c_str());
    if (cfg.stage < 3) { report("stop"); return 0; }

    // Draw a modal dialog (nudge the event loop) then park the cursor centrally.
    auto drawDialog = [&](int frames) {
        for (int i = 0; i < frames && !mac.cpu().halted; ++i) {
            mac.mouseMove((i & 1) ? 2 : -2, (i & 2) ? 1 : -1, false);
            mac.runFrame();
        }
        moveTo(240, 175);
    };

    // Post a mouse click into the OS event queue at (x,y): position the cursor (fills
    // the event's where), drive the physical button down (so the queued mouseDown
    // carries a button-down modifier and control-tracking works), then up. Apps that
    // read GetOSEvent (the Installer) never see our ADB button path, so the posted
    // event is what makes their clicks register.
    auto postClick = [&](int x, int y) {
        moveTo(x, y);
        for (int i = 0; i < 4 && !mac.cpu().halted; ++i) { mac.mouseMove(0, 0, true); mac.runFrame(); }
        mac.postMouseButton(true);
        for (int i = 0; i < 6 && !mac.cpu().halted; ++i) { mac.mouseMove(0, 0, true); mac.runFrame(); }
        for (int i = 0; i < 4 && !mac.cpu().halted; ++i) { mac.mouseMove(0, 0, false); mac.runFrame(); }
        mac.postMouseButton(false);
        for (int i = 0; i < 6 && !mac.cpu().halted; ++i) { mac.mouseMove(0, 0, false); mac.runFrame(); }
    };
    // Find rounded-rect push buttons in an x/y band by pairing top+bottom border rows
    // (long horizontal black runs, 14-34px apart, same left edge). Centers, top-down.
    auto findButtons = [&](int xlo, int xhi, int ylo, int yhi) {
        const u32 base = activePage();
        auto pix = [&](int x, int y) {
            return (mac.read8(base + static_cast<u32>(y) * 64 + (x >> 3)) & (0x80 >> (x & 7))) != 0;
        };
        struct Edge { int y, x0, x1; };
        std::vector<Edge> edges;
        for (int y = ylo; y < yhi; ++y) {
            int run = 0, best = 0, b0 = 0, c0 = 0;
            for (int x = xlo; x < xhi; ++x) {
                if (pix(x, y)) { if (run == 0) c0 = x; if (++run > best) { best = run; b0 = c0; } }
                else run = 0;
            }
            if (best >= 24) edges.push_back({y, b0, b0 + best});
        }
        std::vector<std::pair<int,int>> btns;
        for (size_t i = 0; i < edges.size(); ++i)
            for (size_t j = i + 1; j < edges.size(); ++j) {
                const int dy = edges[j].y - edges[i].y;
                if (dy < 14) continue;
                if (dy > 34) break;
                if (std::abs(edges[i].x0 - edges[j].x0) <= 6 && std::abs(edges[i].x1 - edges[j].x1) <= 6) {
                    btns.push_back({(edges[i].x0 + edges[i].x1) / 2, (edges[i].y + edges[j].y) / 2});
                    i = j; break;
                }
            }
        return btns;
    };
    auto clickButtonNear = [&](std::vector<std::pair<int,int>>& btns, int yWant, const char* nm) {
        int best = -1, bestd = 9999;
        for (size_t i = 0; i < btns.size(); ++i) {
            const int d = std::abs(btns[i].second - yWant);
            if (d < bestd) { bestd = d; best = static_cast<int>(i); }
        }
        if (best < 0 || bestd > 40) { std::printf("  button '%s' near y=%d NOT FOUND\n", nm, yWant); return false; }
        std::printf("  click '%s' @(%d,%d)\n", nm, btns[best].first, btns[best].second);
        postClick(btns[best].first, btns[best].second);
        return true;
    };

    // Stage 3: dismiss the Welcome splash, reach Easy Install, retarget the hard disk.
    drawDialog(700);
    report("welcome");
    shot("3welcome");
    { auto b = findButtons(330, 500, 250, 320);
      if (b.empty()) { std::printf("STAGE3 FAIL: no Welcome OK button\n"); return 0; }
      std::printf("  Welcome OK @(%d,%d)\n", b[0].first, b[0].second);
      postClick(b[0].first, b[0].second); }
    drawDialog(500);
    report("easyinstall");
    shot("3easyinstall");
    if (visibleWin() == 0) { std::printf("STAGE3 FAIL: Easy Install did not open\n"); return 0; }

    // Retarget the destination to the hard disk: the Installer opens on the boot
    // floppy (its own disk) with Install disabled, so click "Switch Disk" to cycle to
    // "OpenMac HD". Switch Disk sits around y=213 in the right button column.
    { auto b = findButtons(400, 498, 70, 300);
      std::printf("  Easy Install buttons:");
      for (auto& bb : b) std::printf(" (%d,%d)", bb.first, bb.second);
      std::printf("\n");
      clickButtonNear(b, cfg.t3y >= 0 ? cfg.t3y : 213, "Switch Disk"); }
    drawDialog(400);
    report("switched");
    shot("3switched");
    if (cfg.stage < 4) { report("stop-after-switch"); return 0; }

    auto insertFloppyPath = [&](const std::string& path, const char* tag) {
        if (path.empty()) return;
        std::ifstream sf(path, std::ios::binary);
        std::vector<u8> img{std::istreambuf_iterator<char>(sf), std::istreambuf_iterator<char>()};
        if (!img.empty()) {
            std::printf("  >> insert %s (%zu bytes) f=?\n", tag, img.size());
            mac.insertFloppy(std::move(img), false);
        }
    };

    // Capture the Memory Manager *query* traps and their results around the Install
    // click -- the Installer's pre-install "enough memory?" check. FreeMem(A01C)/
    // MaxMem(A11D)/PurgeSpace(A162)/CompactMem(A04C)/StackSpace(A065) return sizes in
    // D0 (and A0 for grow/contig); a wrong small value is what triggers "Out of memory".
    struct MemQ { u32 ret; u16 trap; };
    std::vector<MemQ> memPend;
    auto isMemQuery = [](u16 t) {
        return t == 0xA01C || t == 0xA11D || t == 0xA162 || t == 0xA04C ||
               t == 0xA065 || t == 0xA061 || t == 0xA063 || t == 0xA166;
    };
    auto memName = [](u16 t) {
        switch (t) { case 0xA01C: return "FreeMem"; case 0xA11D: return "MaxMem";
            case 0xA162: return "PurgeSpace"; case 0xA04C: return "CompactMem";
            case 0xA065: return "StackSpace"; case 0xA061: return "MaxBlock";
            case 0xA063: return "MaxApplZone"; case 0xA166: return "MaxSizeRsrc";
            default: return "?"; }
    };
    bool logMemQ = true;
    // Ring of recent Toolbox/OS traps; when the Installer raises an alert (Alert/
    // StopAlert/... $A985-$A988) -- e.g. "Out of memory" -- dump the ring to see the
    // operation that failed just before it.
    std::vector<std::pair<u16,u32>> trapRing;
    bool alertDumped = false;
    auto prevTrapM = mac.cpu().onTrap;
    mac.cpu().onTrap = [&](u16 trap, u32 pc) {
        if (prevTrapM) prevTrapM(trap, pc);
        if (logMemQ && isMemQuery(trap) && memPend.size() < 200) memPend.push_back({pc + 2, trap});
        if (logMemQ && (trap & 0xF800) == 0xA800) {   // Toolbox trap
            trapRing.push_back({trap, pc});
            if (trapRing.size() > 60) trapRing.erase(trapRing.begin());
        }
        if (logMemQ && !alertDumped && (trap == 0xA985 || trap == 0xA986 || trap == 0xA987 ||
                trap == 0xA988 || trap == 0xA97C || trap == 0xA97D || trap == 0xA913 ||
                trap == 0xA9BD)) {
            alertDumped = true;
            std::printf("  DIALOG/ALERT trap %04X at pc=%06X -- preceding Toolbox traps:\n", trap, pc);
            for (auto& tr : trapRing)
                std::printf("    %04X %-14s @%06X\n", tr.first,
                            openmac::dbg::trapName(tr.first), tr.second);
        }
    };
    auto prevStepM = mac.cpu().onStep;
    mac.cpu().onStep = [&](u32 pc) {
        if (prevStepM) prevStepM(pc);
        for (size_t i = memPend.size(); i-- > 0;)
            if (memPend[i].ret == pc) {
                std::printf("  MEMQ %-11s -> D0=%d (0x%X)  A0=%06X\n", memName(memPend[i].trap),
                            static_cast<s32>(mac.cpu().d[0]), mac.cpu().d[0], mac.cpu().a[0] & 0xFFFFFF);
                memPend.erase(memPend.begin() + static_cast<long>(i));
                break;
            }
    };

    // Stage 4: click Install (target = HD). Switch Disk ejected the source floppy, so
    // the Installer now asks for it, then for System Additions. Do NOT re-insert the
    // source before Install -- that reverts the target back to the Installer disk.
    { auto b = findButtons(400, 498, 70, 160);
      if (!clickButtonNear(b, 94, "Install")) { std::printf("STAGE4 FAIL: no Install button\n"); return 0; } }
    for (int i = 0; i < 40 && !mac.cpu().halted; ++i) mac.runFrame();
    logMemQ = false;
    mac.cpu().onStep = prevStepM;
    report("clicked-install");
    shot("4installing");

    // Feed disks the Installer requests: System Startup (source) shortly after Install,
    // then System Additions when the Installer ejects Startup to ask for it (.Sony
    // eject diag). Cycle if it asks repeatedly.
    int diskState = 0;          // 0=need source, 1=fed source waiting for additions ask, 2+=cycle
    int oomAt = 0;
    bool ejectPending = false;
    mac.onDiag = [&](const char* m) { if (std::strstr(m, "eject")) ejectPending = true; };
    for (int i = 0; i < 20000 && !mac.cpu().halted; ++i) {
        if (i == 250 && diskState == 0) {
            insertFloppyPath(cfg.floppy1, "System Startup (source)");
            diskState = 1; ejectPending = false;
        } else if (ejectPending && diskState >= 1) {
            ejectPending = false;
            const bool additions = (diskState % 2) == 1;
            insertFloppyPath(additions ? cfg.floppy2 : cfg.floppy1,
                             additions ? "System Additions" : "System Startup");
            ++diskState;
        }
        mac.mouseMove((i & 1) ? 1 : -1, 0, false);   // keep the event loop ticking
        mac.runFrame();
        if (!oomAt && static_cast<u16>(mac.read16(0x0220)) == 0xFF94) {   // MemError = memFullErr (-108)
            oomAt = i;
            std::printf("  *** MemError=-108 (memFullErr) at frame %d pc=%06X; recent PCs:\n   ", i, mac.cpu().pc);
            for (int b = 24; b >= 0; --b) std::printf(" %06X", mac.cpu().recentPc(b));
            std::printf("\n");
        }
        if (i % 800 == 0) {
            const u32 memTop = rd32(0x0108), applZone = rd32(0x02AA), applLimit = rd32(0x0130),
                      heapEnd = rd32(0x0114);
            std::printf("  merge f=%d pc=%06X hdAcc=%u win=%06X ds=%d | MemTop=%06X ApplZone=%06X "
                        "HeapEnd=%06X ApplLimit=%06X appHeap=%uKB MemErr=%d\n", i, mac.cpu().pc,
                        mac.hdAccessCount(), winList(), diskState, memTop, applZone, heapEnd,
                        applLimit, (heapEnd - applZone) / 1024, static_cast<s16>(mac.read16(0x0220)));
            if (i % 3200 == 0) shot("4merge");
        }
    }
    std::printf("  install OOM at frame %d\n", oomAt);
    report("done");
    shot("4done");
    if (!cfg.hdOut.empty()) {
        const auto& hd = mac.hardDiskImage();
        std::ofstream of(cfg.hdOut, std::ios::binary);
        of.write(reinterpret_cast<const char*>(hd.data()),
                 static_cast<std::streamsize>(hd.size()));
        std::printf("HD image (%zu bytes) -> %s\n", hd.size(), cfg.hdOut.c_str());
    }
    return 0;
}

// Dump a 512x342 1-bpp frame from an explicit RAM base (bypasses renderScreen's
// page selection, which can point at the stale alternate video page during Finder
// interactions). base = ram top - 0x5900 (main) or - 0xD900 (alt) for 4 MB.
void dumpBmpAt(Machine& mac, const std::string& path, u32 base) {
    const int w = Machine::kScreenW, h = Machine::kScreenH;
    const int rowBytes = w * 3;
    const u32 imgSize = static_cast<u32>(rowBytes * h);
    const u32 fileSize = 54 + imgSize;
    u8 hdr[54] = {};
    hdr[0] = 'B'; hdr[1] = 'M';
    hdr[2] = static_cast<u8>(fileSize); hdr[3] = static_cast<u8>(fileSize >> 8);
    hdr[4] = static_cast<u8>(fileSize >> 16); hdr[5] = static_cast<u8>(fileSize >> 24);
    hdr[10] = 54; hdr[14] = 40;
    hdr[18] = static_cast<u8>(w); hdr[19] = static_cast<u8>(w >> 8);
    hdr[22] = static_cast<u8>(h); hdr[23] = static_cast<u8>(h >> 8);
    hdr[26] = 1; hdr[28] = 24;
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(hdr), sizeof hdr);
    std::vector<u8> row(static_cast<size_t>(rowBytes));
    for (int y = h - 1; y >= 0; --y) {
        for (int xb = 0; xb < w / 8; ++xb) {
            const u8 bits = mac.read8(base + static_cast<u32>(y) * (w / 8) + xb);
            for (int b = 0; b < 8; ++b) {
                const bool black = (bits & (0x80 >> b)) != 0;
                const u8 v = black ? 0 : 255;
                const int x = xb * 8 + b;
                row[x * 3 + 0] = v; row[x * 3 + 1] = v; row[x * 3 + 2] = v;
            }
        }
        f.write(reinterpret_cast<const char*>(row.data()), rowBytes);
    }
}

void writeWav(const std::string& path, const std::vector<u8>& s, int rate) {
    std::ofstream f(path, std::ios::binary);
    auto w32 = [&](u32 v) { for (int i = 0; i < 4; ++i) f.put(static_cast<char>((v >> (8 * i)) & 0xFF)); };
    auto w16 = [&](u16 v) { f.put(static_cast<char>(v & 0xFF)); f.put(static_cast<char>((v >> 8) & 0xFF)); };
    const u32 n = static_cast<u32>(s.size());
    f.write("RIFF", 4); w32(36 + n); f.write("WAVE", 4);
    f.write("fmt ", 4); w32(16); w16(1); w16(1); w32(static_cast<u32>(rate));
    w32(static_cast<u32>(rate)); w16(1); w16(8);
    f.write("data", 4); w32(n);
    f.write(reinterpret_cast<const char*>(s.data()), n);
}

// Bcc condition mnemonics indexed by (opcode >> 8) & 0xF; index 0/1 are BRA/BSR.
const char* const kBcc[16] = {"BRA","BSR","BHI","BLS","BCC","BCS","BNE","BEQ",
                              "BVC","BVS","BPL","BMI","BGE","BLT","BGT","BLE"};

// ---- conditional breakpoints ---------------------------------------------

// A parsed --break-if predicate: <lhs> <op> <hex>, where lhs is a data reg,
// an address reg, or a 32-bit memory word [addr]. Values are unsigned 32-bit.
struct BreakCond {
    bool active = false;
    int  lhs = 0;      // 0=Dn, 1=An, 2=[mem]
    int  reg = 0;      // register index for lhs 0/1
    u32  addr = 0;     // memory address for lhs 2
    int  op = 0;       // 0 == 1 != 2 < 3 > 4 <= 5 >=
    u32  rhs = 0;
};

// Parse "D0==5", "A0!=0", "[B30]==0" (spaces ignored, hex accepts 0x). Returns
// false and leaves `out` inactive on any syntax error.
bool parseBreakCond(const std::string& in, BreakCond& out) {
    std::string s;
    for (char ch : in) if (ch != ' ' && ch != '\t') s += ch;
    if (s.empty()) return false;
    size_t i = 0;
    if (s[0] == '[') {
        const size_t close = s.find(']');
        if (close == std::string::npos) return false;
        out.lhs = 2;
        out.addr = static_cast<u32>(std::strtoul(s.substr(1, close - 1).c_str(), nullptr, 16));
        i = close + 1;
    } else if (s.size() >= 2 && (s[0] == 'D' || s[0] == 'd' || s[0] == 'A' || s[0] == 'a') &&
               s[1] >= '0' && s[1] <= '7') {
        out.lhs = (s[0] == 'A' || s[0] == 'a') ? 1 : 0;
        out.reg = s[1] - '0';
        i = 2;
    } else {
        return false;
    }
    if (i >= s.size()) return false;
    if      (s.compare(i, 2, "==") == 0) { out.op = 0; i += 2; }
    else if (s.compare(i, 2, "!=") == 0) { out.op = 1; i += 2; }
    else if (s.compare(i, 2, "<=") == 0) { out.op = 4; i += 2; }
    else if (s.compare(i, 2, ">=") == 0) { out.op = 5; i += 2; }
    else if (s[i] == '<')                { out.op = 2; i += 1; }
    else if (s[i] == '>')                { out.op = 3; i += 1; }
    else return false;
    if (i >= s.size()) return false;
    out.rhs = static_cast<u32>(std::strtoul(s.c_str() + i, nullptr, 16));
    out.active = true;
    return true;
}

// Evaluate the predicate against live CPU/bus state (unsigned comparisons).
bool evalBreakCond(const BreakCond& c, Machine& mac) {
    const M68000& cpu = mac.cpu();
    u32 lhs = 0;
    if (c.lhs == 0)      lhs = cpu.d[c.reg];
    else if (c.lhs == 1) lhs = cpu.a[c.reg];
    else                 lhs = (u32(mac.read16(c.addr)) << 16) | mac.read16(c.addr + 2);
    switch (c.op) {
        case 0: return lhs == c.rhs;
        case 1: return lhs != c.rhs;
        case 2: return lhs <  c.rhs;
        case 3: return lhs >  c.rhs;
        case 4: return lhs <= c.rhs;
        case 5: return lhs >= c.rhs;
    }
    return false;
}

// ---- step-over / step-out ------------------------------------------------

// Print the D/A registers that changed between a snapshot and the live CPU.
void reportRegDelta(const u32* dBefore, const u32* aBefore, const M68000& cpu) {
    for (int i = 0; i < 8; ++i)
        if (cpu.d[i] != dBefore[i])
            std::printf("    D%d %08X -> %08X\n", i, dBefore[i], cpu.d[i]);
    for (int i = 0; i < 8; ++i)
        if (cpu.a[i] != aBefore[i])
            std::printf("    A%d %08X -> %08X\n", i, aBefore[i], cpu.a[i]);
}

// Single-step until PC first equals `target`. Returns true if reached.
bool runToPc(Machine& mac, u32 target, u64 maxCycles) {
    while (mac.totalCycles() < maxCycles && !mac.cpu().halted) {
        if (mac.cpu().pc == target) return true;
        mac.stepInstruction();
    }
    return mac.cpu().pc == target;
}

// Run to `target`, then execute the instruction there. If it is a JSR/BSR,
// keep stepping until the call returns (A7 back to its pre-call level); print
// the call, its return site, and the register delta. Otherwise a single step.
void stepOver(Machine& mac, u32 target, u64 maxCycles) {
    if (!runToPc(mac, target, maxCycles)) {
        std::printf("-- step-over: never reached %06X --\n", target);
        return;
    }
    const M68000& cpu = mac.cpu();
    const u16 op = mac.read16(target);
    const bool isCall = (op & 0xFFC0) == 0x4E80 || (op & 0xFF00) == 0x6100;   // JSR / BSR
    u32 dBefore[8], aBefore[8];
    std::memcpy(dBefore, cpu.d, sizeof dBefore);
    std::memcpy(aBefore, cpu.a, sizeof aBefore);
    std::string dis;
    openmac::dbg::disasm(mac, target, dis);
    const std::string sym = openmac::dbg::symbolFor(mac, target);
    if (!isCall) {
        mac.stepInstruction();
        std::printf("-- step %06X %-18s %-24s (not a call) --\n", target, sym.c_str(),
                    dis.c_str());
        reportRegDelta(dBefore, aBefore, cpu);
        std::printf("    -> now pc=%06X\n", cpu.pc);
        return;
    }
    const u32 retSp = cpu.a[7];        // SP just before the call pushes its return address
    long steps = 0;
    const long kCap = 2000000;
    mac.stepInstruction();             // execute the JSR/BSR
    ++steps;
    while (steps < kCap && !mac.cpu().halted && cpu.a[7] < retSp) {
        mac.stepInstruction();
        ++steps;
    }
    std::printf("-- stepped over CALL at %06X %-18s %-24s -> returned to %06X %s "
                "(%ld steps) --\n", target, sym.c_str(), dis.c_str(), cpu.pc,
                openmac::dbg::symbolFor(mac, cpu.pc).c_str(), steps);
    reportRegDelta(dBefore, aBefore, cpu);
}

// Run to `target`, then single-step (disassembling each instruction) until an
// RTS/RTE pops the stack back above the entry SP — the current routine's own
// return — and stop there.
void stepOut(Machine& mac, u32 target, u64 maxCycles) {
    if (!runToPc(mac, target, maxCycles)) {
        std::printf("-- step-out: never reached %06X --\n", target);
        return;
    }
    const M68000& cpu = mac.cpu();
    const u32 entrySp = cpu.a[7];
    u32 dBefore[8], aBefore[8];
    std::memcpy(dBefore, cpu.d, sizeof dBefore);
    std::memcpy(aBefore, cpu.a, sizeof aBefore);
    std::printf("-- step-out from %06X %s (entrySP=%06X) --\n", target,
                openmac::dbg::symbolFor(mac, target).c_str(), entrySp);
    long steps = 0, printed = 0;
    const long kCap = 2000000, kPrintCap = 20000;
    while (steps < kCap && !mac.cpu().halted) {
        const u32 pc = cpu.pc;
        const u16 op = mac.read16(pc);
        const bool isRet = op == 0x4E75 || op == 0x4E73;        // RTS / RTE
        if (printed < kPrintCap) {
            std::string dis;
            openmac::dbg::disasm(mac, pc, dis);
            std::printf("  %06X  %-18s %s\n", pc,
                        openmac::dbg::symbolFor(mac, pc).c_str(), dis.c_str());
            if (++printed == kPrintCap) std::printf("  ... (trace truncated) ...\n");
        }
        mac.stepInstruction();
        ++steps;
        if (isRet && cpu.a[7] > entrySp) {
            std::printf("-- returned to %06X %s (%ld steps) --\n", cpu.pc,
                        openmac::dbg::symbolFor(mac, cpu.pc).c_str(), steps);
            reportRegDelta(dBefore, aBefore, cpu);
            return;
        }
    }
    std::printf("-- step-out: no return within %ld steps (pc=%06X) --\n", steps, cpu.pc);
}

} // namespace

int main(int argc, char** argv) {
    std::string romPath;
    int frames = 60;
    u32 ramMB = 4;
    int profileAt = -1;
    u32 traceToPc = 0;
    u32 watchAddr = 0xFFFFFFFFu;
    bool mouseWalk = false;
    bool bootNudge = false;
    bool keyTest = false;
    std::string swapFloppyPath;
    bool bootDisk = false;
    bool forceRom = false;
    std::string dumpPath;
    std::string floppyPath;
    std::string hdImagePath;   // --harddisk <path>: attach an existing HD image (HFS volume)
    u32 hdBlankMB = 0;   // --harddisk-blank N: attach a blank N-MB hard disk
    u32 hdFormatMB = 0;  // --harddisk-format N: attach a formatted N-MB HFS disk
    bool traceTraps = false, lowmemDump = false, traceOsTraps = false, checkHeapFlag = false;
    bool traceIrq = false, traceAdb = false, traceDevIo = false;
    u32 breakPc = 0, watchMem = 0xFFFFFFFFu;
    u32 breakTrap = 0, tracePc = 0, dumpMemAddr = 0, dumpMemLen = 0;
    int traceCount = 48;
    u32 stepOverPc = 0, stepOutPc = 0;
    bool stepOverSet = false, stepOutSet = false;
    std::string breakCondStr, dumpStructName;
    int breakCount = 0;
    u32 dumpStructAddr = 0;
    bool traceBranches = false, dumpStructSet = false;
    u32 disasmAddr = 0; int disasmCount = 0; bool disasmSet = false;
    bool driveInstall = false;
    int traceIwm = 0; bool noSonyShim = false; bool sonyShimOn = false; bool swimOn = false, swimOff = false;
    u16 sonyLineMask = 0, sonyLineValue = 0;
    bool floppy800k = false, insert800k = false, insert800kRW = false;
    std::string floppy800kPath, insert800kOut, externalPath;
    std::string floppyOutPath;
    std::string insertExternalPath;
    int insertExternalAt = 2600;
    bool externalEmpty = false;
    DriveCfg dcfg;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--rom" && i + 1 < argc) romPath = argv[++i];
        else if (arg == "--floppy" && i + 1 < argc) floppyPath = argv[++i];
        else if (arg == "--harddisk-blank" && i + 1 < argc)
            hdBlankMB = static_cast<u32>(std::atoi(argv[++i]));
        else if (arg == "--harddisk-format" && i + 1 < argc)
            hdFormatMB = static_cast<u32>(std::atoi(argv[++i]));
        else if (arg == "--harddisk" && i + 1 < argc) hdImagePath = argv[++i];
        else if (arg == "--trace-traps") traceTraps = true;
        else if (arg == "--trace-irq") traceIrq = true;
        else if (arg == "--trace-adb") traceAdb = true;
        else if (arg == "--trace-os-traps") traceOsTraps = true;
        else if (arg == "--trace-devio") traceDevIo = true;
        else if (arg == "--trace-iwm" && i + 1 < argc)
            traceIwm = std::atoi(argv[++i]);
        else if (arg == "--no-sony-shim") noSonyShim = true;
        else if (arg == "--sony-shim") sonyShimOn = true;
        else if (arg == "--swim") swimOn = true;
        else if (arg == "--no-swim") swimOff = true;
        else if (arg == "--external" && i + 1 < argc) externalPath = argv[++i];
        else if (arg == "--external-empty") externalEmpty = true;
        else if (arg == "--floppy-800k") {
            floppy800k = true;
            // Optional path: a real 800K image rather than a synthetic volume.
            if (i + 1 < argc && argv[i + 1][0] != '-') floppy800kPath = argv[++i];
        }
        else if (arg == "--insert-800k") insert800k = true;
        else if (arg == "--insert-800k-rw") { insert800k = true; insert800kRW = true; }
        else if (arg == "--insert-800k-out" && i + 1 < argc) insert800kOut = argv[++i];
        // Write the internal drive's medium out at the end of any run, so what the
        // guest did to a disk can be diffed against what it started as.
        else if (arg == "--floppy-out" && i + 1 < argc) floppyOutPath = argv[++i];
        else if (arg == "--insert-external" && i + 1 < argc) insertExternalPath = argv[++i];
        else if (arg == "--insert-external-at" && i + 1 < argc) insertExternalAt = std::atoi(argv[++i]);
        else if (arg == "--sony-lines" && i + 2 < argc) {
            sonyLineMask  = static_cast<u16>(std::strtoul(argv[++i], nullptr, 16));
            sonyLineValue = static_cast<u16>(std::strtoul(argv[++i], nullptr, 16));
        }
        else if (arg == "--break-pc" && i + 1 < argc)
            breakPc = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        else if (arg == "--watch-mem" && i + 1 < argc)
            watchMem = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        else if (arg == "--check-heap") checkHeapFlag = true;
        else if (arg == "--lowmem") lowmemDump = true;
        else if (arg == "--break-trap" && i + 1 < argc)
            breakTrap = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        else if (arg == "--trace-pc" && i + 1 < argc)
            tracePc = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        else if (arg == "--trace-count" && i + 1 < argc) traceCount = std::atoi(argv[++i]);
        else if (arg == "--dump-mem" && i + 2 < argc) {
            dumpMemAddr = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
            dumpMemLen = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        }
        else if (arg == "--frames" && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (arg == "--ram-mb" && i + 1 < argc) ramMB = static_cast<u32>(std::atoi(argv[++i]));
        else if (arg == "--profile" && i + 1 < argc) profileAt = std::atoi(argv[++i]);
        else if (arg == "--trace-to" && i + 1 < argc) {
            traceToPc = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        }
        else if (arg == "--dump-screen" && i + 1 < argc) dumpPath = argv[++i];
        else if (arg == "--watch" && i + 1 < argc) {
            watchAddr = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
        }
        else if (arg == "--mouse-walk") mouseWalk = true;
        else if (arg == "--boot-nudge") bootNudge = true;
        else if (arg == "--key-test") keyTest = true;
        else if (arg == "--swap-floppy" && i + 1 < argc) swapFloppyPath = argv[++i];
        else if (arg == "--boot-disk") bootDisk = true;
        else if (arg == "--force-rom") forceRom = true;
        else if (arg == "--step-over" && i + 1 < argc) {
            stepOverPc = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
            stepOverSet = true;
        }
        else if (arg == "--step-out" && i + 1 < argc) {
            stepOutPc = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
            stepOutSet = true;
        }
        else if (arg == "--break-if" && i + 1 < argc) breakCondStr = argv[++i];
        else if (arg == "--break-count" && i + 1 < argc) breakCount = std::atoi(argv[++i]);
        else if (arg == "--trace-branches") traceBranches = true;
        else if (arg == "--dump-struct" && i + 2 < argc) {
            dumpStructAddr = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
            dumpStructName = argv[++i];
            dumpStructSet = true;
        }
        else if (arg == "--disasm" && i + 2 < argc) {
            disasmAddr = static_cast<u32>(std::strtoul(argv[++i], nullptr, 16));
            disasmCount = std::atoi(argv[++i]);
            disasmSet = true;
        }
        else if (arg == "--drive-install") driveInstall = true;
        else if (arg == "--floppy2" && i + 1 < argc) dcfg.floppy2 = argv[++i];
        else if (arg == "--di-shot" && i + 1 < argc) dcfg.shotBase = argv[++i];
        else if (arg == "--di-hdout" && i + 1 < argc) dcfg.hdOut = argv[++i];
        else if (arg == "--di-t1" && i + 2 < argc) { dcfg.t1x = std::atoi(argv[++i]); dcfg.t1y = std::atoi(argv[++i]); }
        else if (arg == "--di-t2" && i + 2 < argc) { dcfg.t2x = std::atoi(argv[++i]); dcfg.t2y = std::atoi(argv[++i]); }
        else if (arg == "--di-t3" && i + 2 < argc) { dcfg.t3x = std::atoi(argv[++i]); dcfg.t3y = std::atoi(argv[++i]); }
        else if (arg == "--di-gap" && i + 1 < argc) dcfg.dbGap = std::atoi(argv[++i]);
        else if (arg == "--di-down" && i + 1 < argc) dcfg.downF = std::atoi(argv[++i]);
        else if (arg == "--di-up" && i + 1 < argc) dcfg.upF = std::atoi(argv[++i]);
        else if (arg == "--di-mid" && i + 1 < argc) dcfg.midGap = std::atoi(argv[++i]);
        else if (arg == "--di-single") dcfg.singleOnly = true;
        else if (arg == "--di-boot" && i + 1 < argc) dcfg.bootFrames = std::atoi(argv[++i]);
        else if (arg == "--di-stage" && i + 1 < argc) dcfg.stage = std::atoi(argv[++i]);
    }
    if (romPath.empty()) {
        std::fprintf(stderr, "usage: openmac_trace --rom <path> [--frames N] [--ram-mb M]\n");
        return 2;
    }
    std::ifstream f(romPath, std::ios::binary);
    std::vector<u8> rom{std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
    if (rom.empty()) {
        std::fprintf(stderr, "cannot read ROM: %s\n", romPath.c_str());
        return 2;
    }
    std::printf("ROM %zu bytes, header checksum %02X%02X%02X%02X, version %02X%02X\n",
                rom.size(), rom[0], rom[1], rom[2], rom[3], rom[8], rom[9]);

    Machine mac(std::move(rom), {ramMB * 1024u * 1024u});
    mac.onDiag = [](const char* s) { std::printf("[diag] %s\n", s); };
    if (forceRom) mac.setForceRomDisk(true);
    // The Classic has a SWIM, so the chip answers the ROM's probe by default.
    // --no-swim makes it answer as a plain IWM instead, which is worth being
    // able to force: the probe's outcome decides the whole disk path.
    if (swimOff) {
        mac.setSwimEnabled(false);
        std::printf("SWIM answer SUPPRESSED -- the chip reports itself a plain IWM\n");
    } else if (swimOn) {
        std::printf("SWIM/ISM register set enabled (the default)\n");
    }
    // The ROM's own .Sony driver runs the hardware; there is no longer anything
    // to switch. --no-sony-shim is kept as a no-op so old command lines still run.
    if (noSonyShim || sonyShimOn)
        std::printf(".Sony driver: the ROM's own runs the hardware\n");
    if (externalEmpty || (!externalPath.empty() && insert800k)) {
        mac.setExternalDriveAttached(true);
        std::printf("EXTERNAL drive attached, no disk\n");
    }
    if (!externalPath.empty()) {
        std::ifstream ef(externalPath, std::ios::binary);
        std::vector<u8> eimg{std::istreambuf_iterator<char>(ef),
                             std::istreambuf_iterator<char>()};
        if (eimg.empty()) {
            std::fprintf(stderr, "cannot read external disk: %s\n", externalPath.c_str());
            return 2;
        }
        std::printf("EXTERNAL drive: %zu bytes inserted\n", eimg.size());
        mac.insertExternalFloppy(std::move(eimg), false);
    }
    if (sonyLineMask) {
        mac.setSonyLineOverride(sonyLineMask, sonyLineValue);
        std::printf("Sony line override: mask=%04X value=%04X\n", sonyLineMask, sonyLineValue);
    }
    // Disk-chip bus survey: what does the ROM actually ask the IWM/SWIM for?
    // The 16 soft switches (IWM mode); each odd address sets a line, each even clears it.
    static const char* kIwmReg[16] = {
        "ca0 off", "ca0 ON ", "ca1 off", "ca1 ON ", "ca2 off", "ca2 ON ",
        "LSTRB off", "LSTRB ON", "ENABLE off", "ENABLE ON", "sel int", "sel ext",
        "q6 off", "q6 ON ", "q7 off(rd)", "q7 ON(wr)"};
    // Sony drive status lines, addressed CA2:CA1:CA0:SEL (Inside Macintosh III-35).
    // The five unlisted addresses are the SuperDrive-era extensions, which that
    // 1985 table predates -- the ROM polls two of them ($A and $E).
    static const char* kDriveReg[16] = {
        "DIRTN", "CSTIN", "STEP", "WRTPRT", "MOTORON", "TK0", "(unlisted 6)", "TACH",
        "RDDATA0", "RDDATA1", "(unlisted A)", "(unlisted B)", "SIDES", "(unlisted D)",
        "(unlisted E)", "DRVIN"};
    long long iwmTotal = 0;
    unsigned iwmRegHits[16] = {}, driveRegReads[16] = {};
    std::vector<u32> iwmPcs;
    if (traceIwm > 0 || noSonyShim) {
        mac.onIwmAccess = [&, traceIwm](int reg, bool write, u8 data, u8 lines, u32 pc, int drvReg) {
            ++iwmTotal;
            ++iwmRegHits[reg & 15];
            // A q7-off read with q6 on selects the Status register, which carries the
            // currently addressed drive sense line in bit 7.
            if (!write && (reg & 15) == 0xE && (lines & 0x40)) ++driveRegReads[drvReg & 15];
            if (std::find(iwmPcs.begin(), iwmPcs.end(), pc) == iwmPcs.end() && iwmPcs.size() < 64)
                iwmPcs.push_back(pc);
            if (iwmTotal <= traceIwm) {
                // Once the chip is unlocked into ISM mode the same sixteen
                // addresses are a different register file: A3 is the read/write
                // line, so n is written at +512*n and read at +512*(n+8).
                static const char* kIsmW[8] = {"W Data", "W Mark", "W CRC/cfg", "W ParamRAM",
                                               "W Phase", "W Setup", "W Mode-0", "W Mode-1"};
                static const char* kIsmR[8] = {"R Data", "R Mark", "R Error", "R ParamRAM",
                                               "R Phase", "R Setup", "R Status", "R Handshake"};
                const bool ism = mac.iwmInIsmMode();
                const char* name = ism ? ((reg & 8) ? kIsmR[reg & 7] : kIsmW[reg & 7])
                                       : kIwmReg[reg & 15];
                std::printf("%s %5lld pc=%06X %-11s %s %02X  lines=%02X [lstrb=%d enbl=%d "
                            "drv=%d q6=%d q7=%d] drvReg=%X %s\n",
                            ism ? "ISM" : "IWM",
                            iwmTotal, pc, name, write ? "W" : "R", data, lines,
                            (lines >> 3) & 1, (lines >> 4) & 1, (lines >> 5) & 1,
                            (lines >> 6) & 1, (lines >> 7) & 1, drvReg & 15,
                            kDriveReg[drvReg & 15]);
            }
        };
    }
    if (!floppyPath.empty()) {
        std::ifstream ff(floppyPath, std::ios::binary);
        std::vector<u8> img{std::istreambuf_iterator<char>(ff),
                            std::istreambuf_iterator<char>()};
        if (img.empty()) {
            std::fprintf(stderr, "cannot read floppy: %s\n", floppyPath.c_str());
            return 2;
        }
        std::printf("FLOPPY %zu bytes inserted\n", img.size());
        mac.insertFloppy(std::move(img), false);
    }
    if (floppy800k && !insert800k) {
        // An 800K GCR volume: 1600 sectors, the geometry the IWM read path
        // encodes. 1.44MB media is MFM and needs the SWIM's ISM mode, so this is
        // the medium the GCR path is exercised on. With a path, a real 800K
        // image is used instead of a freshly formatted one.
        std::vector<u8> img;
        if (!floppy800kPath.empty()) {
            std::ifstream f8(floppy800kPath, std::ios::binary);
            img.assign(std::istreambuf_iterator<char>(f8), std::istreambuf_iterator<char>());
            if (img.empty()) {
                std::fprintf(stderr, "cannot read 800K image: %s\n", floppy800kPath.c_str());
                return 2;
            }
            std::printf("FLOPPY %zu bytes from %s inserted\n", img.size(),
                        floppy800kPath.c_str());
        } else {
            img = openmac::hfs::formatVolume(819200u, "OpenMac 800K");
            std::printf("FLOPPY %zu bytes (800K HFS) inserted\n", img.size());
        }
        mac.insertFloppy(std::move(img), false);
    }
    if (hdBlankMB > 0) {
        std::vector<u8> hd(static_cast<size_t>(hdBlankMB) * 1024u * 1024u, 0u);
        std::printf("HARD DISK %zu bytes (blank) attached\n", hd.size());
        mac.insertHardDisk(std::move(hd), false);
    }
    if (hdFormatMB > 0) {
        auto hd = openmac::hfs::formatVolume(hdFormatMB * 1024u * 1024u, "OpenMac HD");
        std::printf("HARD DISK %zu bytes (HFS-formatted) attached\n", hd.size());
        mac.insertHardDisk(std::move(hd), false);
    }
    if (!hdImagePath.empty()) {
        std::ifstream hf(hdImagePath, std::ios::binary);
        if (!hf) { std::printf("HARD DISK: cannot open %s\n", hdImagePath.c_str()); return 1; }
        std::vector<u8> hd{std::istreambuf_iterator<char>(hf), std::istreambuf_iterator<char>()};
        std::printf("HARD DISK %zu bytes loaded from %s\n", hd.size(), hdImagePath.c_str());
        mac.insertHardDisk(std::move(hd), false);
    }

    if (traceTraps || breakTrap) {
        mac.cpu().onTrap = [&](u16 trap, u32 pc) {
            if (traceTraps) {
                std::string s;
                if (openmac::dbg::describeIOTrap(mac, trap, pc, mac.cpu().a[0], s))
                    std::printf("TRAP %s\n", s.c_str());
            }
            if (breakTrap && (trap & 0x0FFFu) == (breakTrap & 0x0FFFu)) {
                std::printf("\n=== BREAK trap %04X at pc=%06X ===\n", trap, pc);
                openmac::dbg::dumpRegs(mac.cpu(), stdout);
                openmac::dbg::dumpDriveQueue(mac, stdout);
                openmac::dbg::dumpUnitTable(mac, stdout);
            }
        };
    }

    if (traceIrq) {
        mac.cpu().onInterrupt = [&](int level, u32 vec, u32 pc) {
            const u8 ifr = level == 1 ? mac.viaRegs().ifr : 0;    // VIA IFR = source
            const char* src = (ifr & 0x40) ? "T1" : (ifr & 0x20) ? "T2"
                            : (ifr & 0x10) ? "CB1" : (ifr & 0x04) ? "SR/ADB"
                            : (ifr & 0x02) ? "CA1/VBL" : (ifr & 0x01) ? "CA2" : "?";
            static int n = 0;
            if (n++ < 2000)
                std::printf("IRQ L%d vec=%u pc=%06X ifr=%02X %-8s cyc=%llu\n", level, vec,
                            pc, ifr, src, static_cast<unsigned long long>(mac.totalCycles()));
        };
    }

    u32 wPrevPc = 0, wLastVal = 0;
    bool wFirst = true;
    int bpHits = 0, bpQualified = 0, branchLines = 0;
    BreakCond breakCond;
    if (!breakCondStr.empty() && !parseBreakCond(breakCondStr, breakCond))
        std::fprintf(stderr, "warning: could not parse --break-if '%s'; ignoring\n",
                     breakCondStr.c_str());
    if (breakPc || watchMem != 0xFFFFFFFFu || traceBranches) {
        mac.cpu().onStep = [&](u32 pc) {
            if (breakPc && pc == breakPc &&
                (!breakCond.active || evalBreakCond(breakCond, mac))) {
                ++bpQualified;
                const bool fire = breakCount > 0 ? bpQualified == breakCount : bpHits < 8;
                if (fire) {
                    ++bpHits;
                    std::printf("\n=== BREAK pc=%06X (hit %d) cyc=%llu ===\n", pc, bpHits,
                                static_cast<unsigned long long>(mac.totalCycles()));
                    if (breakCond.active || breakCount > 0)
                        std::printf("  qualifying hit %d%s\n", bpQualified,
                                    breakCount > 0 ? " (matched --break-count)" : "");
                    openmac::dbg::dumpRegs(mac.cpu(), stdout);
                    std::string dis;
                    openmac::dbg::disasm(mac, pc, dis);
                    std::printf("  %06X  %s\n", pc, dis.c_str());
                    openmac::dbg::dumpBacktrace(mac.cpu(), mac, stdout);
                }
            }
            if (traceBranches && branchLines < 5000) {
                const u16 op = mac.read16(pc);
                const char* mnem = nullptr;
                u32 target = 0;
                bool haveTarget = false;
                if ((op & 0xF000) == 0x6000) {              // Bcc / BRA / BSR
                    mnem = kBcc[(op >> 8) & 0xF];
                    if ((op & 0xFF) == 0x00)
                        target = pc + 2 + s16(mac.read16(pc + 2));
                    else if ((op & 0xFF) == 0xFF)
                        target = pc + 2 + s32((u32(mac.read16(pc + 2)) << 16) | mac.read16(pc + 4));
                    else
                        target = pc + 2 + s8(op & 0xFF);
                    haveTarget = true;
                } else if ((op & 0xF0F8) == 0x50C8) {       // DBcc
                    mnem = "DBcc";
                    target = pc + 2 + s16(mac.read16(pc + 2));
                    haveTarget = true;
                } else if ((op & 0xFFC0) == 0x4E80) mnem = "JSR";
                else if ((op & 0xFFC0) == 0x4EC0) mnem = "JMP";
                else if (op == 0x4E75) mnem = "RTS";
                else if (op == 0x4E73) mnem = "RTE";
                else if (op == 0x4E77) mnem = "RTR";
                if (mnem) {
                    ++branchLines;
                    if (haveTarget)
                        std::printf("BR %06X %-18s %-5s %06X %s\n", pc,
                                    openmac::dbg::symbolFor(mac, pc).c_str(), mnem,
                                    target & 0xFFFFFF,
                                    openmac::dbg::symbolFor(mac, target & 0xFFFFFF).c_str());
                    else
                        std::printf("BR %06X %-18s %s\n", pc,
                                    openmac::dbg::symbolFor(mac, pc).c_str(), mnem);
                    if (branchLines == 5000)
                        std::printf("-- branch trace cap (5000) reached --\n");
                }
            }
            if (watchMem != 0xFFFFFFFFu) {
                const u32 v = (static_cast<u32>(mac.read16(watchMem)) << 16) |
                              mac.read16(watchMem + 2);
                if (wFirst) { wLastVal = v; wFirst = false; }
                else if (v != wLastVal) {
                    std::printf("WATCH [%06X] %08X -> %08X by pc=%06X cyc=%llu\n", watchMem,
                                wLastVal, v, wPrevPc,
                                static_cast<unsigned long long>(mac.totalCycles()));
                    wLastVal = v;
                }
            }
            wPrevPc = pc;
        };
    }

    if (traceAdb) {
        mac.onAdbEvent = [&](const char* ev, int state, u32 val) {
            static int n = 0;
            if (n++ >= 4000) return;
            const auto cyc = static_cast<unsigned long long>(mac.totalCycles());
            if (std::strcmp(ev, "state") == 0)
                std::printf("ADB   state=%d%s cyc=%llu\n", state,
                            state == 3 ? " idle" : state == 0 ? " cmd" : "", cyc);
            else if (std::strcmp(ev, "shiftOut") == 0 && state == 0) {
                const int a = (val >> 4) & 0xF, o = (val >> 2) & 3, r = val & 3;
                const char* on = o == 0 ? "reset" : o == 1 ? "flush" : o == 2 ? "listen" : "talk";
                std::printf("ADB > cmd %02X  %d.%s.%d  cyc=%llu\n", val, a, on, r, cyc);
            } else if (std::strcmp(ev, "shiftOut") == 0)
                std::printf("ADB > data %02X (st%d) cyc=%llu\n", val, state, cyc);
            else if (std::strcmp(ev, "shiftIn") == 0)
                std::printf("ADB < %02X (st%d) cyc=%llu\n", val, state, cyc);
            else if (std::strcmp(ev, "arm") == 0)
                std::printf("ADB   arm-%s (st%d) cyc=%llu\n", val ? "in" : "out", state, cyc);
        };
    }

    int excCount = 0;
    mac.cpu().onException = [&](int vector, u32 pc) {
        const bool crash = vector == 2 || vector == 3 || vector == 4 ||
                           vector == 8 || vector == 11;
        if (crash && excCount < 16) {
            std::printf("EXC vec=%d (%s) at pc=%06X  cyc=%llu\n", vector,
                        vector == 2 ? "bus" : vector == 3 ? "addr" :
                        vector == 4 ? "illegal" : vector == 8 ? "priv" : "F-line",
                        pc, static_cast<unsigned long long>(mac.totalCycles()));
            if (vector == 2 || vector == 3 || vector == 11) {   // how we got here
                std::printf("  trail (oldest first):\n   ");
                for (int b = 119; b >= 0; --b) {
                    std::printf(" %06X", mac.cpu().recentPc(b));
                    if (b % 10 == 0) std::printf("\n   ");
                }
                std::printf("\n");
                openmac::dbg::dumpRegs(mac.cpu(), stdout);
                std::string dis;
                openmac::dbg::disasm(mac, pc, dis);
                std::printf("  faulting: %06X  %s\n", pc, dis.c_str());
                openmac::dbg::dumpBacktrace(mac.cpu(), mac, stdout);
                openmac::dbg::checkHeap(mac, stdout);
            }
            ++excCount;
        }
    };

    if (driveInstall) { dcfg.floppy1 = floppyPath; return runDriveInstall(mac, dcfg); }

    if (disasmSet) {
        u32 a = disasmAddr;
        for (int k = 0; k < disasmCount; ++k) {
            std::string dis;
            const int len = openmac::dbg::disasm(mac, a, dis);
            std::printf("  %06X  %-24s %s\n", a, dis.c_str(),
                        openmac::dbg::symbolFor(mac, a).c_str());
            a += len > 0 ? static_cast<u32>(len) : 2;
        }
        return 0;
    }

    if (traceToPc) {
        traceUntil(mac, traceToPc, u64(frames) * Machine::kLinesPerFrame * Machine::kCyclesPerLine);
        return 0;
    }

    if (mouseWalk) {
        for (int i = 0; i < 1900 && !mac.cpu().halted; ++i) mac.runFrame();
        std::string before = dumpPath.empty() ? std::string() : dumpPath + ".before.bmp";
        if (!before.empty()) dumpBmp(mac, before);
        // Drive the mouse down-right for a while, then click.
        for (int i = 0; i < 200 && !mac.cpu().halted; ++i) {
            mac.mouseMove(4, 3, i >= 170);
            mac.runFrame();
        }
        if (!dumpPath.empty()) dumpBmp(mac, dumpPath);
        std::printf("mouse-walk done: halted=%d pc=%06X\n",
                    mac.cpu().halted ? 1 : 0, mac.cpu().pc);
        return 0;
    }

    if (bootNudge) {
        // Reproduce the shell hang: a burst of mouse motion early in boot (while
        // the ROM is still in its boot-time ADB idle-wait), then stop, and check
        // the boot recovers to the desktop instead of wedging at $00BB0A.
        u32 deskBaseReports = 0;
        for (int i = 0; i < frames && !mac.cpu().halted; ++i) {
            if (i >= 200 && i < 340 && (i & 3) == 0) mac.mouseMove(3, 2, false);
            if (i == frames - 200) deskBaseReports = mac.adbStats().mouseReports;
            if (i >= frames - 180 && i < frames - 30 && (i & 3) == 0) mac.mouseMove(4, 3, false);
            mac.runFrame();
            if (i % 400 == 0) {
                const auto s = mac.adbStats();
                std::printf("f=%d pc=%06X kbdPolls=%u mousePolls=%u mouseReports=%u\n",
                            i, mac.cpu().pc, s.kbdPolls, s.mousePolls, s.mouseReports);
            }
        }
        std::printf("DESKTOP mouse: reports in last ~150 frames = %u\n",
                    mac.adbStats().mouseReports - deskBaseReports);
        if (!dumpPath.empty()) dumpBmp(mac, dumpPath);
        // If we wedged in low RAM, single-step the spin loop and dump it so we
        // can see exactly which condition the ROM is stuck waiting on.
        if ((mac.cpu().pc & 0xFFFFFF) < 0x100000) {
            std::printf("-- WEDGE trail (single-stepped from %06X): --\n", mac.cpu().pc);
            for (int k = 0; k < 48 && !mac.cpu().halted; ++k) {
                const u32 pc = mac.cpu().pc;
                std::string dis;
                openmac::dbg::disasm(mac, pc, dis);
                std::printf("  %06X  %-30s D0=%08X D1=%08X A0=%06X A1=%06X A2=%06X\n",
                            pc, dis.c_str(), mac.cpu().d[0], mac.cpu().d[1],
                            mac.cpu().a[0] & 0xFFFFFF, mac.cpu().a[1] & 0xFFFFFF,
                            mac.cpu().a[2] & 0xFFFFFF);
                mac.stepInstruction();
            }
            openmac::dbg::dumpRegs(mac.cpu(), stdout);
            openmac::dbg::dumpBacktrace(mac.cpu(), mac, stdout);
        }
        const auto s = mac.adbStats();
        static std::vector<u32> npix(static_cast<size_t>(Machine::kScreenW) * Machine::kScreenH);
        mac.renderScreen(npix.data());
        long nblack = 0;
        for (u32 p : npix) nblack += (p == 0xFF000000u);
        std::printf("boot-nudge done: pc=%06X kbdPolls=%u mousePolls=%u mouseReports=%u black=%ld\n",
                    mac.cpu().pc, s.kbdPolls, s.mousePolls, s.mouseReports, nblack);
        return 0;
    }

    if (!swapFloppyPath.empty()) {
        // Boot to the INTERACTIVE Finder, then swap a different floppy in and see
        // whether the System mounts it (a new VCB). Without mouse input the boot
        // stalls at a mouse-wait spin ($401606) that never reaches the event loop,
        // so nudge the mouse early (like --boot-nudge) to bring the Finder up, and
        // keep nudging afterwards so it services the disk change.
        auto vcbq = [&](const char* when) {
            auto rd32 = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
            std::printf("VCBQHdr %-7s head=%06X tail=%06X\n", when, rd32(0x358), rd32(0x35C));
        };
        for (int i = 0; i < 2500 && !mac.cpu().halted; ++i) {
            if (i >= 200 && i < 340 && (i & 3) == 0) mac.mouseMove(3, 2, false);
            mac.runFrame();
        }
        vcbq("before");
        std::printf("boot pc=%06X\n", mac.cpu().pc);
        std::ifstream sf(swapFloppyPath, std::ios::binary);
        std::vector<u8> img2{std::istreambuf_iterator<char>(sf), std::istreambuf_iterator<char>()};
        std::printf("swapping in %zu bytes\n", img2.size());
        mac.insertFloppy(std::move(img2), false);
        for (int i = 0; i < 1500 && !mac.cpu().halted; ++i) {
            mac.mouseMove((i & 1) ? 1 : -1, 0, false);   // keep the Finder ticking
            mac.runFrame();
        }
        vcbq("after");
        std::printf("swap: halted=%d pc=%06X\n", mac.cpu().halted ? 1 : 0, mac.cpu().pc);
        return 0;
    }

    if (insert800k) {
        // The P3 milestone. Boot the machine first, THEN insert an 800K GCR
        // volume the way a user would, and see whether the System mounts it
        // through the ROM's own .Sony driver.
        //
        // Inserting before boot is a different test: while the Start Manager is
        // choosing a boot device it ejects a floppy that has no System on it,
        // exactly as a real Mac does, so the disk is gone before the desktop
        // exists. Pair this with --force-rom so the machine has a System to boot
        // from and the floppy is purely a data disk.
        auto vcbq = [&](const char* when) {
            auto rd32 = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
            std::printf("VCBQHdr %-7s head=%06X tail=%06X\n", when, rd32(0x358), rd32(0x35C));
        };
        for (int i = 0; i < 2400 && !mac.cpu().halted; ++i) {
            if (i >= 200 && i < 340 && (i & 3) == 0) mac.mouseMove(3, 2, false);
            mac.runFrame();
        }
        vcbq("before");
        std::printf("boot pc=%06X\n", mac.cpu().pc);
        // Write-protected on purpose. Mounting an HFS volume is not a read-only
        // operation: the System clears the "volume unmounted" attribute and
        // writes the MDB, bitmap and catalog straight back (blocks 2, 3, 16, 17,
        // 28 in this volume). Until the write path exists those writes vanish,
        // the System reads the MDB back unchanged and gives up. A locked disk
        // mounts read-only and needs none of them, which is exactly the isolation
        // the read path wants. --insert-800k-rw drops the lock for the write path.
        std::vector<u8> img;
        if (!floppy800kPath.empty()) {
            std::ifstream f8(floppy800kPath, std::ios::binary);
            img.assign(std::istreambuf_iterator<char>(f8), std::istreambuf_iterator<char>());
            if (img.empty()) {
                std::fprintf(stderr, "cannot read 800K image: %s\n", floppy800kPath.c_str());
                return 2;
            }
        } else {
            img = openmac::hfs::formatVolume(819200u, "OpenMac 800K");
        }
        const std::vector<u8> pristine = img;      // to diff against afterwards
        std::printf("inserting %zu bytes (800K HFS, GCR, %s)\n", img.size(),
                    insert800kRW ? "writable" : "write-protected");
        mac.insertFloppy(std::move(img), !insert800kRW);
        if (!externalPath.empty()) {
            std::ifstream ef(externalPath, std::ios::binary);
            std::vector<u8> eimg{std::istreambuf_iterator<char>(ef),
                                 std::istreambuf_iterator<char>()};
            std::printf("external drive: inserting %zu bytes\n", eimg.size());
            mac.insertExternalFloppy(std::move(eimg), false);
        }
        for (int i = 0; i < 1800 && !mac.cpu().halted; ++i) {
            mac.mouseMove((i & 1) ? 1 : -1, 0, false);
            mac.runFrame();
        }
        vcbq("after");
        // Did the guest's writes actually reach the medium? The System clears the
        // volume-unmounted attribute (MDB @0x0A, logical block 2) the moment it
        // mounts read-write, so that bit is the cheapest end-to-end proof that a
        // nibble the driver wrote came back as a byte in the image.
        {
            const std::vector<u8>& now = mac.floppyImage();
            std::size_t changed = 0, firstBlk = 0;
            bool haveFirst = false;
            if (now.size() == pristine.size()) {
                for (std::size_t i = 0; i < now.size(); ++i)
                    if (now[i] != pristine[i]) {
                        ++changed;
                        if (!haveFirst) { firstBlk = i / 512; haveFirst = true; }
                    }
                const u16 atrbWas = static_cast<u16>((pristine[2 * 512 + 0x0A] << 8) |
                                                     pristine[2 * 512 + 0x0B]);
                const u16 atrbNow = static_cast<u16>((now[2 * 512 + 0x0A] << 8) |
                                                     now[2 * 512 + 0x0B]);
                std::printf("image: %zu bytes changed (first in block %zu); "
                            "MDB drAtrb %04X -> %04X\n",
                            changed, haveFirst ? firstBlk : 0, atrbWas, atrbNow);
            } else {
                std::printf("image: gone (%zu bytes)\n", now.size());
            }
        }
        std::printf("surface: %u bytes written, %u tracks decoded back\n",
                    mac.iwmDataWrites(), mac.floppyTrackWrites());
        if (!insert800kOut.empty()) {
            const std::vector<u8>& out = mac.floppyImage();
            std::ofstream of(insert800kOut, std::ios::binary);
            of.write(reinterpret_cast<const char*>(out.data()),
                     static_cast<std::streamsize>(out.size()));
            std::printf("wrote %zu bytes to %s\n", out.size(), insert800kOut.c_str());
        }
        std::printf("data register: %u reads, %u disk bytes delivered\n",
                    mac.iwmDataReads(), mac.iwmDataBytes());
        unsigned gcrTotal = 0;
        for (int c = 0xAC; c <= 0xBF; ++c) gcrTotal += mac.gcrErrorCount(static_cast<u8>(c));
        std::printf("GCR read failures reported by the ROM: %u\n", gcrTotal);
        for (int c = 0xAC; c <= 0xBF; ++c) {
            const u32 n = mac.gcrErrorCount(static_cast<u8>(c));
            if (n) std::printf("   %02X %-52s %u\n", c,
                               Machine::gcrErrorName(static_cast<u8>(c)), n);
        }
        std::printf("insert-800k: halted=%d pc=%06X\n", mac.cpu().halted ? 1 : 0, mac.cpu().pc);
        return 0;
    }

    if (keyTest) {
        // Boot, then inject a key AFTER the desktop is up and watch the ROM's
        // KeyMap ($174) -- i.e. does keyboard input register post-boot?
        for (int i = 0; i < 1900 && !mac.cpu().halted; ++i) mac.runFrame();
        auto keymap = [&](const char* when) {
            std::printf("KeyMap %-7s", when);
            for (u32 a = 0x174; a < 0x17C; ++a) std::printf(" %02X", mac.read8(a));
            std::printf("\n");
        };
        const auto s0 = mac.adbStats();
        keymap("before");
        mac.keyEvent(0x00, true);                       // 'A' down (ADB 0x00)
        for (int i = 0; i < 60 && !mac.cpu().halted; ++i) mac.runFrame();
        keymap("A-down");
        mac.keyEvent(0x00, false);                      // 'A' up
        for (int i = 0; i < 60 && !mac.cpu().halted; ++i) mac.runFrame();
        keymap("A-up");
        const auto s1 = mac.adbStats();
        std::printf("key-test: kbdEnum=%u kbdPolls %u->%u modifiers %u->%u\n",
                    s1.kbdReg3, s0.kbdPolls, s1.kbdPolls, s0.kbdReg2, s1.kbdReg2);
        return 0;
    }

    if (watchAddr != 0xFFFFFFFFu) {
        auto rd32 = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
        const u64 maxCycles = u64(frames) * Machine::kLinesPerFrame * Machine::kCyclesPerLine;
        u32 last = rd32(watchAddr);
        std::printf("watch [%06X] initial=%08X\n", watchAddr, last);
        int hits = 0;
        while (mac.totalCycles() < maxCycles && !mac.cpu().halted && hits < 60) {
            const u32 pc = mac.cpu().pc;
            mac.stepInstruction();
            const u32 now = rd32(watchAddr);
            if (now != last) {
                std::printf("[%06X] %08X -> %08X  by pc=%06X\n", watchAddr, last, now, pc);
                last = now;
                ++hits;
            }
        }
        std::printf("halted=%d final=%08X\n", mac.cpu().halted ? 1 : 0, last);
        return 0;
    }

    if (tracePc) {
        const u64 maxCycles = u64(frames) * Machine::kLinesPerFrame * Machine::kCyclesPerLine;
        bool tracing = false;
        int traced = 0;
        while (mac.totalCycles() < maxCycles && !mac.cpu().halted && traced < traceCount) {
            const u32 pc = mac.cpu().pc;
            if (pc == tracePc) tracing = true;
            if (tracing) {
                std::string dis;
                openmac::dbg::disasm(mac, pc, dis);
                std::printf("%06X  %-30s D0=%08X D1=%08X A0=%06X A1=%06X\n", pc,
                            dis.c_str(), mac.cpu().d[0], mac.cpu().d[1],
                            mac.cpu().a[0], mac.cpu().a[1]);
                ++traced;
            }
            mac.stepInstruction();
        }
        std::printf("\n");
        openmac::dbg::dumpRegs(mac.cpu(), stdout);
        return 0;
    }

    if (stepOverSet || stepOutSet) {
        const u64 maxCycles = u64(frames) * Machine::kLinesPerFrame * Machine::kCyclesPerLine;
        if (stepOverSet) stepOver(mac, stepOverPc, maxCycles);
        if (stepOutSet)  stepOut(mac, stepOutPc, maxCycles);
        return 0;
    }

    if (traceOsTraps) {
        // Log Memory Manager allocation traps with their results; a NIL result
        // (A0 = 0) is the classic origin of a later NIL-dereference crash.
        struct Pend { u32 ret; u16 trap; };
        std::vector<Pend> pend;
        mac.cpu().onTrap = [&](u16 trap, u32 pc) {
            if ((trap & 0xF800) != 0xA000) return;          // OS traps only
            if (openmac::dbg::trapReturnsPtrInA0(trap)) {
                std::printf("-> %-14s size=%-8d @%06X\n", openmac::dbg::trapName(trap),
                            mac.cpu().d[0], pc);
                pend.push_back({pc + 2, trap});
            } else if ((mac.cpu().a[0] & 0xFFFFFF) == 0) {  // NIL pointer argument
                std::printf("!! %-14s A0=NIL @%06X\n", openmac::dbg::trapName(trap), pc);
            }
        };
        const u64 maxCycles = u64(frames) * Machine::kLinesPerFrame * Machine::kCyclesPerLine;
        while (mac.totalCycles() < maxCycles && !mac.cpu().halted) {
            const u32 pc = mac.cpu().pc;
            for (size_t i = pend.size(); i-- > 0;) {
                if (pend[i].ret == pc) {
                    const u32 a0 = mac.cpu().a[0] & 0xFFFFFF;
                    std::printf("<- %-14s A0=%06X%s\n", openmac::dbg::trapName(pend[i].trap),
                                a0, a0 == 0 ? "   *** NIL (allocation failed) ***" : "");
                    pend.erase(pend.begin() + static_cast<long>(i));
                    break;
                }
            }
            mac.stepInstruction();
        }
        std::printf("\n");
        openmac::dbg::checkHeap(mac, stdout);
        return 0;
    }

    if (traceDevIo) {
        // Device Manager I/O (_Open/_Close/_Read/_Write/_Control/_Status/_KillIO) to our
        // SCSI hard disk (refNum -2 or drive 4): log entry PB and the ioResult once the
        // trap returns, so a mount's read conversation and its result codes are visible.
        struct Pend { u32 ret, pb; u16 trap; };
        std::vector<Pend> pend;
        int trc = 0;   // when >0, dump the next N instructions (set when our read fires)
        mac.cpu().onTrap = [&](u16 trap, u32 pc) {
            if ((trap & 0x0800) != 0) return;               // OS traps only (not Toolbox)
            const u16 t = trap & 0xF0FF;                     // fold async/immediate bits
            if (t < 0xA000 || (t & 0x00FF) > 0x0006) return; // Open..KillIO
            const u32 a0 = mac.cpu().a[0];
            const int16_t refNum = int16_t(mac.read16(a0 + 0x18));
            const int16_t drive  = int16_t(mac.read16(a0 + 0x16));
            if (refNum != -2 && drive != 4) return;
            std::string s;
            openmac::dbg::describeIOTrap(mac, trap, pc, a0, s);
            std::printf("IO-> %s\n", s.c_str());
            pend.push_back({pc + 2, a0, trap});
            if (trc == 0 && (trap & 1)) trc = 4000;  // trace the first WRITE path (A003) once
        };
        // Match the return PC per instruction to read the completed ioResult. The mount
        // is triggered inside runFrame(), so we must drive whole frames (not step raw).
        mac.cpu().onStep = [&](u32 pc) {
            for (size_t i = pend.size(); i-- > 0;) {
                if (pend[i].ret == pc) {
                    const int16_t r = int16_t(mac.read16(pend[i].pb + 0x10));
                    std::printf("IO<- trap=%04X ioResult=%d\n", pend[i].trap, r);
                    pend.erase(pend.begin() + static_cast<long>(i));
                    break;
                }
            }
            if (trc > 0) {
                std::string d;
                openmac::dbg::disasm(mac, pc, d);
                std::printf("  %06X D0=%08X A0=%06X A1=%06X %s\n", pc, mac.cpu().d[0],
                            mac.cpu().a[0] & 0xFFFFFF, mac.cpu().a[1] & 0xFFFFFF, d.c_str());
                --trc;
            }
        };
        for (int i = 0; i < frames && !mac.cpu().halted; ++i) mac.runFrame();
        std::printf("\n");
        return 0;
    }

    bool comboDown = false;
    if (bootDisk) {
        mac.keyEvent(0x37, true); mac.keyEvent(0x3A, true);   // Command, Option
        mac.keyEvent(0x07, true); mac.keyEvent(0x1F, true);   // X, O
        comboDown = true;
    }

    std::vector<u8> allAudio;
    std::vector<u8> tmpAudio;
    for (int i = 0; i < frames; ++i) {
        // Put a disk into the external drive mid-run, the way a user does when an
        // installer asks for the next one. A disk already in the drive at power-on
        // is a different case: the ROM reads the disk-switched line during its own
        // boot-time probe, which clears it, and the System that would have acted on
        // it does not exist yet -- so it sits there unmounted.
        if (!insertExternalPath.empty() && i == insertExternalAt) {
            std::ifstream ef(insertExternalPath, std::ios::binary);
            std::vector<u8> eimg{std::istreambuf_iterator<char>(ef),
                                 std::istreambuf_iterator<char>()};
            std::printf("-- frame %d: inserting %zu bytes into the external drive --\n",
                        i, eimg.size());
            mac.insertExternalFloppy(std::move(eimg), false);
        }
        // The Finder only notices a disk while it is running its event loop, and an
        // idle machine gets no events at all, so keep the mouse alive afterwards.
        if (!insertExternalPath.empty() && i > insertExternalAt)
            mac.mouseMove((i & 1) ? 1 : -1, 0, false);
        if (i == profileAt) profileFrame(mac);
        mac.runFrame();
        mac.drainAudio(tmpAudio);
        allAudio.insert(allAudio.end(), tmpAudio.begin(), tmpAudio.end());
        // Hold Cmd-Opt-X-O until the ROM disk path is armed ($0CB3 latched to
        // $0B, i.e. past the RAM test -- $0210 holds RAM-test garbage before
        // that) AND a boot device has been chosen (BootDrive $0210 leaves the
        // $FFFF the ROM parks there during the search). Then release: the ROM has
        // latched the combo, and holding it into Finder load reads as "rebuild
        // desktop".
        if (comboDown && mac.read8(0x0CB3) == 0x0B && mac.read16(0x0210) != 0xFFFF) {
            mac.keyEvent(0x37, false); mac.keyEvent(0x3A, false);
            mac.keyEvent(0x07, false); mac.keyEvent(0x1F, false);
            comboDown = false;
        }
        const auto& cpu = mac.cpu();

        // Screen activity: count black pixels in the visible buffer.
        static std::vector<u32> pix(static_cast<size_t>(Machine::kScreenW) * Machine::kScreenH);
        mac.renderScreen(pix.data());
        long black = 0;
        for (u32 p : pix) black += (p == 0xFF000000u);

        if (i < 10 || i % 10 == 9 || cpu.halted) {
            std::printf("frame %3d  pc=%06X sr=%04X overlay=%d black=%ld%s%s\n",
                        i + 1, cpu.pc, cpu.getSR(), mac.overlayActive() ? 1 : 0, black,
                        cpu.stopped ? " STOPPED" : "", cpu.halted ? " HALTED" : "");
        }
        if (cpu.halted) break;
    }

    auto rd32 = [&](u32 a) {
        return (u32(mac.read16(a)) << 16) | mac.read16(a + 2);
    };
    std::printf("globals: MemTop=%08X BufPtr=%08X ScrnBase=%08X SoundBase=%08X\n",
                rd32(0x108), rd32(0x10C), rd32(0x824), rd32(0x266));

    if (lowmemDump) {
        openmac::dbg::dumpLowMem(mac, stdout);
        openmac::dbg::dumpDriveQueue(mac, stdout);
        openmac::dbg::dumpUnitTable(mac, stdout);
        openmac::dbg::dumpTimerQueue(mac, stdout);
        openmac::dbg::dumpVia(mac, stdout);
    }
    if (dumpMemLen) {
        std::printf("-- memory $%06X (%u bytes) --\n", dumpMemAddr, dumpMemLen);
        openmac::dbg::dumpMem(mac, dumpMemAddr, dumpMemLen, stdout);
    }
    if (dumpStructSet)
        openmac::dbg::dumpStruct(mac, dumpStructAddr, dumpStructName.c_str(), stdout);
    if (checkHeapFlag) openmac::dbg::checkHeap(mac, stdout);

    if (!dumpPath.empty()) dumpBmp(mac, dumpPath);

    {
        int lo = 255, hi = 0; long nonSilent = 0;
        for (u8 v : allAudio) { if (v < lo) lo = v; if (v > hi) hi = v; if (v < 0x7C || v > 0x84) ++nonSilent; }
        std::printf("\n-- audio: %zu samples, min=%d max=%d non-silent=%ld --\n",
                    allAudio.size(), lo, hi, nonSilent);
        if (!allAudio.empty())
            writeWav("I:/Visual Studio Projects/scratch/openmac/shots/boot.wav", allAudio, 22254);
    }

    if (iwmTotal > 0) {
        std::printf("\n-- IWM/SWIM: %lld accesses; per soft-switch: --\n", iwmTotal);
        for (int i = 0; i < 16; ++i)
            if (iwmRegHits[i]) std::printf("   %-11s %u\n", kIwmReg[i], iwmRegHits[i]);
        std::printf("   drive status lines sampled (CA2:CA1:CA0:SEL):\n");
        for (int i = 0; i < 16; ++i)
            if (driveRegReads[i]) std::printf("     %X %-14s %u\n", i, kDriveReg[i], driveRegReads[i]);
        static const char* kCmdName[8] = {"DIRTN", "?1", "STEP", "?3",
                                          "MOTORON", "?5", "EJECT", "?7"};
        std::printf("   drive commands issued (LSTRB-latched):\n");
        for (int d = 0; d < 2; ++d)
            for (int r = 0; r < 8; ++r)
                if (mac.sonyCommandCount(d, r))
                    std::printf("     drive%d %-8s %u\n", d + 1, kCmdName[r],
                                mac.sonyCommandCount(d, r));
        std::printf("   data register: %u reads, %u disk bytes delivered\n",
                    mac.iwmDataReads(), mac.iwmDataBytes());
        std::printf("   accessing PCs (%zu distinct):", iwmPcs.size());
        for (u32 p : iwmPcs) std::printf(" %06X", p);
        std::printf("\n");
    }

    // What the ROM's own GCR reader made of the nibble stream we handed it.
    {
        unsigned gcrTotal = 0;
        for (int c = 0xB8; c <= 0xBF; ++c) gcrTotal += mac.gcrErrorCount(static_cast<u8>(c));
        if (gcrTotal) {
            std::printf("\n-- GCR read failures reported by the ROM (%u): --\n", gcrTotal);
            for (int c = 0xB8; c <= 0xBF; ++c) {
                const u32 n = mac.gcrErrorCount(static_cast<u8>(c));
                if (n)
                    std::printf("   %02X %-52s %u\n", c,
                                Machine::gcrErrorName(static_cast<u8>(c)), n);
            }
        }
    }

    std::printf("\n-- KeyMap ($174) reads: %u  from PCs:", mac.keyMapReads());
    for (int i = 0; i < mac.keyMapPcCount(); ++i) std::printf(" %06X", mac.keyMapPc(i));
    std::printf(" --\n");
    const auto s = mac.adbStats();
    std::printf("-- ADB: kbd[enum=%u modifiers=%u transitions=%u] "
                "mouse[enum=%u polls=%u reports=%u] --\n",
                s.kbdReg3, s.kbdReg2, s.kbdPolls, s.mouseReg3, s.mousePolls, s.mouseReports);
    std::printf("-- SURFACE READS: internal=%u external=%u phantom-bay=%u --\n",
                mac.surfaceReads(0), mac.surfaceReads(1), mac.surfaceReads(2));
    std::printf("-- ADDRESSED: internal=%u external=%u --\n",
                mac.surfaceReads(3), mac.surfaceReads(4));
    if (!floppyOutPath.empty()) {
        const std::vector<u8>& out = mac.floppyImage();
        std::ofstream of(floppyOutPath, std::ios::binary);
        of.write(reinterpret_cast<const char*>(out.data()),
                 static_cast<std::streamsize>(out.size()));
        std::printf("-- wrote the internal drive's medium (%zu bytes) to %s --\n",
                    out.size(), floppyOutPath.c_str());
    }
    if (mac.hardDiskPresent())
        std::printf("-- HARD DISK: %zu bytes, drive#=%d, block accesses=%u, "
                    "mount attempts=%u last result=%04X --\n",
                    mac.hardDiskImage().size(), mac.hardDiskDriveNum(),
                    mac.hdAccessCount(), mac.diskEvtPosts(), mac.diskEvtResult());
    // What the file system believes it has. A drive that registered but whose
    // volume never mounted, and a volume that mounted against the wrong drive,
    // look identical from outside; these two queues tell them apart. The drive
    // queue ($0308) is what _AddDrive built; the volume queue ($0356) is what
    // _MountVol built, and a disk is only usable when it appears in both.
    {
        auto qWord = [&](u32 a) { return mac.read16(a); };
        auto qLong = [&](u32 a) { return (u32(mac.read16(a)) << 16) | mac.read16(a + 2); };
        auto rd8 = [&](u32 a) {
            const u16 w = mac.read16(a & ~1u);
            return static_cast<u8>((a & 1) ? (w & 0xFF) : (w >> 8));
        };
        std::printf("-- DRIVE QUEUE ($0308): --\n");
        int n = 0;
        for (u32 e = qLong(0x030A); e && n < 8; e = qLong(e), ++n) {
            const u32 blocks = (static_cast<u32>(qWord(e + 14)) << 16) | qWord(e + 12);
            std::printf("   drive %2d  refNum %6d  FSID %04X  %u blocks (%u KB)\n",
                        static_cast<std::int16_t>(qWord(e + 6)), static_cast<std::int16_t>(qWord(e + 8)),
                        qWord(e + 10), blocks, blocks / 2);
        }
        if (n == 0) std::printf("   (empty -- no drive ever registered)\n");

        std::printf("-- VOLUME QUEUE ($0356): --\n");
        n = 0;
        for (u32 v = qLong(0x0358); v && n < 8; v = qLong(v), ++n) {
            char nm[28] = {0};
            const u8 len = rd8(v + 0x2C);          // vcbVN, a Pascal string
            for (u8 i = 0; i < len && i < 27; ++i) nm[i] = static_cast<char>(rd8(v + 0x2D + i));
            const u32 alBlks = qWord(v + 26), alSize = qLong(v + 28), free = qWord(v + 42);
            std::printf("   \"%s\"  drive %d  refNum %d  vRefNum %d  sig %04X  "
                        "%u KB free of %u KB\n",
                        nm, static_cast<std::int16_t>(qWord(v + 72)), static_cast<std::int16_t>(qWord(v + 74)),
                        static_cast<std::int16_t>(qWord(v + 78)), qWord(v + 8),
                        static_cast<u32>((static_cast<u64>(free) * alSize) >> 10),
                        static_cast<u32>((static_cast<u64>(alBlks) * alSize) >> 10));
        }
        if (n == 0) std::printf("   (empty -- nothing is mounted)\n");
    }
    {
        const auto sc = mac.scsiStats();
        std::printf("-- SCSI: reads=%u writes=%u selects=%u commands=%u dataIn=%u dataOut=%u lastCDB=",
                    sc.reads, sc.writes, sc.selects, sc.commands, sc.dataInBytes, sc.dataOutBytes);
        for (int i = 0; i < sc.lastCdbLen; ++i) std::printf("%02X ", sc.lastCdb[i]);
        std::printf("--\n");
        u16 wt[320];
        const int wn = mac.scsiWriteTrace(wt, 320);
        std::printf("-- SCSI writes (reg=val), first %d: --\n", wn);
        static const char* rn[8] = {"ODR","ICR","MR ","TCR","SER","d5 ","d6 ","d7 "};
        for (int i = 0; i < wn; ++i) {
            std::printf(" %s=%02X", rn[(wt[i] >> 8) & 7], wt[i] & 0xFF);
            if ((i + 1) % 8 == 0) std::printf("\n");
        }
        std::printf("\n");
        u8 cd[16 * 12];
        const int cn = mac.scsiCdbHist(cd, 16);
        std::printf("-- SCSI CDB history (%d): --\n", cn);
        for (int i = 0; i < cn; ++i) {
            std::printf("  ");
            for (int j = 0; j < 12; ++j) std::printf("%02X ", cd[i * 12 + j]);
            std::printf("\n");
        }
    }
    std::printf("-- ADB command trace (addr.op.reg), first %zu: --\n",
                mac.adbCmdTrace().size());
    const auto& tr = mac.adbCmdTrace();
    const auto& rs = mac.adbRespTrace();
    for (size_t i = 0; i < tr.size(); ++i) {
        const u8 c = tr[i];
        const char* op = ((c >> 2) & 3) == 0 ? "rst" : ((c >> 2) & 3) == 1 ? "flu"
                       : ((c >> 2) & 3) == 2 ? "LSN" : "TLK";
        std::printf(" %d.%s.%d%s", (c >> 4) & 0xF, op, c & 3,
                    (i < rs.size() && rs[i]) ? "+" : " ");
        if ((i + 1) % 8 == 0) std::printf("\n");
    }
    std::printf("\n");

    std::printf("-- access log (%zu entries) --\n", mac.accessLog().size());
    int shown = 0;
    for (const auto& line : mac.accessLog()) {
        std::printf("%s\n", line.c_str());
        if (++shown >= 60) { std::printf("...\n"); break; }
    }
    return 0;
}
