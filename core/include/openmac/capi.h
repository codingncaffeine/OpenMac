// C ABI over the OpenMac core, for the .NET GUI (P/Invoke) and any other
// non-C++ front-end. Everything is plain C: an opaque machine handle plus flat
// functions. The debugger is first-class here — a log callback and enable flags
// let a front-end turn on the same monitor output the headless trace tool emits
// (trap trace, exception dumps + backtrace, IRQ log) and read it as text.

#ifndef OPENMAC_CAPI_H
#define OPENMAC_CAPI_H

#include <stddef.h>
#include <stdint.h>

#ifdef _WIN32
#  define OMAC_API __declspec(dllexport)
#else
#  define OMAC_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct OMac OMac;

/* Framebuffer geometry (constant for the Classic). */
#define OMAC_SCREEN_W 512
#define OMAC_SCREEN_H 342

/* ---- lifecycle ---- */
OMAC_API OMac* omac_create(const uint8_t* rom, size_t rom_len, uint32_t ram_mb);
OMAC_API void  omac_destroy(OMac*);
OMAC_API void  omac_reset(OMac*);

/* ---- run / video ---- */
OMAC_API void omac_run_frame(OMac*);
/* Fill argb with OMAC_SCREEN_W*OMAC_SCREEN_H pixels (0xAARRGGBB). */
OMAC_API void omac_render(OMac*, uint32_t* argb);

/* ---- audio ---- */
/* Drain pending sound samples (8-bit unsigned PCM, mono, ~22254 Hz) into out;
   returns the count written (<= cap). Poll once per frame after omac_run_frame.
   The ROM synthesizes its own boot chime through this path -- no bundled audio. */
OMAC_API size_t omac_drain_audio(OMac*, uint8_t* out, size_t cap);

/* ---- disks ---- */
OMAC_API void omac_insert_floppy(OMac*, const uint8_t* img, size_t len, int read_only);
OMAC_API void omac_eject_floppy(OMac*);
OMAC_API void omac_insert_harddisk(OMac*, const uint8_t* img, size_t len, int read_only);

/* Format a blank HFS volume of size_bytes into out (which must hold size_bytes).
   Returns 0 on success. Front-ends use this for "Create Hard Disk". */
OMAC_API int omac_format_hfs(uint32_t size_bytes, const char* volume_name, uint8_t* out);

/* ---- input ---- */
OMAC_API void omac_mouse(OMac*, int dx, int dy, int button);
OMAC_API void omac_key(OMac*, int adb_code, int down);

/* ---- debugger / monitor ---- */

/* Registers snapshot for a live panel. */
typedef struct {
    uint32_t d[8];
    uint32_t a[8];
    uint32_t pc;
    uint16_t sr;
    uint64_t cycles;
} OMacRegs;
OMAC_API void omac_regs(OMac*, OMacRegs* out);

/* Sink for monitor text lines. The core calls it (with the user pointer) for
   each event enabled below; a front-end writes them to its log / debug panel. */
typedef void (*OMacLogFn)(void* user, const char* line);
OMAC_API void omac_set_log(OMac*, OMacLogFn fn, void* user);

/* Toggle what the monitor emits to the log sink. Mirrors the trace tool flags. */
#define OMAC_DBG_TRAPS   0x01   /* A-line Toolbox/OS trap trace          */
#define OMAC_DBG_EXCEPT  0x02   /* bus/address/illegal faults + backtrace */
#define OMAC_DBG_IRQ     0x04   /* interrupt log with VIA source          */
#define OMAC_DBG_ADB     0x08   /* decoded ADB bus trace                  */
OMAC_API void omac_debug_enable(OMac*, uint32_t flags);

/* Render a named monitor view as text into out (NUL-terminated, capped at cap).
   name: "regs" | "backtrace" | "lowmem" | "via" | "drives" | "heap" | "disasm". */
OMAC_API void omac_debug_dump(OMac*, const char* name, char* out, size_t cap);

/* Drain buffered monitor lines (newline-separated) into out, then clear the
   buffer. Poll this from the front-end's frame loop; unlike the callback it runs
   off the CPU exception path, so it can't destabilize emulation. */
OMAC_API void omac_poll_log(OMac*, char* out, size_t cap);

/* Version string for the About box / logs. */
OMAC_API const char* omac_version(void);

#ifdef __cplusplus
}
#endif

#endif /* OPENMAC_CAPI_H */
