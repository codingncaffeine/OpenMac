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

/* Force the built-in ROM disk (System 6.0.3 from ROM -- the Cmd-Opt-X-O boot)
   as the boot device. Set before the first omac_run_frame. */
OMAC_API void  omac_set_force_rom_disk(OMac*, int on);

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
/* Put a disk image in a drive. Raw 400K/800K/1.4MB dumps mount as-is; DiskCopy
   4.2 and MacBinary wrappers (or the two nested) are stripped on the way in and
   faithfully reassembled around the guest's writes on the way out. Returns 1 if
   the drive took the disk, 0 if the file is not floppy media (an NDIF image, an
   application, an archive...) -- the drive is then left untouched, and
   omac_floppy_medium says why, in words meant for the person who chose it. */
OMAC_API int omac_insert_floppy(OMac*, const uint8_t* img, size_t len, int read_only);
OMAC_API void omac_eject_floppy(OMac*);

/* The last medium description for a drive (0 = internal, 1 = external):
   geometry and container for an accepted disk, the reason for a refusal.
   Copies at most cap-1 bytes plus a terminator; returns the full length. */
OMAC_API size_t omac_floppy_medium(OMac*, int drive, char* out, size_t cap);

/* The external drive port. Attaching a mechanism makes the ROM register a
   second floppy drive; a disk put in after the machine has started mounts like
   any other. omac_floppy_data copies the medium out so writes can be persisted
   -- pass a null buffer to ask how large it is. */
OMAC_API void omac_set_external_drive(OMac*, int attached);
OMAC_API int omac_insert_floppy2(OMac*, const uint8_t* img, size_t len, int read_only);
OMAC_API void omac_eject_floppy2(OMac*);

/* Is there a disk in that drive right now? 0 = internal, 1 = external. The guest
   ejects disks on its own -- the startup scan drops a non-bootable one, an
   installer swaps between them, the Finder obeys a drag to the Trash -- so a
   front end has to ask rather than assume its own last action still holds. */
OMAC_API int omac_floppy_present(OMac*, int drive);
OMAC_API size_t omac_floppy_data(OMac*, uint8_t* out, size_t cap);
OMAC_API size_t omac_floppy2_data(OMac*, uint8_t* out, size_t cap);
OMAC_API void omac_insert_harddisk(OMac*, const uint8_t* img, size_t len, int read_only);

/* Copy the live hard-disk image (including guest writes) into out; returns the
   byte count, or the full size when out is NULL. Front-ends use this to persist
   the disk back to its file on eject/exit. */
OMAC_API size_t omac_harddisk_data(OMac*, uint8_t* out, size_t cap);

/* ---- second SCSI disk (the folder disk's seat; SCSI ID 1, drive 5) ---- */
OMAC_API void omac_insert_harddisk2(OMac*, const uint8_t* img, size_t len, int read_only);
OMAC_API void omac_detach_harddisk2(OMac*);
OMAC_API size_t omac_harddisk2_data(OMac*, uint8_t* out, size_t cap);

/* ---- HFS folder-volume builder / reader (host-side, no machine needed) ----
   Builder: begin -> add_dir/add_file (parent 2 = the root) -> build (NULL out
   queries the size; the volume is built once and cached) -> free. Names are
   canonicalized to a collation-safe ASCII subset; dates are HFS-epoch seconds
   (0 = a fixed valid date). Reader: open an image (returns NULL if it is not
   a mountable HFS volume), enumerate items, read forks, free. */
OMAC_API void* omac_hfsb_begin(const char* volume_name);
OMAC_API uint32_t omac_hfsb_add_dir(void* b, uint32_t parent, const char* name,
                                    uint32_t cr_date, uint32_t md_date);
OMAC_API void omac_hfsb_add_file(void* b, uint32_t parent, const char* name,
                                 uint32_t type, uint32_t creator, uint16_t fd_flags,
                                 const uint8_t* data, size_t data_len,
                                 const uint8_t* rsrc, size_t rsrc_len,
                                 uint32_t cr_date, uint32_t md_date);
OMAC_API size_t omac_hfsb_build(void* b, uint8_t* out, size_t cap);
OMAC_API size_t omac_hfsb_error(void* b, char* out, size_t cap);
OMAC_API void omac_hfsb_free(void* b);

typedef struct {
    uint32_t id, parent;
    int32_t is_dir;
    uint32_t type, creator;
    uint32_t fd_flags;
    uint32_t cr_date, md_date;
    uint32_t data_len, rsrc_len;
    char name[64];
} OMacHfsItem;
OMAC_API void* omac_hfsr_open(const uint8_t* img, size_t len);
OMAC_API int32_t omac_hfsr_count(void* r);
OMAC_API int32_t omac_hfsr_item(void* r, int32_t index, OMacHfsItem* out);
OMAC_API size_t omac_hfsr_fork(void* r, uint32_t file_id, int32_t rsrc,
                               uint8_t* out, size_t cap);
OMAC_API void omac_hfsr_free(void* r);

/* ---- CD-ROM (an AppleCD SC-class SCSI drive) ---- */
/* The drive itself is a bus device: attach it once (scsi_id 3 is Apple's
   factory default) and it persists across discs. A disc image goes in with
   omac_cd_insert -- .iso, raw 2352-byte MODE1 (.bin), Apple-partitioned or
   bare-HFS masters -- always read-only; nothing is ever copied back out.
   Returns 1 if the drive took it, 0 if refused; omac_cd_medium describes the
   disc (or the refusal) either way. The guest ejects discs on its own (drag to
   the Trash), so poll omac_cd_present rather than trusting the last insert. */
OMAC_API void omac_cd_attach(OMac*, int attached, int scsi_id);
OMAC_API int  omac_cd_attached(OMac*);
OMAC_API int  omac_cd_insert(OMac*, const uint8_t* img, size_t len);
OMAC_API void omac_cd_eject(OMac*);
OMAC_API int  omac_cd_present(OMac*);
OMAC_API size_t omac_cd_medium(OMac*, char* out, size_t cap);

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
