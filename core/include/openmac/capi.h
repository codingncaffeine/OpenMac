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
/* 1 when a second disk was on the bus during the boot scan (its driver is
   resident, so a mid-session attach mounts live; otherwise offer a restart). */
OMAC_API int omac_harddisk2_booted(OMac*);

/* Flush and unmount ONLY the second disk's volume, leaving the boot disk alone.
   The first half of a drop-box republish: the guest's cached blocks land in the
   image and it forgets its catalog, so the host may rebuild the volume and put
   it back. Returns non-zero once the volume is off line. */
OMAC_API int omac_unmount_harddisk2(OMac*);

/* Push out every block the File Manager is still holding. Call this before
   persisting a disk image to a file: the driver serves writes synchronously, so
   the image is current for everything the System has ISSUED, but not for what
   it is still caching -- and an image saved without it was consistent at no
   point in time. Returns non-zero if a volume was flushed. */
OMAC_API int omac_flush_volumes(OMac*);

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
/* Build at an exact size (e.g. 1474560 = a 1.44 MB floppy), packed tight.
   The volume must fit or the build fails with omac_hfsb_error set. */
OMAC_API size_t omac_hfsb_build_sized(void* b, uint32_t size_bytes, uint8_t* out, size_t cap);
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

/* ---- networking (a DaynaPORT SCSI/Link Ethernet adapter, SCSI ID 4) ----
   The device moves raw Ethernet frames; the front end owns the backend (its
   user-mode NAT). Inject queues a host frame toward the guest (returns 1, or
   0 when the ring is full / the device is detached). Drain copies one guest
   frame out and returns its length (0 = nothing waiting); poll it per frame
   like the audio. */
OMAC_API void omac_net_attach(OMac*, int attached, int scsi_id);
OMAC_API int  omac_net_attached(OMac*);
OMAC_API int  omac_net_inject(OMac*, const uint8_t* frame, size_t len);
OMAC_API size_t omac_net_drain(OMac*, uint8_t* out, size_t cap);

/* ---- Quadra 650 ---------------------------------------------------------
   Additive surface with its own opaque handle: 68040 board, color DAFB
   video whose geometry follows what the ROM programs (ask, don't assume),
   EASC audio on the same 8-bit 22.25 kHz drain contract, and a SCSI hard
   disk that takes the same raw HFS images the Classic does. */
typedef struct OMacQ OMacQ;
OMAC_API OMacQ* omac_q_create(const uint8_t* rom, size_t rom_len, uint32_t ram_mb);
OMAC_API void   omac_q_destroy(OMacQ*);
OMAC_API void   omac_q_reset(OMacQ*);
OMAC_API void   omac_q_run_frame(OMacQ*);
OMAC_API int    omac_q_screen_w(OMacQ*);
OMAC_API int    omac_q_screen_h(OMacQ*);
/* Fill argb with omac_q_screen_w * omac_q_screen_h pixels (0xAARRGGBB). */
OMAC_API void   omac_q_render(OMacQ*, uint32_t* argb);
OMAC_API size_t omac_q_drain_audio(OMacQ*, uint8_t* out, size_t cap);
OMAC_API void   omac_q_mouse(OMacQ*, int dx, int dy, int button);
OMAC_API void   omac_q_key(OMacQ*, int adb_code, int down);
OMAC_API void   omac_q_insert_harddisk(OMacQ*, const uint8_t* img, size_t len, int read_only);
OMAC_API size_t omac_q_harddisk_data(OMacQ*, uint8_t* out, size_t cap);

/* The second SCSI disk (ID 1, drive 5) -- the seat the drop box and the folder
   disk ride on. Same shape as the Classic's omac_harddisk2_*.
   _booted reports whether the System has installed that seat's driver: until
   the startup bus scan has run one, a volume put here cannot mount because
   there is nothing to read it with.
   _unmount flushes and unmounts ONLY this volume, leaving the boot disk alone.
   That is the first half of a republish: the guest's cached blocks land in the
   image and it forgets its catalog, so the host may rebuild the volume and put
   it back. Returns non-zero once the volume is off line. */
OMAC_API void   omac_q_insert_harddisk2(OMacQ*, const uint8_t* img, size_t len, int read_only);
OMAC_API void   omac_q_detach_harddisk2(OMacQ*);
OMAC_API size_t omac_q_harddisk2_data(OMacQ*, uint8_t* out, size_t cap);
OMAC_API int    omac_q_harddisk2_booted(OMacQ*);
OMAC_API int    omac_q_unmount_harddisk2(OMacQ*);

/* A text snapshot of CPU, low memory and every device, for capturing what a
   wedged guest was doing. Pass out=NULL to learn the size needed. Reads model
   state only, so taking it cannot disturb the machine. */
OMAC_API size_t omac_q_diagnostics(OMacQ*, char* out, size_t cap);

/* Drain the machine's own diagnostics into a NUL-terminated buffer of newline-
   separated lines, for the front end's log. Always on and low volume: media
   events, and the guest exceptions behind a bomb box -- an access fault names
   the instruction, the address it reached for, and the road in. Call it from
   the frame loop; what does not fit stays queued for the next call. */
OMAC_API void omac_q_poll_log(OMacQ*, char* out, size_t cap);

/* Flush and unmount the guest's volumes, as Shut Down would. Call this before
   persisting a hard disk image when the host application is closing: without
   it the System's cached blocks never reach the image and the volume stays
   flagged unclean, which the Quadra ROM refuses at the next boot. Returns
   non-zero if a volume was settled. */
OMAC_API int omac_q_shutdown_volumes(OMacQ*);

/* The displays the built-in video port can drive. Walk index 0 upward until
   this returns NULL; each name is what omac_q_set_display takes. Naming the
   monitor chooses the resolution: an Apple fixed-frequency display runs one.
   Set it BEFORE the machine boots -- the ROM senses the monitor at startup. */
OMAC_API const char* omac_q_display_name(int index, int* w, int* h);
OMAC_API int omac_q_set_display(OMacQ*, const char* name);

/* The floppy medium as its host file should hold it: the guest's sectors with
   the containers the file arrived in reassembled around them. Returns the disk
   still in the drive, or the one last ejected, so a front end can save it both
   when the user asks and when the machine hands a disk back. Pass out=NULL for
   the size; 0 means there is nothing to save. */
OMAC_API size_t omac_q_floppy_writeback(OMacQ*, uint8_t* out, size_t cap);
/* Floppy in the internal SuperDrive. Raw 400K/800K/1.44MB dumps and DiskCopy
   4.2 / MacBinary wrappers all mount; returns 1 if the drive took the disk,
   0 if the file is not floppy media. omac_q_floppy_data copies the medium out
   so guest writes can be persisted. */
OMAC_API int    omac_q_insert_floppy(OMacQ*, const uint8_t* img, size_t len, int read_only);
OMAC_API void   omac_q_eject_floppy(OMacQ*);
OMAC_API int    omac_q_floppy_present(OMacQ*);
OMAC_API size_t omac_q_floppy_data(OMacQ*, uint8_t* out, size_t cap);
/* CD-ROM on the SCSI bus (AppleCD-class target). Takes a raw ISO/Apple-
   partitioned disc image; returns 1 if accepted. */
OMAC_API int    omac_q_insert_cd(OMacQ*, const uint8_t* img, size_t len);
OMAC_API void   omac_q_eject_cd(OMacQ*);
OMAC_API int    omac_q_cd_present(OMacQ*);

/* ---- Macintosh IIfx (1990) ---------------------------------------------
   A separate opaque handle for the 40 MHz 68030 machine. The supplied ROM
   discovers a slot-$9 Macintosh II Video Card (640x480), the OSS/IOP board,
   and a SCSI disk. RAM is specified in megabytes; legal physical totals are
   4, 8, 16, 20, 32, 64, 68, 80 and 128. Audio is unsigned 8-bit mono PCM;
   query the guest-selected ASC clock with omac_fx_audio_rate(). */
typedef struct OMacFx OMacFx;
OMAC_API OMacFx* omac_fx_create(const uint8_t* rom, size_t rom_len, uint32_t ram_mb);
/* Optional external Apple Macintosh Display Card 8*24 GC declaration ROM.
   Passing null/zero retains OpenMac's synthetic fallback card. */
OMAC_API OMacFx* omac_fx_create_with_video_rom(const uint8_t* rom,
                                                size_t rom_len,
                                                const uint8_t* video_rom,
                                                size_t video_rom_len,
                                                uint32_t ram_mb);
OMAC_API void    omac_fx_destroy(OMacFx*);
OMAC_API void    omac_fx_reset(OMacFx*);
OMAC_API void    omac_fx_run_frame(OMacFx*);
OMAC_API int     omac_fx_screen_w(OMacFx*);
OMAC_API int     omac_fx_screen_h(OMacFx*);
OMAC_API void    omac_fx_render(OMacFx*, uint32_t* argb);
OMAC_API uint32_t omac_fx_audio_rate(OMacFx*);
OMAC_API size_t  omac_fx_drain_audio(OMacFx*, uint8_t* out, size_t cap);
OMAC_API void    omac_fx_mouse(OMacFx*, int dx, int dy, int button);
OMAC_API void    omac_fx_key(OMacFx*, int adb_code, int down);
OMAC_API int     omac_fx_insert_floppy(OMacFx*, const uint8_t* img,
                                        size_t len, int read_only);
OMAC_API void    omac_fx_eject_floppy(OMacFx*);
OMAC_API int     omac_fx_floppy_present(OMacFx*);
OMAC_API size_t  omac_fx_floppy_writeback(OMacFx*, uint8_t* out, size_t cap);
OMAC_API void    omac_fx_insert_harddisk(OMacFx*, const uint8_t* img,
                                          size_t len, int read_only);
OMAC_API void    omac_fx_detach_harddisk(OMacFx*);
OMAC_API size_t  omac_fx_harddisk_data(OMacFx*, uint8_t* out, size_t cap);
/* Flush and unmount drive 4 through the guest before saving/replacing it. */
OMAC_API int     omac_fx_shutdown_harddisk(OMacFx*);
/* The second SCSI disk (ID 1, drive 5) -- the transfer disk's and the folder
   disk's seat, the same shape as the Quadra's omac_q_harddisk2_*. _booted
   reports whether the System has that seat's driver (its drive is in the
   drive queue): the ROM loads it during the startup scan, so a disk put here
   afterwards needs a restart to mount. _unmount flushes and unmounts ONLY this
   volume; non-zero once it is off line. */
OMAC_API void    omac_fx_insert_harddisk2(OMacFx*, const uint8_t* img,
                                           size_t len, int read_only);
OMAC_API void    omac_fx_detach_harddisk2(OMacFx*);
OMAC_API size_t  omac_fx_harddisk2_data(OMacFx*, uint8_t* out, size_t cap);
OMAC_API int     omac_fx_harddisk2_booted(OMacFx*);
OMAC_API int     omac_fx_unmount_harddisk2(OMacFx*);
/* CD-ROM on the IIfx: an AppleCD-class target on the SCSI bus (ID 3 by
   default) plus a ".AppleCD" driver the machine installs once the System is
   up, so a disc mounts without Apple's CD-ROM software (which a stock 7.1
   disk lacks). Takes the same images as the Quadra's drive: .iso, .toast, raw
   MODE1 .bin, Apple-partitioned or bare HFS masters; an ISO-9660-only disc is
   accepted but the guest cannot mount it without Foreign File Access. Insert
   returns 1 if the drive took the disc; the guest may eject it (drag to the
   Trash), so poll _cd_present rather than trusting the last insert. Attaching
   the drive puts the target on the bus for good; a checkpoint taken with it
   loads only into a machine that has it. */
OMAC_API void    omac_fx_attach_cd(OMacFx*, int attached, int scsi_id);
OMAC_API int     omac_fx_cd_attached(OMacFx*);
OMAC_API int     omac_fx_insert_cd(OMacFx*, const uint8_t* img, size_t len);
OMAC_API void    omac_fx_eject_cd(OMacFx*);
OMAC_API int     omac_fx_cd_present(OMacFx*);
OMAC_API size_t  omac_fx_pram_save(OMacFx*, uint8_t* out, size_t cap);
OMAC_API int     omac_fx_pram_load(OMacFx*, const uint8_t* data, size_t len,
                                    uint32_t add_seconds);
OMAC_API size_t  omac_fx_diagnostics(OMacFx*, char* out, size_t cap);
OMAC_API void    omac_fx_poll_log(OMacFx*, char* out, size_t cap);

/* Version string for the About box / logs. */
OMAC_API const char* omac_version(void);

#ifdef __cplusplus
}
#endif

#endif /* OPENMAC_CAPI_H */
