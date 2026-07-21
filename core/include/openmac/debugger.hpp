#pragma once

// A small ROM monitor for OpenMac: register dumps, a trap-name table, named
// low-memory globals, and a compact 68000 disassembler. Intended for the
// headless trace tool and the shell's debug view.

#include "openmac/machine.hpp"

#include <cstdio>
#include <string>

namespace openmac::dbg {

// Name of an A-line trap ($Axxx), or nullptr if unknown.
const char* trapName(u16 opcode);

// Resolve an address to the nearest known symbol as "Name" or "Name+offset",
// or "" if none is known. Sources: low-memory globals and the OS trap
// dispatch table (so a ROM handler resolves to its trap name). Reads live
// state through the bus, so trap handlers resolve once the table is built.
std::string symbolFor(Machine& mac, u32 addr);

// True for OS traps that return a pointer/handle in A0 (New/Realloc/Recover);
// a zero result is the classic source of a later NIL-dereference crash.
bool trapReturnsPtrInA0(u16 opcode);

// Named low-memory global (Inside Macintosh). size is 1/2/4 bytes.
struct LowMem {
    u32 addr;
    const char* name;
    int size;
};

// Format the CPU register file (D0-D7, A0-A7, PC, SR) to `out`.
void dumpRegs(const M68000& cpu, std::FILE* out);

// Dump the well-known low-memory globals with their current values.
void dumpLowMem(Machine& mac, std::FILE* out);

// Disassemble one instruction at `pc`. Appends the text to `out` and returns
// the instruction length in bytes (>= 2). Reads through the machine bus.
int disasm(Machine& mac, u32 pc, std::string& out);

// Hex + ASCII dump of `len` bytes at `addr`.
void dumpMem(Machine& mac, u32 addr, u32 len, std::FILE* out);

// Walk the drive queue (DrvQHdr) and list each mounted drive.
void dumpDriveQueue(Machine& mac, std::FILE* out);

// Walk the unit table and show each driver's DCE (flags, refNum, I/O queue),
// which reveals whether a driver is stuck busy with a pending request.
void dumpUnitTable(Machine& mac, std::FILE* out);

// One-line summary of an I/O trap and its parameter block, for trap tracing.
// Returns false if the trap is not an I/O trap worth logging.
bool describeIOTrap(Machine& mac, u16 trap, u32 pc, u32 a0, std::string& out);

// Walk the A6 frame-pointer chain and print the return-address call stack.
void dumpBacktrace(const M68000& cpu, Machine& mac, std::FILE* out);

// Walk the current heap zone (TheZone) and report block counts, flagging the
// first block whose header looks corrupt — a common cause of NIL handles.
void checkHeap(Machine& mac, std::FILE* out);

// Walk the Time Manager task queue (tm_var at low-mem $0B30, queue at +8) and
// list the queued TMTasks. An empty queue here is a known boot-patch crash.
void dumpTimerQueue(Machine& mac, std::FILE* out);

// Dump the VIA 6522 registers and decode the pending+enabled interrupt
// sources (T1/T2/SR/CB1/CA1...), the heart of the machine's interrupt flow.
void dumpVia(Machine& mac, std::FILE* out);

} // namespace openmac::dbg
