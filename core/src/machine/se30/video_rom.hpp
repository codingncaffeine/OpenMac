#pragma once

// OpenMac's own declaration ROM for the Macintosh SE/30's built-in screen, for
// a machine started without the file of the real one. It answers the System
// the way the real ROM does, call for call: the same resources and mode, a
// start-up routine that makes the two VIA1 lines outputs and fills both
// buffers with gray, and a driver whose answers were compared with the real
// driver's under System 6.0.8 and 7.5.
//
// What the real driver was seen to do, and this one does too:
//   Control 0  reset: the main buffer shown and filled with gray; answers mode
//              $80, page 0 and the main buffer's address
//           1  nothing
//           2  set mode: mode $80 only; page 0 = main buffer, 1 = alternate;
//              answers the buffer's address
//           5  gray page: fills the buffer being shown, whatever page is named
//           7  the vertical-blank interrupt on (0) or off (not 0)
//           others: an error (-17), the color table and gamma calls among them
//   Status  1  nothing, no error
//           2  mode $80, the page shown and its buffer's address
//           3  writes a whole two-entry color table (white, black) with a new
//              seed at the table pointer, whatever entries are asked for
//           4  two pages, whatever mode is named
//           5  a page's address, the reverse of set mode's: page 0 answers
//              the alternate buffer's, page 1 the main one's; -18 past page 1
//           7  a zero word, with the interrupt on or off
//           others: an error (-18)
//
// The routines are 68000 code written for this file. The identifiers the
// System matches on (the board ID, the resource types and names, the mode's
// parameters) are the hardware's.
//
// Reference: Designing Cards and Drivers for the Macintosh Family, 3rd ed.
// (declaration ROMs, video drivers); the SE/30 hardware dossier; the real
// video ROM's answers as observed through the System. Clean-room.

#include "../declrom.hpp"
#include "video.hpp"

#include <vector>

namespace openmac {

class Se30VideoRom {
public:
    static constexpr std::size_t kRomBytes = 0x2000;       // 8 KB, as the real one

    static std::vector<u8> build() {
        declrom::RomBuilder r;
        r.label("directory");
        r.offsetEntry(1, "board_resource");
        r.offsetEntry(0x80, "video_resource");
        r.endList();

        r.label("board_resource");
        r.offsetEntry(1, "board_type");
        r.offsetEntry(2, "board_name");
        r.dataEntry(32, 0x000C);                           // the board ID
        r.offsetEntry(34, "primary_init");
        r.offsetEntry(36, "vendor_info");
        r.endList();
        r.label("board_type");
        r.word(1); r.word(0); r.word(0); r.word(0);
        r.label("board_name"); r.cstring("Macintosh SE/30 Internal Video");
        even(r);
        r.label("vendor_info");
        r.offsetEntry(1, "vendor_name");
        r.offsetEntry(3, "vendor_revision");
        r.endList();
        r.label("vendor_name"); r.cstring("OpenMac");
        even(r);
        r.label("vendor_revision"); r.cstring("OpenMac-1.0");
        even(r);

        r.label("primary_init");
        {
            const std::vector<u8> code = buildStartUpRoutine();
            r.longword(static_cast<u32>(code.size() + 12));
            r.byte(2);                                     // revision
            r.byte(2);                                     // 68020 code
            r.word(0);
            r.longword(4);                                 // the code follows
            r.data.insert(r.data.end(), code.begin(), code.end());
            even(r);
        }

        r.label("video_resource");
        r.offsetEntry(1, "video_type");
        r.offsetEntry(2, "video_name");
        r.offsetEntry(4, "driver_directory");
        r.dataEntry(8, 1);                                 // hardware device ID
        r.offsetEntry(10, "minor_base");
        r.offsetEntry(11, "minor_length");
        r.offsetEntry(0x80, "mode_list");
        r.endList();
        r.label("video_type");
        r.word(3); r.word(1); r.word(1); r.word(9);        // display, video, Apple, this screen
        r.label("video_name"); r.cstring("Display_Video_Apple_MacSE/30 Video");
        even(r);
        r.label("minor_base"); r.longword(0);
        r.label("minor_length"); r.longword(Se30Video::kMainOffset + Se30Video::kBufferBytes);

        r.label("driver_directory");
        r.offsetEntry(2, "driver_block");                  // 68020
        r.endList();
        r.label("driver_block");
        {
            const std::vector<u8> driver = buildDriver();
            r.longword(static_cast<u32>(driver.size() + 4));
            r.data.insert(r.data.end(), driver.begin(), driver.end());
            even(r);
        }

        r.label("mode_list");
        r.offsetEntry(1, "mode_parameters");
        r.dataEntry(3, 2);                                 // two pages
        r.dataEntry(4, 1);                                 // a fixed color table
        r.endList();
        r.label("mode_parameters");
        r.longword(46);                                    // this block's size
        r.longword(Se30Video::kMainOffset);
        r.word(static_cast<u16>(Se30Video::kRowBytes));
        r.word(0); r.word(0); r.word(Se30Video::kHeight); r.word(Se30Video::kWidth);
        r.word(0);                                         // version
        r.word(0); r.longword(0);                          // no packing
        r.longword(72u << 16); r.longword(72u << 16);      // dots an inch
        r.word(0);                                         // indexed
        r.word(1);                                         // one bit a dot
        r.word(1); r.word(1);
        r.longword(0);

        declrom::finish(r, kRomBytes, 0x0F);
        return r.data;
    }

private:
    // Low memory: the base of VIA1, and the vectors a driver leaves through.
    static constexpr u16 kVia = 0x01D4;
    static constexpr u16 kJIODone = 0x08FC;
    static constexpr u16 kJVBLTask = 0x0D28;
    // VIA1's registers from its base: the two port registers and their
    // direction registers.
    static constexpr u16 kPortA = 0x1E00;
    static constexpr u16 kDirectionB = 0x0400;
    static constexpr u16 kDirectionA = 0x0600;

    static void even(declrom::RomBuilder& r) {
        if (r.data.size() & 1u) r.byte(0);
    }

    // Fills the buffer at A1 with gray: 171 pairs of rows, $AA bytes then
    // $55 bytes. Uses D0 and D2; A1 ends past the buffer.
    static void emitGray(declrom::CodeBuilder& c) {
        c.label("gray");
        c.word(0x343C); c.word(Se30Video::kHeight / 2 - 1);   // MOVE.W #170,D2
        c.label("gray_pair");
        c.word(0x700F);                                       // MOVEQ #15,D0
        c.label("gray_even");
        c.word(0x22FC); c.longword(0xAAAAAAAAu);              // MOVE.L #$AAAAAAAA,(A1)+
        c.branch(0x51C8, "gray_even");                        // DBF D0
        c.word(0x700F);                                       // MOVEQ #15,D0
        c.label("gray_odd");
        c.word(0x22FC); c.longword(0x55555555u);              // MOVE.L #$55555555,(A1)+
        c.branch(0x51C8, "gray_odd");                         // DBF D0
        c.branch(0x51CA, "gray_pair");                        // DBF D2
        c.word(0x4E75);                                       // RTS
    }

    // The board resource's start-up routine, run by the Slot Manager with A0
    // at its parameter block (the slot number in its first byte, the status
    // word at +2).
    static std::vector<u8> buildStartUpRoutine() {
        declrom::CodeBuilder c;
        c.word(0x48E7); c.word(0xE040);                       // MOVEM.L D0-D2/A1,-(SP)
        c.word(0x317C); c.word(1); c.word(2);                 // MOVE.W #1,2(A0): done
        c.word(0x2278); c.word(kVia);                         // MOVEA.L (VIA).W,A1
        c.word(0x08E9); c.word(6); c.word(kPortA);            // BSET #6: the main buffer
        c.word(0x08E9); c.word(6); c.word(kDirectionA);       // BSET #6: that line an output
        c.word(0x08D1); c.word(6);                            // BSET #6,(A1): interrupt off
        c.word(0x08E9); c.word(6); c.word(kDirectionB);       // BSET #6: that line an output
        // The screen's memory in the form that holds in both addressing
        // modes: $Fss0 0000 for slot s.
        c.word(0x7000);                                       // MOVEQ #0,D0
        c.word(0x1010);                                       // MOVE.B (A0),D0
        c.word(0x3200);                                       // MOVE.W D0,D1
        c.word(0xE949);                                       // LSL.W #4,D1
        c.word(0x8240);                                       // OR.W D0,D1
        c.word(0x0041); c.word(0x0F00);                       // ORI.W #$0F00,D1
        c.word(0x4841);                                       // SWAP D1
        c.word(0xE989);                                       // LSL.L #4,D1
        c.word(0x2241);                                       // MOVEA.L D1,A1
        c.word(0xD3FC); c.longword(Se30Video::kMainOffset);   // ADDA.L #$8040,A1
        c.branch(0x6100, "gray");                             // BSR
        c.word(0x2241);                                       // MOVEA.L D1,A1
        c.word(0x43E9); c.word(Se30Video::kAlternateOffset);  // LEA $40(A1),A1
        c.branch(0x6100, "gray");                             // BSR
        c.word(0x4CDF); c.word(0x0207);                       // MOVEM.L (SP)+,D0-D2/A1
        c.word(0x4E75);                                       // RTS
        emitGray(c);
        c.patch();
        return c.code;
    }

    // A Control or Status call ends here: an immediate call returns, any
    // other goes through the Device Manager's exit with the result in D0.
    static void emitDriverExit(declrom::CodeBuilder& c) {
        c.word(0x0828); c.word(0x0009); c.word(0x0006);       // BTST #9,ioTrap(A0)
        c.branch(0x6600, "driver_rts");                       // immediate: RTS
        c.word(0x2078); c.word(kJIODone);                     // MOVEA.L (JIODone).W,A0
        c.word(0x4ED0);                                       // JMP (A0)
    }

    static std::vector<u8> buildDriver() {
        declrom::CodeBuilder c;
        c.word(0x4C00);                                       // locked, Control and Status
        c.word(0); c.word(0); c.word(0);
        const std::size_t openOff = c.code.size(); c.word(0);
        c.word(0);                                            // no Prime routine
        const std::size_t ctlOff = c.code.size(); c.word(0);
        const std::size_t statOff = c.code.size(); c.word(0);
        const std::size_t closeOff = c.code.size(); c.word(0);
        constexpr char kDriverName[] = ".Display_Video_Apple_MacSE/30 Video";
        c.code.push_back(static_cast<u8>(sizeof(kDriverName) - 1));
        c.code.insert(c.code.end(), kDriverName, kDriverName + sizeof(kDriverName) - 1);
        if (c.code.size() & 1u) c.code.push_back(0);
        c.word(0);                                            // version

        // Open: A0 = the parameter block, A1 = the device control entry.
        // The interrupt handler goes on the slot's queue with the slot
        // number as its parameter, then the interrupt is switched on.
        c.label("open");
        c.word(0x48E7); c.word(0x00C0);                       // MOVEM.L A0-A1,-(SP)
        c.pcAddress(0x41FA, "slot_queue");                    // LEA slot_queue(PC),A0
        c.word(0x4290);                                       // CLR.L (A0)
        c.word(0x317C); c.word(6); c.word(4);                 // a slot interrupt element
        c.word(0x4268); c.word(6);                            // priority 0
        c.word(0x7000);                                       // MOVEQ #0,D0
        c.word(0x1029); c.word(0x0028);                       // MOVE.B dCtlSlot(A1),D0
        c.word(0x2140); c.word(12);                           // its parameter: the slot
        c.pcAddress(0x43FA, "slot_handler");                  // LEA slot_handler(PC),A1
        c.word(0x2149); c.word(8);                            // its routine
        c.word(0xA075);                                       // _SIntInstall
        c.word(0x2078); c.word(kVia);                         // MOVEA.L (VIA).W,A0
        c.word(0x0890); c.word(6);                            // BCLR #6,(A0): interrupt on
        c.word(0x4CDF); c.word(0x0300);                       // MOVEM.L (SP)+,A0-A1
        c.word(0x7000);                                       // MOVEQ #0,D0
        c.word(0x4E75);                                       // RTS

        // Control: the call's code in D0, its record at A2, VIA1 at A3.
        c.label("control");
        c.word(0x48E7); c.word(0x60F0);                       // MOVEM.L D1-D2/A0-A3,-(SP)
        c.word(0x3028); c.word(0x001A);                       // MOVE.W csCode(A0),D0
        c.word(0x2468); c.word(0x001C);                       // MOVEA.L csParam(A0),A2
        c.word(0x2678); c.word(kVia);                         // MOVEA.L (VIA).W,A3
        c.word(0x4A40); c.branch(0x6700, "control_reset");
        c.word(0x0C40); c.word(1); c.branch(0x6700, "control_good");
        c.word(0x0C40); c.word(2); c.branch(0x6700, "control_mode");
        c.word(0x0C40); c.word(5); c.branch(0x6700, "control_gray");
        c.word(0x0C40); c.word(7); c.branch(0x6700, "control_interrupt");
        c.label("control_bad");
        c.word(0x70EF);                                       // MOVEQ #-17,D0
        c.branch(0x6000, "control_done");

        c.label("control_reset");
        c.word(0x08EB); c.word(6); c.word(kPortA);            // BSET #6: the main buffer
        c.word(0x34BC); c.word(0x0080);                       // mode $80
        c.word(0x426A); c.word(6);                            // page 0
        c.word(0x7200);                                       // MOVEQ #0,D1
        c.branch(0x6100, "page_address");
        c.word(0x2548); c.word(8);                            // its address
        c.word(0x2248);                                       // MOVEA.L A0,A1
        c.branch(0x6100, "gray");
        c.branch(0x6000, "control_good");

        c.label("control_mode");
        c.word(0x0C52); c.word(0x0080);                       // CMPI.W #$80,(A2)
        c.branch(0x6600, "control_bad");
        c.word(0x322A); c.word(6);                            // MOVE.W page,D1
        c.word(0x0C41); c.word(1);                            // CMPI.W #1,D1
        c.branch(0x6200, "control_bad");                      // past page 1
        c.branch(0x6700, "control_mode_alternate");
        c.word(0x08EB); c.word(6); c.word(kPortA);            // BSET #6: the main buffer
        c.branch(0x6000, "control_mode_address");
        c.label("control_mode_alternate");
        c.word(0x08AB); c.word(6); c.word(kPortA);            // BCLR #6: the alternate
        c.label("control_mode_address");
        c.branch(0x6100, "page_address");
        c.word(0x2548); c.word(8);                            // its address
        c.branch(0x6000, "control_good");

        c.label("control_gray");
        c.branch(0x6100, "page_shown");
        c.branch(0x6100, "page_address");
        c.word(0x2248);                                       // MOVEA.L A0,A1
        c.branch(0x6100, "gray");
        c.branch(0x6000, "control_good");

        c.label("control_interrupt");
        c.word(0x4A12);                                       // TST.B (A2)
        c.branch(0x6600, "control_interrupt_off");
        c.word(0x0893); c.word(6);                            // BCLR #6,(A3): on
        c.branch(0x6000, "control_good");
        c.label("control_interrupt_off");
        c.word(0x08D3); c.word(6);                            // BSET #6,(A3): off
        c.label("control_good");
        c.word(0x7000);                                       // MOVEQ #0,D0
        c.label("control_done");
        c.word(0x4CDF); c.word(0x0F06);                       // MOVEM.L (SP)+,D1-D2/A0-A3
        emitDriverExit(c);

        c.label("status");
        c.word(0x48E7); c.word(0x60F0);                       // MOVEM.L D1-D2/A0-A3,-(SP)
        c.word(0x3028); c.word(0x001A);                       // MOVE.W csCode(A0),D0
        c.word(0x2468); c.word(0x001C);                       // MOVEA.L csParam(A0),A2
        c.word(0x2678); c.word(kVia);                         // MOVEA.L (VIA).W,A3
        c.word(0x0C40); c.word(1); c.branch(0x6700, "status_good");
        c.word(0x0C40); c.word(2); c.branch(0x6700, "status_mode");
        c.word(0x0C40); c.word(3); c.branch(0x6700, "status_table");
        c.word(0x0C40); c.word(4); c.branch(0x6700, "status_pages");
        c.word(0x0C40); c.word(5); c.branch(0x6700, "status_address");
        c.word(0x0C40); c.word(7); c.branch(0x6700, "status_interrupt");
        c.label("status_bad");
        c.word(0x70EE);                                       // MOVEQ #-18,D0
        c.branch(0x6000, "status_done");

        c.label("status_mode");
        c.word(0x34BC); c.word(0x0080);                       // mode $80
        c.branch(0x6100, "page_shown");
        c.word(0x3541); c.word(6);                            // the page
        c.branch(0x6100, "page_address");
        c.word(0x2548); c.word(8);                            // its address
        c.branch(0x6000, "status_good");

        // A color table at the pointer: a new seed, a device's table of two
        // entries, white then black.
        c.label("status_table");
        c.word(0x598F);                                       // SUBQ.L #4,SP
        c.word(0xAA28);                                       // _GetCTSeed
        c.word(0x2052);                                       // MOVEA.L (A2),A0
        c.word(0x20DF);                                       // MOVE.L (SP)+,(A0)+
        c.word(0x20FC); c.longword(0x80000001u);              // flags, last entry's number
        c.word(0x4258);                                       // CLR.W (A0)+
        c.word(0x20FC); c.longword(0xFFFFFFFFu);              // white
        c.word(0x30FC); c.word(0xFFFF);
        c.word(0x4298);                                       // CLR.L (A0)+: black
        c.word(0x4298);                                       // CLR.L (A0)+
        c.branch(0x6000, "status_good");

        c.label("status_pages");
        c.word(0x357C); c.word(2); c.word(6);                 // two pages
        c.branch(0x6000, "status_good");

        c.label("status_address");
        c.word(0x322A); c.word(6);                            // MOVE.W page,D1
        c.word(0x0C41); c.word(1);                            // CMPI.W #1,D1
        c.branch(0x6200, "status_bad");                       // past page 1
        c.word(0x0A41); c.word(1);                            // EORI.W #1,D1: the other buffer
        c.branch(0x6100, "page_address");
        c.word(0x2548); c.word(8);
        c.branch(0x6000, "status_good");

        c.label("status_interrupt");
        c.word(0x4252);                                       // CLR.W (A2)
        c.label("status_good");
        c.word(0x7000);                                       // MOVEQ #0,D0
        c.label("status_done");
        c.word(0x4CDF); c.word(0x0F06);                       // MOVEM.L (SP)+,D1-D2/A0-A3
        emitDriverExit(c);

        // Close: the handler off the slot's queue, the interrupt off.
        c.label("close");
        c.word(0x48E7); c.word(0x00C0);                       // MOVEM.L A0-A1,-(SP)
        c.word(0x7000);                                       // MOVEQ #0,D0
        c.word(0x1029); c.word(0x0028);                       // MOVE.B dCtlSlot(A1),D0
        c.pcAddress(0x41FA, "slot_queue");                    // LEA slot_queue(PC),A0
        c.word(0xA076);                                       // _SIntRemove
        c.word(0x2078); c.word(kVia);                         // MOVEA.L (VIA).W,A0
        c.word(0x08D0); c.word(6);                            // BSET #6,(A0): interrupt off
        c.word(0x4CDF); c.word(0x0300);                       // MOVEM.L (SP)+,A0-A1
        c.word(0x7000);                                       // MOVEQ #0,D0
        c.label("driver_rts");
        c.word(0x4E75);                                       // RTS

        // D1 = the page being shown: 0 while VIA1's line picks the main buffer.
        c.label("page_shown");
        c.word(0x7200);                                       // MOVEQ #0,D1
        c.word(0x082B); c.word(6); c.word(kPortA);            // BTST #6,port A
        c.branch(0x6600, "page_shown_done");
        c.word(0x7201);                                       // MOVEQ #1,D1
        c.label("page_shown_done");
        c.word(0x4E75);                                       // RTS

        // A0 = the buffer of page D1 (0 = main), from the device's base in
        // the control entry at A1.
        c.label("page_address");
        c.word(0x2069); c.word(0x002A);                       // MOVEA.L dCtlDevBase(A1),A0
        c.word(0x4A41);                                       // TST.W D1
        c.branch(0x6600, "page_address_alternate");
        c.word(0xD1FC); c.longword(Se30Video::kMainOffset);   // ADDA.L #$8040,A0
        c.word(0x4E75);                                       // RTS
        c.label("page_address_alternate");
        c.word(0x41E8); c.word(Se30Video::kAlternateOffset);  // LEA $40(A0),A0
        c.word(0x4E75);                                       // RTS

        emitGray(c);

        // The slot's interrupt: the request is dropped by raising VIA1's
        // line and lowering it again, then the slot's vertical-blank tasks
        // run. A1 holds the slot number; D0 not zero says "it was mine".
        c.label("slot_handler");
        c.word(0x2009);                                       // MOVE.L A1,D0
        c.word(0x2078); c.word(kVia);                         // MOVEA.L (VIA).W,A0
        c.word(0x08D0); c.word(6);                            // BSET #6,(A0)
        c.word(0x0890); c.word(6);                            // BCLR #6,(A0)
        c.word(0x2078); c.word(kJVBLTask);                    // MOVEA.L (JVBLTask).W,A0
        c.word(0x4E90);                                       // JSR (A0)
        c.word(0x7001);                                       // MOVEQ #1,D0
        c.word(0x4E75);                                       // RTS
        c.label("slot_queue");
        for (int index = 0; index < 8; ++index) c.word(0);
        c.patch();

        auto putHeaderOffset = [&](std::size_t at, const char* label) {
            const u16 value = static_cast<u16>(c.labels.at(label));
            c.code[at] = static_cast<u8>(value >> 8);
            c.code[at + 1] = static_cast<u8>(value);
        };
        putHeaderOffset(openOff, "open");
        putHeaderOffset(ctlOff, "control");
        putHeaderOffset(statOff, "status");
        putHeaderOffset(closeOff, "close");
        return c.code;
    }
};

} // namespace openmac
