/*
    Copyright (c) 2014 Yaniv Kamay,
    All rights reserved.

    Source code is provided for evaluation purposes only. Modification or use in
    source code for any other purpose is prohibited.

    Binary code (i.e. the binary form of source code form) is allowed to use for
    evaluation purposes only. Modification or use in binary code for any other
    purpose is prohibited.

    Redistribution, in source form or in binary form, with or without modification,
    are not permitted.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTOR BE LIABLE FOR
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
    ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
    IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "utils.h"
#include "bios.h"
#include "platform.h"


#define VRAM_BASE 0xb800
#define NUM_COLUMNS 80
#define NUM_ROWS 25

#define BACKGROUND_COLOR 0xc
#define BLACK 0
#define VGA_FADE_DELAY 300
#define FADE_OUT_DELAY_MSEC 100
#define FADE_IN_DELAY_MSEC 150
#define MESSAGE_LINE 23
#define MARGIN 2
#define UI_DELAY_MSEC 1000

#define BAR_WAIT_MSEC 3000
#define BAR_COLOR 7
#define BAR_PROGRESS_COLOR 4
#define BAR_TEXT_COLOR 0xb

#define MENU_START_LINE 4
#define MENU_WIDTH 40
#define MENU_HIGHT 10
#define MENU_BACK_COLOR 0xd
#define MENU_TEXT_COLOR 7
#define MENU_SELECTION_COLOR 0xf


typedef _Packed struct Cell {
    char ch;
    uint8_t attrib;
} Cell;


const char *bios_str = "Ignition v1.0";
const char *enter_boot = "Press <ESC> to enter boot menu";


static bool_t set_video_mode(uint8_t mode, bool_t erase)
{
    if (!erase) mode |= 0x80;

    __asm {
        mov ah, 0
        mov al, mode
        int 10h
    }

    return TRUE;
}


static void hide_cursor()
{
    __asm {
        mov ah, 0x01
        mov ch, 1
        mov cl, 0
        int 10h
    }
}


static void init_gray_palet()
{
    uint8_t i;

    outb(IO_PORT_VGA_PALETTE_WRITE_INDEX, 0);

    for (i = 0; i < 16; i++) {
        outb(IO_PORT_VGA_PALETTE_DATA, i * 0x3f / 16);
        outb(IO_PORT_VGA_PALETTE_DATA, i * 0x3f / 16);
        outb(IO_PORT_VGA_PALETTE_DATA, i * 0x3f / 16);

        __asm {
            mov ax, 0x1000
            mov bl, i
            mov bh, i
            int 10h
        }
    }
}


static void fill_area(uint top, uint left, uint bottom, uint right, uint color)
{
    if (top >= NUM_ROWS || bottom > NUM_ROWS) {
        return;
    }

    if (left >= NUM_COLUMNS || right > NUM_COLUMNS) {
        return;
    }

    for (; top < bottom; top++) {
        Cell __far * ptr = FAR_POINTER(Cell, VRAM_BASE, top * NUM_COLUMNS * sizeof(Cell));
        uint col;

        for (col = left; col < right; col++) {
            ptr[col].ch = ' ';
            ptr[col].attrib = (ptr[col].attrib & 0x0f) | (color  << 4);
        }
    }
}


static void fill_back(uint top, uint left, uint bottom, uint right, uint color)
{
    if (top >= NUM_ROWS || bottom > NUM_ROWS) {
        return;
    }

    if (left >= NUM_COLUMNS || right > NUM_COLUMNS) {
        return;
    }

    for (; top < bottom; top++) {
        Cell __far * ptr = FAR_POINTER(Cell, VRAM_BASE, top * NUM_COLUMNS * sizeof(Cell));
        uint col;

        for (col = left; col < right; col++) {
            ptr[col].attrib = (ptr[col].attrib & 0x0f) | (color  << 4);
        }
    }
}


static void fade(uint _top, uint left, uint bottom, uint right,
                     uint stop_color, uint msec)
{
    bool_t more;

    if (_top >= NUM_ROWS || bottom > NUM_ROWS) {
        return;
    }

    if (left >= NUM_COLUMNS || right > NUM_COLUMNS) {
        return;
    }

    do {
        uint line;

        more = FALSE;

        for (line = _top; line < bottom; line++) {
            Cell __far * ptr = FAR_POINTER(Cell, VRAM_BASE, line * NUM_COLUMNS * sizeof(Cell));
            uint col;

            for (col = left; col < right; col++) {
                uint now = ptr[col].attrib & 0x0f;

                if (now < stop_color) {
                    now += 1;
                } else if (now > stop_color) {
                    now -= 1;
                }

                more = more || (now != stop_color);
                ptr[col].attrib = (ptr[col].attrib & 0xf0) | now;
            }
        }

        delay(msec);
    } while (more);
}


static void draw_string(const char __far * str, uint row, uint col, uint color, uint limit)
{
    Cell __far * ptr = FAR_POINTER(Cell, VRAM_BASE, row * NUM_COLUMNS * sizeof(Cell));
    limit = MIN(NUM_COLUMNS, NUM_COLUMNS);
    limit = MIN(NUM_COLUMNS, col + limit);

    if (row >= NUM_ROWS || col >= NUM_COLUMNS) {
        return;
    }

    for (; *str && col < limit; col++, str++) {
        ptr[col].ch = *str;
        ptr[col].attrib = (ptr[col].attrib & 0xf0) | color;
    }
}


static bool_t get_key(uint8_t __far * scan)
{
    bool_t ok;
    uint8_t key;

    __asm {
        mov ah, 1
        int 16h
        jz no_key
        mov ah, 0
        int 16h
        mov key, ah
        mov ah, 1
        mov ok, ah
        jmp done
    no_key:
        xor ax, ax
        mov ok, al
    done:
    }

    *scan = key;

    return ok;
}


static char get_key_sync()
{
    char key;

    __asm {
        mov ah, 0
        int 16h
        mov key, ah
    }

    return key;
}


static void flush_kbd()
{
    uint8_t scan;

    while (get_key(&scan)) {
        get_key_sync();
    }
}


static void boot_menu_select(uint current, uint new)
{
    uint menue_start_col = (NUM_COLUMNS - MENU_WIDTH) / 2;

    fill_back(MENU_START_LINE + current, menue_start_col, MENU_START_LINE + current + 1,
              menue_start_col + MENU_WIDTH, MENU_BACK_COLOR);
    fill_back(MENU_START_LINE + new, menue_start_col, MENU_START_LINE + new + 1,
              menue_start_col + MENU_WIDTH, MENU_SELECTION_COLOR);
}


static void boot_menu()
{
    char __far * option_str;
    uint i = 0;
    uint menue_start_col = (NUM_COLUMNS - MENU_WIDTH) / 2;
    uint selection;
    uint last_index;

    fill_area(MENU_START_LINE - 1, menue_start_col - 2, MENU_START_LINE + MENU_HIGHT + 1,
              menue_start_col + MENU_WIDTH + 2, MENU_BACK_COLOR);
    fill_area(MENU_START_LINE, menue_start_col, MENU_START_LINE + MENU_HIGHT,
              menue_start_col + MENU_WIDTH, MENU_BACK_COLOR);

    while ((option_str = boot_get_option(i)) && i < MENU_HIGHT) {
        draw_string(option_str, MENU_START_LINE + i, menue_start_col, MENU_TEXT_COLOR, MENU_WIDTH);
        ++i;
    }

    boot_menu_select(0, 0);
    selection = 0;
    last_index = i - 1;

    flush_kbd();

    for (;;) {
        uint8_t scan_code = get_key_sync();

        switch (scan_code) {
        case KBD_SCAN_ESCAPE:
            return;
        case KBD_EXT_PAD_ENTER:
            boot_set_boot_option(selection);
            return;
        case KBD_SCAN_PAD_UP:
            if (selection == 0) {
                break;
            }

            boot_menu_select(selection, selection - 1);
            selection -= 1;
            break;
        case KBD_SCAN_PAD_DOWN:
            if (selection == last_index) {
                break;
            }

            boot_menu_select(selection, selection + 1);
            selection += 1;
            break;
        }
    }
}


static void boot_menu_opportunity()
{
    uint bar_start = MARGIN + string_length(bios_str) + 1;
    uint bar_end = NUM_COLUMNS - MARGIN;
    uint message_start = ((bar_end - bar_start) - string_length(enter_boot))  / 2 + bar_start;
    uint i;

    fill_area(MESSAGE_LINE, bar_start, MESSAGE_LINE + 1, bar_end, BAR_COLOR);
    draw_string(enter_boot, MESSAGE_LINE, message_start, BAR_TEXT_COLOR, ~0);

    flush_kbd();

    for (i = 1; i <= (bar_end - bar_start); i++) {
        uint8_t scan_code = 0;

        fill_back(MESSAGE_LINE, bar_start, MESSAGE_LINE + 1, bar_start + i, BAR_PROGRESS_COLOR);
        delay(BAR_WAIT_MSEC / (bar_end - bar_start));

        while (get_key(&scan_code)) {
            if (scan_code == KBD_SCAN_ESCAPE) {
                fill_area(MESSAGE_LINE, bar_start, MESSAGE_LINE + 1, bar_end, BAR_PROGRESS_COLOR);
                boot_menu();
                return;
            }
        }
    }
}


void ui()
{
    uint8_t num_strings = ebda_read_byte(OFFSET_OF_PRIVATE(num_display_strings));
    uint32_t ebda_seg = bda_read_word(BDA_OFFSET_EBDA);
    address_t __far * strings = FAR_POINTER(address_t, ebda_seg,
                                            OFFSET_OF_PRIVATE(display_strings));
    uint i;

    set_video_mode(3, FALSE);
    hide_cursor();
    init_gray_palet();
    fill_area(0, 0, NUM_ROWS, NUM_COLUMNS, BACKGROUND_COLOR);

    for (i = 0; i < num_strings; i++) {
        char __far * str = (char __far *)strings[i];
        draw_string(str, MESSAGE_LINE, MARGIN, BACKGROUND_COLOR, ~0);
        fade(MESSAGE_LINE, MARGIN, MESSAGE_LINE + 1, MARGIN + string_length(str), BLACK,
             FADE_OUT_DELAY_MSEC);
        fade(MESSAGE_LINE, MARGIN, MESSAGE_LINE + 1, MARGIN + string_length(str), BACKGROUND_COLOR,
             FADE_OUT_DELAY_MSEC);
    }

    draw_string(bios_str, MESSAGE_LINE, MARGIN, BACKGROUND_COLOR, ~0);
    fade(MESSAGE_LINE, MARGIN, MESSAGE_LINE + 1, MARGIN + string_length(bios_str),
         BLACK, FADE_IN_DELAY_MSEC);

    if (boot_multiple_options()) {
        boot_menu_opportunity();
    } else {
        delay(UI_DELAY_MSEC);
    }

    set_video_mode(3, TRUE);
}

