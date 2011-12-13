/*
    Copyright (c) 2013 Yaniv Kamay,
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

#include "types.h"
#include "bios.h"
#include "utils.h"
#include "platform.h"
#include "defs.h"
#include "error_codes.h"

void int18_handler();
void int19_handler();
void far_call_no_return();

#define BOOT_OPT_FLAG_DONE (1 << 0)

enum  {
    BOOT_OPT_TYPE_INVALID,
    BOOT_OPT_TYPE_HD,
    BOOT_OPT_TYPE_CD,
};


void on_int19()
{
    uint32_t ebda_seg = bda_read_word(BDA_OFFSET_EBDA);
    uint8_t _far * boot_order = FAR_POINTER(uint8_t, ebda_seg, OFFSET_OF_PRIVATE(boot_order));
    BootOption __far * options = FAR_POINTER(BootOption, ebda_seg, OFFSET_OF_PRIVATE(boot_options));
    uint i;

    for (i = 0; i < MAX_BOOT_OPTIONS; i++) {
        BootOption __far * now;

        now = &options[boot_order[i]];

        if (now->type == BOOT_OPT_TYPE_INVALID) {
            break;
        }

        if (now->flags & BOOT_OPT_FLAG_DONE) {
            continue;
        }

        switch (now->type) {
        case BOOT_OPT_TYPE_HD:
        case BOOT_OPT_TYPE_CD: {
            uint16_t code_off = (uint16_t)now->boot_handler;
            uint16_t code_seg = (uint32_t)now->boot_handler >> 16;
            uint8_t user_data = now->user_data;

            now->flags |= BOOT_OPT_FLAG_DONE;
            D_MESSAGE("%x:%x hd_id %u", code_seg, code_off, user_data);
            __asm {
                mov dl, user_data
                push code_seg
                push code_off
                jmp far_call_no_return
            }

            break;
        }
        default:
            D_MESSAGE("invalid type");
            bios_error(BIOS_ERROR_BOOT_INVALID_TYPE);
        }
    }

    D_MESSAGE("no valid boot option found");
    freeze();
}


static void hd_boot()
{
    ATADevice __far * device;
    uint8_t hd_id;
    uint i;

    __asm { mov hd_id, dl}

    D_MESSAGE("0x%x", hd_id);

    device = FAR_POINTER(ATADevice, bda_read_word(BDA_OFFSET_EBDA), OFFSET_OF_PRIVATE(ata_devices));

    for ( i = 0; i < MAX_ATA_DEVICES; i++, device++) {
        if (!ata_is_hd(device) || device->hd_id != hd_id) {
            continue;
        }

        if (!ata_read_sectors(device, 0, 1, FAR_POINTER(uint8_t, 0, 0x7c00))) {
            D_MESSAGE("read failed");
            freeze();
        }

        term_printf("boot...");

        if (read_word(0, 0x7c00 + 510) == 0xaa55) {
            __asm {
                cli
                xor ax, ax
                mov ds, ax
                mov es, ax
                mov fs, ax
                mov gs, ax
                mov ss, ax
                mov sp, 0x7c00
                mov dl, hd_id
                db 0xea
                dw 0x7c00, 0x0000
            }
        }

        D_MESSAGE("invalid boot sector");
        break;
    }

    D_MESSAGE("not on device list");
    freeze();
}


static void cd_boot()
{
    D_MESSAGE("not yet");
    INT(0x18);
}


void boot_add_hd(ATADevice __far * device)
{
    uint32_t ebda_seg = bda_read_word(BDA_OFFSET_EBDA);
    BootOption __far * options = FAR_POINTER(BootOption, ebda_seg, OFFSET_OF_PRIVATE(boot_options));
    uint i;

    for ( i = 0; i < MAX_BOOT_OPTIONS; i++) {
        if (options[i].type) {
            continue;
        }

        options[i].type = BOOT_OPT_TYPE_HD;
        options[i].boot_handler = FAR_POINTER(void, get_cs(), FUNC_OFFSET(hd_boot));
        options[i].description = FAR_POINTER(char, ebda_seg, (uint)&device->description);
        options[i].user_data = device->hd_id;
        return;
    }

    D_MESSAGE("out of boot option slots");
    bios_warn(BIOS_WARN_BOOT_OUT_OF_SLOTS);
}


void boot_add_cd(ATADevice __far * device)
{
    uint32_t ebda_seg = bda_read_word(BDA_OFFSET_EBDA);
    BootOption __far * options = FAR_POINTER(BootOption, ebda_seg, OFFSET_OF_PRIVATE(boot_options));
    uint i;

    for ( i = 0; i < MAX_BOOT_OPTIONS; i++) {
        if (options[i].type) {
            continue;
        }

        options[i].type = BOOT_OPT_TYPE_CD;
        options[i].boot_handler = FAR_POINTER(void, get_cs(), FUNC_OFFSET(cd_boot));
        options[i].description = FAR_POINTER(char, ebda_seg, (uint)&device->description);
        options[i].user_data = 0;
        return;
    }

    D_MESSAGE("out of boot option slots");
    bios_warn(BIOS_WARN_BOOT_OUT_OF_SLOTS);
}


void boot()
{
    ebda_write_byte(OFFSET_OF_PRIVATE(bios_flags),
                    ebda_read_byte(OFFSET_OF_PRIVATE(bios_flags)) & ~BIOS_FLAGS_UNREAL);
    ebda_write_byte(OFFSET_OF_PRIVATE(call_select), CALL_SELECT_NOP);
    call32();

    INT(0x19);
}


void boot_init()
{
    uint32_t ebda_seg = bda_read_word(BDA_OFFSET_EBDA);
    uint8_t _far * boot_order = FAR_POINTER(uint8_t, ebda_seg, OFFSET_OF_PRIVATE(boot_order));
    uint i;

    for (i = 0; i < MAX_BOOT_OPTIONS; i++) {
        boot_order[i] = i;
    }

    set_int_vec(0x18, get_cs(), FUNC_OFFSET(int18_handler));
    set_int_vec(0x19, get_cs(), FUNC_OFFSET(int19_handler));
}

