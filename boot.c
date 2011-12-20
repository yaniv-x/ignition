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
        if (device->id != hd_id || !ata_is_hd(device)) {
            continue;
        }

        if (!ata_hd_read(device, 0, 1, FAR_POINTER(uint8_t, 0, 0x7c00))) {
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
        INT(0x18);
        break;
    }

    D_MESSAGE("not on device list");
    freeze();
}


#define CD_BOOT_RECORD_SECTOR 0x11
#define CD_BOOT_SEG 0x8000


typedef _Packed struct CDBootRecord {
    uint8_t indicator; // = 0
    uint8_t identifier[5]; // = "CD001"
    uint8_t version; // = 1
    uint8_t boot_sys_indentifier[32]; // = "EL TORITO SPECIFICATION" zero padded
    uint8_t _0[32]; // mbz
    uint32_t boot_catalog_sector;
    uint8_t _1[1973]; // mbz
} CDBootRecord;


typedef _Packed struct ValidationEntry {
    uint8_t header_id; // = 1
    uint8_t platform_id; // = 0 for 80x86
    uint8_t _0[2]; // mbz
    uint8_t id_string[24];
    uint16_t checksum;
    uint16_t signature; // = 0xaa55
} ValidationEntry;


typedef _Packed struct BootEntry {
    uint8_t indicator; // = 0x88
    uint8_t media_type; // = 0 no-emulation; = 4 hard disk 0x80
    uint16_t load_segment; // if = 0 then 0x7c0
    uint8_t system_type; // = byte 5 of the boot record
    uint8_t _0; // mbz
    uint16_t sectors_count;
    uint32_t start_sector;
    uint8_t _1[20]; //mbz
} BootEntry;


static bool_t cd_boot_prepare(ATADevice __far * device)
{
    if (!ata_cdrom_prevent_removal(device)) {
        D_MESSAGE("prevent removal failed");
        return FALSE;
    }

    if (!ata_cdrom_start(device)) {
        D_MESSAGE("no media");
        ata_cdrom_allow_removal(device);
        return FALSE;
    }

    return TRUE;
}


static bool_t cd_boot_find_catalog(ATADevice __far * device, uint32_t __far * catalog_sector)
{
    CDBootRecord __far * boot_record = FAR_POINTER(CDBootRecord, CD_BOOT_SEG, 0);

    if (ata_cdrom_read(device, CD_BOOT_RECORD_SECTOR, 1, boot_record) != 1) {
        D_MESSAGE("read boot record failed");
        return FALSE;
    }

    if (boot_record->indicator != 0) {
        D_MESSAGE("bad indicator 0x%x", boot_record->indicator);
        return FALSE;
    }

    if (string_cmp_n(boot_record->identifier, "CD001", sizeof(boot_record->identifier))) {
        D_MESSAGE("bad identifier %x-%x-%x-%x-%x"
                  , boot_record->identifier[0]
                  , boot_record->identifier[1]
                  , boot_record->identifier[2]
                  , boot_record->identifier[3]
                  , boot_record->identifier[4]);
        return FALSE;
    }

    if (boot_record->version != 1) {
        D_MESSAGE("bad version 0x%x", boot_record->version);
        return FALSE;
    }

    if (string_cmp(boot_record->boot_sys_indentifier, "EL TORITO SPECIFICATION")) {
        D_MESSAGE("bad sys_indentifier %S", boot_record->boot_sys_indentifier);
        return FALSE;
    }

    D_MESSAGE("boot catalog @ 0x%lx (%S)",
              boot_record->boot_catalog_sector,
              boot_record->boot_sys_indentifier);

    *catalog_sector = boot_record->boot_catalog_sector;
    return TRUE;
}


static bool_t cd_boot_validate_catalog(ValidationEntry __far * validation)
{
    if (validation->header_id != 1) {
        D_MESSAGE("bad header ID 0x%x", validation->header_id);
        return FALSE;
    }

    if (validation->platform_id != 0) {
        D_MESSAGE("platform is not 80x86 0x%x", validation->platform_id);
        return FALSE;
    }

    if (validation->signature != 0xaa55) {
        D_MESSAGE("bad signature 0x%x", validation->signature);
        return FALSE;
    }

    if (checksum16(validation, sizeof(*validation) / 2) != 0) {
        D_MESSAGE("checksum failed");
        return FALSE;
    }

    D_MESSAGE("validation str \"%S\"", validation->id_string);
    return TRUE;
}


static void do_cd_boot(ATADevice __far * device, uint32_t catalog_sector)
{
    ValidationEntry __far * validation;
    BootEntry __far * boot_entry;
    void __far * catalog;
    uint16_t num_sectors;
    uint16_t code_seg;
    uint8_t id;

    catalog = FAR_POINTER(CDBootRecord, CD_BOOT_SEG, 0);

    if (ata_cdrom_read(device, catalog_sector, 1, catalog) != 1) {
        D_MESSAGE("read catalog @ 0x%lx failed", catalog_sector);
        return;
    }

    validation = catalog;

    if (!cd_boot_validate_catalog(validation)) {
        return;
    }

    boot_entry = (BootEntry __far *)(validation + 1);

    if (boot_entry->indicator != 0x88) {
        D_MESSAGE("not bootable 0x%x", boot_entry->indicator);
        return;
    }

    if (boot_entry->media_type != 0) {
        D_MESSAGE("emulation 0x%x is not supported", boot_entry->media_type);
        return;
    }

    code_seg = boot_entry->load_segment ? boot_entry->load_segment : 0x7c0;
    num_sectors = (boot_entry->sectors_count % 4) ? boot_entry->sectors_count / 4 + 1 :
                                                    boot_entry->sectors_count / 4;
    //todo: verfiy valid address

    if (ata_cdrom_read(device, boot_entry->start_sector, num_sectors,
                       FAR_POINTER(void, code_seg, 0)) != num_sectors) {
        D_MESSAGE("load failed");
        return;
    }

    ata_cdrom_allow_removal(device);

    id = device->id;

    __asm {
        mov dl, id
        push code_seg
        push 0
        jmp far_call_no_return
    }
}


static void cd_boot()
{
    ATADevice __far * device;
    uint8_t device_id;
    uint i;

    __asm { mov device_id, dl}

    D_MESSAGE("0x%x", device_id);

    device = FAR_POINTER(ATADevice, bda_read_word(BDA_OFFSET_EBDA), OFFSET_OF_PRIVATE(ata_devices));

    for ( i = 0; i < MAX_ATA_DEVICES; i++, device++) {
        if (device->id != device_id || !ata_is_cdrom(device)) {
            continue;
        }

        if (cd_boot_prepare(device)) {
            uint32_t catalog_sector;

            if (cd_boot_find_catalog(device, &catalog_sector)) {
                do_cd_boot(device, catalog_sector);
            }

            ata_cdrom_allow_removal(device);
        }

        INT(0x18);
    }

    D_MESSAGE("not on device list");
    freeze();
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
        options[i].user_data = device->id;
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
        options[i].user_data = device->id;
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

