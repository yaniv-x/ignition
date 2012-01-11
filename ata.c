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

#include "utils.h"
#include "pci.h"
#include "platform.h"
#include "bios.h"
#include "error_codes.h"

#define ATA_FLAGS_ATA (1 << 0)
#define ATA_FLAGS_ATAPI (1 << 1)
#define ATA_FLAGS_DMA (1 << 2)
#define ATA_FLAGS_48BIT (1 << 3)

#define ATA_PROGIF_PRIMARY_NOT_FIX (1 << 1)
#define ATA_PROGIF_SECONDARY_NOT_FIX (1 << 3)

#define ATA_0_LAGACY_IO_CMD 0x01f0
#define ATA_0_LAGACY_IO_CTRL 0x03f6
#define ATA_0_LAGACY_IRQ_LINE 14
#define ATA_1_LAGACY_IO_CMD 0x0170
#define ATA_1_LAGACY_IO_CTRL 0x0376
#define ATA_1_LAGACY_IRQ_LINE 15


#define ATA_CONTROL_DISABLE_INTERRUPT_MASK (1 << 1)
#define ATA_CONTROL_RESET_MASK (1 << 2)
#define ATA_CONTROL_HOB_MASK (1 << 7)

#define ATA_STATUS_BUSY_MASK (1 << 7)
#define ATA_STATUS_READY_MASK (1 << 6)
#define ATA_STATUS_DATA_REQUEST_MASK (1 << 3)
#define ATA_STATUS_ERROR_MASK (1 << 0)
#define ATA_STATUS_CHK_MASK ATA_STATUS_ERROR_MASK

#define ATA_DEVICE_LBA_MASK (1 << 6)
#define ATA_DEVICE_ADDRESS_MASK ((1 << 4) - 1)

#define ATA_IO_DATA 0
#define ATA_IO_ERROR 1
#define ATA_IO_FEATURE ATA_IO_ERROR
#define ATA_IO_SECTOR_COUNT 2
#define ATA_IO_REASON ATA_IO_SECTOR_COUNT
#define ATA_IO_LBA_LOW 3
#define ATA_IO_SECTOR_POS ATA_IO_LBA_LOW
#define ATA_IO_LBA_MID 4
#define ATA_IO_CYL_LOW ATA_ID_LBA_MID
#define ATA_IO_LBA_HIGH 5
#define ATA_IO_CYL_HIGH ATA_ID_LBA_HIGH
#define ATA_IO_DEVICE 6
#define ATA_IO_HEAD ATA_ID_DEVICE
#define ATA_IO_COMMAND 7
#define ATA_IO_STATUS ATA_IO_COMMAND

#define ATA_PIO_BLOCK_SIZE 512

#define ATA_ID_OFFSET_GENERAL_CONF 0
#define ATA_ID_GENERAL_CONF_NOT_ATA_MASK (1 << 15)
#define ATA_ID_GENERAL_ATAPI_MASK (0x3 << 14)
#define ATA_ID_GENERAL_ATAPI (0x2 << 14)
#define ATA_ID_GENERAL_CONF_REMOVABLE_MASK (1 << 7)
#define ATA_ID_GENERAL_CONF_INCOMPLETE (1 << 2)
#define ATA_ID_GENERAL_COMMAND_SET_SHIFT 8
#define ATA_ID_GENERAL_COMMAND_SET_BITS 5
#define ATA_ID_GENERAL_COMMAND_SET_CD 0x05
#define ATA_ID_GENERAL_CONF_PACKET_SIZE_MASM 0x03
#define ATA_COMPAT_ID_OFFSET_CYL 1
#define ATA_COMPAT_ID_OFFSET_HEAD 3
#define ATA_COMPAT_ID_OFFSET_SECTORS 6
#define ATA_ID_OFFSET_MODEL 27
#define ATA_ID_MODEL_NUM_CHARS 40
#define ATA_ID_OFFSET_CAP1 49
#define ATA_ID_CAP1_LBA_MASK (1 << 9)
#define ATA_ID_CAP1_DMA_MASK (1 << 8)
#define ATA_ID_OFFSET_VERSION 80
#define ATA_ID_VERSION_ATA6_MASK (1 << 6)
#define ATA_ID_OFFSET_ADDRESABEL_SECTORS 60
#define ATA_ID_OFFSET_CMD_SET_2 83
#define ATA_ID_OFFSET_CMD_SET_2_ENABLE 86
#define ATA_ID_CMD_SET_2_48BIT_MASK (1 << 10)
#define ATA_ID_OFFSET_ADDR_SECTORS_48 100

#define ATA_CMD_READ_SECTORS 0x20
#define ATA_CMD_WRITE_SECTORS 0x30
#define ATA_CMD_READ_SECTORS_EXT 0x24
#define ATA_CMD_WRITE_SECTORS_EXT 0x34
#define ATA_CMD_IDENTIFY_DEVICE 0xec
#define ATA_CMD_PACKET 0xa0
#define ATA_CMD_IDENTIFY_PACKET_DEVICE 0xa1

#define SOFT_REST_TIMEOUT_MS 5000
#define INVALID_INDEX ~0
#define BAD_ADDRESS ~0UL
#define SECTORS_PER_TRACK 63
#define MAX_HEADS 255
#define MAX_CYLINDERS 1024
#define MAX_SECTORS ((uint32_t)MAX_CYLINDERS * MAX_HEADS * SECTORS_PER_TRACK)
#define HD_SECTOR_SIZE 512
#define CD_SECTOR_SIZE 2048
#define MAX_HD 0x70 // id 0-0x6f, 0x7f - 0x70 are reserved for cd/dvd drives.
                    // in reality max hd is limit by MAX_ATA_DEVICES

#define ATAPI_PACKET_SIZE_MAX 16
#define ATAPI_PIO_MAX_TRANSFER 0xfffe
#define ATA_REASON_SENSE_SHIFT 4
#define ATA_REASON_CD_MASK (1 << 0)
#define ATA_REASON_IO_MASK (1 << 1)
#define ATA_REASON_REL_MASK (1 << 2)

#define SCSI_CMD_TEST_UNIT_READY 0x00
#define SCSI_CMD_REQUEST_SENSE 0x03

#define MMC_CMD_START_STOP 0x1b
#define MMC_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL 0x1e
#define MMC_CMD_READ 0x28

#define MMC_PREVENT_UNLOCK 0
#define MMC_PREVENT_LOCK 1
#define MMC_PREVENT_PERSISTENT_ALLOW 2
#define MMC_PREVENT_PERSISTENT_PREVENT 3

#define SCSI_SENSE_UNIT_ATTENTION 0x06


#define CURRENT_REG 0
#define PREVIOUS_REG 1


typedef struct PIOCmd {
    uint8_t regs[8];
    uint16_t blocks;
    uint16_t __far * buf;
} PIOCmd;


typedef struct PIOCmdExt {
    uint8_t regs[6][2];
    uint8_t device;
    uint8_t command;
    uint16_t blocks;
    uint16_t __far * buf;
} PIOCmdExt;


typedef _Packed struct FixDiskParamTable{ /*translated*/
    uint16_t logi_cylinders;
    uint8_t logi_heads;
    uint8_t signature;
    uint8_t physi_sec_per_track;
    uint16_t precompensation;
    uint8_t _3;
    uint8_t drive_control_byte;
    uint16_t physi_cylinders;
    uint8_t physi_heads;
    uint16_t lending_zone;
    uint8_t logi_sec_per_track;
    uint8_t checksum;
} FixDiskParamTable;


typedef _Packed struct HDCmosParam {
    uint16_t cylinders;
    uint8_t heads;
    uint16_t precompensation;
    uint8_t control;
    uint16_t landing_zone;
    uint8_t sectors;
} HDCmosParam;


void int13_handler();
void int13_fd_emulate_handler();
void int13_hd_emulate_handler();


static EmulatedDev __far * get_emulated()
{
    uint16_t ebda_seg = bda_read_word(BDA_OFFSET_EBDA);
    return FAR_POINTER(EmulatedDev, ebda_seg, OFFSET_OF_PRIVATE(emulated_dev));
}


static void wait_ready(uint16_t cmd_port, uint16_t ctrl_port, uint8_t device_reg)
{
    while ((inb(ctrl_port) & ATA_STATUS_BUSY_MASK)) {
        D_MESSAGE("unexpecte busy");
        delay(1000);
    }

    outb(cmd_port + ATA_IO_DEVICE, device_reg);

    while ((inb(ctrl_port) & (ATA_STATUS_BUSY_MASK | ATA_STATUS_READY_MASK)) !=
                                                                        ATA_STATUS_READY_MASK) {
        D_MESSAGE("unexpecte");
        delay(1000);
    }
}


static bool_t is_busy(uint16_t alt_port)
{
    return !!(inb(alt_port) & ATA_STATUS_BUSY_MASK);
}


static uint ata_pio_in(uint16_t cmd_port, uint16_t ctrl_port, uint16_t blocks,
                       uint16_t __far * dest)
{
    uint8_t status;
    uint i = 0;

    for (i = 0; i < blocks; i++) {
        for (;;) {
            STI();
            HALT();
            CLI();

            if (!is_busy(ctrl_port)) {
                break;
            }
        }

        status = inb(ctrl_port);

        if (!(status & ATA_STATUS_DATA_REQUEST_MASK)) {
            D_MESSAGE("expecting DRQ"); //error
            break;
        }

        in_words(cmd_port, dest, ATA_PIO_BLOCK_SIZE / 2);
        dest += ATA_PIO_BLOCK_SIZE / 2;
    }

    return i;
}


static uint ata_pio_out(uint16_t cmd_port, uint16_t ctrl_port, uint16_t blocks,
                        uint16_t __far * src)
{
    uint8_t status;
    int i = 0;

    for (;; i++) {
        for (;;) {
            STI();
            HALT();
            CLI();

            if (!is_busy(ctrl_port)) {
                break;
            }
        }

        if (i == blocks) {
            break;
        }

        status = inb(ctrl_port);

        if (!(status & ATA_STATUS_DATA_REQUEST_MASK)) {
            D_MESSAGE("expecting DRQ"); //error
            break;
        }

        out_words(cmd_port, src, ATA_PIO_BLOCK_SIZE / 2);
        src += ATA_PIO_BLOCK_SIZE / 2;
    }

    return i;
}


typedef uint (*ata_specific_handler_t)(uint16_t cmd_port, uint16_t ctrl_port, uint16_t blocks,
                                       uint16_t __far * src);

static uint ata_pio_cmd(uint16_t cmd_port, uint16_t ctrl_port, PIOCmd __far * cmd,
                        ata_specific_handler_t handler)
{
    uint i;

    NO_INTERRUPT();

    wait_ready(cmd_port, ctrl_port, cmd->regs[ATA_IO_DEVICE]);

    for (i = ATA_IO_FEATURE; i < ATA_IO_DEVICE; i++) {
        outb(cmd_port + i, cmd->regs[i]);
    }

    outb(cmd_port + ATA_IO_COMMAND, cmd->regs[ATA_IO_COMMAND]);

    return handler(cmd_port, ctrl_port, cmd->blocks, cmd->buf);
}


static uint ata_pio_cmd_ext(uint16_t cmd_port, uint16_t ctrl_port, PIOCmdExt __far * cmd,
                            ata_specific_handler_t handler)
{
    uint i;

    NO_INTERRUPT();

    wait_ready(cmd_port, ctrl_port, cmd->device);

    for (i = ATA_IO_FEATURE; i < ATA_IO_DEVICE; i++) {
        outb(cmd_port + i, cmd->regs[i][PREVIOUS_REG]);
        outb(cmd_port + i, cmd->regs[i][CURRENT_REG]);
    }

    outb(cmd_port + ATA_IO_COMMAND, cmd->command);

    return handler(cmd_port, ctrl_port, cmd->blocks, cmd->buf);
}


static void identify_device(uint16_t cmd_port, uint16_t ctrl_port, uint8_t id_cmd,
                            uint16_t __far * buf)
{
    uint16_t save_word;
    PIOCmd cmd;
    uint i;

    mem_set(&cmd, 0, sizeof(cmd));
    cmd.blocks = 1;
    cmd.buf = buf;
    cmd.regs[ATA_IO_COMMAND] = id_cmd;

    if (ata_pio_cmd(cmd_port, ctrl_port, &cmd, ata_pio_in) != 1) {
        D_MESSAGE("id command failed");
        bios_error(BIOS_ERROR_ATA_ID_FAILED);
    }
}


static bool_t init_common(ATADevice __far * device, uint16_t __far * identity)
{
    uint i;

    if ((identity[ATA_ID_OFFSET_GENERAL_CONF] & ATA_ID_GENERAL_CONF_INCOMPLETE)) {
        D_MESSAGE("incomplete");
        return FALSE;
    }

    if (!(identity[ATA_ID_OFFSET_VERSION] & ATA_ID_VERSION_ATA6_MASK)) {
        D_MESSAGE("not ATA-6");
        return FALSE;
    }

    if ((identity[ATA_ID_OFFSET_CAP1] & ATA_ID_CAP1_DMA_MASK)) {
        device->flags |= ATA_FLAGS_DMA;
    }

    for (i = 0; i < ATA_DESCRIPTION_MAX; i += 2) {
        device->description[i] = identity[ATA_ID_OFFSET_MODEL + (i >> 1)] >> 8;
        device->description[i + 1] = identity[ATA_ID_OFFSET_MODEL + (i >> 1)];
    }

    device->description[ATA_DESCRIPTION_MAX] = 0;

    return TRUE;
}


static void init_translat_params(ATADevice __far * device)
{
    uint32_t sectors = MIN(device->sectors, MAX_SECTORS);
    uint32_t heads = sectors / SECTORS_PER_TRACK / MAX_CYLINDERS;

    if (heads > 128) {
        heads = 255;
    } else if (heads > 64) {
        heads = 128;
    } else if (heads > 32) {
        heads = 64;
    } else if (heads > 16) {
        heads = 32;
    } else  {
        heads = 16;
    }

    device->logi_heads = heads;
    device->logi_cylinders = sectors / SECTORS_PER_TRACK / heads;
}


static bool_t init_hd_params(ATADevice __far * device, uint16_t __far * identity)
{
    FixDiskParamTable __far * fdpt = NULL;
    uint8_t hd_id;

    if ((identity[ATA_ID_OFFSET_GENERAL_CONF] & ATA_ID_GENERAL_CONF_NOT_ATA_MASK)) {
        D_MESSAGE("not ATA");
        return FALSE;
    }

    device->flags |= ATA_FLAGS_ATA;

    if (!init_common(device, identity)) {
        return FALSE;
    }

    if (identity[ATA_ID_OFFSET_GENERAL_CONF] & ATA_ID_GENERAL_CONF_REMOVABLE_MASK) {
        D_MESSAGE("removable");
        return FALSE;
    }

    if (!(identity[ATA_ID_OFFSET_CAP1] & ATA_ID_CAP1_LBA_MASK)) {
        D_MESSAGE("no LBA");
        return FALSE;
    }

    if ((identity[ATA_ID_OFFSET_CMD_SET_2] & ATA_ID_CMD_SET_2_48BIT_MASK)) {

        ASSERT((identity[ATA_ID_OFFSET_CMD_SET_2_ENABLE] & ATA_ID_CMD_SET_2_48BIT_MASK)); //48BIT is
                                                                                          // fixed
        D_MESSAGE("48bit");
        device->flags |= ATA_FLAGS_48BIT;

        device->sectors = ((uint64_t)identity[ATA_ID_OFFSET_ADDR_SECTORS_48 + 2] << 32) |
                          ((uint64_t)identity[ATA_ID_OFFSET_ADDR_SECTORS_48 + 1] << 16) |
                          identity[ATA_ID_OFFSET_ADDR_SECTORS_48];
    } else {
        device->sectors = ((uint32_t)identity[ATA_ID_OFFSET_ADDRESABEL_SECTORS + 1] << 16) |
                          identity[ATA_ID_OFFSET_ADDRESABEL_SECTORS];
    }

    device->physi_heads = identity[ATA_COMPAT_ID_OFFSET_HEAD];
    device->physi_sec_per_track = identity[ATA_COMPAT_ID_OFFSET_SECTORS];
    device->physi_cylinders = identity[ATA_COMPAT_ID_OFFSET_CYL];

    init_translat_params(device);

    hd_id = bda_read_byte(BDA_OFFSET_HD_ATTACHED);

    if (hd_id == MAX_HD) {
        return FALSE;
    }

    bda_write_byte(BDA_OFFSET_HD_ATTACHED, hd_id + 1);
    // BDA_OFFSET_HD_CONTROL_BYTE ?

    device->id = hd_id | 0x80;

    if (hd_id == 0) {
        fdpt = FAR_POINTER(FixDiskParamTable, bda_read_word(BDA_OFFSET_EBDA), EBDA_OFFSET_FDPT_0);
        set_int_vec(0x41, bda_read_word(BDA_OFFSET_EBDA), EBDA_OFFSET_FDPT_0);
    } else if (hd_id == 1) {
        fdpt = FAR_POINTER(FixDiskParamTable, bda_read_word(BDA_OFFSET_EBDA), EBDA_OFFSET_FDPT_1);
        set_int_vec(0x46, bda_read_word(BDA_OFFSET_EBDA), EBDA_OFFSET_FDPT_1);
    }

    if (fdpt) {
        HDCmosParam cmos_param;

        ebda_write_byte(EBDA_OFFSET_HD_COUNT, ebda_read_word(EBDA_OFFSET_HD_COUNT) + 1);

        fdpt->logi_cylinders = device->logi_cylinders;
        fdpt->logi_heads = device->logi_heads;
        fdpt->signature = 0xa0;
        fdpt->physi_sec_per_track = device->physi_sec_per_track;
        fdpt->precompensation = 0xffff;
        fdpt->drive_control_byte = 0xc0 | (device->logi_heads > 8 ? (1 << 3) : 0);
        fdpt->physi_cylinders = device->physi_cylinders;
        fdpt->physi_heads = device->physi_heads;
        fdpt->lending_zone = 0xffff;
        fdpt->logi_sec_per_track = SECTORS_PER_TRACK;
        fdpt->checksum = checksum8(fdpt, sizeof(*fdpt));

        cmos_param.cylinders = fdpt->logi_cylinders;
        cmos_param.heads = fdpt->logi_heads;
        cmos_param.precompensation = fdpt->precompensation;
        cmos_param.control = fdpt->drive_control_byte;
        cmos_param.landing_zone = fdpt->lending_zone;
        cmos_param.sectors = fdpt->logi_sec_per_track;

        if (hd_id == 1) {
            rtc_write(CMOS_OFFSET_HD_TYPE, rtc_read(CMOS_OFFSET_HD_TYPE) | 0x0f);
            rtc_write(CMOS_OFFSET_HD1_EXT_TYPE, 47);
            rtc_write_buf(CMOS_OFFSET_HD1_PARAMS, &cmos_param, sizeof(cmos_param));
        } else {
            rtc_write(CMOS_OFFSET_HD_TYPE , rtc_read(CMOS_OFFSET_HD_TYPE) | 0xf0);
            rtc_write(CMOS_OFFSET_HD0_EXT_TYPE, 47);
            rtc_write_buf(CMOS_OFFSET_HD0_PARAMS, &cmos_param, sizeof(cmos_param));
        }
    }

    return TRUE;
}


static bool_t add_ata_device(ATADevice __far * device)
{
    uint16_t identity_buf[ATA_PIO_BLOCK_SIZE / 2];
    EBDA* ebda = 0;

    D_MESSAGE("0x%x 0x%x", device->cmd_port, device->ctrl_port);

    identify_device(device->cmd_port, device->ctrl_port, ATA_CMD_IDENTIFY_DEVICE,
                    identity_buf);

    if (!init_hd_params(device, identity_buf)) {
        return FALSE;
    }

    D_MESSAGE("%S", device->description);

    boot_add_hd(device);

    return TRUE;
}


static bool_t init_cdrom_params(ATADevice __far * device, uint16_t __far * identity)
{
    uint16_t command_set;
    uint8_t packet_size;
    uint8_t cd_id;

    if ((identity[ATA_ID_OFFSET_GENERAL_CONF] & ATA_ID_GENERAL_ATAPI_MASK)
                                                             != ATA_ID_GENERAL_ATAPI) {
        D_MESSAGE("not ATAPI");
        return FALSE;
    }

    device->flags |= ATA_FLAGS_ATAPI;

    if (!init_common(device, identity)) {
        return FALSE;
    }

    if (!(identity[ATA_ID_OFFSET_GENERAL_CONF] & ATA_ID_GENERAL_CONF_REMOVABLE_MASK)) {
        D_MESSAGE("not removable");
        return FALSE;
    }

    command_set = identity[ATA_ID_OFFSET_GENERAL_CONF] >> ATA_ID_GENERAL_COMMAND_SET_SHIFT;
    command_set &= ((1 << ATA_ID_GENERAL_COMMAND_SET_BITS) - 1);

    if (command_set != ATA_ID_GENERAL_COMMAND_SET_CD) {
        D_MESSAGE("unsupported command set 0x%x", command_set);
        return FALSE;
    }

    packet_size = identity[ATA_ID_OFFSET_GENERAL_CONF] & ATA_ID_GENERAL_CONF_PACKET_SIZE_MASM;

    if (packet_size == 0) {
        device->u.cd.packet_size = 12;
    } else if (packet_size == 1) {
        device->u.cd.packet_size = 16;
    } else {
        D_MESSAGE("invalid packet size");
        return FALSE;
    }

    cd_id = ebda_read_byte(OFFSET_OF_PRIVATE(next_cd_id));

    if (cd_id > 0x0f) {
        return FALSE;
    }

    device->id = 0xf0 | cd_id;
    ebda_write_byte(OFFSET_OF_PRIVATE(next_cd_id), cd_id + 1);

    return TRUE;
}


static bool_t add_atapi_device(ATADevice __far * device)
{
    uint16_t identity_buf[ATA_PIO_BLOCK_SIZE / 2];
    EBDA* ebda = 0;

    D_MESSAGE("0x%x 0x%x ", device->cmd_port, device->ctrl_port);

    identify_device(device->cmd_port, device->ctrl_port, ATA_CMD_IDENTIFY_PACKET_DEVICE,
                    identity_buf);

    if (!init_cdrom_params(device, identity_buf)) {
        return FALSE;
    }

    D_MESSAGE("%S", device->description);

    boot_add_cd(device);

    return TRUE;
}


static bool_t reg_test(uint16_t port)
{
    outb(port + ATA_IO_LBA_LOW, 0xb3);
    outb(port + ATA_IO_LBA_MID, 0xa8);

    return inb(port + ATA_IO_LBA_LOW) == 0xb3 && inb(port + ATA_IO_LBA_MID) == 0xa8;
}


void ata_int_cb(uint device_id)
{
    EBDA* ebda = 0;
    ATADevice* device = &ebda->private.ata_devices[device_id];

    inb(device->cmd_port + ATA_IO_STATUS);
}


static bool_t _reset_device(uint16_t cmd_port, uint16_t ctrl_port, uint8_t __far regs[])
{
    uint wait_time = 0;
    uint i;

    outb(ctrl_port, ATA_CONTROL_RESET_MASK);
    delay(1); // need to wait 5 micro
    outb(ctrl_port, 0);
    delay(2); // need to wait 2 mili

    while ((inb(ctrl_port) & ATA_STATUS_BUSY_MASK)) {
        if (wait_time >= SOFT_REST_TIMEOUT_MS) {
            bios_warn(BIOS_WARN_ATA_RESET_TIMEOUT);
            return FALSE;
        }

        delay(100);
        wait_time += 100;
    }

    for (i = 1; i <= ATA_IO_STATUS; i++) {
        regs[i] = inb(cmd_port + i);
    }

    return TRUE;
}


static uint prob_device(uint16_t cmd_port, uint16_t ctrl_port, uint line)
{
    ATADevice __far * device_info;
    uint8_t regs[8];
    EBDA *ebda = 0;
    bool_t atapi;
    uint16_t seg;
    bool_t ok;
    uint i;

    // in linux the first step is writing and reading from the device regs in order to detect
    // the preset of a device. according to ata-6 spec "The contents of this register are valid
    // only when BSY and DRQ are cleared to zero. If this register is written when BSY or DRQ
    // is set to one, the result is indeterminate"
    //
    // taking the above into account, first perform soft reset and only if BSY is clear read
    // the regs (according to my tests on physical machine, the BSY flag is always clear for
    // unused ports)

    if (!_reset_device(cmd_port, ctrl_port, regs)) {
        D_MESSAGE("mulfunction device/controller or no device @ 0x%x, 0x%x",
                  cmd_port, ctrl_port);
        return INVALID_INDEX;
    }

    if (!reg_test(cmd_port)) {
        D_MESSAGE("no device @ 0x%x, 0x%x", cmd_port, ctrl_port);
        return INVALID_INDEX;
    }

    if ((regs[ATA_IO_STATUS] & ATA_STATUS_ERROR_MASK)) {
        D_MESSAGE("mulfunction @ 0x%x, 0x%x", cmd_port, ctrl_port);
        bios_error(BIOS_ERROR_ATA_MALFUNCTION);
    }

    if (regs[ATA_IO_SECTOR_COUNT] != 1 || regs[ATA_IO_LBA_LOW] != 1) {
        D_MESSAGE("invalid signature @0x%x,0x%x", cmd_port, ctrl_port);
        bios_error(BIOS_ERROR_ATA_INVALID_SIGNATURE);
    }

    if (regs[ATA_IO_LBA_MID] == 0 && regs[ATA_IO_LBA_HIGH] == 0) {
        atapi = FALSE;
    } else if (regs[ATA_IO_LBA_MID] == 0x14 && regs[ATA_IO_LBA_HIGH] == 0xeb) {
        atapi = TRUE;
    } else {
        D_MESSAGE("invalid signature @0x%x,0x%x", cmd_port, ctrl_port);
        bios_error(BIOS_ERROR_ATA_INVALID_SIGNATURE);
    }

    if (regs[ATA_IO_ERROR] != 1) {
        D_MESSAGE("diagnostic failed (0x%x, 0x%x)", cmd_port, ctrl_port);
        bios_error(BIOS_ERROR_ATA_DIAGNOSTIC_FAILED);
    }

    seg = bda_read_word(BDA_OFFSET_EBDA);
    device_info = FAR_POINTER(ATADevice, seg, OFFSET_OF_PRIVATE(ata_devices));

    for (i = 0; i < MAX_ATA_DEVICES && device_info->cmd_port; i++, device_info++) ;

    if (i == MAX_ATA_DEVICES) {
        D_MESSAGE("out of ata slot");
        bios_warn(BIOS_WARN_ATA_OUT_OF_SLOT);
        return INVALID_INDEX;
    }

    device_info->cmd_port = cmd_port;
    device_info->ctrl_port = ctrl_port;

    register_interrupt_handler(line, ata_int_cb, i);

    if (atapi) {
        ok = add_atapi_device(device_info);
    } else {
        ok = add_ata_device(device_info);
    }

    if (!ok) {
        unregister_interrupt_handler(line, ata_int_cb, i);
        mem_reset(device_info, sizeof(*device_info));
        return INVALID_INDEX;
    }

    return i;
}


static void update_bm_state(ATADevice __far * device, bool_t primary)
{
    uint pci_bus = device->pci_bus;
    uint pci_device = device->pci_device;
    uint16_t bm_io = pci_read_32(pci_bus, pci_device, PCI_OFFSET_BAR_4) & PCI_BAR_IO_ADDRESS_MASK;
    uint16_t offset = primary ? 0x02 : 0x0a;
    bool_t dma = !!(device->flags & ATA_FLAGS_DMA);
    uint8_t val = (dma) ? (1 << 5) : 0;

    ASSERT(bm_io);
    outb(bm_io + offset, val);

    D_MESSAGE("%s dma is %s", primary ? "primary" : "secondary", dma ? "on" : "off");
}


static void set_pci_stuff(uint slot, uint pci_bus, uint pci_device, uint pci_func, uint prog_if,
                             bool_t primary)
{
    ATADevice __far * device;
    uint16_t seg;

    seg = bda_read_word(BDA_OFFSET_EBDA);
    device = FAR_POINTER(ATADevice, seg, OFFSET_OF_PRIVATE(ata_devices[slot]));

    device->pci_bus = pci_bus;
    device->pci_device = pci_device;
    device->pci_func = pci_func;

    if ((prog_if & 0x80)) {
        update_bm_state(device, primary);
    }
}


static void init_ide(uint bus, uint device, uint function, uint prog_if)
{
    uint16_t primary_cmd_port;
    uint16_t primary_ctrl_port;
    uint16_t secondary_cmd_port;
    uint16_t secondary_ctrl_port;
    uint8_t primary_line;
    uint8_t secondary_line;
    uint slot;

    if ((prog_if & ATA_PROGIF_PRIMARY_NOT_FIX)) {
        primary_cmd_port = pci_read_32(bus, device, PCI_OFFSET_BAR_0) & PCI_BAR_IO_ADDRESS_MASK;
        primary_ctrl_port = pci_read_32(bus, device, PCI_OFFSET_BAR_1) & PCI_BAR_IO_ADDRESS_MASK;
        primary_line = pci_read_8(bus, device, PCI_OFFSET_INTERRUPT_LINE);
    } else {
        primary_cmd_port = ATA_0_LAGACY_IO_CMD;
        primary_ctrl_port = ATA_0_LAGACY_IO_CTRL;
        primary_line = ATA_0_LAGACY_IRQ_LINE;
    }

    if ((prog_if & ATA_PROGIF_PRIMARY_NOT_FIX)) {
        secondary_cmd_port = pci_read_32(bus, device, PCI_OFFSET_BAR_2) & PCI_BAR_IO_ADDRESS_MASK;
        secondary_ctrl_port = pci_read_32(bus, device, PCI_OFFSET_BAR_3) & PCI_BAR_IO_ADDRESS_MASK;
        secondary_line = pci_read_8(bus, device, PCI_OFFSET_INTERRUPT_LINE);
    } else {
        secondary_cmd_port = ATA_1_LAGACY_IO_CMD;
        secondary_ctrl_port = ATA_1_LAGACY_IO_CTRL;
        secondary_line = ATA_1_LAGACY_IRQ_LINE;
    }

    if (primary_cmd_port) {
        slot = prob_device(primary_cmd_port, primary_ctrl_port, primary_line);

        if (slot != INVALID_INDEX) {
            set_pci_stuff(slot, bus, device, function, prog_if, TRUE);
        }
    }

    if (secondary_cmd_port) {
        slot = prob_device(secondary_cmd_port, secondary_ctrl_port, secondary_line);

        if (slot != INVALID_INDEX) {
            set_pci_stuff(slot, bus, device, function, prog_if, FALSE);
        }
    }
}


static int ata_pci_cb(uint bus, uint device, void __far * opaque)
{
    PCIDeviceType type;

    pci_get_class(bus, device, &type);

    if (type.class != PCI_CLASS_MASS_STORAGE || type.sub_class != PCI_MASS_STORAGE_SUBCLASS_IDE) {
        return FALSE;
    }

    if (!pci_is_io_enabled(bus, device)) {
        return FALSE;
    }

    init_ide(bus, device, 0, type.prog_if);

    return FALSE;
}


static ATADevice __far * find_hd(uint8_t id)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    ATADevice __far * device = FAR_POINTER(ATADevice, seg, OFFSET_OF_PRIVATE(ata_devices));
    uint i;

    for (i = 0; i < MAX_ATA_DEVICES; i++, device++) {
        if (ata_is_hd(device) && device->id == id) {
            return device;
        }
    }

    return NULL;
}


static ATADevice __far * find_device(uint8_t id)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    ATADevice __far * device = FAR_POINTER(ATADevice, seg, OFFSET_OF_PRIVATE(ata_devices));
    uint i;

    for (i = 0; i < MAX_ATA_DEVICES; i++, device++) {
        if (device->id == id) {
            return id ? device : NULL;
        }
    }

    return NULL;
}


static bool_t reset_device(ATADevice __far * device)
{
    uint8_t regs[8];
    uint wait_time = 0;

    if (!_reset_device(device->cmd_port, device->ctrl_port, regs)) {
        D_MESSAGE("timeout");
        bios_warn(BIOS_WARN_ATA_RESET_FAILED);
        return FALSE;
    }

    if ((regs[ATA_IO_STATUS] & ATA_STATUS_ERROR_MASK)) {
        D_MESSAGE("error");
        bios_warn(BIOS_WARN_ATA_RESET_FAILED);
        return FALSE;
    }

    if (regs[ATA_IO_SECTOR_COUNT] != 1 || regs[ATA_IO_LBA_LOW] != 1) {
        D_MESSAGE("bad signature");
        bios_warn(BIOS_WARN_ATA_RESET_FAILED);
        return FALSE;
    }

    if (regs[ATA_IO_LBA_MID] == 0 && regs[ATA_IO_LBA_HIGH] == 0) {
        if (!ata_is_hd(device)) {
            D_MESSAGE("mismatch");
            bios_error(BIOS_ERROR_ATA_TYPE_MISMATCH);
        }
    } else if (regs[ATA_IO_LBA_MID] == 0x14 && regs[ATA_IO_LBA_HIGH] == 0xeb) {
       if (!ata_is_cdrom(device)) {
            D_MESSAGE("mismatch");
            bios_error(BIOS_ERROR_ATA_TYPE_MISMATCH);
        }
    } else {
        D_MESSAGE("bad signature");
        bios_warn(BIOS_WARN_ATA_RESET_FAILED);
        return FALSE;
    }

    if (regs[ATA_IO_ERROR] != 1) {
        D_MESSAGE("diagnostic failed");
        bios_warn(BIOS_WARN_ATA_RESET_FAILED);
        return FALSE;
    }

    return TRUE;
}


static uint ata_hd_write(ATADevice __far * device, uint64_t address, uint count,
                         uint8_t __far * src)
{
    ASSERT(count > 0 && count <= 256);
    ASSERT(address + count > address && address + count <= device->sectors);

    if (address + count <= (1UL << 28) - 1) {
        PIOCmd cmd;

        mem_set(&cmd, 0, sizeof(cmd));
        cmd.blocks = count;
        cmd.buf = (uint16_t __far *)src;

        cmd.regs[ATA_IO_SECTOR_COUNT] = (count == 256) ? 0 : count;
        cmd.regs[ATA_IO_LBA_LOW] = address;
        cmd.regs[ATA_IO_LBA_MID] = address >> 8;
        cmd.regs[ATA_IO_LBA_HIGH] = address >> 16;
        cmd.regs[ATA_IO_DEVICE] = ATA_DEVICE_LBA_MASK | ((address >> 24) & ATA_DEVICE_ADDRESS_MASK);
        cmd.regs[ATA_IO_COMMAND] = ATA_CMD_WRITE_SECTORS;

        return ata_pio_cmd(device->cmd_port, device->ctrl_port, &cmd, ata_pio_out);
    } else {
        PIOCmdExt cmd;

        mem_set(&cmd, 0, sizeof(cmd));
        cmd.blocks = count;
        cmd.buf = (uint16_t __far *)src;

        cmd.regs[ATA_IO_SECTOR_COUNT][CURRENT_REG] = count;
        cmd.regs[ATA_IO_LBA_LOW][PREVIOUS_REG] = address >> 40;
        cmd.regs[ATA_IO_LBA_MID][PREVIOUS_REG] = address >> 32;
        cmd.regs[ATA_IO_LBA_LOW][PREVIOUS_REG] = address >> 24;
        cmd.regs[ATA_IO_LBA_HIGH][CURRENT_REG] = address >> 16;
        cmd.regs[ATA_IO_LBA_MID][CURRENT_REG] = address >> 8;
        cmd.regs[ATA_IO_LBA_LOW][CURRENT_REG] = address;
        cmd.device = ATA_DEVICE_LBA_MASK;
        cmd.command = ATA_CMD_WRITE_SECTORS_EXT;

        return ata_pio_cmd_ext(device->cmd_port, device->ctrl_port, &cmd, ata_pio_out);
    }
}


static bool_t transmit_packet_command(ATADevice __far * device, void __far * command_data,
                                      uint16_t max_transfer)
{
    uint8_t status;
    uint8_t reason;
    uint i;

    wait_ready(device->cmd_port, device->ctrl_port, 0);

    for (i = ATA_IO_FEATURE; i < ATA_IO_LBA_MID; i++) {
        outb(device->cmd_port + i, 0);
    }

    outb(device->cmd_port + ATA_IO_LBA_MID, max_transfer);
    outb(device->cmd_port + ATA_IO_LBA_HIGH, max_transfer >> 8);
    outb(device->cmd_port + ATA_IO_COMMAND, ATA_CMD_PACKET);

    while (((status = inb(device->ctrl_port)) & ATA_STATUS_BUSY_MASK)); //todo: timout

    if ((status & ATA_STATUS_CHK_MASK) || !(status & ATA_STATUS_DATA_REQUEST_MASK)) {
        D_MESSAGE("failed on status 0x%x", status);
        return FALSE;
    }

    reason = inb(device->cmd_port + ATA_IO_REASON);

    if ((reason & (ATA_REASON_REL_MASK | ATA_REASON_IO_MASK)) || !(reason & ATA_REASON_CD_MASK)) {
        D_MESSAGE("failed on reason 0x%x", reason);
        return FALSE;
    }

    out_words(device->cmd_port, command_data, device->u.cd.packet_size / 2);

    for (;;) {
        STI();
        HALT();
        CLI();

        if (!is_busy(device->ctrl_port)) {
            break;
        }
    }

    return TRUE;
}


static void request_sense(ATADevice __far * device)
{
    uint8_t command_data[ATAPI_PACKET_SIZE_MAX];

    mem_reset(command_data, sizeof(command_data));
    command_data[0] = SCSI_CMD_REQUEST_SENSE;

    if (!transmit_packet_command(device, command_data, 0) ||
                                               (inb(device->ctrl_port) & ATA_STATUS_CHK_MASK)) {
        D_MESSAGE("failed");
    }
}


static bool_t send_packet_command(ATADevice __far * device, void __far * command_data,
                                  uint16_t max_transfer)
{
    for (;;) {
        if (!transmit_packet_command(device, command_data, max_transfer)) {
            return FALSE;
        }

        if ((inb(device->ctrl_port) & ATA_STATUS_CHK_MASK)) {
            uint8_t err = inb(device->cmd_port + ATA_IO_ERROR);

            if ((err >> ATA_REASON_SENSE_SHIFT) == SCSI_SENSE_UNIT_ATTENTION) {
                request_sense(device);
                continue;
            }
        }

        break;
    }

    return TRUE;
}


bool_t ata_cdrtom_test_ready(ATADevice __far * device)
{
    uint8_t command_data[ATAPI_PACKET_SIZE_MAX];

    mem_reset(command_data, sizeof(command_data));
    command_data[0] = SCSI_CMD_TEST_UNIT_READY;

    if (!send_packet_command(device, command_data, 0)) {
        return FALSE;
    }

    return !(inb(device->ctrl_port) & ATA_STATUS_CHK_MASK);
}


bool_t ata_cdrom_prevent_removal(ATADevice __far * device)
{
    uint8_t command_data[ATAPI_PACKET_SIZE_MAX];

    mem_reset(command_data, sizeof(command_data));
    command_data[0] = MMC_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL;
    command_data[4] = MMC_PREVENT_PERSISTENT_PREVENT;

    if (!send_packet_command(device, command_data, 0)) {
        return FALSE;
    }

    return !(inb(device->ctrl_port) & ATA_STATUS_CHK_MASK);
}


bool_t ata_cdrom_allow_removal(ATADevice __far * device)
{
    uint8_t command_data[ATAPI_PACKET_SIZE_MAX];

    mem_reset(command_data, sizeof(command_data));
    command_data[0] = MMC_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL;
    command_data[4] = MMC_PREVENT_PERSISTENT_ALLOW;

    if (!send_packet_command(device, command_data, 0)) {
        return FALSE;
    }

    return !(inb(device->ctrl_port) & ATA_STATUS_CHK_MASK);
}


bool_t ata_cdrom_start(ATADevice __far * device)
{
    uint8_t command_data[ATAPI_PACKET_SIZE_MAX];

    mem_reset(command_data, sizeof(command_data));
    command_data[0] = MMC_CMD_START_STOP;
    command_data[4] = 0x3; // load and start and make it ready

    if (!send_packet_command(device, command_data, 0)) {
        return FALSE;
    }

    return !(inb(device->ctrl_port) & ATA_STATUS_CHK_MASK);
}


static uint16_t ata_recive(ATADevice __far * device, uint16_t count, uint8_t __far * dest)
{
    uint32_t n = (uint32_t)count * CD_SECTOR_SIZE;
    uint16_t trans_size;
    uint8_t status;
    uint8_t reason;

    do {
        status = inb(device->ctrl_port);

        if ((status & ATA_STATUS_CHK_MASK) || !(status & ATA_STATUS_DATA_REQUEST_MASK)) {
            D_MESSAGE("failed on status 0x%x", status);
            return ((uint32_t)count * CD_SECTOR_SIZE - n) / CD_SECTOR_SIZE;
        }

        reason = inb(device->cmd_port + ATA_IO_REASON);

        if ((reason & (ATA_REASON_CD_MASK | ATA_REASON_REL_MASK)) ||
                                                                 !(reason & ATA_REASON_IO_MASK)) {
            D_MESSAGE("failed on reason 0x%x", reason);
            return ((uint32_t)count * CD_SECTOR_SIZE - n) / CD_SECTOR_SIZE;
        }

        trans_size = inb(device->cmd_port + ATA_IO_LBA_HIGH);
        trans_size = (trans_size << 8) | inb(device->cmd_port + ATA_IO_LBA_MID);

        if (!trans_size || (trans_size & 1) || trans_size > ATAPI_PIO_MAX_TRANSFER ||
                                                                                 trans_size > n) {
            D_MESSAGE("failed on size 0x%x n 0x%lx", reason, n);
            return ((uint32_t)count * CD_SECTOR_SIZE - n) / CD_SECTOR_SIZE;
        }

        in_words(device->cmd_port, dest, trans_size / 2);
        n -= trans_size;
        dest += trans_size;

        for (;;) {
            STI();
            HALT();
            CLI();

            if (!is_busy(device->ctrl_port)) {
                break;
            }
        }

    } while (n);

    return count;
}


uint ata_cdrom_read(ATADevice __far * device, uint32_t sector, uint16_t count, void __far * dest)
{
    uint8_t command_data[ATAPI_PACKET_SIZE_MAX];
    uint8_t status;

    if (count == 0) {
        return 0;
    }

    mem_reset(command_data, sizeof(command_data));
    command_data[0] = MMC_CMD_READ;
    command_data[2] = sector >> 24;
    command_data[3] = sector >> 16;
    command_data[4] = sector >> 8;
    command_data[5] = sector;
    command_data[7] = count >> 8;
    command_data[8] = count;

    if (!send_packet_command(device, command_data, 0xfffe)) {
        return 0;
    }

    if ((inb(device->ctrl_port) & ATA_STATUS_CHK_MASK)) {
        return 0;
    }

    return ata_recive(device, count, dest);
}


uint ata_read(ATADevice __far * device, uint64_t address, uint count,
              uint8_t __far * dest)
{
    if (ata_is_hd(device)) {
        return ata_hd_read(device, address, count, dest);
    } else {
        return ata_cdrom_read(device, address, count, dest);
    }
}


uint32_t chs_to_lba(ATADevice __far * device, uint16_t cylinder, uint8_t head, uint8_t sector)
{
    if (cylinder >= device->logi_cylinders || head >= device->logi_heads || !sector ||
                                                                    sector > SECTORS_PER_TRACK) {
        D_MESSAGE("bad CHS address 0x%x(0x%x) 0x%x(0x%x) 0x%x",
                  cylinder, device->logi_cylinders, head,  device->logi_heads, sector);
        return BAD_ADDRESS;
    }
    return ((uint32_t)cylinder * device->logi_heads + head) * 63 + (sector - 1);
}


enum {
    HD_ERR_SUCCESS = 0,
    HD_ERR_BAD_COMMAND_OR_PARAM = 1,
    HD_ERR_WRITE_PROTECTED = 3,
    HD_ERR_READ_FAILED = 4,
    HD_ERR_RESET_FAILED = 5,
};


typedef _Packed struct EDDPacket {
    uint8_t packet_size;
    uint8_t _0;
    uint8_t count; // < 128
    uint8_t _1;
    far_ptr_16_t buf_address; // in case of ~0 then flat_buf_address is used
    uint64_t block_address;
    uint64_t flat_buf_address;
} EDDPacket;


typedef _Packed struct EDDDriveParams {
    uint16_t size;
    uint16_t flags;
    uint32_t cylinders;
    uint32_t heads;
    uint32_t sectors_per_track;
    uint64_t sectors;
    uint16_t bytes_per_sector;
    far_ptr_16_t device_param_table;
    uint16_t key;
    uint8_t length_of_path;
    uint8_t _0;
    uint16_t _1;
    uint8_t host_type[4];
    uint8_t interface[8];
    uint8_t interface_path[8];
    uint8_t device_path[8];
    uint8_t _2;
    uint8_t device_path_checksum;
} EDDDriveParams;


typedef _Packed struct CDBootSpecPacket {
    uint8_t size;
    uint8_t media_type;
    uint8_t drive_id;
    uint8_t controller_id;
    uint32_t disk_image_lba;
    uint16_t device_specification;
    uint16_t user_buf_seg;
    uint16_t load_seg;
    uint16_t load_count;
    uint8_t int13_f8_ch;
    uint8_t int13_f8_cl;
    uint8_t int13_f8_dh;
} CDBootSpecPacket;


static void int13_error(UserRegs __far * context, uint8_t err)
{
    AH(context) = err;
    bda_write_byte(BDA_OFFSET_HD_RESAULT, err);
    context->flags |= (1 << CPU_FLAGS_CF_BIT);
}


static void int13_success(UserRegs __far * context)
{
    bda_write_byte(BDA_OFFSET_HD_RESAULT, HD_ERR_SUCCESS);
    AH(context) = HD_ERR_SUCCESS;
    context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
}


static void edd_write_sectors(UserRegs __far * context)
{
    EDDPacket __far * packet = FAR_POINTER(EDDPacket, context->ds, SI(context));
    ATADevice __far * device;
    uint8_t __far * src;
    int n;

    if (!(device = find_hd(DL(context)))) {
        D_MESSAGE("no device 0x%x", DL(context));
        int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
        return;
    }

    if (packet->packet_size < 16 ) {
        D_MESSAGE("bad packet size");
        int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
        return;
    }

    if (packet->count > 0x7f) {
        D_MESSAGE("bad count");
        int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
        return;
    }

    if (packet->buf_address != ~0UL) {
        src = (uint8_t __far *)packet->buf_address;
    } else {
        if (packet->packet_size < sizeof(*packet)) {
            D_MESSAGE("bad packet");
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            return;
        }

        D_MESSAGE("flat");
        src = FAR_POINTER(uint8_t, packet->flat_buf_address >> 4,
                          packet->flat_buf_address & 0x0f);
    }

    n = ata_hd_write(device, packet->block_address, packet->count, src);

    if (n != packet->count) {
        D_MESSAGE("read failed");
        packet->count = n;
        int13_error(context, HD_ERR_READ_FAILED);
        return;
    }

    int13_success(context);
    return;
}


void on_int13(UserRegs __far * context)
{
    //todo: call next handler in case of device not found condition
    switch (AH(context)) {
    case INT13_FUNC_EDD_SEEK:
    case INT13_FUNC_EDD_VERIFY:
        D_MESSAGE("implement me 0x%x", AH(context));
        freeze();
        break;
    case INT13_FUNC_EDD_READ_SECTORS: {
        EDDPacket __far * packet = FAR_POINTER(EDDPacket, context->ds, SI(context));
        ATADevice __far * device;
        uint8_t __far * dest;
        int n;

        if (!(device = find_device(DL(context)))) {
            D_MESSAGE("no device 0x%x", DL(context));
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        if (packet->packet_size < 16 ) {
            D_MESSAGE("bad packet size");
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        if (packet->count > 0x7f) {
            D_MESSAGE("bad count");
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        if (packet->buf_address != ~0UL) {
            dest = (uint8_t __far *)packet->buf_address;
        } else {
            if (packet->packet_size < sizeof(*packet)) {
                D_MESSAGE("bad packet");
                int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
                break;
            }

            D_MESSAGE("flat");
            dest = FAR_POINTER(uint8_t, packet->flat_buf_address >> 4,
                               packet->flat_buf_address & 0x0f);
        }

        n = ata_read(device, packet->block_address, packet->count, dest);

        if (n != packet->count) {
            D_MESSAGE("read failed");
            packet->count = n;
            int13_error(context, HD_ERR_READ_FAILED);
            break;
        }

        int13_success(context);
        break;
    }
    case INT13_FUNC_READ_SECTORS: {
        ATADevice __far * device = find_hd(DL(context));
        uint8_t __far * dest;
        uint32_t address;
        uint n;

        if (!device || !AL(context)) {
            D_MESSAGE("bad args ax 0x%x dl 0x%x", AX(context), DL(context));
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        address = chs_to_lba(device,(((uint16_t)CL(context) & 0xc0) << 2) | CH(context),
                             DH(context), CL(context) & 0x3f);

        if (address == BAD_ADDRESS || address + AL(context) > device->sectors) {
            D_MESSAGE("bad address ax 0x%x dl 0x%x", AX(context), DL(context));
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        dest = FAR_POINTER(uint8_t, context->es, BX(context));
        n = ata_hd_read(device, address, AL(context), dest);

        if (n != AL(context)) {
            D_MESSAGE("read failed");
            AL(context) = n;
            int13_error(context, HD_ERR_READ_FAILED);
            break;
        }

        int13_success(context);
        break;
    }
    case INT13_FUNC_EDD_WRITE_SECTORS:
        edd_write_sectors(context);
        break;
    case INT13_FUNC_WRITE_SECTORS: {
        ATADevice __far * device = find_hd(DL(context));
        uint8_t __far * src;
        uint32_t address;
        uint n;

        if (!device || !AL(context)) {
            D_MESSAGE("bad args ax 0x%x dl 0x%x", AX(context), DL(context));
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        address = chs_to_lba(device,(((uint16_t)CL(context) & 0xc0) << 2) | CH(context),
                             DH(context), CL(context) & 0x3f);

        if (address == BAD_ADDRESS || address + AL(context) > device->sectors) {
            D_MESSAGE("bad address ax 0x%x dl 0x%x", AX(context), DL(context));
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        src = FAR_POINTER(uint8_t, context->es, BX(context));
        n = ata_hd_write(device, address, AL(context), src);

        if (n != AL(context)) {
            D_MESSAGE("write failed");
            AL(context) = n;
            int13_error(context, HD_ERR_READ_FAILED);
            break;
        }

        int13_success(context);
        break;
    }
    case INT13_FUNC_GET_DRIVE_PARAMETERS: {
        ATADevice __far * device = find_hd(DL(context));

        if (DL(context) <= 1) {
            AX(context) = 0;
            BX(context) = 0;
            CX(context) = 0;
            DH(context) = 0;
            context->es = 0;
            int13_success(context);
            break;
        }

        if (!device) {
            D_MESSAGE("bad args ax 0x%x dl 0x%x", AX(context), DL(context));
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        AL(context) = 0;
        CL(context) = (((device->logi_cylinders - 1) >> 2) & 0xc0) | 63;
        CH(context) = device->logi_cylinders - 1;
        DH(context) = device->logi_heads - 1;
        DL(context) = bda_read_byte(BDA_OFFSET_HD_ATTACHED);
        int13_success(context);
        break;
    }
    case INT13_FUNC_EDD_INSTALLATION_CHECK: {
        ATADevice __far * device;

        if (BX(context) != 0x55aa || !(device = find_hd(DL(context)))) {
            D_MESSAGE("bad args ax 0x%x dl 0x%x bx 0x%x", AX(context), DL(context), BX(context));
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        AH(context) = 0x30; //EDD 3.0
        CX(context) = (1 << 0); // Fixed disk subset (42h, 43h, 44h, 47h, and 48)
        BX(context) = 0xaa55;
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        bda_write_byte(BDA_OFFSET_HD_RESAULT, HD_ERR_SUCCESS);
        break;
    }
    case INT13_FUNC_EDD_GET_DRIVE_PARAMS: {
        EDDDriveParams __far * params = FAR_POINTER(EDDDriveParams, context->ds, SI(context));
        ATADevice __far * device = find_hd(DL(context));
        uint16_t size = params->size;

        if (size < OFFSET_OF(EDDDriveParams, device_param_table) || !device) {
            D_MESSAGE("bad args ax 0x%x dl 0x%x size 0x%x", AX(context), DL(context), size);
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        mem_reset(params, params->size);

        params->sectors = device->sectors;
        params->bytes_per_sector = HD_SECTOR_SIZE;

        int13_success(context);

        if (size < OFFSET_OF(EDDDriveParams, key)) {
            params->size = OFFSET_OF(EDDDriveParams, device_param_table);
            break;
        }

        params->device_param_table = ~params->device_param_table;

        if (size < sizeof(EDDDriveParams)) {
            params->size = OFFSET_OF(EDDDriveParams, key);
            break;
        }

        params->size = sizeof(EDDDriveParams);

        params->key = 0x0bedd;
        params->length_of_path = sizeof(EDDDriveParams) - OFFSET_OF(EDDDriveParams, key);
        ASSERT(params->length_of_path  == 36);
        string_copy(&params->host_type, "PCI");
        string_copy(&params->interface, "ATA");
        params->interface_path[0] = device->pci_bus;
        params->interface_path[1] = device->pci_device;
        params->interface_path[2] = device->pci_func;
        params->device_path[0] = 0; // master
        params->device_path_checksum = checksum8(&params->key, params->length_of_path);
        break;
    }
    case INT13_FUNC_GET_DISK_TYPE: {
        ATADevice __far * device = find_hd(DL(context));

        if (!device) {
            D_MESSAGE("no device 0x%x", DL(context));
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
        } else {
            uint32_t sectors = device->logi_cylinders - 1; // according to the "Undocumented PC"
                                                           // one cylinder is removed for
                                                           // diagnostic. hazard ?

            sectors = sectors * device->logi_heads * SECTORS_PER_TRACK;
            AH(context) = 0x03;
            CX(context) = sectors >> 16;
            DX(context) = sectors;
            bda_write_byte(BDA_OFFSET_HD_RESAULT, HD_ERR_SUCCESS);
            context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        }

        break;
    }
    case INT13_FUNC_DISK_CONTROLLER_RESET:
        // no floppy, just fall through
    case INT13_FUNC_RESET: {
        ATADevice __far * device = find_hd(DL(context));

        if (!device) {
            D_MESSAGE("no device 0x%x", DL(context));
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
        } else {
            if (reset_device(device)) {
                int13_success(context);
            } else {
                D_MESSAGE("reste failed");
                int13_error(context, HD_ERR_RESET_FAILED);
            }
        }
        break;
    }
    case INT13_FUNC_TERM_DISK_EMULATION: {
        switch (AL(context)) {
        case INT13_TERM_DISK_EMULATION_TERM:
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        case INT13_TERM_DISK_EMULATION_QUERY: {
            CDBootSpecPacket __far * spec;
            uint8_t device;

            spec = FAR_POINTER(CDBootSpecPacket, context->ds, SI(context));
            device = DL(context);

           if (!find_device(device)) {
               D_MESSAGE("no device 0x%x", device);
               int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
               break;
           }

           mem_reset(spec, sizeof(*spec));
           spec->size = sizeof(*spec);
           spec->media_type = 0;
           spec->drive_id = device;
           int13_success(context);
           break;
        }
        default:
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
        }
        break;
    }
    default:
        D_MESSAGE("not supported 0x%lx", context->eax);
        int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
    }
}


static void fill_emul_spec(CDBootSpecPacket __far * spec, uint8_t device)
{
    EmulatedDev __far * emulated = get_emulated();

    mem_reset(spec, sizeof(*spec));
    spec->size = sizeof(*spec);

    if (device == 0x80) {
        spec->media_type = 4;
    } else if (emulated->fd_type == 2) {
        spec->media_type = 1;
    } else if (emulated->fd_type == 4) {
        spec->media_type = 2;
    } else {
        spec->media_type = 3;
    }

    spec->media_type |= (1 << 6); // ATAPI
    spec->drive_id = device;
    spec->disk_image_lba = emulated->image_lba;
    spec->int13_f8_ch = emulated->cylinders - 1;
    spec->int13_f8_cl = (((emulated->cylinders - 1) >> 2) & 0xc0) | emulated->sec_per_track;
    spec->int13_f8_dh = emulated->heads - 1;

}


static uint emulate_read_sectors(uint8_t dev_id, uint64_t start, uint8_t count, void __far * dest)
{
    EDDPacket packet;
    uint16_t packet_ptr;
    bool_t ok;

    packet_ptr = (uint16_t)&packet;
    packet.packet_size = sizeof(packet);
    packet._0 = 0;
    packet.count = count;
    packet._1 = 0;
    packet.buf_address = (far_ptr_16_t)dest;
    packet.block_address = start;
    packet.flat_buf_address = 0;

    __asm {
        push ds
        mov ax, ss
        mov ds, ax
        mov si, packet_ptr
        mov dl, dev_id
        mov ah, INT13_FUNC_EDD_READ_SECTORS
        int 0x13
        pop ds
        jc failed
        mov al, 1
        jmp result
    failed:
        mov al, 0
    result:
        mov ok, al
    }

    if (!ok) {
        count = (packet.count != count) ? packet.count : 0;
    }

    return count;
}


static uint emulate_read(uint8_t dev_id, uint32_t offset, uint32_t lba, uint count,
                         uint8_t __far * dest)
{
    uint skip = (lba % 4);
    uint8_t __far * tmp;
    uint ret = 0;
    uint full;

    if (skip) {
        tmp = FAR_POINTER(uint8_t, bda_read_word(BDA_OFFSET_EBDA), OFFSET_OF_PRIVATE(read_buf));

        if (!emulate_read_sectors(dev_id, offset + lba / 4, 1, tmp)) {
            return 0;
        }

        ret = MIN(4 - skip, count);
        mem_copy(dest, tmp + HD_SECTOR_SIZE * skip, ret * HD_SECTOR_SIZE);

        count -= ret;
        lba += ret;
        dest += ret * HD_SECTOR_SIZE;
    }

    full = count / 4;

    if (full) {
        if (emulate_read_sectors(dev_id, offset + lba / 4, full, dest) != full) {
            return ret;
        }

        full *= 4;
        count -= full;
        lba += full;
        dest += full * HD_SECTOR_SIZE;
        ret += full;
    }

    if (count) {
        tmp = FAR_POINTER(uint8_t, bda_read_word(BDA_OFFSET_EBDA), OFFSET_OF_PRIVATE(read_buf));

        if (!emulate_read_sectors(dev_id, offset + lba / 4, 1, tmp)) {
            return ret;
        }

        mem_copy(dest, tmp, count * HD_SECTOR_SIZE);
        ret += count;
    }

    return ret;
}


uint on_int13_hd_emulate(UserRegs __far * context)
{
    switch (AH(context)) {
    case INT13_FUNC_READ_SECTORS: {
        EmulatedDev __far * emulated = get_emulated();
        uint16_t cylinder;
        uint8_t sector;
        uint8_t count;
        uint8_t head;
        uint32_t lba;
        uint n;

        if (DL(context) != 0x80) {
            goto no_dev_emulate;
        }

        emulated = get_emulated();

        count = AL(context);
        if (!count) {
            D_MESSAGE("bad args ax 0x%x", AX(context));
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        cylinder = (((uint16_t)CL(context) & 0xc0) << 2) | CH(context);

        if (cylinder >= emulated->cylinders) {
            D_MESSAGE("INT13_FUNC_READ_SECTORS: invalid cylinder 0x%x", cylinder);
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        head = DH(context);

        if (head >= emulated->heads) {
            D_MESSAGE("INT13_FUNC_READ_SECTORS: invalid head 0x%x", head);
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        sector = CL(context);

        if (sector < 1 || sector > emulated->sec_per_track) {
            D_MESSAGE("INT13_FUNC_READ_SECTORS: invalid sector 0x%x", sector);
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        lba = cylinder;
        lba = (lba * emulated->heads + head) * emulated->sec_per_track + (sector - 1);

        n = emulate_read(emulated->device_id, emulated->image_lba, lba, count,
                         FAR_POINTER(void, context->es, BX(context)));

        if (n != count) {
            D_MESSAGE("read failed");
            AL(context) = n;
            int13_error(context, HD_ERR_READ_FAILED);
            break;
        }

        int13_success(context);
        break;
    }
    case INT13_FUNC_GET_DRIVE_PARAMETERS: {
        EmulatedDev __far * dev;

        if (DL(context) != 0x80) {
            goto no_dev_emulate;
        }

        dev = get_emulated();
        AL(context) = 0;
        CL(context) = (((dev->cylinders - 1) >> 2) & 0xc0) | dev->sec_per_track;
        CH(context) = dev->cylinders - 1;
        DH(context) = dev->heads - 1;
        DL(context) = bda_read_byte(BDA_OFFSET_HD_ATTACHED);

        int13_success(context);
        break;
    }
    case INT13_FUNC_GET_DISK_TYPE: {
        EmulatedDev __far * dev;
        uint32_t sectors;

        if (DL(context) != 0x80) {
            goto no_dev_emulate;
        }

        dev = get_emulated();
        sectors = dev->cylinders - 1;
        sectors = sectors * dev->heads * dev->sec_per_track;
        AH(context) = 0x03;
        CX(context) = sectors >> 16;
        DX(context) = sectors;
        bda_write_byte(BDA_OFFSET_HD_RESAULT, HD_ERR_SUCCESS);
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);

        break;
    }
    case INT13_FUNC_RESET:
    case INT13_FUNC_DISK_CONTROLLER_RESET:
        if (DL(context) != 0x80) {
            goto no_dev_emulate;
        }

        int13_success(context);
        break;
    case INT13_FUNC_WRITE_SECTORS:
        if (DL(context) != 0x80) {
            goto no_dev_emulate;
        }

        int13_error(context, HD_ERR_WRITE_PROTECTED);
        break;
    case INT13_FUNC_EDD_INSTALLATION_CHECK:
    case INT13_FUNC_EDD_READ_SECTORS:
    case INT13_FUNC_EDD_WRITE_SECTORS:
    case INT13_FUNC_EDD_VERIFY:
    case INT13_FUNC_EDD_SEEK:
    case INT13_FUNC_EDD_GET_DRIVE_PARAMS:
        if (DL(context) != 0x80) {
            goto no_dev_emulate;
        }

        D_MESSAGE("edd is not supported for the emulate device 0x%x", AH(context));
        int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
        break;
    case INT13_FUNC_TERM_DISK_EMULATION: {
        CDBootSpecPacket __far * spec;
        uint8_t device;

        spec = FAR_POINTER(CDBootSpecPacket, context->ds, SI(context));
        device = DL(context);

        switch (AL(context)) {
        case INT13_TERM_DISK_EMULATION_TERM:
            if (device != 0x80 && device != 0x7f) {
                goto no_dev_emulate;
            }

            device = 0x80;
            fill_emul_spec(spec, device);
            ata_stop_emulation();
            int13_success(context);
            break;
        case INT13_TERM_DISK_EMULATION_QUERY: {
            if (device != 0x80) {
                goto no_dev_emulate;
            }

            fill_emul_spec(spec, device);
            int13_success(context);
            break;
        }
        default:
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
        }
        break;
    }
    default:
        D_MESSAGE("unhandled function 0x%x", AH(context));
        int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
    }

    return INT13_HANDLED;

no_dev_emulate:
    if ((DL(context) & 0x80) && (DL(context) & ~0x80) < MAX_HD) {
        return INT13_DEC_AND_NEXT;
    } else {
        return INT13_CALL_NEXT;
    }
}


uint on_int13_fd_emulate(UserRegs __far * context)
{
    switch (AH(context)) {
    case INT13_FUNC_READ_SECTORS: {
        EmulatedDev __far * emulated = get_emulated();
        uint8_t cylinder;
        uint8_t sector;
        uint8_t count;
        uint8_t head;
        uint32_t lba;
        uint n;

        if (DL(context)) {
            goto no_dev_emulate;
        }

        count = AL(context);

        if (!count || count > 36) {
            D_MESSAGE("INT13_FUNC_READ_SECTORS: invalid sector count 0x%x", count);
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        cylinder = CH(context);

        if (cylinder >= emulated->cylinders) {
            D_MESSAGE("INT13_FUNC_READ_SECTORS: invalid cylinder 0x%x", cylinder);
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        head = DH(context);

        if (head >= emulated->heads) {
            D_MESSAGE("INT13_FUNC_READ_SECTORS: invalid head 0x%x", head);
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        sector = CL(context);

        if (sector < 1 || sector > emulated->sec_per_track) {
            D_MESSAGE("INT13_FUNC_READ_SECTORS: invalid sector 0x%x", sector);
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        lba = cylinder;
        lba = (lba * emulated->heads + head) * emulated->sec_per_track + (sector - 1);

        if (lba + count > emulated->sectors) {
            D_MESSAGE("INT13_FUNC_READ_SECTORS: bad range lba 0x%lx count 0x%x sectors 0x%lx",
                     sector, count, emulated->sectors);
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        n = emulate_read(emulated->device_id, emulated->image_lba, lba, count,
                         FAR_POINTER(void, context->es, BX(context)));

        if (n != count) {
            D_MESSAGE("read failed");
            AL(context) = n;
            int13_error(context, HD_ERR_READ_FAILED);
            break;
        }

        int13_success(context);
        break;
    }
    case INT13_FUNC_GET_DRIVE_PARAMETERS: {
        EmulatedDev __far * dev;

        if (DL(context)) {
            goto no_dev_emulate;
        }

        dev = get_emulated();
        AL(context) = 0;
        BH(context) = 0;
        BL(context) = dev->fd_type;

        CL(context) = dev->sec_per_track;
        CH(context) = dev->cylinders - 1;
        DH(context) = dev->heads - 1;
        DL(context) = 1; // number of fd installed

        context->es = 0;
        DI(context) = 0;

        int13_success(context);
        break;
    }
    case INT13_FUNC_GET_DISK_TYPE:
        if (DL(context)) {
            goto no_dev_emulate;
        }

        AH(context) = 0x01; // drive present, does not support line change
        bda_write_byte(BDA_OFFSET_HD_RESAULT, HD_ERR_SUCCESS);
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    case INT13_FUNC_RESET:
    case INT13_FUNC_DISK_CONTROLLER_RESET:
        if (DL(context)) {
            goto no_dev_emulate;
        }

        int13_success(context);
        break;
    case INT13_FUNC_WRITE_SECTORS:
        if (DL(context)) {
            goto no_dev_emulate;
        }

        int13_error(context, HD_ERR_WRITE_PROTECTED);
        break;
    case INT13_FUNC_EDD_INSTALLATION_CHECK:
    case INT13_FUNC_EDD_READ_SECTORS:
    case INT13_FUNC_EDD_WRITE_SECTORS:
    case INT13_FUNC_EDD_VERIFY:
    case INT13_FUNC_EDD_SEEK:
    case INT13_FUNC_EDD_GET_DRIVE_PARAMS:
        if (DL(context)) {
            goto no_dev_emulate;
        }

        D_MESSAGE("edd is not supported for the emulate device 0x%x", AH(context));
        int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
        break;
    case INT13_FUNC_TERM_DISK_EMULATION: {
        CDBootSpecPacket __far * spec;
        uint8_t device;

        spec = FAR_POINTER(CDBootSpecPacket, context->ds, SI(context));
        device = DL(context);

        switch (AL(context)) {
        case INT13_TERM_DISK_EMULATION_TERM:
            if (device != 0 && device != 0x7f) {
                goto no_dev_emulate;
            }

            device = 0;
            fill_emul_spec(spec, device);
            ata_stop_emulation();
            int13_success(context);
            break;
        case INT13_TERM_DISK_EMULATION_QUERY: {
            if (device != 0) {
                goto no_dev_emulate;
            }

            fill_emul_spec(spec, device);
            int13_success(context);
            break;
        }
        default:
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
        }
        break;
    }
    default:
        D_MESSAGE("unhandled function 0x%x", AH(context));
        int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
    }

    return INT13_HANDLED;

no_dev_emulate:
    if (DL(context) & 0x80) {
        return INT13_CALL_NEXT;
    } else {
        return INT13_DEC_AND_NEXT;
    }
}


bool_t ata_is_cdrom(ATADevice __far * device)
{
    return !!(device->flags & ATA_FLAGS_ATAPI);
}


bool_t ata_is_hd(ATADevice __far * device)
{
    return !!(device->flags & ATA_FLAGS_ATA);
}


uint ata_hd_read(ATADevice __far * device, uint64_t address, uint count,
                 uint8_t __far * dest)
{
    ASSERT(count > 0 && count <= 256);
    ASSERT(address + count > address && address + count <= device->sectors);

    if (address + count <= (1UL << 28) - 1) {
        PIOCmd cmd;

        mem_set(&cmd, 0, sizeof(cmd));
        cmd.blocks = count;
        cmd.buf = (uint16_t __far *)dest;

        cmd.regs[ATA_IO_SECTOR_COUNT] = (count == 256) ? 0 : count;
        cmd.regs[ATA_IO_LBA_LOW] = address;
        cmd.regs[ATA_IO_LBA_MID] = address >> 8;
        cmd.regs[ATA_IO_LBA_HIGH] = address >> 16;
        cmd.regs[ATA_IO_DEVICE] = ATA_DEVICE_LBA_MASK | ((address >> 24) & ATA_DEVICE_ADDRESS_MASK);
        cmd.regs[ATA_IO_COMMAND] = ATA_CMD_READ_SECTORS;

        return ata_pio_cmd(device->cmd_port, device->ctrl_port, &cmd, ata_pio_in);
    } else {
        PIOCmdExt cmd;

        mem_set(&cmd, 0, sizeof(cmd));
        cmd.blocks = count;
        cmd.buf = (uint16_t __far *)dest;

        cmd.regs[ATA_IO_SECTOR_COUNT][CURRENT_REG] = count;
        cmd.regs[ATA_IO_LBA_LOW][PREVIOUS_REG] = address >> 40;
        cmd.regs[ATA_IO_LBA_MID][PREVIOUS_REG] = address >> 32;
        cmd.regs[ATA_IO_LBA_LOW][PREVIOUS_REG] = address >> 24;
        cmd.regs[ATA_IO_LBA_HIGH][CURRENT_REG] = address >> 16;
        cmd.regs[ATA_IO_LBA_MID][CURRENT_REG] = address >> 8;
        cmd.regs[ATA_IO_LBA_LOW][CURRENT_REG] = address;
        cmd.device = ATA_DEVICE_LBA_MASK;
        cmd.command = ATA_CMD_READ_SECTORS_EXT;

        return ata_pio_cmd_ext(device->cmd_port, device->ctrl_port, &cmd, ata_pio_in);
    }
}


bool_t ata_start_fd_emulation(ATADevice __far * device, uint32_t image_lba, uint type)
{
    EmulatedDev __far * emulated_dev;
    uint8_t hd_attached;
    uint16_t flags;
    uint16_t vec_seg;
    uint16_t vec_offset;

    NO_INTERRUPT();

    if (!ata_is_cdrom(device)) {
        return FALSE;
    }

    flags = ebda_read_word(OFFSET_OF_PRIVATE(bios_flags));

    if ((flags & (BIOS_FLAGS_FD_EMULATION | BIOS_FLAGS_HD_EMULATION))) {
        D_MESSAGE("only a single emulatiion is supported");
        return FALSE;
    }

    // update BDA_OFFSET_EQUIPMENT, cmos data, etc.

    emulated_dev = get_emulated();
    emulated_dev->device_id = device->id;
    emulated_dev->image_lba = image_lba;
    emulated_dev->heads = 0x02;
    emulated_dev->cylinders = 0x50;

    switch (type) {
    case 1:
        emulated_dev->sec_per_track = 0x0f;
        emulated_dev->fd_type = 2;
        break;
    case 2:
        emulated_dev->sec_per_track =0x12;
        emulated_dev->fd_type = 4;
        break;
    case 3:
        emulated_dev->sec_per_track = 0x24;
        emulated_dev->fd_type = 5;
        break;
    default:
        D_MESSAGE("invalid type");
        return FALSE;
    }

    emulated_dev->sectors = emulated_dev->cylinders * emulated_dev->heads *
                            emulated_dev->sec_per_track;

    ebda_write_word(OFFSET_OF_PRIVATE(bios_flags), flags | BIOS_FLAGS_FD_EMULATION);

    get_int_vec(0x13, &vec_seg, &vec_offset);
    ebda_write_word(OFFSET_OF_PRIVATE(int13_emu_next_seg), vec_seg);
    ebda_write_word(OFFSET_OF_PRIVATE(int13_emu_next_offset), vec_offset);
    set_int_vec(0x13, get_cs(), FUNC_OFFSET(int13_fd_emulate_handler));

    return TRUE;
}


typedef _Packed struct PartitionInfo {
    uint8_t status; // 0x80 = bootable
    uint8_t start[3];
    uint8_t type;
    uint8_t end[3];
    uint32_t start_lba;
    uint32_t sectors;
} PartitionInfo;


bool_t ata_start_hd_emulation(ATADevice __far * device, uint32_t image_lba, uint8_t __far * mbr)
{
    EmulatedDev __far * emulated_dev;
    PartitionInfo __far * part;
    uint8_t hd_attached;
    uint16_t flags;
    uint16_t vec_seg;
    uint16_t vec_offset;

    NO_INTERRUPT();

    if (!ata_is_cdrom(device)) {
        return FALSE;
    }

    flags = ebda_read_word(OFFSET_OF_PRIVATE(bios_flags));

    if ((flags & (BIOS_FLAGS_HD_EMULATION | BIOS_FLAGS_FD_EMULATION ))) {
        D_MESSAGE("only a single emulatiion is supported");
        return FALSE;
    }

    hd_attached = bda_read_byte(BDA_OFFSET_HD_ATTACHED);

    if (hd_attached == MAX_HD) {
        return FALSE;
    }

    if (mbr[0x1fe] != 0x55 || mbr[0x1ff] != 0xaa) {
        D_MESSAGE("invalid mbr signature");
        return FALSE;
    }

    part = (PartitionInfo __far *)(mbr + 0x1be);

    D_MESSAGE("emulated_dev is not fully initialized");
    freeze();

    emulated_dev = get_emulated();
    emulated_dev->device_id = device->id;
    emulated_dev->image_lba = image_lba;

    ebda_write_word(OFFSET_OF_PRIVATE(bios_flags), flags | BIOS_FLAGS_HD_EMULATION);
    bda_write_byte(BDA_OFFSET_HD_ATTACHED, hd_attached + 1);

    get_int_vec(0x13, &vec_seg, &vec_offset);
    ebda_write_word(OFFSET_OF_PRIVATE(int13_emu_next_seg), vec_seg);
    ebda_write_word(OFFSET_OF_PRIVATE(int13_emu_next_offset), vec_offset);
    set_int_vec(0x13, get_cs(), FUNC_OFFSET(int13_hd_emulate_handler));

    return TRUE;
}


void ata_stop_emulation()
{
    EmulatedDev __far * emulated_dev;
    ATADevice __far * device;
    uint16_t flags;

    NO_INTERRUPT();

    flags = ebda_read_word(OFFSET_OF_PRIVATE(bios_flags));

    if (!(flags & (BIOS_FLAGS_HD_EMULATION | BIOS_FLAGS_FD_EMULATION))) {
        return;
    }

    if ((flags & BIOS_FLAGS_HD_EMULATION)) {
        uint8_t hd_attached;
        hd_attached = bda_read_byte(BDA_OFFSET_HD_ATTACHED);
        bda_write_byte(BDA_OFFSET_HD_ATTACHED, hd_attached - 1);
    }

    flags &= ~(BIOS_FLAGS_HD_EMULATION | BIOS_FLAGS_FD_EMULATION);
    ebda_write_word(OFFSET_OF_PRIVATE(bios_flags), flags);

    set_int_vec(0x13, ebda_read_word(OFFSET_OF_PRIVATE(int13_emu_next_seg)),
                      ebda_read_word(OFFSET_OF_PRIVATE(int13_emu_next_offset)));
}


void ata_init()
{
    HDCmosParam cmos_param;

    rtc_write(CMOS_OFFSET_HD_TYPE, 0); // no hd

    // although not valid, resetting all hd data
    rtc_write(CMOS_OFFSET_HD0_EXT_TYPE, 0);
    rtc_write(CMOS_OFFSET_HD1_EXT_TYPE, 0);
    mem_reset(&cmos_param, sizeof(cmos_param));
    rtc_write_buf(CMOS_OFFSET_HD0_PARAMS, &cmos_param, sizeof(cmos_param));
    rtc_write_buf(CMOS_OFFSET_HD1_PARAMS, &cmos_param, sizeof(cmos_param));

    set_int_vec(0x13, get_cs(), FUNC_OFFSET(int13_handler));
    pci_for_each(ata_pci_cb, NULL);
}

