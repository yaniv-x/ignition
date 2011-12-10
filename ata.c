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

#define ATA_FLAGS_ATAPI (1 << 0)
#define ATA_FLAGS_DMA (1 << 1)
#define ATA_FLAGS_48BIT (1 << 2)

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

#define ATA_DEVICE_LBA_MASK (1 << 6)
#define ATA_DEVICE_ADDRESS_MASK ((1 << 4) - 1)

#define ATA_IO_DATA 0
#define ATA_IO_ERROR 1
#define ATA_IO_FEATURE ATA_IO_ERROR
#define ATA_IO_SECTOR_COUNT 2
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
#define ATA_CMD_IDENTIFY_PACKET_DEVICE 0xa1


#define SOFT_REST_TIMEOUT_MS 5000
#define INVALID_INDEX ~0
#define BAD_ADDRESS ~0UL
#define SECTORS_PER_TRACK 63
#define MAX_HEADS 255
#define MAX_CYLINDERS 1024
#define MAX_SECTORS ((uint32_t)MAX_CYLINDERS * MAX_HEADS * SECTORS_PER_TRACK)
#define HD_SECTOR_SIZE 512


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


void int13_handler();


static void wait_ready(uint16_t alt_port)
{
    while ((inb(alt_port) & (ATA_STATUS_BUSY_MASK | ATA_STATUS_READY_MASK)) !=
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
        uint word_count;

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

        word_count = ATA_PIO_BLOCK_SIZE / 2;

        while (word_count--) {
            *dest++ = inw(cmd_port);
        }
    }

    return i;
}


static uint ata_pio_out(uint16_t cmd_port, uint16_t ctrl_port, uint16_t blocks,
                        uint16_t __far * src)
{
    uint8_t status;
    int i = 0;

    for (;; i++) {
        uint word_count;

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

        word_count = ATA_PIO_BLOCK_SIZE / 2;

        while (word_count--) {
             outw(cmd_port, *src++);
        }
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

    wait_ready(ctrl_port);

    for (i = ATA_IO_FEATURE; i <= ATA_IO_COMMAND; i++) {
        outb(cmd_port + i, cmd->regs[i]);
    }

    return handler(cmd_port, ctrl_port, cmd->blocks, cmd->buf);
}


static uint ata_pio_cmd_ext(uint16_t cmd_port, uint16_t ctrl_port, PIOCmdExt __far * cmd,
                            ata_specific_handler_t handler)
{
    uint i;

    NO_INTERRUPT();

    wait_ready(ctrl_port);

    for (i = ATA_IO_FEATURE; i < ATA_IO_DEVICE; i++) {
        outb(cmd_port + i, cmd->regs[i][PREVIOUS_REG]);
        outb(cmd_port + i, cmd->regs[i][CURRENT_REG]);
    }

    outb(cmd_port + ATA_IO_DEVICE, cmd->device);
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

    if ((identity[ATA_ID_OFFSET_GENERAL_CONF] & ATA_ID_GENERAL_CONF_NOT_ATA_MASK)) {
        device->flags |= ATA_FLAGS_ATAPI;
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
    bda_write_byte(BDA_OFFSET_HD_ATTACHED, hd_id + 1);

    device->hd_id = hd_id | 0x80;

    if (hd_id == 0) {
        fdpt = FAR_POINTER(FixDiskParamTable, bda_read_word(BDA_OFFSET_EBDA), EBDA_OFFSET_FDPT_0);
        set_int_vec(0x41, bda_read_word(BDA_OFFSET_EBDA), EBDA_OFFSET_FDPT_0);
    } else if (hd_id == 1) {
        fdpt = FAR_POINTER(FixDiskParamTable, bda_read_word(BDA_OFFSET_EBDA), EBDA_OFFSET_FDPT_1);
        set_int_vec(0x46, bda_read_word(BDA_OFFSET_EBDA), EBDA_OFFSET_FDPT_1);
    }

    if (fdpt) {
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
        fdpt->logi_sec_per_track = 63;
        fdpt->checksum = checksum8(fdpt, sizeof(*fdpt));
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
    uint8_t hd_id;
    uint16_t command_set;

    if ((identity[ATA_ID_OFFSET_GENERAL_CONF] & ATA_ID_GENERAL_ATAPI_MASK)
                                                             != ATA_ID_GENERAL_ATAPI) {
        D_MESSAGE("not ATAPI");
        return FALSE;
    }

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
        if (ata_is_hd(device) && device->hd_id == id) {
            return device;
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
        D_MESSAGE("timeout");
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


static uint ata_write_sectors(ATADevice __far * device, uint64_t address, uint count,
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


void on_int13(UserRegs __far * context)
{
    switch (AH(context)) {
    case INT13_FUNC_EDD_WRITE_SECTORS:
    case INT13_FUNC_EDD_SEEK:
    case INT13_FUNC_EDD_VERIFY:
        D_MESSAGE("implement me x%x", AH(context));
        freeze();
        break;
    case INT13_FUNC_EDD_READ_SECTORS: {
        EDDPacket __far * packet = FAR_POINTER(EDDPacket, context->ds, SI(context));
        ATADevice __far * device;
        uint8_t __far * dest;
        int n;

        if (packet->packet_size < 16 || !(device = find_hd(DL(context)))) {
            D_MESSAGE("bad packet");
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        if (packet->count > 0x7f || packet->block_address >= device->sectors ||
                                          packet->block_address + packet->count > device->sectors) {
            D_MESSAGE("bad packet 2");
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

        n = ata_read_sectors(device, packet->block_address, packet->count, dest);

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
        uint8_t __far * src;
        uint32_t address;
        uint n;

        if (!device || !AL(context)) {
            D_MESSAGE("bad args");
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        address = chs_to_lba(device,(((uint16_t)CL(context) & 0xc0) << 2) | CH(context),
                             DH(context), CL(context) & 0x3f);

        if (address == BAD_ADDRESS || address + AL(context) > device->sectors) {
            D_MESSAGE("bad args");
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        src = FAR_POINTER(uint8_t, context->es, BX(context));
        n = ata_read_sectors(device, address, AL(context), src);

        if (n != AL(context)) {
            D_MESSAGE("read failed");
            AL(context) = n;
            int13_error(context, HD_ERR_READ_FAILED);
            break;
        }

        int13_success(context);
        break;
    }
    case INT13_FUNC_WRITE_SECTORS: {
        ATADevice __far * device = find_hd(DL(context));
        uint8_t __far * dest;
        uint32_t address;
        uint n;

        if (!device || !AL(context)) {
            D_MESSAGE("bad args");
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        address = chs_to_lba(device,(((uint16_t)CL(context) & 0xc0) << 2) | CH(context),
                             DH(context), CL(context) & 0x3f);

        if (address == BAD_ADDRESS || address + AL(context) > device->sectors) {
            D_MESSAGE("bad args");
            int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
            break;
        }

        dest = FAR_POINTER(uint8_t, context->es, BX(context));
        n = ata_write_sectors(device, address, AL(context), dest);

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

        if (!device) {
            D_MESSAGE("bad args");
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
            D_MESSAGE("bad args");
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
            D_MESSAGE("bad args");
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
            AL(context) = 0x03;
            CX(context) = sectors >> 16;
            DX(context) = sectors;

            int13_success(context);
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
    default:
        D_MESSAGE("not supported 0x%lx", context->eax);
        int13_error(context, HD_ERR_BAD_COMMAND_OR_PARAM);
    }
}


bool_t ata_is_cdrom(ATADevice __far * device)
{
    return !!(device->flags & ATA_FLAGS_ATAPI);
}


bool_t ata_is_hd(ATADevice __far * device)
{
    return !(device->flags & ATA_FLAGS_ATAPI);
}


uint ata_read_sectors(ATADevice __far * device, uint64_t address, uint count,
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


void ata_init()
{
    set_int_vec(0x13, get_cs(), FUNC_OFFSET(int13_handler));
    pci_for_each(ata_pci_cb, NULL);
}

