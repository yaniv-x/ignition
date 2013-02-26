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

#include "bios32.h"
#include "utils.h"
#include "platform.h"
#include "pci.h"
#include "nox.h"
#include "error_codes.h"


typedef _Packed struct GenAddress {
    uint8_t address_space;
    uint8_t bits_width;
    uint8_t bits_offset;
    uint8_t access_size;
    uint64_t address;
} GenAddress;


typedef _Packed struct DescriptionHeader {
    uint32_t signature;
    uint32_t length;
    uint8_t revision;
    uint8_t checksum;
    uint8_t oem_id[6];
    uint8_t oem_table_id[8];
    uint32_t oem_revision;
    uint32_t creator_id;
    uint32_t creator_revision;
} DescriptionHeader;


typedef _Packed struct RSDT { // root system description table
    DescriptionHeader header;
    uint32_t fadt_address;
    uint32_t madt_address;
} RSDT;


typedef _Packed struct XSDT { // 64-bit root system description table
    DescriptionHeader header;
    uint64_t fadt_address;
    uint64_t madt_address;
} XSDT;


typedef _Packed struct FADT { // fixed ACPI description table
    DescriptionHeader header;
    uint32_t facs_address;
    uint32_t dsdt_address;
    uint8_t _0;
    uint8_t prefered_profile;
    uint16_t sci_int;
    uint32_t smi_cmd;
    uint8_t acpi_enable;
    uint8_t acpi_disable;
    uint8_t s4bios_req;
    uint8_t pstate_cnt;
    uint32_t pm1a_evt_block;
    uint32_t pm1b_evt_block;
    uint32_t pm1a_cnt_block;
    uint32_t pm1b_cnt_block;
    uint32_t pm2_cnt_block;
    uint32_t pm_tmr_block;
    uint32_t gpe0_block;
    uint32_t gpe1_block;
    uint8_t pm1_evt_len;
    uint8_t pm1_cnt_len;
    uint8_t pm2_cnt_len;
    uint8_t pm_tmr_len;
    uint8_t gpe0_blk_len;
    uint8_t gpe1_blk_len;
    uint8_t gpe1_base;
    uint8_t cst_cnt;
    uint16_t p_lvl2_lat;
    uint16_t p_lvl3_lat;
    uint16_t flush_size;
    uint16_t flush_stride;
    uint8_t duty_offset;
    uint8_t duty_width;
    uint8_t day_alarm;
    uint8_t mon_alarm;
    uint8_t century;
    uint16_t iapc_boot_arch;
    uint8_t _1;
    uint32_t flags;
    GenAddress reset_reg;
    uint8_t reset_magic;
    uint8_t _2[3];
    uint64_t x_facs_address;
    uint64_t x_dsdt_address;
    GenAddress x_pm1a_evt_blk;
    GenAddress x_pm1b_evt_blk;
    GenAddress x_pm1a_cnt_blk;
    GenAddress x_pm1b_cnt_blk;
    GenAddress x_pm2_cnt_blk;
    GenAddress x_pm_tmr_blk;
    GenAddress x_gpe0_blk;
    GenAddress x_gpe1_blk;
} FADT;


typedef _Packed struct FACS {
    uint32_t signature;
    uint32_t length;
    uint32_t hard_signature;
    uint32_t firm_work_vector;
    uint32_t global_lock;
    uint32_t flags;
    uint64_t x_firm_work_vector;
    uint8_t version;
    uint8_t reserved[31];
} FACS;


typedef _Packed struct DSDT {
    DescriptionHeader header;
} DSDT;


typedef _Packed struct LocalApic {
    uint8_t type;
    uint8_t length;
    uint8_t acpi_processor_id;
    uint8_t apic_id;
    uint32_t flags;
} LocalApic;

typedef _Packed struct IoApic {
    uint8_t type;
    uint8_t length;
    uint8_t io_apic_id;
    uint8_t reserved;
    uint32_t io_apic_address;
    uint32_t global_sys_interrupt_start;
} IoApic;



typedef _Packed struct MADT {
    DescriptionHeader header;
    uint32_t local_apic_address;
    uint32_t flags;
    IoApic io_apic;
    //need to add non maskable interrupt source?
    //need to add local APIC NMI?
    LocalApic local_apic[1];
} MADT;


#define OEMID "OBJCX"       // DSDT.asl dependency
#define MODEL_ID "NOXMB01"  // DSDT.asl dependency


#define SET_GEN_IO(a, size, addr) {  \
    a.address_space = 1;             \
    a.bits_width = size * 8;         \
    a.bits_offset = 0;               \
    a.access_size = size;            \
    a.address = addr;                \
}


static uint16_t get_io_base()
{
    return pci_read_32(0, PM_CONTROLLER_SLOT, PCI_OFFSET_BAR_0) & PCI_BAR_IO_ADDRESS_MASK;
}


static void init_firmware_acpi_control_struct(FACS* facs)
{
    mem_reset(facs, sizeof(*facs));
    facs->signature = FOUR_CHARS('FACS');;
    facs->length = sizeof(*facs);
    facs->hard_signature = 0xbfbfbfbf; // todo: real signature
    facs->version = 1;
}


#include "DSDT.c"

static DSDT* init_differentiated_sys_description_table()
{
    return (DSDT*)dsdt;
}


static void init_fix_acpi_descriptor_table(FADT* fadt)
{
    uint16_t io_base = get_io_base();
    FACS* facs = (FACS*)ALIGN(zalloc(sizeof(*facs) + 63), 64);

    mem_reset(fadt, sizeof(*fadt));
    fadt->header.signature = FOUR_CHARS('FACP');
    fadt->header.length = sizeof(*fadt);
    fadt->header.revision = 3;
    mem_copy(fadt->header.oem_id, OEMID, sizeof(fadt->header.oem_id));
    mem_copy(fadt->header.oem_table_id, MODEL_ID, sizeof(fadt->header.oem_table_id));
    fadt->header.oem_revision = 1;

    fadt->facs_address = (uint32_t)facs;
    fadt->x_facs_address = fadt->facs_address;
    fadt->dsdt_address = (uint32_t)init_differentiated_sys_description_table();
    fadt->x_dsdt_address = fadt->dsdt_address;
    fadt->prefered_profile = 1; // desktop
    fadt->sci_int = PM_IRQ_LINE;
    ASSERT(PM_IO_ENABLE == PM_IO_STATUS + 2);
    fadt->pm1a_evt_block = io_base + PM_IO_STATUS;
    fadt->pm1a_cnt_block = io_base + PM_IO_CONTROL;
    fadt->pm_tmr_block = io_base + PM_IO_TIMER;
    fadt->pm1_evt_len = 4;
    fadt->pm1_cnt_len = 2;
    fadt->pm_tmr_len = 4;
    fadt->p_lvl2_lat = 0x7fff; // > 100 => C2 is not supported
    fadt->p_lvl3_lat = 0x7fff; // > 1000 => C3 is not supported
    fadt->century = CMOS_OFFSET_CENTURY;
    fadt->iapc_boot_arch = (1 << 1); // 8042 or equivalent keyboard controller
    fadt->flags = (1 << 0) /*WBINDV*/ | (1 << 1) /*WBINDV_FLUSH*/ | (1 << 8) /* 32 bit timer*/ |
                  (1 << 10) /* support reset register*/;
    SET_GEN_IO(fadt->reset_reg, 1, io_base + PM_IO_RESET);
    fadt->reset_magic = PM_RESET_MAGIC;
    SET_GEN_IO(fadt->x_pm1a_evt_blk, 2, io_base + PM_IO_STATUS); // x2 ?
    SET_GEN_IO(fadt->x_pm1a_cnt_blk, 2, io_base + PM_IO_CONTROL);
    SET_GEN_IO(fadt->x_pm_tmr_blk, 4, io_base + PM_IO_TIMER);

    // todo: support day_alarm and mon_alarm

    fadt->header.checksum = checksum8(fadt, sizeof(*fadt));

    init_firmware_acpi_control_struct(facs);
}


static MADT* init_multiple_apic_descriptor_table()
{
    uint num_cpus = platform_get_reg(PLATFORM_REG_NUM_CPUS);
    uint madt_size = sizeof(MADT) + sizeof(LocalApic) * (num_cpus - 1);
    MADT* madt = (MADT*)zalloc(madt_size);
    uint32_t* io_apic_select = (uint32_t*)IO_APIC_ADDRESS;
    uint32_t* io_apic_window = (uint32_t*)(IO_APIC_ADDRESS + 0x10);

    D_MESSAGE("num of cpus is %u", num_cpus);

    mem_reset(madt, madt_size);
    madt->header.signature = FOUR_CHARS('APIC');
    madt->header.length = madt_size;
    madt->header.revision = 1;
    mem_copy(madt->header.oem_id, OEMID, sizeof(madt->header.oem_id));
    mem_copy(madt->header.oem_table_id, MODEL_ID, sizeof(madt->header.oem_table_id));
    madt->header.oem_revision = 1;
    madt->local_apic_address = LOCAL_APIC_ADDRESS;
    madt->flags = (1 << 0); // has dual 8259

    madt->io_apic.type = 1; // IO APIC
    madt->io_apic.length = sizeof(madt->io_apic);
    *io_apic_select = 0;
    madt->io_apic.io_apic_id = (*io_apic_window >> 24);
    madt->io_apic.io_apic_address = IO_APIC_ADDRESS;
    madt->io_apic.global_sys_interrupt_start = 0;

    return madt;
}

static void init_root_sys_descriptor_table(RSDT* rsdt, XSDT* xsdt)
{
    FADT* fadt = (FADT*)zalloc(sizeof(*fadt));

    mem_reset(rsdt, sizeof(*rsdt));
    rsdt->header.signature = FOUR_CHARS('RSDT');
    rsdt->header.length = sizeof(*rsdt);
    rsdt->header.revision = 1;
    mem_copy(rsdt->header.oem_id, OEMID, sizeof(rsdt->header.oem_id));
    mem_copy(rsdt->header.oem_table_id, MODEL_ID, sizeof(rsdt->header.oem_table_id));
    rsdt->header.oem_revision = 1;
    rsdt->fadt_address = (uint32_t)fadt;
    rsdt->madt_address = (uint32_t)init_multiple_apic_descriptor_table();
    rsdt->header.checksum = checksum8(rsdt, sizeof(*rsdt));

    mem_reset(xsdt, sizeof(*xsdt));
    xsdt->header.signature = FOUR_CHARS('XSDT');
    xsdt->header.length = sizeof(*xsdt);
    xsdt->header.revision = 1;
    mem_copy(xsdt->header.oem_id, OEMID, sizeof(xsdt->header.oem_id));
    mem_copy(xsdt->header.oem_table_id, MODEL_ID, sizeof(xsdt->header.oem_table_id));
    xsdt->header.oem_revision = 1;
    xsdt->fadt_address = rsdt->fadt_address;
    xsdt->madt_address = rsdt->madt_address;
    xsdt->header.checksum = checksum8(xsdt, sizeof(*xsdt));

    init_fix_acpi_descriptor_table(fadt);
}


void acpi_update_self()
{
    RSDP* rsdp = &get_ebda()->rsdp;
    RSDT* rsdt = (RSDT*)rsdp->rsdt_address;
    MADT* madt = (MADT*)rsdt->madt_address;
    uint num_cpus = platform_get_reg(PLATFORM_REG_NUM_CPUS);
    uint32_t apic_id = *(uint32_t*)(LOCAL_APIC_ADDRESS + 0x20) >> 24;
    uint madt_size = sizeof(MADT) + sizeof(LocalApic) * (num_cpus - 1);

    if (apic_id >= num_cpus) {
        bios_error(BIOS_ERROR_ACPI_OUT_OF_CPU_SLOTS);
    }

    if (madt->local_apic[apic_id].length) {
        bios_error(BIOS_ERROR_APIC_ID_COLLISION);
    }

    madt->local_apic[apic_id].type = 0; // local APIC
    madt->local_apic[apic_id].length = sizeof(madt->local_apic[0]);
    madt->local_apic[apic_id].acpi_processor_id = apic_id;
    madt->local_apic[apic_id].apic_id = apic_id;
    madt->local_apic[apic_id].flags = (1 << 0);
}


void acpi_finalize_MADT()
{
    RSDP* rsdp = &get_ebda()->rsdp;
    RSDT* rsdt = (RSDT*)rsdp->rsdt_address;
    MADT* madt = (MADT*)rsdt->madt_address;
    uint num_cpus = platform_get_reg(PLATFORM_REG_NUM_CPUS);
    uint madt_size = sizeof(MADT) + sizeof(LocalApic) * (num_cpus - 1);
    uint i;

    for (i = 0; i < num_cpus; i++) {
        if (!madt->local_apic[i].length) {
            bios_error(BIOS_ERROR_ACPI_INVALID_SLOT);
        }
    }

    madt->header.checksum = checksum8(madt, madt_size);
}

extern RSDP __RSDP;

void init_acpi()
{
    RSDP* rsdp = &get_ebda()->rsdp;
    RSDT* rsdt = (RSDT*)zalloc(sizeof(*rsdt));
    XSDT* xsdt = (XSDT*)zalloc(sizeof(*xsdt));

    ASSERT(((uint32_t)rsdp & 0x0f) == 0); // must be allign on 16 byte
    ASSERT((uint8_t*)(rsdp + 1) <= (uint8_t*)get_ebda() + KB); // must be in the first KB of EDBA
    ASSERT(string_length(OEMID) == 5);
    ASSERT(string_length(MODEL_ID) == 7);

    mem_reset(rsdp, sizeof(*rsdp));
    mem_copy(rsdp->signature, "RSD PTR ", sizeof(rsdp->signature));
    mem_copy(rsdp->oem_str, OEMID, sizeof(rsdp->oem_str));
    rsdp->revision = 2;
    rsdp->rsdt_address = (uint32_t)rsdt;
    rsdp->xsdt_address = (uint64_t)xsdt;
    rsdp->length = sizeof(*rsdp);
    rsdp->checksum = checksum8(rsdp, OFFSET_OF(RSDP, length));
    rsdp->ext_checksum = checksum8(rsdp, sizeof(*rsdp));

    init_root_sys_descriptor_table(rsdt, xsdt);

    // workaround for Haiku r1alpha4
    mem_copy(&__RSDP, rsdp, sizeof(__RSDP));

    acpi_update_self();
}

