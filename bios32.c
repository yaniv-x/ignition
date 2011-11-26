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
#include "defs.h"
#include "nox.h"
#include "error_codes.h"
#include "utils.h"
#include "pci.h"
#include "platform.h"


#define globals get_ebda_private()


void ret_16(void);

#define PCI_CLASS_MASS_STORAGE 0x01
#define PCI_MASS_STORAGE_SUBCLASS_IDE 0x01
#define PCI_CLASS_DISPLAY 0x03
#define PCI_DISPLAY_SUBCLASS_VGA 0x00
#define PCI_VGA_PROGIF_VGACOMPAT 0x00

#define EXP_ROM_BAR PCI_NUM_BARS
#define MID_RAM_RANGE_ALIGMENT_MB 512

#define FLAGS_IDE_RES_WAS_CLAIMED (1 << 0)
#define FLAGS_VGA_RES_WAS_CLAIMED (1 << 1)


#define BDA_WORD(offset) ((uint16_t*)(bda + (offset)))
#define EBDA_BYTE(offset) (get_ebda()->public + (offset))


#define DUMB_ALLOC_ALIGNMENT 4
#define DUMB_ALLOC_START 0x00060000
#define DUMB_ALLOC_SIZE (64 * 1024)


typedef struct PCIBarResouce {
    uint bus;
    uint device;
    uint bar;
    uint64_t size;
    uint mapped;
    struct PCIBarResouce* next;
} PCIBarResouce;


typedef struct PCIDevDescriptor {
    uint32_t bus;
    uint32_t device;
    struct PCIDevDescriptor* next;
} PCIDevDescriptor;


static uint8_t* bda = (uint8_t*)BIOS_DATA_AREA_ADDRESS;


static inline void post_and_halt(uint8_t code)
{
    post(code);
    freeze();
}


static void cpuid(uint32_t* a, uint32_t* b, uint32_t* c, uint32_t* d)
{
    __asm {
        mov esi, a
        mov eax, [esi]
        cpuid
        mov [esi], eax
        mov esi, b
        mov [esi], ebx
        mov esi, c
        mov [esi], ecx
        mov esi, d
        mov [esi], edx
    }
}


static uint32_t get_cr0()
{
    uint32_t r;

    __asm {
        mov eax, cr0
        mov r, eax
    }

    return r;
}


static uint64_t read_msr(uint32_t index)
{
    uint32_t h, l;
    __asm {
        mov ecx, index
        rdmsr
        mov h, edx
        mov l, eax
    }

    return (uint64_t)h << 32 | l;
}


static void write_msr(uint32_t index, uint64_t val)
{
    uint32_t l = val;
    uint32_t h = val >> 32;

    __asm {
        mov ecx, index
        mov edx, h
        mov eax, l
        wrmsr
    }
}


static void mem_set(void* from, uint8_t patern, uint32_t n)
{
    uint8_t* now = from;
    uint8_t* end = now + n;
    for (; now < end; now++) *now = patern;
}


static void mem_reset(void* from, uint32_t n)
{
    uint8_t* now = from;
    uint8_t* end = now + n;
    for (; now < end; now++) *now = 0;
}


static void mem_copy(void* dest, const void* src, uint32_t size)
{
    const uint8_t* from = src;
    uint8_t* to = dest;
    uint32_t i;

    for ( i = 0; i < size; i++) {
        to[i] = from[i];
    }
}


static void string_copy(uint8_t* dest, const uint8_t* src, uint32_t buf_size)
{
    const uint8_t* end;

    if (!buf_size) {
        return;
    }

    end = src + MIN(string_length(src), buf_size - 1);

    for (; src < end; src++, dest++) *dest = *src;

    *dest = EOS;
}


static EBDA* get_ebda()
{
    uint32_t seg = *(uint16_t*)(bda + BDA_OFFSET_EBDA);
    EBDA* ebda = (EBDA*)(seg << 4);
    return ebda;
}


static EBDAPrivate* get_ebda_private()
{
    return &get_ebda()->private;
}


static void bios_error(uint16_t err)
{
    uint32_t err_code;

    if (globals->platform_io) {
        err_code = PLATFORM_MK_ERR(PLATFORM_ERR_TYPE_ERROR, PLATFORM_ERR_SUBSYS_BIOS, err);
        outd(globals->platform_io + PLATFORM_IO_ERROR, err_code);
    }

    freeze();
}


static void detect_platform()
{
    uint16_t vendor_id;
    uint16_t device_id;
    uint8_t revision;

    uint32_t address = pci_config_address(0, 0, 0);

    outd(IO_PORT_PCI_ADDRESS, address);

    if (ind(IO_PORT_PCI_ADDRESS) != address) {
        post_and_halt(POST_CODE_DETECT_PLATFORM_FAILED);
    }

    vendor_id = pci_read_16(0, 0, PCI_OFFSET_VENDOR);
    device_id = pci_read_16(0, 0, PCI_OFFSET_DEVICE);
    revision = pci_read_8(0, 0, PCI_OFFSET_REVISION);
    if (vendor_id != NOX_PCI_VENDOR_ID ||
        device_id != NOX_PCI_DEV_ID_HOST_BRIDGE ||
        revision != NOX_PCI_DEV_HOST_BRIDGE_REV) {
        post_and_halt(POST_CODE_DETECT_PLATFORM_FAILED);
    }

    post(POST_CODE_PLATFORM_OK);
}


static void platform_notify_debug_string()
{
    outb(globals->platform_io + PLATFORM_IO_LOG, 0);
}


static inline uint32_t pci_bar_to_size(uint32_t bar)
{
    uint32_t mask = (bar & PCI_BAR_IO_MASK) ? PCI_BAR_IO_ADDRESS_MASK : PCI_BAR_MEM_ADDRESS_MASK;
    int lsb = find_lsb_32(bar & mask);
    return (lsb < 0) ? 0 : (1 << lsb);
}


static inline uint64_t pci_bar64_to_size(uint64_t bar)
{
    int lsb = find_lsb_64(bar & ~0xfULL);
    return (lsb < 0) ? 0 : (1ULL << lsb);
}


static int pci_disable_device(uint32_t bus, uint32_t device, void __far * opaque)
{
    uint16_t command = pci_read_16(bus, device, PCI_OFFSET_COMMAND);
    command &= ~(PCI_COMMAND_ENABLE_IO | PCI_COMMAND_ENABLE_MEM);
    command |= PCI_COMMAND_DISABLE_INTERRUPT;
    pci_write_16(bus, device, PCI_OFFSET_COMMAND, command);
    return FALSE;
}


static int pci_enable_device(uint32_t bus, uint32_t device, void __far * opaque)
{
    uint16_t command = pci_read_16(bus, device, PCI_OFFSET_COMMAND);
    command |= (PCI_COMMAND_ENABLE_IO | PCI_COMMAND_ENABLE_MEM);
    command &= ~PCI_COMMAND_DISABLE_INTERRUPT;
    pci_write_16(bus, device, PCI_OFFSET_COMMAND, command);
    return FALSE;
}


address_t dumb_zalloc(uint32_t size)
{
    address_t ret;

    size = ALIGN(size, DUMB_ALLOC_ALIGNMENT);

    if (globals->alloc_end - globals->alloc_pos < size) {
        post_and_halt(POST_CODE_DUMB_OOM);
    }

    ret = globals->alloc_pos;
    globals->alloc_pos += size;
    mem_set((void*)ret, 0, size);

    return ret;
}


static void pci_add_bar(PCIBarResouce** head, uint32_t bus, uint32_t device, uint bar,
                        uint64_t size)
{
    PCIBarResouce* item = (PCIBarResouce*)dumb_zalloc(sizeof(*item));
    item->bus = bus;
    item->device = device;
    item->bar = bar;
    item->size = size;

    while (*head && (*head)->size >= item->size) head = &(*head)->next;

    item->next = *head;
    *head = item;
}


static void pci_add_io_bar(uint32_t bus, uint32_t device, uint bar, uint32_t size)
{
    if (size > PCI_IO_MAX_SIZE || size < PCI_IO_MIN_SIZE) {
        post_and_halt(POST_CODE_BAR_SIZE_FAILED);
    }

    pci_add_bar((PCIBarResouce**)&globals->io_bars, bus, device, bar, size);
}


static void pci_add_32bit_bar(uint32_t bus, uint32_t device, uint bar, uint32_t size)
{
    if (size < PCI_MEM_MIN_SIZE) {
        post_and_halt(POST_CODE_BAR_SIZE_FAILED);
    }

    size = MAX(size, PAGE_SIZE);
    pci_add_bar((PCIBarResouce**)&globals->mem_bars, bus, device, bar, size);
    pci_add_bar((PCIBarResouce**)&globals->mem32_bars, bus, device, bar, size);
}


static void pci_add_64bit_bar(uint32_t bus, uint32_t device, uint bar, uint64_t size)
{
    if (size < PCI_MEM_MIN_SIZE) {
        post_and_halt(POST_CODE_BAR_SIZE_FAILED);
    }

    size = MAX(size, PAGE_SIZE);
    pci_add_bar((PCIBarResouce**)&globals->mem_bars, bus, device, bar, size);
    pci_add_bar((PCIBarResouce**)&globals->mem64_bars, bus, device, bar, size);
}


static void pci_add_exp_rom_bar(uint32_t bus, uint32_t device, uint32_t rom_size)
{
    if (rom_size > PCI_ROM_MAX_SIZE || rom_size < PCI_ROM_MIN_SIZE) {
        post_and_halt(POST_CODE_PCI_EXP_ROM_SIZE_INVALID);
    }

    rom_size = MAX(rom_size, PAGE_SIZE);
    pci_add_bar((PCIBarResouce**)&globals->mem_bars, bus, device, EXP_ROM_BAR, rom_size);
    pci_add_bar((PCIBarResouce**)&globals->mem32_bars, bus, device, EXP_ROM_BAR, rom_size);
}


static uint32_t pci_read_bar_for_size(uint32_t bus, uint32_t device, uint bar)
{
    uint bar_address;
    uint32_t original_val;
    uint32_t val;

    if (bar >= PCI_NUM_BARS) {
        post_and_halt(POST_CODE_BAR_INDEX_INVALID);
    }

    bar_address = PCI_OFFSET_BAR_0 + bar * 4;
    original_val = pci_read_32(bus, device, bar_address);
    pci_write_32(bus, device, bar_address , ~0);
    val = pci_read_32(bus, device, bar_address);
    pci_write_32(bus, device, bar_address , original_val);

    return val;
}


static void pci_get_class(uint32_t bus, uint32_t device, PCIDeviceType* type)
{
    uint32_t config_data = pci_read_32(bus, device, PCI_OFFSET_REVISION);

    type->prog_if = config_data >> 8;
    type->sub_class = config_data >> 16;
    type->class = config_data >> 24;
}


static int is_fixed_bar(uint32_t bus, uint32_t device, uint bar)
{
    PCIDeviceType type;

    if (bar > 3) {
        return FALSE;
    }

    pci_get_class(bus, device, &type);

    if (type.class == PCI_CLASS_MASS_STORAGE && type.sub_class == PCI_MASS_STORAGE_SUBCLASS_IDE) {
        uint compatibility_bit = (bar / 2) * 2;
        return !(type.prog_if & (1 << compatibility_bit));
    }

    return FALSE;
}


static void pci_mark_for_activation(uint32_t bus, uint32_t device)
{
    PCIDevDescriptor* descriptor = (PCIDevDescriptor*)dumb_zalloc(sizeof(*descriptor));
    PCIDevDescriptor** now;

    descriptor->bus = bus;
    descriptor->device = device;

    for (now = (PCIDevDescriptor**)&globals->activation_list; *now; now = &(*now)->next);

    *now = descriptor;
}


static int pci_enable_test(uint32_t bus, uint32_t device)
{
    PCIDeviceType type;

    pci_get_class(bus, device, &type);

    switch (type.class) {
    case PCI_CLASS_MASS_STORAGE:
        if (type.sub_class == PCI_MASS_STORAGE_SUBCLASS_IDE &&
               (~type.prog_if & 0x0a /*fixed mode of operation in primary or secondary channel*/)) {
            if (globals->stage1_flags & FLAGS_IDE_RES_WAS_CLAIMED) {
                return FALSE;
            }

            globals->stage1_flags |= FLAGS_IDE_RES_WAS_CLAIMED;
        }
        break;
    case PCI_CLASS_DISPLAY:
        if (type.sub_class == PCI_DISPLAY_SUBCLASS_VGA &&
                                                       type.prog_if == PCI_VGA_PROGIF_VGACOMPAT) {
            if (globals->stage1_flags & FLAGS_VGA_RES_WAS_CLAIMED) {
                return FALSE;
            }

            globals->stage1_flags |= FLAGS_VGA_RES_WAS_CLAIMED;
        }
        break;
    }

    return TRUE;
}


static int pci_collect_resources(uint32_t bus, uint32_t device, void __far * opaque)
{
    uint32_t exp_rom_bar;
    uint bar = 0;

    if (!pci_enable_test(bus, device)) {
        return FALSE;
    }

    pci_mark_for_activation(bus, device);

    for (; bar < PCI_NUM_BARS; bar++) {
        uint32_t val;

        if (is_fixed_bar(bus, device, bar)) {
            continue;
        }

        val = pci_read_bar_for_size(bus, device, bar);

        if (!val) {
            continue;
        }

        if ((val & PCI_BAR_IO_MASK)) {
            pci_add_io_bar(bus, device, bar, pci_bar_to_size(val));
        } else {
            uint mem_type = (val >> 1) & 0x3;
            switch (mem_type) {
            case PCI_MEMTYPE_32:
                pci_add_32bit_bar(bus, device, bar, pci_bar_to_size(val));
                break;
            case PCI_MEMTYPE_64: {
                uint64_t bar64 = pci_read_bar_for_size(bus, device, bar + 1);
                bar64 = (bar64 << 32) | val;
                pci_add_64bit_bar(bus, device, bar, pci_bar64_to_size(bar64));
                ++bar;
                break;
            }
            default:
                post_and_halt(POST_CODE_BAR_MEM_TYPE_INVALID);
            }
        }
    }

    pci_write_32(bus, device, PCI_OFFSET_ROM_ADDRESS, ~PCI_ROM_ENABLE_MASK);
    exp_rom_bar = pci_read_32(bus, device, PCI_OFFSET_ROM_ADDRESS);
    exp_rom_bar &= PCI_ROM_ADDRESS_MASK;
    pci_write_32(bus, device, PCI_OFFSET_ROM_ADDRESS, 0);

    if (exp_rom_bar) {
        uint32_t rom_size = pci_bar_to_size(exp_rom_bar & PCI_ROM_ADDRESS_MASK);
        pci_add_exp_rom_bar(bus, device, rom_size);
    }

    return FALSE;
}


static inline void early_map_platform_io()
{
    uint32_t io_bar;
    uint16_t command;
    uint32_t bar_size;

    pci_write_32(0, 0, PCI_OFFSET_BAR_0, ~0);
    io_bar = pci_read_32(0, 0, PCI_OFFSET_BAR_0);

    if (!(io_bar & PCI_BAR_IO_MASK)) {
        post_and_halt(POST_CODE_BAR_TYPE_FAILED);
    }

    bar_size = pci_bar_to_size(io_bar);

    if (bar_size < PLATFORM_IO_END) {
        post_and_halt(POST_CODE_BAR_SIZE_FAILED);
    }

    pci_write_32(0, 0, PCI_OFFSET_BAR_0, PCI_BAR_IO_MASK | PCI_BASE_IO_ADDRESS);
    command = pci_read_16(0, 0, PCI_OFFSET_COMMAND);
    pci_write_16(0, 0, PCI_OFFSET_COMMAND, command | PCI_COMMAND_ENABLE_IO);
}


static uint64_t to_power_of_two(uint64_t val)
{
    int msb = find_msb_32(val);
    uint64_t r;

    if (msb < 0) {
        return 0;
    }

    r = 1 << msb;

    return (r == val) ? r : (r << 1);
}


static void init_platform()
{
    uint8_t lock = inb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_LOCK);
    uint64_t below_4g_sum;

    if (lock) {
        post(POST_CODE_LOCKED);
        restart();
    }

    outb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_LOCK, 0);

    outb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_SELECT, PLATFORM_REG_BELOW_1M_USED_PAGES);
    globals->below_1m_used_pages = ind(PCI_BASE_IO_ADDRESS + PLATFORM_IO_REGISTER);

    outb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_SELECT, PLATFORM_REG_ABOVE_1M_PAGES);
    globals->above_1m_pages = ind(PCI_BASE_IO_ADDRESS + PLATFORM_IO_REGISTER);

    outb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_SELECT, PLATFORM_REG_BELOW_4G_PAGES);
    globals->below_4g_pages = ind(PCI_BASE_IO_ADDRESS + PLATFORM_IO_REGISTER);

    outb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_SELECT, PLATFORM_REG_BELOW_4G_USED_PAGES);
    globals->below_4g_used_pages = ind(PCI_BASE_IO_ADDRESS + PLATFORM_IO_REGISTER);

    outb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_SELECT, PLATFORM_REG_ABOVE_4G_PAGES);
    globals->above_4g_pages  = ind(PCI_BASE_IO_ADDRESS + PLATFORM_IO_REGISTER);

    below_4g_sum = globals->above_1m_pages + (MB >> PAGE_SHIFT);
    below_4g_sum = ALIGN(below_4g_sum, (MID_RAM_RANGE_ALIGMENT_MB * MB) >> PAGE_SHIFT);
    below_4g_sum += to_power_of_two(globals->below_4g_pages);

    if (globals->below_1m_used_pages > (MB - ((640 + 128) * KB) >> PAGE_SHIFT) ||
                                    globals->below_4g_used_pages > globals->below_4g_pages ||
                                    below_4g_sum > 4 * (GB >> PAGE_SHIFT)) {
        bios_error(BIOS_ERROR_INVALID_PLATFORM_ARGS);
    }

    globals->pci32_hole_start = (globals->above_1m_pages << PAGE_SHIFT) + MB;
    globals->pci32_hole_start = ALIGN(globals->pci32_hole_start, MID_RAM_RANGE_ALIGMENT_MB * MB);
    globals->pci32_hole_end = (GB >> PAGE_SHIFT) * 4 - to_power_of_two(globals->below_4g_pages);
    globals->pci32_hole_end <<= PAGE_SHIFT;

    globals->pci64_hole_start = 4ULL * GB + ((uint64_t)globals->above_4g_pages << PAGE_SHIFT);
    globals->pci64_hole_start = to_power_of_two(globals->pci64_hole_start);
}


static void pci_asign_io()
{
    uint32_t address = PCI_BASE_IO_ADDRESS;

    PCIBarResouce* item = (PCIBarResouce*)globals->io_bars;

    for (; item; item = item->next) {
        uint32_t bar_val;
        uint32_t bar_address;

        if (address + item->size > (1 << 16)) {
            post_and_halt(POST_CODE_PCI_OOM);
        }

        bar_address = PCI_OFFSET_BAR_0 + item->bar * 4;
        bar_val = pci_read_32(item->bus, item->device, bar_address);
        bar_val &= ~PCI_BAR_IO_ADDRESS_MASK;
        bar_val |= address;
        pci_write_32(item->bus, item->device, bar_address , bar_val);
        address += item->size;
    }
}


static inline PCIBarResouce* pci_first_unmapped(PCIBarResouce* bar_list)
{
    PCIBarResouce* item = bar_list;

    for (; item && item->mapped; item = item->next);

    return item;
}


static int pci_asign_mem32(PCIBarResouce* bar_list)
{
    uint32_t pci_hole_start = globals->pci32_hole_start;
    uint32_t pci_end_address = globals->pci32_hole_end;
    PCIBarResouce* item;
    uint32_t alloc_start;
    uint32_t address;

    for (;;) {
        if (!(item = pci_first_unmapped(bar_list))) {
            return TRUE;
        }

        if (item->size > pci_end_address - pci_hole_start) {
            return FALSE;
        }

        alloc_start = ALIGN(pci_hole_start, item->size);
        address = alloc_start;

        for (; item; item = item->next) {
            uint32_t bar_val;
            uint32_t bar_address;

            if (item->mapped || item->size > pci_end_address - address) {
                continue;
            }

            if (item->bar == EXP_ROM_BAR) {
                pci_write_32(item->bus, item->device, PCI_OFFSET_ROM_ADDRESS , address);
            } else {
                bar_address = PCI_OFFSET_BAR_0 + item->bar * 4;
                bar_val = pci_read_32(item->bus, item->device, bar_address);
                bar_val &= ~PCI_BAR_MEM_ADDRESS_MASK;
                bar_val |= address;
                pci_write_32(item->bus, item->device, bar_address , bar_val);

                if (((bar_val >> 1) & 0x3) == PCI_MEMTYPE_64) {
                    pci_write_32(item->bus, item->device, bar_address + 4 , 0);
                }
            }

            address += item->size;
            item->mapped = TRUE;
        }

        pci_end_address = alloc_start;
    }
}


static void pci_asign_mem64()
{
    PCIBarResouce* item = (PCIBarResouce*)globals->mem64_bars;
    uint64_t pci_hole_start;
    uint64_t pci_hole_end;
    uint64_t alloc_start;

    if (!item) {
        return;
    }

    pci_hole_start = globals->pci64_hole_start;
    pci_hole_end = 1ULL << globals->address_lines;

    if (item->size > pci_hole_end - pci_hole_start) {
        post_and_halt(POST_CODE_PCI_OOM);
    }

    alloc_start = ALIGN(pci_hole_start, item->size);

    for (; item; item = item->next) {
        uint32_t bar_val;
        uint32_t bar_address;

        if (item->size > pci_hole_end - alloc_start) {
            post_and_halt(POST_CODE_PCI_OOM);
        }

        bar_address = PCI_OFFSET_BAR_0 + item->bar * 4;
        bar_val = pci_read_32(item->bus, item->device, bar_address);
        bar_val &= ~PCI_BAR_MEM_ADDRESS_MASK;
        bar_val |= alloc_start;
        pci_write_32(item->bus, item->device, bar_address , bar_val);
        pci_write_32(item->bus, item->device, bar_address + 4, alloc_start >> 32);

        alloc_start += item->size;
    }
}


static void pci_activation()
{
    PCIDevDescriptor* descriptor = (PCIDevDescriptor*)globals->activation_list;

    for (; descriptor; descriptor = descriptor->next) {
        PCIDeviceType type;
        uint bus = descriptor->bus;
        uint device = descriptor->device;
        uint8_t interrupt_pin;

        pci_enable_device(bus, device, NULL);

        pci_get_class(bus, device, &type);

        switch (type.class) {
        case PCI_CLASS_MASS_STORAGE:
            if (type.sub_class == PCI_MASS_STORAGE_SUBCLASS_IDE) {
                if (type.prog_if & 0x80) {
                    uint16_t command = pci_read_16(bus, device, PCI_OFFSET_COMMAND);
                    uint16_t bm_io;

                    command |= PCI_COMMAND_BUS_MASTER;
                    pci_write_16(bus, device, PCI_OFFSET_COMMAND, command);

                    //set device 0 dma active for both primary and secondary channel.
                    //todo: use ATA_ID_CAP1_DMA_MASK
                    bm_io = pci_read_32(bus, device, PCI_OFFSET_BAR_4) & PCI_BAR_IO_ADDRESS_MASK;
                    outb(bm_io + 0x02, (1 << 5));
                    outb(bm_io + 0x0a, (1 << 5));
                }
            }
            break;
        }

        interrupt_pin = pci_read_8(bus, device, PCI_OFFSET_INTERRUPT_PIN);

        if (interrupt_pin) {
            post_and_halt(POST_CODE_TODO_UPDATE_INT_LINE);
        }
    }
}


static void init_pci()
{
    pci_for_each(pci_disable_device, NULL);
    pci_for_each(pci_collect_resources, NULL);
    pci_asign_io();

    if (!pci_asign_mem32((PCIBarResouce*)globals->mem_bars)) {
        if (!pci_asign_mem32((PCIBarResouce*)globals->mem32_bars)) {
            post_and_halt(POST_CODE_PCI_OOM);
        }

        pci_asign_mem64();
    }

    pci_activation();
}


static int pci_add_routing(uint32_t bus, uint32_t device, void __far * opaque)
{
    PrivateData* p = (PrivateData*)(globals->platform_ram + PLATFORM_BIOS_DATA_START);
    uint index = p->irq_routing_table_size;
    IRQOption *ent;

    if (index == sizeof(p->irq_routing_table) / sizeof(p->irq_routing_table[0])) {
        bios_error(BIOS_ERROR_OUT_OF_ROUTING_SLOTS);
    }

    ent = &p->irq_routing_table[index];

    ent->bus = bus;
    ent->device = device << 3;
    ent->int_a_link = index * 4;
    ent->int_b_link = index * 4 + 1;
    ent->int_c_link = index * 4 + 2;
    ent->int_d_link = index * 4 + 3;
    ent->int_a_bitmap = NOX_PCI_IRQ_LINES_MASK;
    ent->int_b_bitmap = ent->int_a_bitmap;
    ent->int_c_bitmap = ent->int_b_bitmap;
    ent->int_d_bitmap = ent->int_c_bitmap;
    ent->slot = 0;
    ent->reserved = 0;

    p->irq_routing_table_size++;

    return FALSE;
}


static void init_irq_routing()
{
    PrivateData* p = (PrivateData*)(globals->platform_ram + PLATFORM_BIOS_DATA_START);
    p->irq_routing_table_size = 0;

    pci_for_each(pci_add_routing, NULL);
}


static void enable_a20()
{
    uint8_t val = inb(IO_PORT_SYSCTRL);
    outb(IO_PORT_SYSCTRL, val | (1 << SYSCTRL_A20_BIT));
}


static void init_globals()
{
    uint32_t eax, ebx, ecx, edx;

    globals->alloc_start = DUMB_ALLOC_START;
    globals->alloc_end = DUMB_ALLOC_START + DUMB_ALLOC_SIZE;
    globals->alloc_pos = globals->alloc_start;
    globals->real_hard_int_ss = BIOS_HARD_INT_STACK_ADDRESS >> 4;
    globals->real_hard_int_sp = BIOS_HARD_INT_STACK_SIZE_KB * KB;

    eax = 0x80000008;
    cpuid(&eax, &ebx, &ecx, &edx);
    globals->address_lines = eax & 0xff;

    globals->rom_load_address = EXP_ROM_START_ADDRESS;
}


static void reset_platform_io()
{
    globals->platform_io = pci_read_32(0, 0, PCI_OFFSET_BAR_0) & PCI_BAR_IO_ADDRESS_MASK;
    globals->platform_ram = pci_read_32(0, 0, PCI_OFFSET_BAR_0 + 4) & PCI_BAR_MEM_ADDRESS_MASK;
}


static inline void init_cpu()
{
    uint32_t eax;
    uint32_t ebx;
    uint32_t ecx;
    uint32_t edx;
    uint32_t flags;

    post(POST_CODE_CPU);

    flags = get_eflags();
    put_eflags(flags ^ (1 << CPU_FLAGS_ID_BIT));

    if (get_eflags() == flags) {
        // no cpuid support
        platform_debug_string(__FUNCTION__ ": failed");
        freeze();
    }

    put_eflags(flags);

    eax = 1;
    cpuid(&eax, &ebx, &ecx, &edx);

    if ((edx & MANDATORY_CPU_FEATURES_MASK) != MANDATORY_CPU_FEATURES_MASK) {
        platform_debug_string(__FUNCTION__ ": failed");
        freeze();
    }

    *EBDA_BYTE(EBDA_OFFSET_CPU_FAMILY) = eax & 0xf;
    *EBDA_BYTE(EBDA_OFFSET_CPU_STEPPING) = (eax >> 8) & 0xf;

    if ((edx & (1 << CPU_FEATURE_FPU_BIT))) {
        *BDA_WORD(BDA_OFFSET_EQUIPMENT) |= (1 << BDA_EQUIPMENT_COPROCESSOR_BIT);
    }

    *EBDA_BYTE(EBDA_OFFSET_CACHE_CONTROL) = (get_cr0() & CR0_CD) ? 1 : 0;

    if (globals->address_lines > NOX_ADDRESS_LINES) {
        platform_debug_string(__FUNCTION__ ": address lines conflict");
        freeze();
    }
}


static void rtc_write(uint index, uint8_t val)
{
    outb(IO_PORT_RTC_INDEX, index | RTC_NMI_MASK);
    outb(IO_PORT_RTC_DATA, val);
}


static uint8_t rtc_read(uint index)
{
    outb(IO_PORT_RTC_INDEX, index | RTC_NMI_MASK);
    return inb(IO_PORT_RTC_DATA);
}


static inline void init_rtc()
{
    post(POST_CODE_RTC);

    // normal clock rate + clear periodic rate
    rtc_write(0x0a, (RTC_REG_A_DIVIDER_NORMAL << RTC_REG_A_DIVIDER_SHIFT));

    // activate clock + disable all timers and interrupts + BCD mode + preserv daylight
    rtc_write(0x0b, (rtc_read(0x0b) & RTC_REG_B_DAYLIGHT_MASK) | RTC_REG_B_24_HOUR_MASK);

    // rtc regs c and d will be reset on read
    rtc_read(0x0c);
    rtc_read(0x0d);

    // bios periodic timer rate is 1024hz
}


static void set_mttr_var_range(uint slot, uint8_t type, uint64_t address, uint64_t size)
{
    uint bits = globals->address_lines;
    uint msr_address;
    uint64_t mask;

    mask = ~(size - 1) & ((1ULL << bits) - 1);
    msr_address = MSR_MTRR_PHYS_BASE_0 + slot * 2;

    format_mem_str((char*)globals->platform_ram, PLATFORM_LOG_BUF_SIZE,
                   "mttr[%u] base 0x%llx mask 0x%llx size %llu%s",
                   slot,
                   address | type,
                   mask | MTRR_PHYS_MASK_VALID,
                   (size < MB) ? size / KB : size / MB,
                   (size < MB) ? "KB" : "MB");
    platform_notify_debug_string();
    write_msr(msr_address, address | type);
    write_msr(msr_address + 1, mask | MTRR_PHYS_MASK_VALID);
}


static void setup_mttr()
{
    uint64_t mtrr_cap;
    uint64_t mtrr_default;
    uint64_t size;
    uint mtrr_var_count;
    uint slot = 0;
    uint i;

    mtrr_cap = read_msr(MSR_MTRR_CAP);

    if (!(mtrr_cap & MTRR_CAP_FIX_MASK) || !(mtrr_cap & MTRR_CAP_WC_MASK)
        || (mtrr_var_count = (mtrr_cap & MTRR_CAP_COUNT_MASK)) < MTRR_MAX_VAR) {
        platform_debug_string(__FUNCTION__ ": unsuported MTRR");
        freeze();
    }

    mtrr_default = MTRR_DEFAULT_FIXED_ENABLE_MASK | MTRR_DEFAULT_ENABLE_MASK; // default type is UC

    // base mem type is WB
    write_msr(MSR_MTRR_FIX_64_0000, 0x0606060606060606ULL);
    write_msr(MSR_MTRR_FIX_16_8000, 0x0606060606060606ULL);

    // video frame buf type is UC
    write_msr(MSR_MTRR_FIX_16_A000, 0);

    for (i = MSR_MTRR_FIX_4_C000; i <= MSR_MTRR_FIX_4_F800; i++) {
        // bios and expention roms area WP (as if allowing modify rejection)
        write_msr(i, 0x0505050505050505ULL);
    }

    if (globals->above_1m_pages) {
        uint32_t bytes = (globals->above_1m_pages << PAGE_SHIFT) + MB;
        uint64_t address = 0;
        int msb;

        while (bytes) {
            if (slot == mtrr_var_count) {
                platform_debug_string(__FUNCTION__ ": out of var slot");
                freeze();
            }

            if (bytes < MID_RAM_RANGE_ALIGMENT_MB * MB) {
                bytes = to_power_of_two(bytes);
            }

            msb = find_msb_32(bytes);
            size = 1U << msb;

            set_mttr_var_range(slot++, 6, address, size);
            bytes -= size;
            address += size;
        }
    }

    if (mtrr_var_count - slot < (globals->above_4g_pages ? 2 : 1)) {
        platform_debug_string(__FUNCTION__ ": out of var slot");
        freeze();
    }

    size = to_power_of_two(globals->below_4g_used_pages) << PAGE_SHIFT;
    // high bios type is WB (maybe WP)
    set_mttr_var_range(slot++, 6, 4ULL * GB - size, size);

    if (globals->above_4g_pages) {
        size = to_power_of_two(globals->above_4g_pages) << PAGE_SHIFT;
        // ram above 4g type is WB
        set_mttr_var_range(slot, 6, 4ULL * GB, size);
    }

    write_msr(MSR_MTRR_DEFAULT, MTRR_DEFAULT_FIXED_ENABLE_MASK | MTRR_DEFAULT_ENABLE_MASK);
}


static inline void init_mem()
{
    uint32_t extended_memory_kb;

    post(POST_CODE_MEM);

    *BDA_WORD(BDA_OFFSET_MAIN_MEM_SIZE) = BASE_MEMORY_SIZE_KB - BIOS_EBDA_SIZE_KB;

    //640k base memory
    rtc_write(0x15, BASE_MEMORY_SIZE_KB);
    rtc_write(0x16, BASE_MEMORY_SIZE_KB >> 8);

    //extended memory
    extended_memory_kb = MIN(globals->above_1m_pages << 2, 63 * KB);
    rtc_write(0x17, extended_memory_kb);
    rtc_write(0x18, extended_memory_kb >> 8);
    rtc_write(0x30, extended_memory_kb);
    rtc_write(0x31, extended_memory_kb >> 8);

    setup_mttr();
}


static inline void init_pit()
{
    post(POST_CODE_PIT);

    outb(IO_PORT_MISC, 0); // timer 2 gate off + disable speaker

    //set sys timer (0) mode 2 tick == 54.9ms
    outb(IO_PORT_TIMER_CONTROL,
         (0 << PIT_SELECTOR_SHIFT) | (3 << PIT_RW_SHIFT) | (2 << PIT_MODE_SHIFT));
    outb(IO_PORT_TIMER_0, 0);
    outb(IO_PORT_TIMER_0, 0);

    //set dram refresh timer (1) mode 2 tick == 15us
    //nox imp update the timer on read so no unnecessary horse power is consumed
    outb(IO_PORT_TIMER_CONTROL,
         (1 << PIT_SELECTOR_SHIFT) | (3 << PIT_RW_SHIFT) | (2 << PIT_MODE_SHIFT));
    outb(IO_PORT_TIMER_1, 18);
    outb(IO_PORT_TIMER_1, 0);

    //set general use timer (2) mode 3 18.207 ticks/sec or tick == 54.9ms
    outb(IO_PORT_TIMER_CONTROL,
         (2 << PIT_SELECTOR_SHIFT) | (3 << PIT_RW_SHIFT) | (3 << PIT_MODE_SHIFT));
    outb(IO_PORT_TIMER_2, 0);
    outb(IO_PORT_TIMER_2, 0);
}


static inline void init_pic()
{
    post(POST_CODE_PIC);

    // reset master
    outb(IO_PORT_PIC1, PIC_ICW1_MASK | PIC_ICW1_ICW4);
    outb(IO_PORT_PIC1 + 1, PIC1_ADDRESS);
    outb(IO_PORT_PIC1 + 1, (1 << PIC1_SLAVE_PIN));
    outb(IO_PORT_PIC1 + 1, PIC_ICW4_8086_MODE);
    outb(IO_PORT_PIC1 + 1, 0xff); // mask all
    outb(IO_PORT_ELCR1, ~((1 << PIC1_TIMER_PIN) | (1 << PIC1_KEYBOARD_PIN) |
                          (1 << PIC1_SLAVE_PIN)));

    // reset slave
    outb(IO_PORT_PIC2, PIC_ICW1_MASK | PIC_ICW1_ICW4);
    outb(IO_PORT_PIC2 + 1, PIC2_ADDRESS);
    outb(IO_PORT_PIC2 + 1, PIC1_SLAVE_PIN);
    outb(IO_PORT_PIC2 + 1, PIC_ICW4_8086_MODE);
    outb(IO_PORT_PIC2 + 1, 0xff); // mask all
    outb(IO_PORT_ELCR2, ~((1 << PIC2_RTC_PIN) | (1 << PIC2_DMA_PIN)));

    outb(IO_PORT_PIC1 + 1, ~0x04); // unmask slave
}


static bool_t load_expansion_rom(uint8_t *address, uint32_t size, uint bus, uint device,
                                 PCIDeviceType* type)
{
    ASSERT((size % PCI_ROM_GRANULARITY) == 0);

    size /= PCI_ROM_GRANULARITY;

    while (size) {
        CodeImageHeader* header = (CodeImageHeader*)address;
        PCIExpRomData* pci_data;
        uint image_size;
        uint8_t* dest;
        uint8_t* end;

        if (header->signature != EXP_ROM_SIGNATURE) {
            D_MESSAGE("0x%x.0x%x: invalid rom signature", bus, device);
            return FALSE;
        }

        if (header->code_image_size > size) {
            D_MESSAGE("0x%x.0x%x: invalid size", bus, device);
            return FALSE;
        }

        if (header->pci_data_offset + sizeof(PCIExpRomData) > size * PCI_ROM_GRANULARITY) {
            D_MESSAGE("0x%x.0x%x: invalid offset to pci struct", bus, device);
            return FALSE;
        }

        pci_data = (PCIExpRomData*)(address + header->pci_data_offset);

        if (pci_data->signature != PCI_ROM_SIGNATURE ||
            pci_data->struct_size != sizeof(PCIExpRomData) ||
            pci_data->revision != 0 ||
            pci_data->code_image_size != header->code_image_size)
        {
            D_MESSAGE("0x%x.0x%x:: invalid pci struct", bus, device);
            return FALSE;
        }

        image_size = header->code_image_size * PCI_ROM_GRANULARITY;

        if (pci_data->vendor_id != pci_read_16(bus, device, PCI_OFFSET_VENDOR) ||
            pci_data->device_id != pci_read_16(bus, device, PCI_OFFSET_DEVICE) ||
            pci_data->class != type->class ||
            pci_data->sub_class != type->sub_class ||
            pci_data->prog_if != type->prog_if ||
            pci_data->code_type != 0)
        {
            if (pci_data->flags && (1 << 7)) {
                break;
            }

            address += image_size;
            size -= header->code_image_size;
            continue;
        }

        D_MESSAGE("0x%x.0x%x: found", bus, device);

        dest = (uint8_t*)get_ebda_private()->rom_load_address;
        end = (uint8_t*)BIOS32_START_ADDRESS;

        if (image_size > end - dest) {
            D_MESSAGE("0x%x.0x%x: no space", bus, device);
            return FALSE;
        }

        mem_copy(dest, address, image_size);
        get_ebda_private()->rom_load_address += image_size;
        get_ebda_private()->loaded_rom_address = (uint32_t)dest;
        get_ebda_private()->loaded_rom_bus = bus;
        get_ebda_private()->loaded_rom_device = device;
        D_MESSAGE("0x%x.0x%x: @0x%x", bus, device, dest);
        return TRUE;
    }

    return FALSE;
}


static  int load_rom_cb(uint32_t bus, uint32_t device, void __far * opaque)
{
    uint8_t __far * index = opaque;
    uint32_t exp_rom_address;
    uint32_t exp_rom_size;
    PCIDeviceType type;
    uint32_t class;
    uint8_t ok;

    class = pci_read_32(bus, device, PCI_OFFSET_REVISION);
    type.class = class >> 24;
    type.sub_class = (class >> 16) & 0xff;
    type.prog_if = (class >> 8) & 0xff;

    if (type.class == PCI_CLASS_DISPLAY && type.sub_class == PCI_DISPLAY_SUBCLASS_VGA &&
                                                        type.prog_if == PCI_VGA_PROGIF_VGACOMPAT) {
        return FALSE;
    }

    exp_rom_address = pci_read_32(bus, device, PCI_OFFSET_ROM_ADDRESS);

    if (!exp_rom_address) {
        return FALSE;
    }

    if (*index < get_ebda_private()->rom_load_index) {
        *index++;
        return FALSE;
    }

    pci_write_32(bus, device, PCI_OFFSET_ROM_ADDRESS, ~PCI_ROM_ENABLE_MASK);
    exp_rom_size = pci_read_32(bus, device, PCI_OFFSET_ROM_ADDRESS) & PCI_ROM_ADDRESS_MASK;
    exp_rom_size = pci_bar_to_size(exp_rom_size);
    pci_write_32(bus, device, PCI_OFFSET_ROM_ADDRESS, exp_rom_address | PCI_ROM_ENABLE_MASK);

    ASSERT(exp_rom_size);

    ok = load_expansion_rom((uint8_t*)exp_rom_address, exp_rom_size, bus, device, &type);

    pci_write_32(bus, device, PCI_OFFSET_ROM_ADDRESS, exp_rom_address);
    get_ebda_private()->call_ret_val = ok;
    get_ebda_private()->rom_load_index++;
    pci_write_32(bus, device, PCI_OFFSET_ROM_ADDRESS, exp_rom_address);

    return TRUE;
}


static void load_rom()
{
    uint8_t index = 0;
    get_ebda_private()->call_ret_val = FALSE;
    pci_for_each(load_rom_cb, &index);
}


static void load_vga()
{
    PCIDeviceType type;
    uint index;
    uint bus;
    uint device;

    NO_INTERRUPT();

    type.class = PCI_CLASS_DISPLAY;
    type.sub_class = PCI_DISPLAY_SUBCLASS_VGA;
    type.prog_if = PCI_VGA_PROGIF_VGACOMPAT;

    index = 0;
    for (;; index++) {
        uint32_t exp_rom_address;
        uint32_t exp_rom_size;
        bool_t ok;

        if (!pci_find_class(index, &type, &bus, &device)) {
            get_ebda_private()->call_ret_val = FALSE;
            D_MESSAGE("not found");
            return;
        }

        if (!pci_is_io_enabled(bus, device)) {
            continue;
        }

        D_MESSAGE("found");

        ASSERT(pci_is_mem_enabled(bus, device));

        exp_rom_address = pci_read_32(bus, device, PCI_OFFSET_ROM_ADDRESS);
        exp_rom_address &= PCI_ROM_ADDRESS_MASK;

        if (!exp_rom_address) {
            bios_error(BIOS_ERROR_NO_VGA_EXP_ROM);
        }

        pci_write_32(bus, device, PCI_OFFSET_ROM_ADDRESS, ~PCI_ROM_ENABLE_MASK);
        exp_rom_size = pci_read_32(bus, device, PCI_OFFSET_ROM_ADDRESS) & PCI_ROM_ADDRESS_MASK;
        exp_rom_size = pci_bar_to_size(exp_rom_size);
        pci_write_32(bus, device, PCI_OFFSET_ROM_ADDRESS, exp_rom_address | PCI_ROM_ENABLE_MASK);

        ASSERT(exp_rom_size);

        ok = load_expansion_rom((uint8_t*)exp_rom_address, exp_rom_size, bus, device, &type);
        pci_write_32(bus, device, PCI_OFFSET_ROM_ADDRESS, exp_rom_address);
        get_ebda_private()->call_ret_val = ok;

        return;
    }
}


static void init()
{
    post(POST_CODE_INIT32);

    enable_a20();
    init_globals();
    detect_platform();
    pci_for_each(pci_disable_device, NULL);
    early_map_platform_io();
    init_platform();
    init_pci();
    reset_platform_io();

    platform_debug_string("hello :)");

    ASSERT(sizeof(EBDA) <= BIOS_EBDA_DATA_KB * KB);
    ASSERT(*(uint16_t*)(bda + BDA_OFFSET_EBDA) == (BIOS_EBDA_ADDRESS >> 4));
    ASSERT(OFFSET_OF(EBDAPrivate, real_mode_ss) == PRIVATE_OFFSET_REAL_MODE_SS);
    ASSERT(OFFSET_OF(EBDAPrivate, real_mode_sp) == PRIVATE_OFFSET_REAL_MODE_SP);
    ASSERT(OFFSET_OF(EBDAPrivate, real_user_ss) == PRIVATE_OFFSET_USER_SS);
    ASSERT(OFFSET_OF(EBDAPrivate, real_user_sp) == PRIVATE_OFFSET_USER_SP);
    ASSERT(OFFSET_OF(EBDAPrivate, real_hard_int_ss) == PRIVATE_OFFSET_HARD_INT_SS);
    ASSERT(OFFSET_OF(EBDAPrivate, real_hard_int_sp) == PRIVATE_OFFSET_HARD_INT_SP);
    ASSERT(OFFSET_OF(EBDAPrivate, bios_flags) == PRIVATE_OFFSET_FLAGS);

    DBG_MESSAGE("sizeof(EBDA) is %u", sizeof(EBDA));

    init_cpu();
    init_rtc();
    init_mem();
    init_pit();
    init_pic();
    init_irq_routing();

    get_ebda_private()->call_ret_val = TRUE;
}


void entry()
{
    switch (get_ebda_private()->call_select) {
    case CALL_SELECT_INIT:
        init();
        break;
    case CALL_SELECT_LOAD_VGA:
        load_vga();
        break;
    case CALL_SELECT_LOAD_EXP_ROM:
        load_rom();
        break;
    case CALL_SELECT_NOP:
        get_ebda_private()->call_ret_val = TRUE;
        break;
    default:
        bios_error(BIOS_ERROR_UNEXPECTED_IP);
    }

    ret_16();
}

