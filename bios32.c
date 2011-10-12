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

#include "common.c"


#define ASSERT(x) if (!(x)) { halt();}


void halt(void);

enum {
    PCI_ADDRESS_INDEX_SHIFT = 2,
    PCI_ADDRESS_INDEX_BITS = 6,
    PCI_ADDRESS_FUNCTION_SHIFT = 8,
    PCI_ADDRESS_FUNCTION_BITS = 3,
    PCI_ADDRESS_DEVICE_SHIFT = 11,
    PCI_ADDRESS_DEVICE_BITS = 5,
    PCI_ADDRESS_BUS_SHIFT = 16,
    PCI_ADDRESS_BUS_BITS = 8,
    PCI_ADDRESS_ENABLED_MASK = (1 << 31),
    PCI_ADDRESS_MASK = ~((1 << 2) - 1),

    PCI_BUS_MASK = (1 << PCI_ADDRESS_BUS_BITS) - 1,
    PCI_BUS_MAX = PCI_BUS_MASK,
    PCI_DEVICE_MASK = (1 << PCI_ADDRESS_DEVICE_BITS) - 1,
    PCI_DEVICE_MAX = PCI_DEVICE_MASK,
    PCI_FUNCTION_MASK = (1 << PCI_ADDRESS_FUNCTION_BITS) - 1,
    PCI_FUNCTION_MAX = PCI_FUNCTION_MASK,
    PCI_INDEX_MASK = (1 << PCI_ADDRESS_INDEX_BITS) - 1,
    PCI_INDEX_MAX = PCI_INDEX_MASK,

    PCI_VENDOE_INVALID = 0xffff,

    PCI_OFFSET_VENDOR = 0x00,
    PCI_OFFSET_DEVICE = 0x02,
    PCI_OFFSET_COMMAND = 0x04,
    PCI_OFFSET_STATUS = 0x06,
    PCI_OFFSET_REVISION = 0x08,
    PCI_OFFSET_CLASS = 0x09,
    PCI_OFFSET_CACHE_LINE = 0x0c,
    PCI_OFFSET_CACHE_LATENCY = 0x0d,
    PCI_OFFSET_BAR_0 = 0x10,
    PCI_OFFSET_BAR_1 = 0x14,
    PCI_OFFSET_BAR_2 = 0x18,
    PCI_OFFSET_BAR_3 = 0x1c,
    PCI_OFFSET_BAR_4 = 0x20,
    PCI_OFFSET_BAR_5 = 0x24,
    PCI_OFFSET_SUBSYS_VENDOR = 0x2c,
    PCI_OFFSET_SUBSYS_ID = 0x2e,
    PCI_OFFSET_ROM_ADDRESS = 0x30,
    PCI_OFFSET_INTERRUPT_LINE = 0x3c,
    PCI_OFFSET_INTERRUPT_PIN = 0x3d,

    PCI_COMMAND_ENABLE_IO = (1 << 0),
    PCI_COMMAND_ENABLE_MEM = (1 << 1),
    PCI_COMMAND_DISABLE_INTERRUPT = (1 << 10),

    PCI_BAR_IO_MASK = (1 << 0),

    PCI_BASE_IO_ADDRESS = 0xc000,
};


#define NOX_PCI_VENDOR_ID 0x1aaa
#define NOX_PCI_DEV_ID_HOST_BRIDGE 0x0001
#define NOX_PCI_DEV_HOST_BRIDGE_REV 1

enum {
    PLATFORM_IO_LOCK = 0x00,
    PLATFORM_IO_INDEX = 0x01,
    PLATFORM_IO_ERROR = 0x04,
    PLATFORM_IO_DATA = 0x08,
    PLATFORM_IO_END = 0x0c,
};

enum {
    PLATFORM_REG_BELOW_1M_USED_PAGES,
    PLATFORM_REG_ABOVE_1M_PAGES,
    PLATFORM_REG_BELOW_4G_PAGES,
    PLATFORM_REG_BELOW_4G_USED_PAGES,
    PLATFORM_REG_ABOVE_4G_PAGES,
};


static inline void post_and_halt(uint8_t code)
{
    post(code);
    halt();
}


static inline void outd(uint16_t port, uint32_t val)
{
    _asm {
        mov dx, port
        mov eax, val
        out dx, eax
    }
}

static inline uint32_t ind(uint16_t port)
{
    uint32_t val;

    _asm {
        mov dx, port
        in eax, dx
        mov val, eax
    }

    return val;
}


static inline uint32_t pci_config_address(uint32_t bus, uint32_t device, uint32_t index)
{
    uint32_t function = 0;

    ASSERT(bus <= PCI_BUS_MAX && device <= PCI_DEVICE_MAX && index <= PCI_INDEX_MAX);

    return  PCI_ADDRESS_ENABLED_MASK |
            ((bus & PCI_BUS_MAX) << PCI_ADDRESS_BUS_SHIFT) |
            ((device & PCI_DEVICE_MAX) << PCI_ADDRESS_DEVICE_SHIFT) |
            ((function & PCI_FUNCTION_MAX)  << PCI_ADDRESS_FUNCTION_SHIFT) |
            ((index & PCI_INDEX_MAX) << PCI_ADDRESS_INDEX_SHIFT);
}


static uint32_t pci_read_32(uint32_t bus, uint32_t device, uint32_t offset)
{
    ASSERT((offset & 0x3) == 0);
    outd(IO_PORT_PCI_ADDRESS, pci_config_address(bus, device, offset >> 2));
    return ind(IO_PORT_PCI_DATA);
}


static void pci_write_32(uint32_t bus, uint32_t device, uint32_t offset, uint32_t val)
{
    ASSERT((offset & 0x3) == 0);
    outd(IO_PORT_PCI_ADDRESS, pci_config_address(bus, device, offset >> 2));
    outd(IO_PORT_PCI_DATA, val);
}


static uint16_t pci_read_16(uint32_t bus, uint32_t device, uint32_t offset)
{
    ASSERT((offset & 0x1) == 0);
    outd(IO_PORT_PCI_ADDRESS, pci_config_address(bus, device, offset >> 2));
    return inw(IO_PORT_PCI_DATA + (offset & 0x2));
}


static void pci_write_16(uint32_t bus, uint32_t device, uint32_t offset, uint16_t val)
{
    ASSERT((offset & 0x1) == 0);
    outd(IO_PORT_PCI_ADDRESS, pci_config_address(bus, device, offset >> 2));
    outw(IO_PORT_PCI_DATA + (offset & 0x2), val);
}


static uint16_t pci_read_8(uint32_t bus, uint32_t device, uint32_t offset)
{
    outd(IO_PORT_PCI_ADDRESS, pci_config_address(bus, device, offset >> 2));
    return inb(IO_PORT_PCI_DATA + (offset & 0x3));
}


static void pci_write_8(uint32_t bus, uint32_t device, uint32_t offset, uint8_t val)
{
    outd(IO_PORT_PCI_ADDRESS, pci_config_address(bus, device, offset >> 2));
    outb(IO_PORT_PCI_DATA + (offset & 0x3), val);
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


typedef int (*pci_for_each_cb)(uint32_t bus, uint32_t device);

static void pci_for_each(pci_for_each_cb cb)
{
    uint32_t bus;
    uint32_t device;

    for (bus = 0; bus <= PCI_BUS_MAX; bus++) {
        for (device = 0; device <= PCI_DEVICE_MAX; device++) {
            uint16_t vendor_id = pci_read_16(bus, device, PCI_OFFSET_VENDOR);

            if (vendor_id == PCI_VENDOE_INVALID) {
                continue;
            }

            if (cb(bus, device)) {
                return;
            }
        }
    }
}


static int pci_disable_device(uint32_t bus, uint32_t device)
{
    uint16_t command = pci_read_16(bus, device, PCI_OFFSET_COMMAND);
    command &= ~(PCI_COMMAND_ENABLE_IO | PCI_COMMAND_ENABLE_MEM);
    command |= PCI_COMMAND_DISABLE_INTERRUPT;
    pci_write_16(bus, device, PCI_OFFSET_COMMAND, command);
    return FALSE;
}


static inline uint32_t pci_bar_to_size(uint32_t bar)
{
    uint32_t mask = (bar & PCI_BAR_IO_MASK) ? 0x3 : 0xf;
    return (~bar | mask) + 1;
}


static inline void map_platform_io()
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


typedef struct Globals {
    uint32_t below_1m_used_pages;
    uint32_t above_1m_pages;
    uint32_t below_4g_pages;
    uint32_t below_4g_used_pages;
    uint32_t above_4g_pages;
} Globals;

static Globals* globals = (Globals*)0x00000500;

static void init_platform()
{
    uint8_t lock = inb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_LOCK);

    if (lock) {
        post(POST_CODE_LOCKED);
        restart();
    }

    outb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_INDEX, PLATFORM_REG_BELOW_1M_USED_PAGES);
    globals->below_1m_used_pages = ind(PCI_BASE_IO_ADDRESS + PLATFORM_IO_DATA);

    outb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_INDEX, PLATFORM_REG_ABOVE_1M_PAGES);
    globals->above_1m_pages = ind(PCI_BASE_IO_ADDRESS + PLATFORM_IO_DATA);

    outb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_INDEX, PLATFORM_REG_BELOW_4G_PAGES);
    globals->below_4g_pages = ind(PCI_BASE_IO_ADDRESS + PLATFORM_IO_DATA);

    outb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_INDEX, PLATFORM_REG_BELOW_4G_USED_PAGES);
    globals->below_4g_used_pages = ind(PCI_BASE_IO_ADDRESS + PLATFORM_IO_DATA);

    outb(PCI_BASE_IO_ADDRESS + PLATFORM_IO_INDEX, PLATFORM_REG_ABOVE_4G_PAGES);
    globals->above_4g_pages  = ind(PCI_BASE_IO_ADDRESS + PLATFORM_IO_DATA);
}


static void globals_test()
{
    outd(PCI_BASE_IO_ADDRESS + PLATFORM_IO_ERROR, 0);
    outd(PCI_BASE_IO_ADDRESS + PLATFORM_IO_ERROR, globals->below_1m_used_pages);
    outd(PCI_BASE_IO_ADDRESS + PLATFORM_IO_ERROR, globals->above_1m_pages);
    outd(PCI_BASE_IO_ADDRESS + PLATFORM_IO_ERROR, globals->below_4g_pages);
    outd(PCI_BASE_IO_ADDRESS + PLATFORM_IO_ERROR, globals->below_4g_used_pages);
    outd(PCI_BASE_IO_ADDRESS + PLATFORM_IO_ERROR, globals->above_4g_pages);
    halt();
}

void init()
{
    post(POST_CODE_INIT32);
    detect_platform();
    pci_for_each(pci_disable_device);
    map_platform_io();
    init_platform();

    globals_test();

    restart();
}

