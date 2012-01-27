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
#include "utils.h"
#include "pci.h"
#include "nox.h"
#include "platform.h"
#include "pcibios.h"


void pci_for_each(pci_for_each_cb cb, void __far * opaque)
{
    uint32_t bus;
    uint32_t device;

    for (bus = 0; bus <= PCI_BUS_MAX; bus++) {
        for (device = 0; device <= PCI_DEVICE_MAX; device++) {
            uint16_t vendor_id = pci_read_16(bus, device, PCI_OFFSET_VENDOR);

            if (vendor_id == PCI_VENDOE_INVALID) {
                continue;
            }

            if (cb(bus, device, opaque)) {
                return;
            }
        }
    }
}


typedef struct PCIFindDeviceInfo {
    uint16_t vendor_id;
    uint16_t device_id;
    uint16_t request_index;
    uint16_t current_index;
    uint16_t bus;
    uint16_t device;
    uint16_t function;
} PCIFindDeviceInfo;


uint32_t pci_config_address(uint32_t bus, uint32_t device, uint32_t index)
{
    uint32_t function = 0;

    ASSERT(bus <= PCI_BUS_MAX && device <= PCI_DEVICE_MAX && index <= PCI_INDEX_MAX);

    return  PCI_ADDRESS_ENABLED_MASK |
            ((bus & PCI_BUS_MAX) << PCI_ADDRESS_BUS_SHIFT) |
            ((device & PCI_DEVICE_MAX) << PCI_ADDRESS_DEVICE_SHIFT) |
            ((function & PCI_FUNCTION_MAX)  << PCI_ADDRESS_FUNCTION_SHIFT) |
            ((index & PCI_INDEX_MAX) << PCI_ADDRESS_INDEX_SHIFT);
}


uint32_t pci_read_32(uint bus, uint device, uint offset)
{
    ASSERT((offset & 0x3) == 0);
    outd(IO_PORT_PCI_ADDRESS, pci_config_address(bus, device, offset >> 2));
    return ind(IO_PORT_PCI_DATA);
}


void pci_write_32(uint bus, uint device, uint offset, uint32_t val)
{
    ASSERT((offset & 0x3) == 0);
    outd(IO_PORT_PCI_ADDRESS, pci_config_address(bus, device, offset >> 2));
    outd(IO_PORT_PCI_DATA, val);
}


uint16_t pci_read_16(uint bus, uint device, uint offset)
{
    ASSERT((offset & 0x1) == 0);
    outd(IO_PORT_PCI_ADDRESS, pci_config_address(bus, device, offset >> 2));
    return inw(IO_PORT_PCI_DATA + (offset & 0x2));
}


void pci_write_16(uint bus, uint device, uint offset, uint16_t val)
{
    ASSERT((offset & 0x1) == 0);
    outd(IO_PORT_PCI_ADDRESS, pci_config_address(bus, device, offset >> 2));
    outw(IO_PORT_PCI_DATA + (offset & 0x2), val);
}


uint8_t pci_read_8(uint bus, uint device, uint offset)
{
    outd(IO_PORT_PCI_ADDRESS, pci_config_address(bus, device, offset >> 2));
    return inb(IO_PORT_PCI_DATA + (offset & 0x3));
}


void pci_write_8(uint bus, uint device, uint offset, uint8_t val)
{
    outd(IO_PORT_PCI_ADDRESS, pci_config_address(bus, device, offset >> 2));
    outb(IO_PORT_PCI_DATA + (offset & 0x3), val);
}


static int pci_find_device_cb(uint bus, uint device, void __far * opaque)
{
    PCIFindDeviceInfo* info = (PCIFindDeviceInfo*)opaque;
    uint16_t device_id;

    uint16_t vendor_id = pci_read_16(bus, device, PCI_OFFSET_VENDOR);

    if (vendor_id != info->vendor_id) {
        return FALSE;
    }

    device_id = pci_read_16(bus, device, PCI_OFFSET_DEVICE);

    if (device_id != info->device_id) {
        return FALSE;
    }

    if (info->current_index < info->request_index) {
        info->current_index++;
        return FALSE;
    }

    info->bus = 0;
    info->function = 0;
    info->device = device;

    return TRUE;
}


typedef struct PCIFindClassInfo {
    uint32_t class;
    uint16_t request_index;
    uint16_t current_index;
    uint16_t bus;
    uint16_t device;
    uint16_t function;
} PCIFindClassInfo;


static int pci_find_class_cb(uint bus, uint device, void __far * opaque)
{
    PCIFindClassInfo* info = (PCIFindClassInfo*)opaque;
    uint16_t device_id;

    uint32_t class = pci_read_32(bus, device, PCI_OFFSET_REVISION);

    if ((class >> 8) != info->class) {
        return FALSE;
    }

    if (info->current_index < info->request_index) {
        info->current_index++;
        return FALSE;
    }

    info->bus = 0;
    info->function = 0;
    info->device = device;

    return TRUE;
}


bool_t pci_is_io_enabled(uint bus, uint device)
{
    NO_INTERRUPT();

    return !!(pci_read_32(bus, device, PCI_OFFSET_COMMAND) & PCI_COMMAND_ENABLE_IO);
}


bool_t pci_is_mem_enabled(uint bus, uint device)
{
    NO_INTERRUPT();

    return !!(pci_read_32(bus, device, PCI_OFFSET_COMMAND) & PCI_COMMAND_ENABLE_MEM);
}


bool_t pci_find_class(uint index, PCIDeviceType __far * type, uint __far * bus,
                       uint __far * device)
{
    PCIFindClassInfo info;
    info.class = ((uint32_t)type->class << 16) | ((uint32_t)type->sub_class << 8) | type->prog_if;
    info.request_index = index;
    info.current_index = 0;
    info.bus = 0xffff;

    pci_for_each(pci_find_class_cb, &info);

    if (info.bus != 0xffff) {
        *bus = info.bus;
        *device = info.device;
        return TRUE;
    } else {
        return FALSE;
    }
}


void pci_get_class(uint bus, uint device, PCIDeviceType __far * type)
{
    uint32_t config_data = pci_read_32(bus, device, PCI_OFFSET_REVISION);

    type->prog_if = config_data >> 8;
    type->sub_class = config_data >> 16;
    type->class = config_data >> 24;
}


static void pcibios_get_irq_option(UserRegs __far * context)
{
    IRQRoutingOptionBuffer __far * buff;
    IRQOption __far * dest;
    uint required_size;
    uint8_t table_size;
    uint i;

    D_MESSAGE("start");

    BIOS_PRIVATE_READ(irq_routing_table_size, &table_size);
    required_size = table_size * sizeof(IRQOption);

    buff = FAR_POINTER(IRQRoutingOptionBuffer, context->es, (offset_t)context->edi);

    BX(context) = NOX_PCI_IRQ_EXCLUSIVE_MASK;

    if (required_size > buff->size) {
        D_MESSAGE("0x%x is too small, 0x%x needed", (uint)buff->size, required_size);
        buff->size = required_size;
        AH(context) = PCIBIOS_BUFFER_TOO_SMALL;
        return;
    }

    buff->size = required_size;

    dest = FAR_POINTER(IRQOption, buff->seg, buff->offset);

    for (i = 0; i < table_size; i++) {
        BIOS_PRIVATE_READ( irq_routing_table[i], &dest[i]);
    }

    AH(context) = PCIBIOS_SUCCESSFUL;
    FLAGS(context) &= ~(1 << CPU_FLAGS_CF_BIT);
}


void pcibios_set_irq(UserRegs __far * context)
{
    PCmdSetIRQ args;
    uint function;

    args.pin = CL(context);
    args.irq = CH(context);
    args.bus = BH(context);
    args.device = BL(context) >> 3;
    function = BL(context) & 0x7;

    if (args.pin < 0x0a || args.pin > 0x0d || args.irq > 15 || function || args.bus) {
        D_MESSAGE("failed ebx 0x%lx ecx 0x%lx", context->ebx, context->ecx);
        AH(context) = PCIBIOS_SET_FAILED;
    }

    args.ret_val = 0;
    platform_command(PALTFORM_CMD_SET_PCI_IRQ, &args, sizeof(args));

    if (!args.ret_val) {
        D_MESSAGE("config failed");
        AH(context) = PCIBIOS_SET_FAILED;
    } else {
        AH(context) = PCIBIOS_SUCCESSFUL;
        FLAGS(context) &= ~(1 << CPU_FLAGS_CF_BIT);
    }
}


void pcibios_service(UserRegs __far * context)
{
    NO_INTERRUPT();

    D_MESSAGE("al 0x%x", AL(context));

    FLAGS(context) |= (1 << CPU_FLAGS_CF_BIT);

    if (AH(context) != PCIBIOS_FUNC) {
        D_MESSAGE("0x%x is not pcibios function", AH(context));
        AH(context) = PCIBIOS_FUNC_NOT_SUPPORTED;
        FLAGS(context) |= (1 << CPU_FLAGS_CF_BIT);
        return;
    }

    switch (AL(context)) {
    case PCIBIOS_FUNC_PRESENT:
        context->edx = FOUR_CHARS('PCI ');

        // pci present
        AH(context) = 0;

        // config access mechanism #1
        AL(context) = (1 << 0);

        // version 2.1 (BCD)
        BH(context) = 0x02;
        BL(context) = 0x01;

        // number of last bus
        CL(context) = 0;

        FLAGS(context) &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    case PCIBIOS_FUNC_FIND_DEVICE: {
        PCIFindDeviceInfo info;
        info.vendor_id = DX(context);
        info.device_id = CX(context);
        info.request_index = SI(context);
        info.current_index = 0;
        info.bus = 0xffff;

        if (info.vendor_id == 0xffff) {
            D_MESSAGE("find device: invalid vendor id");
            AH(context) = PCIBIOS_BAD_VANDOR_ID;
            break;
        }

        pci_for_each(pci_find_device_cb, &info);

        if (info.bus != 0xffff) {
            BH(context) = info.bus;
            BL(context) = (info.device << 3) | info.function;
            AH(context) = PCIBIOS_SUCCESSFUL;
            FLAGS(context) &= ~(1 << CPU_FLAGS_CF_BIT);
        } else {
            AH(context) = PCIBIOS_DEVICE_NOT_FOUND;
        }

        break;
    }
    case PCIBIOS_FUNC_FIND_CLASS: {
        PCIFindClassInfo info;
        info.class = context->ecx;
        info.request_index = SI(context);
        info.current_index = 0;
        info.bus = 0xffff;

        pci_for_each(pci_find_class_cb, &info);

        if (info.bus != 0xffff) {
            BH(context) = info.bus;
            BL(context) = (info.device << 3) | info.function;
            AH(context) = PCIBIOS_SUCCESSFUL;
            FLAGS(context) &= ~(1 << CPU_FLAGS_CF_BIT);
        } else {
            AH(context) = PCIBIOS_DEVICE_NOT_FOUND;
        }

        break;
    }
    case PCIBIOS_FUNC_READ_BYTE: {
        uint address = DI(context);

        if (address > 255) {
            D_MESSAGE("rb: bad address 0x%x", address);
            AH(context) = PCIBIOS_BAD_REGISTER;
            break;
        }

        if ((BL(context) & 0x7)) {
            CL(context) = 0xff;
        } else {
            CL(context) = pci_read_8(BH(context), BL(context) >> 3, address);
        }

        AH(context) = PCIBIOS_SUCCESSFUL;
        FLAGS(context) &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case PCIBIOS_FUNC_READ_WORD: {
        uint address = DI(context);

        if (address > 255 || (address & 0x01)) {
            D_MESSAGE("rw: bad address 0x%x", address);
            AH(context) = PCIBIOS_BAD_REGISTER;
            break;
        }

        if ((BL(context) & 0x7)) {
            CX(context) = 0xffff;
        } else {
            CX(context) = pci_read_16(BH(context), BL(context) >> 3, address);
        }

        AH(context) = PCIBIOS_SUCCESSFUL;
        FLAGS(context) &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case PCIBIOS_FUNC_READ_DWORD: {
        uint address = DI(context);

        if (address > 255 || (address & 0x03)) {
            D_MESSAGE("rd: bad address 0x%x", address);
            AH(context) = PCIBIOS_BAD_REGISTER;
            break;
        }

        if ((BL(context) & 0x7)) {
            context->ecx = 0xffffffff;
        } else {
            context->ecx = pci_read_32(BH(context), BL(context) >> 3, address);
        }

        AH(context) = PCIBIOS_SUCCESSFUL;
        FLAGS(context) &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case PCIBIOS_FUNC_WRITE_BYTE: {
        uint address = DI(context);

        if (address > 255) {
            D_MESSAGE("wb: bad address 0x%x", address);
            AH(context) = PCIBIOS_BAD_REGISTER;
            break;
        }

        if (!(BL(context) & 0x7)) {
            pci_write_8(BH(context), BL(context) >> 3, address, CL(context));
        }

        AH(context) = PCIBIOS_SUCCESSFUL;
        FLAGS(context) &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case PCIBIOS_FUNC_WRITE_WORD: {
        uint address = DI(context);

        if (address > 255 || (address & 0x01)) {
            D_MESSAGE("ww: bad address 0x%x", address);
            AH(context) = PCIBIOS_BAD_REGISTER;
            break;
        }

        if (!(BL(context) & 0x7)) {
            pci_write_16(BH(context), BL(context) >> 3, address, CX(context));
        }

        AH(context) = PCIBIOS_SUCCESSFUL;
        FLAGS(context) &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case PCIBIOS_FUNC_WRITE_DWORD: {
        uint address = DI(context);

        if (address > 255 || (address & 0x03)) {
            D_MESSAGE("wd: bad address 0x%x", address);
            AH(context) = PCIBIOS_BAD_REGISTER;
            break;
        }

        if (!(BL(context) & 0x7)) {
            pci_write_32(BH(context), BL(context) >> 3, address, context->ecx);
        }

        AH(context) = PCIBIOS_SUCCESSFUL;
        FLAGS(context) &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case PCIBIOS_FUNC_GET_IRQ_OPTION:
        pcibios_get_irq_option(context);
        break;
    case PCIBIOS_FUNC_SET_PCI_IRQ:
        pcibios_set_irq(context);
        break;
    default:
        D_MESSAGE("not supported 0x%x", AL(context));
        AH(context) = PCIBIOS_FUNC_NOT_SUPPORTED;
        FLAGS(context) |= (1 << CPU_FLAGS_CF_BIT);
    }
}

