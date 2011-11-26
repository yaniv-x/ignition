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

#ifndef _H_PCI
#define _H_PCI

#define PCI_ADDRESS_INDEX_SHIFT 2
#define PCI_ADDRESS_INDEX_BITS 6
#define PCI_ADDRESS_FUNCTION_SHIFT 8
#define PCI_ADDRESS_FUNCTION_BITS 3
#define PCI_ADDRESS_DEVICE_SHIFT 11
#define PCI_ADDRESS_DEVICE_BITS 5
#define PCI_ADDRESS_BUS_SHIFT 16
#define PCI_ADDRESS_BUS_BITS 8
#define PCI_ADDRESS_ENABLED_MASK (1UL << 31)
#define PCI_ADDRESS_MASK (~((1UL << 2) - 1))

#define PCI_BUS_MASK ((1UL << PCI_ADDRESS_BUS_BITS) - 1)
#define PCI_BUS_MAX PCI_BUS_MASK
#define PCI_DEVICE_MASK ((1UL << PCI_ADDRESS_DEVICE_BITS) - 1)
#define PCI_DEVICE_MAX PCI_DEVICE_MASK
#define PCI_FUNCTION_MASK ((1UL << PCI_ADDRESS_FUNCTION_BITS) - 1)
#define PCI_FUNCTION_MAX PCI_FUNCTION_MASK
#define PCI_INDEX_MASK ((1UL << PCI_ADDRESS_INDEX_BITS) - 1)
#define PCI_INDEX_MAX PCI_INDEX_MASK

#define PCI_VENDOE_INVALID 0xffff

#define PCI_OFFSET_VENDOR 0x00
#define PCI_OFFSET_DEVICE 0x02
#define PCI_OFFSET_COMMAND 0x04
#define PCI_OFFSET_STATUS 0x06
#define PCI_OFFSET_REVISION 0x08
#define PCI_OFFSET_CLASS 0x09
#define PCI_OFFSET_CACHE_LINE 0x0c
#define PCI_OFFSET_CACHE_LATENCY 0x0d
#define PCI_OFFSET_BAR_0 0x10
#define PCI_OFFSET_BAR_1 0x14
#define PCI_OFFSET_BAR_2 0x18
#define PCI_OFFSET_BAR_3 0x1c
#define PCI_OFFSET_BAR_4 0x20
#define PCI_OFFSET_BAR_5 0x24
#define PCI_OFFSET_SUBSYS_VENDOR 0x2c
#define PCI_OFFSET_SUBSYS_ID 0x2e
#define PCI_OFFSET_ROM_ADDRESS 0x30
#define PCI_OFFSET_INTERRUPT_LINE 0x3c
#define PCI_OFFSET_INTERRUPT_PIN 0x3d

#define PCI_COMMAND_ENABLE_IO (1 << 0)
#define PCI_COMMAND_ENABLE_MEM (1 << 1)
#define PCI_COMMAND_BUS_MASTER (1 << 2)
#define PCI_COMMAND_DISABLE_INTERRUPT (1 << 10)

#define PCI_BAR_IO_MASK (1UL << 0)
#define PCI_BAR_IO_ADDRESS_MASK ~0x3UL
#define PCI_BAR_MEM_ADDRESS_MASK ~0xfUL

#define PCI_ROM_ENABLE_MASK 1UL
#define PCI_ROM_FIRST_ADDRESS_BIT 11
#define PCI_ROM_MIN_SIZE (1UL << PCI_ROM_FIRST_ADDRESS_BIT)
#define PCI_ROM_ADDRESS_MASK (~(PCI_ROM_MIN_SIZE - 1))
#define PCI_ROM_MAX_SIZE (16UL * MB)

#define PCI_BASE_IO_ADDRESS 0xc000
#define PCI_NUM_BARS 6

#define PCI_MEMTYPE_32 0
#define PCI_MEMTYPE_64 2

#define PCI_IO_MIN_SIZE 4
#define PCI_IO_MAX_SIZE 256

#define PCI_MEM_MIN_SIZE 16

#define PCI_ROM_GRANULARITY 512
#define PCI_ROM_SIGNATURE FOUR_CHARS('PCIR')
#define EXP_ROM_SIGNATURE 0xaa55


typedef struct PCIDeviceType {
    uint8_t class;
    uint8_t sub_class;
    uint8_t prog_if;
} PCIDeviceType;


typedef _Packed struct CodeImageHeader {
    uint16_t signature;         // 0xaa55
    uint8_t code_image_size;    // in 512 byte units
    uint8_t jump[3];            // entry point
    uint8_t data[18];           // unique data (copyright etc.)
    uint16_t pci_data_offset;   // offset to PCIExpRomData
} CodeImageHeader;


typedef _Packed struct PCIExpRomData {
    uint32_t signature;     //  'PCIR'
    uint16_t vendor_id;
    uint16_t device_id;
    uint16_t reserved_0;
    uint16_t struct_size;
    uint8_t revision;       //  0
    uint8_t prog_if;
    uint8_t sub_class;
    uint8_t class;
    uint16_t code_image_size;
    uint16_t code_revision;
    uint8_t code_type;      //  0 => x86
    uint8_t flags;          //  bit 7: set => last code image, bits 6-0: reserved
    uint16_t reserved_1;
} PCIExpRomData;


uint32_t pci_config_address(uint32_t bus, uint32_t device, uint32_t index);

uint32_t pci_read_32(uint bus, uint device, uint offset);
uint16_t pci_read_16(uint bus, uint device, uint offset);
uint8_t pci_read_8(uint bus, uint device, uint offset);

void pci_write_32(uint bus, uint device, uint offset, uint32_t val);
void pci_write_16(uint bus, uint device, uint offset, uint16_t val);
void pci_write_8(uint bus, uint device, uint offset, uint8_t val);

typedef int (*pci_for_each_cb)(uint bus, uint device, void __far * opaque);
void pci_for_each(pci_for_each_cb cb, void __far * opaque);

bool_t pci_is_io_enabled(uint bus, uint device);
bool_t pci_is_mem_enabled(uint bus, uint device);
bool_t pci_find_class(uint index, PCIDeviceType __far * type, uint __far * bus,
                      uint __far * device);

void pcibios_service(UserRegs __far * context);

#endif

