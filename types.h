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

#ifndef _H_TYPES
#define _H_TYPES

#include "base_types.h"
#include "defs.h"
#include "pcibios.h"


#define INT_HANDLERS_POOL_SIZE 16
#define MAX_ATA_DEVICES 8
#define MAX_BOOT_OPTIONS 8


typedef _Packed struct discriptor_t {
    uint32_t low;
    uint32_t high;
} discriptor_t;


typedef _Packed struct discriptor_reg_t {
    uint16_t limit;
    uint32_t address;
} discriptor_reg_t;


typedef void (*int_cb_t)(uint opaque);

typedef _Packed struct IntHandler {
    int_cb_t cb;
    uint opaque;
    struct IntHandler* next;
} IntHandler;

#define ATA_DESCRIPTION_MAX 32

typedef _Packed struct ATADevice {
    uint8_t flags;
    uint8_t id;
    uint16_t cmd_port;
    uint16_t ctrl_port;
    char description[ATA_DESCRIPTION_MAX + 1];
    uint64_t sectors;
    uint16_t logi_cylinders;        // 1-1048
    uint8_t logi_heads;             // 1-255
    uint16_t physi_heads;
    uint16_t physi_sec_per_track;
    uint16_t physi_cylinders;
    uint8_t pci_bus;
    uint8_t pci_device;
    uint8_t pci_func;
    union {
        _Packed struct {
            uint8_t dumy;
        } hd;

        _Packed struct {
            uint8_t packet_size;
        } cd;
    } u;
} ATADevice;


typedef _Packed struct BootOption {
   uint8_t type;
   uint8_t flags;
   void __far * boot_handler;
   char __far * description;
   uint8_t user_data;
} BootOption;


typedef _Packed struct EBDAPrivate {
    uint32_t pmode_stack_base;
    uint16_t real_mode_ss;
    uint16_t real_mode_sp;
    uint16_t real_user_ss;
    uint16_t real_user_sp;
    uint16_t real_hard_int_ss;
    uint16_t real_hard_int_sp;
    uint8_t bios_flags;
    uint8_t next_cd_id;

    uint32_t below_1m_used_pages;
    uint32_t above_1m_pages;
    uint32_t below_4g_pages;
    uint32_t below_4g_used_pages;
    uint32_t above_4g_pages;
    uint32_t pci32_hole_start;
    uint32_t pci32_hole_end;
    uint64_t pci64_hole_start;

    uint16_t platform_io;
    address_t platform_ram;
    uint8_t address_lines;
    uint8_t nmi_mask;
    uint8_t rtc_reg_c;
    uint8_t rtc_priodoc_refs;
    uint32_t rtc_priodoc_ticks;

    uint8_t call_select;
    uint8_t call_ret_val;

    uint32_t ext_copy_words;
    uint32_t ext_copy_src;
    uint32_t ext_copy_dest;

    uint32_t alloc_start;
    uint32_t alloc_pos;
    uint32_t alloc_end;
    uint32_t stage1_flags;
    address_t activation_list;
    address_t io_bars;
    address_t mem_bars;
    address_t mem32_bars;
    address_t mem64_bars;
    uint32_t rom_load_address;
    uint8_t rom_load_index;
    uint32_t loaded_rom_address;
    uint8_t loaded_rom_bus;
    uint8_t loaded_rom_device;
    IntHandler* int_handlers[PIC_NUM_LINES * PIC_NUM_CHIPS];
    IntHandler handlers_pool[INT_HANDLERS_POOL_SIZE];
    ATADevice ata_devices[MAX_ATA_DEVICES];
    uint8_t boot_order[MAX_BOOT_OPTIONS];
    BootOption boot_options[MAX_BOOT_OPTIONS];
} EBDAPrivate;


typedef struct PrivateData {
    uint8_t irq_routing_table_size;
    IRQOption irq_routing_table[32];
} PrivateData;


typedef _Packed struct EBDA {
    uint8_t public[EBDA_PRIVATE_START];
    EBDAPrivate private;
} EBDA;

#ifdef _M_I86
typedef _Packed struct UserRegs {
    uint16_t gs;
    uint16_t fs;
    uint16_t es;
    uint16_t ds;

    uint32_t edi;
    uint32_t esi;
    uint32_t ebp;
    uint32_t esp;
    uint32_t ebx;
    uint32_t edx;
    uint32_t ecx;
    uint32_t eax;

    uint16_t ip;
    uint16_t cs;
    uint16_t flags;
} UserRegs;
#else
typedef _Packed struct UserRegs {
    uint16_t gs;
    uint16_t _0;
    uint16_t fs;
    uint16_t _1;
    uint16_t es;
    uint16_t _2;
    uint16_t ds;
    uint16_t _3;

    uint32_t edi;
    uint32_t esi;
    uint32_t ebp;
    uint32_t esp;
    uint32_t ebx;
    uint32_t edx;
    uint32_t ecx;
    uint32_t eax;

    uint32_t eflags;
} UserRegs;
#endif


#endif

