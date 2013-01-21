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
#include "platform.h"
#include "pci.h"
#include "bios.h"

void call_rom_init(uint16_t offset, uint16_t seg, uint8_t bus, uint8_t device);
void hard_interrup_0();
void hard_interrup_1();
void hard_interrup_2();
void hard_interrup_3();
void hard_interrup_4();
void hard_interrup_5();
void hard_interrup_6();
void hard_interrup_7();
void hard_interrup_8();
void hard_interrup_9();
void hard_interrup_10();
void hard_interrup_11();
void hard_interrup_12();
void hard_interrup_13();
void hard_interrup_14();
void hard_interrup_15();
void pit_interrupt_handler();
void rtc_interrupt_handler();
void int11_handler();
void int12_handler();
void int15_handler();
void int1a_handler();
void unhandled_int_handler();
void dummy_interrupt();


uint16_t set_ds(uint16_t data_seg)
{
    uint16_t prev;

    _asm {
        mov prev, ds
        mov ds, data_seg
    }

    return prev;
}


uint16_t get_cs()
{
    uint16_t code_seg;

     _asm {
        mov ax, cs
        mov code_seg, ax
    }

    return code_seg;
}


uint16_t get_ds()
{
    uint16_t data_seg;

     _asm {
        mov ax, ds
        mov data_seg, ax
    }

    return data_seg;
}


void restore_ds()
{
   _asm {
        mov ax, cs
        mov ds, ax
    }
}


uint16_t read_word(uint16_t seg, uint16_t offset)
{
    uint16_t val;

    seg = set_ds(seg);
    val = *(uint16_t*)offset;
    set_ds(seg);

    return val;
}


static uint32_t read_dword(uint16_t seg, uint16_t offset)
{
    uint32_t val;

    seg = set_ds(seg);
    val = *(uint32_t*)offset;
    set_ds(seg);

    return val;
}


static void write_word(uint16_t seg, uint16_t offset, uint16_t val)
{
    seg = set_ds(seg);
    *(uint16_t*)offset = val;
    set_ds(seg);
}


static void write_dword(uint16_t seg, uint16_t offset, uint32_t val)
{
    seg = set_ds(seg);
    *(uint32_t*)offset = val;
    set_ds(seg);
}


static uint8_t read_byte(uint16_t seg, uint16_t offset)
{
    uint8_t val;

    seg = set_ds(seg);
    val = *(uint8_t*)offset;
    set_ds(seg);

    return val;
}


static void write_byte(uint16_t seg, uint16_t offset, uint8_t val)
{
    seg = set_ds(seg);
    *(uint8_t*)offset = val;
    set_ds(seg);
}


uint8_t bda_read_byte(uint16_t offset)
{
    return read_byte(BIOS_DATA_AREA_ADDRESS >> 4, offset);
}


void bda_write_byte(uint16_t offset, uint8_t val)
{
    write_byte(BIOS_DATA_AREA_ADDRESS >> 4, offset, val);
}


uint16_t bda_read_word(uint16_t offset)
{
    return read_word(BIOS_DATA_AREA_ADDRESS >> 4, offset);
}


void bda_write_word(uint16_t offset, uint16_t val)
{
    write_word(BIOS_DATA_AREA_ADDRESS >> 4, offset, val);
}


uint32_t bda_read_dword(uint16_t offset)
{
    return read_dword(BIOS_DATA_AREA_ADDRESS >> 4, offset);
}


static void bda_write_dword(uint16_t offset, uint32_t val)
{
    write_dword(BIOS_DATA_AREA_ADDRESS >> 4, offset, val);
}


uint8_t ebda_read_byte(uint16_t offset)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    return read_byte(seg, offset);
}


void ebda_write_byte(uint16_t offset, uint8_t val)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    write_byte(seg, offset, val);
}


uint16_t ebda_read_word(uint16_t offset)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    return read_word(seg, offset);
}


void ebda_write_word(uint16_t offset, uint16_t val)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    write_word(seg, offset, val);
}


uint32_t ebda_read_dword(uint16_t offset)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    return read_dword(seg, offset);
}


void ebda_write_dword(uint16_t offset, uint32_t val)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    write_dword(seg, offset, val);
}


 bool_t is_hard_int_context()
{
    return !!(ebda_read_word(OFFSET_OF_PRIVATE(bios_flags)) & BIOS_FLAGS_HARD_INT);
}


void set_irq_context()
{
    uint16_t flags = ebda_read_word(OFFSET_OF_PRIVATE(bios_flags));

    if ((flags & BIOS_FLAGS_HARD_INT)) {
        //irq stack is disabled, for that reason it is not an error
        //bios_error(BIOS_ERROR_DOUBLE_HARD_INT);
    }

    ebda_write_word(OFFSET_OF_PRIVATE(bios_flags), flags | BIOS_FLAGS_HARD_INT);
}


void clear_irq_context()
{
    uint16_t flags = ebda_read_word(OFFSET_OF_PRIVATE(bios_flags));
    ebda_write_word(OFFSET_OF_PRIVATE(bios_flags), flags & ~BIOS_FLAGS_HARD_INT);
}


static inline void init_bios_data_area()
{
    post(POST_CODE_BDA);

    mem_reset(FAR_POINTER(void, 0, BIOS_DATA_AREA_ADDRESS), BIOS_DATA_AREA_SIZE);
    mem_reset(FAR_POINTER(void, BIOS_EBDA_ADDRESS >> 4, 0), BIOS_EBDA_DATA_KB * KB);

    bda_write_word(BDA_OFFSET_EBDA, BIOS_EBDA_ADDRESS >> 4);
    ebda_write_byte(EBDA_OFFSET_SIZE, BIOS_EBDA_SIZE_KB);
}


typedef _Packed struct IntVector {
    uint16_t offset;
    uint16_t seg;
} IntVector;


void get_int_vec(uint8_t index, uint16_t __far * seg, uint16_t __far * offset)
{
    IntVector* entry = NULL;
    uint16_t prev_seg;

    entry += index;
    prev_seg = set_ds(0);
    *seg = entry->seg;
    *offset = entry->offset;
    set_ds(prev_seg);
}


void set_int_vec(uint8_t index, uint16_t seg, uint16_t offset)
{
    IntVector* entry = NULL;
    uint16_t prev_seg;

    entry += index;
    prev_seg = set_ds(0);
    entry->seg = seg;
    entry->offset = offset;
    set_ds(prev_seg);
}


void on_hard_interrupt(uint16_t line)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    IntHandler* handler;
    EBDA *ebda;
    uint i;

    set_ds(seg);
    ebda = 0;
    handler = ebda->private.int_handlers[line];

    if (handler) {
        do {
            handler->cb(handler->opaque);
            ASSERT(get_ds() == seg);
            handler = handler->next;
        } while (handler);

        restore_ds();
        bda_write_byte(BDA_OFFSET_LAST_IRQ, ~0);
    } else {
        restore_ds();
        platform_printf("unhandled interrupt %u", line);
        bda_write_byte(BDA_OFFSET_LAST_IRQ, 1 << line);
    }

    if (line > 7) {
        outb(IO_PORT_PIC2, PIC_SPECIFIC_EOI_MASK | (line - 8));
        line = 2;
    }

    outb(IO_PORT_PIC1, PIC_SPECIFIC_EOI_MASK | line);
}


void register_interrupt_handler(uint line, int_cb_t cb, uint opaque)
{
    uint16_t seg;
    EBDA *ebda;
    uint i;

    ASSERT(line < PIC_NUM_LINES * PIC_NUM_CHIPS);
    ASSERT((1 << line) & NOX_PCI_IRQ_LINES_MASK);
    ASSERT(cb);
    NO_INTERRUPT();

    seg = bda_read_word(BDA_OFFSET_EBDA);

    set_ds(seg);
    ebda = 0;

    for (i= 0; i < INT_HANDLERS_POOL_SIZE; i++) {
        if (ebda->private.handlers_pool[i].cb) {
            continue;
        }

        ebda->private.handlers_pool[i].cb = cb;
        ebda->private.handlers_pool[i].opaque = opaque;
        ebda->private.handlers_pool[i].next = ebda->private.int_handlers[line];
        ebda->private.int_handlers[line] = &ebda->private.handlers_pool[i];
        restore_ds();

        if (line > 7) {
            outb(IO_PORT_PIC2 + 1, inb(IO_PORT_PIC2 + 1) & ~(1 << (line - 8)));
        } else {
            outb(IO_PORT_PIC1 + 1, inb(IO_PORT_PIC1 + 1) & ~(1 << line));
        }
        return;
    }

    restore_ds();
    D_MESSAGE("out of interrupt slots");
    bios_error(BISO_ERROR_REGISTER_INT_FAILED);
}


void unregister_interrupt_handler(uint line, int_cb_t cb, uint opaque)
{
    IntHandler** handler;
    uint16_t seg;
    EBDA *ebda;

    ASSERT(line < PIC_NUM_LINES * PIC_NUM_CHIPS);
    ASSERT(cb);
    NO_INTERRUPT();

    D_MESSAGE("");

    seg = bda_read_word(BDA_OFFSET_EBDA);

    set_ds(seg);
    ebda = 0;

    handler = &ebda->private.int_handlers[line];

    while (*handler) {
        IntHandler* h = *handler;

        if (h->cb == cb &&  h->opaque == opaque) {
            *handler = h->next;
            mem_reset(h, sizeof(*h));
            restore_ds();
            return;
        }

        handler = &h->next;
    }

    restore_ds();
    D_MESSAGE("not found");
    bios_error(BISO_ERROR_UNREGISTER_INT_FAILED);
}


static void init_int_vector()
{
    uint i;

    for (i = 0; i < 0x08; i++) {
        set_int_vec(i, get_cs(), FUNC_OFFSET(unhandled_int_handler));
    }

    set_int_vec(0x08, get_cs(), FUNC_OFFSET(hard_interrup_0));
    set_int_vec(0x09, get_cs(), FUNC_OFFSET(hard_interrup_1));
    set_int_vec(0x0a, get_cs(), FUNC_OFFSET(hard_interrup_2));
    set_int_vec(0x0b, get_cs(), FUNC_OFFSET(hard_interrup_3));
    set_int_vec(0x0c, get_cs(), FUNC_OFFSET(hard_interrup_4));
    set_int_vec(0x0d, get_cs(), FUNC_OFFSET(hard_interrup_5));
    set_int_vec(0x0e, get_cs(), FUNC_OFFSET(hard_interrup_6));
    set_int_vec(0x0f, get_cs(), FUNC_OFFSET(hard_interrup_7));

    for (i = 0x10; i < 0x70; i++) {
        set_int_vec(i, get_cs(), FUNC_OFFSET(unhandled_int_handler));
    }

    set_int_vec(0x70, get_cs(), FUNC_OFFSET(hard_interrup_8));
    set_int_vec(0x71, get_cs(), FUNC_OFFSET(hard_interrup_9));
    set_int_vec(0x72, get_cs(), FUNC_OFFSET(hard_interrup_10));
    set_int_vec(0x72, get_cs(), FUNC_OFFSET(hard_interrup_11));
    set_int_vec(0x74, get_cs(), FUNC_OFFSET(hard_interrup_12));
    set_int_vec(0x75, get_cs(), FUNC_OFFSET(hard_interrup_13));
    set_int_vec(0x76, get_cs(), FUNC_OFFSET(hard_interrup_14));
    set_int_vec(0x77, get_cs(), FUNC_OFFSET(hard_interrup_15));

    for (i = 0x78; i < 0x100; i++) {
        set_int_vec(i, get_cs(), FUNC_OFFSET(unhandled_int_handler));
    }
}


void on_pit_interrupt()
{
    uint32_t* counter;

    set_ds(BDA_SEG);

    *(uint8_t*)BDA_OFFSET_LAST_IRQ = ~0;

    counter = (uint32_t*)BDA_OFFSET_TICKS;

    if (++*counter == BISO_SYS_TIME_ROLLOVER) {
        *counter = 0;
        *(uint8_t*)BDA_OFFSET_TICKS_ROLLOVER = 1;
    }

    if ((*counter % 36) == 0) {
        restore_ds();
        platform_debug_string("tick");
    }

    INT(0x1c);

    outb(IO_PORT_PIC1, PIC_SPECIFIC_EOI_MASK | 0);
}


static void setup_pit_irq()
{
    set_int_vec(0x08, get_cs(), FUNC_OFFSET(pit_interrupt_handler));
    set_int_vec(0x1c, get_cs(), FUNC_OFFSET(dummy_interrupt));
    outb(IO_PORT_PIC1 + 1, (inb(IO_PORT_PIC1 + 1) & ~(1 << PIC1_TIMER_PIN)));
}


static void setup_rtc_irq()
{
    set_int_vec(0x70, get_cs(), FUNC_OFFSET(rtc_interrupt_handler));
    outb(IO_PORT_PIC2 + 1, (inb(IO_PORT_PIC2 + 1) & ~(1 << PIC2_RTC_PIN)));
    set_int_vec(0x15, get_cs(), FUNC_OFFSET(int15_handler));
    set_int_vec(0x1a, get_cs(), FUNC_OFFSET(int1a_handler));
}


void rtc_write(uint index, uint8_t val)
{
    outb(IO_PORT_RTC_INDEX, index | ebda_read_byte(OFFSET_OF_PRIVATE(nmi_mask)));
    outb(IO_PORT_RTC_DATA, val);
}


void rtc_write_buf(uint index, void __far * buf, uint size)
{
    uint8_t __far * byte_ptr = buf;

    ASSERT(index > RTC_REGD && index + size > index && index + size <= 128);

    while (size--) {
        rtc_write(index++, *byte_ptr++);
    }
}


uint8_t rtc_read(uint index)
{
    outb(IO_PORT_RTC_INDEX, index | ebda_read_byte(OFFSET_OF_PRIVATE(nmi_mask)));
    return inb(IO_PORT_RTC_DATA);
}


static void rtc_periodic_ref(uint8_t user)
{
    uint8_t refs;

    refs = ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_refs));

    if (!refs) {
        rtc_write(0x0a, (rtc_read(0x0a) & ~RTC_REG_A_RATE_MASK) | RTC_REG_A_RATE_976U);
        rtc_write(0x0b, rtc_read(0x0b) | RTC_REG_B_ENABLE_PERIODIC_MASK);
    }

    refs |= user;

    ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_refs), refs);
}


static void rtc_periodic_unref(uint8_t user)
{
    uint8_t refs;

    refs = ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_refs));

    refs &= ~user;

    ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_refs), refs);

    if (!refs) {
        rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_PERIODIC_MASK);
    }
}


static uint8_t start_wait(uint32_t micro, far_ptr_16_t usr_ptr)
{
    uint8_t regb;

    bda_write_byte(BDA_OFFSET_WAIT_FLAGS, bda_read_byte(BDA_OFFSET_WAIT_FLAGS) | BDA_WAIT_IN_USE);

    bda_write_dword(BDA_OFFSET_WAIT_COUNT_MICRO, micro);
    bda_write_dword(BDA_OFFSET_WAIT_USR_PTR, usr_ptr);

    rtc_periodic_ref(BIOS_PERIODIC_USER_WAIT_SERVICE);

    return regb;
}


_Packed struct { // 0xF000:E6f5 in IBM PC and 100%-compatible BIOSes
    uint16_t size;
    uint8_t model;
    uint8_t sub_model;
    uint8_t revision;
    uint8_t func_inf_1;
    uint8_t func_inf_2;
    uint8_t func_inf_3;
    uint8_t func_inf_4;
    uint8_t func_inf_5;
} config_table = {
    .size = sizeof(config_table),
    .model = 0xfc,
    .revision = 1,
    .func_inf_1 = (1 << 6) /*2nd interrupt controller*/ |
                  (1 << 5) /* real time clock installed*/ |
                  (1 << 4) /*calls int 0x15/0x4f on key reciced*/ |
                  (1 << 2) /*using ebda*/,
    .func_inf_2 = (1 << 6) /*int 0x16/0x09 supported*/,

};


typedef _Packed struct MemMapEnt {
    uint64_t address;
    uint64_t size;
    uint32_t type;
} MemMapEnt;


enum {
    MEM_TYPE_AVAIABLE = 1,
    MEM_TYPE_RESERVED,
    MEM_TYPE_ACPI_RECLAIMABEL,
    MEM_TYPE_ACPI_NVS,
};


enum {
    MEM_MAP_INDEX_BASE = 0,
    MEM_MAP_INDEX_EBDA,
    MEM_MAP_INDEX_BIOS,
    MEM_MAP_INDEX_ABOVE_1M,
    MEM_MAP_INDEX_IO_APIC,
    MEM_MAP_INDEX_LOCAL_APIC,
    MEM_MAP_INDEX_BELOW_4G,
    MEM_MAP_INDEX_ABOVE_4G,
};


static bool_t big_mem_get_map(UserRegs __far * context)
{
    MemMapEnt __far * ent;

    if (context->edx != 0x534D4150 /*SMAP*/) {
        return FALSE;
    }

    context->eax = context->edx;

    if (context->ecx < sizeof(MemMapEnt)) {
        D_MESSAGE("invalid size");
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
        return TRUE;
    }

    context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
    ent = FAR_POINTER(MemMapEnt, context->es, DI(context));

    switch (context->ebx) {
    case MEM_MAP_INDEX_BASE:
        ent->address = 0;
        ent->size = (uint64_t)(BASE_MEMORY_SIZE_KB - ebda_read_byte(BIOS_EBDA_SIZE_KB)) * KB;
        ent->type = MEM_TYPE_AVAIABLE;
        context->ebx = MEM_MAP_INDEX_EBDA;
        context->ecx = sizeof(MemMapEnt);
        break;
    case MEM_MAP_INDEX_EBDA:
        ent->address = (uint64_t)bda_read_word(BDA_OFFSET_EBDA) << 4;
        ent->size = (uint64_t)ebda_read_byte(BIOS_EBDA_SIZE_KB) * KB;
        if (ent->address != (BASE_MEMORY_SIZE_KB - BIOS_EBDA_SIZE_KB) * KB ||
                                                              ent->size != BIOS_EBDA_SIZE_KB * KB) {
            D_MESSAGE("MEM_MAP_INDEX_EBDA: EBDA changed");
        }
        ent->type = MEM_TYPE_RESERVED;
        context->ebx = MEM_MAP_INDEX_BIOS;
        context->ecx = sizeof(MemMapEnt);
        break;
    case MEM_MAP_INDEX_BIOS:
        ent->address = BIOS32_START_ADDRESS;
        ent->size = BIOS32_SIZE + BIOS16_SIZE;
        ent->type = MEM_TYPE_RESERVED;
        if (ebda_read_dword(OFFSET_OF_PRIVATE(above_1m_pages))) {
            context->ebx = MEM_MAP_INDEX_ABOVE_1M;
        } else {
            context->ebx = MEM_MAP_INDEX_IO_APIC;
        }
        context->ecx = sizeof(MemMapEnt);
        break;
    case MEM_MAP_INDEX_ABOVE_1M: {
        uint32_t above_1m_pages = ebda_read_dword(OFFSET_OF_PRIVATE(above_1m_pages));
        if (!above_1m_pages) {
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            return TRUE;
        }
        ent->address = 1ULL * MB;
        ent->size = above_1m_pages * KB * 4;
        ent->type = MEM_TYPE_AVAIABLE;
        context->ebx = MEM_MAP_INDEX_IO_APIC;
        context->ecx = sizeof(MemMapEnt);
        break;
    }
    case MEM_MAP_INDEX_IO_APIC:
        ent->address = IO_APIC_ADDRESS;
        ent->size = 4 * KB;
        ent->type = MEM_TYPE_RESERVED;
        context->ebx = MEM_MAP_INDEX_LOCAL_APIC;
        context->ecx = sizeof(MemMapEnt);
        break;
    case MEM_MAP_INDEX_LOCAL_APIC:
        ent->address = LOCAL_APIC_ADDRESS;
        ent->size = 4 * KB;
        ent->type = MEM_TYPE_RESERVED;
        context->ebx = MEM_MAP_INDEX_BELOW_4G;
        context->ecx = sizeof(MemMapEnt);
        break;
    case MEM_MAP_INDEX_BELOW_4G: {
        uint32_t below_4g_pages = ebda_read_dword(OFFSET_OF_PRIVATE(below_4g_pages));
        ent->address = 4ULL * GB - (below_4g_pages * KB * 4);
        ent->size = below_4g_pages * KB * 4;
        ent->type = MEM_TYPE_RESERVED;
        if (ebda_read_dword(OFFSET_OF_PRIVATE(above_4g_pages))) {
            context->ebx = MEM_MAP_INDEX_ABOVE_4G;
        } else {
            context->ebx = 0;
        }
        context->ecx = sizeof(MemMapEnt);
        break;
    }
    case MEM_MAP_INDEX_ABOVE_4G: {
        uint32_t above_4g_pages = ebda_read_dword(OFFSET_OF_PRIVATE(above_4g_pages));

        if (!above_4g_pages) {
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            return TRUE;
        }

        ent->address = 4ULL * GB;
        ent->size = (uint64_t)above_4g_pages * KB * 4;
        ent->type = MEM_TYPE_AVAIABLE;
        context->ebx = 0;
        context->ecx = sizeof(MemMapEnt);
        break;
    }
    default:
        D_MESSAGE("invalid index");
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
    }

    return TRUE;
}


static uint32_t descriptor_to_address(uint8_t __far * descriptor)
{
    uint32_t address;

    address = descriptor[2] | ((uint32_t)descriptor[3] << 8);
    address |= ((uint32_t)descriptor[4] << 16);
    address |= ((uint32_t)descriptor[7] << 24);

    return address;
}


void on_int15(UserRegs __far * context)
{
    switch (AH(context)) {
    case INT15_FUNC_KBD_INTERCEPT:
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
        break;
    case INT15_FUNC_EVENT_WAIT_CTRL:
        switch (AL(context)) {
        case INT15_EVENT_WAIT_SET: {
            uint32_t micro;

            if (bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & BDA_WAIT_IN_USE) {
                AX(context) = 0;
                context->flags |= (1 << CPU_FLAGS_CF_BIT);
                return;
            }

            micro = ((uint32_t)CX(context) << 16) | DX(context);

            if (!micro) {
                bda_write_dword(BDA_OFFSET_WAIT_COUNT_MICRO, 0);
                write_dword(context->es, BX(context),
                            read_dword(context->es, BX(context)) | BDA_WAIT_ELAPSED);
            } else {
                AL(context) = start_wait(((uint32_t)CX(context) << 16) | DX(context),
                                         ((uint32_t)context->es << 16) | BX(context));
            }

            context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
            break;
        }
        case INT15_EVENT_WAIT_CANCEL:
            rtc_periodic_unref(BIOS_PERIODIC_USER_WAIT_SERVICE);
            bda_write_byte(BDA_OFFSET_WAIT_FLAGS,
                           bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & ~BDA_WAIT_IN_USE);
            context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
            break;
        default:
            D_MESSAGE("not supported 0x%lx", context->eax);
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            AH(context) = 0x86;
        }
    case INT15_FUNC_WAIT: {
        uint32_t micro;

        if (bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & BDA_WAIT_IN_USE) {
            AX(context) = 0;
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            return;
        }

        if ((micro = ((uint32_t)CX(context) << 16) | DX(context))) {

            bda_write_byte(BDA_OFFSET_WAIT_FLAGS,
                           bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & ~BDA_WAIT_ELAPSED);

            start_wait(micro, ((uint32_t)BDA_SEG << 16) | BDA_OFFSET_WAIT_FLAGS);

            for (;;) {
                if (bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & BDA_WAIT_ELAPSED) {
                    break;
                }

                STI();
                HALT();
                CLI();
            }
        }

        bda_write_byte(BDA_OFFSET_WAIT_FLAGS,
                       bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & ~BDA_WAIT_ELAPSED);

        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        AX(context) = 0;
        break;
    }
    case INT15_FUNC_DEVICE_BUSY:
    case INT15_FUNC_DEVICE_POST:
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    case INT15_FUNC_GET_EXT_MEMORY_SIZE: {
        uint32_t above_1m = ebda_read_dword(OFFSET_OF_PRIVATE(above_1m_pages));
        if (above_1m > 63 * (MB >> PAGE_SHIFT)) {
            AX(context) = 63 * KB;
        } else {
            AX(context) = above_1m * 4;
        }
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case INT15_FUNC_GET_BIG_MEMORY_SIZE: {
        uint32_t above_1m = ebda_read_dword(OFFSET_OF_PRIVATE(above_1m_pages));
        above_1m <<= 2;
        AX(context) = above_1m;
        DX(context) = above_1m >> 16;
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case INT15_FUNC_BIG_MEM: {
        switch (AL(context)) {
        case INT15_BIG_MEM_GET_SIZE: {
            uint32_t above_1m = ebda_read_dword(OFFSET_OF_PRIVATE(above_1m_pages));

            if (above_1m > 15 * (MB >> PAGE_SHIFT)) {
                AX(context) = 15 * KB;
                above_1m -= 15 * (MB >> PAGE_SHIFT);
                above_1m /= (64 / 4);
                BX(context) = above_1m;
            } else {
                AX(context) = above_1m * 4;
                BX(context) = 0;
            }

            CX(context) = AX(context);
            DX(context) = BX(context);

            context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
            break;
        }
        case INT15_BIG_MEM_GET_MAP:
            if (!big_mem_get_map(context)) {
                goto not_supported;
            }
            break;
        default:
             D_MESSAGE("not supported 0x%lx", context->eax);
             context->flags |= (1 << CPU_FLAGS_CF_BIT);
             AH(context) = 0x86;
        }
        break;
    }
    case INT15_FUNC_GET_CONFIG:
        context->es = BIOS16_CODE_SEGMENT;
        BX(context) = (uint)&config_table;
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    case INT15_FUNC_TARGRT_OPERATING_MODE:
        if (AX(context) != 0xec00) {
            goto not_supported;
        }

        switch(BL(context)) {
        case 1: // lagacy
        case 2: // long mode
        case 3: // mixed mode
            context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
            break;
        default:
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
        }

        AH(context) = 0;
        break;
    case INT15_FUNC_GET_EBDA_SEG:
        context->es = bda_read_word(BDA_OFFSET_EBDA);
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    case INT15_FUNC_MOUSE:
        mouse_service(context);
        break;
    case INT15_FUNC_COPY_EXT_MEM: {
        uint32_t address;
        uint32_t pmod_stack;

        NO_INTERRUPT();

        ebda_write_word(OFFSET_OF_PRIVATE(ext_copy_words), CX(context));
        address = descriptor_to_address(FAR_POINTER(uint8_t, context->es, SI(context) + 0x10));
        ebda_write_dword(OFFSET_OF_PRIVATE(ext_copy_src), address);
        address = descriptor_to_address(FAR_POINTER(uint8_t, context->es, SI(context) + 0x18));
        ebda_write_dword(OFFSET_OF_PRIVATE(ext_copy_dest), address);

        // interrupt are disabled during 32bit call => hard int stack can be used
        pmod_stack = (uint32_t)ebda_read_word(OFFSET_OF_PRIVATE(real_hard_int_ss)) << 4;
        pmod_stack |= ebda_read_word(OFFSET_OF_PRIVATE(real_hard_int_sp));

        ebda_write_dword(OFFSET_OF_PRIVATE(pmode_stack_base), pmod_stack);
        ebda_write_byte(OFFSET_OF_PRIVATE(call_select), CALL_SELECT_COPY_MEM);
        call32();

        AH(context) = 0;
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case 0xe9:
        if (AX(context) != 0xe980) { //SpeedStep legacy applet interface
            goto not_supported;
        }
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
        AH(context) = 0x86;
        break;
    case INT15_FUNC_APM:
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
        AH(context) = 0x86;
        break;
    case 0: // turn cassette on
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
        AH(context) = 0x86;
        break;
    default:
        D_MESSAGE("not supported 0x%lx", context->eax);
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
        AH(context) = 0x86;
    }

    return;

not_supported:
    D_MESSAGE("not supported 0x%lx", context->eax);
    context->flags |= (1 << CPU_FLAGS_CF_BIT);
    AH(context) = 0x86;
}


void on_int11(UserRegs __far * context)
{
    AX(context) = bda_read_word(BDA_OFFSET_EQUIPMENT);
}


void on_int12(UserRegs __far * context)
{
    AX(context) = bda_read_word(BDA_OFFSET_MAIN_MEM_SIZE);
}


void on_int1a(UserRegs __far * context)
{
    switch (AH(context)) {
    case INT1A_FUNC_GET_SYS_TIME: {
        uint32_t ticks = bda_read_dword(BDA_OFFSET_TICKS);

        DX(context) = ticks;
        CX(context) = ticks >> 16;
        AL(context) = bda_read_byte(BDA_OFFSET_TICKS_ROLLOVER);
        bda_write_byte(BDA_OFFSET_TICKS_ROLLOVER, 0);
        break;
    }
    case INT1A_FUNC_SET_SYS_TIME: {
        uint32_t ticks = ((uint32_t)CX(context) << 16) | DX(context);

        ticks %= BISO_SYS_TIME_ROLLOVER;

        bda_write_dword(BDA_OFFSET_TICKS, ticks);
        bda_write_byte(BDA_OFFSET_TICKS_ROLLOVER, 0);
        break;
    }
    case INT1A_FUNC_GET_TIME:
        if ((rtc_read(0x0a) & RTC_REG_A_UPDATE_IN_PROGRESS)) {
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            break;
        }

        CH(context) = rtc_read(RTC_HOURS);
        CL(context) = rtc_read(RTC_MINUTES);
        DH(context) = rtc_read(RTC_SECONDS);
        DL(context) = rtc_read(RTC_REGB) & 1;
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    case INT1A_FUNC_SET_TIME: {
        uint8_t regb = rtc_read(RTC_REGB);

        regb &= ~(RTC_REG_B_DAYLIGHT_MASK | RTC_REG_B_BINARY_MASK);
        regb |= (DL(context) ? RTC_REG_B_DAYLIGHT_MASK : 0) | RTC_REG_B_HALT_CLOCK_MASK |
                                                              RTC_REG_B_24_HOUR_MASK;
        rtc_write(RTC_REGB, regb);
        rtc_write(RTC_HOURS, CH(context));
        rtc_write(RTC_MINUTES, CL(context));
        rtc_write(RTC_SECONDS, DH(context));
        rtc_write(RTC_REGB, regb & ~RTC_REG_B_HALT_CLOCK_MASK);
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case INT1A_FUNC_GET_DATE:
        if ((rtc_read(0x0a) & RTC_REG_A_UPDATE_IN_PROGRESS)) {
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            break;
        }

        CH(context) = rtc_read(CMOS_OFFSET_ISA_CENTURY);
        CL(context) = rtc_read(RTC_YEAR);
        DH(context) = rtc_read(RTC_MOUNTH);
        DL(context) = rtc_read(RTC_DAY_OF_MOUNTH);
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    case INT1A_FUNC_SET_DATE: {
        uint8_t regb = rtc_read(RTC_REGB);

        regb &= ~RTC_REG_B_BINARY_MASK;
        regb |= RTC_REG_B_HALT_CLOCK_MASK | RTC_REG_B_24_HOUR_MASK;

        rtc_write(RTC_REGB, regb);

        rtc_write(CMOS_OFFSET_ISA_CENTURY, CH(context));
        rtc_write(RTC_YEAR, CL(context));
        rtc_write(RTC_MOUNTH, DH(context));
        rtc_write(RTC_DAY_OF_MOUNTH, DL(context));

        rtc_write(RTC_REGB, regb & ~RTC_REG_B_HALT_CLOCK_MASK);
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case INT1A_FUNC_SET_ALARM: {
        uint16_t flags = ebda_read_word(OFFSET_OF_PRIVATE(bios_flags));
        uint8_t reg_c;

        if ((flags & BIOS_FLAGS_ALRM_ACTIVE_MASK)) {
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            break;
        }

        rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_ALARM_MASK);
        rtc_write(RTC_HOURS_ALARM, CH(context));
        rtc_write(RTC_MINUTES_ALARM, CL(context));
        rtc_write(RTC_SECONDS_ALARM, DH(context));
        rtc_write(0x0b, rtc_read(0x0b) | RTC_REG_B_ENABLE_ALARM_MASK);

        // AF barrier
        reg_c = bda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c));
        reg_c = (reg_c | rtc_read(0x0c)) & ~RTC_REG_C_ALARM_INTERRUPT_MASK;
        ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c), reg_c);

        ebda_write_word(OFFSET_OF_PRIVATE(bios_flags), flags | BIOS_FLAGS_ALRM_ACTIVE_MASK);
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case INT1A_FUNC_CANCEL_ALARM: {
        uint16_t flags = ebda_read_word(OFFSET_OF_PRIVATE(bios_flags));
        ebda_write_word(OFFSET_OF_PRIVATE(bios_flags), flags & ~BIOS_FLAGS_ALRM_ACTIVE_MASK);
        rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_ALARM_MASK);
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case INT1A_FUNC_PCIBIOS:
        pcibios_service(context);
        break;
    default:
        D_MESSAGE("not supported 0x%lx", context->eax);
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
        AH(context) = 0x86;
    }
}


void on_unhandled_int(UserRegs __far * context)
{
    uint8_t opcode = read_byte(context->cs, context->ip - 2);

    if (opcode == 0xcd) {
        D_MESSAGE("int 0x%x, eax 0x%lx", read_byte(context->cs, context->ip - 1), context->eax);
        return;
    }

    opcode = read_byte(context->cs, context->ip - 1);

    switch (opcode) {
    case 0xcc: //debug interrupt
        D_MESSAGE("int 0x%x, eax 0x%lx", 3, context->eax);
        break;
    case 0xce: // into
        D_MESSAGE("int 0x%x, eax 0x%lx", 4, context->eax);
        break;
    default:
        D_MESSAGE("unknown");
    }
}


void on_rtc_interrupt()
{
    uint16_t flags;
    uint8_t regc;

    regc = ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c));
    ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c), 0);
    regc |= rtc_read(0x0c);

    if ((regc & RTC_REG_C_PERIODIC_INTERRUPT_MASK)) {
        uint32_t counter;

        if ((bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & BDA_WAIT_IN_USE)) {
            counter = bda_read_dword(BDA_OFFSET_WAIT_COUNT_MICRO);

            if (counter < BIOS_PERIODIC_MICRO) { // ~1 mili in mode 6
                far_ptr_16_t ptr;
                uint32_t val;

                bda_write_dword(BDA_OFFSET_WAIT_COUNT_MICRO, 0);
                rtc_periodic_unref(BIOS_PERIODIC_USER_WAIT_SERVICE);
                ptr = bda_read_dword(BDA_OFFSET_WAIT_USR_PTR);
                val = read_dword(ptr >> 16, ptr & 0xffff);
                write_dword(ptr >> 16, ptr & 0xffff, (val | BDA_WAIT_ELAPSED));
                bda_write_byte(BDA_OFFSET_WAIT_FLAGS,
                               bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & ~BDA_WAIT_IN_USE);
            } else {
                bda_write_dword(BDA_OFFSET_WAIT_COUNT_MICRO, counter - BIOS_PERIODIC_MICRO);
            }
        }

        counter = ebda_read_dword(OFFSET_OF(EBDA, private) +
                                  OFFSET_OF(EBDAPrivate, rtc_priodoc_ticks));

        ebda_write_dword(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_ticks),
                         counter + 1);

        if (!ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_refs))) {
            // unexpected
            rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_PERIODIC_MASK);
        }
    }

    flags = ebda_read_word(OFFSET_OF_PRIVATE(bios_flags));

    if ((flags & BIOS_FLAGS_ALRM_ACTIVE_MASK)) {
        if ((regc & RTC_REG_C_ALARM_INTERRUPT_MASK)) {
            INT(0x4a);
        } else {
            // unexpected
            rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_ALARM_MASK);
        }
    }

    if ((regc & RTC_REG_C_UPDATE_INTERRUPT_MASK)) {
        // unexpected.
        rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_UPDATE_MASK);
    }

    outb(IO_PORT_PIC1, PIC_SPECIFIC_EOI_MASK | 2);
    outb(IO_PORT_PIC2, PIC_SPECIFIC_EOI_MASK | 0);
}


void delay(uint32_t milisec)
{
    uint64_t micro;
    uint32_t flags;

    if (!milisec) {
        return;
    }

    flags = get_eflags();
    micro = (uint64_t)milisec * 1000 + BIOS_PERIODIC_MICRO;

    ebda_write_dword(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_ticks), 0);
    rtc_periodic_ref(BIOS_PERIODIC_USER_INTERNAL_DELAY);

    for (;;) {
        uint64_t t;

        STI();
        HALT();
        CLI();

        t = ebda_read_dword(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_ticks));
        t *= BIOS_PERIODIC_MICRO;

        if (t >= micro) {
            break;
        }

    }

    rtc_periodic_unref(BIOS_PERIODIC_USER_INTERNAL_DELAY);
    put_eflags(flags);
}


static void hard_delay(uint32_t milisec)
{
    milisec++;

    while (milisec) {
        uint8_t reg_c = rtc_read(0x0c);

        if ((reg_c & RTC_REG_C_PERIODIC_INTERRUPT_MASK)) {
            milisec--;
        }

        reg_c |= ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c));
        ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c), reg_c);
    }
}


void beep()
{
    uint16_t countdown = PIT_FREQUENCY / BIOS_BEEP_Hz;
    uint8_t misc;

    NO_INTERRUPT();

    // set timer 2: mode 3, 16bit, binary
    outb(IO_PORT_TIMER_CONTROL, (2 << PIT_SELECTOR_SHIFT) |
                                (3 << PIT_RW_SHIFT) |
                                (3 << PIT_MODE_SHIFT));

    outb(IO_PORT_TIMER_2, countdown);
    outb(IO_PORT_TIMER_2, countdown >> 8);

    misc = inb(IO_PORT_MISC);
    outb(IO_PORT_MISC, misc | 0x3);
    hard_delay(BIOS_BEEP_DURATION_MS);
    outb(IO_PORT_MISC, (misc & ~0x3));
}


static void int_exp_rom()
{
    uint32_t start;
    uint8_t bus;
    uint8_t device;
    uint16_t rom_seg;
    uint8_t size_before;
    uint8_t size_after;

    ebda_write_word(OFFSET_OF_PRIVATE(bios_flags),
                    ebda_read_word(OFFSET_OF_PRIVATE(bios_flags)) | BIOS_FLAGS_UNREAL);
    ebda_write_byte(OFFSET_OF_PRIVATE(call_select), CALL_SELECT_NOP);
    call32();

    start = ebda_read_dword(OFFSET_OF_PRIVATE(loaded_rom_address));
    bus = ebda_read_byte(OFFSET_OF_PRIVATE(loaded_rom_bus));
    device = ebda_read_byte(OFFSET_OF_PRIVATE(loaded_rom_device));
    device <<= 3;
    rom_seg = start >> 4;

    size_before = read_byte(rom_seg, OFFSET_OF(CodeImageHeader, code_image_size));

    call_rom_init(OFFSET_OF(CodeImageHeader, jump), rom_seg, bus, device);

    size_after = read_byte(rom_seg, OFFSET_OF(CodeImageHeader, code_image_size));

    if (size_after > size_before) {
        bios_error(BIOS_ERROR_EXP_ROM_NEW_SIZE);
    } else if (size_after < size_before) {
        uint32_t load_address;

        load_address = ebda_read_dword(OFFSET_OF_PRIVATE(rom_load_address));
        load_address -= (size_before - size_after) * PCI_ROM_GRANULARITY;
        load_address = ALIGN(load_address, 2 * KB);
        ebda_write_dword(OFFSET_OF_PRIVATE(rom_load_address), load_address);
    }

    ebda_write_word(OFFSET_OF_PRIVATE(bios_flags),
                    ebda_read_word(OFFSET_OF_PRIVATE(bios_flags)) & ~BIOS_FLAGS_UNREAL);
    ebda_write_byte(OFFSET_OF_PRIVATE(call_select), CALL_SELECT_NOP);
    call32();
}


static void play_note(uint32_t frequency, uint duration)
{
    uint32_t countdown = 1193182UL / frequency;
    uint8_t misc;

    if (!countdown || countdown > 0xffff) {
        return;
    }

    // set timer 2: mode 3, 16bit, binary
    outb(IO_PORT_TIMER_CONTROL, (2 << PIT_SELECTOR_SHIFT) |
                                (3 << PIT_RW_SHIFT) |
                                (3 << PIT_MODE_SHIFT));

    outb(IO_PORT_TIMER_2, countdown);
    outb(IO_PORT_TIMER_2, countdown >> 8);

    misc = inb(IO_PORT_MISC);
    outb(IO_PORT_MISC, misc | 0x3);
    delay(duration);
    outb(IO_PORT_MISC, (misc & ~0x3));
}


static void term_put_char_cb(void __far * opaque, char val)
{
    __asm {
        push bx
        mov ah, 0x0e
        mov al, val
        mov bx, 0
        int  0x10
        pop bx
    }
}


void term_printf(const char __far * format, ...)
{
    uint8_t __far * args = (uint8_t __far *)&format;
    format_str(term_put_char_cb, NULL, format, SKIP_STACK_ARG(const char __far *, args));
}


void init()
{
    uint i;

    post(POST_CODE_INIT16);
    init_bios_data_area();

    ebda_write_dword(OFFSET_OF_PRIVATE(pmode_stack_base), BIOS32_STAGE1_STACK_BASE);
    ebda_write_byte(OFFSET_OF_PRIVATE(call_select), CALL_SELECT_INIT);
    call32();

    init_int_vector();

    platform_debug_string("log from 16bit");

    setup_pit_irq();
    setup_rtc_irq();

    keyboard_init();
    boot_init();

    set_int_vec(0x11, get_cs(), FUNC_OFFSET(int11_handler));
    set_int_vec(0x12, get_cs(), FUNC_OFFSET(int12_handler));

    ebda_write_byte(OFFSET_OF_PRIVATE(call_select), CALL_SELECT_LOAD_VGA);
    call32();

    if (ebda_read_byte(OFFSET_OF_PRIVATE(call_ret_val))) {
        int_exp_rom();
    }

    ata_init();

    for (;;) {
        ebda_write_byte(OFFSET_OF_PRIVATE(call_select), CALL_SELECT_LOAD_EXP_ROM);
        call32();

        if (!ebda_read_byte(OFFSET_OF_PRIVATE(call_ret_val))) {
            break;
        }

        int_exp_rom();
        // todo: call PnP BEV or BCV as required and update boot options tables
    }

    STI();

    post(POST_CODE_TMP);

    boot();

    bios_error(BIOS_ERROR_UNEXPECTED_IP);


    //Octave 6
    play_note(1046, 250);
    delay(1000);
    play_note(1175, 250);
    delay(1000);
    play_note(1328, 250);
    delay(1000);
    play_note(1397, 250);
    delay(1000);
    play_note(1568, 250);
    delay(1000);
    play_note(1760, 250);
    delay(1000);
    play_note(1975, 250);
    delay(1000);

    for (;;) {
        uint8_t scan;
        uint8_t ascii;
        char str[100];
        char ascii_str[2];

        ascii_str[1] = 0;

        __asm {
            mov ah, INT16_FUNC_READ_KEY
            int 16h
            mov scan, ah
            mov ascii, al
        }

        ascii_str[0] = ascii;

        format_mem_str(str, sizeof(str), "scan 0x%x ascii 0x%x %S", scan, ascii, ascii_str);
        platform_debug_string(str);
    }

    for ( i = 0; i < 10; i++) {
        delay(2000);
        platform_debug_string("post delay");
    }

    for (;;) {
         __asm {
            mov ah, INT15_FUNC_WAIT
            mov cx, 0x98
            mov dx, 0x9680
            int 15h
        }
        platform_debug_string("post wait");
    }

    restart();
}

