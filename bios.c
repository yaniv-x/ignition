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


#include "common.c"

#define OFFSET_OF(type, member) ((uint16_t)&((type*)0)->member)
#define CLI() __asm { cli}
#define STI() __asm { sti}

void call32(void);
void unhandled_interrupt(void);
void unhandled_irq0(void);
void unhandled_irq1(void);
void unhandled_irq2(void);
void unhandled_irq3(void);
void unhandled_irq4(void);
void unhandled_irq5(void);
void unhandled_irq6(void);
void unhandled_irq7(void);
void unhandled_irq8(void);
void unhandled_irq9(void);
void unhandled_irq10(void);
void unhandled_irq11(void);
void unhandled_irq12(void);
void unhandled_irq13(void);
void unhandled_irq14(void);
void unhandled_irq15(void);

#define FUNC_OFFSET(function) (uint16_t)(function)


static uint16_t set_ds(uint16_t data_seg)
{
    uint16_t prev;

    _asm {
        mov prev, ds
        mov ds, data_seg
    }

    return prev;
}


static uint16_t get_cs()
{
    uint16_t code_seg;

     _asm {
        mov ax, cs
        mov code_seg, ax
    }

    return code_seg;
}


static uint16_t get_ds()
{
    uint16_t data_seg;

     _asm {
        mov ax, ds
        mov data_seg, ax
    }

    return data_seg;
}

static void restore_ds()
{
   _asm {
        mov ax, cs
        mov ds, ax
    }
}


static uint16_t read_word(uint16_t seg, uint16_t offset)
{
    uint16_t val;

    seg = set_ds(seg);
    val = *(uint16_t*)offset;
    set_ds(seg);

    return val;
}


static void write_word(uint16_t seg, uint16_t offset, uint16_t val)
{
    seg = set_ds(seg);
    *(uint16_t*)offset = val;
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


static void mem_set(uint16_t seg, uint16_t offset, uint8_t patern, uint16_t n)
{
    uint8_t* now = (uint8_t*)offset;
    uint8_t* end = now + n;

    seg = set_ds(seg);

    for (; now < end; now++) *now = patern;

    set_ds(seg);
}


static void mem_reset(uint16_t seg, uint16_t offset, uint16_t n)
{
    mem_set(seg, offset, 0, n);
}


static void bda_write_byte(uint16_t offset, uint8_t val)
{
    write_byte(BIOS_DATA_AREA_ADDRESS >> 4, offset, val);
}


static uint16_t bda_read_word(uint16_t offset)
{
    return read_word(BIOS_DATA_AREA_ADDRESS >> 4, offset);
}


static void bda_write_word(uint16_t offset, uint16_t val)
{
    write_word(BIOS_DATA_AREA_ADDRESS >> 4, offset, val);
}


static void ebda_write_byte(uint16_t offset, uint16_t val)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    write_byte(seg, offset, val);
}


static uint16_t ebda_read_word(uint16_t offset)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    return read_word(seg, offset);
}


static void platform_debug_print(char* str)
{
    uint16_t port = ebda_read_word(OFFSET_OF(EBDA, private) +
                                   OFFSET_OF(EBDAPrivate, platform_io));

    uint32_t flags = get_eflags();
    CLI();

    outb(port + PLATFORM_IO_SELECT, PLATFORM_REG_WRITE_POS);
    outd(port + PLATFORM_IO_REGISTER, 0);

    do {
        outb(port + PLATFORM_IO_PUT_BYTE, *str);
    } while (*str++);

    outb(port + PLATFORM_IO_LOG, 0);

    put_eflags(flags);
}


static inline void init_bios_data_area()
{
    post(POST_CODE_BDA);

    mem_reset(0, BIOS_DATA_AREA_ADDRESS, BIOS_DATA_AREA_SIZE);
    mem_reset(BIOS_EXTENDED_DATA_AREA_ADDRESS >> 4, 0, BIOS_EXTENDED_DATA_AREA_KB * KB);

    bda_write_word(BDA_OFFSET_EBDA, BIOS_EXTENDED_DATA_AREA_ADDRESS >> 4);
    ebda_write_byte(EBDA_OFFSET_SIZE, BIOS_EXTENDED_DATA_AREA_KB);
}


typedef _Packed struct IntVector {
    uint16_t offset;
    uint16_t seg;
} IntVector;


static void set_int_vec(uint8_t index, uint16_t seg, uint16_t offset)
{
    IntVector* entry = NULL;
    uint16_t prev_seg;

    entry += index;
    prev_seg = set_ds(0);
    entry->seg = seg;
    entry->offset = offset;
    set_ds(prev_seg);
}


void on_unhandled_irq(uint16_t irq)
{
    if (irq > 7) {
        outb(IO_PORT_PIC2, PIC_SPECIFIC_EOI_MASK | (irq % 8));
        irq = 2;
    }

    outb(IO_PORT_PIC1, PIC_SPECIFIC_EOI_MASK | irq);
    bda_write_byte(BDA_OFFSET_LAST_IRQ, 1 << irq);

    platform_debug_print(__FUNCTION__);
}


static void init_int_vector()
{
    uint i;

    for (i = 0; i < 0x08; i++) {
        set_int_vec(i, get_cs(), FUNC_OFFSET(unhandled_interrupt));
    }

    set_int_vec(0x08, get_cs(), FUNC_OFFSET(unhandled_irq0));
    set_int_vec(0x09, get_cs(), FUNC_OFFSET(unhandled_irq1));
    set_int_vec(0x0a, get_cs(), FUNC_OFFSET(unhandled_irq2));
    set_int_vec(0x0b, get_cs(), FUNC_OFFSET(unhandled_irq3));
    set_int_vec(0x0c, get_cs(), FUNC_OFFSET(unhandled_irq4));
    set_int_vec(0x0d, get_cs(), FUNC_OFFSET(unhandled_irq5));
    set_int_vec(0x0e, get_cs(), FUNC_OFFSET(unhandled_irq6));
    set_int_vec(0x0f, get_cs(), FUNC_OFFSET(unhandled_irq7));

    for (i = 0x10; i < 0x70; i++) {
        set_int_vec(i, get_cs(), FUNC_OFFSET(unhandled_interrupt));
    }

    set_int_vec(0x70, get_cs(), FUNC_OFFSET(unhandled_irq8));
    set_int_vec(0x71, get_cs(), FUNC_OFFSET(unhandled_irq9));
    set_int_vec(0x72, get_cs(), FUNC_OFFSET(unhandled_irq10));
    set_int_vec(0x72, get_cs(), FUNC_OFFSET(unhandled_irq11));
    set_int_vec(0x74, get_cs(), FUNC_OFFSET(unhandled_irq12));
    set_int_vec(0x75, get_cs(), FUNC_OFFSET(unhandled_irq13));
    set_int_vec(0x76, get_cs(), FUNC_OFFSET(unhandled_irq14));
    set_int_vec(0x77, get_cs(), FUNC_OFFSET(unhandled_irq15));

    for (i = 0x78; i < 0x100; i++) {
        set_int_vec(i, get_cs(), FUNC_OFFSET(unhandled_interrupt));
    }
}


void init()
{
    post(POST_CODE_INIT16);
    init_bios_data_area();

    call32();

    init_int_vector();

    platform_debug_print("log from 16bit");

    STI();
    outb(IO_PORT_PIC1 + 1, (inb(IO_PORT_PIC1 + 1) & ~1)); // unmask pit

    post(POST_CODE_TMP);

    for (;;);

    restart();
}

