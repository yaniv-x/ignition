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


void call32(void);
void unhandled_interrupt(void);


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


static void init_int_vector()
{
    uint8_t i = 0;

    do {
        set_int_vec(i, get_cs(), FUNC_OFFSET(unhandled_interrupt));
    } while (++i);
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


static inline void init_bios_data_area()
{
    post(POST_CODE_BDA);

    mem_reset(0, BIOS_DATA_AREA_ADDRESS, BIOS_DATA_AREA_SIZE);
    mem_reset(BIOS_EXTENDED_DATA_AREA_ADDRESS >> 4, 0, BIOS_EXTENDED_DATA_AREA_KB * KB);

    bda_write_word(BDA_OFFSET_EBDA, BIOS_EXTENDED_DATA_AREA_ADDRESS >> 4);
    ebda_write_byte(EBDA_OFFSET_SIZE, BIOS_EXTENDED_DATA_AREA_KB);
}


void init()
{
    post(POST_CODE_INIT16);
    init_bios_data_area();
    init_int_vector();

    call32();

    for (;;) post(POST_CODE_TMP);

    restart();
}

