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

#ifndef _H_COMMON
#define _H_COMMON

#include "base_types.h"

#define ALIGN(a, b) (((a) + ((b) - 1)) & ~((b) - 1))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define OFFSET_OF(type, member) ((offset_t)&((type*)0)->member)
#define SIZE_OF(type, member) (sizeof(((type*)0)->member))

#define FOUR_CHARS(x) (((0UL + x) >> 24) | (((0UL + x) & 0x00ff0000UL) >> 8) | \
                       (((0UL + x) & 0x0000ff00UL) << 8) | ((0UL + x) << 24))

#define CLI() __asm { cli }
#define HALT() __asm { hlt}
#define INT(x) __asm { int x}

#define REG_HI(x) (((uint8_t __far *)&x)[1])
#define REG_LOW(x) (((uint8_t __far *)&x)[0])
#define REG_WORD(x) (((uint16_t __far *)&x)[0])

#define AL(context) REG_LOW(context->eax)
#define AH(context) REG_HI(context->eax)
#define AX(context) REG_WORD(context->eax)

#define BL(context) REG_LOW(context->ebx)
#define BH(context) REG_HI(context->ebx)
#define BX(context) REG_WORD(context->ebx)

#define CL(context) REG_LOW(context->ecx)
#define CH(context) REG_HI(context->ecx)
#define CX(context) REG_WORD(context->ecx)

#define DL(context) REG_LOW(context->edx)
#define DH(context) REG_HI(context->edx)
#define DX(context) REG_WORD(context->edx)
#define SI(context) REG_WORD(context->esi)
#define DI(context) REG_WORD(context->edi)

#ifdef _M_I86
#define FLAGS(context) context->flags
#else
#define FLAGS(context) REG_WORD(context->eflags)
#endif


#ifdef _M_I386
#define FAR_POINTER(type, seg, offset) \
                       ((type __far *)(((uint64_t)(seg) << 32) | (uint32_t)(offset)))
#else
#define FAR_POINTER(type, seg, offset) \
                       ((type __far *)(((uint32_t)(seg) << 16) | (uint16_t)(offset)))
#endif

#define SKIP_STACK_ARG(type, from) \
    (from = (uint8_t __far *)(from + ALIGN(sizeof(type), sizeof(int))))

#define POP_STACK_ARG(type, from) (                                     \
    from = (uint8_t __far *)(from + ALIGN(sizeof(type), sizeof(int))),  \
    *(type __far *)(from - ALIGN(sizeof(type), sizeof(int)))            \
)

#define ASSERT(x) if (!(x)) {                                                   \
    CLI();                                                                      \
    platform_debug_string(__FUNCTION__ ": ASSERT("#x") failed. halting...");    \
    freeze();                                                                   \
}

#define NO_INTERRUPT() ASSERT(!(get_eflags() & (1 << CPU_FLAGS_IF_BIT)))

#define D_MESSAGE(format, ...) platform_printf(__FUNCTION__ ": " format, ## __VA_ARGS__)

#ifdef SHOW_TRACE
#define TRACE(verb) {                                       \
    uint32_t flags = get_eflags();                          \
    uint16_t seg;                                           \
                                                            \
    CLI();                                                  \
    seg = get_ds();                                         \
    restore_ds();                                           \
    platform_printf("TRACE "#verb": %s", __FUNCTION__);     \
    set_ds(seg);                                            \
    put_eflags(flags);                                      \
}
#else
#define TRACE(verb)
#endif

#define TRACE_IN() TRACE(IN)
#define TRACE_OUT() TRACE(OUT)

uint32_t get_eflags();
void put_eflags(uint32_t flags);
void freeze();

void restart();
void post(uint8_t code);

int find_lsb_32(uint32_t val);
int find_lsb_64(uint64_t val);
int find_msb_32(uint32_t val);
int find_msb_64(uint64_t val);

void outb(uint16_t port, uint8_t val);
void outw(uint16_t port, uint16_t val);
void outd(uint16_t port, uint32_t val);

uint8_t inb(uint16_t port);
uint16_t inw(uint16_t port);
uint32_t ind(uint16_t port);


typedef void (*format_str_cb)(void __far * opaque, char ch);

void format_str(format_str_cb cb, void __far * opaque, const char __far * format,
                uint8_t __far * args);
void format_mem_str(char __far *  dest, uint len, const char __far * format, ...);

void mem_set(void __far * ptr, uint8_t patern, uint size);
void mem_reset(void __far * ptr, uint size);
void mem_copy(void __far * dest, const void __far * src, uint32_t size);

uint32_t string_length(const char __far *  str);
void string_copy(char __far *  dest, const char __far *  src);
void string_copy_n(char __far *  dest, const char __far *  src, uint dest_size);
int string_cmp(const char __far *  s1, const char __far *  s2);
int string_cmp_n(const char __far *  s1, const char __far *  s2, uint size);

int8_t checksum8(void __far * start, uint size);
int16_t checksum16(void __far * start, uint words);

void bios_error(uint16_t code);
void bios_warn(uint16_t code);
void bios_info(uint16_t code);

static inline uint64_t read_tsc()
{
    uint32_t hi;
    uint32_t low;

    __asm {
        rdtsc
        mov low, eax
        mov hi, edx
    }

    return ((uint64_t)hi << 32) | low;
}

#endif

