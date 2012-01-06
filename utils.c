
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
#include "utils.h"
#include "nox.h"
#include "platform.h"


void outb(uint16_t port, uint8_t val)
{
    _asm {
        mov dx, port
        mov al, val
        out dx, al
    }
}


void outw(uint16_t port, uint16_t val)
{
    _asm {
        mov dx, port
        mov ax, val
        out dx, ax
    }
}


void outd(uint16_t port, uint32_t val)
{
    _asm {
        mov dx, port
        mov eax, val
        out dx, eax
    }
}


uint8_t inb(uint16_t port)
{
    uint8_t val;

    _asm {
        mov dx, port
        in al, dx
        mov val, al
    }

    return val;
}


uint16_t inw(uint16_t port)
{
    uint16_t val;

    _asm {
        mov dx, port
        in ax, dx
        mov val, ax
    }

    return val;
}


uint32_t ind(uint16_t port)
{
    uint32_t val;

    _asm {
        mov dx, port
        in eax, dx
        mov val, eax
    }

    return val;
}


uint32_t get_eflags()
{
    uint32_t r;

    __asm {
       pushf
       pop eax
       mov r, eax
    }

    return r;
}


void put_eflags(uint32_t flags)
{
    __asm {
       push flags
       popf
    }
}


void post(uint8_t code)
{
    outb(IO_PORT_POST_CODE, code);
}


void restart()
{
    outb(IO_PORT_SYSCTRL, 1 << SYSCTRL_RESET_BIT);
}


void freeze()
{
    for (;;) {
        CLI();
        HALT();
    }
}


int find_lsb_32(uint32_t val)
{
    int i;

    for (i = 0; i < 32; i++) {
        if (val & (1 << i)) {
            return i;
        }
    }

    return -1;
}


int find_msb_32(uint32_t val)
{
    int i;

    for (i = 31; i >=0; i--) {
        if (val & (1 << i)) {
            return i;
        }
    }

    return i;
}


int find_lsb_64(uint64_t val)
{
    int i;

    for (i = 0; i < 64; i++) {
        if (val & (1ULL << i)) {
            return i;
        }
    }

    return -1;
}


void mem_set(void __far * ptr, uint8_t patern, uint size)
{
    uint8_t __far * now = (uint8_t __far *)ptr;
    uint8_t __far * end = now + size;

    for (; now < end; now++) *now = patern;
}


void mem_reset(void __far * ptr, uint size)
{
    mem_set(ptr, 0, size);
}


void mem_copy(void __far * dest, const void __far * src, uint32_t size)
{
    const uint8_t __far * from = src;
    uint8_t __far * to = dest;
    uint32_t i;

    for ( i = 0; i < size; i++) {
        to[i] = from[i];
    }
}


uint32_t string_length(const uint8_t __far * str)
{
    uint32_t len = 0;

    while (str[len]) len++;

    return len;
}


void string_copy(char __far *  dest, const char __far *  src)
{
    for (;; dest++, src++) {
        if (!(*dest = *src)) {
            return;
        }
    }
}


void string_copy_n(char __far *  dest, const char __far *  src, uint dest_size)
{
    const uint8_t __far * end;

    if (!dest_size) {
        return;
    }

    end = src + MIN(string_length(src), dest_size - 1);

    for (; src < end; src++, dest++) *dest = *src;

    *dest = EOS;
}


int string_cmp(const char __far *  s1, const char __far *  s2)
{
    uint i;

    for (i = 0; s1[i] && s1[i] == s2[i]; i++ ) ;

    return s1[i] - s2[i];
}


int string_cmp_n(const char __far *  s1, const char __far *  s2, uint n)
{
    uint i;

    for (i = 0; n && s1[i] && s1[i] == s2[i]; i++, n--) ;

    return n ? s1[i] - s2[i] : 0;
}


int8_t checksum8(void __far * start, uint size)
{
    int8_t res = 0;
    uint8_t __far * now = (uint8_t __far *)start;
    uint8_t __far * end = now + size;

    for (; now < end; now++) {
        res += *now;
    }

    return -res;
}


int16_t checksum16(void __far * start, uint size)
{
    int16_t res = 0;
    uint16_t __far * now = (uint16_t __far *)start;
    uint16_t __far * end = now + size;

    for (; now < end; now++) {
        res += *now;
    }

    return -res;
}


static void format_put_x(format_str_cb cb, void __far * opaque, uint64_t val, uint bits)
{
    static char conv_table[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
        'a', 'b', 'c', 'd', 'e', 'f',
    };

    int shift = bits - 4;

    while (shift && !((val >> shift) & 0xf)) shift -=4;

    for (; shift >= 0 ; shift -= 4) {
        cb(opaque, conv_table[(val >> shift) & 0xf]);
    }
}


static void format_put_u(format_str_cb cb, void __far * opaque, uint64_t val)
{
    uint64_t tmp = val / 10;
    uint64_t div = 1;

    for (; tmp; div *= 10, tmp /= 10);

    for (; div; div /= 10) {
        cb(opaque, 0x30 + val / div);
        val %= div;
    }
}


static void format_put_str(format_str_cb cb, void __far * opaque, const char * str)
{
    uint32_t start_len;

    if (!str) {
        format_put_str(cb, opaque, "NULL");
        return;
    }

    for (; *str; str++) cb(opaque, *str);
}


static void format_put_far_str(format_str_cb cb, void __far * opaque, const char __far * str)
{
    uint32_t start_len;

    if (!str) {
        format_put_str(cb, opaque, "NULL");
        return;
    }

    for (; *str; str++) cb(opaque, *str);
}


void format_str(format_str_cb cb, void __far * opaque, const char __far * format,
                uint8_t __far * args)
{
    uint consumed = FALSE;
    uint length = 0;

    enum {
        FORMAT_STATE_ORDINARY = 0,
        FORMAT_STATE_START,
        FORMAT_STATE_FLAGS,
        FORMAT_STATE_WIDTH,
        FORMAT_STATE_PRECISION,
        FORMAT_STATE_LENGTH,

        FORMAT_LENGTH_L = 1,
        FORMAT_LENGTH_LL = 2,
    };

    int stage = FORMAT_STATE_ORDINARY;

    for (; *format; format++) {
        if (*format == '%') {
            if (!stage) {
                stage = FORMAT_STATE_START;
                continue;
            }

            if (stage != FORMAT_STATE_START) {
                // invalid format
                break;
            }

            stage = FORMAT_STATE_ORDINARY;
        }

        if (!stage) {
            cb(opaque, *format);
            continue;
        }

        switch (*format) {
        case 'x': {
            uint64_t val;
            uint bits;

            if (length == FORMAT_LENGTH_LL) {
                val = POP_STACK_ARG(uint64_t, args);
                bits = 64;
            } else {
#ifdef _M_I86
                if (length == FORMAT_LENGTH_L) {
                    val = POP_STACK_ARG(uint32_t, args);
                    bits = 32;
                } else {
                    val = POP_STACK_ARG(uint16_t, args);
                    bits = 16;
                }
#else
                val = POP_STACK_ARG(uint32_t, args);
                bits = 32;
#endif
            }

            format_put_x(cb, opaque, val, bits);
            consumed = TRUE;
            break;
        }
        case 'u': {
            uint64_t val;

            if (length == FORMAT_LENGTH_LL) {
                val = POP_STACK_ARG(uint64_t, args);
            } else {
#ifdef _M_I86
                if (length == FORMAT_LENGTH_L) {
                    val = POP_STACK_ARG(uint32_t, args);
                } else {
                    val = POP_STACK_ARG(uint16_t, args);
                }
#else
                val = POP_STACK_ARG(uint32_t, args);
#endif
            }

            format_put_u(cb, opaque, val);
            consumed = TRUE;
            break;
        }
        case 's': {
            format_put_str(cb, opaque, POP_STACK_ARG(const char *, args));
            consumed = TRUE;
            break;
        }
        case 'S': {
            format_put_far_str(cb, opaque, POP_STACK_ARG(const char __far *, args));
            consumed = TRUE;
            break;
        }
        case 'l':
            stage = FORMAT_STATE_LENGTH;

            if (!length) {
                length = FORMAT_LENGTH_L;
                break;
            }

            if (length == FORMAT_LENGTH_L) {
                length = FORMAT_LENGTH_LL;
                break;
            }

            // invalid format
            return;
        default:
            // invalid format
            return;
        }

        if (consumed) {
            stage = FORMAT_STATE_ORDINARY;
            consumed = FALSE;
            length = 0;
        }
    }
}


typedef struct FormatMemStr {
    char __far *  dest;
    uint pos;
    uint end;
} FormatMemStr;


static void format_str_mem_cb(void __far * opaque, char ch)
{
    FormatMemStr __far * data = ( FormatMemStr __far *)opaque;

    if (data->pos == data->end) {
        return;
    }

    data->dest[data->pos++] = ch;
}


void format_mem_str(char __far *  dest, uint len, const char __far * format, ...)
{
    uint8_t __far * args;
    FormatMemStr data;

    if (len == 0) {
        return;
    }

    data.dest = dest;
    data.pos = 0;
    data.end = len - 1;

    args = (uint8_t __far *)&format;
    format_str(format_str_mem_cb, &data, format, SKIP_STACK_ARG(const char __far *, args));

    dest[data.pos] = 0;
}


void bios_error(uint16_t code)
{
    uint32_t error_code = PLATFORM_MK_ERR(PLATFORM_ERR_TYPE_ERROR, PLATFORM_ERR_SUBSYS_BIOS, code);
    platform_report_error(error_code);
    freeze();
}


void bios_warn(uint16_t code)
{
    uint32_t warn_code = PLATFORM_MK_ERR(PLATFORM_ERR_TYPE_WARN, PLATFORM_ERR_SUBSYS_BIOS, code);
    platform_report_error(warn_code);
}


void bios_info(uint16_t code)
{
    uint32_t info_code = PLATFORM_MK_ERR(PLATFORM_ERR_TYPE_INFO, PLATFORM_ERR_SUBSYS_BIOS, code);
    platform_report_error(info_code);
}

