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

static inline void outb(uint16_t port, uint8_t val)
{
    _asm {
        mov dx, port
        mov al, val
        out dx, al
    }
}


static inline void outw(uint16_t port, uint16_t val)
{
    _asm {
        mov dx, port
        mov ax, val
        out dx, ax
    }
}


static inline void outd(uint16_t port, uint32_t val)
{
    _asm {
        mov dx, port
        mov eax, val
        out dx, eax
    }
}


static inline uint8_t inb(uint16_t port)
{
    uint8_t val;

    _asm {
        mov dx, port
        in al, dx
        mov val, al
    }

    return val;
}


static inline uint16_t inw(uint16_t port)
{
    uint16_t val;

    _asm {
        mov dx, port
        in ax, dx
        mov val, ax
    }

    return val;
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


static uint32_t get_eflags()
{
    uint32_t r;

    __asm {
       pushf
       pop eax
       mov r, eax
    }

    return r;
}


static void put_eflags(uint32_t flags)
{
    __asm {
       push flags
       popf
    }
}


static inline void post(uint8_t code)
{
    outb(IO_PORT_POST_CODE, code);
}


static inline void restart()
{
    outb(IO_PORT_SYSCTRL, 1 << SYSCTRL_RESET_BIT);
}



static int find_lsb_32(uint32_t val)
{
    int i;

    for (i = 0; i < 32; i++) {
        if (val & (1 << i)) {
            return i;
        }
    }

    return -1;
}


static int find_msb_32(uint32_t val)
{
    int i;

    for (i = 31; i >=0; i--) {
        if (val & (1 << i)) {
            return i;
        }
    }

    return i;
}


static int find_lsb_64(uint64_t val)
{
    int i;

    for (i = 0; i < 64; i++) {
        if (val & (1ULL << i)) {
            return i;
        }
    }

    return -1;
}


static uint32_t format_put_x(char FAR *  dest, uint32_t len, uint64_t val, uint bits)
{
    static char conv_table[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
        'a', 'b', 'c', 'd', 'e', 'f',
    };

    uint32_t start_len = len;
    int shift = bits - 4;

    while (shift && !((val >> shift) & 0xf)) shift -=4;

    for (; len && shift >= 0 ; len--, dest++, shift -= 4) {
        *dest = conv_table[(val >> shift) & 0xf];
    }

    return start_len - len;
}


static uint32_t format_put_u(char FAR *  dest, uint32_t len, uint64_t val)
{
    uint32_t start_len = len;
    uint64_t tmp = val / 10;
    uint64_t div = 1;

    for (; tmp; div *= 10, tmp /= 10);

    for (; len && div; div /= 10, len--, dest++) {
        *dest = 0x30 + val / div;
        val %= div;
    }

    return start_len - len;
}


static uint32_t format_put_str(char FAR *  dest, uint32_t len, char FAR * str)
{
    uint32_t start_len;

    if (!str) {
        return format_put_str(dest, len, "NULL");
    }

    start_len = len;

    for (; len && *str; len--, str++, dest++) *dest = *str;

    return start_len - len;
}


static void format_str(char FAR *  dest, const char FAR * format, uint32_t len, ...)
{
    uint32_t FAR * args = &len + 1;
    uint advance = 0;
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

    if (!len--) {
        return;
    }

    for (; *format && len; format++) {
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
            *dest++ = *format;
            --len;
            continue;
        }

        switch (*format) {
        case 'x': {
            uint64_t val;
            uint bits;

            if (length == FORMAT_LENGTH_LL) {
                val = *(uint64_t FAR *)args;
                args += 2;
                bits = 64;
            } else {
#ifdef _M_I86
                if (length == FORMAT_LENGTH_L) {
                    val = *args++;
                    bits = 32;
                } else {
                    uint16_t FAR * short_arg = (uint16_t FAR *)args;
                    val = *short_arg++;
                    args = (uint32_t FAR *)short_arg;
                    bits = 16;
                }
#else
                val = *args++;
                bits = 32;
#endif
            }

            advance = format_put_x(dest, len, val, bits);

            break;
        }
        case 'u': {
            uint64_t val;

            if (length == FORMAT_LENGTH_LL) {
                val = *(uint64_t FAR *)args;
                args += 2;
            } else {
#ifdef _M_I86
                if (length == FORMAT_LENGTH_L) {
                    val = *args++;
                } else {
                    uint16_t FAR * short_arg = (uint16_t FAR *)args;
                    val = *short_arg;
                    args = (uint32_t FAR *)(short_arg + 1);
                }
#else
                val = *args++;
#endif
            }
            advance = format_put_u(dest, len, val);
            break;
        }
        case 's':
            advance = format_put_str(dest, len, (char FAR *)*args++);
            break;
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
            len = 0;
            break;
        default:
            // invalid format
            len = 0;
        }

        if (advance) {
            len -= advance;
            dest += advance;
            stage = FORMAT_STATE_ORDINARY;
            advance = 0;
            length = 0;
        }
    }

    *dest = 0;
}

