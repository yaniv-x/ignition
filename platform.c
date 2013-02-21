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

#include "platform.h"
#include "pci.h"
#include "utils.h"
#include "nox.h"


static uint16_t get_port()
{
    return pci_read_32(0, 0, PCI_OFFSET_BAR_0) & PCI_BAR_IO_ADDRESS_MASK;
}


void platform_report_error(uint32_t code)
{
    uint32_t flags = get_eflags();
    uint16_t port;

    CLI();
    port = get_port();

    if (port) {
        outd(port + PLATFORM_IO_ERROR, code);
    }

    put_eflags(flags);
}


void platform_read(uint32_t offset, void __far * in_dest, uint32_t size)
{
    uint8_t __far * dest = in_dest;
    uint16_t port;

    NO_INTERRUPT();
    ASSERT(offset + size < (PLATFORM_MEM_PAGES << PAGE_SHIFT));

    port = get_port();
    outb(port + PLATFORM_IO_SELECT, PLATFORM_REG_READ_POS);
    outd(port + PLATFORM_IO_REGISTER, offset);

    for ( ; size--; dest++) {
        *dest = inb(port + PLATFORM_IO_BYTE);
    }
}


void platform_write(uint32_t offset, const void __far * in_src, uint32_t size)
{
    const uint8_t __far * src = in_src;
    uint16_t port;

    NO_INTERRUPT();

    port = get_port();
    outb(port + PLATFORM_IO_SELECT, PLATFORM_REG_WRITE_POS);
    outd(port + PLATFORM_IO_REGISTER, offset);

    for ( ; size--; src++) {
        outb(port + PLATFORM_IO_BYTE, *src);
    }
}

void platform_debug_string(const char __far * str)
{
    uint32_t eflags = get_eflags();
    CLI();

    platform_write(PLATFORM_LOG_BUF_START, str, string_length(str) + 1);
    outb(get_port() + PLATFORM_IO_LOG, 0);

    put_eflags(eflags);
}


void platform_command(uint8_t cmd, void __far * args, uint32_t args_size)
{
    uint32_t eflags = get_eflags();
    CLI();

    ASSERT(args_size <= PLATFORM_CMD_BUF_SIZE);

    platform_write(PLATFORM_CMD_BUF_START, args, args_size);
    outb(get_port() + PLATFORM_IO_CMD, cmd);
    platform_read(PLATFORM_CMD_BUF_START, args, args_size);

    put_eflags(eflags);
}


uint32_t platform_get_reg(uint8_t reg_index)
{
    uint32_t eflags = get_eflags();
    uint32_t ret;

    CLI();

    ASSERT(reg_index < PLATFORM_REG_NUM_REGS);

    outb(get_port() + PLATFORM_IO_SELECT, reg_index);
    ret = ind(get_port() + PLATFORM_IO_REGISTER);

    put_eflags(eflags);

    return ret;
}


typedef struct PlatformPrintf {
    uint pos;
    uint end;
    uint16_t port;
} PlatformPrintf;


static void platform_printf_cb(void __far * opaque, char ch)
{
    PlatformPrintf __far * data = (PlatformPrintf __far *)opaque;

    if (data->pos == data->end) {
        return;
    }

    outb(data->port + PLATFORM_IO_BYTE, ch);
    data->pos++;
}


void platform_printf(const char __far * format, ...)
{
    uint8_t __far * args;
    PlatformPrintf data;
    uint32_t eflags = get_eflags();

    CLI();

    data.pos = 0;
    data.end = PLATFORM_LOG_BUF_SIZE - 1;
    data.port = get_port();

    outb(data.port + PLATFORM_IO_SELECT, PLATFORM_REG_WRITE_POS);
    outd(data.port + PLATFORM_IO_REGISTER, PLATFORM_LOG_BUF_START);

    args = (uint8_t __far *)&format;
    format_str(platform_printf_cb, &data, format, SKIP_STACK_ARG(const char __far *, args));

    outb(data.port + PLATFORM_IO_BYTE, 0);
    outb(data.port + PLATFORM_IO_LOG, 0);

    put_eflags(eflags);
}

