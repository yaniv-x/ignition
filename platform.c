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


static uint16_t pci_get_platform_io()
{
    return pci_read_32(0, 0, PCI_OFFSET_BAR_0) & PCI_BAR_IO_ADDRESS_MASK;
}


void platform_read(uint32_t offset, void __far * in_dest, uint32_t size)
{
    uint8_t __far * dest = in_dest;
    uint16_t port = pci_get_platform_io();

    NO_INTERRUPT();
    ASSERT(offset + size < (PLATFORM_MEM_PAGES << PAGE_SHIFT));

    outb(port + PLATFORM_IO_SELECT, PLATFORM_REG_READ_POS);
    outd(port + PLATFORM_IO_REGISTER, offset);

    for ( ; size--; dest++) {
        *dest = inb(port + PLATFORM_IO_BYTE);
    }
}


void platform_write(uint32_t offset, const void __far * in_src, uint32_t size)
{
    const uint8_t __far * src = in_src;
    uint16_t port = pci_get_platform_io();

    NO_INTERRUPT();

    outb(port + PLATFORM_IO_SELECT, PLATFORM_REG_WRITE_POS);
    outd(port + PLATFORM_IO_REGISTER, offset);

    for ( ; size--; src++) {
        outb(port + PLATFORM_IO_BYTE, *src);
    }
}

void platform_debug_string(const char __far * str)
{
    NO_INTERRUPT();

    platform_write(PLATFORM_LOG_BUF_START, str, string_length(str) + 1);
    outb(pci_get_platform_io() + PLATFORM_IO_LOG, 0);
}


void platform_command(uint8_t cmd, void __far * args, uint32_t args_size)
{
    ASSERT(args_size <= PLATFORM_CMD_BUF_SIZE);
    platform_write(PLATFORM_CMD_BUF_START, args, args_size);
    outb(pci_get_platform_io() + PLATFORM_IO_CMD, cmd);
}

