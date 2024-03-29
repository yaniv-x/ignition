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

#ifndef _H_PLATFORM
#define _H_PLATFORM

#include "types.h"
#include "utils.h"
#include "nox.h"

#define BIOS_PRIVATE_READ(member, ptr) {                                        \
    ASSERT(sizeof(*ptr) == SIZE_OF(PrivateData, member));                       \
    platform_read(PLATFORM_BIOS_DATA_START + OFFSET_OF(PrivateData, member),    \
                  ptr, SIZE_OF(PrivateData, member));                           \
}

#define BIOS_PRIVATE_WRITE(member, ptr) {                                       \
    ASSERT(sizeof(*ptr) == SIZE_OF(PrivateData, member));                       \
    platform_write(PLATFORM_BIOS_DATA_START + OFFSET_OF(PrivateData, member),   \
                  ptr, SIZE_OF(PrivateData, member));                           \
}

void platform_report_error(uint32_t code);
void platform_read(uint32_t offset, void __far * in_dest, uint32_t size);
void platform_write(uint32_t offset, const void __far * in_src, uint32_t size);
void platform_command(uint8_t cmd, void __far * args, uint32_t args_size);
void platform_debug_string(const char __far * str);
void platform_printf(const char __far * format, ...);
uint32_t platform_get_reg(uint8_t reg_index);

#endif

