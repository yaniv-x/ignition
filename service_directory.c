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
#include "platform.h"

#define PCI_SERVICE_SIGNATURE FOUR_CHARS('$PCI')

void pcibios_service_entry(void);


void directory_service(UserRegs __far * context)
{
    platform_debug_string(__FUNCTION__);

    context->eflags &= ~(1 << CPU_FLAGS_CF_BIT);

    if (context->ebx) {
        AL(context) = 0x81;
        return;
    }

    switch (context->eax) {
    case PCI_SERVICE_SIGNATURE:
        AL(context) = 0x00;
        context->ebx = BIOS32_START_ADDRESS;
        context->ecx = BIOS32_SIZE;
        context->edx = (uint32_t)pcibios_service_entry;
        D_MESSAGE("pcibios returned");
        break;
    default:
        D_MESSAGE("no service 0x%x", context->eax);
        AL(context) = 0x80;
    }
}

