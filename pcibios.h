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

#ifndef _H_PCIBIOS
#define _H_PCIBIOS

#include "base_types.h"

#define PCIBIOS_FUNC 0xb1
#define PCIBIOS_FUNC_PRESENT 0x01
#define PCIBIOS_FUNC_FIND_DEVICE 0x02
#define PCIBIOS_FUNC_FIND_CLASS 0x03
#define PCIBIOS_FUNC_READ_BYTE 0x08
#define PCIBIOS_FUNC_READ_WORD 0x09
#define PCIBIOS_FUNC_READ_DWORD 0x0a
#define PCIBIOS_FUNC_WRITE_BYTE 0x0b
#define PCIBIOS_FUNC_WRITE_WORD 0x0c
#define PCIBIOS_FUNC_WRITE_DWORD 0x0d
#define PCIBIOS_FUNC_GET_IRQ_OPTION 0x0e
#define PCIBIOS_FUNC_SET_PCI_IRQ 0x0f

#define PCIBIOS_SUCCESSFUL 0x00
#define PCIBIOS_FUNC_NOT_SUPPORTED 0x81
#define PCIBIOS_BAD_VANDOR_ID 0x83
#define PCIBIOS_DEVICE_NOT_FOUND 0x86
#define PCIBIOS_BAD_REGISTER 0x87
#define PCIBIOS_SET_FAILED 0x88
#define PCIBIOS_BUFFER_TOO_SMALL 0x89


typedef _Packed struct IRQOption {
    uint8_t bus;
    uint8_t device;

    uint8_t int_a_link;
    uint16_t int_a_bitmap;
    uint8_t int_b_link;
    uint16_t int_b_bitmap;
    uint8_t int_c_link;
    uint16_t int_c_bitmap;
    uint8_t int_d_link;
    uint16_t int_d_bitmap;

    uint8_t slot;
    uint8_t reserved;
} IRQOption;


typedef _Packed struct IRQRoutingOptionBuffer {
    uint16_t size;
    offset_t offset;
    uint16_t seg;
} IRQRoutingOptionBuffer;


#endif

