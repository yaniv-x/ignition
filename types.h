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

#ifndef _H_TYPES
#define _H_TYPES

#include "defs.h"

typedef unsigned __int8 uint8_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int64 uint64_t;
typedef unsigned int uint;
typedef uint32_t address_t;

typedef _Packed struct discriptor_t {
    uint32_t low;
    uint32_t high;
} discriptor_t;


typedef _Packed struct discriptor_reg_t {
    uint16_t limit;
    uint32_t address;
} discriptor_reg_t;


typedef _Packed struct EBDAPrivate {
    uint16_t real_mode_ss;
    uint16_t real_mode_sp;
    uint32_t below_1m_used_pages;
    uint32_t above_1m_pages;
    uint32_t below_4g_pages;
    uint32_t below_4g_used_pages;
    uint32_t above_4g_pages;
    uint32_t pci32_hole_start;
    uint32_t pci32_hole_end;
    uint64_t pci64_hole_start;

    uint16_t platform_io;
    address_t platform_ram;
    uint8_t address_lines;

    uint32_t alloc_start;
    uint32_t alloc_pos;
    uint32_t alloc_end;
    uint32_t flags;
    address_t activation_list;
    address_t io_bars;
    address_t mem_bars;
    address_t mem32_bars;
    address_t mem64_bars;
} EBDAPrivate;


typedef _Packed struct EBDA {
    uint8_t public[EBDA_PRIVATE_START];
    EBDAPrivate private;
} EBDA;


#endif

