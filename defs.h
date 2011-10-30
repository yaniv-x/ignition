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

#ifndef _H_DEFS
#define _H_DEFS

#ifdef _M_I86
#define FAR __far
#else
#define FAR
#endif

#define NULL 0
#define EOS 0

#define PAGE_SHIFT 12
#define PAGE_SIZE (1UL << PAGE_SHIFT)

#define TRUE 1
#define FALSE 0

#define KB 1024UL
#define MB (1024 * KB)
#define GB (1024 * MB)

#define IO_PORT_PIC1 0x20
#define IO_PORT_TIMER_0 0x40
#define IO_PORT_TIMER_1 0x41
#define IO_PORT_TIMER_2 0x42
#define IO_PORT_TIMER_CONTROL 0x43
#define IO_PORT_KBD_DATA 0x60
#define IO_PORT_MISC 0x61
#define IO_PORT_KBD_COMMAND 0x64
#define IO_PORT_KBD_STATUS IO_PORT_KBD_COMMAND
#define IO_PORT_RTC_INDEX 0x70
#define IO_PORT_RTC_DATA 0x71
#define IO_PORT_POST_CODE 0x80
#define IO_PORT_SYSCTRL 0x92
#define IO_PORT_PIC2 0xa0
#define IO_PORT_ELCR1 0x4d0
#define IO_PORT_ELCR2 0x4d1
#define IO_PORT_PCI_ADDRESS 0xcf8
#define IO_PORT_PCI_DATA 0xcfc

#define SYSCTRL_RESET_BIT 0
#define SYSCTRL_A20_BIT 1

#define RTC_NMI_MASK (1 << 7)

#define RTC_REG_A_DIVIDER_SHIFT 4
#define RTC_REG_A_DIVIDER_NORMAL 0x2
#define RTC_REG_A_DIVIDER_32 0x3
#define RTC_REG_A_DIVIDER_128 0x0
#define RTC_REG_A_RATE_MASK ((1 << 4) - 1)
#define RTC_REG_A_RATE_976U 6

#define RTC_REG_B_DAYLIGHT_MASK (1 << 0)
#define RTC_REG_B_24_HOUR_MASK (1 << 1)
#define RTC_REG_B_ENABLE_PERIODIC_MASK (1 << 6)

#define RTC_REG_C_PERIODIC_INTERRUPT_MASK (1 << 6)

#define PIC_ICW1_MASK (1 << 4)
#define PIC_ICW1_ICW4 (1 << 0)
#define PIC_ICW4_8086_MODE (1 << 0)
#define PIC_SPECIFIC_EOI_MASK (0x3 << 5)
#define PIC1_ADDRESS 0x08
#define PIC1_TIMER_PIN 0
#define PIC1_KEYBOARD_PIN 1
#define PIC1_SLAVE_PIN 2
#define PIC2_ADDRESS 0x70
#define PIC2_RTC_PIN 0
#define PIC2_MOUSE_PIN 4
#define PIC2_DMA_PIN 5

#define PIT_SELECTOR_SHIFT 6
#define PIT_RW_SHIFT 4
#define PIT_MODE_SHIFT 1

#define KBDCTRL_CMD_READ_COMMAND_BYTE 0x20
#define KBDCTRL_CMD_WRITE_COMMAND_BYTE 0x60
#define KBDCTRL_CMD_DISABLE_MOUSE 0xa7
#define KBDCTRL_CMD_ENABLE_MOUSE 0xa8
#define KBDCTRL_CMD_MOUSE_INTERFACE_TEST 0xa9
#define KBDCTRL_CMD_SELF_TEST 0xaa
#define KBDCTRL_CMD_KEYBOARD_INTERFACE_TEST 0xab
#define KBDCTRL_CMD_DISABLE_KEYBOARD 0xad
#define KBDCTRL_CMD_ENABLE_KEYBOARD 0xae
#define KBDCTRL_CMD_WRITE_TO_MOUSE 0xd4

#define KBDCTRL_STATUS_DATA_READY_MASK (1 << 0)
#define KBDCTRL_STATUS_WRITE_DISALLOWED_MASK (1 << 1)
#define KBDCTRL_STATUS_SELF_TEST_MASK (1 << 2)
#define KBDCTRL_STATUS_LAST_INPUT_COMMAND_MASK (1 << 3)
#define KBDCTRL_STATUS_MOUSE_DATA_READY_MASK (1 << 5)
#define KBDCTRL_SELF_TEST_REPLAY 0x55

#define KBDCTRL_COMMAND_BYTE_TRANSLATE_MASK (1 << 6)
#define KBDCTRL_COMMAND_BYTE_IRQ12_MASK (1 << 1)
#define KBDCTRL_COMMAND_BYTE_IRQ1_MASK (1 << 0)

#define KBD_CMD_RESET 0xff
#define KBD_CMD_ENABLE_SCANNING 0xf4

#define MOUSE_CMD_RESET 0xff

#define KBD_ACK 0xfa
#define KBD_SELF_TEST_REPLAY 0xaa

#define POST_CODE_START16   1
#define POST_CODE_INIT16    2
#define POST_CODE_START32   3
#define POST_CODE_INIT32    4
#define POST_CODE_DETECT_PLATFORM_FAILED  5
#define POST_CODE_PLATFORM_OK 6
#define POST_CODE_BAR_TYPE_FAILED 7
#define POST_CODE_BAR_SIZE_FAILED 8
#define POST_CODE_BAR_INDEX_INVALID 9
#define POST_CODE_BAR_MEM_TYPE_INVALID 10
#define POST_CODE_LOCKED 11
#define POST_CODE_DUMB_OOM 12
#define POST_CODE_PCI_OOM 13
#define POST_CODE_TODO_UPDATE_INT_LINE 14
#define POST_CODE_PCI_EXP_ROM_SIZE_INVALID 15
#define POST_CODE_BDA 16
#define POST_CODE_CPU 17
#define POST_CODE_RTC 18
#define POST_CODE_MEM 19
#define POST_CODE_PIT 20
#define POST_CODE_KEYBOARD 21
#define POST_CODE_MOUSE 22
#define POST_CODE_PIC 23
#define POST_CODE_BACK 24
#define POST_CODE_TMP 255

#define BASE_MEMORY_SIZE_KB 640UL

#define BIOS_DATA_AREA_ADDRESS 0x400
#define BIOS_DATA_AREA_SIZE 0x100
#define BIOS_EXTENDED_DATA_AREA_KB 1
#define BIOS_EXTENDED_DATA_AREA_ADDRESS ((BASE_MEMORY_SIZE_KB - BIOS_EXTENDED_DATA_AREA_KB) * KB)

#define BDA_SEG 0x40

#define BDA_OFFSET_EBDA 0x0e
#define BDA_OFFSET_EQUIPMENT 0x10
#define BDA_OFFSET_MAIN_MEM_SIZE 0x13
#define BDA_OFFSET_LAST_IRQ 0x6b
#define BDA_OFFSET_TICKS 0x6c
#define BDA_OFFSET_TICKS_ROLLOVER 0x70
#define BDA_OFFSET_WAIT_USR_PTR 0x98
#define BDA_OFFSET_WAIT_COUNT_MICRO 0x9c
#define BDA_OFFSET_WAIT_FLAGS 0xa0

#define BDA_EQUIPMENT_COPROCESSOR_BIT 1
#define BDA_EQUIPMENT_MOUSE_BIT 2

#define BDA_WAIT_IN_USE (1 << 0)
#define BDA_WAIT_ELAPSED (1 << 7)

#define EBDA_OFFSET_SIZE 0
#define EBDA_OFFSET_CACHE_CONTROL 0x68
#define EBDA_OFFSET_CPU_FAMILY 0xee
#define EBDA_OFFSET_CPU_STEPPING 0xef
#define EBDA_PRIVATE_START 0x140 // the last known used offset according to "The Unocumented PC"
                                 // is dword @ 11dh. leaving some space in order to be on the
                                 // safe side.

#define BIOS_MICRO_PER_TICK 976

#define PRIVATE_OFFSET_SS 0
#define PRIVATE_OFFSET_SP 2

#define CPU_FLAGS_ID_BIT 21
#define CPU_FLAGS_CF_BIT 0

// cpu id func 1 - edx
#define CPU_FEATURE_FPU_BIT 0
#define CPU_FEATURE_TSC_BIT 4
#define CPU_FEATURE_MSR_BIT 5
#define CPU_FEATURE_APIC_BIT 9
#define CPU_FEATURE_MTRR_BIT 12


#define MANDATORY_CPU_FEATURES_MASK (   \
    (1 << CPU_FEATURE_TSC_BIT) |        \
    (1 << CPU_FEATURE_MSR_BIT) |        \
    (1 << CPU_FEATURE_APIC_BIT) |       \
    (1 << CPU_FEATURE_MTRR_BIT)         \
)

#define CR0_PE (1 << 0)
#define CR0_CD (1 << 30)

#define SD_CS (0x18 << 8)
#define SD_DS (0x12 << 8)
#define SD_PRESENT (1 << 15)
#define SD_32 (1 << 22)
#define SD_4K (1 << 23)
#define SD_HLIMIT_SHIFT 16

#define CODE_SEGMENT_SELECTOR (1 << 3)
#define DATA_SEGMENT_SELECTOR (2 << 3)
#define CODE16_SEGMENT_SELECTOR (3 << 3)
#define DATA16_SEGMENT_SELECTOR (4 << 3)
#define UNREAL_SEGMENT_SELECTOR (5 << 3)

#define PROTECTED_START_ADDRESS ((1024 - 128) * 1024)
#define PROTECTED_STACK_BASE (1024 * 256)
#define BACK_FROM_PM_START_ADDRESS 0x0002
#define BIOS16_CODE_SEGMENT 0xf000

#define MSR_MTRR_CAP 0xfe
#define MSR_MTRR_PHYS_BASE_0 0x200
#define MSR_MTRR_PHYS_MASK_0 0x201
#define MSR_MTRR_PHYS_BASE_1 0x202
#define MSR_MTRR_PHYS_MASK_1 0x203
#define MSR_MTRR_PHYS_BASE_2 0x204
#define MSR_MTRR_PHYS_MASK_2 0x205
#define MSR_MTRR_PHYS_BASE_3 0x206
#define MSR_MTRR_PHYS_MASK_3 0x207
#define MSR_MTRR_PHYS_BASE_4 0x208
#define MSR_MTRR_PHYS_MASK_4 0x209
#define MSR_MTRR_PHYS_BASE_5 0x20a
#define MSR_MTRR_PHYS_MASK_5 0x20b
#define MSR_MTRR_PHYS_BASE_6 0x20c
#define MSR_MTRR_PHYS_MASK_6 0x20d
#define MSR_MTRR_PHYS_BASE_7 0x20e
#define MSR_MTRR_PHYS_MASK_7 0x20f
#define MSR_MTRR_FIX_64_0000 0x250
#define MSR_MTRR_FIX_16_8000 0x258
#define MSR_MTRR_FIX_16_A000 0x259
#define MSR_MTRR_FIX_4_C000 0x268
#define MSR_MTRR_FIX_4_C800 0x269
#define MSR_MTRR_FIX_4_D000 0x26a
#define MSR_MTRR_FIX_4_D800 0x26b
#define MSR_MTRR_FIX_4_E000 0x26c
#define MSR_MTRR_FIX_4_E800 0x26d
#define MSR_MTRR_FIX_4_F000 0x26e
#define MSR_MTRR_FIX_4_F800 0x26f
#define MSR_MTRR_DEFAULT 0x2ff


#define MTRR_MAX_VAR 8

#define MTRR_CAP_FIX_MASK (1 << 8)
#define MTRR_CAP_WC_MASK (1 << 10)
#define MTRR_CAP_COUNT_MASK (MTRR_CAP_FIX_MASK - 1)

#define MTRR_DEFAULT_ENABLE_MASK (1 << 11)
#define MTRR_DEFAULT_FIXED_ENABLE_MASK (1 << 10)
#define MTRR_DEFAULT_TYPE_MASK ((1 << 8) - 1)

#define MTRR_PHYS_MASK_VALID (1 << 11)

#endif

