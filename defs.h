/*
    Copyright (c) 2013-2014 Yaniv Kamay,
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

#define NULL 0
#define EOS 0

#define PAGE_SHIFT 12
#define PAGE_SIZE (1UL << PAGE_SHIFT)

#define TRUE 1
#define FALSE 0

#define KB 1024UL
#define MB (1024UL * KB)
#define GB (1024ULL * MB)

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
#define IO_PORT_SYSCTRL 0x92
#define IO_PORT_PIC2 0xa0
#define IO_PORT_POST_CODE 0x300
#define IO_PORT_VGA_PALETTE_WRITE_INDEX 0x3c8
#define IO_PORT_VGA_PALETTE_DATA 0x3c9
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
#define RTC_REG_A_UPDATE_IN_PROGRESS (1 << 7)

#define RTC_REG_B_DAYLIGHT_MASK (1 << 0)
#define RTC_REG_B_24_HOUR_MASK (1 << 1)
#define RTC_REG_B_BINARY_MASK (1 << 2)
#define RTC_REG_B_ENABLE_UPDATE_MASK (1 << 4)
#define RTC_REG_B_ENABLE_ALARM_MASK (1 << 5)
#define RTC_REG_B_ENABLE_PERIODIC_MASK (1 << 6)
#define RTC_REG_B_HALT_CLOCK_MASK (1 << 7)

#define RTC_REG_C_INTERRUPT_MASK (1 << 7)
#define RTC_REG_C_PERIODIC_INTERRUPT_MASK (1 << 6)
#define RTC_REG_C_ALARM_INTERRUPT_MASK (1 << 5)
#define RTC_REG_C_UPDATE_INTERRUPT_MASK (1 << 4)

#define RTC_SECONDS 0x00
#define RTC_SECONDS_ALARM 0x01
#define RTC_MINUTES 0x02
#define RTC_MINUTES_ALARM 0x03
#define RTC_HOURS 0x04
#define RTC_HOURS_ALARM 0x05
#define RTC_DAY_OF_WEEK 0x06
#define RTC_DAY_OF_MOUNTH 0x07
#define RTC_MOUNTH 0x08
#define RTC_YEAR 0x09
#define RTC_REGA 0x0a
#define RTC_REGB 0x0b
#define RTC_REGC 0x0c
#define RTC_REGD 0x0d

#define CMOS_OFFSET_DIAGNOSTIC 0x0e
#define CMOS_OFFSET_SHUTDOWN_STASUS 0x0f
#define CMOS_OFFSET_FLOPPY_TYPE 0x10
#define CMOS_OFFSET_HD_TYPE 0x12
#define CMOS_OFFSET_EQUIPMENT_BYTE 0x14
#define CMOS_OFFSET_BASE_MEM_LOW 0x15
#define CMOS_OFFSET_BASE_MEM_HIGH 0x16
#define CMOS_OFFSET_EXT_MEM_LOW_0 0x17
#define CMOS_OFFSET_EXT_MEM_HIGH_0 0x18
#define CMOS_OFFSET_HD0_EXT_TYPE 0x19
#define CMOS_OFFSET_HD1_EXT_TYPE 0x1a
#define CMOS_OFFSET_HD0_PARAMS 0x1b
#define CMOS_OFFSET_HD1_PARAMS 0x24
#define CMOS_OFFSET_EXT_MEM_LOW_1 0x30
#define CMOS_OFFSET_EXT_MEM_HIGH_1 0x31
#define CMOS_OFFSET_ISA_CENTURY 0x32 // remove ?
#define CMOS_OFFSET_CENTURY 0x37

#define PIC_ICW1_MASK (1 << 4)
#define PIC_ICW1_ICW4 (1 << 0)
#define PIC_ICW4_8086_MODE (1 << 0)
#define PIC_SPECIFIC_EOI_MASK (0x3 << 5)
#define PIC_NUM_LINES 8
#define PIC_NUM_CHIPS 2
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
#define PIT_FREQUENCY 1193182UL

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
#define KBDCTRL_CMD_WRITE_KBD_OUTPUT 0xd2

#define KBDCTRL_STATUS_DATA_READY_MASK (1 << 0)
#define KBDCTRL_STATUS_WRITE_DISALLOWED_MASK (1 << 1)
#define KBDCTRL_STATUS_SELF_TEST_MASK (1 << 2)
#define KBDCTRL_STATUS_LAST_INPUT_COMMAND_MASK (1 << 3)
#define KBDCTRL_STATUS_MOUSE_DATA_READY_MASK (1 << 5)
#define KBDCTRL_SELF_TEST_REPLAY 0x55

#define KBDCTRL_COMMAND_BYTE_TRANSLATE_MASK (1 << 6)
#define KBDCTRL_COMMAND_BYTE_IRQ12_MASK (1 << 1)
#define KBDCTRL_COMMAND_BYTE_IRQ1_MASK (1 << 0)

#define KBD_CMD_LED 0xed
#define KBD_CMD_GET_ID 0xf2
#define KBD_CMD_ENABLE_SCANNING 0xf4
#define KBD_CMD_REPEAT_RATE 0xf3
#define KBD_CMD_RESET 0xff

#define KBD_SCAN_ESCAPE 0x01
#define KBD_SCAN_L_CTRL 0x1d
#define KBD_SCAN_L_SHIFT 0x2a
#define KBD_SCAN_R_SHIFT 0x36
#define KBD_SCAN_L_ALT 0x38
#define KBD_SCAN_CAPS_LOCK 0x3a
#define KBD_SCAN_NUM_LOCK 0x45
#define KBD_SCAN_SCROLL_LOCK 0x46
#define KBD_SCAN_PAD_UP 0x48
#define KBD_SCAN_PAD_DOWN 0x50
#define KBD_SCAN_INSERT 0x52
#define KBD_SCAN_DEL 0x53
#define KBD_SCAN_SYS_REQ 0x54

#define KBD_EXT_PAD_ENTER 0x1c
#define KBD_EXT_SCAN_R_CTRL 0x1d
#define KBD_EXT_PAD_DIV 0x35
#define KBD_EXT_SCAN_PRINT 0x37
#define KBD_EXT_SCAN_R_ALT 0x38
#define KBD_EXT_SCAN_INSERT 0x52
#define KBD_EXT_DEL 0x53
#define KBD_EXT_LEFT_META 0x54
#define KBD_EXT_RIGHT_META 0x5c
#define KBD_EXT_RIGHT_MENU 0x5d

#define KBD_DEFAULT_RATE 0x0b /* 10.9 characters per second */
#define KBD_DEFAULT_DELAY 1 /* 500 ms delay */

#define KBD_BREAK_MASK (1 << 7)

#define KBD_MAX_KEY_SCAN 0x58
#define KBD_EXT_TRANS_START 0x47

#define MOUSE_CMD_SCALING_1_1 0xe6
#define MOUSE_CMD_SCALING_2_1 0xe7
#define MOUSE_CMD_RESOLUTION 0xe8
#define MOUSE_CMD_STATUS 0xe9
#define MOUSE_CMD_ENABLE_DATA_REPORTING 0xf4
#define MOUSE_CMD_SAMPLE_RATE 0xf3
#define MOUSE_CMD_ENABLE_DATA_REPORTING 0xf4
#define MOUSE_CMD_DISABLE_DATA_REPORTING 0xf5
#define MOUSE_CMD_RESET 0xff

#define KBD_ACK 0xfa
#define KBD_NAK 0xfe
#define KBD_SELF_TEST_REPLAY 0xaa

#define POST_CODE_START16 5
#define POST_CODE_INIT16 10
#define POST_CODE_INIT32 15
#define POST_CODE_DETECT_CPU_FAILED 20
#define POST_CODE_DETECT_CPU_OK 25
#define POST_CODE_DETECT_PLATFORM_FAILED 30
#define POST_CODE_PLATFORM_OK 35
#define POST_CODE_BAR_TYPE_FAILED 40
#define POST_CODE_BAR_SIZE_FAILED 45
#define POST_CODE_BAR_INDEX_INVALID 50
#define POST_CODE_BAR_MEM_TYPE_INVALID 55
#define POST_CODE_LOCKED 60
#define POST_CODE_DUMB_OOM 65
#define POST_CODE_PCI_OOM 70
#define POST_CODE_TODO_UPDATE_INT_LINE 75
#define POST_CODE_PCI_EXP_ROM_SIZE_INVALID 80
#define POST_CODE_BDA 85
#define POST_CODE_CPU 90
#define POST_CODE_RTC 95
#define POST_CODE_MEM 100
#define POST_CODE_PIT 105
#define POST_CODE_KEYBOARD 110
#define POST_CODE_MOUSE 115
#define POST_CODE_PIC 120
#define POST_CODE_AP16 125
#define POST_CODE_TMP 255

#define BASE_MEMORY_SIZE_KB 640UL

#define BIOS_DATA_AREA_ADDRESS 0x400
#define BIOS_DATA_AREA_SIZE 0x100
#define BIOS_HARD_INT_STACK_SIZE_KB 1
#define BIOS_EBDA_DATA_KB 4
#define BIOS_EBDA_SIZE_KB (BIOS_HARD_INT_STACK_SIZE_KB + BIOS_EBDA_DATA_KB)
#define BIOS_EBDA_ADDRESS ((BASE_MEMORY_SIZE_KB - BIOS_EBDA_SIZE_KB) * KB)
#define BIOS_HARD_INT_STACK_ADDRESS (BIOS_EBDA_ADDRESS + BIOS_EBDA_DATA_KB * KB)

#define BIOS_FLAGS_ALRM_ACTIVE_MASK (1 << 0)
#define BIOS_FLAGS_KBD_LEDS_DATA (1 << 1)
#define BIOS_FLAGS_KBD_WAIT (1 << 2)
#define BIOS_FLAGS_HARD_INT (1 << 3)
#define BIOS_FLAGS_UNREAL (1 << 4)
#define BIOS_FLAGS_KBD_RATE_TRIGGER (1 << 5)
#define BIOS_FLAGS_KBD_RATE_CMD_ACK (1 << 6)
#define BIOS_FLAGS_KBD_RATE_DATA_ACK (1 << 7)
#define BIOS_FLAGS_HD_EMULATION (1 << 8)
#define BIOS_FLAGS_FD_EMULATION (1 << 9)

#define BIOS_BEEP_Hz 1760
#define BIOS_BEEP_DURATION_MS 64

// every 24 hours (24 * 60 * 60 * 1000 / (2^16 * 1000 / PIT_FREQUENCY))
#define BISO_SYS_TIME_ROLLOVER 0x1800b2UL

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
#define BDA_OFFSET_KBD_FLAGS_1 0x17
#define BDA_OFFSET_KBD_ALT_PAD_AREA 0x19
#define BDA_OFFSET_KBD_HEAD 0x1a
#define BDA_OFFSET_KBD_TAIL 0x1c
#define BDA_OFFSET_KBD_BUF 0x1e
#define BDA_OFFSET_KBD_CTRL_BREAK 0X71
#define BDA_OFFSET_KBD_BUF_START 0X80
#define BDA_OFFSET_KBD_BUF_END 0X82
#define BDA_OFFSET_KBD_FLAGS_2 0x96
#define BDA_OFFSET_HD_ATTACHED 0x75
#define BDA_OFFSET_HD_RESAULT 0x74
#define BDA_OFFSET_HD_CONTROL_BYTE 0x76

#define BDA_KBD_DEFAULT_BUF_SIZE 32

#define BDA_KBD_FLAGS_1_LEDS_SHIFT 4

#define BDA_KBD_FLAGS_1_INSERT_DOWN (1 << 15)
#define BDA_KBD_FLAGS_1_CAPS_DOWN (1 << 14)
#define BDA_KBD_FLAGS_1_NUM_DOWN (1 << 13)
#define BDA_KBD_FLAGS_1_SCROLL_DOWN (1 << 12)
#define BDA_KBD_FLAGS_1_POUSE_ACTIVE (1 << 11)
#define BDA_KBD_FLAGS_1_SYS (1 << 10)
#define BDA_KBD_FLAGS_1_L_ALT (1 << 9)
#define BDA_KBD_FLAGS_1_L_CTRL (1 << 8)
#define BDA_KBD_FLAGS_1_INSERT (1 << 7)
#define BDA_KBD_FLAGS_1_CAPS_LOCK (1 << 6)
#define BDA_KBD_FLAGS_1_NUM_LOCK (1 << 5)
#define BDA_KBD_FLAGS_1_SCROLL_LOCK (1 << 4)
#define BDA_KBD_FLAGS_1_ALT (1 << 3)
#define BDA_KBD_FLAGS_1_CTRL (1 << 2)
#define BDA_KBD_FLAGS_1_L_SHIFT (1 << 1)
#define BDA_KBD_FLAGS_1_R_SHIFT (1 << 0)

#define BDA_KBD_FLAGS_2_LEDS_SHIFT 8
#define BDA_KBD_FLAGS_2_LEDS_MASK (0x7 << BDA_KBD_FLAGS_2_LEDS_SHIFT)

#define BDA_KBD_FLAGS_2_LEDS_IN_PROGRESS (1 << 14)
#define BDA_KBD_FLAGS_2_CAPS_LED (1 << 10)
#define BDA_KBD_FLAGS_2_NUM_LED (1 << 9)
#define BDA_KBD_FLAGS_2_SCROLL_LED (1 << 8)
#define BDA_KBD_FLAGS_2_KB_TYPE (1 << 4)
#define BDA_KBD_FLAGS_2_R_ALT (1 << 3)
#define BDA_KBD_FLAGS_2_R_CTRL (1 << 2)
#define BDA_KBD_FLAGS_2_E0 (1 << 1)
#define BDA_KBD_FLAGS_2_E1 (1 << 0)

#define BDA_EQUIPMENT_COPROCESSOR_BIT 1
#define BDA_EQUIPMENT_MOUSE_BIT 2

#define BDA_WAIT_IN_USE (1 << 0)
#define BDA_WAIT_ELAPSED (1 << 7)

#define EBDA_OFFSET_SIZE 0
#define EBDA_OFFSET_MOUSE_HANDLER 0x22
#define EBDA_OFFSET_MOUSE_FLAGS 0x26
#define EBDA_OFFSET_MOUSE_DATA 0x28
#define EBDA_OFFSET_FDPT_0 0x3d
#define EBDA_OFFSET_FDPT_1 0x4d
#define EBDA_OFFSET_CACHE_CONTROL 0x68
#define EBDA_OFFSET_HD_COUNT 0x70
#define EBDA_OFFSET_CPU_FAMILY 0xee
#define EBDA_OFFSET_CPU_STEPPING 0xef
#define EBDA_OFFSET_KBD_RATE 0x6e
#define EBDA_OFFSET_KBD_DELAY 0x6f
#define EBDA_OFFSET_KBD_ID 0x117

#define EBDA_PUBLIC_END 0x140 // the last known used offset according to "The Unocumented PC"
                                 // is dword @ 11dh. leaving some space in order to be on the
                                 // safe side.
#define EBDA_ACPI_SIZE 36
#define EBDA_PRIVATE_START (EBDA_PUBLIC_END + EBDA_ACPI_SIZE)

#define EBDA_MOUSE_FLAGS_HANDLER (1 << 15)
#define EBDA_MOUSE_FLAGS_ENABLED (1 << 14)
#define EBDA_MOUSE_FLAGS_INITILIZED (1 << 13)
#define EBDA_MOUSE_FLAGS_READY (1 << 12)
#define EBDA_MOUSE_FLAGS_PKT_SIZE_SHIFT 8
#define EBDA_MOUSE_FLAGS_PKT_SIZE_MASK (0x7 << EBDA_MOUSE_FLAGS_PKT_SIZE_SHIFT)
#define EBDA_MOUSE_FLAGS_CMD_IN_PROGRESS (1 << 7)
#define EBDA_MOUSE_FLAGS_RESEND (1 << 6)
#define EBDA_MOUSE_FLAGS_ACK (1 << 5)
#define EBDA_MOUSE_FLAGS_ERR (1 << 4)
#define EBDA_MOUSE_FLAGS_UNEXPECTED (1 << 3)
#define EBDA_MOUSE_FLAGS_DATA_POS_MASK (EBDA_MOUSE_FLAGS_UNEXPECTED - 1)

#define BIOS_PERIODIC_MICRO 976
#define BIOS_PERIODIC_USER_WAIT_SERVICE (1 << 0)
#define BIOS_PERIODIC_USER_INTERNAL_DELAY (1 << 1)

#define INT13_FUNC_DISK_CONTROLLER_RESET 0x00
#define INT13_FUNC_READ_SECTORS 0x02
#define INT13_FUNC_WRITE_SECTORS 0x03
#define INT13_FUNC_GET_DRIVE_PARAMETERS 0x08
#define INT13_FUNC_RESET 0x0d
#define INT13_FUNC_GET_DISK_TYPE 0x15
#define INT13_FUNC_EDD_INSTALLATION_CHECK 0x41
#define INT13_FUNC_EDD_READ_SECTORS 0x42
#define INT13_FUNC_EDD_WRITE_SECTORS 0x43
#define INT13_FUNC_EDD_VERIFY 0x44
#define INT13_FUNC_EDD_SEEK 0x47
#define INT13_FUNC_EDD_GET_DRIVE_PARAMS 0x48
#define INT13_FUNC_TERM_DISK_EMULATION 0x4b
    #define INT13_TERM_DISK_EMULATION_TERM 0
    #define INT13_TERM_DISK_EMULATION_QUERY 1

#define INT15_FUNC_KBD_INTERCEPT 0x4f
#define INT15_FUNC_APM 0x53
#define INT15_FUNC_EVENT_WAIT_CTRL 0x83
    #define INT15_EVENT_WAIT_SET 0
    #define INT15_EVENT_WAIT_CANCEL 1
#define INT15_FUNC_WAIT 0x86
#define INT15_FUNC_COPY_EXT_MEM 0x87
#define INT15_FUNC_GET_EXT_MEMORY_SIZE 0x88
#define INT15_FUNC_GET_BIG_MEMORY_SIZE 0x8a
#define INT15_FUNC_DEVICE_BUSY 0x90
#define INT15_FUNC_DEVICE_POST 0x91
#define INT15_FUNC_GET_CONFIG 0xc0
#define INT15_FUNC_GET_EBDA_SEG 0xc1
#define INT15_FUNC_MOUSE 0xc2
    #define INT15_MOUSE_ENABLE_DISABLE 0
    #define INT15_MOUSE_RESET 1
    #define INT15_MOUSE_SET_SAMPLING_RATE 2
    #define INT15_MOUSE_SET_RESOLUTION 3
    #define INT15_MOUSE_GET_ID 4
    #define INT15_MOUSE_INITIALIZE 5
    #define INT15_EXTENDED_COMMANDS 6
    #define INT15_SET_HANDLER 7
#define INT15_FUNC_BIG_MEM 0xe8
    #define INT15_BIG_MEM_GET_SIZE 0x01
    #define INT15_BIG_MEM_GET_MAP 0x20
#define INT15_FUNC_TARGRT_OPERATING_MODE 0xec

#define INT16_FUNC_READ_KEY 0x00
#define INT16_FUNC_PEEK_KEY 0x01
#define INT16_FUNC_GET_SHIFT_FLAGS 0x02
#define INT16_FUNC_RATE_AND_DELAY 0x03
    #define INT16_RATE_AND_DELAY_RESET 0
    #define INT16_RATE_AND_DELAY_SET 5
    #define INT16_RATE_AND_DELAY_GET 6
#define INT16_FUNC_STOR_KEY 0x05
#define INT16_FUNC_F3_CAP 0x09
#define INT16_FUNC_READ_KEY_EXT 0x10
#define INT16_FUNC_PEEK_KEY_EXT 0x11
#define INT16_FUNC_GET_EXT_SHIFT_FLAGS 0x12

#define INT1A_FUNC_GET_SYS_TIME 0x00
#define INT1A_FUNC_SET_SYS_TIME 0x01
#define INT1A_FUNC_GET_TIME 0x02
#define INT1A_FUNC_SET_TIME 0x03
#define INT1A_FUNC_GET_DATE 0x04
#define INT1A_FUNC_SET_DATE 0x05
#define INT1A_FUNC_SET_ALARM 0x06
#define INT1A_FUNC_CANCEL_ALARM 0x07
#define INT1A_FUNC_PCIBIOS 0xb1

#define PRIVATE_OFFSET_PM_STACK_BASE 0
#define PRIVATE_OFFSET_REAL_MODE_SS 4
#define PRIVATE_OFFSET_REAL_MODE_SP 6
#define PRIVATE_OFFSET_USER_SS 8
#define PRIVATE_OFFSET_USER_SP 10
#define PRIVATE_OFFSET_HARD_INT_SS 12
#define PRIVATE_OFFSET_HARD_INT_SP 14
#define PRIVATE_OFFSET_FLAGS 16
#define PRIVATE_OFFSET_INT13_EMU_SEG 18
#define PRIVATE_OFFSET_INT13_EMU_OFFSET 20
#define PRIVATE_OFFSET_AP_LOCK 22

#define CPU_FLAGS_ID_BIT 21
#define CPU_FLAGS_IF_BIT 9
#define CPU_FLAGS_ZF_BIT 6
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

#define CR0_CD (1 << 30)
#define CR0_NW (1 << 29)
#define CR0_PE (1 << 0)

#define SD_CS (0x1a << 8)
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

#define EXP_ROM_START_ADDRESS 0xc0000UL
#define BIOS32_CODE_SEGMENT 0xe000
#define BIOS32_START_ADDRESS ((0UL + BIOS32_CODE_SEGMENT) << 4)
#define BIOS32_SIZE (64UL * KB)
#define BIOS32_STAGE1_STACK_BASE (1024UL * 256)
#define BACK_FROM_PM_START_ADDRESS 0x0002
#define BIOS16_CODE_SEGMENT 0xf000
#define BIOS16_SIZE (64UL * KB)
#define BIOS16_STACK_BASE 0xeff0
#define AP16_STACK_BASE 0xfff0
#define AP32_STACK_BASE (1024UL * 272)

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

#define CALL_SELECT_NOP 1
#define CALL_SELECT_INIT 2
#define CALL_SELECT_LOAD_VGA 3
#define CALL_SELECT_LOAD_EXP_ROM 4
#define CALL_SELECT_SMP 5
#define CALL_SELECT_AP 6
#define CALL_SELECT_COPY_MEM 7

#define INT13_HANDLED 0
#define INT13_DEC_AND_NEXT 1
#define INT13_CALL_NEXT 2

#endif

