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
#include "defs.h"
#include "nox.h"


#include "common.c"

#define OFFSET_OF(type, member) ((uint16_t)&((type*)0)->member)
#define CLI() __asm { cli}
#define STI() __asm { sti}
#define HALT() __asm { hlt}
#define INT(x) __asm { int x}

void call32(void);
void unhandled_interrupt(void);
void unhandled_irq0(void);
void unhandled_irq1(void);
void unhandled_irq2(void);
void unhandled_irq3(void);
void unhandled_irq4(void);
void unhandled_irq5(void);
void unhandled_irq6(void);
void unhandled_irq7(void);
void unhandled_irq8(void);
void unhandled_irq9(void);
void unhandled_irq10(void);
void unhandled_irq11(void);
void unhandled_irq12(void);
void unhandled_irq13(void);
void unhandled_irq14(void);
void unhandled_irq15(void);
void pit_interrupt_handler();
void rtc_interrupt_handler();
void int15_handler();
void int1a_handler();


#define FUNC_OFFSET(function) (uint16_t)(function)


#define REG_HI(x) (((uint8_t FAR *)&x)[1])
#define REG_LOW(x) (((uint8_t FAR *)&x)[0])

typedef _Packed struct UserRegs {
    uint16_t gs;
    uint16_t fs;
    uint16_t es;
    uint16_t ds;

    uint16_t di;
    uint16_t si;
    uint16_t bp;
    uint16_t sp;
    uint16_t bx;
    uint16_t dx;
    uint16_t cx;
    uint16_t ax;

    uint16_t ip;
    uint16_t cs;
    uint16_t flags;
} UserRegs;


static uint16_t set_ds(uint16_t data_seg)
{
    uint16_t prev;

    _asm {
        mov prev, ds
        mov ds, data_seg
    }

    return prev;
}


static uint16_t get_cs()
{
    uint16_t code_seg;

     _asm {
        mov ax, cs
        mov code_seg, ax
    }

    return code_seg;
}


static uint16_t get_ds()
{
    uint16_t data_seg;

     _asm {
        mov ax, ds
        mov data_seg, ax
    }

    return data_seg;
}

static void restore_ds()
{
   _asm {
        mov ax, cs
        mov ds, ax
    }
}


static uint16_t read_word(uint16_t seg, uint16_t offset)
{
    uint16_t val;

    seg = set_ds(seg);
    val = *(uint16_t*)offset;
    set_ds(seg);

    return val;
}


static uint32_t read_dword(uint16_t seg, uint16_t offset)
{
    uint32_t val;

    seg = set_ds(seg);
    val = *(uint32_t*)offset;
    set_ds(seg);

    return val;
}


static void write_word(uint16_t seg, uint16_t offset, uint16_t val)
{
    seg = set_ds(seg);
    *(uint16_t*)offset = val;
    set_ds(seg);
}


static void write_dword(uint16_t seg, uint16_t offset, uint32_t val)
{
    seg = set_ds(seg);
    *(uint32_t*)offset = val;
    set_ds(seg);
}


static uint8_t read_byte(uint16_t seg, uint16_t offset)
{
    uint8_t val;

    seg = set_ds(seg);
    val = *(uint8_t*)offset;
    set_ds(seg);

    return val;
}


static void write_byte(uint16_t seg, uint16_t offset, uint8_t val)
{
    seg = set_ds(seg);
    *(uint8_t*)offset = val;
    set_ds(seg);
}


static void mem_set(uint16_t seg, uint16_t offset, uint8_t patern, uint16_t n)
{
    uint8_t* now = (uint8_t*)offset;
    uint8_t* end = now + n;

    seg = set_ds(seg);

    for (; now < end; now++) *now = patern;

    set_ds(seg);
}


static void mem_reset(uint16_t seg, uint16_t offset, uint16_t n)
{
    mem_set(seg, offset, 0, n);
}


static uint8_t bda_read_byte(uint16_t offset)
{
    return read_byte(BIOS_DATA_AREA_ADDRESS >> 4, offset);
}


static void bda_write_byte(uint16_t offset, uint8_t val)
{
    write_byte(BIOS_DATA_AREA_ADDRESS >> 4, offset, val);
}


static uint16_t bda_read_word(uint16_t offset)
{
    return read_word(BIOS_DATA_AREA_ADDRESS >> 4, offset);
}


static void bda_write_word(uint16_t offset, uint16_t val)
{
    write_word(BIOS_DATA_AREA_ADDRESS >> 4, offset, val);
}


static uint32_t bda_read_dword(uint16_t offset)
{
    return read_dword(BIOS_DATA_AREA_ADDRESS >> 4, offset);
}


static void bda_write_dword(uint16_t offset, uint32_t val)
{
    write_dword(BIOS_DATA_AREA_ADDRESS >> 4, offset, val);
}


static uint8_t ebda_read_byte(uint16_t offset)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    return read_byte(seg, offset);
}


static void ebda_write_byte(uint16_t offset, uint16_t val)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    write_byte(seg, offset, val);
}


static uint16_t ebda_read_word(uint16_t offset)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    return read_word(seg, offset);
}


static void platform_debug_print(char FAR * str)
{
    uint16_t port = ebda_read_word(OFFSET_OF(EBDA, private) +
                                   OFFSET_OF(EBDAPrivate, platform_io));

    uint32_t flags = get_eflags();
    CLI();

    outb(port + PLATFORM_IO_SELECT, PLATFORM_REG_WRITE_POS);
    outd(port + PLATFORM_IO_REGISTER, 0);

    do {
        outb(port + PLATFORM_IO_PUT_BYTE, *str);
    } while (*str++);

    outb(port + PLATFORM_IO_LOG, 0);

    put_eflags(flags);
}


static void freeze()
{
    for (;;) {
        CLI();
        HALT();
    }
}


static inline void init_bios_data_area()
{
    post(POST_CODE_BDA);

    mem_reset(0, BIOS_DATA_AREA_ADDRESS, BIOS_DATA_AREA_SIZE);
    mem_reset(BIOS_EXTENDED_DATA_AREA_ADDRESS >> 4, 0, BIOS_EXTENDED_DATA_AREA_KB * KB);

    bda_write_word(BDA_OFFSET_EBDA, BIOS_EXTENDED_DATA_AREA_ADDRESS >> 4);
    ebda_write_byte(EBDA_OFFSET_SIZE, BIOS_EXTENDED_DATA_AREA_KB);
}


typedef _Packed struct IntVector {
    uint16_t offset;
    uint16_t seg;
} IntVector;


static void set_int_vec(uint8_t index, uint16_t seg, uint16_t offset)
{
    IntVector* entry = NULL;
    uint16_t prev_seg;

    entry += index;
    prev_seg = set_ds(0);
    entry->seg = seg;
    entry->offset = offset;
    set_ds(prev_seg);
}


void on_unhandled_irq(uint16_t irq)
{
    char buf[100]; // todo: swich to private stack
    format_str(buf, "unhandled irq %u", 100, irq);
    platform_debug_print(buf);

    if (irq > 7) {
        outb(IO_PORT_PIC2, PIC_SPECIFIC_EOI_MASK | (irq % 8));
        irq = 2;
    }

    outb(IO_PORT_PIC1, PIC_SPECIFIC_EOI_MASK | irq);
    bda_write_byte(BDA_OFFSET_LAST_IRQ, 1 << irq);
}


static void init_int_vector()
{
    uint i;

    for (i = 0; i < 0x08; i++) {
        set_int_vec(i, get_cs(), FUNC_OFFSET(unhandled_interrupt));
    }

    set_int_vec(0x08, get_cs(), FUNC_OFFSET(unhandled_irq0));
    set_int_vec(0x09, get_cs(), FUNC_OFFSET(unhandled_irq1));
    set_int_vec(0x0a, get_cs(), FUNC_OFFSET(unhandled_irq2));
    set_int_vec(0x0b, get_cs(), FUNC_OFFSET(unhandled_irq3));
    set_int_vec(0x0c, get_cs(), FUNC_OFFSET(unhandled_irq4));
    set_int_vec(0x0d, get_cs(), FUNC_OFFSET(unhandled_irq5));
    set_int_vec(0x0e, get_cs(), FUNC_OFFSET(unhandled_irq6));
    set_int_vec(0x0f, get_cs(), FUNC_OFFSET(unhandled_irq7));

    for (i = 0x10; i < 0x70; i++) {
        set_int_vec(i, get_cs(), FUNC_OFFSET(unhandled_interrupt));
    }

    set_int_vec(0x70, get_cs(), FUNC_OFFSET(unhandled_irq8));
    set_int_vec(0x71, get_cs(), FUNC_OFFSET(unhandled_irq9));
    set_int_vec(0x72, get_cs(), FUNC_OFFSET(unhandled_irq10));
    set_int_vec(0x72, get_cs(), FUNC_OFFSET(unhandled_irq11));
    set_int_vec(0x74, get_cs(), FUNC_OFFSET(unhandled_irq12));
    set_int_vec(0x75, get_cs(), FUNC_OFFSET(unhandled_irq13));
    set_int_vec(0x76, get_cs(), FUNC_OFFSET(unhandled_irq14));
    set_int_vec(0x77, get_cs(), FUNC_OFFSET(unhandled_irq15));

    for (i = 0x78; i < 0x100; i++) {
        set_int_vec(i, get_cs(), FUNC_OFFSET(unhandled_interrupt));
    }
}


void on_pit_interrupt()
{
    uint32_t* counter;

    set_ds(BDA_SEG);

    *(uint8_t*)BDA_OFFSET_LAST_IRQ = ~0;

    counter = (uint32_t*)BDA_OFFSET_TICKS;

    if (++*counter == 0x18038aUL) { //24 hours
        *counter = 0;
        *(uint8_t*)BDA_OFFSET_TICKS_ROLLOVER = 1;
    }

    if ((*counter % 36) == 0) {
        restore_ds();
        platform_debug_print("tick");
    }

    INT(0x1c);

    outb(IO_PORT_PIC1, PIC_SPECIFIC_EOI_MASK | 0);
}


static void setup_pit_irq()
{
    set_int_vec(0x08, get_cs(), FUNC_OFFSET(pit_interrupt_handler));
    outb(IO_PORT_PIC1 + 1, (inb(IO_PORT_PIC1 + 1) & ~(1 << PIC1_TIMER_PIN)));
}


static void setup_rtc_irq()
{
    set_int_vec(0x70, get_cs(), FUNC_OFFSET(rtc_interrupt_handler));
    outb(IO_PORT_PIC2 + 1, (inb(IO_PORT_PIC2 + 1) & ~(1 << PIC2_RTC_PIN)));
    set_int_vec(0x15, get_cs(), FUNC_OFFSET(int15_handler));
    set_int_vec(0x1a, get_cs(), FUNC_OFFSET(int1a_handler));
}


static void rtc_write(uint index, uint8_t val)
{
    outb(IO_PORT_RTC_INDEX, index | ebda_read_byte(OFFSET_OF(EBDA, private) +
                                                   OFFSET_OF(EBDAPrivate, nmi_mask)));
    outb(IO_PORT_RTC_DATA, val);
}


static uint8_t rtc_read(uint index)
{
    outb(IO_PORT_RTC_INDEX, index | ebda_read_byte(OFFSET_OF(EBDA, private) +
                                                   OFFSET_OF(EBDAPrivate, nmi_mask)));
    return inb(IO_PORT_RTC_DATA);
}


static uint8_t start_wait(uint32_t micro, far_ptr_t usr_ptr)
{
    uint8_t regb;

    bda_write_byte(BDA_OFFSET_WAIT_FLAGS, bda_read_byte(BDA_OFFSET_WAIT_FLAGS) | BDA_WAIT_IN_USE);

    bda_write_dword(BDA_OFFSET_WAIT_COUNT_MICRO, micro);
    bda_write_dword(BDA_OFFSET_WAIT_USR_PTR, usr_ptr);

    rtc_write(0x0a, (rtc_read(0x0a) & ~RTC_REG_A_RATE_MASK) | RTC_REG_A_RATE_976U);
    regb = rtc_read(0x0b) | RTC_REG_B_ENABLE_PERIODIC_MASK;
    rtc_write(0x0b, regb);

    return regb;
}


void on_int15(UserRegs __far * context)
{
    switch (REG_HI(context->ax)) {
    case 0x83:
        switch (REG_LOW(context->ax)) {
        case 0: {
            uint32_t micro = ((uint32_t)context->cx << 16) | context->dx;

            if (!micro) {
                write_dword(context->es, context->bx,
                            read_dword(context->es, context->bx) | BDA_WAIT_ELAPSED);
            } else {
                if (bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & BDA_WAIT_IN_USE) {
                    context->ax = 0;
                    context->flags |= (1 << CPU_FLAGS_CF_BIT);
                    return;
                }

                REG_LOW(context->ax) = start_wait(((uint32_t)context->cx << 16) | context->dx,
                                                  ((uint32_t)context->es << 16) | context->bx);
            }
            context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
            break;
        }
        case 1:
            rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_PERIODIC_MASK);
            bda_write_byte(BDA_OFFSET_WAIT_FLAGS,
                           bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & ~BDA_WAIT_IN_USE);
            rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_PERIODIC_MASK);
            context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
            break;
        default:
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            REG_HI(context->ax) = 0x86;
        }
    case 0x86: {
        uint32_t micro = ((uint32_t)context->cx << 16) |context->dx;

        if (!micro) {
             bda_write_byte(BDA_OFFSET_WAIT_FLAGS,
                            bda_read_byte(BDA_OFFSET_WAIT_FLAGS) | BDA_WAIT_ELAPSED);
        } else {
            if (bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & BDA_WAIT_IN_USE) {
                context->ax = 0;
                context->flags |= (1 << CPU_FLAGS_CF_BIT);
                return;
            }

            bda_write_byte(BDA_OFFSET_WAIT_FLAGS,
                           bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & ~BDA_WAIT_ELAPSED);
            start_wait(micro, ((uint32_t)BDA_SEG << 16) | BDA_OFFSET_WAIT_FLAGS);

            for (;;) {
                if (bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & BDA_WAIT_ELAPSED) {
                    break;
                }

                STI();
                HALT();
                CLI();
            }
        }

        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        context->ax = 0;
        break;
    }
    default:
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
        REG_HI(context->ax) = 0x86;
    }
}


void on_int1a(UserRegs __far * context)
{
    switch (REG_HI(context->ax)) {
    case 0x02: // get_time
        if ((rtc_read(0x0a) & RTC_REG_A_UPDATE_IN_PROGRESS)) {
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            break;
        }

        REG_HI(context->cx) = rtc_read(RTC_HOURS);
        REG_LOW(context->cx) = rtc_read(RTC_MINUTES);
        REG_HI(context->dx) = rtc_read(RTC_SECONDS);
        REG_LOW(context->dx) = rtc_read(RTC_REGB) & 1;
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    case 0x06: { // set alarm
        uint8_t flags = ebda_read_byte(OFFSET_OF(EBDA, private) +
                                       OFFSET_OF(EBDAPrivate, bios_flags));

        if ((flags & BIOS_FLAGS_ALRM_ACTIVE_MASK)) {
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            break;
        }

        rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_ALARM_MASK);
        rtc_write(RTC_HOURS_ALARM, REG_HI(context->cx));
        rtc_write(RTC_MINUTES_ALARM, REG_LOW(context->cx));
        rtc_write(RTC_SECONDS_ALARM, REG_HI(context->dx));
        rtc_write(0x0b, rtc_read(0x0b) | RTC_REG_B_ENABLE_ALARM_MASK);
        ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c),
                        rtc_read(0x0c) & ~RTC_REG_C_ALARM_INTERRUPT_MASK); // AF barrier
        ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags),
                        flags | BIOS_FLAGS_ALRM_ACTIVE_MASK);
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case 0x07: { // cancle alarm
        uint8_t flags = ebda_read_byte(OFFSET_OF(EBDA, private) +
                                       OFFSET_OF(EBDAPrivate, bios_flags));
        ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags),
                        flags & ~BIOS_FLAGS_ALRM_ACTIVE_MASK);
        rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_ALARM_MASK);
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    default:
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
        REG_HI(context->ax) = 0x86;
    }
}


void on_rtc_interrupt()
{
    uint8_t regc;
    uint8_t flags;

    regc = ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c));
    ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c), 0);
    regc |= rtc_read(0x0c);

    if ((regc & RTC_REG_C_PERIODIC_INTERRUPT_MASK) &&
                                        (bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & BDA_WAIT_IN_USE)) {
        uint32_t counter = bda_read_dword(BDA_OFFSET_WAIT_COUNT_MICRO);
        if (counter < BIOS_MICRO_PER_TICK) { // ~1 mili in mode 6
            far_ptr_t ptr;
            uint32_t val;

            bda_write_dword(BDA_OFFSET_WAIT_COUNT_MICRO, 0);
            rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_PERIODIC_MASK);
            ptr = bda_read_dword(BDA_OFFSET_WAIT_USR_PTR);
            val = read_dword(ptr >> 16, ptr & 0xffff);
            write_dword(ptr >> 16, ptr & 0xffff, (val | BDA_WAIT_ELAPSED));
            bda_write_byte(BDA_OFFSET_WAIT_FLAGS,
                           bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & ~BDA_WAIT_IN_USE);
        } else {
            bda_write_dword(BDA_OFFSET_WAIT_COUNT_MICRO, counter - BIOS_MICRO_PER_TICK);
        }
    }

    flags = ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags));

    if ((flags & BIOS_FLAGS_ALRM_ACTIVE_MASK) && (regc & RTC_REG_C_ALARM_INTERRUPT_MASK)) {
        INT(0x4a);
    }

    outb(IO_PORT_PIC1, PIC_SPECIFIC_EOI_MASK | 2);
    outb(IO_PORT_PIC2, PIC_SPECIFIC_EOI_MASK | 0);
}


static uint8_t kbd_receive_data()
{
    while (!(inb(IO_PORT_KBD_STATUS) & KBDCTRL_STATUS_DATA_READY_MASK)) {
        //todo: add delay
        platform_debug_print(__FUNCTION__ ": no pending data");
    }

    return inb(IO_PORT_KBD_DATA);
}


static uint8_t kbd_receive_keyboard_data()
{
    for (;;) {
        uint8_t status;
        uint8_t data;

        status = inb(IO_PORT_KBD_STATUS);

        if (!(status & KBDCTRL_STATUS_DATA_READY_MASK)) {
            platform_debug_print(__FUNCTION__ ": no pending data");
            //todo: add delay
            continue;
        }

        data = inb(IO_PORT_KBD_DATA);

        if ((status & KBDCTRL_STATUS_MOUSE_DATA_READY_MASK)) {
            //todo: send data to mouse handler
            platform_debug_print(__FUNCTION__ ": droping mouse data");
            continue;
        }

        return data;
    }
}


static uint8_t kbd_receive_mouse_data()
{
    for (;;) {
        uint8_t status;
        uint8_t data;

        status = inb(IO_PORT_KBD_STATUS);

        if (!(status & KBDCTRL_STATUS_DATA_READY_MASK)) {
            platform_debug_print(__FUNCTION__ ": no pending data");
            //todo: add delay
            continue;
        }

        data = inb(IO_PORT_KBD_DATA);

        if (!(status & KBDCTRL_STATUS_MOUSE_DATA_READY_MASK)) {
            //todo: send data to keyboard handler
            platform_debug_print(__FUNCTION__ ": droping keyboard data");
            continue;
        }

        return data;
    }
}


static void kbd_send_data(uint8_t val)
{
    uint8_t status;

    while ((inb(IO_PORT_KBD_STATUS) & KBDCTRL_STATUS_WRITE_DISALLOWED_MASK)) {
        //todo: add delay
        platform_debug_print(__FUNCTION__ ": unable to write");
    }

    outb(IO_PORT_KBD_DATA, val);
}


static inline void init_keyboard()
{
    uint8_t command_byte;

    post(POST_CODE_KEYBOARD);

    outb(IO_PORT_KBD_COMMAND, KBDCTRL_CMD_SELF_TEST);

    if (kbd_receive_data() != KBDCTRL_SELF_TEST_REPLAY) {
        platform_debug_print(__FUNCTION__ ": keyboard self test failed, halting...");
        freeze();
    }

    outb(IO_PORT_KBD_COMMAND, KBDCTRL_CMD_ENABLE_KEYBOARD);
    outb(IO_PORT_KBD_COMMAND, KBDCTRL_CMD_KEYBOARD_INTERFACE_TEST);

    if (kbd_receive_data()) {
        platform_debug_print(__FUNCTION__ ": kbd interface failed, halting...");
        freeze();
    }

    kbd_send_data(KBD_CMD_RESET);

    if (kbd_receive_keyboard_data() != KBD_ACK ||
                                            kbd_receive_keyboard_data() != KBD_SELF_TEST_REPLAY) {
        platform_debug_print(__FUNCTION__ ": kbd reset failed, halting...");
        freeze();
    }

    outb(IO_PORT_KBD_COMMAND, KBDCTRL_CMD_READ_COMMAND_BYTE);
    command_byte = kbd_receive_data();
    outb(IO_PORT_KBD_COMMAND, KBDCTRL_CMD_WRITE_COMMAND_BYTE);
    outb(IO_PORT_KBD_DATA, command_byte | KBDCTRL_COMMAND_BYTE_IRQ1_MASK |
                           KBDCTRL_COMMAND_BYTE_TRANSLATE_MASK);

    kbd_send_data(KBD_CMD_ENABLE_SCANNING);
    if (kbd_receive_keyboard_data() != KBD_ACK) {
        platform_debug_print(__FUNCTION__ ": enable kbd failed, halting...");
        freeze();
    }
}


static inline void init_mouse()
{
    uint8_t command_byte;

    post(POST_CODE_MOUSE);

    outb(IO_PORT_KBD_COMMAND, KBDCTRL_CMD_ENABLE_MOUSE);
    outb(IO_PORT_KBD_COMMAND, KBDCTRL_CMD_MOUSE_INTERFACE_TEST);

    if (kbd_receive_data()) {
        platform_debug_print(__FUNCTION__ ": mouse interface failed, halting...");
        freeze();
    }

    outb(IO_PORT_KBD_COMMAND, KBDCTRL_CMD_WRITE_TO_MOUSE);
    kbd_send_data(MOUSE_CMD_RESET);

    if (kbd_receive_mouse_data() != KBD_ACK || kbd_receive_mouse_data() != KBD_SELF_TEST_REPLAY ||
                                                                        kbd_receive_mouse_data()) {
        platform_debug_print(__FUNCTION__ ": mouse reset failed, halting...");
        freeze();
    }

    outb(IO_PORT_KBD_COMMAND, KBDCTRL_CMD_DISABLE_MOUSE);

    outb(IO_PORT_KBD_COMMAND, KBDCTRL_CMD_READ_COMMAND_BYTE);
    command_byte = kbd_receive_data();
    outb(IO_PORT_KBD_COMMAND, KBDCTRL_CMD_WRITE_COMMAND_BYTE);
    outb(IO_PORT_KBD_DATA, command_byte | KBDCTRL_COMMAND_BYTE_IRQ12_MASK);

    bda_write_word(BDA_OFFSET_EQUIPMENT,
                   bda_read_word(BDA_OFFSET_EQUIPMENT) | (1 << BDA_EQUIPMENT_MOUSE_BIT));
}



void init()
{
    post(POST_CODE_INIT16);
    init_bios_data_area();

    call32();

    init_int_vector();

    platform_debug_print("log from 16bit");

    setup_pit_irq();
    setup_rtc_irq();

    STI();

    init_keyboard();
    init_mouse();

    post(POST_CODE_TMP);

    for (;;) {
         __asm {
            mov ah, 86h
            mov cx, 0x98
            mov dx, 0x9680
            int 15h
        }
        platform_debug_print("post wait");
    }

    restart();
}

