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
#include "error_codes.h"

#include "common.c"

#define OFFSET_OF(type, member) ((uint16_t)&((type*)0)->member)
#define CLI() __asm { cli}

#define STI() {                                                 \
    if (is_hard_int_context()) {                                \
        bios_error(BIOS_ERROR_STI_WHILE_IN_IRQ_CONTEXT);        \
    }                                                           \
                                                                \
    __asm { sti}                                                \
}

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
void keyboard_interrupt_handler();
void mouse_interrupt_handler();
void int15_handler();
void int16_handler();
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


static void freeze()
{
    for (;;) {
        CLI();
        HALT();
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


static uint32_t ebda_read_dword(uint16_t offset)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    return read_dword(seg, offset);
}


static void ebda_write_dword(uint16_t offset, uint32_t val)
{
    uint16_t seg = bda_read_word(BDA_OFFSET_EBDA);
    write_dword(seg, offset, val);
}


static void bios_error(uint16_t code)
{
    uint16_t error_code = PLATFORM_MK_ERR(PLATFORM_ERR_TYPE_ERROR, PLATFORM_ERR_SUBSYS_BIOS, code);
    uint16_t port = ebda_read_word(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, platform_io));
    outd(port + PLATFORM_IO_ERROR, error_code);
    freeze();
}


static void bios_warn(uint16_t code)
{
    uint16_t error_code = PLATFORM_MK_ERR(PLATFORM_ERR_TYPE_WARN, PLATFORM_ERR_SUBSYS_BIOS, code);
    uint16_t port = ebda_read_word(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, platform_io));
    outd(port + PLATFORM_IO_ERROR, error_code);
}


static void bios_info(uint16_t code)
{
    uint16_t error_code = PLATFORM_MK_ERR(PLATFORM_ERR_TYPE_INFO, PLATFORM_ERR_SUBSYS_BIOS, code);
    uint16_t port = ebda_read_word(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, platform_io));
    outd(port + PLATFORM_IO_ERROR, error_code);
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


static uint8_t is_hard_int_context()
{
    return ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags)) &
           BIOS_FLAGS_HARD_INT;
}


void set_irq_context()
{
    uint8_t flags = ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags));

    if ((flags & BIOS_FLAGS_HARD_INT)) {
        bios_error(BIOS_ERROR_DOUBLE_HARD_INT);
    }

    ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags),
                    flags | BIOS_FLAGS_HARD_INT);
}


void clear_irq_context()
{
    uint8_t flags = ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags));

    ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags),
                    flags & ~BIOS_FLAGS_HARD_INT);
}


static inline void init_bios_data_area()
{
    post(POST_CODE_BDA);

    mem_reset(0, BIOS_DATA_AREA_ADDRESS, BIOS_DATA_AREA_SIZE);
    mem_reset(BIOS_EBDA_ADDRESS >> 4, 0, BIOS_EBDA_DATA_KB * KB);

    bda_write_word(BDA_OFFSET_EBDA, BIOS_EBDA_ADDRESS >> 4);
    ebda_write_byte(EBDA_OFFSET_SIZE, BIOS_EBDA_SIZE_KB);
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
    char buf[100];
    format_str(buf, "unhandled irq %u", sizeof(buf), irq);
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


static void rtc_periodic_ref(uint8_t user)
{
    uint8_t refs;

    refs = ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_refs));

    if (!refs) {
        rtc_write(0x0a, (rtc_read(0x0a) & ~RTC_REG_A_RATE_MASK) | RTC_REG_A_RATE_976U);
        rtc_write(0x0b, rtc_read(0x0b) | RTC_REG_B_ENABLE_PERIODIC_MASK);
    }

    refs |= user;

    ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_refs), refs);
}


static void rtc_periodic_unref(uint8_t user)
{
    uint8_t refs;

    refs = ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_refs));

    refs &= ~user;

    ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_refs), refs);

    if (!refs) {
        rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_PERIODIC_MASK);
    }
}


static uint8_t start_wait(uint32_t micro, far_ptr_t usr_ptr)
{
    uint8_t regb;

    bda_write_byte(BDA_OFFSET_WAIT_FLAGS, bda_read_byte(BDA_OFFSET_WAIT_FLAGS) | BDA_WAIT_IN_USE);

    bda_write_dword(BDA_OFFSET_WAIT_COUNT_MICRO, micro);
    bda_write_dword(BDA_OFFSET_WAIT_USR_PTR, usr_ptr);

    rtc_periodic_ref(BIOS_PERIODIC_USER_WAIT_SERVICE);

    return regb;
}


void on_int15(UserRegs __far * context)
{
    switch (REG_HI(context->ax)) {
    case INT15_FUNC_KBD_INTERCEPT:
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
        break;
    case INT15_FUNC_EVENT_WAIT_CTRL:
        switch (REG_LOW(context->ax)) {
        case INT15_EVENT_WAIT_SET: {
            uint32_t micro;

            if (bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & BDA_WAIT_IN_USE) {
                context->ax = 0;
                context->flags |= (1 << CPU_FLAGS_CF_BIT);
                return;
            }

            micro = ((uint32_t)context->cx << 16) | context->dx;

            if (!micro) {
                bda_write_dword(BDA_OFFSET_WAIT_COUNT_MICRO, 0);
                write_dword(context->es, context->bx,
                            read_dword(context->es, context->bx) | BDA_WAIT_ELAPSED);
            } else {
                REG_LOW(context->ax) = start_wait(((uint32_t)context->cx << 16) | context->dx,
                                                  ((uint32_t)context->es << 16) | context->bx);
            }

            context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
            break;
        }
        case INT15_EVENT_WAIT_CANCEL:
            rtc_periodic_unref(BIOS_PERIODIC_USER_WAIT_SERVICE);
            bda_write_byte(BDA_OFFSET_WAIT_FLAGS,
                           bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & ~BDA_WAIT_IN_USE);
            context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
            break;
        default:
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            REG_HI(context->ax) = 0x86;
        }
    case INT15_FUNC_WAIT: {
        uint32_t micro;

        if (bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & BDA_WAIT_IN_USE) {
            context->ax = 0;
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            return;
        }

        if ((micro = ((uint32_t)context->cx << 16) | context->dx)) {

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

        bda_write_byte(BDA_OFFSET_WAIT_FLAGS,
                       bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & ~BDA_WAIT_ELAPSED);

        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        context->ax = 0;
        break;
    }
    case INT15_FUNC_DEVICE_BUSY:
    case INT15_FUNC_DEVICE_POST:
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    default:
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
        REG_HI(context->ax) = 0x86;
    }
}


void on_int1a(UserRegs __far * context)
{
    switch (REG_HI(context->ax)) {
    case INT1A_FUNC_GET_TIME:
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
    case INT1A_FUNC_SET_ALARM: {
        uint8_t flags = ebda_read_byte(OFFSET_OF(EBDA, private) +
                                       OFFSET_OF(EBDAPrivate, bios_flags));
        uint8_t reg_c;

        if ((flags & BIOS_FLAGS_ALRM_ACTIVE_MASK)) {
            context->flags |= (1 << CPU_FLAGS_CF_BIT);
            break;
        }

        rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_ALARM_MASK);
        rtc_write(RTC_HOURS_ALARM, REG_HI(context->cx));
        rtc_write(RTC_MINUTES_ALARM, REG_LOW(context->cx));
        rtc_write(RTC_SECONDS_ALARM, REG_HI(context->dx));
        rtc_write(0x0b, rtc_read(0x0b) | RTC_REG_B_ENABLE_ALARM_MASK);

        // AF barrier
        reg_c = bda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c));
        reg_c = (reg_c | rtc_read(0x0c)) & ~RTC_REG_C_ALARM_INTERRUPT_MASK;
        ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c), reg_c);

        ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags),
                        flags | BIOS_FLAGS_ALRM_ACTIVE_MASK);
        context->flags &= ~(1 << CPU_FLAGS_CF_BIT);
        break;
    }
    case INT1A_FUNC_CANCEL_ALARM: {
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

    if ((regc & RTC_REG_C_PERIODIC_INTERRUPT_MASK)) {
        uint32_t counter;

        if ((bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & BDA_WAIT_IN_USE)) {
            counter = bda_read_dword(BDA_OFFSET_WAIT_COUNT_MICRO);

            if (counter < BIOS_PERIODIC_MICRO) { // ~1 mili in mode 6
                far_ptr_t ptr;
                uint32_t val;

                bda_write_dword(BDA_OFFSET_WAIT_COUNT_MICRO, 0);
                rtc_periodic_unref(BIOS_PERIODIC_USER_WAIT_SERVICE);
                ptr = bda_read_dword(BDA_OFFSET_WAIT_USR_PTR);
                val = read_dword(ptr >> 16, ptr & 0xffff);
                write_dword(ptr >> 16, ptr & 0xffff, (val | BDA_WAIT_ELAPSED));
                bda_write_byte(BDA_OFFSET_WAIT_FLAGS,
                               bda_read_byte(BDA_OFFSET_WAIT_FLAGS) & ~BDA_WAIT_IN_USE);
            } else {
                bda_write_dword(BDA_OFFSET_WAIT_COUNT_MICRO, counter - BIOS_PERIODIC_MICRO);
            }
        }

        counter = ebda_read_dword(OFFSET_OF(EBDA, private) +
                                  OFFSET_OF(EBDAPrivate, rtc_priodoc_ticks));

        ebda_write_dword(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_ticks),
                         counter + 1);

        if (!ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_refs))) {
            // unexpected
            rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_PERIODIC_MASK);
        }
    }

    flags = ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags));

    if ((flags & BIOS_FLAGS_ALRM_ACTIVE_MASK)) {
        if ((regc & RTC_REG_C_ALARM_INTERRUPT_MASK)) {
            INT(0x4a);
        } else {
            // unexpected
            rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_ALARM_MASK);
        }
    }

    if ((regc & RTC_REG_C_UPDATE_INTERRUPT_MASK)) {
        // unexpected.
        rtc_write(0x0b, rtc_read(0x0b) & ~RTC_REG_B_ENABLE_UPDATE_MASK);
    }

    outb(IO_PORT_PIC1, PIC_SPECIFIC_EOI_MASK | 2);
    outb(IO_PORT_PIC2, PIC_SPECIFIC_EOI_MASK | 0);
}


static void delay(uint32_t milisec)
{
    uint64_t micro;

    if (!milisec) {
        return;
    }

    micro = (uint64_t)milisec * 1000 + BIOS_PERIODIC_MICRO;

    ebda_write_dword(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_ticks), 0);
    rtc_periodic_ref(BIOS_PERIODIC_USER_INTERNAL_DELAY);

    for (;;) {
        uint64_t t;

        STI();
        HALT();
        CLI();

        t = ebda_read_dword(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_priodoc_ticks));
        t *= BIOS_PERIODIC_MICRO;

        if (t >= micro) {
            break;
        }

    }

    rtc_periodic_unref(BIOS_PERIODIC_USER_INTERNAL_DELAY);
}


static void hard_delay(uint32_t milisec)
{
    milisec++;

    while (milisec) {
        uint8_t reg_c = rtc_read(0x0c);

        if ((reg_c & RTC_REG_C_PERIODIC_INTERRUPT_MASK)) {
            milisec--;
        }

        reg_c |= ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c));
        ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, rtc_reg_c), reg_c);
    }
}


static void beep()
{
    uint16_t countdown = PIT_FREQUENCY / BIOS_BEEP_Hz;
    uint8_t misc;

    // set timer 2: mode 3, 16bit, binary
    outb(IO_PORT_TIMER_CONTROL, (2 << PIT_SELECTOR_SHIFT) |
                                (3 << PIT_RW_SHIFT) |
                                (3 << PIT_MODE_SHIFT));

    outb(IO_PORT_TIMER_2, countdown);
    outb(IO_PORT_TIMER_2, countdown >> 8);

    misc = inb(IO_PORT_MISC);
    outb(IO_PORT_MISC, misc | 0x3);
    hard_delay(BIOS_BEEP_DURATION_MS);
    outb(IO_PORT_MISC, (misc & ~0x3));
}


typedef struct ScanTrans {
    // high byte scan code, low byte ascii
    uint16_t no_modifiers;
    uint16_t shift;
    uint16_t control;
    uint16_t alt;
} ScanTrans;


static const ScanTrans scan_trans[KBD_MAX_KEY_SCAN + 1] = { // from "The Undocumented PC"
    {0x0000, 0x0000, 0x0000, 0x0000},
    {0x011b, 0x011b, 0x011b, /**/0x0100}, /* ESCAPE */
    {0x0231, 0x0221, 0x0000, 0x7800}, /* 1 */
    {0x0332, 0x0340, 0x0300, 0x7900}, /* 2 */
    {0x0433, 0x0423, 0x0000, 0x7a00}, /* 3 */
    {0x0534, 0x0524, 0x0000, 0x7b00}, /* 4 */
    {0x0635, 0x0625, 0x0000, 0x7c00}, /* 5 */
    {0x0736, 0x075e, 0x071e, 0x7d00}, /* 6 */
    {0x0837, 0x0826, 0x0000, 0x7e00}, /* 7 */
    {0x0938, 0x092a, 0x0000, 0x7f00}, /* 8 */
    {0x0a39, 0x0a28, 0x0000, 0x8000}, /* 9 */
    {0x0b30, 0x0b29, 0x0000, 0x8100}, /* 0 */
    {0x0c30, 0x0c5f, 0x0c1f, 0x8200}, /* - */
    {0x0d3d, 0x0d2b, 0x0000, 0x8300}, /* = */
    {0x0e08, 0x0e08, 0x0e7f, /**/0x0e00}, /* BACKSPACE */
    {0x0f09, 0x0f00, /**/0x9400, /**/0xa500}, /* TAB */
    {0x1071, 0x1051, 0x1011, 0x1000}, /* q */
    {0x1177, 0x1157, 0x1017, 0x1100}, /* w */
    {0x1265, 0x1245, 0x1205, 0x1200}, /* e */
    {0x1372, 0x1352, 0x1312, 0x1300}, /* r */
    {0x1474, 0x1454, 0x1414, 0x1400}, /* t */
    {0x1579, 0x1559, 0x1519, 0x1500}, /* y */
    {0x1675, 0x1655, 0x1615, 0x1600}, /* u */
    {0x1769, 0x1749, 0x1709, 0x1700}, /* i */
    {0x186f, 0x184f, 0x180f, 0x1800}, /* o */
    {0x1970, 0x1950, 0x1910, 0x1900}, /* p */
    {0x1a5b, 0x1a7b, 0x1a1b, /**/0x1a00}, /* [ */
    {0x1b5d, 0x1b7d, 0x1b1d, /**/0x1b00}, /* ] */
    {0x1c0d, 0x1c0d, 0x1c0a, /**/0x1c00}, /* ENTER */
    {0x0000, 0x0000, 0x0000, 0x0000}, /* LEFT_CONTROL */
    {0x1e61, 0x1e41, 0x1e01, 0x1e00}, /* a */
    {0x1f73, 0x1f53, 0x1f13, 0x1f00}, /* s */
    {0x2064, 0x2044, 0x2004, 0x2000}, /* d */
    {0x2166, 0x2146, 0x2106, 0x2100}, /* f */
    {0x2267, 0x2247, 0x2207, 0x2200}, /* g */
    {0x2368, 0x2348, 0x2308, 0x2300}, /* h */
    {0x246a, 0x244a, 0x240a, 0x2400}, /* j */
    {0x256b, 0x254b, 0x250b, 0x2500}, /* k */
    {0x266c, 0x264c, 0x260c, 0x2600}, /* l */
    {0x273b, 0x273a, 0x0000, /**/0x2700}, /* ; */
    {0x2827, 0x2822, 0x0000, /**/0x2800}, /* ' */
    {0x2960, 0x297e, 0x0000, 0x2900}, /* ` */
    {0x0000, 0x0000, 0x0000, 0x0000}, /* LEFT_SHIFT */
    {0x2b5c, 0x2b7c, 0x2b1c, /**/0x2b00}, /* \ */
    {0x2c7a, 0x2c5a, 0x2c1a, 0x2c00}, /* z */
    {0x2d78, 0x2d58, 0x2d18, 0x2d00}, /* x */
    {0x2e63, 0x2e43, 0x2e03, 0x2e00}, /* c */
    {0x2f76, 0x2f56, 0x2f16, 0x2f00}, /* v */
    {0x3062, 0x3042, 0x3002, 0x3000}, /* b */
    {0x316e, 0x314e, 0x310e, 0x3100}, /* n */
    {0x326d, 0x324d, 0x320d, 0x3200}, /* m */
    {0x332c, 0x333c, 0x0000, /**/0x3300}, /* , */
    {0x342e, 0x343e, 0x0000, /**/0x3400}, /* . */
    {0x352f, 0x353f, 0x0000, /**/0x3500}, /* / */
    {0x0000, 0x0000, 0x0000, 0x0000}, /* RIGHT_SHIFT */
    {0x372a, 0x0000, 0x9600, /**/0x3700}, /* PAD_* */
    {0x0000, 0x0000, 0x0000, 0x0000}, /* LEFT_ALT */
    {0x3920, 0x3920, 0x3920, 0x3920}, /* SPACE */
    {0x0000, 0x0000, 0x0000, 0x0000}, /* CAPSLOCK */
    {0x3b00, 0x5400, 0x5e00, 0x6800}, /* F1 */
    {0x3c00, 0x5500, 0x5f00, 0x6900}, /* F2 */
    {0x3d00, 0x5600, 0x6000, 0x6a00}, /* F3 */
    {0x3e00, 0x5700, 0x6100, 0x6b00}, /* F4 */
    {0x3f00, 0x5800, 0x6200, 0x6c00}, /* F5 */
    {0x4000, 0x5900, 0x6300, 0x6d00}, /* F6 */
    {0x4100, 0x5a00, 0x6400, 0x6e00}, /* F7 */
    {0x4200, 0x5b00, 0x6500, 0x6f00}, /* F8 */
    {0x4300, 0x5c00, 0x6600, 0x7000}, /* F9 */
    {0x4400, 0x5d00, 0x6700, 0x7100}, /* F10 */
    {0x0000, 0x0000, 0x0000, 0x0000}, /* NUMLOCK */
    {0x0000, 0x0000, 0x0000, 0x0000}, /* SCROLLLOCK */
    {0x4700, 0x4737, 0x7700, 0x0007 /* ### */}, /* PAD_HOME */
    {0x4800, 0x4838, /**/0x8d00, 0x0008 /* ### */}, /* PAD_UP */
    {0x4900, 0x4939, 0x8400, 0x0009 /* ### */}, /* PAD_PGUP */
    {0x4a2d, 0x4a2d, /**/0x8e00, /**/0x4a00}, /* PAD_- */
    {0x4b00, 0x4b34, 0x7300, 0x0004 /* ### */}, /* PAD_LEFT */
    {0x4c00, 0x4c35, /**/0x8f00, 0x0005 /* ### */}, /* PAD_CENTER */
    {0x4d00, 0x4d36, 0x7400, 0x0006 /* ### */}, /* PAD_RIGHT */
    {0x4e2b, 0x4e2b, /**/0x9000, /**/0x4e00}, /* PAD_+ */
    {0x4f00, 0x4f31, 0x7500, 0x0001 /* ### */}, /* PAD_END */
    {0x5000, 0x5032, /**/0x9100, 0x0002 /* ### */}, /* PAD_DOWN */
    {0x5100, 0x5133, 0x7600, 0x0003 /* ### */}, /* PAD_PGDOWN */
    {0x5200, 0x5230, /**/0x9200, 0x0000 /* ### */}, /* PAD_INSERT */
    {0x5300, 0x532e, /**/0x9300, 0x0000}, /* PAD_DEL */
    {0x0000, 0x0000, 0x0000, 0x0000},
    {0x0000, 0x0000, 0x0000, 0x0000},
    {0x0000, 0x0000, 0x0000, 0x0000},
    /*0x57*/{0x8500, /**/0x8700, /**/0x8900, /**/0x8b00}, /* F11 */
    /*0x58*/{0x8600, /**/0x8800, /**/0x8a00, /**/0x8c00}, /* F12 */
};


static const ScanTrans pad_enter_trans[1] = {
    /*1c*/ {0xe00d, /**/0xe00d, /**/0xe00a, /**/0xa600}, /* num enter   */
};


static const ScanTrans gray_div_trans[1] = {
    /*35*/ {0xe02f, /**/0xe02f, /**/0x9500, /**/0xa400}, /* gray / */
};


static const ScanTrans ext_scan_trans[] = {
    /*47*/ {0x47e0, 0x47e0, 0x77e0, /**/0x9700}, /* gray home */
    /*48*/ {0x48e0, 0x48e0, /**/0x8de0, /**/0x9800}, /* gray up */
    /*49*/ {0x49e0, 0x49e0, 0x84e0, /**/0x9900}, /* gray pg-up */
    /*4a*/ {0x0000, 0x0000, 0x0000, /**/0x0000},
    /*4b*/ {0x4be0, 0x4be0, 0x73e0, /**/0x9b00}, /* gray left */
    /*4c*/ {0x0000, 0x0000, 0x0000, 0x0000},
    /*4d*/ {0x4de0, 0x4de0, 0x74e0, /**/0x9d00}, /* gray right */
    /*4e*/ {0x0000, 0x0000, 0x0000, 0x0000},
    /*4f*/ {0x4fe0, 0x4fe0, 0x75e0, /**/0x9f00}, /* gray end */
    /*50*/ {0x50e0, 0x50e0, /**/0x91e0, /**/0x9800}, /* gray down */
    /*51*/ {0x51e0, 0x51e0, 0x76e0, /**/0xa100}, /* gray pg-down */
    /*52*/ {0x52e0, 0x52e0, /**/0x92e0, /**/0xa200}, /* gray insert */
    /*53*/ {0x53e0, 0x53e0, /**/0x93e0, /**/0xa3e0}, /* gray del */
};


static const uint8_t ext_trans_size = sizeof(ext_scan_trans) / sizeof(ext_scan_trans[0]);


static uint8_t kbd_is_pad_num(uint8_t scan)
{
    return scan >= 0x47 && scan <= 0x52 && scan != 0x4a && scan != 0x4e;
}


static uint8_t kbd_is_alpha(uint8_t scan)
{
    return (scan >= 0x10 && scan <= 0x19) ||
           (scan >= 0x1e && scan <= 0x26) ||
           (scan >= 0x2c && scan <= 0x32);
}


static const uint16_t ext_codes[] = {
    0x0100,
    0x0e00,
    0x1a00,
    0x1b00,
    0x1c00,
    0x2700,
    0x2800,
    0x2b00,
    0x3300,
    0x3400,
    0x3500,
    0x4a00,
    0x4e00,
    0x8700,
    0x8800,
    0x8900,
    0x8a00,
    0x8b00,
    0x8c00,
    0x8d00,
    0x8e00,
    0x8f00,
    0x9000,
    0x9100,
    0x9200,
    0x9300,
    0x9400,
    0x9500,
    0x9700,
    0x9800,
    0x9900,
    0x9b00,
    0x9d00,
    0x9f00,
    0xa000,
    0xa100,
    0xa200,
    0xa300,
    0xa400,
    0xa500,
    0xa600,
    0x0000,
};


static uint16_t kbd_ext_to_nonext_key(uint16_t key)
{
    const uint16_t* ext_code = ext_codes;

    if ((key & 0xff) == 0xe0) {
        if ((key >> 8) <= KBD_MAX_KEY_SCAN) {
            key = scan_trans[key >> 8].no_modifiers;
        } else {
            key &= 0xff00;
        }
    } else if ((key >> 8) == 0xe0) {
        switch (key & 0xff) {
        case 0x0d:
            return 0x1c0d;
        case 0x0a:
            return 0x1c0a;
        case 0x2f:
            return 0x352f;
        default:
            return 0;
        }
    }

    while (*ext_code) {
        if (*ext_code++ == key) {
            return 0;
        }
    }

    return key;
}


static void kbd_push_key(uint16_t key_val)
{
    uint16_t buf_start = bda_read_word(BDA_OFFSET_KBD_BUF_START);
    uint16_t buf_size = bda_read_word(BDA_OFFSET_KBD_BUF_END) - buf_start;
    uint16_t head = bda_read_word(BDA_OFFSET_KBD_HEAD) - buf_start;
    uint16_t tail = bda_read_word(BDA_OFFSET_KBD_TAIL) - buf_start;
    uint16_t next = (tail + 2) % buf_size;

    if (head > 30 || tail > 30 || (tail & 1) || (head & 1)) {
        platform_debug_print(__FUNCTION__ ": bad pointers");
        restart();
    }

    if (next == head){
        beep();
        return;
    }

    bda_write_word(buf_start + tail, key_val);
    bda_write_word(BDA_OFFSET_KBD_TAIL, buf_start + next);

    if (ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags) &
                                                    BIOS_FLAGS_KBD_WAIT)) {
        __asm {
            mov ah, INT15_FUNC_DEVICE_POST
            mov al, 0x02 ; device type keyboard
            clc
            int 0x15
        }
    }
}


static uint16_t kbd_pop_key()
{
    uint16_t buf_start = bda_read_word(BDA_OFFSET_KBD_BUF_START);
    uint16_t buf_size = bda_read_word(BDA_OFFSET_KBD_BUF_END) - buf_start;
    uint16_t head = bda_read_word(BDA_OFFSET_KBD_HEAD) - buf_start;
    uint16_t tail = bda_read_word(BDA_OFFSET_KBD_TAIL) - buf_start;
    uint16_t next = (head + 2) % buf_size;
    uint16_t key;

    if (head == tail) {
        return 0;
    }

    if (head > 30 || tail > 30 || (tail & 1) || (head & 1)) {
        platform_debug_print(__FUNCTION__ ": bad pointers");
        restart();
    }

    key = bda_read_word(buf_start + head);

    bda_write_word(BDA_OFFSET_KBD_HEAD, buf_start + next);

    return key;
}


static uint16_t kbd_pop_nonext_key()
{
    for (;;) {
        uint16_t key = kbd_pop_key();

        if (key && !(key = kbd_ext_to_nonext_key(key))) {
            continue;
        }

        return key;
    }
}


static uint16_t kbd_peek_key()
{
    uint16_t buf_start = bda_read_word(BDA_OFFSET_KBD_BUF_START);
    uint16_t buf_size = bda_read_word(BDA_OFFSET_KBD_BUF_END) - buf_start;
    uint16_t head = bda_read_word(BDA_OFFSET_KBD_HEAD) - buf_start;
    uint16_t tail = bda_read_word(BDA_OFFSET_KBD_TAIL) - buf_start;
    uint16_t next = (head + 2) % buf_size;
    uint16_t key;

    if (head == tail) {
        return 0;
    }

    if (head > 30 || tail > 30 || (tail & 1) || (head & 1)) {
        platform_debug_print(__FUNCTION__ ": bad pointers");
        restart();
    }

    key = bda_read_word(buf_start + head);

    return key;
}


static uint16_t kbd_peek_nonext_key()
{
    for (;;) {
        uint16_t key = kbd_peek_key();

        if (key && !(key = kbd_ext_to_nonext_key(key))) {
            kbd_pop_key();
            continue;
        }

        return key;
    }
}


static void kbd_update_right_alt_ctrl(uint8_t is_break, uint16_t mask)
{
    uint16_t flags_1 = bda_read_word(BDA_OFFSET_KBD_FLAGS_1);
    uint16_t flags_2 = bda_read_word(BDA_OFFSET_KBD_FLAGS_2);

    if (is_break) {
        flags_2 &= ~mask;                 // RIGHT_MASK

        if (!(flags_1 & (mask << 6))) {   // LEFT_MASK
            flags_1 &= ~mask;             // SHARED_MASK
        }
    } else {
        flags_1 |= mask;
        flags_2 |= mask;
    }

    bda_write_word(BDA_OFFSET_KBD_FLAGS_1, flags_1);
    bda_write_word(BDA_OFFSET_KBD_FLAGS_2, flags_2);
}


static void kbd_puse_alt_pad()
{
    uint16_t flags_1 = bda_read_word(BDA_OFFSET_KBD_FLAGS_1);
    uint8_t ascii;

    if ((flags_1 & BDA_KBD_FLAGS_1_ALT)) {
        return;
    }

    ascii = bda_read_byte(BDA_OFFSET_KBD_ALT_PAD_AREA);

    if (ascii) {
        bda_write_byte(BDA_OFFSET_KBD_ALT_PAD_AREA, 0);
        kbd_push_key(ascii);
    }
}


static void kbd_warn_ext_scan(uint8_t scan)
{
    char str[30];

    format_str(str, "unhandled ext scan 0x%x", sizeof(str), scan);
    platform_debug_print(str);
}


static void kbd_warn_scan(uint8_t scan)
{
    char str[30];

    format_str(str, "unhandled scan 0x%x", sizeof(str), scan);
    platform_debug_print(str);
}


static void kbd_process_ext(uint8_t scan)
{
    uint8_t is_break = (scan & KBD_BREAK_MASK);
    const ScanTrans* trans;
    uint16_t flags_1;
    uint16_t key_val;

    scan &= ~KBD_BREAK_MASK;

    switch (scan) {
    case KBD_EXT_SCAN_R_ALT:
        kbd_update_right_alt_ctrl(is_break, BDA_KBD_FLAGS_1_ALT);
        kbd_puse_alt_pad();
        return;
    case KBD_EXT_SCAN_R_CTRL:
        kbd_update_right_alt_ctrl(is_break, BDA_KBD_FLAGS_1_CTRL);
        return;
    case KBD_EXT_SCAN_PRINT:
    case KBD_EXT_LEFT_META:
    case KBD_EXT_RIGHT_META:
    case KBD_EXT_RIGHT_MENU:
        kbd_warn_ext_scan(scan | is_break);
        return;
    case 0x1c:
        trans = pad_enter_trans;
        break;
    case 0x35:
        trans = gray_div_trans;
        break;
    default:
        if (scan < KBD_EXT_TRANS_START || scan >= KBD_EXT_TRANS_START + ext_trans_size) {
            kbd_warn_ext_scan(scan);
            return;
        }

        trans = &ext_scan_trans[scan - KBD_EXT_TRANS_START];
    }

    flags_1 = bda_read_word(BDA_OFFSET_KBD_FLAGS_1);

    if (is_break) {
        if (scan == KBD_EXT_SCAN_INSERT) {
            bda_write_word(BDA_OFFSET_KBD_FLAGS_1, flags_1 & ~BDA_KBD_FLAGS_1_INSERT_DOWN);
        }
        return;
    }

    if ((flags_1 & BDA_KBD_FLAGS_1_ALT)) {
        key_val = trans->alt;
    } else if ((flags_1 & BDA_KBD_FLAGS_1_CTRL)) {
        key_val = trans->control;
    } else if (flags_1 & (BDA_KBD_FLAGS_1_L_SHIFT | BDA_KBD_FLAGS_1_R_SHIFT)) {
        key_val = trans->shift;
    }else {
        if (scan == KBD_EXT_SCAN_INSERT) {
            flags_1 ^= BDA_KBD_FLAGS_1_INSERT;
            flags_1 |= BDA_KBD_FLAGS_1_INSERT_DOWN;
            bda_write_word(BDA_OFFSET_KBD_FLAGS_1, flags_1);
        }
        key_val = trans->no_modifiers;
    }

    if (!key_val) {
        return;
    }

    kbd_push_key(key_val);

}


static uint8_t kbd_process_compound(uint8_t scan)
{
    uint16_t flags_2 = bda_read_word(BDA_OFFSET_KBD_FLAGS_2);

    if (flags_2 & BDA_KBD_FLAGS_2_E0) {
        kbd_process_ext(scan);
        flags_2 = bda_read_word(BDA_OFFSET_KBD_FLAGS_2);
        bda_write_word(BDA_OFFSET_KBD_FLAGS_2, flags_2 & ~BDA_KBD_FLAGS_2_E0);
        return TRUE;
    }

    if (flags_2 & BDA_KBD_FLAGS_2_E1) {
        // for now
        platform_debug_print(__FUNCTION__ ": not unhandled 0xe1");
        bda_write_word(BDA_OFFSET_KBD_FLAGS_2, flags_2 & ~BDA_KBD_FLAGS_2_E1);
        return TRUE;
    }

    if (scan == 0xe0) {
        bda_write_word(BDA_OFFSET_KBD_FLAGS_2, flags_2 | BDA_KBD_FLAGS_2_E0);
        return TRUE;
    }

    if (scan == 0xe1) {
        bda_write_word(BDA_OFFSET_KBD_FLAGS_2, flags_2 | BDA_KBD_FLAGS_2_E1);
        return TRUE;
    }

    return 0;
}



static void kbd_send_data_sync(uint8_t val)
{
    uint8_t status;

    while ((inb(IO_PORT_KBD_STATUS) & KBDCTRL_STATUS_WRITE_DISALLOWED_MASK)) {
        // unexpected
        bios_warn(BIOS_WARN_KBD_WRITE_BLOCKED);
    }

    outb(IO_PORT_KBD_DATA, val);
}


static uint8_t kbd_handle_leds(uint8_t scan)
{
    uint16_t flags_2 = bda_read_word(BDA_OFFSET_KBD_FLAGS_2);
    uint8_t bios_flags;

    if (!(flags_2 & BDA_KBD_FLAGS_2_LEDS_IN_PROGRESS)) {
        return FALSE;
    }

    if (scan != KBD_ACK) {
        platform_debug_print("todo: reset the keyboard");
        freeze();
        return TRUE;
    }

    bios_flags = ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags));

    if ((bios_flags & BIOS_FLAGS_KBD_LEDS_DATA)) {
        uint16_t leds;

        bios_flags &= ~BIOS_FLAGS_KBD_LEDS_DATA;
        ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags), bios_flags);

        leds = (bda_read_word(BDA_OFFSET_KBD_FLAGS_2) >> BDA_KBD_FLAGS_2_LEDS_SHIFT) & 0x7;
        kbd_send_data_sync(leds);

        return TRUE;
    }

    bda_write_word(BDA_OFFSET_KBD_FLAGS_2, flags_2 & ~BDA_KBD_FLAGS_2_LEDS_IN_PROGRESS);
    return TRUE;
}


static void kbd_update_leds(uint16_t flags_1)
{
    uint16_t leds = (flags_1 >> BDA_KBD_FLAGS_1_LEDS_SHIFT) & 0x7;
    uint16_t flags_2 = bda_read_word(BDA_OFFSET_KBD_FLAGS_2) & ~BDA_KBD_FLAGS_2_LEDS_MASK;
    uint8_t bios_flags;

    flags_2 |= (leds << BDA_KBD_FLAGS_2_LEDS_SHIFT) | BDA_KBD_FLAGS_2_LEDS_IN_PROGRESS;

    bda_write_word(BDA_OFFSET_KBD_FLAGS_2, flags_2);

    bios_flags = ebda_read_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags));
    bios_flags |= BIOS_FLAGS_KBD_LEDS_DATA;
    ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags), bios_flags);

    kbd_send_data_sync(KBD_CMD_LED);
}


static void kbd_update_lock(uint8_t is_break, uint16_t state_mask, uint16_t down_mask)
{
    uint16_t flags_1 = bda_read_word(BDA_OFFSET_KBD_FLAGS_1);

    if (!is_break) {
        flags_1 ^= state_mask;
        flags_1 |= down_mask;
        bda_write_word(BDA_OFFSET_KBD_FLAGS_1, flags_1);
        kbd_update_leds(flags_1);
    } else {
        flags_1 &= ~down_mask;
        bda_write_word(BDA_OFFSET_KBD_FLAGS_1, flags_1);
    }
}


static void kbd_update_left_alt_ctrl(uint8_t is_break, uint16_t mask)
{
    uint16_t flags_1 = bda_read_word(BDA_OFFSET_KBD_FLAGS_1);

    if (is_break) {
        uint16_t flags_2 = bda_read_word(BDA_OFFSET_KBD_FLAGS_2);
        flags_1 &= ~(mask << 6); // LEFT_MASK

        if (!(flags_2 & mask)) { // RIGHT_MASK
            flags_1 &= ~mask;    // SHARED_MASK
        }
    } else {
        flags_1 |= mask | (mask << 6);
    }

    bda_write_word(BDA_OFFSET_KBD_FLAGS_1, flags_1);
}


static uint8_t kbd_process_modifiers(uint8_t scan)
{
    uint8_t is_break = (scan & KBD_BREAK_MASK);

    scan &= ~KBD_BREAK_MASK;

    switch (scan) {
    case KBD_SCAN_L_CTRL:
        kbd_update_left_alt_ctrl(is_break, BDA_KBD_FLAGS_1_CTRL);
        return TRUE;
    case KBD_SCAN_L_SHIFT: {
        uint16_t flags_1 = bda_read_word(BDA_OFFSET_KBD_FLAGS_1);
        flags_1 = is_break ? (flags_1 & ~BDA_KBD_FLAGS_1_L_SHIFT) :
                             (flags_1 | BDA_KBD_FLAGS_1_L_SHIFT);
        bda_write_word(BDA_OFFSET_KBD_FLAGS_1, flags_1);
        return TRUE;
    }
    case KBD_SCAN_R_SHIFT: {
        uint16_t flags_1 = bda_read_word(BDA_OFFSET_KBD_FLAGS_1);
        flags_1 = is_break ? (flags_1 & ~BDA_KBD_FLAGS_1_R_SHIFT) :
                             (flags_1 | BDA_KBD_FLAGS_1_R_SHIFT);
        bda_write_word(BDA_OFFSET_KBD_FLAGS_1, flags_1);
        return TRUE;
    }
    case KBD_SCAN_L_ALT:
        kbd_update_left_alt_ctrl(is_break, BDA_KBD_FLAGS_1_ALT);
        kbd_puse_alt_pad();
        return TRUE;
    case KBD_SCAN_CAPS_LOCK:
        kbd_update_lock(is_break, BDA_KBD_FLAGS_1_CAPS_LOCK, BDA_KBD_FLAGS_1_CAPS_DOWN);
        return TRUE;
    case KBD_SCAN_NUM_LOCK:
        kbd_update_lock(is_break, BDA_KBD_FLAGS_1_NUM_LOCK, BDA_KBD_FLAGS_1_NUM_DOWN);
        return TRUE;
    case KBD_SCAN_SCROLL_LOCK:
        kbd_update_lock(is_break, BDA_KBD_FLAGS_1_SCROLL_LOCK, BDA_KBD_FLAGS_1_SCROLL_DOWN);
        return TRUE;
    case KBD_SCAN_SYS_REQ:
        kbd_warn_scan(KBD_SCAN_SYS_REQ);
        return TRUE;
    }

    return FALSE;
}


static uint8_t kbd_handle_alt_num(uint8_t scan)
{
    uint16_t new_val;

    if (!kbd_is_pad_num(scan)) {
        return FALSE;
    }

    new_val = bda_read_byte(BDA_OFFSET_KBD_ALT_PAD_AREA);
    new_val *= 10;

    new_val += scan_trans[scan].alt;

    if (new_val > 0xff) {
        beep();
        new_val = 0;
    }

    bda_write_byte(BDA_OFFSET_KBD_ALT_PAD_AREA, new_val);

    return TRUE;
}


static void process_scan(uint8_t scan)
{
    uint16_t flags_1;
    uint16_t key_val;
    uint8_t is_break;

    if (kbd_handle_leds(scan)) {
        return;
    }

    if (kbd_process_compound(scan)) {
        return;
    }

    if (kbd_process_modifiers(scan)) {
        return;
    }

    is_break = scan & KBD_BREAK_MASK;
    scan &= ~KBD_BREAK_MASK;

    if (scan > KBD_MAX_KEY_SCAN) {
        kbd_warn_scan(scan | is_break);
        return;
    }

    flags_1 = bda_read_word(BDA_OFFSET_KBD_FLAGS_1);

    if (is_break) {
        if (scan == KBD_SCAN_INSERT) {
            flags_1 &= ~BDA_KBD_FLAGS_1_INSERT_DOWN;
            bda_write_word(BDA_OFFSET_KBD_FLAGS_1, flags_1);
        }

        return;
    }

    if ((flags_1 & BDA_KBD_FLAGS_1_ALT)) {
        if (kbd_handle_alt_num(scan)) {
            return;
        }

        key_val = scan_trans[scan].alt;
    } else if ((flags_1 & BDA_KBD_FLAGS_1_CTRL)) {
        key_val = scan_trans[scan].control;
    } else {
        uint8_t shift = flags_1 & (BDA_KBD_FLAGS_1_L_SHIFT | BDA_KBD_FLAGS_1_R_SHIFT);
        uint8_t lock_shift = (kbd_is_pad_num(scan) && (flags_1 & BDA_KBD_FLAGS_1_NUM_LOCK)) ||
                             (kbd_is_alpha(scan) && (flags_1 & BDA_KBD_FLAGS_1_CAPS_LOCK));
        shift = !!shift ^ !!lock_shift;

        if (shift) {
            key_val = scan_trans[scan].shift;
        } else {

            if (scan == KBD_SCAN_INSERT) {
                flags_1 ^= BDA_KBD_FLAGS_1_INSERT;
                flags_1 |= BDA_KBD_FLAGS_1_INSERT_DOWN;
                bda_write_word(BDA_OFFSET_KBD_FLAGS_1, flags_1);
            }

            key_val = scan_trans[scan].no_modifiers;
        }
    }

    if (!key_val) {
        return;
    }

    kbd_push_key(key_val);
}


void on_keyboard_interrupt()
{
    uint8_t val;

    restore_ds();
    val = inb(IO_PORT_KBD_STATUS);

    if ((val & KBDCTRL_STATUS_DATA_READY_MASK)) {
        if ((val & KBDCTRL_STATUS_MOUSE_DATA_READY_MASK)) {
            // unexpected.
            bios_info(BIOS_INFO_KBD_INT_MOUSE_DATA);
        } else {
            uint8_t process;

            val = inb(IO_PORT_KBD_DATA);

            __asm {
                mov ah, INT15_FUNC_KBD_INTERCEPT
                mov al, val
                int 0x15
                jnc no_process
                mov val, al
                mov ah, 1
                jmp done
            no_process:
                mov ah, 0
            done:
                mov process, ah
            }

            if (process) {
                process_scan(val);
            }
        }
    }

    outb(IO_PORT_PIC1, PIC_SPECIFIC_EOI_MASK | PIC1_KEYBOARD_PIN);
}


void on_int16(UserRegs __far * context)
{
    switch (REG_HI(context->ax)) {
    case INT16_FUNC_READ_KEY:
    case INT16_FUNC_READ_KEY_EXT: {
        uint16_t key;
        uint8_t self_wait = FALSE;

        for (;;) {
            if (REG_HI(context->ax) == INT16_FUNC_READ_KEY) {
                key = kbd_pop_nonext_key();
            } else {
                key = kbd_pop_key();
            }

            if (key) {
                break;
            }

            if (self_wait) {
                STI();
                HALT();
                CLI();
            } else {
                uint8_t bios_flags = ebda_read_byte(OFFSET_OF(EBDA, private) +
                                                    OFFSET_OF(EBDAPrivate, bios_flags));
                ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags),
                                bios_flags | BIOS_FLAGS_KBD_WAIT);

                __asm {
                    mov ah, INT15_FUNC_DEVICE_BUSY
                    mov al, 0x02 ; device type keyboard
                    int 0x15
                    jc no_self_wait ; clear if driver must perform wait
                    mov al, 1
                    mov self_wait, al
                no_self_wait:
                }

                bios_flags = ebda_read_byte(OFFSET_OF(EBDA, private) +
                                                    OFFSET_OF(EBDAPrivate, bios_flags));
                ebda_write_byte(OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, bios_flags),
                                bios_flags & ~BIOS_FLAGS_KBD_WAIT);
            }
        }

        context->ax = key; // ah scan, al ascii
        break;
    }
    case INT16_FUNC_PEEK_KEY: {
        uint16_t key = kbd_peek_nonext_key();

        if (key) {
            context->flags &= ~(1 << CPU_FLAGS_ZF_BIT);
            context->ax = key;
        } else {
            context->flags |= (1 << CPU_FLAGS_ZF_BIT);
        }

        break;
    }
    case INT16_FUNC_PEEK_KEY_EXT: {
         uint16_t key = kbd_peek_key();

        if (key) {
            context->flags &= ~(1 << CPU_FLAGS_ZF_BIT);
            context->ax = key;
        } else {
            context->flags |= (1 << CPU_FLAGS_ZF_BIT);
        }

        break;
    }
    default:
        context->flags |= (1 << CPU_FLAGS_CF_BIT);
        REG_HI(context->ax) = 0x86;
    }
}


static uint8_t kbd_receive_data()
{
    while (!(inb(IO_PORT_KBD_STATUS) & KBDCTRL_STATUS_DATA_READY_MASK)) {
        delay(10);
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
            delay(10);
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
            delay(10);
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
        delay(10);
        platform_debug_print(__FUNCTION__ ": unable to write");
    }

    outb(IO_PORT_KBD_DATA, val);
}


static void kbd_send_command(uint8_t val)
{
    outb(IO_PORT_KBD_COMMAND, val);
}


static inline void init_keyboard()
{
    uint8_t command_byte;

    post(POST_CODE_KEYBOARD);

    kbd_send_command(KBDCTRL_CMD_SELF_TEST);

    if (kbd_receive_data() != KBDCTRL_SELF_TEST_REPLAY) {
        platform_debug_print(__FUNCTION__ ": keyboard self test failed, halting...");
        freeze();
    }

    kbd_send_command(KBDCTRL_CMD_ENABLE_KEYBOARD);
    kbd_send_command(KBDCTRL_CMD_KEYBOARD_INTERFACE_TEST);

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

    kbd_send_command(KBDCTRL_CMD_READ_COMMAND_BYTE);
    command_byte = kbd_receive_data();
    kbd_send_command(KBDCTRL_CMD_WRITE_COMMAND_BYTE);
    kbd_send_data(command_byte | KBDCTRL_COMMAND_BYTE_IRQ1_MASK |
                  KBDCTRL_COMMAND_BYTE_TRANSLATE_MASK);

    kbd_send_data(KBD_CMD_GET_ID);
    if (kbd_receive_keyboard_data() != KBD_ACK) {
        platform_debug_print(__FUNCTION__ ": enable kbd failed, halting...");
        freeze();
    }

    ebda_write_byte(EBDA_OFFSET_KBD_ID, kbd_receive_keyboard_data());
    ebda_write_byte(EBDA_OFFSET_KBD_ID + 1, kbd_receive_keyboard_data());

    ebda_write_byte(EBDA_OFFSET_KBD_RATE, 0x0b);    /* 10.9 characters per second */
    ebda_write_byte(EBDA_OFFSET_KBD_DELAY, 1);      /* 500 ms delay */

    kbd_send_data(KBD_CMD_ENABLE_SCANNING);
    if (kbd_receive_keyboard_data() != KBD_ACK) {
        platform_debug_print(__FUNCTION__ ": enable kbd failed, halting...");
        freeze();
    }

    bda_write_word(BDA_OFFSET_KBD_FLAGS_2, BDA_KBD_FLAGS_2_KB_TYPE);

    bda_write_word(BDA_OFFSET_KBD_HEAD, BDA_OFFSET_KBD_BUF);
    bda_write_word(BDA_OFFSET_KBD_TAIL, BDA_OFFSET_KBD_BUF);
    bda_write_word(BDA_OFFSET_KBD_BUF_START, BDA_OFFSET_KBD_BUF);
    bda_write_word(BDA_OFFSET_KBD_BUF_END, BDA_OFFSET_KBD_BUF + BDA_KBD_DEFAULT_BUF_SIZE);

    set_int_vec(0x09, get_cs(), FUNC_OFFSET(keyboard_interrupt_handler));
    set_int_vec(0x16, get_cs(), FUNC_OFFSET(int16_handler));
    outb(IO_PORT_PIC1 + 1, (inb(IO_PORT_PIC1 + 1) & ~(1 << PIC1_KEYBOARD_PIN)));
}


void on_mouse_interrupt()
{
    uint8_t val;

    val = inb(IO_PORT_KBD_STATUS);

    if ((val & KBDCTRL_STATUS_DATA_READY_MASK)) {
        if (!(val & KBDCTRL_STATUS_MOUSE_DATA_READY_MASK)) {
            // unexpected.
            bios_info(BIOS_INFO_MOUSE_INT_KBD_DATA);
        } else {
            val = inb(IO_PORT_KBD_DATA);
        }
    }

    outb(IO_PORT_PIC1, PIC_SPECIFIC_EOI_MASK | PIC1_SLAVE_PIN);
    outb(IO_PORT_PIC2, PIC_SPECIFIC_EOI_MASK | PIC2_MOUSE_PIN);
}


static inline void init_mouse()
{
    uint8_t command_byte;

    post(POST_CODE_MOUSE);

    kbd_send_command(KBDCTRL_CMD_ENABLE_MOUSE);
    kbd_send_command(KBDCTRL_CMD_MOUSE_INTERFACE_TEST);

    if (kbd_receive_data()) {
        platform_debug_print(__FUNCTION__ ": mouse interface failed, halting...");
        freeze();
    }

    kbd_send_command(KBDCTRL_CMD_WRITE_TO_MOUSE);
    kbd_send_data(MOUSE_CMD_RESET);

    if (kbd_receive_mouse_data() != KBD_ACK || kbd_receive_mouse_data() != KBD_SELF_TEST_REPLAY ||
                                                                        kbd_receive_mouse_data()) {
        platform_debug_print(__FUNCTION__ ": mouse reset failed, halting...");
        freeze();
    }

    kbd_send_command(KBDCTRL_CMD_DISABLE_MOUSE);

    kbd_send_command(KBDCTRL_CMD_READ_COMMAND_BYTE);
    command_byte = kbd_receive_data();
    kbd_send_command(KBDCTRL_CMD_WRITE_COMMAND_BYTE);
    kbd_send_data(command_byte | KBDCTRL_COMMAND_BYTE_IRQ12_MASK);

    bda_write_word(BDA_OFFSET_EQUIPMENT,
                   bda_read_word(BDA_OFFSET_EQUIPMENT) | (1 << BDA_EQUIPMENT_MOUSE_BIT));

    set_int_vec(0x74, get_cs(), FUNC_OFFSET(mouse_interrupt_handler));
    outb(IO_PORT_PIC2 + 1, (inb(IO_PORT_PIC2 + 1) & ~(1 << PIC2_MOUSE_PIN)));
}


static void play_note(uint32_t frequency, uint duration)
{
    uint32_t countdown = 1193182UL / frequency;
    uint8_t misc;

    if (!countdown || countdown > 0xffff) {
        return;
    }

    // set timer 2: mode 3, 16bit, binary
    outb(IO_PORT_TIMER_CONTROL, (2 << PIT_SELECTOR_SHIFT) |
                                (3 << PIT_RW_SHIFT) |
                                (3 << PIT_MODE_SHIFT));

    outb(IO_PORT_TIMER_2, countdown);
    outb(IO_PORT_TIMER_2, countdown >> 8);

    misc = inb(IO_PORT_MISC);
    outb(IO_PORT_MISC, misc | 0x3);
    delay(duration);
    outb(IO_PORT_MISC, (misc & ~0x3));
}


void init()
{
    uint i;

    post(POST_CODE_INIT16);
    init_bios_data_area();

    call32();

    init_int_vector();

    platform_debug_print("log from 16bit");

    setup_pit_irq();
    setup_rtc_irq();

    init_keyboard();
    init_mouse();

    STI();

    post(POST_CODE_TMP);

    //Octave 6
    play_note(1046, 250);
    delay(1000);
    play_note(1175, 250);
    delay(1000);
    play_note(1328, 250);
    delay(1000);
    play_note(1397, 250);
    delay(1000);
    play_note(1568, 250);
    delay(1000);
    play_note(1760, 250);
    delay(1000);
    play_note(1975, 250);
    delay(1000);

    for (;;) {
        uint8_t scan;
        uint8_t ascii;
        char str[100];
        char ascii_str[2];

        ascii_str[1] = 0;

        __asm {
            mov ah, INT16_FUNC_READ_KEY
            int 16h
            mov scan, ah
            mov ascii, al
        }

        ascii_str[0] = ascii;

        format_str(str, "scan 0x%x ascii 0x%x %s", sizeof(str), scan, ascii, ascii_str);
        platform_debug_print(str);
    }

    for ( i = 0; i < 10; i++) {
        delay(2000);
        platform_debug_print("post delay");
    }

    for (;;) {
         __asm {
            mov ah, INT15_FUNC_WAIT
            mov cx, 0x98
            mov dx, 0x9680
            int 15h
        }
        platform_debug_print("post wait");
    }

    restart();
}

