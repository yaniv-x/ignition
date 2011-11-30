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

#ifndef _H_BIOS
#define _H_BIOS

#include "types.h"

#define STI() {                                                 \
    if (is_hard_int_context()) {                                \
        bios_error(BIOS_ERROR_STI_WHILE_IN_IRQ_CONTEXT);        \
    }                                                           \
                                                                \
    __asm { sti}                                                \
}

#define OFFSET_OF_PRIVATE(x) (OFFSET_OF(EBDA, private) + OFFSET_OF(EBDAPrivate, x))

#define FUNC_OFFSET(function) (uint16_t)(function)

uint16_t get_cs();
uint16_t get_ds();
uint16_t set_ds(uint16_t ds);
void restore_ds();

void call32();

uint16_t read_word(uint16_t seg, uint16_t offset);

uint16_t bda_read_word(uint16_t offset);
uint8_t bda_read_byte(uint16_t offset);
void bda_write_byte(uint16_t offset, uint8_t val);

uint16_t ebda_read_word(uint16_t offset);
uint8_t ebda_read_byte(uint16_t offset);
void ebda_write_byte(uint16_t offset, uint16_t val);

void delay(uint32_t milisec);
uint8_t is_hard_int_context();
void register_interrupt_handler(uint line, int_cb_t cb, uint opaque);
void unregister_interrupt_handler(uint line, int_cb_t cb, uint opaque);
void set_int_vec(uint8_t index, uint16_t seg, uint16_t offset);


bool_t ata_is_cdrom(ATADevice __far * device);
bool_t ata_is_hd(ATADevice __far * device);
bool_t ata_read_sectors(ATADevice __far * device, uint32_t address, uint count,
                        uint8_t __far * dest);
void ata_init();


void boot_add_hd(ATADevice __far * device);
void boot_add_cd(ATADevice __far * device);
void boot();
void boot_init();

#endif

