%if 0
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
%endif

segment ENTRY class=CODE USE16 align=1 CPU=686
group DGROUP ENTRY

%define BIOS16_STACK_BASE 0xfff0

%include "defs.inc"
%include "asm.inc"

%macro SOFT_INT_PUSH_ALL 0
    pushad
    push ds
    push es
    push fs
    push gs
%endmacro

%macro SOFT_INT_POP_ALL 0
    pop gs
    pop fs
    pop es
    pop ds
    popad
%endmacro

extern _init
extern _on_hard_interrupt
extern _set_irq_context
extern _clear_irq_context
extern _on_int19
extern _freeze

global _unhandled_interrupt
global _call32


entry:
    jmp .start
    jmp .back_from_32bit
.start:
    mov al, RTC_NMI_MASK
    out IO_PORT_RTC_INDEX, al ; disable NMI
    cli
    mov al, POST_CODE_START16
    mov dx, IO_PORT_POST_CODE
    out dx, al

    mov ax, cs
    mov ds, ax
    xor ax, ax
    mov ss, ax
    mov sp, BIOS16_STACK_BASE
    call _init
    cli
.infloop:
    hlt
    jmp .infloop

.real_mode_idt:
    dw 1024 - 1
    dd 0

.back_from_32bit:
    mov edi, BIOS_DATA_AREA_ADDRESS + BDA_OFFSET_EBDA
    xor eax, eax
    mov ax, [edi]
    shl eax, 4
    add eax, EBDA_PRIVATE_START + PRIVATE_OFFSET_FLAGS
    mov edi, eax
    mov al, [edi]
    test al, BIOS_FLAGS_UNREAL
    jnz .unreal
    mov ax, DATA16_SEGMENT_SELECTOR
    jmp .set
.unreal:
    mov ax, UNREAL_SEGMENT_SELECTOR
.set:
    mov cx, DATA16_SEGMENT_SELECTOR
    mov ds, cx
    lidt [.real_mode_idt]

    mov ss, ax
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax

    mov eax, cr0
    and eax, ~CR0_PE
    mov cr0, eax

    jmp BIOS16_CODE_SEGMENT:.in_real_mode

.in_real_mode:
    xor ax, ax
    mov ds, ax
    mov ds, [BIOS_DATA_AREA_ADDRESS + BDA_OFFSET_EBDA]
    mov ss, [EBDA_PRIVATE_START + PRIVATE_OFFSET_REAL_MODE_SS]
    mov sp, [EBDA_PRIVATE_START + PRIVATE_OFFSET_REAL_MODE_SP]

    pop ds
    pop es
    pop fs
    pop gs
    popf
    popa
    ret

%macro HARD_INTERRUPT 1
global _hard_interrup_%1
_hard_interrup_%1:
    push ds
    push ax

    xor ax, ax
    mov ds, ax
    mov ds, [BIOS_DATA_AREA_ADDRESS + BDA_OFFSET_EBDA]
    mov [EBDA_PRIVATE_START + PRIVATE_OFFSET_USER_SS], ss
    mov [EBDA_PRIVATE_START + PRIVATE_OFFSET_USER_SP], sp
    mov ss, [EBDA_PRIVATE_START + PRIVATE_OFFSET_HARD_INT_SS]
    mov sp, [EBDA_PRIVATE_START + PRIVATE_OFFSET_HARD_INT_SP]

    pusha
    push es
    push fs
    push gs

    mov ax, cs
    mov ds, ax

    call _set_irq_context
    push word %1
    call _on_hard_interrupt
    add sp, 2
    call _clear_irq_context

    pop gs
    pop fs
    pop es
    popa

    xor ax, ax
    mov ds, ax
    mov ds, [BIOS_DATA_AREA_ADDRESS + BDA_OFFSET_EBDA]
    mov ss, [EBDA_PRIVATE_START + PRIVATE_OFFSET_USER_SS]
    mov sp, [EBDA_PRIVATE_START + PRIVATE_OFFSET_USER_SP]

    pop ax
    pop ds

    iret
%endmacro

HARD_INTERRUPT 0
HARD_INTERRUPT 1
HARD_INTERRUPT 2
HARD_INTERRUPT 3
HARD_INTERRUPT 4
HARD_INTERRUPT 5
HARD_INTERRUPT 6
HARD_INTERRUPT 7
HARD_INTERRUPT 8
HARD_INTERRUPT 9
HARD_INTERRUPT 10
HARD_INTERRUPT 11
HARD_INTERRUPT 12
HARD_INTERRUPT 13
HARD_INTERRUPT 14
HARD_INTERRUPT 15

%macro IRQ_HANDLER 1
extern _on_%1_interrupt
global _%1_interrupt_handler
_%1_interrupt_handler:
    push ds

    ; for now disable stack swap (windows loader assume the stack will not be
    ; change)

    ;push ax

    ;xor ax, ax
    ;mov ds, ax
    ;mov ds, [BIOS_DATA_AREA_ADDRESS + BDA_OFFSET_EBDA]
    ;mov [EBDA_PRIVATE_START + PRIVATE_OFFSET_USER_SS], ss
    ;mov [EBDA_PRIVATE_START + PRIVATE_OFFSET_USER_SP], sp
    ;mov ss, [EBDA_PRIVATE_START + PRIVATE_OFFSET_HARD_INT_SS]  BUG: need to be relative
    ;mov sp, [EBDA_PRIVATE_START + PRIVATE_OFFSET_HARD_INT_SP]

    pusha
    push es
    push fs
    push gs

    mov ax, cs
    mov ds, ax

    call _set_irq_context
    call _on_%1_interrupt
    call _clear_irq_context

    pop gs
    pop fs
    pop es
    popa

    ;xor ax, ax
    ;mov ds, ax
    ;mov ds, [BIOS_DATA_AREA_ADDRESS + BDA_OFFSET_EBDA]
    ;mov ss, [EBDA_PRIVATE_START + PRIVATE_OFFSET_USER_SS]
    ;mov sp, [EBDA_PRIVATE_START + PRIVATE_OFFSET_USER_SP]

    ;pop ax
    pop ds

    iret
%endmacro

IRQ_HANDLER pit ; org F000h:FEA5h in IBM PC and 100%-compatible BIOSes
IRQ_HANDLER rtc
IRQ_HANDLER keyboard ; org F000h:E82Eh in IBM PC and 100%-compatible BIOSes
IRQ_HANDLER mouse

%macro INT_HANDLER 1
extern _on_%1
global _%1_handler
_%1_handler:
    SOFT_INT_PUSH_ALL

    mov ax, cs
    mov ds, ax

    mov ax, sp
    push ss
    push ax
    call _on_%1
    add sp, 4

    SOFT_INT_POP_ALL
    iret
%endmacro

INT_HANDLER int11 ; org F000h:F84Dh in IBM PC and 100%-compatible BIOSes
INT_HANDLER int12 ; org F000h:F841h in IBM PC and 100%-compatible BIOSes
INT_HANDLER int13 ; org F000h:EC59h in IBM PC and 100%-compatible BIOSes
                  ; (the fixed address handler will call int13, int13_hd_emulate, or
                  ; int13_fd_emulate according to bios_flags)
INT_HANDLER int15 ; org F000h:F859h in IBM PC and 100%-compatible BIOSes
INT_HANDLER int16 ; org F000h:E82Eh in IBM PC and 100%-compatible BIOSes
INT_HANDLER int1a ; org F000h:FE6Eh in IBM PC and 100%-compatible BIOSes
INT_HANDLER unhandled_int


%macro INT_13_EMU 1
extern _on_int13_%1_emulate
global _int13_%1_emulate_handler
_int13_%1_emulate_handler:

    SOFT_INT_PUSH_ALL

    mov ax, cs
    mov ds, ax

    mov ax, sp
    push ss
    push ax
    call _on_int13_%1_emulate
    add sp, 4

    cmp ax, INT13_HANDLED
    je .handled
    cmp ax, INT13_DEC_AND_NEXT
    je .dec_and_next
    cmp ax, INT13_CALL_NEXT
    je .next
    call _freeze

.dec_and_next:
    SOFT_INT_POP_ALL
    push bp

    sub sp, 4
    mov bp, sp

    push ds
    push ax

    xor ax, ax
    mov ds, ax
    mov ds, [BIOS_DATA_AREA_ADDRESS + BDA_OFFSET_EBDA]
    mov ax, [EBDA_PRIVATE_START + PRIVATE_OFFSET_INT13_EMU_OFFSET]
    mov [bp], ax
    mov ax, [EBDA_PRIVATE_START + PRIVATE_OFFSET_INT13_EMU_SEG]
    mov [bp + 2], ax

    pop ax
    pop ds

    pushf
    dec dl
    call far [bp]
    inc dl

    add sp, 4
    mov bp, sp
    push ax
    pushf
    pop ax
    and ax, (1 << CPU_FLAGS_CF_BIT)
    and word [bp + 6], ~(1 << CPU_FLAGS_CF_BIT)
    or word [bp + 6], ax
    pop ax

    pop bp
    iret

.next:
    SOFT_INT_POP_ALL
    push bp

    sub sp, 4
    mov bp, sp

    push ds
    push ax

    xor ax, ax
    mov ds, ax
    mov ds, [BIOS_DATA_AREA_ADDRESS + BDA_OFFSET_EBDA]
    mov ax, [EBDA_PRIVATE_START + PRIVATE_OFFSET_INT13_EMU_OFFSET]
    mov [bp], ax
    mov ax, [EBDA_PRIVATE_START + PRIVATE_OFFSET_INT13_EMU_SEG]
    mov [bp + 2], ax

    pop ax
    pop ds

    pushf
    call far [bp]

    add sp, 4
    mov bp, sp
    push ax
    pushf
    pop ax
    and ax, (1 << CPU_FLAGS_CF_BIT)
    and word [bp + 6], ~(1 << CPU_FLAGS_CF_BIT)
    or word [bp + 6], ax
    pop ax

    pop bp
    iret

.handled:
    SOFT_INT_POP_ALL
    iret

%endmacro

INT_13_EMU fd
INT_13_EMU hd

global _int18_handler
_int18_handler:
global _int19_handler
_int19_handler: ; org F000h:E6F2h in IBM PC and 100%-compatible BIOSes
    ; disable NMI
    mov al, RTC_NMI_MASK
    out IO_PORT_RTC_INDEX, al

    ; reset stack
    xor ax, ax
    mov ss, ax
    mov sp, BIOS16_STACK_BASE

    ; disable A20
    mov dx, IO_PORT_SYSCTRL
    in al, dx
    and al, ~(1 << SYSCTRL_A20_BIT)
    out dx, al

    mov ax, cs
    mov ds, ax

    call _on_int19


global _dummy_interrupt
_dummy_interrupt: ; org F000h:FF53h in IBM PC and 100%-compatible BIOSes
    iret

align 8
gdt dd 0x00000000, 0x00000000; first descriptor must be null descriptor
    dd 0x0000ffff, (SD_CS | SD_PRESENT | SD_32 | SD_4K | (0x0f << SD_HLIMIT_SHIFT)) ; 32bit code
    dd 0x0000ffff, (SD_DS | SD_PRESENT| SD_32 | SD_4K | (0x0f << SD_HLIMIT_SHIFT))  ; 32bit data
    dd 0x0000000f, (SD_CS | SD_PRESENT | SD_4K | 0x0f) ; 16bit code base is 0xf0000 and limit 64kb
    dd 0x0000000f, (SD_DS | SD_PRESENT| SD_4K | 0x0f)  ; 16bit data base is 0xf0000 limit is 64kb
    dd 0x0000ffff, (SD_DS | SD_PRESENT| SD_4K | (0x0f << SD_HLIMIT_SHIFT)) ; 16bit unreal data
gdt_end:


_call32:
    pusha
    pushf
    push gs
    push fs
    push es
    push ds

    mov dx, ds
    xor ax, ax
    mov ds, ax
    mov ds, [BIOS_DATA_AREA_ADDRESS + BDA_OFFSET_EBDA]
    mov [EBDA_PRIVATE_START + PRIVATE_OFFSET_REAL_MODE_SS], ss
    mov [EBDA_PRIVATE_START + PRIVATE_OFFSET_REAL_MODE_SP], sp
    mov ds, dx

    mov bp, sp
    sub sp, 6
    mov al, RTC_NMI_MASK
    out IO_PORT_RTC_INDEX, al ; disable NMI
    cli
    mov word [bp - 6], 0
    mov word [bp - 4], 0
    mov word [bp - 2], 0
    lidt [bp - 6]
    mov ax, gdt_end - gdt - 1
    mov [bp - 6], ax
    mov dx, ds
    shr dx, 12
    mov ax, ds
    shl ax, 4
    add ax, gdt
    adc dx, 0
    mov [bp - 2], dx
    mov [bp - 4], ax
    lgdt [bp - 6]
    mov eax, cr0
    or eax, CR0_PE
    mov cr0, eax
    jmp dword CODE_SEGMENT_SELECTOR:(BIOS32_CODE_SEGMENT << 4)

