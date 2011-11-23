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

segment _TEXT class=ENTRY USE16 align=1 CPU=686
group DGROUP _TEXT


%include "defs.inc"


extern _init
extern _on_unhandled_irq
extern _set_irq_context
extern _clear_irq_context


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
    mov sp, 0xfff0
    call _init
    cli
.infloop:
    hlt
    jmp .infloop

.real_mode_idt:
    dw 1024 - 1
    dd 0

.back_from_32bit:
    mov al, POST_CODE_BACK
    mov dx, IO_PORT_POST_CODE
    out dx, al

    mov ax, DATA16_SEGMENT_SELECTOR
    mov ss, ax
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax

    lidt [.real_mode_idt]

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

%macro UNHANDLE_IRQ 1
global _unhandled_irq%1
_unhandled_irq%1:
    push word %1
    call _on_unhandled_irq
    add esp, 2
    iret
%endmacro

UNHANDLE_IRQ 0
UNHANDLE_IRQ 1
UNHANDLE_IRQ 2
UNHANDLE_IRQ 3
UNHANDLE_IRQ 4
UNHANDLE_IRQ 5
UNHANDLE_IRQ 6
UNHANDLE_IRQ 7
UNHANDLE_IRQ 8
UNHANDLE_IRQ 9
UNHANDLE_IRQ 10
UNHANDLE_IRQ 11
UNHANDLE_IRQ 12
UNHANDLE_IRQ 13
UNHANDLE_IRQ 14
UNHANDLE_IRQ 15

%macro IRQ_HANDLER 1
extern _on_%1_interrupt
global _%1_interrupt_handler
_%1_interrupt_handler:
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
    call _on_%1_interrupt
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

IRQ_HANDLER pit ; org F000h:FEA5h in IBM PC and 100%-compatible BIOSes
IRQ_HANDLER rtc
IRQ_HANDLER keyboard ; org F000h:E82Eh in IBM PC and 100%-compatible BIOSes
IRQ_HANDLER mouse

%macro INT_HANDLER 1
extern _on_int%1
global _int%1_handler
_int%1_handler:
    pushad
    push ds
    push es
    push fs
    push gs

    mov ax, cs
    mov ds, ax

    mov ax, sp
    push ss
    push ax
    call _on_int%1
    add sp, 4

    pop gs
    pop fs
    pop es
    pop ds
    popad

    iret
%endmacro

INT_HANDLER 15 ; org F000h:F859h in IBM PC and 100%-compatible BIOSes
INT_HANDLER 16 ; org F000h:E82Eh in IBM PC and 100%-compatible BIOSes
INT_HANDLER 1a ; org F000h:FE6Eh in IBM PC and 100%-compatible BIOSes

_unhandled_interrupt:
    iret

align 8
gdt dd 0x00000000, 0x00000000; first descriptor must be null descriptor
    dd 0x0000ffff, (SD_CS | SD_PRESENT | SD_32 | SD_4K | (0x0f << SD_HLIMIT_SHIFT)) ; 32bit code
    dd 0x0000ffff, (SD_DS | SD_PRESENT| SD_32 | SD_4K | (0x0f << SD_HLIMIT_SHIFT))  ; 32bit data
    dd 0x00000010, (SD_CS | SD_PRESENT | SD_4K | 0x0f) ; 16bit code base is 0xf0000 and limit 64kb
    dd 0x00000010, (SD_DS | SD_PRESENT| SD_4K | 0x0f)  ; 16bit data base is 0xf0000 limit is 64kb
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
    jmp dword CODE_SEGMENT_SELECTOR:PROTECTED_START_ADDRESS

