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


extern  _init


global _unhandled_interrupt
global _call32


entry:
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

_unhandled_interrupt:
    iret

align 8
gdt dd 0x00000000, 0x00000000; first descriptor must be null descriptor
    dd 0x0000ffff, (SD_CS | SD_PRESENT | SD_32 | SD_4K | (0x0f << SD_HLIMIT_SHIFT)) ; cs
    dd 0x0000ffff, (SD_DS | SD_PRESENT| SD_32 | SD_4K | (0x0f << SD_HLIMIT_SHIFT))  ; ds
gdt_end:


_call32:
    mov bp, sp
    sub sp, 6
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

