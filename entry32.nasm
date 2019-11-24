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

cpu 686
segment _TEXT class=CODE USE32 align=1
group DGROUP _TEXT


%include "defs.inc"


extern _entry
extern _directory_service
extern _pcibios_service


start:
    mov ax, DATA_SEGMENT_SELECTOR
    mov ds, ax
    mov es, ax
    mov ss, ax
    xor eax, eax
    mov ax, [(BDA_SEG << 4) + BDA_OFFSET_EBDA]
    shl eax, 4
    add eax, EBDA_PRIVATE_START + PRIVATE_OFFSET_PM_STACK_BASE
    mov esp, [eax]

    call _entry

.infloop:
    hlt
    jmp .infloop


global _ret_16
_ret_16:
    jmp dword CODE16_SEGMENT_SELECTOR:BACK_FROM_PM_START_ADDRESS


global ___RSDP
align 16
___RSDP:
times 64 db 0


; 32bit service directory
align 16
global DIRECTORY_SERVICE
DIRECTORY_SERVICE:
    db "_32_"
    dd directory_service + 0xe0000
    db 0            ; revision
    db 1            ; size
    db 0            ; checksum
    times 5 db 0    ; resrved mbz


directory_service:
    ; all regs must be preserve except fot those that are use for output

    pushfd
    pushad
    push ds
    push es
    push fs
    push gs

    cli
    mov eax, esp
    mov ebx, ss
    push ebx
    push eax
    call _directory_service
    add esp, 8

    pop gs
    pop fs
    pop es
    pop ds
    popad
    popfd

    retf

global _pcibios_service_entry
_pcibios_service_entry:
    ; all regs must be preserve except fot those that are use for output

    pushfd
    pushad
    push ds
    push es
    push fs
    push gs

    cli
    mov eax, esp
    mov ebx, ss
    push ebx
    push eax
    call _pcibios_service
    add esp, 8

    pop gs
    pop fs
    pop es
    pop ds
    popad
    popfd

    retf

