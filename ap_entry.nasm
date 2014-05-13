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


segment AP_ENTRY class=CODE USE16 align=4096 CPU=686
group DGROUP AP_ENTRY


%include "defs.inc"
%include "asm.inc"


extern _ap_init

global _ap_entry
_ap_entry:
    mov al, POST_CODE_AP16
    mov dx, IO_PORT_POST_CODE
    out dx, al
    jmp BIOS16_CODE_SEGMENT:.fix_cs

.fix_cs:
    xor ax, ax
    mov ds, ax
    mov ds, [BIOS_DATA_AREA_ADDRESS + BDA_OFFSET_EBDA]

.spin_lock:
    mov ax, 1
    lock xchg [EBDA_PRIVATE_START + PRIVATE_OFFSET_AP_LOCK], ax
    cmp ax, 0
    jne .spin_lock

    mov al, POST_CODE_AP16 + 1
    mov dx, IO_PORT_POST_CODE
    out dx, al

    mov ax, BIOS16_CODE_SEGMENT
    mov ds, ax
    xor ax, ax
    mov ss, ax
    mov sp, AP16_STACK_BASE
    call _ap_init
    cli
.infloop:
    hlt
    jmp .infloop

