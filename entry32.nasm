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

segment _TEXT class=CODE USE32 align=1 CPU=686
group DGROUP _TEXT


%include "defs.inc"


extern  _init


start:
    mov al, POST_CODE_START32
    mov dx, IO_PORT_POST_CODE
    out dx, al

    mov ax, DATA_SEGMENT_SELECTOR
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, PROTECTED_STACK_BASE

    call _init

.infloop:
    hlt
    jmp .infloop

global _halt
_halt:
    cli
.infloop:
    hlt
    jmp .infloop


; copy from watcom i8rs386.asm
global __U8LS
__U8LS:
    mov     ecx,ebx         ; get shift-count into cl
    and     cl,03fH         ; get mod 64 shift count
    test    cl,020H         ; see if count >= 32
    jnz     .g_or_e_32
    shld    edx,eax,cl
    shl     eax,cl
    ret                     ; and return!!!

.g_or_e_32:
    mov     edx,eax         ; shift lo into hi (1st 32 bits now shifted)
    sub     cl,020H         ; knock off 32-bits of shifting
    xor     eax,eax         ; lo 32 bits are now zero
    shl     edx,cl          ; shift remaining part
    ret                     ; and return!!!


; copy from watcom i8rs386.asm
global __U8RS
__U8RS:
    mov     ecx,ebx         ; get shift-count into cl
    and     cl,03fH         ; get mod 64 shift count
    test    cl,020H         ; see if count >= 32
    jnz     .g_or_e_32
    shrd    eax,edx,cl
    shr     edx,cl
    ret                     ; and return!!!

.g_or_e_32:
    mov     eax,edx
    sub     ecx,020H        ; knock off 32-bits of shifting
    xor     edx,edx         ; zero extend result
    shr     eax,cl
    ret

