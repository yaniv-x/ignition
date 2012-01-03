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

segment _TEXT class=CODE USE16 align=1 CPU=686
group DGROUP _TEXT

%include "defs.inc"
%include "asm.inc"

extern _freeze


; void call_rom_init(uint16_t offset, uint16_t seg, uint8_t bus, uint8_t device)
global _call_rom_init
_call_rom_init:
    push bp
    mov bp, sp
    mov ah, [bp + 8]
    mov al, [bp + 10]
    shl al, 3
    PUSH_ALL
    call far [bp + 4]
    POP_ALL
    pop bp
    ret


global _far_call_no_return
_far_call_no_return:
    mov bp, sp
    call far [bp]
    call _freeze


; void in_words(uint16_t port, void __far * src, uint16_t num_words)
global _in_words
_in_words:
    push bp
    mov bp, sp
    push es
    push di

    mov dx, [bp + 4]
    mov di, [bp + 6]
    mov es, [bp + 8]
    mov cx, [bp + 10]
    rep insw

    pop di
    pop es
    pop bp
    ret


; void out_words(uint16_t port, void __far * src, uint16_t num_words)
global _out_words
_out_words:
    push bp
    mov bp, sp
    push ds
    push si

    mov dx, [bp + 4]
    mov si, [bp + 6]
    mov ds, [bp + 8]
    mov cx, [bp + 10]

    rep outsw

    pop si
    pop ds
    pop bp
    ret


global _call_mouse_handler
_call_mouse_handler:

    xor ax, ax
    mov ds, ax
    mov ds, [BIOS_DATA_AREA_ADDRESS + BDA_OFFSET_EBDA]

    mov al, [EBDA_OFFSET_MOUSE_DATA + 0]
    push ax
    mov al, [EBDA_OFFSET_MOUSE_DATA + 1]
    push ax
    mov al, [EBDA_OFFSET_MOUSE_DATA + 2]
    push ax
    xor ax, ax
    push ax

    sti
    call far [EBDA_OFFSET_MOUSE_HANDLER]
    cli

    add sp, 8

    mov ax, cs
    mov ds, ax

    ret

