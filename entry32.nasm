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

; copy from watcom i8d.asm
global   __I8D
__I8D:
        or      edx,edx         ; check sign of dividend
        js      .divneg          ; handle case where dividend < 0
        or      ecx,ecx         ; check sign of divisor
        js      .notU8D          ; easy case if it is also positive

        ; dividend >= 0, divisor >= 0
        call  __U8D ; docall  __U8D           ; - ...
        ret                     ; - ...

        ; dividend >= 0, divisor < 0
.notU8D neg     ecx             ; take positive value of divisor
        neg     ebx             ; ...
        sbb     ecx,0           ; ...
        call  __U8D ; docall  __U8D           ; do unsigned division
        neg     edx             ; negate quotient
        neg     eax             ; ...
        sbb     edx,0           ; ...
        ret                     ; and return

.divneg:                         ; dividend is negative
        neg     edx             ; take absolute value of dividend
        neg     eax             ; ...
        sbb     edx,0           ; ...
        or      ecx,ecx         ; check sign of divisor
        jns     .negres          ; negative result if divisor > 0

        ; dividend < 0, divisor < 0
        neg     ecx             ; negate divisor too
        neg     ebx             ; ...
        sbb     ecx,0           ; ...
        call  __U8D ;docall  __U8D           ; and do unsigned division
        neg     ecx             ; negate remainder
        neg     ebx             ; ...
        sbb     ecx,0           ; ...
        ret                     ; and return

        ; dividend < 0, divisor >= 0
.negres: call  __U8D ; docall  __U8D           ; do unsigned division
        neg     ecx             ; negate remainder
        neg     ebx             ; ...
        sbb     ecx,0           ; ...
        neg     edx             ; negate quotient
        neg     eax             ; ...
        sbb     edx,0           ; ...
        ret                     ; and return

; copy from watcom i8d.asm
global __U8D
__U8D:
        or      ecx,ecx         ; check for easy case
        jne     .noteasy        ; easy if divisor is 16 bit
        dec     ebx             ; decrement divisor
        je .L1 ;_if     ne              ; if not dividing by 1
          inc   ebx             ; - put divisor back
          cmp   ebx,edx         ; - if quotient will be >= 64K
          jnbe .L2 ;_if   be              ; - then
;
;       12-aug-88, added thanks to Eric Christensen from Fox Software
;       divisor < 64K, dividend >= 64K, quotient will be >= 64K
;
;       *note* this sequence is used in ltoa's #pragmas; any bug fixes
;              should be reflected in ltoa's code bursts
;
            mov   ecx,eax       ; - - save low word of dividend
            mov   eax,edx       ; - - get high word of dividend
            sub   edx,edx       ; - - zero high part
            div   ebx           ; - - divide bx into high part of dividend
            xchg  eax,ecx       ; - - swap high part of quot,low word of dvdnd
          .L2: ; _endif                ; - endif
          div   ebx             ; - calculate low part
          mov   ebx,edx         ; - get remainder
          mov   edx,ecx         ; - get high part of quotient
          sub   ecx,ecx         ; - zero high part of remainder
        .L1: ;     _endif                  ; endif
        ret                     ; return


.noteasy:                        ; have to work to do division
;
;       check for divisor > dividend
;
        ;_guess                  ; guess: divisor > dividend
          cmp   ecx,edx         ; - quit if divisor <= dividend
          jb .L5 ;_quif b               ; - . . .
          jne .L3;_if   e               ; - if high parts are the same
            cmp   ebx,eax       ; - - compare the lower order words
            ja .L4 ;_if   be            ; - - if divisor <= dividend
              sub   eax,ebx     ; - - - calulate remainder
              mov   ebx,eax     ; - - - ...
              sub   ecx,ecx     ; - - - ...
              sub   edx,edx     ; - - - quotient = 1
              mov   eax,1       ; - - - ...
              ret               ; - - - return
            .L4: ;_endif              ; - - endif
          .L3: ;_endif                ; - endif
          sub   ecx,ecx         ; - set divisor = 0 (this will be quotient)
          sub   ebx,ebx         ; - ...
          xchg  eax,ebx         ; - return remainder = dividend
          xchg  edx,ecx         ; - and quotient = 0
          ret                   ; - return
        .L5: ; _endguess               ; endguess
        push    ebp              ; save work registers
        push    esi              ; ...
        push    edi              ; ...
        sub     esi,esi           ; zero quotient
        mov     edi,esi           ; ...
        mov     ebp,esi           ; and shift count
.moveup:                         ; loop until divisor > dividend
          add ebx, ebx ;_shl    ebx,1         ; - divisor *= 2
          adc ecx, ecx ;_rcl    ecx,1         ; - ...
          jc      .backup        ; - know its bigger if carry out
          inc     ebp           ; - increment shift count
          cmp     ecx,edx       ; - check if its bigger yet
          jb      .moveup        ; - no, keep going
          ja      .divlup        ; - if below, know we're done
          cmp     ebx,eax       ; - check low parts (high parts equal)
          jbe     .moveup        ; until divisor > dividend
.divlup:                         ; division loop
        clc                     ; clear carry for rotate below
        .L6: ;_loop                   ; loop
          .L7: ;_loop                 ; - loop
            adc esi, esi ; _rcl  esi,1         ; - - shift bit into quotient
            adc edi, edi ; _rcl  edi,1         ; - - . . .
            dec   ebp           ; - - quif( -- shift < 0 ) NB carry not changed
            js    .donediv       ; - - ...
.backup:                         ; - - entry to remove last shift
            rcr   ecx,1         ; - - divisor /= 2 (NB also used by 'backup')
            rcr   ebx,1         ; - - ...
            sub   eax,ebx       ; - - dividend -= divisor
            sbb   edx,ecx       ; - - c=1 iff it won't go
            cmc                 ; - - c=1 iff it will go
          jb .L7 ;_until  nc            ; - until it won't go
          .L8: ;_loop                 ; - loop
            add esi, esi ;_shl  esi,1         ; - - shift 0 into quotient
            adc edi, edi ;_rcl  edi,1         ; - - . . .
            dec   ebp           ; - - going to add, check if done
            js    .toomuch       ; - - if done, we subtracted to much
            shr   ecx,1         ; - - divisor /= 2
            rcr   ebx,1         ; - - ...
            add   eax,ebx       ; - - dividend += divisor
            adc   edx,ecx       ; - - c = 1 iff bit of quotient should be 1
          jae .L8 ;_until  c             ; - until divisor will go into dividend
        jmp .L6 ;_endloop                ; endloop
.toomuch:                        ; we subtracted too much
        add     eax,ebx         ; dividend += divisor
        adc     edx,ecx         ; ...
.donediv:                        ; now quotient in di;si, remainder in dx;ax
        mov     ebx,eax         ; move remainder to cx;bx
        mov     ecx,edx         ; ...
        mov     eax,esi         ; move quotient to dx;ax
        mov     edx,edi         ; ...
        pop     edi             ; restore registers
        pop     esi             ; ...
        pop     ebp             ; ...
        ret                     ; and return


; copy from watcom i8m386.asm
global __U8M
global __I8M
__U8M:
__I8M:
        test    edx,edx         ; first check for easy (hiwords == 0) case
        jnz     .L1
        test    ecx,ecx
        jnz     .L1
        mul     ebx
        ret

.L1:    push    eax             ; save M1.l
        push    edx             ; save M1.h
        mul     ecx             ; calc M1.l * M2.h -> eax
        mov     ecx,eax         ; save M1.l * M2.h in ecx
        pop     eax             ; get  M1.h in eax
        mul     ebx             ; calc M1.h * M2.l -> eax
        add     ecx,eax         ; add  above to previous total
        pop     eax             ; get  M1.l in eax
        mul     ebx             ; calc M1.l * M2.l -> edx:eax
        add     edx,ecx         ; add previous hiword contribs to hiword
        ret                     ; and return!!!

