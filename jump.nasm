bits 16

jmp 0xf000:0x0000

times (16 - ($-$$)) db 0

