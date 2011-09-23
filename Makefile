
WL=~/watcom/binl/wlink
WCC=~/watcom/binl/wcc
WCC386=~/watcom/binl/wcc386
NASM=NASM

%.inc : %.h
	tr -t '#' '%' < $< | sed s/^'\/''\*'/"%if 0"/ | sed s/^'\*''\/'/"%endif"/ > $@

%.o : %.nasm
	nasm -f obj -o $@ $<

ignition.bin : bios.bin bios32.bin
	cp bios32.bin ignition.bin
	cat bios.bin >> $@

bios32.bin : entry32.o bios32.o
	$(WL) option q @bios32.link
	[[ $$(stat -c%s $@) -le 65536 ]]
	[[ $$(stat -c%s $@) -eq 65536 ]] || dd bs=1 count=1 seek=65535 if=/dev/zero of=$@

entry32.o : entry32.nasm defs.inc

bios32.o : bios32.c types.h defs.h common.c
	$(WCC386) -q -6 -ecc -zls -s -os $<

bios.bin : entry.o bios.o jump.bin
	$(WL) option q @bios.link
	[[ $$(stat -c%s $@) -le 65520 ]]
	dd bs=1 count=16 seek=65520 if=jump.bin of=$@

entry.o : entry.nasm defs.inc

jump.bin : jump.nasm
	nasm -f bin -o $@ $<

bios.o : bios.c  types.h defs.h common.c
	$(WCC) -q -0 -ecc -zls -ms  -zc -s $<

clean :
	rm -f *.o *.bin defs.inc

all : ignition.bin

dump : bios.bin
	objdump -b binary -m i386 -Maddr16,data16 -D $< | less

dump32 : bios32.bin
	objdump -b binary -m i386 -Maddr32,data32 -D $< | less

