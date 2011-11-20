
LINK=~/watcom/binl/wlink
CC=~/watcom/binl/wcc
CC386=~/watcom/binl/wcc386

C_SOURCES_16 = bios.c
C_SOURCES_32 = bios32.c
C_SOURCES = $(C_SOURCES_16) $(C_SOURCES_32)

NASM_SOURCES_16 = entry.nasm
NASM_SOURCES_32 = entry32.nasm
NASM_SOURCES = $(NASM_SOURCES_16) $(NASM_SOURCES_32)

C_OBJECTS_16 = $(C_SOURCES_16:%.c=%.o)
C_OBJECTS_32 = $(C_SOURCES_32:%.c=%.o)

$(C_OBJECTS_16) : C_COMPILE = $(CC) -q -6 -ecc -zls -ms -zc -zu -s -os -we
$(C_OBJECTS_32) : C_COMPILE = $(CC386) -q -6 -ecc -zls -s -os -we

DEP_DIR = .deps
DEP_FILE = $(DEP_DIR)/$(*F).d

OBJECTS_16 = $(C_OBJECTS_16) $(NASM_SOURCES_16:%.nasm=%.o) jump.bin
OBJECTS_32 = $(C_OBJECTS_32) $(NASM_SOURCES_32:%.nasm=%.o)

AUTO_GEN = defs.inc


%.o : %.c
	@mkdir -p $(DEP_DIR)
	$(C_COMPILE) -fr=/dev/null -i=.. -ad=$(DEP_FILE) $<


%.inc : %.h
	@echo generateing $@
	@tr -t '#' '%' < $< | sed s/^'\/''\*'/"%if 0"/ | sed s/^'\*''\/'/"%endif"/ > $@.tmp
	@sed s/[.]*"\/\/"/";"/ < $@.tmp > $@
	@rm $@.tmp


%.o : %.nasm
	@mkdir -p $(DEP_DIR)
	@nasm -M -MT $@ $< > $(DEP_FILE)
	nasm -f obj -o $@ $<


ignition.bin : bios.bin bios32.bin
	cp bios32.bin ignition.bin
	cat bios.bin >> $@


bios32.bin : $(AUTO_GEN) $(OBJECTS_32)
	$(LINK) option q @bios32.link
	@echo output size is $$(stat -c%s $@)
	[[ $$(stat -c%s $@) -le 65536 ]]
	[[ $$(stat -c%s $@) -eq 65536 ]] || dd bs=1 count=1 seek=65535 if=/dev/zero of=$@


bios.bin : $(AUTO_GEN) $(OBJECTS_16)
	$(LINK) option q @bios.link
	@echo output size is $$(stat -c%s $@)
	[[ $$(stat -c%s $@) -le 65520 ]]
	dd bs=1 count=16 seek=65520 if=jump.bin of=$@


jump.bin : jump.nasm
	nasm -f bin -o $@ $<


clean :
	rm -f *.o *.bin defs.inc
	rm -rf .deps


all : ignition.bin


dump : bios.bin
	objdump -b binary -m i386 -Maddr16,data16 -D $< | less


dump32 : bios32.bin
	objdump -b binary -m i386 -Maddr32,data32 -D $< | less


-include $(C_SOURCES:%.c=$(DEP_DIR)/%.d)
-include $(NASM_SOURCES:%.nasm=$(DEP_DIR)/%.d)

