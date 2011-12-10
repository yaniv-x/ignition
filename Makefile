
LINK=~/watcom/binl/wlink
CC=~/watcom/binl/wcc
CC386=~/watcom/binl/wcc386

C_SOURCES_16 = bios.c pci_16.c utils_16.c platform_16.c ata.c boot.c
C_SOURCES_32 = bios32.c pci_32.c utils_32.c service_directory.c platform_32.c
C_SOURCES = $(C_SOURCES_16) $(C_SOURCES_32)

C_OBJECTS_16 = $(C_SOURCES_16:%.c=%.o)
C_OBJECTS_32 = $(C_SOURCES_32:%.c=%.o)

$(C_OBJECTS_16) : C_COMPILE = $(CC) -q -6 -ecc -zls -ms -zc -zu -s -os -we
$(C_OBJECTS_32) : C_COMPILE = $(CC386) -q -6 -ecc -zls -s -os -we

NASM_SOURCES_16 = entry.nasm
NASM_SOURCES_32 = entry32.nasm
NASM_SOURCES = $(NASM_SOURCES_16) $(NASM_SOURCES_32)

NASM_OBJECTS_16 = $(NASM_SOURCES_16:%.nasm=%.o)
NASM_OBJECTS_32 = $(NASM_SOURCES_32:%.nasm=%.o)

DEP_DIR = .deps
DEP_FILE = $(DEP_DIR)/$(*F).d
TMP_DEP_FILE = $(DEP_DIR)/$(*F).tmp

OBJECTS_16 = $(C_OBJECTS_16) $(NASM_OBJECTS_16) jump.bin
OBJECTS_32 = $(C_OBJECTS_32) $(NASM_OBJECTS_32)

AUTO_GEN = defs.inc


%.o : %.c
	@mkdir -p $(DEP_DIR)
	$(C_COMPILE) -fr=/dev/null -i=.. -ad=$(TMP_DEP_FILE) $<
	@cp $(TMP_DEP_FILE) $(DEP_FILE)
	@sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' -e '/^$$/ d' -e 's/$$/ :/' \
                                                        < $(TMP_DEP_FILE) >> $(DEP_FILE)
	@rm $(TMP_DEP_FILE)


%.inc : %.h
	@echo generateing $@
	@tr -t '#' '%' < $< | sed s/^'\/''\*'/"%if 0"/ | sed s/^'\*''\/'/"%endif"/ > $@.tmp
	@sed s/[.]*"\/\/"/";"/ < $@.tmp > $@
	@rm $@.tmp


%.o : %.nasm
	@mkdir -p $(DEP_DIR)
	@nasm -M -MT $@ $< > $(TMP_DEP_FILE)
	@cp $(TMP_DEP_FILE) $(DEP_FILE)
	@sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' -e '/^$$/ d' -e 's/$$/ :/' \
                                                        < $(TMP_DEP_FILE) >> $(DEP_FILE)
	@rm $(TMP_DEP_FILE)
	nasm -f obj -o $@ $<


.DELETE_ON_ERROR:


ignition.bin : bios.bin bios32.bin
	@rm -f $@
	cp bios32.bin ignition.bin
	cat bios.bin >> $@


bios32.bin : $(AUTO_GEN) $(OBJECTS_32) fixup
	$(LINK) option q @bios32.link
	@echo output size is $$(stat -c%s $@)
	@./fixup $@ $(subst .bin,.map, $@) DIRECTORY_SERVICE,16,10
	[[ $$(stat -c%s $@) -le 65536 ]]
	[[ $$(stat -c%s $@) -eq 65536 ]] || dd bs=1 count=1 seek=65535 if=/dev/zero of=$@


bios.bin : $(AUTO_GEN) $(OBJECTS_16)
	$(LINK) option q @bios.link
	@echo output size is $$(stat -c%s $@)
	[[ $$(stat -c%s $@) -le 65520 ]]
	dd bs=1 count=16 seek=65520 if=jump.bin of=$@

fixup : fixup.c
	gcc -O0 -g -o fixup fixup.c

jump.bin : jump.nasm
	nasm -f bin -o $@ $<


clean :
	rm -f *.o *.bin defs.inc bios32.map bios.map fixup
	rm -rf .deps


all : ignition.bin


dump : bios.bin
	objdump -b binary -m i386 -Maddr16,data16 -D $< | less


dump32 : bios32.bin
	objdump -b binary -m i386 -Maddr32,data32 -D $< | less


-include $(C_SOURCES:%.c=$(DEP_DIR)/%.d)
-include $(NASM_SOURCES:%.nasm=$(DEP_DIR)/%.d)

