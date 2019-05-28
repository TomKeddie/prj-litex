GIT_VERSION := $(shell git describe --tags)

include ../include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

INCLUDES := -I$(BIOS_DIRECTORY)/../foboot/sw/include $(INCLUDES)
CFLAGS += -DGIT_VERSION=u\"$(GIT_VERSION)\"

OBJECTS=main.o usb-desc.o  usb-dev.o  usb-epfifo.o  usb-unififo.o dfu.o spi.o

all: bios.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifeq ($(CPUENDIANNESS),little)
	$(PYTHON) -m litex.soc.tools.mkmscimg $@ --little
else
	$(PYTHON) -m litex.soc.tools.mkmscimg $@
endif

bios.elf: $(BIOS_DIRECTORY)/linker.ld $(OBJECTS)

%.elf: ../libbase/crt0-$(CPU)-ctr.o ../libbase/libbase-nofloat.a ../libcompiler_rt/libcompiler_rt.a
	$(LD) $(LDFLAGS) -T $(BIOS_DIRECTORY)/linker.ld -N -o $@ \
		../libbase/crt0-$(CPU)-ctr.o \
		$(OBJECTS) \
		-L../libbase \
		-L../libcompiler_rt \
		-lbase-nofloat -lcompiler_rt

%.o: $(BIOS_DIRECTORY)/%.c
	$(compile)

%.o: $(BIOS_DIRECTORY)/../foboot/sw/src/%.c
	-rm -r $(BIOS_DIRECTORY)/../foboot/sw/include/generated
# riscv64-unknown-elf-gcc -E -Wp,-v -xc /dev/null
	$(compile)

clean:
	$(RM) $(OBJECTS) bios.elf bios.bin .*~ *~

.PHONY: all clean
all:
