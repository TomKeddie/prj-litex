include ../include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

OBJECTS=main.o

all: blink.bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
ifeq ($(CPUENDIANNESS),little)
	$(PYTHON) -m litex.soc.tools.mkmscimg $@ --little
else
	$(PYTHON) -m litex.soc.tools.mkmscimg $@
endif

blink.elf: $(BLINK_DIRECTORY)/linker.ld $(OBJECTS)

%.elf: ../libbase/crt0-$(CPU)-ctr.o ../libbase/libbase-nofloat.a ../libcompiler_rt/libcompiler_rt.a
	$(LD) $(LDFLAGS) -T $(BLINK_DIRECTORY)/linker.ld -N -o $@ \
		../libbase/crt0-$(CPU)-ctr.o \
		$(OBJECTS) \
		-L../libbase \
		-L../libcompiler_rt \
		-lbase-nofloat -lcompiler_rt

%.o: $(BLINK_DIRECTORY)/%.c
	$(compile)

clean:
	$(RM) $(OBJECTS) blink.elf blink.bin .*~ *~

.PHONY: all clean
all:
