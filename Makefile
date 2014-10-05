CC	:= avr-gcc
LD	:= avr-ld
OBJCOPY	:= avr-objcopy
OBJDUMP	:= avr-objdump
SIZE	:= avr-size

TARGET = ispprog
SOURCE = $(wildcard *.c)

# select MCU
MCU = atmega16

AVRDUDE_PROG := -c butterfly -b 19200 -P /dev/ttyUSB0
#AVRDUDE_PROG := -c dragon_isp -P usb

# ---------------------------------------------------------------------------

ifeq ($(MCU), atmega16)
# (ext. crystal)
AVRDUDE_MCU=m16
AVRDUDE_FUSES=lfuse:w:0xff:m hfuse:w:0xda:m
endif

# ---------------------------------------------------------------------------

CFLAGS = -pipe -g -Os -mmcu=$(MCU) -Wall -fdata-sections -ffunction-sections
CFLAGS += -Wa,-adhlns=$(*F).lst
LDFLAGS = -Wl,-Map,$(@:.elf=.map),--cref,--relax,--gc-sections

# ---------------------------------------------------------------------------

$(TARGET): $(TARGET).elf
	@$(SIZE) -B -x --mcu=$(MCU) $<

$(TARGET).elf: $(SOURCE:.c=.o)
	@echo " Linking file:  $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
	@$(OBJDUMP) -h -S $@ > $(@:.elf=.lss)
	@$(OBJCOPY) -j .text -j .data -O ihex $@ $(@:.elf=.hex)
	@$(OBJCOPY) -j .text -j .data -O binary $@ $(@:.elf=.bin)

%.o: %.c $(MAKEFILE_LIST)
	@echo " Building file: $<"
	@$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -rf $(SOURCE:.c=.o) $(SOURCE:.c=.lst) $(addprefix $(TARGET), .elf .map .lss .hex .bin)

install: $(TARGET).elf
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) -U flash:w:$(<:.elf=.hex)

fuses:
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) $(patsubst %,-U %, $(AVRDUDE_FUSES))
