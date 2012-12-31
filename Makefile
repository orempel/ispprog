PRG            = ispprog
OBJ            = ispprog.o
MCU_TARGET     = atmega16
OPTIMIZE       = -Os

DEFS           =
LIBS           =

# You should not have to change anything below here.

CC             = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
SIZE           = avr-size

all: $(PRG).elf lst text
	$(SIZE) -x -A $(PRG).elf

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.o $(PRG).lst $(PRG).map $(PRG).elf $(PRG).hex $(PRG).bin

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin

hex:  $(PRG).hex
bin:  $(PRG).bin

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

install: text
#	uisp -dprog=stk200 --erase --upload if=$(PRG).hex --verify
	avrdude -p m16 -c butterfly -b 19200 -P /dev/ttyUSB0 -U flash:w:$(PRG).hex
