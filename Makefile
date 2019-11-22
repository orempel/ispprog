CC	:= avr-gcc
LD	:= avr-ld
OBJCOPY	:= avr-objcopy
OBJDUMP	:= avr-objdump
SIZE	:= avr-size

TARGET = ispprog
SOURCE = $(wildcard *.c)
BUILD_DIR = build

#CONFIG = ispprog
CONFIG = ispprog2

AVRDUDE_PROG := -c butterfly -b 19200 -P /dev/ttyUSB0
#AVRDUDE_PROG := -c dragon_isp -P usb

# ---------------------------------------------------------------------------

ifeq ($(CONFIG), ispprog)
# (7.3728MHz ext. crystal)
AVRDUDE_MCU=m16
AVRDUDE_FUSES=lfuse:w:0xff:m hfuse:w:0xda:m

MCU=atmega16
endif

ifeq ($(CONFIG), ispprog2)
# (8MHz int. osc, 2.7V BOD)
AVRDUDE_MCU=m328p -F
AVRDUDE_FUSES=lfuse:w:0xe2:m hfuse:w:0xdc:m efuse:w:0x02:m

MCU = atmega328p
endif

# ---------------------------------------------------------------------------

CFLAGS = -pipe -g -Os -mmcu=$(MCU) -Wall -fdata-sections -ffunction-sections
CFLAGS += -Wa,-adhlns=$(BUILD_DIR)/$(*D)/$(*F).lst -MMD -MP -MF $(BUILD_DIR)/$(*D)/$(*F).d
CFLAGS += -DCONFIG_$(CONFIG)=1
LDFLAGS = -Wl,-Map,$(@:.elf=.map),--cref,--relax,--gc-sections

# ---------------------------------------------------------------------------

$(TARGET): $(BUILD_DIR)/$(TARGET).elf
	@$(SIZE) -B -x --mcu=$(MCU) $<

$(BUILD_DIR)/$(TARGET).elf: $(patsubst %,$(BUILD_DIR)/%,$(SOURCE:.c=.o))
	@echo " Linking file:  $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
	@$(OBJDUMP) -h -S $@ > $(@:.elf=.lss)
	@$(OBJCOPY) -j .text -j .data -O ihex $@ $(@:.elf=.hex)
	@$(OBJCOPY) -j .text -j .data -O binary $@ $(@:.elf=.bin)

$(BUILD_DIR)/%.o: %.c $(MAKEFILE_LIST)
	@echo " Building file: $<"
	@$(shell test -d $(BUILD_DIR)/$(*D) || mkdir -p $(BUILD_DIR)/$(*D))
	@$(CC) $(CFLAGS) -o $@ -c $<

include $(shell find $(BUILD_DIR) -name \*.d 2> /dev/null)

clean:
	rm -rf $(BUILD_DIR)

install: $(BUILD_DIR)/$(TARGET).elf
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) -U flash:w:$(<:.elf=.hex)

fuses:
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) $(patsubst %,-U %, $(AVRDUDE_FUSES))
