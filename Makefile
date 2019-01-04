BUILDDIR = build
SRCDIR = src
INCDIR = inc
TARGET = main

# Define the linker script location and chip architecture
LD_SCRIPT = LinkerScript.ld
MCU_SPEC = cortex-m4

# Toolchain definitions (ARM bare metal defaults)
TOOLCHAIN = /usr
CC = $(TOOLCHAIN)/bin/arm-none-eabi-gcc
AS = $(TOOLCHAIN)/bin/arm-none-eabi-as
LD = $(TOOLCHAIN)/bin/arm-none-eabi-ld
OC = $(TOOLCHAIN)/bin/arm-none-eabi-objcopy
OD = $(TOOLCHAIN)/bin/arm-none-eabi-objdump
OS = $(TOOLCHAIN)/bin/arm-none-eabi-size

# Assembly directives
ASFLAGS += -c
ASFLAGS += -O0
ASFLAGS += -mcpu=$(MCU_SPEC)
ASFLAGS += -mthumb
ASFLAGS += -Wall
# Set error messages to appear on a single line
ASFLAGS += -fmessage-length=0

# C compilation directives
CFLAGS += -mcpu=$(MCU_SPEC)
CFLAGS += -mthumb
CFLAGS += -Wall
CFLAGS += -g
# Set error messages to appear on a single line
CFLAGS += -fmessage-length=0
# Set system to ignore semihosted junk
CFLAGS += --specs=nosys.specs
CFLAGS += -I$(INCDIR)

# Linker directives
LSCRIPT = ./$(LD_SCRIPT)
LFLAGS += -mcpu=$(MCU_SPEC)
LFLAGS += -mthumb
LFLAGS += -Wall
LFLAGS += --specs=nosys.specs
LFLAGS += -nostdlib
LFLAGS += -lgcc
LFLAGS += -T$(LSCRIPT)

VPATH = $(SRCDIR)

C_SRC 	= ./main.c
C_SRC	+= ./system_stm32f4xx.c
AS_SRC 	= ./startup_stm32.s

OBJS =  $(C_SRC:.c=.o)
OBJS += $(AS_SRC:.s=.o)

.PHONY: all
all: $(TARGET).bin

%.o: %.s
	mkdir -p $(BUILDDIR)
	$(CC) -x assembler-with-cpp $(ASFLAGS) $< -o $(BUILDDIR)/$@

%.o: %.c
	mkdir -p $(BUILDDIR)
	$(CC) -c $(CFLAGS) $(INLCUDE) $< -o $(BUILDDIR)/$@

$(TARGET).elf: $(OBJS)
	$(CC) $(addprefix $(BUILDDIR)/,$^) $(LFLAGS) -o $(BUILDDIR)/$@

$(TARGET).bin: $(TARGET).elf
	$(OC) -S -O binary $(BUILDDIR)/$< $(BUILDDIR)/$@
	$(OS) $(BUILDDIR)/$<

.PHONY: clean
clean:
	rm -rf $(BUILDDIR)





