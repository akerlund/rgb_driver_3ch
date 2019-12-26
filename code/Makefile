#Project settings
PROJECT_NAME = frame_led
SOURCES = main.c
BUILD_DIR = build/

OBJECTS = $(SOURCES:%.c=$(BUILD_DIR)%.o)
TARGET_ELF = $(BUILD_DIR)$(PROJECT_NAME).elf
TARGET_BIN = $(TARGET_ELF:%.elf=%.bin)

#Toolchain settings
TOOLCHAIN = arm-none-eabi

CC = $(TOOLCHAIN)-gcc
OBJCOPY = $(TOOLCHAIN)-objcopy
OBJDUMP = $(TOOLCHAIN)-objdump
SIZE = $(TOOLCHAIN)-size

#Target CPU options
CPU_DEFINES = -mthumb -mcpu=cortex-m3 -msoft-float -DSTM32F1

#Compiler options
CFLAGS += -g -c -std=gnu99 -Os -Wall -fno-common -ffunction-sections -w
CFLAGS += -fdata-sections 
CFLAGS += $(CPU_DEFINES)

INCLUDE_PATHS += -Ilib/libopencm3/include -Iinc

LINK_SCRIPT = stm32f100x6.ld

LINK_FLAGS = --static -Llib/libopencm3/lib 
LINK_FLAGS += -Llib/libopencm3/lib/stm32/f1 
LINK_FLAGS += -T$(LINK_SCRIPT) -lopencm3_stm32f1
LINK_FLAGS += -Wl,--gc-sections -nostartfiles -nodefaultlibs -nostdlib

LIBS = libopencm3_stm32f1.a

#Not used for now but we should add it
DEBUG_FLAGS = -g   

#Directories
vpath %.c src
vpath %.o $(BUILD_DIR)
vpath %.ld lib/libopencm3/lib/stm32/f1
vpath %.a lib/libopencm3/lib

default: $(TARGET_BIN)

$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) -O binary $(TARGET_ELF) $(TARGET_BIN)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET_ELF): $(BUILD_DIR) $(LIBS) $(OBJECTS) $(LINK_SCRIPT)
	$(CC) $(OBJECTS) $(LINK_FLAGS) -o $(TARGET_ELF)

$(OBJECTS): $(BUILD_DIR)%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDE_PATHS) $^ -o $@

$(LINK_SCRIPT): libopencm3_stm32f1.a

libopencm3_stm32f1.a: lib/libopencm3/.git
	cd lib/libopencm3; $(MAKE) lib/stm32/f1

lib/libopencm3/.git:
	cd lib/libopencm3; git submodule init
	cd lib/libopencm3; git submodule update

clean:
	rm -f $(OBJECTS) $(TARGET_ELF) $(TARGET_BIN)

deep-clean: clean
	cd lib/libopencm3; $(MAKE) clean

	

.PHONY: default clean deep-clean libopencm3

#makefile_debug:
