# Simple makefile template for STM32 project
#
# - include and list objects in OBJS
# - define TARGET

TARGET = main
OBJS = src/main.o
HAL = 	stm32f4xx_hal.o \
	stm32f4xx_hal_cortex.o \
	stm32f4xx_hal_gpio.o \
	stm32f4xx_hal_rcc.o \
	stm32f4xx_hal_rcc_ex.o \
	stm32f4xx_hal_dma.o \
	stm32f4xx_hal_i2c.o \
	stm32f4xx_hal_i2s.o \
	stm32f4xx_hal_i2s_ex.o \
	stm32f4xx_hal_spi.o \
	stm32f4xx_hal_dac.o \

GRM = \

src/main.o: src/processor.hh src/debug_pins.hh src/leds.hh src/button.hh src/dac_and_mic.hh src/system.hh src/accelerometer.hh

OPTIM ?= 2
TOOLCHAIN_DIR ?= /Applications/ARM/bin/

CXX = $(TOOLCHAIN_DIR)arm-none-eabi-g++
CC = $(TOOLCHAIN_DIR)arm-none-eabi-gcc
OBJCOPY = $(TOOLCHAIN_DIR)arm-none-eabi-objcopy
GDB = $(TOOLCHAIN_DIR)arm-none-eabi-gdb

CMSIS_DIR = lib/CMSIS/
HAL_DIR = lib/HAL/
GRM_DIR = lib/libgrm/

INC =   -I src/ \
	-I . \
	-I lib/DSP/ \
	-I $(CMSIS_DIR) \
	-I $(HAL_DIR) \
	-I $(GRM_DIR) \

LDSCRIPT = $(CMSIS_DIR)/STM32F407VGTx_FLASH.ld

ARCHFLAGS = -mcpu=cortex-m4 \
		-mthumb \
		-mfloat-abi=hard \
		-mfpu=fpv4-sp-d16 \
		-mthumb-interwork \
		-DSTM32F407xx \

CPPFLAGS= $(INC) -std=c++17

CFLAGS= $(ARCHFLAGS) \
	-g \
	-O$(OPTIM) \
	-DUSE_HAL_DRIVER \
	-fdata-sections \
	-ffunction-sections \

CXXFLAGS=$(CFLAGS) \
	-std=c++11 \
	-fno-rtti \
	-fno-exceptions \
	-Wno-psabi \

LDFLAGS= $(ARCHFLAGS) -T $(LDSCRIPT) \
	-Wl,--gc-sections \
	-nostartfiles \
	-nostdlib \

STARTUP = $(CMSIS_DIR)startup_stm32f407xx
SYSTEM = $(CMSIS_DIR)system_stm32f4xx

OBJS += $(STARTUP).o \
	$(SYSTEM).o \
	$(addprefix $(HAL_DIR), $(HAL)) \
	$(addprefix $(GRM_DIR), $(GRM)) \

all: $(TARGET).bin

%.elf: $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.o: %.s
	$(CC) -c -x assembler-with-cpp $(ASFLAGS) $< -o $@

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin

flash: $(TARGET).bin
	openocd -f interface/stlink.cfg \
		-c "adapter speed 4000; transport select hla_swd" \
		-f target/stm32f4x.cfg \
		-c "program main.bin verify reset exit 0x08000000"
	ls -l $(TARGET).bin

erase:
	openocd -f interface/stlink.cfg \
		-c "adapter speed 4000; transport select hla_swd" \
		-f target/stm32f4x.cfg \
		-c "init; halt; stm32f4x mass_erase 0; exit" \

debug:
	$(TOOLCHAIN_DIR)arm-none-eabi-gdb $(TARGET).elf --eval-command="target remote localhost:3333"

.PRECIOUS: $(OBJS) $(TARGET).elf

# File dependencies:
src/main.o: src/leds.hh src/dac.hh src/button.hh src/system.hh \
	    src/debug_pins.hh src/microphone.hh \
	    src/dac_and_mic.hh src/accelerometer.hh \
	    src/parameters.hh src/internal_dac.hh \
