# example disassemble
# arm-none-eabi-objdump -dS stm32f4-rom-emulator.elf >asm.out

# Put your stlink folder here so make burn will work.
STLINK=~/stlink.git

#SRCS=main.c system_stm32f4xx.c stm32f4xx_it.c
#SRCS=main.c system_stm32f4xx.c working-stm32f4xx_it.c

#
#
SRCS=diskio.c  ff.c main.c stm32f4_discovery.c  stm32f4_discovery_sdio_sd.c system_stm32f4xx.c misc_handlers.c

# Library modules
SRCS += stm32f4xx_syscfg.c misc.c stm32f4xx_gpio.c stm32f4xx_rcc.c stm32f4xx_usart.c stm32f4xx_sdio.c stm32f4xx_dma.c stm32f4xx_exti.c stm32f4xx_pwr.c 
#SRCS += stm32f4xx_tim.c 
#SRCS += stm32f4_discovery.c

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=amstradcpc-rom-emulator

#######################################################################################

#STM_COMMON=../../..
# You need libs somewhere!
STM_COMMON=../STM32F4-Discovery_FW_V1.1.0

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld 
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I.
#CFLAGS += --specs=nosys.specs
CFLAGS += -specs=nano.specs -specs=rdimon.specs -lc -lrdimon


#CFLAGS += -DFDC_CMD_LOGGING
#CFLAGS += -DFDC_STATUS_LOGGING
#CFLAGS += -DFDC_READDATA_LOGGING
#CFLAGS += -DFDC_LOGGING
#CFLAGS += -DENABLE_SEMIHOSTING
#CFLAGS += -DENABLE_SEMIHOSTING_SEEK
#CFLAGS += -DENABLE_SEMIHOSTING_CHANGE_DISK
#CFLAGS += -DENABLE_SEMIHOSTING_SECTOR_TABLE
#CFLAGS += -DENABLE_SEMIHOSTING_DISK_INFO
#CFLAGS += -DENABLE_SEMIHOSTING_SAVE_TRACK
#CFLAGS += -DDBGIO
#CFLAGS += -DDEBUG_MAIN
#CFLAGS += -DDEBUG_ROMEN
#CFLAGS += -DDEBUG_IORQ
#CFLAGS += -DDEBUG_EXTI1
#CFLAGS += -DDEBUG_EXTI4
#CFLAGS += -DDEBUG_SPURIOUS_EVENT_MREQ
#CFLAGS += -DDISABLE_ALL_BUT_SHOW_MAIN_THREAD_ACTIVITY

# Include files from STM libraries
CFLAGS += -I$(STM_COMMON)/Utilities/STM32F4-Discovery
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/Include 
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Include
CFLAGS += -I$(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/inc


# add startup file to build
SRCS += startup_stm32f4xx.s 
# You need to end asm files in capital S to get them to see preprocessor directives
SRCS += interrupt.S

OBJS = $(SRCS:.c=.o)

vpath %.c $(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src $(STM_COMMON)/Utilities/STM32F4-Discovery

.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin


# Flash the STM32F4
#burn: proj
#	$(STLINK)/st-flash write $(PROJ_NAME).bin 0x8000000
