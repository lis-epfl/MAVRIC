# Binaries will be generated with this name
PROJ_NAME=LEQuad

# Put your MAVRIC_Library code directory here
MAVRIC_LIB=../../MAVRIC_Library

# Put your project source folder
MAVRIC_SRC=../src/

# Put your source files here (or *.c, etc)

#SRCS  = $(MAVRIC_SRC)/main.cpp

SRCS += $(MAVRIC_LIB)/util/buffer.c
SRCS += $(MAVRIC_LIB)/util/coord_conventions.c
SRCS += $(MAVRIC_LIB)/util/kalman.c
SRCS += $(MAVRIC_LIB)/util/linear_algebra.c
SRCS += $(MAVRIC_LIB)/util/print_util.c
SRCS += $(MAVRIC_LIB)/util/quick_trig.c

SRCS += $(MAVRIC_LIB)/hal/emu/adxl345_driver.c
SRCS += $(MAVRIC_LIB)/hal/emu/bmp085.c
SRCS += $(MAVRIC_LIB)/hal/emu/itg3200.c
SRCS += $(MAVRIC_LIB)/hal/emu/lsm330dlc.c
SRCS += $(MAVRIC_LIB)/hal/emu/radar_module_driver.c
SRCS += $(MAVRIC_LIB)/hal/emu/time_keeper.c
SRCS += $(MAVRIC_LIB)/hal/emu/analog_monitor.c
SRCS += $(MAVRIC_LIB)/hal/emu/hmc5883l.c
SRCS += $(MAVRIC_LIB)/hal/emu/spektrum_satellite.c
SRCS += $(MAVRIC_LIB)/hal/emu/led.c
SRCS += $(MAVRIC_LIB)/hal/emu/piezo_speaker.c
SRCS += $(MAVRIC_LIB)/hal/emu/udp_stream.c

# SRCS += $(MAVRIC_SRC)/main.cpp

# Normally you shouldn't need to change anything below this line!
#######################################################################################

CC=gcc
OBJCOPY=objcopy

CFLAGS  = -g -O2 -Wall  -std=c11
# CFLAGS += -ansi -pedantic
CFLAGS += -fdiagnostics-color=always
# CFLAGS += -D_BSD_SOURCE
CFLAGS += -D_DEFAULT_SOURCE

# Include files from MAVRIC library and source folder
CFLAGS += -I.
CFLAGS += -I$(MAVRIC_LIB)/
CFLAGS += -I$(MAVRIC_LIB)/libs/mavlink/include/common/
CFLAGS += -I$(MAVRIC_LIB)/libs/mavlink/include/mavric/
CFLAGS += -I$(MAVRIC_LIB)/libs/
CFLAGS += -I$(MAVRIC_LIB)/util/
CFLAGS += -I$(MAVRIC_LIB)/hal/include
CFLAGS += -I$(MAVRIC_LIB)/hal/emu/include
CFLAGS += -I$(MAVRIC_LIB)/control/
CFLAGS += -I$(MAVRIC_LIB)/sensing/
CFLAGS += -I$(MAVRIC_LIB)/runtime/
CFLAGS += -I$(MAVRIC_LIB)/communication/
CFLAGS += -I$(MAVRIC_SRC)/

# add startup file to build 
OBJS = $(SRCS:.c=.o)


.PHONY: clean proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin

# Flash the STM32F4
burn: proj
	st-flash write $(PROJ_NAME).bin 0x8000000
