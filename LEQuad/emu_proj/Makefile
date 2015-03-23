################################################################################
# MAVRIC MAKEFILE
#
# Configure only the first part of this makefile
################################################################################

# Binaries will be generated with this name
PROJ_NAME=LEQuad

# ------------------------------------------------------------------------------
# PROJECT FOLDER
# ------------------------------------------------------------------------------
# Project source folder
MAVRIC_SRC=../src/

# Include paths for project folder
SRCS_INC += -I${MAVRIC_SRC}
SRCS_INC += -I${MAVRIC_SRC}/config/
SRCS_INC += -I${MAVRIC_SRC}/asf/common/user_board/

# Sources files for project folder (*.c and *.cpp)
SRCS += main.cpp
SRCS += central_data.c
SRCS += boardsupport.c


# ------------------------------------------------------------------------------
# MAVRIC LIBRARY
# ------------------------------------------------------------------------------
# MAVRIC_Library code directory
MAVRIC_LIB=../../MAVRIC_Library/

# Include folders for Library
LIB_INC += -I$(MAVRIC_LIB)/
LIB_INC += -I$(MAVRIC_LIB)/libs/mavlink/include/common/
LIB_INC += -I$(MAVRIC_LIB)/libs/mavlink/include/mavric/
LIB_INC += -I$(MAVRIC_LIB)/libs/
LIB_INC += -I$(MAVRIC_LIB)/util/
LIB_INC += -I$(MAVRIC_LIB)/hal/include
LIB_INC += -I$(MAVRIC_LIB)/hal/emu/include
LIB_INC += -I$(MAVRIC_LIB)/control/
LIB_INC += -I$(MAVRIC_LIB)/sensing/
LIB_INC += -I$(MAVRIC_LIB)/runtime/
LIB_INC += -I$(MAVRIC_LIB)/communication/
LIB_INC += -I$(MAVRIC_SRC)/

# Library source files (*.c and *.cpp)
LIB_SRCS += util/buffer.c
LIB_SRCS += util/coord_conventions.c
LIB_SRCS += util/kalman.c
LIB_SRCS += util/linear_algebra.c
LIB_SRCS += util/print_util.c
LIB_SRCS += util/quick_trig.c

LIB_SRCS += hal/emu/adxl345_driver.c
LIB_SRCS += hal/emu/analog_monitor.c
LIB_SRCS += hal/emu/bmp085.c
LIB_SRCS += hal/emu/hmc5883l.c
LIB_SRCS += hal/emu/itg3200.c
LIB_SRCS += hal/emu/led.c
LIB_SRCS += hal/emu/lsm330dlc.c
LIB_SRCS += hal/emu/piezo_speaker.c
LIB_SRCS += hal/emu/radar_module_driver.c
LIB_SRCS += hal/emu/pwm_servos.c
LIB_SRCS += hal/emu/spektrum_satellite.c
LIB_SRCS += hal/emu/time_keeper.c
LIB_SRCS += hal/emu/udp_stream.c


# ------------------------------------------------------------------------------
# C COMPILER OPTIONS
# ------------------------------------------------------------------------------
CC = gcc
OBJCOPY = objcopy

CFLAGS  = -g -O2 -Wall -std=c11
CFLAGS += -fdiagnostics-color=auto
CFLAGS += -D_DEFAULT_SOURCE

# Include files from MAVRIC library and source folder
CFLAGS += -I.
CFLAGS += ${LIB_INC}
CFLAGS += ${SRCS_INC}


# ------------------------------------------------------------------------------
# C++ COMPILER OPTIONS
# ------------------------------------------------------------------------------
CXX = g++
OBJCOPY = objcopy

CXXFLAGS  = -g -O2 -Wall -std=c++11
CXXFLAGS += -fdiagnostics-color=auto
CXXFLAGS += -D_DEFAULT_SOURCE

# Include files from MAVRIC library and source folder
CXXFLAGS += -I.
CXXFLAGS += ${LIB_INC}
CXXFLAGS += ${SRCS_INC}

################################################################################
# Normally you shouldn't need to change anything below this line!
################################################################################

# ------------------------------------------------------------------------------
# OBJECT FILES
# ------------------------------------------------------------------------------
BUILD_DIR = build

# Get the names of the .o files from .c and .cpp files
OBJS += $(addprefix ${BUILD_DIR}/, $(addsuffix .o, $(basename $(LIB_SRCS))))
OBJS += $(addprefix ${BUILD_DIR}/, $(addsuffix .o, $(basename $(SRCS))))


# ------------------------------------------------------------------------------
# COMMANDS FOR FANCY OUTPUT
# ------------------------------------------------------------------------------
NO_COLOR=\033[0m
OK_COLOR=\033[32;01m
ERROR_COLOR=\033[31;01m
WARN_COLOR=\033[33;01m
 
OK_STRING=$(OK_COLOR)[OK]$(NO_COLOR)
ERROR_STRING=$(ERROR_COLOR)[ERRORS]$(NO_COLOR)
WARN_STRING=$(WARN_COLOR)[WARNINGS]$(NO_COLOR)
 
AWK_CMD = awk '{ printf "%-30s %-10s\n",$$1, $$2; }'
PRINT_ERROR = printf "$@ $(ERROR_STRING)\n" | $(AWK_CMD) && printf "$(CMD)\n$$LOG\n" && false
PRINT_WARNING = printf "$@ $(WARN_STRING)\n" | $(AWK_CMD) && printf "$(CMD)\n$$LOG\n"
PRINT_OK = printf "$@ $(OK_STRING)\n" | $(AWK_CMD)
BUILD_CMD = LOG=$$($(CMD) 2>&1) ; if [ $$? -eq 1 ]; then $(PRINT_ERROR); elif [ "$$LOG" != "" ] ; then $(PRINT_WARNING); else $(PRINT_OK); fi;


# ------------------------------------------------------------------------------
# MAKEFILE RULES
# ------------------------------------------------------------------------------

# Main rule
all: proj
proj: $(PROJ_NAME).elf

# Linking
${PROJ_NAME}.elf: ${OBJS}
	@$(CXX) $(LDFLAGS) $^ -o $@ 
	@$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	@$(BUILD_CMD)
	
# C files in Library
${BUILD_DIR}/%.o: ${MAVRIC_LIB}/%.c
	@mkdir -p $(dir $@)
	@$(CC) -c $< -o $@ $(CFLAGS)
	@$(BUILD_CMD)

# CPP files in Library
${BUILD_DIR}/%.o: ${MAVRIC_LIB}/%.cpp
	@mkdir -p $(dir $@)
	@$(CXX) -c $< -o $@ $(CXXFLAGS)
	@$(BUILD_CMD)

# C files in source folder
${BUILD_DIR}/%.o: ${MAVRIC_SRC}/%.c
	@mkdir -p $(dir $@)
	@$(CC) -c $< -o $@ $(CFLAGS)
	@$(BUILD_CMD)

# CPP files in SRC folder
${BUILD_DIR}/%.o: ${MAVRIC_SRC}/%.cpp
	@mkdir -p $(dir $@)
	@$(CXX) -c $< -o $@ $(CXXFLAGS)
	@$(BUILD_CMD)

.PHONY: clean
clean:
	@rm -f $(OBJS) $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin
	@$(PRINT_OK)