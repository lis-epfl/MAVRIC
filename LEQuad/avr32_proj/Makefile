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
MAVRIC_SRC=../src

# Include paths for project folder
SRCS_INC += -I${MAVRIC_SRC}/
SRCS_INC += -I${MAVRIC_SRC}/config/
SRCS_INC += -I${MAVRIC_SRC}/asf
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/include
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/programs
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/programs/data_extract
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/programs/data_extract/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/programs/data_get
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/programs/data_get/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/programs/adpcm_encode
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/programs/adpcm_encode/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/programs/adpcm_streaming
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/programs/adpcm_streaming/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/programs/data_print
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/programs/data_print/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/print_complex_vect
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/print_complex_vect/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/serial_scope
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/serial_scope/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/resampling_coefficients_generation
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/resampling_coefficients_generation/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/benchmark
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/benchmark/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/benchmark/resources
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/print_real_vect
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/print_real_vect/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/adpcm_encoder
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/adpcm_encoder/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/twiddle_factors_generator
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/utils/scripts/twiddle_factors_generator/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/at32uc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/at32uc/docsrc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/at32uc/basic
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/at32uc/basic/transforms
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/at32uc/basic/filters
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/at32uc/basic/operators
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/at32uc/basic/vectors
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/generic
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/generic/advanced
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/generic/advanced/resampling
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/generic/advanced/adpcm
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/generic/basic
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/generic/basic/transforms
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/generic/basic/filters
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/generic/basic/operators
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/generic/basic/windowing
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/generic/basic/vectors
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/dsp/dsplib/generic/basic/signal_generation
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/services/delay
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/utils
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/utils/startup
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/utils/libs
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/utils/libs/dsplib
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/utils/libs/dsplib/include
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/utils/libs/dsplib/at32ucr3fp
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/utils/libs/dsplib/at32ucr3fp/gcc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/utils/preprocessor
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/pm
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/adcifa
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/usart
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/twim
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/intc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/ast
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/pevc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/tc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/spi
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/usbc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/pdca
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/cpu
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/cpu/cycle_counter
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/eic
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/dacifb
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/pwm
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/flashc
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/gpio
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/twis
SRCS_INC += -I${MAVRIC_SRC}/asf/avr32/drivers/scif
SRCS_INC += -I${MAVRIC_SRC}/asf/common
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/usb
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/usb/udc
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/usb/class
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/usb/class/cdc
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/usb/class/cdc/device
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/clock
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/clock/uc3c
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/serial
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/serial/uc3_usart
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/spi
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/spi/uc3_spi
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/twi
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/twi/uc3_twim
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/sleepmgr
SRCS_INC += -I${MAVRIC_SRC}/asf/common/services/sleepmgr/uc3
SRCS_INC += -I${MAVRIC_SRC}/asf/common/utils
SRCS_INC += -I${MAVRIC_SRC}/asf/common/utils/stdio
SRCS_INC += -I${MAVRIC_SRC}/asf/common/utils/stdio/stdio_usb
SRCS_INC += -I${MAVRIC_SRC}/asf/common/utils/stdio/stdio_serial
SRCS_INC += -I${MAVRIC_SRC}/asf/common/utils/interrupt
SRCS_INC += -I${MAVRIC_SRC}/asf/common/boards
SRCS_INC += -I${MAVRIC_SRC}/asf/common/boards/user_board

# Sources files for project folder (*.c and *.cpp)
SRCS += main.cpp
SRCS += central_data.c
SRCS += boardsupport.c
SRCS += mavlink_telemetry.c
SRCS += tasks.c

SRCS += asf/avr32/drivers/intc/exception.S
# SRCS += exception.S
SRCS += asf/avr32/utils/startup/startup_uc3.S
SRCS += asf/avr32/utils/startup/trampoline_uc3.S
SRCS += asf/avr32/drivers/adcifa/adcifa.c
SRCS += asf/avr32/drivers/ast/ast.c
SRCS += asf/avr32/drivers/dacifb/dacifb.c
SRCS += asf/avr32/drivers/eic/eic.c
SRCS += asf/avr32/drivers/flashc/flashc.c
SRCS += asf/avr32/drivers/gpio/gpio.c
SRCS += asf/avr32/drivers/intc/intc.c
SRCS += asf/avr32/drivers/pdca/pdca.c
SRCS += asf/avr32/drivers/pevc/pevc.c
SRCS += asf/avr32/drivers/pm/pm_uc3c.c  
SRCS += asf/avr32/drivers/pm/power_clocks_lib.c
SRCS += asf/avr32/drivers/pwm/pwm4.c
SRCS += asf/avr32/drivers/scif/scif_uc3c.c
SRCS += asf/avr32/drivers/spi/spi.c
SRCS += asf/avr32/drivers/tc/tc.c
SRCS += asf/avr32/drivers/twim/twim.c
SRCS += asf/avr32/drivers/usart/usart.c
SRCS += asf/avr32/drivers/usbc/usbc_device.c
SRCS += asf/avr32/services/delay/delay.c
SRCS += asf/avr32/utils/startup/startup_uc3.S
SRCS += asf/common/boards/user_board/init.c
SRCS += asf/common/services/clock/uc3c/osc.c
SRCS += asf/common/services/clock/uc3c/pll.c
SRCS += asf/common/services/clock/uc3c/sysclk.c
SRCS += asf/common/services/sleepmgr/uc3/sleepmgr.c
SRCS += asf/common/services/spi/uc3_spi/spi_master.c
SRCS += asf/common/services/usb/class/cdc/device/udi_cdc.c
SRCS += asf/common/services/usb/class/cdc/device/udi_cdc_desc.c
SRCS += asf/common/services/usb/udc/udc.c
SRCS += asf/common/utils/stdio/read.c
SRCS += asf/common/utils/stdio/write.c
SRCS += asf/common/utils/stdio/stdio_usb/stdio_usb.c

# ------------------------------------------------------------------------------
# MAVRIC LIBRARY
# ------------------------------------------------------------------------------
# MAVRIC_Library code directory
MAVRIC_LIB=../../MAVRIC_Library

# Include folders for Library
LIB_INC += -I$(MAVRIC_LIB)/
LIB_INC += -I$(MAVRIC_LIB)/libs/mavlink/include/common/
LIB_INC += -I$(MAVRIC_LIB)/libs/mavlink/include/mavric/
LIB_INC += -I$(MAVRIC_LIB)/libs/fats/
LIB_INC += -I$(MAVRIC_LIB)/libs/
LIB_INC += -I$(MAVRIC_LIB)/util/
LIB_INC += -I$(MAVRIC_LIB)/hal/include
LIB_INC += -I$(MAVRIC_LIB)/hal/avr32/include
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
LIB_SRCS += util/sma.c

LIB_SRCS += hal/avr32/adc_int.c                   
LIB_SRCS += hal/avr32/ads1274.c                   
LIB_SRCS += hal/avr32/adxl345_driver.c                   
LIB_SRCS += hal/avr32/airspeed_analog.c           
LIB_SRCS += hal/avr32/analog_monitor.c            
LIB_SRCS += hal/avr32/analog_monitor_telemetry.c  
LIB_SRCS += hal/avr32/battery.c                    
LIB_SRCS += hal/avr32/bmp085.c                    
LIB_SRCS += hal/avr32/bmp085_telemetry.c          
LIB_SRCS += hal/avr32/curvace.c                   
LIB_SRCS += hal/avr32/dac_dma.c                   
LIB_SRCS += hal/avr32/fat_fs_mounting.c
LIB_SRCS += hal/avr32/fat_fs_mounting_telemetry.c
LIB_SRCS += hal/avr32/gps_ublox.c                 
LIB_SRCS += hal/avr32/gps_ublox_telemetry.c       
LIB_SRCS += hal/avr32/hmc5883l.c             
LIB_SRCS += hal/avr32/i2c_driver_int.c       
LIB_SRCS += hal/avr32/itg3200.c              
LIB_SRCS += hal/avr32/led.c                  
LIB_SRCS += hal/avr32/lsm330dlc.c            
LIB_SRCS += hal/avr32/piezo_speaker.c        
LIB_SRCS += hal/avr32/pwm_servos.c           
LIB_SRCS += hal/avr32/radar_driver.c         
LIB_SRCS += hal/avr32/radar_module_driver.c
LIB_SRCS += hal/avr32/sd_spi.c
LIB_SRCS += hal/avr32/servos.c                              
LIB_SRCS += hal/avr32/servos_telemetry.c
LIB_SRCS += hal/avr32/sonar_i2cxl.c
LIB_SRCS += hal/avr32/sonar_telemetry.c
LIB_SRCS += hal/avr32/spektrum_satellite.c
LIB_SRCS += hal/avr32/spi_buffered.c
LIB_SRCS += hal/avr32/time_keeper.c
LIB_SRCS += hal/avr32/uart_int.c
LIB_SRCS += hal/avr32/usb_int.c
LIB_SRCS += hal/avr32/xbee.c

LIB_SRCS += control/adaptive_parameter.c          
LIB_SRCS += control/attitude_controller.c         
LIB_SRCS += control/attitude_controller_p2.c      
LIB_SRCS += control/attitude_error_estimator.c    
LIB_SRCS += control/joystick_parsing.c            
LIB_SRCS += control/joystick_parsing_telemetry.c  
LIB_SRCS += control/navigation.c                   
LIB_SRCS += control/pid_controller.c               
LIB_SRCS += control/servos_mix_quadcopter_cross.c  
LIB_SRCS += control/servos_mix_quadcopter_diag.c   
LIB_SRCS += control/stabilisation.c
LIB_SRCS += control/stabilisation_copter.c
LIB_SRCS += control/stabilisation_copter_default_config.c
LIB_SRCS += control/stabilisation_telemetry.c
LIB_SRCS += control/vector_field_waypoint.c
LIB_SRCS += control/velocity_controller_copter.c

LIB_SRCS += sensing/ahrs.c            
LIB_SRCS += sensing/ahrs_telemetry.c  
LIB_SRCS += sensing/imu.c             
LIB_SRCS += sensing/imu_telemetry.c                  
LIB_SRCS += sensing/position_estimation.c            
LIB_SRCS += sensing/position_estimation_telemetry.c
LIB_SRCS += sensing/qfilter.c
LIB_SRCS += sensing/simulation.c
LIB_SRCS += sensing/simulation_telemetry.c   
LIB_SRCS += sensing/launch_detection.c

LIB_SRCS += runtime/scheduler.c  
LIB_SRCS += runtime/scheduler_telemetry.c

LIB_SRCS += communication/acoustic.c                      
LIB_SRCS += communication/acoustic_telemetry.c      
LIB_SRCS += communication/console.c                 
LIB_SRCS += communication/data_logging.c            
LIB_SRCS += communication/hud_telemetry.c           
LIB_SRCS += communication/mavlink_communication.c     
LIB_SRCS += communication/mavlink_message_handler.c   
LIB_SRCS += communication/mavlink_stream.c            
LIB_SRCS += communication/mavlink_waypoint_handler.c  
LIB_SRCS += communication/onboard_parameters.c
LIB_SRCS += communication/remote.c
LIB_SRCS += communication/remote_telemetry.c
LIB_SRCS += communication/state.c
LIB_SRCS += communication/state_machine.c
LIB_SRCS += communication/state_machine_custom.c
LIB_SRCS += communication/state_telemetry.c

LIB_SRCS += libs/fat_fs/diskio.c  
LIB_SRCS += libs/fat_fs/ff.c  
LIB_SRCS += libs/fat_fs/option/ccsbcs.c
LIB_SRCS += libs/fat_fs/option/syscall.c
LIB_SRCS += libs/fat_fs/option/unicode.c


# ------------------------------------------------------------------------------
# PREPROCESSOR OPTIONS
# ------------------------------------------------------------------------------
PREPROC_OPT  = -DBOARD=USER_BOARD 
PREPROC_OPT += -DDSP_OPTIMIZATION=DSP_OPTI_SPEED 
PREPROC_OPT += -DDSP_ADPCM
PREPROC_OPT += -DDSP_RESAMPLING 
PREPROC_OPT += -DDSP_FILTERS 
PREPROC_OPT += -DDSP_OPERATORS 
PREPROC_OPT += -DDSP_SIGNAL_GENERATION
PREPROC_OPT += -DDSP_TRANSFORMS 
PREPROC_OPT += -DDSP_VECTORS 
PREPROC_OPT += -DDSP_WINDOWING 
PREPROC_OPT += -DUDD_ENABLE

# ------------------------------------------------------------------------------
# ASSEMBLY COMPILER OPTIONS
# ------------------------------------------------------------------------------
AS = avr32-gcc
ASFLAGS = -x assembler-with-cpp -c -mpart=uc3c1512c -mrelax 

# Include files from MAVRIC library and source folder
ASFLAGS += -I.
ASFLAGS += ${LIB_INC}
ASFLAGS += ${SRCS_INC}
ASFLAGS += -MMD -MP

# ------------------------------------------------------------------------------
# C COMPILER OPTIONS
# ------------------------------------------------------------------------------
CC = avr32-gcc

CFLAGS  = -O3 -mhard-float -fdata-sections -muse-rodata-section -g2 -Wall 
CFLAGS += -mpart=uc3c1512c -c -std=gnu99 -Wstrict-prototypes -Wmissing-prototypes 
CFLAGS += -Werror-implicit-function-declaration -Wpointer-arith -mrelax -MMD -MP
CFLAGS += ${PREPROC_OPT}

# Include files from MAVRIC library and source folder
CFLAGS += -I.
CFLAGS += ${LIB_INC}
CFLAGS += ${SRCS_INC}


# ------------------------------------------------------------------------------
# C++ COMPILER OPTIONS
# ------------------------------------------------------------------------------
CXX = avr32-g++

CXXFLAGS  = -O3 -std=gnu++0x -mhard-float -fdata-sections -muse-rodata-section 
CXXFLAGS += -g2 -Wall -mpart=uc3c1512c -c -Wpointer-arith -mrelax -MMD -MP
# CXXFLAGS += -D_DEFAULT_SOURCE 
CXXFLAGS += ${PREPROC_OPT}

# Include files from MAVRIC library and source folder
CXXFLAGS += -I.
CXXFLAGS += ${LIB_INC}
CXXFLAGS += ${SRCS_INC}

# ------------------------------------------------------------------------------
# LINKER OPTIONS
# ------------------------------------------------------------------------------
LINKER = avr32-g++
OBJCOPY = avr32-objcopy
SIZER = avr32-size 

OBJCOPYFLAGS = -R .eeprom -R .fuse -R .lock -R .signature

LDFLAGS  = -nostartfiles -Wl,-Map="mavric.map" -lm
LDFLAGS += -Wl,--gc-sections -mpart=uc3c1512c -Wl,--relax -Wl,-e,_trampoline

# Include files from MAVRIC library and source folder
LDFLAGS += -I.
LDFLAGS += ${LIB_INC}
LDFLAGS += ${SRCS_INC}


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
# DEPENDENCY FILES (*.d)
# ------------------------------------------------------------------------------
DEPS += $(addsuffix .d, $(basename $(OBJS)))	# create list of dependency files
-include $(DEPS)								# include existing dependency files

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
	@$(SIZER) $^

# Linking
${PROJ_NAME}.elf: ${OBJS}
	@echo Linking...
	@$(CXX) $(LDFLAGS) $^ -o $@  
	@$(OBJCOPY) -O ihex ${OBJCOPYFLAGS} $(PROJ_NAME).elf $(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	@$(BUILD_CMD)

# Assembly files in Library
${BUILD_DIR}/%.o: ${MAVRIC_LIB}/%.S
	@mkdir -p $(dir $@)
	@$(AS) $(ASFLAGS) -c $< -o $@
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

# Assembly files in source folder
${BUILD_DIR}/%.o: ${MAVRIC_SRC}/%.S
	@mkdir -p $(dir $@)
	@$(AS) $(ASFLAGS) -c $< -o $@
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
	@rm -f $(OBJS) $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin $(DEPS)
	@$(PRINT_OK)

.DEFAULT_GOAL := all