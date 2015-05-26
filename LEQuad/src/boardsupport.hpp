/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file boardsupport.h
 * 
 * \author MAV'RIC Team
 *   
 * \brief Initialization of all hardware related elements (communication lines, 
 * sensors devices, etc)
 *
 ******************************************************************************/


#ifndef BOARDSUPPORT_H_
#define BOARDSUPPORT_H_

#include "central_data.hpp"
#include "sonar_i2cxl.hpp"
#include "hmc5883l.hpp"
#include "gps_ublox.hpp"

extern "C" {
	#include "uart_int.h"
	#include "sysclk.h"
	#include "sleepmgr.h"
	#include "led.h"
	#include "delay.h"
	#include "user_board/user_board.h"
	#include "buffer.h"

	#include "time_keeper.h"
	#include "i2c_driver_int.h"
	#include "print_util.h"
	// #include "mavlink_stream.h"
	//#include "simulation.h"
	#include "bmp085.h"
	#include "lsm330dlc.h"
	#include "analog_monitor.h"
	#include "piezo_speaker.h"
	#include "gpio.h"

	#include "xbee.h"
	#include "console.h"
	#include "stdio_usb.h"

	#include "pwm_servos.h"
	#include "spektrum_satellite.h"
	#include "conf_platform.h"

	#include "analog_monitor_default_config.h"
	#include "twim_default_config.h"
	#include "uart_default_config.h"
	#include "usb_default_config.h"
}

#define BOARD USER_BOARD

/**
 * \brief	Initialize the hardware related elements (communication lines, sensors devices, etc)
 *
 * \param	central_data		The pointer to the structure where all central data is stored
 *
 * \return	The initialization status of each module, succeed == true
 */
bool boardsupport_init(Central_data* p_central_data);



#endif /* BOARDSUPPORT_H_ */