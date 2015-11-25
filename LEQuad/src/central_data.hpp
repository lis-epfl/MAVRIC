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
 * \file central_data.h
 * 
 * \author MAV'RIC Team
 *   
 * \brief Place where the central data is stored and initialized
 *
 ******************************************************************************/


#ifndef CENTRAL_DATA_H_
#define CENTRAL_DATA_H_

#include <stdbool.h>
#include <stdint.h>


#include "imu.hpp"
#include "gps.hpp"
#include "sonar.hpp"
#include "file.hpp"

#include "stabilisation_copter.hpp"
#include "mavlink_communication.hpp"
#include "onboard_parameters.hpp"
#include "mavlink_waypoint_handler.hpp"
#include "navigation.hpp"
#include "hud_telemetry.hpp"
#include "state_machine.hpp"
#include "data_logging.hpp"
#include "qfilter.hpp"
#include "mavlink_stream.hpp"
#include "simulation.hpp"
#include "position_estimation.hpp"
#include "state.hpp"
#include "manual_control.hpp"
#include "battery.hpp"



#include "fat_fs_mounting.hpp"


extern "C" 
{
	#include "time_keeper.h"
	#include "ahrs.h"
	#include "pid_controller.h"
	#include "print_util.h"
	#include "coord_conventions.h"
	#include "stabilisation.h"
	#include "servos_mix_quadcopter_diag.h"


	#include "sd_spi.h"
}

/**
 * \brief The central data structure
 */
class Central_data
{
public:
	/**
	 * \brief   Constructor
	 */
	Central_data(Imu& imu, Barometer& barometer, Gps& gps, Sonar& sonar, Serial& serial_mavlink, Satellite& satellite, File& file_flash, Battery& battery, servos_t& servos);


	/**
	 * \brief   Initialisation
	 * \return [description]
	 */
	bool init(void);


	/**
	 * Public members
	 */	
	Imu& 			imu;				///< Reference to IMU
	Barometer&		barometer;			///< Reference to barometer
	Gps& 			gps;				///< Reference to GPS
	Sonar& 			sonar;				///< Reference to sonar
	Serial&			serial_mavlink;		///< Reference to telemetry serial
	Satellite&		satellite;			///< Reference to remote control satellite
	File& 			file_flash;			///< Reference to flash storage
	Battery& 		battery;			///< Reference to battery
	servos_t& 		servos;				///< Reference to servos structure

	scheduler_t	scheduler;
	mavlink_communication_t mavlink_communication;
	
	command_t command;
	servos_mix_quadcotper_diag_t servo_mix;


	qfilter_t attitude_filter;									///< The qfilter structure
	ahrs_t ahrs;												///< The attitude estimation structure

	control_command_t controls;									///< The control structure used for rate and attitude modes
	control_command_t controls_nav;								///< The control nav structure used for velocity modes

	manual_control_t manual_control;							///< The joystick parsing structure
	
	stabilisation_copter_t stabilisation_copter;				///< The stabilisation structure for copter
	
	position_estimation_t position_estimation;					///< The position estimaton structure
	mavlink_waypoint_handler_t waypoint_handler;
	navigation_t navigation;									///< The structure to perform GPS navigation
	
	State state;												///< The structure with all state information
	state_machine_t state_machine;								///< The structure for the state machine
		
	hud_telemetry_structure_t hud_structure;					///< The HUD structure
	
	sd_spi_t sd_spi;											///< The sd_SPI driver structure	
	data_logging_t data_logging;								///< The log data structure	
	fat_fs_mounting_t fat_fs_mounting; 							///< The Fat fs system file mounting structure
};


#endif /* CENTRAL_DATA_H_ */
