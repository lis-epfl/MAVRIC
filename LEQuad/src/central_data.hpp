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

#include "megafly_rev4.hpp"
#include "sonar_i2cxl.hpp"
#include "stabilisation_copter.hpp"
#include "mavlink_communication.hpp"
#include "onboard_parameters.hpp"
#include "mavlink_waypoint_handler.hpp"
#include "navigation.hpp"
#include "hud_telemetry.hpp"
#include "state_machine.hpp"
#include "data_logging.hpp"
#include "fat_fs_mounting.hpp"
#include "acoustic.hpp" 
#include "qfilter.hpp"
#include "imu.hpp"
#include "mavlink_stream.hpp"
#include "simulation.hpp"
#include "position_estimation.hpp"
#include "state.hpp"
#include "manual_control.hpp"
#include "gps_ublox.hpp"

extern "C" 
{
	#include <stdbool.h>
	#include <stdint.h>

	#include "stdbool.h"

	#include "time_keeper.h"
	#include "ahrs.h"

	#include "pid_controller.h"
	#include "streams.h"
	#include "buffer.h"
	#include "print_util.h"

	#include "coord_conventions.h"

	#include "analog_monitor.h"
	#include "stabilisation.h"

	#include "sd_spi.h"

	#include "attitude_controller_p2.h"
	#include "servos.h"
	#include "pwm_servos.h"
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
	 * @brief   Constructor
	 */
	Central_data(imu_t& imu, I2c& i2c_sonar, Bmp085& baro, Lsm330dlc& gyracc, Hmc5883l& magneto);

	/**
	 * @brief   Initialisation
	 * @return [description]
	 */
	bool init(Serial& uart_mavlink, Barometer& barometer, Satellite& satellite);

	/**
	 * Public members
	 * 
	 */	
	// Megafly_rev4 	board;
	imu_t& 			imu;				///< The IMU structure
	Sonar_i2cxl 	sonar;
	// Barometer& 		barometer;
	Bmp085& 		barometer;			// TODO: use Barometer interface instead
	Lsm330dlc& 		gyroaccelero;		// TODO: use gyro+accelero interface instead
	Hmc5883l&		magnetometer;		// TODO: use magnetometer interface instead

	scheduler_t	scheduler;
	mavlink_communication_t mavlink_communication;
	attitude_controller_p2_t attitude_controller;
	command_t command;
	servo_mix_quadcotper_diag_t servo_mix;
	servos_t servos;

	analog_monitor_t analog_monitor;							///< The analog to digital converter structure

	qfilter_t attitude_filter;									///< The qfilter structure
	ahrs_t ahrs;												///< The attitude estimation structure
	control_command_t controls;									///< The control structure used for rate and attitude modes
	control_command_t controls_nav;								///< The control nav structure used for velocity modes
	control_command_t controls_joystick;						///< The control structure for the joystick

	manual_control_t manual_control;							///< The joystick parsing structure
	
	stabilisation_copter_t stabilisation_copter;					///< The stabilisation structure for copter

	gps_t gps;													///< The GPS structure
	
	audio_t audio_data;
	
	simulation_model_t sim_model;								///< The simulation model structure
	
	position_estimation_t position_estimation;					///< The position estimaton structure
	
	// aliases
	byte_stream_t *telemetry_down_stream;						///< The pointer to the downcoming telemetry byte stream
	byte_stream_t *telemetry_up_stream;							///< The pointer to the upcoming telemetry byte stream
	byte_stream_t *debug_out_stream;							///< The pointer to the outgoing debug byte stream
	byte_stream_t *debug_in_stream;								///< The pointer to the incoming debug byte stream
	
	mavlink_waypoint_handler_t waypoint_handler;
	
	navigation_t navigation;									///< The structure to perform GPS navigation
	
	state_t state;												///< The structure with all state information
	state_machine_t state_machine;								///< The structure for the state machine
	
	//Barometer pressure;											///< The pressure structure
	
	hud_telemetry_structure_t hud_structure;					///< The HUD structure
	
	sd_spi_t sd_spi;											///< The sd_SPI driver structure
	
	data_logging_t data_logging;								///< The log data structure	
	fat_fs_mounting_t fat_fs_mounting; 							///< The Fat fs system file mounting structure
};


#endif /* CENTRAL_DATA_H_ */
