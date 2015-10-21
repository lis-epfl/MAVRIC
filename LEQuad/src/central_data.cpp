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
 * \file central_data.c
 *
 * \author MAV'RIC Team
 *   
 * \brief Place where the central data is stored and initialized
 *
 ******************************************************************************/


#include "central_data.hpp"
#include "stabilisation_copter_default_config.hpp"
#include "data_logging_default_config.hpp"
#include "mavlink_communication_default_config.hpp"
// #include "conf_imu.hpp"
#include "position_estimation_default_config.hpp"
// #include "simulation_default_config.hpp"
#include "remote_default_config.hpp"
#include "state_default_config.hpp"
#include "manual_control_default_config.hpp"

extern "C" 
{
	#include "time_keeper.h"
	#include "navigation_default_config.h"
	#include "qfilter_default_config.h"
	#include "scheduler_default_config.h"
	#include "attitude_controller_p2_default_config.h"
	#include "servos_mix_quadcopter_diag_default_config.h"

	#include "conf_platform.h"
}


Central_data::Central_data(Imu& imu, Barometer& barometer, Gps& gps, Sonar& sonar, Serial& serial_mavlink, Satellite& satellite, File& file_flash, servos_t& servos, analog_monitor_t& analog_monitor):
	imu( imu ),
	barometer( barometer ),
	gps( gps ),
	sonar( sonar ),
	serial_mavlink( serial_mavlink ),
	satellite( satellite ),
	file_flash( file_flash ),
	servos( servos ),
	analog_monitor( analog_monitor )
{}


bool Central_data::init(void)
{
	bool init_success = true;

	// Init main sheduler
	init_success &= scheduler_init(&scheduler, scheduler_default_config());
	
	time_keeper_delay_ms(100); 

	// Init mavlink communication
	mavlink_communication_conf_t mavlink_communication_config = mavlink_communication_default_config();
	mavlink_communication_config.mavlink_stream_config.sysid = MAVLINK_SYS_ID;
	init_success &= mavlink_communication_init(	&mavlink_communication, 
												mavlink_communication_config, 
												&serial_mavlink,
												&state,
												&file_flash );
	
	time_keeper_delay_ms(100); 

	// Init state structure
	init_success &= state_init(	&state,
								state_default_config(),
								&analog_monitor); 
	
	time_keeper_delay_ms(100);

	//Init state_machine	
	init_success &= state_machine_init( &state_machine,
										&state,
										&gps,
										&manual_control);

	time_keeper_delay_ms(100);

	// Init ahrs
	init_success &= ahrs_init(&ahrs);

	time_keeper_delay_ms(100);

	
	// Init qfilter
	init_success &= qfilter_init(   &attitude_filter,
									qfilter_default_config(),
									&imu,
									&ahrs);
	
	time_keeper_delay_ms(100);
	
	// Init position_estimation_init
	init_success &= position_estimation_init(   	&position_estimation,
													position_estimation_default_config(),
													&state,
													&barometer,
													&sonar,
													&gps,
													&ahrs,
													&data_logging);
	
	time_keeper_delay_ms(100);

	// Init navigation
	init_success &= navigation_init(&navigation,
									navigation_default_config(),
									&controls_nav,
									&ahrs.qe,
									&waypoint_handler,
									&position_estimation,
									&state,
									&manual_control,
									&mavlink_communication);/*,
									&sonar_i2cxl);*/
	
	time_keeper_delay_ms(100);


	// Init waypoint handler
	init_success &= waypoint_handler_init(  &waypoint_handler,
											&position_estimation,
											&ahrs,
											&state,
											&mavlink_communication,
											&mavlink_communication.mavlink_stream);
	waypoint_handler_init_homing_waypoint(&waypoint_handler);
	waypoint_handler_nav_plan_init(&waypoint_handler);
	

	time_keeper_delay_ms(100);

	
	// Init stabilisers
	init_success &= stabilisation_copter_init(	&stabilisation_copter,
												stabilisation_copter_default_config(),
												&controls,
												&ahrs,
												&position_estimation,
												&command.torque,
												&command.thrust);
	
	time_keeper_delay_ms(100);

	init_success &= stabilisation_init( &controls);

	time_keeper_delay_ms(100);//add delay to be able to print on console init message for the following module
	
	// Init hud	
	init_success &= hud_telemetry_init(	&hud_structure, 
										&position_estimation,
										&controls,
										&ahrs);
	
	time_keeper_delay_ms(100);
	
	// Init servo mixing
	init_success &= servos_mix_quadcotper_diag_init( &servo_mix,
													 servos_mix_quadcopter_diag_default_config(),
													 &command.torque,
													 &command.thrust,
													 &servos);

	time_keeper_delay_ms(100);

	// Init manual control
	init_success &= manual_control_init(&manual_control,
										&satellite,
										manual_control_default_config(),
										remote_default_config());

	time_keeper_delay_ms(100);

	//Init data logging
	//TODO: not working here

	init_success &= fat_fs_mounting_init(	&fat_fs_mounting,
											data_logging_default_config(),
											&state);


	time_keeper_delay_ms(100);

	init_success &= data_logging_create_new_log_file(	&data_logging,
														"Log_file",
														true,
														&fat_fs_mounting,
														mavlink_communication.mavlink_stream.sysid);

	return init_success;
}
