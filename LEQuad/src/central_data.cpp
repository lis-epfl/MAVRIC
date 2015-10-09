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
#include "boardsupport.hpp"
#include "stabilisation_copter_default_config.hpp"
#include "data_logging_default_config.hpp"
#include "mavlink_communication_default_config.hpp"
#include "conf_imu.hpp"
#include "position_estimation_default_config.hpp"
#include "simulation_default_config.hpp"
#include "remote_default_config.hpp"
#include "state_default_config.hpp"

extern "C" 
{
	#include "time_keeper.h"
	#include "navigation_default_config.h"
	#include "servos_default_config.h"
	#include "qfilter_default_config.h"
	#include "scheduler_default_config.h"
	#include "attitude_controller_p2_default_config.h"
	#include "servos_mix_quadcopter_diag_default_config.h"
}


Central_data::Central_data(imu_t& imu, I2c& i2c_sonar, Bmp085& baro, Lsm330dlc& gyracc, Hmc5883l& magneto, File& file):
	imu(imu),
	sonar( Sonar_i2cxl(i2c_sonar) ),
	barometer(baro),
	gyroaccelero(gyracc),
	magnetometer(magneto),
	file_flash(file)
{	
	;
}

bool Central_data::init(Serial& uart_mavlink, Barometer& barometer, Satellite& satellite )
{
	bool init_success = true;

	// Init servos
	init_success &= servos_init( &servos, servos_default_config());
	servos_set_value_failsafe( &servos );
	pwm_servos_write_to_hardware( &servos );

	time_keeper_delay_ms(100);	

	// Init GPS
	// gps_ublox_init( &gps, &board.uart3 );	

	// Init main sheduler
	init_success &= scheduler_init(&scheduler, scheduler_default_config());
	
	time_keeper_delay_ms(100); 

	// Init mavlink communication
	mavlink_communication_conf_t mavlink_communication_config = mavlink_communication_default_config();
	mavlink_communication_config.mavlink_stream_config.sysid = MAVLINK_SYS_ID;
	init_success &= mavlink_communication_init(	&mavlink_communication, 
												mavlink_communication_config, 
												&uart_mavlink,
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
										&sim_model,
										&remote,
										&joystick_parsing,
										&gps);
	time_keeper_delay_ms(100);

	// Init imu
	init_success &= imu_init(   &imu,
								imu_config(),
								&state );
	
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
													&sonar.data,
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
									&joystick_parsing,
									&remote,
									&mavlink_communication);/*,
									&sonar_i2cxl);*/
	
	time_keeper_delay_ms(100);


	// Init waypont handler
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
	
	time_keeper_delay_ms(100);

	// Init simulation (should be done after position_estimation)
	init_success &= simulation_init(&sim_model,
									simulation_default_config(),
									&ahrs,
									&imu,
									&position_estimation,
									&barometer,
									&gps,
									&sonar.data,
									&state,
									&state.nav_plan_active,
									&servo_mix);

	time_keeper_delay_ms(100);//add delay to be able to print on console init message for the following module
	
	// Init hud	
	init_success &= hud_telemetry_init(	&hud_structure, 
										&position_estimation,
										&controls,
										&ahrs);
	
	time_keeper_delay_ms(100);
	
	init_success &= joystick_parsing_init(	&joystick_parsing,
											&state);
	time_keeper_delay_ms(100);
	
	// Init sonar
	// init_success &= sonar_i2cxl_init(&sonar_i2cxl);


	// Init servo mixing
	init_success &= servo_mix_quadcotper_diag_init( &servo_mix,
													servo_mix_quadcopter_diag_default_config(),
													&command.torque,
													&command.thrust,
													&servos);

	// Init remote
	init_success &= remote_init( 	&remote,
									&satellite,
									remote_default_config());


	//Init data logging
	//TODO: not working here
	init_success &= data_logging_create_new_log_file(	&data_logging,
														"Log_file",
														true,
														&fat_fs_mounting,
														mavlink_communication.mavlink_stream.sysid);

	return init_success;
}
