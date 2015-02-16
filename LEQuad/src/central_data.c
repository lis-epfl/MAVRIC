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


#include "central_data.h"
#include "delay.h"

#include "conf_imu.h"

#include "ahrs_default_config.h"
#include "stabilisation_copter_default_config.h"
#include "navigation_default_config.h"
#include "servos_default_config.h"
#include "qfilter_default_config.h"
#include "position_estimation_default_config.h"
#include "simulation_default_config.h"
#include "scheduler_default_config.h"
#include "remote_default_config.h"
#include "data_logging_default_config.h"
#include "state_default_config.h"
#include "mavlink_communication_default_config.h"
#include "attitude_controller_p2_default_config.h"
#include "servos_mix_quadcopter_diag_default_config.h"

static central_data_t central_data;

bool central_data_init()
{	
	bool init_success = true;
	
	// Init servos
	//servo_pwm_init(central_data.servos);
	init_success &= servos_init( &central_data.servos, &servos_default_config);
	servos_set_value_failsafe( &central_data.servos );
	pwm_servos_write_to_hardware( &central_data.servos );

	delay_ms(100);	


	// Init main sheduler
	init_success &= scheduler_init(&central_data.scheduler, &scheduler_default_config);
	
	delay_ms(100); 

	// Init mavlink communication
	mavlink_communication_conf_t mavlink_communication_config = mavlink_communication_default_config;
	mavlink_communication_config.mavlink_stream_config.sysid = MAVLINK_SYS_ID;
	init_success &= mavlink_communication_init(	&central_data.mavlink_communication, 
												&mavlink_communication_config, 
												central_data.telemetry_up_stream, 
												central_data.telemetry_down_stream);
	
	delay_ms(100); 

	// Init state structure
	init_success &= state_init(	&central_data.state,
								&state_default_config,
								&central_data.analog_monitor); 
	
	delay_ms(100);

	//Init state_machine	
	init_success &= state_machine_init( &central_data.state_machine,
										&central_data.state,
										&central_data.waypoint_handler,
										&central_data.sim_model,
										&central_data.remote,
										&central_data.navigation);
	delay_ms(100);

	// Init imu
	init_success &= imu_init(   &central_data.imu,
								&imu_config,
								&central_data.state);
	
	delay_ms(100);

	// Init ahrs
	init_success &= ahrs_init(	&central_data.ahrs, &ahrs_default_config);

	delay_ms(100);

	
	// Init qfilter
	init_success &= qfilter_init(   &central_data.attitude_filter,
									&qfilter_default_config,
									&central_data.imu,
									&central_data.ahrs);
	
	delay_ms(100);
	
	// Init position_estimation_init
	init_success &= position_estimation_init(   	&central_data.position_estimation,
													&position_estimation_default_config,
													&central_data.state,
													&central_data.pressure,
													&central_data.gps,
													&central_data.ahrs,
													&central_data.imu);
	
	delay_ms(100);

	// Init navigation
	init_success &= navigation_init(&central_data.navigation,
									&navigation_default_config,
									&central_data.controls_nav,
									&central_data.ahrs.qe,
									&central_data.waypoint_handler,
									&central_data.position_estimation,
									&central_data.state,
									&central_data.joystick_parsing,
									&central_data.remote,
									&central_data.mavlink_communication);/*,
									&central_data.sonar_i2cxl);*/
	
	delay_ms(100);

	// Init waypont handler
	init_success &= waypoint_handler_init(  &central_data.waypoint_handler,
											&central_data.position_estimation,
											&central_data.ahrs,
											&central_data.state,
											&central_data.mavlink_communication,
											&central_data.mavlink_communication.mavlink_stream);
	waypoint_handler_init_homing_waypoint(&central_data.waypoint_handler);
	waypoint_handler_nav_plan_init(&central_data.waypoint_handler);
	
	delay_ms(100);

	
	// Init stabilisers
	init_success &= stabilisation_copter_init(	&central_data.stabilisation_copter,
												&stabilisation_copter_default_config,
												&central_data.controls,
												&central_data.imu,
												&central_data.ahrs,
												&central_data.position_estimation,
												&central_data.servos);
	
	delay_ms(100);

	init_success &= stabilisation_init( &central_data.controls);
	
	delay_ms(100);

	// Init simulation (should be done after position_estimation)
	init_success &= simulation_init(&central_data.sim_model,
									&simulation_default_config,
									&central_data.ahrs,
									&central_data.imu,
									&central_data.position_estimation,
									&central_data.pressure,
									&central_data.gps,
									&central_data.state,
									&central_data.servos,
									&central_data.state.nav_plan_active);

	delay_ms(100);//add delay to be able to print on console init message for the following module
	
	// Init hud	
	init_success &= hud_telemetry_init(	&central_data.hud_structure, 
										&central_data.position_estimation,
										&central_data.controls,
										&central_data.ahrs);
	
	delay_ms(100);
	
	init_success &= joystick_parsing_init(	&central_data.joystick_parsing,
											&central_data.state);
	delay_ms(100);
	
	// Init sonar
	sonar_i2cxl_init(&central_data.sonar_i2cxl);

	// Init P^2 attitude controller
	attitude_controller_p2_init( 	&central_data.attitude_controller,
									&attitude_controller_p2_default_config,
									&central_data.command.attitude,
									&central_data.command.torque,
									&central_data.ahrs );

	// Init servo mixing
	init_success &= servo_mix_quadcotper_diag_init( &central_data.servo_mix, 
													&servo_mix_quadcopter_diag_default_config, 
													&central_data.command.torque, 
													&central_data.command.thrust, 
													&central_data.servos);

	// Init remote
	init_success &= remote_init( 	&central_data.remote, 
									&remote_default_config);

	
	//Init data logging
	init_success &= data_logging_init(  &central_data.data_logging,
										&data_logging_default_config,
										&central_data.state);
										
	return init_success;
}

central_data_t* central_data_get_pointer_to_struct(void)
{
	return (central_data_t*)&central_data;
}