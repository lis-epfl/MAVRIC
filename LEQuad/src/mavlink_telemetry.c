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
 * \file mavlink_telemetry.c
 *
 * \author MAV'RIC Team
 *   
 * \brief Definition of the messages sent by the autopilot to the ground station
 *
 ******************************************************************************/


#include "mavlink_telemetry.h"
#include "central_data.h"
#include "onboard_parameters.h"
#include "mavlink_stream.h"
#include "scheduler.h"
#include "tasks.h"
#include "mavlink_waypoint_handler.h"
#include "analog_monitor.h"
#include "state.h"
#include "position_estimation.h"
#include "sonar_i2cxl.h"

#include "acoustic_telemetry.h"
#include "airspeed_analog_telemetry.h"
#include "fat_fs_mounting_telemetry.h"
#include "hud_telemetry.h"
#include "remote_telemetry.h"
#include "servos_telemetry.h"
#include "state_telemetry.h"
#include "gps_ublox_telemetry.h"
#include "imu_telemetry.h"
#include "bmp085_telemetry.h"
#include "ahrs_telemetry.h"
#include "position_estimation_telemetry.h"
#include "stabilisation_telemetry.h"
#include "joystick_parsing_telemetry.h"
#include "simulation_telemetry.h"
#include "scheduler_telemetry.h"
#include "sonar_telemetry.h"
#include "servos_mix_wing_telemetry.h"

#include "analog_monitor_telemetry.h"

#include "conf_platform.h"

central_data_t *central_data;

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Add onboard logging parameters
 *
 * \param	data_logging			The pointer to the data logging structure
 *
 * \return	The initialization status of the module, succeed == true
 */
bool mavlink_telemetry_add_data_logging_parameters(data_logging_t* data_logging);

/**
 * \brief   Initialise the callback functions
 * 
 * \param   central_data            The pointer to the central_data structure
 *
 * \return	The initialization status of the module, succeed == true
 */
bool mavlink_telemetry_init_communication_module(central_data_t *central_data);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


bool mavlink_telemetry_add_data_logging_parameters(data_logging_t* data_logging)
{
	bool init_success = true;
	
	// Add your logging parameters here, name length max = MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN = 16
	// Supported type: all numeric types included in mavlink_message_type_t (i.e. all except MAVLINK_TYPE_CHAR)
	
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->imu.scaled_accelero.data[X], "acc_x", 4);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->imu.scaled_accelero.data[Y], "acc_y", 4);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->imu.scaled_accelero.data[Z], "acc_z", 4);
	
	//init_success &= data_logging_add_parameter_double(data_logging, &central_data->position_estimation.local_position.origin.latitude,	"origin_latitude", 7);
	//init_success &= data_logging_add_parameter_double(data_logging, &central_data->position_estimation.local_position.origin.longitude, "origin_longitude", 7);
	//init_success &= data_logging_add_parameter_float(data_logging,	&central_data->position_estimation.local_position.origin.altitude,	"origin_altitude", 3);
	
	//init_success &= data_logging_add_parameter_float(data_logging,	&central_data->position_estimation.local_position.pos[0], "local_x", 3);
	//init_success &= data_logging_add_parameter_float(data_logging,	&central_data->position_estimation.local_position.pos[1], "local_y", 3);
	//init_success &= data_logging_add_parameter_float(data_logging,	&central_data->position_estimation.local_position.pos[2], "local_z", 3);
	
	//init_success &= data_logging_add_parameter_double(data_logging, &central_data->gps.latitude, "latitude", 7);
	//init_success &= data_logging_add_parameter_double(data_logging, &central_data->gps.longitude, "longitude", 7);
	//init_success &= data_logging_add_parameter_float(data_logging,	&central_data->gps.altitude, "altitude", 3);
	
	//init_success &= data_logging_add_parameter_int8(data_logging, &central_data->state_machine.rc_check, "rc_check");
	//init_success &= data_logging_add_parameter_uint32(data_logging, (uint32_t*)&central_data->state_machine.rc_check, "rc_check");
	
	//init_success &= data_logging_add_parameter_uint32(data_logging, (uint32_t*)&central_data->state.mav_state, "mav_state");
	//init_success &= data_logging_add_parameter_uint8(data_logging, &central_data->state.mav_mode.byte, "mav_mode");
	
	
	
	////////////
	// TUNING //
	////////////
	// Miscellaneous
	init_success &= data_logging_add_parameter_uint8(data_logging, &central_data->state.mav_mode.byte, "mav_mode");
	init_success &= data_logging_add_parameter_uint32(data_logging, (uint32_t*)&central_data->state.mav_mode_custom, "mode_custom");
	
	// Global output
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->servos.servo[M_WING_RIGHT].value, "servo_r", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->servos.servo[M_WING_LEFT].value, "servo_l", 3);
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->servos.servo[M_WING_THRUST].value, "servo_t", 3);
	
	
	
	///////////////////////
	//// PID: PITCH RATE //
	///////////////////////
	//// Gains
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].p_gain, "GainPPitch", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].integrator.gain, "GainIPitch", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].differentiator.gain, "GainDPitch", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].clip_max, "CMaxPitch", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].clip_min, "CMinPitch", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].integrator.clip_pre, "CPreIPitch", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].integrator.clip, "CIPitch", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].differentiator.clip, "CDPitch", 3);
			//
	//// Error (==> possible to compute reference)
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].error, "ErrPitch", 3);
			//
	//// Feedback
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->ahrs.angular_speed[PITCH], "GyroPitch", 3);
			//
	//// Command
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.output.rpy[PITCH], "OutPitch", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].integrator.accumulator, "OutIPitch", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[PITCH].differentiator.previous, "PrevDPitch", 3);
	//
	//
	//
	//////////////////////
	//// PID: ROLL RATE //
	//////////////////////
	//// Gains
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].p_gain, "GainPRoll", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].integrator.gain, "GainIRoll", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].differentiator.gain, "GainDRoll", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].clip_max, "CMaxRoll", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].clip_min, "CMinRoll", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].integrator.clip_pre, "CPreIRoll", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].integrator.clip, "CIRoll", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].differentiator.clip, "CDRoll", 3);
			//
	//// Error (==> possible to compute reference)
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].error, "ErrRoll", 3);
			//
	//// Feedback
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->ahrs.angular_speed[ROLL], "GyroRoll", 3);
			//
	//// Command
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.output.rpy[ROLL], "OutRoll", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].integrator.accumulator, "OutIRoll", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser.rpy_controller[ROLL].differentiator.previous, "PrevDRoll", 3);
	
	
	
	//////////////////////
	// PID: PITCH ANGLE //
	//////////////////////
	// Gains
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].p_gain, "GainPPitch", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].integrator.gain, "GainIPitch", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].differentiator.gain, "GainDPitch", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].clip_max, "CMaxPitch", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].clip_min, "CMinPitch", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].integrator.clip_pre, "CPreIPitch", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].integrator.clip, "CIPitch", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].differentiator.clip, "CDPitch", 3);
	
	// Error (==> possible to compute reference)
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].error, "ErrPitch", 3);
	//
	//// Feedback
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->ahrs.up_vec.v[0], "UpVectPitch", 3);
	//
	//// Command
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.output.rpy[PITCH], "OutPitch", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].integrator.accumulator, "OutIPitch", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[PITCH].differentiator.previous, "PrevDPitch", 3);
	
	
	
	/////////////////////
	// PID: ROLL ANGLE //
	/////////////////////
	// Gains
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].p_gain, "GainPRoll", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].integrator.gain, "GainIRoll", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].differentiator.gain, "GainDRoll", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].clip_max, "CMaxRoll", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].clip_min, "CMinRoll", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].integrator.clip_pre, "CPreIRoll", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].integrator.clip, "CIRoll", 3);
// 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].differentiator.clip, "CDRoll", 3);
	
	// Error (==> possible to compute reference)
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].error, "ErrRoll", 3);
	//
	//// Feedback
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->ahrs.up_vec.v[1], "UpVectRoll", 3);
	//
	//// Command
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.output.rpy[ROLL], "OutRoll", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].integrator.accumulator, "OutIRoll", 3);
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser.rpy_controller[ROLL].differentiator.previous, "PrevDRoll", 3);
	
	
	
	//////////////////////////
	// PID: THRUST VELOCITY //
	//////////////////////////
	// Gains
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser.thrust_controller.p_gain, "GainP_thr", 3);
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser.thrust_controller.integrator.gain, "GainI_thr", 3);
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser.thrust_controller.differentiator.gain, "GainD_thr", 3);
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser.thrust_controller.clip_max, "CMax_thr", 3);
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser.thrust_controller.clip_min, "CMin_thr", 3);
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser.thrust_controller.integrator.clip_pre, "CPreI_thr", 3);
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser.thrust_controller.integrator.clip, "CI_thr", 3);
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser.thrust_controller.differentiator.clip, "CD_thr", 3);
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.thrust_apriori, "Apriori_thr", 3);
 	
 	// Error (==> possible to compute reference)
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser.thrust_controller.error, "Err_thr", 3);
 	
 	// Feedback
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->airspeed_analog.airspeed, "fee_thr", 3);
	init_success &= data_logging_add_parameter_float(data_logging, &central_data->ahrs.up_vec.v[0], "UpVectPitch", 3);		// Influence a lot the airspeed...
 	
 	// Command
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser.output.thrust, "Out_thr", 3);
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser.thrust_controller.integrator.accumulator, "OutI_thr", 3);
 	init_success &= data_logging_add_parameter_float(data_logging, &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser.thrust_controller.differentiator.previous, "PrevD_thr", 3);
	
	
	
	//////////////
	// AIRSPEED //
	//////////////
	// Sensor
	init_success &= data_logging_add_parameter_float(data_logging, &central_data->airspeed_analog.differential_pressure, "Ai_pres_dif", 3);
	init_success &= data_logging_add_parameter_float(data_logging, &central_data->airspeed_analog.pressure_offset, "Ai_off", 3);
	init_success &= data_logging_add_parameter_float(data_logging, &central_data->airspeed_analog.airspeed, "Ai_airspeed", 3);
	init_success &= data_logging_add_parameter_float(data_logging, &central_data->airspeed_analog.raw_airspeed, "Ai_spd_raw", 3);
//	init_success &= data_logging_add_parameter_float(data_logging, &central_data->airspeed_analog.scaled_airspeed, "Ai_spd_sca", 3);
	
	// GPS
	init_success &= data_logging_add_parameter_float(data_logging, &central_data->gps.ground_speed, "gps_grdspd", 3);
	init_success &= data_logging_add_parameter_float(data_logging, &central_data->gps.speed, "gps_3d", 3);
	
	init_success &= data_logging_add_parameter_float(data_logging, &central_data->position_estimation.vel[0], "vel_0", 3);
	init_success &= data_logging_add_parameter_float(data_logging, &central_data->position_estimation.vel[1], "vel_1", 3);
	init_success &= data_logging_add_parameter_float(data_logging, &central_data->position_estimation.vel[2], "vel_2", 3);
	
	
	

	
	return init_success;
};

bool mavlink_telemetry_init_communication_module(central_data_t *central_data)
{
	bool init_success = true;
	
	init_success &= state_telemetry_init(   &central_data->state,
	&central_data->mavlink_communication.message_handler);

	init_success &= imu_telemetry_init( &central_data->imu,
	&central_data->mavlink_communication.message_handler);

	init_success &= remote_telemetry_init(  &central_data->remote,
	&central_data->mavlink_communication.message_handler);

	init_success &= joystick_parsing_telemetry_init(&central_data->joystick_parsing,
	&central_data->mavlink_communication.message_handler);

	init_success &= simulation_telemetry_init(  &central_data->sim_model,
	&central_data->mavlink_communication.message_handler);
	
	init_success &= position_estimation_telemetry_init(	&central_data->position_estimation,
	&central_data->mavlink_communication.message_handler);
	
	init_success &= fat_fs_mounting_telemetry_init(	&central_data->fat_fs_mounting,
	&central_data->mavlink_communication.message_handler);
	
	init_success &= gps_ublox_telemetry_init( &central_data->gps,
	&central_data->mavlink_communication.message_handler);

	init_success &= servo_mix_wing_telemetry_init(	&central_data->servo_mix,
	&central_data->mavlink_communication.message_handler);

	init_success &= airspeed_analog_telemetry_init(	&central_data->airspeed_analog,
	&central_data->mavlink_communication.message_handler);

	return init_success;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


bool mavlink_telemetry_add_onboard_parameters(onboard_parameters_t * onboard_parameters)
{
	bool init_success = true;
	//stabiliser_t* rate_stabiliser = &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser;
	//stabiliser_t* attitude_stabiliser = &central_data->stabilisation_wing.stabiliser_stack.attitude_stabiliser;
	stabiliser_t* velocity_stabiliser= &central_data->stabilisation_wing.stabiliser_stack.velocity_stabiliser;
	//stabiliser_t* position_stabiliser= &central_data->stabilisation_copter.stabiliser_stack.position_stabiliser;
	
	// System ID
	init_success &= onboard_parameters_add_parameter_int32    ( onboard_parameters , (int32_t*)&central_data->mavlink_communication.mavlink_stream.sysid              , "ID_SYSID"         );

	// Simulation mode
	//init_success &= onboard_parameters_add_parameter_int32    ( onboard_parameters , ( int32_t*)&central_data->state.simulation_mode              , "Sim_mode"         );
	
	// Test attitude controller gains
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_angle[ROLL]  , "gainA_Roll"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_angle[PITCH] , "gainA_Pitch"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_angle[YAW]   , "gainA_Yaw"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_rate[ROLL]   , "gainR_Roll"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_rate[PITCH]  , "gainR_Pitch"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_rate[YAW]    , "gainR_Yaw"     );

	// Roll rate PID
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].p_gain				, "RollRate_P_G");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].integrator.gain		, "RollRate_I_G");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].integrator.clip_pre	, "RollRate_I_CP");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].integrator.clip		, "RollRate_I_C");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].differentiator.gain	, "RollRate_D_G");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].differentiator.clip	, "RollRate_D_C");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].clip_max				, "RollRate_C_Max");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].clip_min				, "RollRate_C_Min");
	
	//// Roll attitude PID
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].p_gain					, "RollAtti_P_G");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].integrator.gain		, "RollAtti_I_G");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].integrator.clip_pre	, "RollAtti_I_CP");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].integrator.clip		, "RollAtti_I_C");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].differentiator.gain	, "RollAtti_D_G");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].differentiator.clip	, "RollAtti_D_C");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].clip_max				, "RollAtti_C_Max");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].clip_min				, "RollAtti_C_Min");

	// Pitch rate PID
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].p_gain				, "PitchRate_P_G");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].integrator.gain		, "PitchRate_I_G");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].integrator.clip_pre	, "PitchRate_I_CP");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].integrator.clip		, "PitchRate_I_C");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].differentiator.gain	, "PitchRate_D_G");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].differentiator.clip	, "PitchRate_D_C");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].clip_max			, "PitchRate_C_Max");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].clip_min			, "PitchRate_C_Min");
	
	//// Pitch attitude PID
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].p_gain				, "PitchAtti_P_G");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].integrator.gain		, "PitchAtti_I_G");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].integrator.clip_pre	, "PitchAtti_I_CP");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].integrator.clip		, "PitchAtti_I_C");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].differentiator.gain	, "PitchAtti_D_G");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].differentiator.clip	, "PitchAtti_D_C");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].clip_max				, "PitchAtti_C_Max");
	//init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].clip_min				, "PitchAtti_C_Min");
	
	//// Thrust velocity PID
 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &velocity_stabiliser->thrust_controller.p_gain					, "ThruVel_P_G");
 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &velocity_stabiliser->thrust_controller.integrator.gain			, "ThruVel_I_G");
 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &velocity_stabiliser->thrust_controller.integrator.clip_pre		, "ThruVel_I_CP");
 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &velocity_stabiliser->thrust_controller.integrator.clip			, "ThruVel_I_C");
 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &velocity_stabiliser->thrust_controller.differentiator.gain		, "ThruVel_D_G");
 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &velocity_stabiliser->thrust_controller.differentiator.clip		, "ThruVel_D_C");
 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &velocity_stabiliser->thrust_controller.clip_max				, "ThruVel_C_Max");
 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &velocity_stabiliser->thrust_controller.clip_min				, "ThruVel_C_Min");
 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters , &central_data->stabilisation_wing.thrust_apriori				, "ThruVel_aprio");

	// Yaw rate PID
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].p_gain                          , "YawRPid_P_G"      );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].clip_max                        , "YawRPid_P_CLmx"   );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].clip_min                        , "YawRPid_P_CLmn"   );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].integrator.maths_clip           , "YawRPid_I_CLip"   );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].integrator.postgain             , "YawRPid_I_PstG"   );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].integrator.pregain              , "YawRPid_I_PreG"   );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].differentiator.maths_clip       , "YawRPid_D_Clip"   );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].differentiator.gain             , "YawRPid_D_Gain"   );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[YAW].differentiator.LPF              , "YawRPid_D_LPF"    );
	
	// Yaw attitude PID
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].p_gain                      , "YawAPid_P_G"      );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].clip_max                    , "YawAPid_P_CLmx"   );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].clip_min                    , "YawAPid_P_CLmn"   );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].integrator.maths_clip       , "YawAPid_I_CLip"   );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].integrator.postgain         , "YawAPid_I_PstG"   );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].integrator.pregain          , "YawAPid_I_PreG"   );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].differentiator.maths_clip   , "YawAPid_D_Clip"   );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].differentiator.gain         , "YawAPid_D_Gain"   );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].differentiator.LPF          , "YawAPid_D_LPF"    );


	// Roll velocity PID
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[ROLL].p_gain                     , "RollVPid_P_G"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[ROLL].integrator.postgain        , "RollVPid_I_PstG"  );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[ROLL].integrator.pregain         , "RollVPid_I_PreG"  );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[ROLL].integrator.maths_clip      , "RollVPid_I_Clip"  );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[ROLL].differentiator.gain        , "RollVPid_D_Gain"  );


	// Pitch velocity PID
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[PITCH].p_gain                    , "PitchVPid_P_G"    );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[PITCH].integrator.postgain       , "PitchVPid_I_PstG" );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[PITCH].integrator.pregain        , "PitchVPid_I_PreG" );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[PITCH].integrator.maths_clip     , "PitchVPid_I_Clip" );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->rpy_controller[PITCH].differentiator.gain       , "PitchVPid_D_Gain" );

	// Thrust velocity PID
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.p_gain                        , "ThrVPid_P_G"      );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.integrator.clip_pre            , "ThrVPid_I_PreG"   );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.differentiator.gain           , "ThrVPid_D_Gain"   );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.soft_zone_width               , "ThrVPid_soft"     );

	// Roll position PID
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->rpy_controller[ROLL].p_gain                     , "RollPPid_P_G"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->rpy_controller[ROLL].integrator.postgain        , "RollPPid_I_PstG"  );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->rpy_controller[ROLL].integrator.pregain         , "RollPPid_I_PreG"  );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->rpy_controller[ROLL].differentiator.gain        , "RollPPid_D_Gain"  );

	// Pitch position PID
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->rpy_controller[PITCH].p_gain                    , "PitchPPid_P_G"    );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->rpy_controller[PITCH].integrator.postgain       , "PitchPPid_I_PstG" );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->rpy_controller[PITCH].integrator.pregain        , "PitchPPid_I_PreG" );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->rpy_controller[PITCH].differentiator.gain       , "PitchPPid_D_Gain" );

	// Thrust position PID
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->thrust_controller.p_gain                        , "ThrPPid_P_G"      );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->thrust_controller.integrator.postgain           , "ThrPPid_I_PstG"   );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->thrust_controller.integrator.pregain            , "ThrPPid_I_PreG"   );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->thrust_controller.differentiator.gain           , "ThrPPid_D_Gain"   );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->thrust_controller.differentiator.LPF            , "ThrPPid_D_LPF"    );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &position_stabiliser->thrust_controller.soft_zone_width               , "ThrPPid_soft"     );


	// qfilter
	//init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->attitude_filter.kp                                        , "QF_kp_acc"        );
	//init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->attitude_filter.kp_mag                                    , "QF_kp_mag"        );
	//init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].differentiator.gain         , "YawAPid_D_Gain"   );
	
	// Biaises
	/*
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_gyro.bias[X]									  , "Bias_Gyro_X"      );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_gyro.bias[Y]									  , "Bias_Gyro_Y"      );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_gyro.bias[Z]									  , "Bias_Gyro_Z"      );
	
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_accelero.bias[X]								  , "Bias_Acc_X"       );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_accelero.bias[Y]								  , "Bias_Acc_Y"       );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_accelero.bias[Z]								  , "Bias_Acc_Z"       );
	
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_compass.bias[X]								  , "Bias_Mag_X"       );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_compass.bias[Y]								  , "Bias_Mag_Y"       );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_compass.bias[Z]								  , "Bias_Mag_Z"       );
	*/
	
	// Scale factor
	/*
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_gyro.scale_factor[X]							  , "Scale_Gyro_X"     );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_gyro.scale_factor[Y]							  , "Scale_Gyro_Y"     );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_gyro.scale_factor[Z]							  , "Scale_Gyro_Z"     );
	
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_accelero.scale_factor[X]                       , "Scale_Acc_X"      );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_accelero.scale_factor[Y]                       , "Scale_Acc_Y"      );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_accelero.scale_factor[Z]                       , "Scale_Acc_Z"      );
	
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_compass.scale_factor[X]                        , "Scale_Mag_X"      );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_compass.scale_factor[Y]                        , "Scale_Mag_Y"      );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_compass.scale_factor[Z]                        , "Scale_Mag_Z"      );
	*/

	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimation.kp_alt_baro                              , "Pos_kp_alt_baro"       );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimation.kp_vel_baro                              , "Pos_kp_velb"      );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimation.kp_pos_gps[0]                            , "Pos_kp_pos0"      );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimation.kp_pos_gps[1]                            , "Pos_kp_pos1"      );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimation.kp_pos_gps[2]                            , "Pos_kp_pos2"      );
	


	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.dist2vel_gain								, "vel_dist2Vel"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.cruise_speed									, "vel_cruiseSpeed"  );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.max_climb_rate								, "vel_climbRate"    );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.soft_zone_size								, "vel_softZone"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.hovering_controller.p_gain					, "vel_hover_Pgain"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.hovering_controller.differentiator.gain		, "vel_hover_Dgain"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.wpt_nav_controller.p_gain					, "vel_wpt_Pgain"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.wpt_nav_controller.differentiator.gain		, "vel_wpt_Dgain"     );
	
//	init_success &= onboard_parameters_add_parameter_int32    ( onboard_parameters , ( int32_t*)&central_data->state_machine.low_battery_counter			, "safe_count"     );

	//init_success &= onboard_parameters_add_parameter_int32(onboard_parameters, (int32_t*) &central_data->state.remote_active,"Remote_Active");
	//init_success &= onboard_parameters_add_parameter_int32(onboard_parameters, (int32_t*) &central_data->state.source_mode,"Remote_Src_Mode");

	init_success &= onboard_parameters_add_parameter_int32(onboard_parameters,(int32_t*)&central_data->fat_fs_mounting.log_data, "Log_continue");
	
	
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->servo_mix.config.trim_roll		, "trim_roll"     );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->servo_mix.config.trim_pitch		, "trim_pitch"     );
	
// 	init_success &= onboard_parameters_add_parameter_int32(onboard_parameters, &central_data->stabilisation_wing.tuning,		"tuning");
// 	init_success &= onboard_parameters_add_parameter_int32(onboard_parameters, &central_data->stabilisation_wing.tuning_axis,	"tun_axis");
// 	init_success &= onboard_parameters_add_parameter_int32(onboard_parameters, &central_data->stabilisation_wing.tuning_steps,	"tun_steps");
// 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters, &central_data->stabilisation_wing.pitch_up,		"tun_p_up");
// 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters, &central_data->stabilisation_wing.pitch_down,	"tun_p_down");
// 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters, &central_data->stabilisation_wing.roll_right,	"tun_r_r");
// 	init_success &= onboard_parameters_add_parameter_float(onboard_parameters, &central_data->stabilisation_wing.roll_left,		"tun_r_l");
// 	
	init_success &= onboard_parameters_add_parameter_float(onboard_parameters, &central_data->attitude_filter_madgwick.beta,							"Mad_beta");
	init_success &= onboard_parameters_add_parameter_float(onboard_parameters, &central_data->attitude_filter_madgwick.zeta,							"Mad_zeta");
	init_success &= onboard_parameters_add_parameter_int32(onboard_parameters,(int32_t*)&central_data->attitude_filter_madgwick.acceleration_correction, "Mad_cor_ena");
	init_success &= onboard_parameters_add_parameter_float(onboard_parameters, &central_data->attitude_filter_madgwick.correction_speed,				"Mad_cor_spe");
	
	// Airspeed
	init_success &= onboard_parameters_add_parameter_float(onboard_parameters, &central_data->airspeed_analog.conversion_factor,	"Ai_conv_fact");
	init_success &= onboard_parameters_add_parameter_float(onboard_parameters, &central_data->airspeed_analog.correction_gain,		"Ai_corr_gain");
	init_success &= onboard_parameters_add_parameter_float(onboard_parameters, &central_data->airspeed_analog.correction_offset,	"Ai_corr_off");
	
	
	return init_success;
}


bool mavlink_telemetry_init(void)
{
	bool init_success = true;
	
	central_data = central_data_get_pointer_to_struct();
	
	init_success &= mavlink_telemetry_add_data_logging_parameters(&central_data->data_logging);

	init_success &= mavlink_telemetry_init_communication_module(central_data);
	
	mavlink_communication_t* mavlink_communication = &central_data->mavlink_communication;
	
	stabiliser_t* stabiliser_show = &central_data->stabilisation_wing.stabiliser_stack.rate_stabiliser;

	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  1000000,  RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&state_telemetry_send_heartbeat,								&central_data->state, 					MAVLINK_MSG_ID_HEARTBEAT			);// ID 0
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  1000000,	 RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&state_telemetry_send_status,									&central_data->state,					MAVLINK_MSG_ID_SYS_STATUS			);// ID 1
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  1000000,  RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&gps_ublox_telemetry_send_raw,									&central_data->gps,						MAVLINK_MSG_ID_GPS_RAW_INT			);// ID 24
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&imu_telemetry_send_scaled,										&central_data->imu, 					MAVLINK_MSG_ID_SCALED_IMU			);// ID 26
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  100000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&imu_telemetry_send_raw,										&central_data->imu, 					MAVLINK_MSG_ID_RAW_IMU				);// ID 27
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&bmp085_telemetry_send_pressure,								&central_data->pressure,				MAVLINK_MSG_ID_SCALED_PRESSURE		);// ID 29
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  200000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&ahrs_telemetry_send_attitude,									&central_data->ahrs,				 	MAVLINK_MSG_ID_ATTITUDE				);// ID 30
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&ahrs_telemetry_send_attitude_quaternion,						&central_data->ahrs,				 	MAVLINK_MSG_ID_ATTITUDE_QUATERNION	);// ID 31
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&position_estimation_telemetry_send_position,					&central_data->position_estimation, 	MAVLINK_MSG_ID_LOCAL_POSITION_NED	);// ID 32
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&position_estimation_telemetry_send_global_position,			&central_data->position_estimation, 	MAVLINK_MSG_ID_GLOBAL_POSITION_INT	);// ID 33
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&remote_telemetry_send_scaled,									&central_data->remote,					MAVLINK_MSG_ID_RC_CHANNELS_SCALED	);// ID 34
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&remote_telemetry_send_raw,										&central_data->remote,					MAVLINK_MSG_ID_RC_CHANNELS_RAW		);// ID 35
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  1000000,  RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&servos_telemetry_mavlink_send,									&central_data->servos, 					MAVLINK_MSG_ID_SERVO_OUTPUT_RAW		);// ID 36
	
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,	 RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&joystick_parsing_telemetry_send_manual_ctrl_msg,				&central_data->joystick_parsing,		MAVLINK_MSG_ID_MANUAL_CONTROL		);// ID 69
	//init_success &= mavlink_communication_add_msg_send(mavlink_communication,  200000,   RUN_REGULAR,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&stabilisation_telemetry_send_control,						&central_data->controls, 				MAVLINK_MSG_ID_MANUAL_CONTROL		);// ID 69
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&hud_telemetry_send_message,									&central_data->hud_structure, 			MAVLINK_MSG_ID_VFR_HUD				);// ID 74
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&simulation_telemetry_send_state,								&central_data->sim_model, 				MAVLINK_MSG_ID_HIL_STATE			);// ID 90
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  500000,	 RUN_NEVER,	   PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&simulation_telemetry_send_quaternions,							&central_data->sim_model,				MAVLINK_MSG_ID_HIL_STATE_QUATERNION	);// ID 115
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&sonar_telemetry_send,											&central_data->sonar_i2cxl.data, 		MAVLINK_MSG_ID_DISTANCE_SENSOR	);// ID 119
	
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  200000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&stabilisation_telemetry_send_rpy_speed_thrust_setpoint,		stabiliser_show,						MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT	);// ID 160
	
	//init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&scheduler_telemetry_send_rt_stats,								&central_data->scheduler, 				MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);// ID 251
	//init_success &= mavlink_communication_add_msg_send(mavlink_communication,  100000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&sonar_telemetry_send,							&central_data->sonar_i2cxl.data, 			MAVLINK_MSG_ID_DISTANCE_SENSOR	);// ID 132
	//init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&acoustic_telemetry_send,										&central_data->audio_data, 				MAVLINK_MSG_ID_DEBUG_VECT			);// ID 250
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&airspeed_analog_telemetry_send,								&central_data->airspeed_analog, 		MAVLINK_MSG_ID_DEBUG_VECT			);// ID 250
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&analog_monitor_telemetry_send_differential_pressure,			&central_data->analog_monitor, 			MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);// ID 251
	
	scheduler_sort_tasks(&central_data->mavlink_communication.scheduler);
	
	print_util_dbg_print("MAVlink telemetry initialiased\r\n");
	
	return init_success;
}
