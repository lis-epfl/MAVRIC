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
#include "data_logging_telemetry.h"
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
	
	// if _USE_LFN == 0: Name: max 8 characters + 3 for extension; if _USE_LFN != 0: Name: max 255 characters + more flexible extension type
	init_success &= data_logging_create_new_log_file(data_logging, "NewFile", central_data->mavlink_communication.mavlink_stream.sysid);
	
	// Add your logging parameters here, name length max = MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN = 16
	// Supported type: all numeric types included in mavlink_message_type_t (i.e. all except MAVLINK_TYPE_CHAR)
	
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->imu.scaled_accelero.data[X], "acc_x");
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->imu.scaled_accelero.data[Y], "acc_y");
	//init_success &= data_logging_add_parameter_float(data_logging, &central_data->imu.scaled_accelero.data[Z], "acc_z");
	
	init_success &= data_logging_add_parameter_double(data_logging, &central_data->position_estimation.local_position.origin.latitude,	"origin_latitude");
	init_success &= data_logging_add_parameter_double(data_logging, &central_data->position_estimation.local_position.origin.longitude, "origin_longitude");
	init_success &= data_logging_add_parameter_float(data_logging,	&central_data->position_estimation.local_position.origin.altitude,	"origin_altitude");
	
	init_success &= data_logging_add_parameter_float(data_logging,	&central_data->position_estimation.local_position.pos[0], "local_x");
	init_success &= data_logging_add_parameter_float(data_logging,	&central_data->position_estimation.local_position.pos[1], "local_y");
	init_success &= data_logging_add_parameter_float(data_logging,	&central_data->position_estimation.local_position.pos[2], "local_z");
	
	init_success &= data_logging_add_parameter_double(data_logging, &central_data->gps.latitude, "latitude");
	init_success &= data_logging_add_parameter_double(data_logging, &central_data->gps.longitude, "longitude");
	init_success &= data_logging_add_parameter_float(data_logging,	&central_data->gps.altitude, "altitude");
	
	//init_success &= data_logging_add_parameter_int8(data_logging, &central_data->state_machine.rc_check, "rc_check");
	//init_success &= data_logging_add_parameter_uint32(data_logging, (uint32_t*)&central_data->state_machine.rc_check, "rc_check");
	
	//init_success &= data_logging_add_parameter_uint32(data_logging, (uint32_t*)&central_data->state.mav_state, "mav_state");
	init_success &= data_logging_add_parameter_uint8(data_logging, &central_data->state.mav_mode.byte, "mav_mode");
	
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
	
	init_success &= data_logging_telemetry_init(	&central_data->data_logging,
	&central_data->mavlink_communication.message_handler);
	
	return init_success;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


bool mavlink_telemetry_add_onboard_parameters(onboard_parameters_t * onboard_parameters)
{
	bool init_success = true;
	
	//stabiliser_t* rate_stabiliser = &central_data->stabilisation_copter.stabiliser_stack.rate_stabiliser;
	//stabiliser_t* attitude_stabiliser = &central_data->stabilisation_copter.stabiliser_stack.attitude_stabiliser;
	stabiliser_t* velocity_stabiliser= &central_data->stabilisation_copter.stabiliser_stack.velocity_stabiliser;
	//stabiliser_t* position_stabiliser= &central_data->stabilisation_copter.stabiliser_stack.position_stabiliser;
	
	// System ID
	init_success &= onboard_parameters_add_parameter_int32    ( onboard_parameters , (int32_t*)&central_data->mavlink_communication.mavlink_stream.sysid              , "ID_SYSID"         );

	// Simulation mode
	init_success &= onboard_parameters_add_parameter_int32    ( onboard_parameters , ( int32_t*)&central_data->state.simulation_mode              , "Sim_mode"         );
	
	// Test attitude controller gains
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_angle[ROLL]  , "gainA_Roll"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_angle[PITCH] , "gainA_Pitch"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_angle[YAW]   , "gainA_Yaw"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_rate[ROLL]   , "gainR_Roll"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_rate[PITCH]  , "gainR_Pitch"     );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->attitude_controller.p_gain_rate[YAW]    , "gainR_Yaw"     );

	// Roll rate PID
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].p_gain                         , "RollRPid_P_G"     );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].integrator.maths_clip          , "RollRPid_I_CLip"  );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].integrator.postgain            , "RollRPid_I_PstG"  );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].integrator.pregain             , "RollRPid_I_PreG"  );
	//init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].differentiator.maths_clip      , "RollRPid_D_Clip"  );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].differentiator.gain            , "RollRPid_D_Gain"  );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[ROLL].differentiator.LPF             , "RollRPid_D_LPF"   );
	
	// Roll attitude PID
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].p_gain                     , "RollAPid_P_G"     );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].integrator.maths_clip      , "RollAPid_I_CLip"  );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].integrator.postgain        , "RollAPid_I_PstG"  );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].integrator.pregain         , "RollAPid_I_PreG"  );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].differentiator.maths_clip  , "RollAPid_D_Clip"  );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].differentiator.gain        , "RollAPid_D_Gain"  );
	//init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &attitude_stabiliser->rpy_controller[ROLL].differentiator.LPF         , "RollAPid_D_LPF"   );

	// Pitch rate PID
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].p_gain                        , "PitchRPid_P_G"    );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].integrator.maths_clip         , "PitchRPid_I_CLip" );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].integrator.postgain           , "PitchRPid_I_PstG" );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].integrator.pregain            , "PitchRPid_I_PreG" );
	//init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].differentiator.maths_clip     , "PitchRPid_D_Clip" );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].differentiator.gain           , "PitchRPid_D_Gain" );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &rate_stabiliser->rpy_controller[PITCH].differentiator.LPF            , "PitchRPid_D_LPF"  );
	
	// Pitch attitude PID
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].p_gain                    , "PitchAPid_P_G"    );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].integrator.maths_clip     , "PitchAPid_I_CLip" );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].integrator.postgain       , "PitchAPid_I_PstG" );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].integrator.pregain        , "PitchAPid_I_PreG" );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].differentiator.maths_clip , "PitchAPid_D_Clip" );
	//init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].differentiator.gain       , "PitchAPid_D_Gain" );
	//init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &attitude_stabiliser->rpy_controller[PITCH].differentiator.LPF        , "PitchAPid_D_LPF"  );

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
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.p_gain                        , "ThrVPid_P_G"      );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.integrator.postgain           , "ThrVPid_I_PstG"   );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.integrator.pregain            , "ThrVPid_I_PreG"   );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.differentiator.gain           , "ThrVPid_D_Gain"   );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.differentiator.LPF            , "ThrVPid_D_LPF"    );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &velocity_stabiliser->thrust_controller.soft_zone_width               , "ThrVPid_soft"     );

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
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->attitude_filter.kp                                        , "QF_kp_acc"        );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->attitude_filter.kp_mag                                    , "QF_kp_mag"        );
	//init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &attitude_stabiliser->rpy_controller[YAW].differentiator.gain         , "YawAPid_D_Gain"   );
	
	// Biaises
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_gyro.bias[X]									  , "Bias_Gyro_X"      );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_gyro.bias[Y]									  , "Bias_Gyro_Y"      );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_gyro.bias[Z]									  , "Bias_Gyro_Z"      );
	
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_accelero.bias[X]								  , "Bias_Acc_X"       );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_accelero.bias[Y]								  , "Bias_Acc_Y"       );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_accelero.bias[Z]								  , "Bias_Acc_Z"       );
	
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_compass.bias[X]								  , "Bias_Mag_X"       );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_compass.bias[Y]								  , "Bias_Mag_Y"       );
	init_success &= onboard_parameters_add_parameter_float ( onboard_parameters , &central_data->imu.calib_compass.bias[Z]								  , "Bias_Mag_Z"       );
	
	// Scale factor
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_gyro.scale_factor[X]							  , "Scale_Gyro_X"     );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_gyro.scale_factor[Y]							  , "Scale_Gyro_Y"     );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_gyro.scale_factor[Z]							  , "Scale_Gyro_Z"     );
	
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_accelero.scale_factor[X]                       , "Scale_Acc_X"      );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_accelero.scale_factor[Y]                       , "Scale_Acc_Y"      );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_accelero.scale_factor[Z]                       , "Scale_Acc_Z"      );
	
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_compass.scale_factor[X]                        , "Scale_Mag_X"      );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_compass.scale_factor[Y]                        , "Scale_Mag_Y"      );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->imu.calib_compass.scale_factor[Z]                        , "Scale_Mag_Z"      );

	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimation.kp_alt_baro                              , "Pos_kp_alt_baro"       );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimation.kp_vel_baro                              , "Pos_kp_velb"      );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimation.kp_pos_gps[0]                            , "Pos_kp_pos0"      );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimation.kp_pos_gps[1]                            , "Pos_kp_pos1"      );
	init_success &= onboard_parameters_add_parameter_float  ( onboard_parameters , &central_data->position_estimation.kp_pos_gps[2]                            , "Pos_kp_pos2"      );
	


	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.dist2vel_gain                            , "vel_dist2Vel"     );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.cruise_speed                            , "vel_cruiseSpeed"  );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.max_climb_rate                          , "vel_climbRate"    );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.soft_zone_size							  , "vel_softZone"     );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.hovering_controller.p_gain				  , "vel_hover_Pgain"     );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.hovering_controller.differentiator.gain	  , "vel_hover_Dgain"     );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.wpt_nav_controller.p_gain				  , "vel_wpt_Pgain"     );
	init_success &= onboard_parameters_add_parameter_float    ( onboard_parameters , &central_data->navigation.wpt_nav_controller.differentiator.gain	  , "vel_wpt_Dgain"     );
	

	init_success &= onboard_parameters_add_parameter_int32(onboard_parameters, (int32_t*) &central_data->state.remote_active,"Remote_Active");
	init_success &= onboard_parameters_add_parameter_int32(onboard_parameters, (int32_t*) &central_data->state.use_mode_from_remote, "Remote_Use_Mode");

	init_success &= onboard_parameters_add_parameter_int32(onboard_parameters,(int32_t*)&central_data->data_logging.log_data, "Log_continue");
	
	return init_success;
}


bool mavlink_telemetry_init(void)
{
	bool init_success = true;
	
	central_data = central_data_get_pointer_to_struct();
	
	init_success &= mavlink_telemetry_add_data_logging_parameters(&central_data->data_logging);

	init_success &= mavlink_telemetry_init_communication_module(central_data);
	
	mavlink_communication_t* mavlink_communication = &central_data->mavlink_communication;
	
	stabiliser_t* stabiliser_show = &central_data->stabilisation_copter.stabiliser_stack.rate_stabiliser;

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
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,   RUN_REGULAR,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&remote_telemetry_send_raw,										&central_data->remote,					MAVLINK_MSG_ID_RC_CHANNELS_RAW		);// ID 35
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  1000000,  RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&servos_telemetry_mavlink_send,									&central_data->servos, 					MAVLINK_MSG_ID_SERVO_OUTPUT_RAW		);// ID 36
	
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,	 RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&joystick_parsing_telemetry_send_manual_ctrl_msg,				&central_data->joystick_parsing,		MAVLINK_MSG_ID_MANUAL_CONTROL		);// ID 69
	//init_success &= mavlink_communication_add_msg_send(mavlink_communication,  200000,   RUN_REGULAR,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&stabilisation_telemetry_send_control,						&central_data->controls, 				MAVLINK_MSG_ID_MANUAL_CONTROL		);// ID 69
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&hud_telemetry_send_message,									&central_data->hud_structure, 			MAVLINK_MSG_ID_VFR_HUD				);// ID 74
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&simulation_telemetry_send_state,								&central_data->sim_model, 				MAVLINK_MSG_ID_HIL_STATE			);// ID 90
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  500000,	 RUN_NEVER,	   PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&simulation_telemetry_send_quaternions,							&central_data->sim_model,				MAVLINK_MSG_ID_HIL_STATE_QUATERNION	);// ID 115
	//init_success &= mavlink_communication_add_msg_send(mavlink_communication,  100000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&sonar_telemetry_send,										&central_data->sonar_i2cxl, 			MAVLINK_MSG_ID_DISTANCE_SENSOR	);// ID 119
	
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  200000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&stabilisation_telemetry_send_rpy_speed_thrust_setpoint,		stabiliser_show,						MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT	);// ID 160
	
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&scheduler_telemetry_send_rt_stats,								&central_data->scheduler, 				MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);// ID 251
	//init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&analog_monitor_telemetry_send_sonar,							&central_data->analog_monitor, 			MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);// ID 251
	init_success &= mavlink_communication_add_msg_send(mavlink_communication,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (mavlink_send_msg_function_t)&acoustic_telemetry_send,										&central_data->audio_data, 				MAVLINK_MSG_ID_DEBUG_VECT			);// ID 250
	
	scheduler_sort_tasks(&central_data->mavlink_communication.scheduler);
	
	print_util_dbg_print("MAVlink telemetry initialiased\r\n");
	
	return init_success;
}