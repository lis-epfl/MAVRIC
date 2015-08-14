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
 * \file pitch_estimator.c
 * 
 * \author MAV'RIC Team
 * \author Basil Huber
 *   
 * \brief This file implements an estimator for the pitch angle (assuming 2D motion)
 * 			For use in TP only
 * 			This estimator can only be used if roll ~ 0 and yaw constant
 *
 ******************************************************************************/

#include "pitch_estimator.h"
#include "print_util.h"
#include "time_keeper.h"
#include <math.h>

//------------------------------------------------------------------------------
// SYMBOLS FOR BIASES AND SCALES OF IMU
//------------------------------------------------------------------------------

#define ACCELERO_X_BIAS 0
#define ACCELERO_Z_BIAS 0
#define ACCELERO_X_SCALE 1
#define ACCELERO_Z_SCALE 1
#define GYRO_Y_BIAS 0
#define GYRO_Y_SCALE 1.0f/818.5111f

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Estimate pitch based on accelerometer data
 * 
 * \param	acc_x 		Acceleration in x-axis [g]
 * \param	acc_z	 	Acceleration in x-axis [g]
 *
 * \return 				estimated pitch in rad
 */
float estimate_pitch_accelerometer(float acc_x, float acc_z);

/**
 * \brief	Estimate pitch based on gyro data
 * 
 * \param	gyro_y 		Rate in y axis [rad/s]
 * \param	gyro_y_old 	Rate in y axis of last iteration [rad/s]
 * \param 	pitch_old 	Last estimate pitch (by this function) [rad]
 * \param 	deltaT		Time step between estimations [s]
 *
 * \return 				estimated pitch [rad]
 */
float estimate_pitch_gyro(float gyro_y, float gyro_y_old, float pitch_old, float deltaT);


/**
 * \brief	Estimate pitch by fusing accelerometer and gyro data
 * 
 * \param	pitch_accelero_filtered		Estimated pitch based on accelerometer data with filter applied [rad]
 * \param	pitch_gyro_filtered			Estimated pitch based on gyroscope data with filter applied [rad]
 *
 * \return 								Estimated pitch [rad]
 */
float estimate_pitch_fused(float pitch_accelero_filtered, float pitch_gyro_filtered);


/**
 * \brief	First order low pass filter
 *
 * \param	x 	 		Current unfiltered value
 * \param	y_old		low passed value of last iteration
 * \param 	deltaT		Time step between estimations [s]
 * \param 	tau			Time constant of the filter (tau = 1/(2 *pi *f_c), where f_c is the cutoff freq) [s]
 *
 * \return 				low passed value
 */
float low_pass_filter(float x, float y_old, float deltaT, float tau);


/**
 * \brief	First order high pass filter
 *
 * \param	x 	 		Current unfiltered value
 * \param	x_old 		unfiltered value of last iteration
 * \param	y_old		low passed value of last iteration
 * \param 	deltaT		Time step between estimations [s]
 * \param 	tau			Time constant of the filter (tau = 1/(2 *pi *f_c), where f_c is the cutoff freq) [s]
 *
 * \return 				high passed value
 */
float high_pass_filter(float x, float x_old, float y_old, float deltaT, float tau);


/**
 * \brief	Correct bias and scale of gyro or accelero values
 *
 * \param	value_raw	raw data (gyro/accelero)
 * \param	bias 		bias of the sensor value
 * \param	scale 		scale of the sensor value
 *
 * \return 				corrected value
 */
 float correct_measurement(float value_raw, float bias, float scale);


/**
 * \brief	Make a angle continuous i.e., prevent jumps at +/- PI
 *
 * \param	phi 		current value [rad]
 * \param	phi_old 	value of last iteration [rad]
 *
 * \return 				value [rad]
 */
float make_angle_continuous(float phi, float phi_old);


/**
 * \brief	Move angle betwee +/- PI
 *
 * \param	phi 		current value [rad]
 *
 * \return 				value [rad]
 */
 float angle_pi(float phi);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


float estimate_pitch_accelerometer(float acc_x, float acc_z)
{
	return atan2(acc_x, -acc_z);
}


float estimate_pitch_gyro(float gyro_y, float gyro_y_old, float pitch_old, float deltaT)
{
	float pitch =  (gyro_y + gyro_y_old)*deltaT/2.0f + pitch_old;
	return pitch;
}


float estimate_pitch_fused(float pitch_accelero_filtered, float pitch_gyro_filtered)
{
	return pitch_accelero_filtered + pitch_gyro_filtered;
}


float low_pass_filter(float x, float y_old, float deltaT, float tau)
{
	float alpha = deltaT / (deltaT + tau);
	return alpha*x + (1 - alpha)*y_old;
}


float high_pass_filter(float x, float x_old, float y_old, float deltaT, float tau)
{
	float alpha = tau / (deltaT + tau);
	return alpha*(x-x_old) + alpha*y_old;
}


float correct_measurement(float value_raw, float bias, float scale)
{
	return (value_raw + bias) * scale;
}


float make_angle_continuous(float phi, float phi_old)
{
	while(phi - phi_old > PI)
		phi -= 2*PI;
	while(phi - phi_old < -PI)
		phi += 2*PI;
	return phi;
}


float angle_pi(float phi)
{
	return fmod(phi + PI, 2*PI) - PI;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

/**
 * \brief	Initialize the pitch estimator
 * 
 * \param	estimator	The pointer to the pitch estimator structure
 * \param 	imu 		Pointer to the imu structure
 *
 * \return 				returns true if init was successful
 */
 bool pitch_estimator_init(pitch_estimator_t* estimator, imu_t* imu)
{
	/* set imu */
	estimator->imu = imu;
	/* set current values to zero */
	estimator->pitch_accelero_raw 		= 0;
	estimator->pitch_accelero_scaled	= 0;
	estimator->pitch_accelero_filtered 	= 0;
	estimator->pitch_gyro_raw			= 0;
	estimator->pitch_gyro_scaled		= 0;
	estimator->pitch_gyro_filtered 		= 0;
	estimator->pitch_fused 				= 0;
	estimator->gyro_y_raw				= 0;
	estimator->gyro_y_scaled			= 0;
	estimator->timestamp 				= 0;
	return true;
}

void pitch_estimator_update(pitch_estimator_t* estimator){

	/* values of this iteration*/
	float pitch_accelero_raw		= 0;
	float pitch_accelero_scaled 	= 0;
	float pitch_accelero_filtered 	= 0;
	float pitch_gyro_raw			= 0;
	float pitch_gyro_scaled			= 0;
	float pitch_gyro_filtered 		= 0;
	float pitch_fused 				= 0;

	/* values of last iteration (this variables are for readability only */
	const float pitch_accelero_filtered_old = estimator->pitch_accelero_filtered;
	const float pitch_gyro_raw_old			= estimator->pitch_gyro_raw;
	const float pitch_gyro_scaled_old		= estimator->pitch_gyro_scaled;
	const float pitch_gyro_filtered_old 	= estimator->pitch_gyro_filtered;
	const float gyro_y_raw_old 				= estimator->gyro_y_raw;
	const float gyro_y_scaled_old			= estimator->gyro_y_scaled;
	const uint32_t timestamp_old 			= estimator->timestamp;	

	/* get new measurements */
	float accelero_x_raw					=  estimator->imu->raw_accelero.data[0];
	float accelero_z_raw					= -estimator->imu->raw_accelero.data[2];
	float gyro_y_raw						= -estimator->imu->raw_gyro.data[1];
	uint32_t timestamp 						=  estimator->imu->last_update;
	float deltaT = time_keeper_ticks_to_seconds(timestamp - timestamp_old);

	/* correct bias and scale of measurememts */
	float accelero_x_scaled = correct_measurement(accelero_x_raw, ACCELERO_X_BIAS, ACCELERO_X_SCALE);
	float accelero_z_scaled = correct_measurement(accelero_z_raw, ACCELERO_Z_BIAS, ACCELERO_Z_SCALE);
	float gyro_y_scaled = correct_measurement(gyro_y_raw, GYRO_Y_BIAS, GYRO_Y_SCALE);

	/* get filter time constant from qgroundcontrol */
	float tau 								= 100*deltaT;

	/* ------------------------------------------------------------------------------*
	 *		         Here comes the interesting part of the code 					 *
	 * ------------------------------------------------------------------------------*/

	/* estimate pitch based on RAW ACCELEROMETER data */
	pitch_accelero_raw = estimate_pitch_accelerometer(accelero_x_raw, accelero_z_raw);
	//pitch_accelero_raw = make_angle_continuous(pitch_accelero_raw, pitch_accelero_raw_old);
	/* estimate pitch based on SCALED ACCELEROMETER accelerometer data */
	pitch_accelero_scaled = estimate_pitch_accelerometer(accelero_x_scaled, accelero_z_scaled);
	//pitch_accelero_scaled = make_angle_continuous(pitch_accelero_scaled, pitch_accelero_scaled_old);
	/* filter SCALED ACCELEROMETER data */
	pitch_accelero_filtered = low_pass_filter(pitch_accelero_scaled, pitch_accelero_filtered_old, deltaT, tau);

	/* estimate pitch based on gyro data */
	if(timestamp > 0)
	{
		pitch_gyro_raw = estimate_pitch_gyro(gyro_y_raw, gyro_y_raw_old, pitch_gyro_raw_old, deltaT);
		pitch_gyro_raw = angle_pi(pitch_gyro_raw);
		pitch_gyro_scaled = estimate_pitch_gyro(gyro_y_scaled, gyro_y_scaled_old, pitch_gyro_scaled_old, deltaT);
		pitch_gyro_scaled = angle_pi(pitch_gyro_scaled);
	}else
	{
		/* if it is the first mesurement, take pitch of accelerometer (to have decent init value) */
		pitch_gyro_raw = pitch_accelero_raw;
		pitch_gyro_scaled = pitch_accelero_scaled;
	}
	pitch_gyro_filtered = high_pass_filter(pitch_gyro_scaled, pitch_gyro_scaled_old, pitch_gyro_filtered_old, deltaT, tau);
	
	/* estimate pitch by fusing accelerometer and gyro data */
	pitch_fused = estimate_pitch_fused(pitch_accelero_filtered, pitch_gyro_filtered);


	/* ------------------------------------------------------------------------------*
	 *		         Here ends the interesting part of the code 					 *
	 * ------------------------------------------------------------------------------*/

	/* write values to estimator_pitch struct */
	estimator->pitch_accelero_raw 		= pitch_accelero_raw;
	estimator->pitch_accelero_scaled 	= pitch_accelero_scaled;
	estimator->pitch_accelero_filtered 	= pitch_accelero_filtered;
	estimator->pitch_gyro_raw 			= pitch_gyro_raw;
	estimator->pitch_gyro_scaled		= pitch_gyro_scaled;
	estimator->pitch_gyro_filtered		= pitch_gyro_filtered;
	estimator->pitch_fused 				= pitch_fused;
	estimator->gyro_y_raw 				= gyro_y_raw;
	estimator->gyro_y_scaled			= gyro_y_scaled;
	estimator->timestamp 				= timestamp;
}


