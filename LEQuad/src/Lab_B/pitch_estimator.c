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
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Estimate pitch based on accelerometer data
 * 
 * \param	acc_x 		Acceleration in x-axis in g
 * \param	acc_z	 	Acceleration in x-axis in g
 *
 * \return 				estimated pitch in rad
 */
float estimate_pitch_accelerometer(float acc_x, float acc_z);

/**
 * \brief	Estimate pitch based on gyro data
 * 
 * \param	gyro_y 		Rate in y axis in rad/s
 * \param	gyro_y_old 	Rate in y axis of last iteration rad/s
 * \param 	pitch_old 	Last estimate pitch (by this function) in rad
 * \param 	deltaT		Time step between estimations in seconds
 *
 * \return 				estimated pitch in rad
 */
float estimate_pitch_gyro(float gyro_y, float gyro_y_old, float pitch_old, float deltaT);


/**
 * \brief	Estimate pitch by fusing accelerometer and gyro data
 * 
 * \param	pitch_accelero 		Estimated pitch based on accelerometer data
 * \param	pitch_accelero_old 	Estimated pitch based on accelerometer data of last iteration
 * \param	pitch_gyro 			Estimated pitch based on gyroscope data
 * \param	pitch_gyro_old 		Estimated pitch based on gyroscope data of last iteration
 * \param 	deltaT				Time step between estimations in seconds
 * \param 	tau					Time constant of the filter (tau = 1/(2 *pi *f_c), where f_c is the cutoff freq) in seconds	
 *
 * \return 						Estimated pitch in rad
 */
float estimate_pitch_fused(float pitch_accelero, float pitch_accelero_old, float pitch_gyro, float pitch_gyro_old, float deltaT, float tau);


/**
 * \brief	First order low pass filter
 *
 * \param	x 	 		Current unfiltered value
 * \param	y_old		low passed value of last iteration
 * \param 	deltaT		Time step between estimations in seconds
 * \param 	tau			Time constant of the filter (tau = 1/(2 *pi *f_c), where f_c is the cutoff freq) in seconds
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
 * \param 	deltaT		Time step between estimations in seconds
 * \param 	tau			Time constant of the filter (tau = 1/(2 *pi *f_c), where f_c is the cutoff freq) in seconds
 *
 * \return 				high passed value
 */
float high_pass_filter(float x, float x_old, float y_old, float deltaT, float tau);


/**
 * \brief	Make a angle continuous i.e., prevent jumps at +/- PI
 *
 * \param	phi 		current value in rad
 * \param	phi_old 	value of last iteration
 *
 * \return 				value in rad
 */
float make_angle_continuous(float phi, float phi_old);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


float estimate_pitch_accelerometer(float acc_x, float acc_z)
{
	return atan2(acc_x, acc_z);
}


float estimate_pitch_gyro(float gyro_y, float gyro_y_old, float pitch_old, float deltaT)
{
	float pitch =  (gyro_y + gyro_y_old)*deltaT/2.0f + pitch_old;
	return pitch;
}


float estimate_pitch_fused(float pitch_accelero, float pitch_accelero_old, float pitch_gyro, float pitch_gyro_old, float deltaT, float tau)
{
	return 0;
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


float make_angle_continuous(float phi, float phi_old)
{
	while(phi - phi_old > PI)
		phi -= 2*PI;
	while(phi - phi_old < -PI)
		phi += 2*PI;
	return phi;
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
	estimator->pitch_accelero 			= 0;
	estimator->pitch_accelero_filtered 	= 0;
	estimator->pitch_gyro 				= 0;
	estimator->pitch_gyro_filtered 		= 0;
	estimator->pitch_fused 				= 0;
	estimator->accelero_x 				= 0;
	estimator->accelero_z 				= 0;
	estimator->gyro_y 					= 0;
	estimator->timestamp 				= 0;
	return true;
}

void pitch_estimator_update(pitch_estimator_t* estimator){

	/* values of this iteration*/
	float pitch_accelero 			= 0;
	float pitch_accelero_filtered 	= 0;
	float pitch_gyro 				= 0;
	float pitch_gyro_filtered 		= 0;
	float pitch_fused 				= 0;

	/* values of last iteration (this variables are for readability only */
	const float pitch_accelero_old 			= estimator->pitch_accelero;
	const float pitch_accelero_filtered_old = estimator->pitch_accelero_filtered;
	const float pitch_gyro_old 				= estimator->pitch_gyro;
	const float pitch_gyro_filtered_old 	= estimator->pitch_gyro_filtered;
	const float pitch_fused_old 			= estimator->pitch_fused;
	const float accelero_x_old 				= estimator->accelero_x;
	const float accelero_z_old 				= estimator->accelero_z;
	const float gyro_y_old 					= estimator->gyro_y;
	const uint32_t timestamp_old 			= estimator->timestamp;	

	/* get new measurements */
	float accelero_x = estimator->imu->raw_accelero.data[0];
	float accelero_z = estimator->imu->raw_accelero.data[2];
	float gyro_y = estimator->imu->scaled_gyro.data[1];
	uint32_t timestamp = estimator->imu->last_update;
	float deltaT = time_keeper_ticks_to_seconds(timestamp - timestamp_old);


	float tau = 100*deltaT;

	/* estimate pitch based on accelerometer data */
	pitch_accelero = estimate_pitch_accelerometer(accelero_x, accelero_z);
	pitch_accelero = make_angle_continuous(pitch_accelero, pitch_accelero_old);
	pitch_accelero_filtered = low_pass_filter(pitch_accelero, pitch_accelero_filtered_old, deltaT, tau);	

	/* estimate pitch based on gyro data */
	if(timestamp > 0 && deltaT > 0)
	{
		pitch_gyro = estimate_pitch_gyro(gyro_y, gyro_y_old, pitch_gyro_old, deltaT);
	}else if(estimator->timestamp <= 0)
	{
		/* if it is the first mesurement, take pitch of accelerometer (to have decent init value) */
		pitch_gyro = pitch_accelero;
	}else
	{
		pitch_gyro = pitch_gyro_old;
	}
	pitch_gyro_filtered = high_pass_filter(pitch_gyro, pitch_gyro_old, pitch_gyro_filtered_old, deltaT, tau);
	
	/* estimate pitch by fusing accelerometer and gyro data */
	pitch_fused = estimate_pitch_fused(pitch_accelero, pitch_accelero_old, pitch_gyro, pitch_gyro_old, deltaT, 100*deltaT);

	/* write values to estimator_pitch struct */
	estimator->pitch_accelero 			= pitch_accelero;
	estimator->pitch_accelero_filtered 	= pitch_accelero_filtered;
	estimator->pitch_gyro 				= pitch_gyro;
	estimator->pitch_gyro_filtered		= pitch_gyro_filtered;
	estimator->pitch_fused 				= pitch_fused;
	estimator->accelero_x 				= accelero_x;
	estimator->accelero_z 				= accelero_z;
	estimator->gyro_y 					= gyro_y;
	estimator->timestamp 				= timestamp;
}


