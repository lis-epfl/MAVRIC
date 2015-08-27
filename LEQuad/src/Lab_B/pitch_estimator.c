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
#include "conf_imu.h"

//------------------------------------------------------------------------------
// SYMBOLS FOR BIASES AND SCALES OF IMU
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Estimate pitch based on accelerometer data
 * 
 * \param	acc_x 		Acceleration in x-axis [g]
 * \param	acc_y 		Acceleration in x-axis [g]
 * \param	acc_z	 	Acceleration in x-axis [g]
 *
 * \return 				estimated pitch in rad
 */
float estimate_pitch_accelerometer(float acc_x, float acc_y, float acc_z);

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
float estimate_pitch_gyro(float gyro_y, float pitch_old, float deltaT);


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
static float low_pass_filter(float x, float y_old, float deltaT, float tau);


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


float estimate_pitch_accelerometer(float acc_x, float acc_y, float acc_z)
{
	return 0;
}


float estimate_pitch_gyro(float gyro_y, float pitch_old, float deltaT)
{
	return 0;
}


float estimate_pitch_fused(float pitch_accelero_filtered, float pitch_gyro_filtered)
{
	return 0;
}


static float low_pass_filter(float x, float y_old, float deltaT, float tau)
{
	y_old = make_angle_continuous(y_old, x);
	return 0;
}


float high_pass_filter(float x, float x_old, float y_old, float deltaT, float tau)
{
	x_old = make_angle_continuous(x_old, x);
	return 0;
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
	while(phi > PI)
		phi -= 2*PI;
	while(phi < -PI)
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
	imu_lab_b_init(&estimator->imu_lab_b, imu);

	/* set current values to zero */
	estimator->pitch_accelero 			= 0;
	estimator->pitch_accelero_filtered 	= 0;
	estimator->pitch_gyro				= 0;
	estimator->pitch_gyro_filtered 		= 0;
	estimator->pitch_fused 				= 0;
	estimator->gyro_y_scaled			= 0;
	estimator->timestamp 				= 0;
	estimator->filter_constant 			= 1;
	return true;
}

void pitch_estimator_update(pitch_estimator_t* estimator){

	imu_lab_b_update(&estimator->imu_lab_b);
	
	/* values of this iteration*/
	float pitch_accelero;
	float pitch_accelero_filtered;
	float pitch_gyro;
	float pitch_gyro_filtered;
	float pitch_fused;

	/* values of last iteration (this variables are for readability only */
	const float pitch_accelero_filtered_old = estimator->pitch_accelero_filtered;
	const float pitch_gyro_old 				= estimator->pitch_gyro;
	const float pitch_gyro_filtered_old 	= estimator->pitch_gyro_filtered;
	const uint32_t timestamp_old 			= estimator->timestamp;	

	/* get new measurements */
	float accelero_x_scaled 				= estimator->imu_lab_b.scaled[0];
	float accelero_y_scaled 				= estimator->imu_lab_b.scaled[1];
	float accelero_z_scaled 				= estimator->imu_lab_b.scaled[2];
	float gyro_y_scaled 					= estimator->imu_lab_b.scaled[4];

	uint32_t timestamp 						= estimator->imu_lab_b.timestamp;
	const float deltaT = time_keeper_ticks_to_seconds(timestamp - timestamp_old);

	/* get filter time constant from qgroundcontrol */
	const float tau = estimator->filter_constant;

	/* ------------------------------------------------------------------------------*
	 *		         Here comes the interesting part of the code 					 *
	 * ------------------------------------------------------------------------------*/
	
	/* estimate pitch based on SCALED ACCELEROMETER accelerometer data */
	pitch_accelero = estimate_pitch_accelerometer(accelero_x_scaled, accelero_y_scaled, accelero_z_scaled);
	
	/* check if we have to reset the filters and estimations */
	if(estimator->imu_lab_b.reset_filter <= 0)
	{
		/* filter SCALED ACCELEROMETER data */
		pitch_accelero_filtered = low_pass_filter(pitch_accelero, pitch_accelero_filtered_old, deltaT, tau);

		/* estimate pitch based on gyro data */
		pitch_gyro = estimate_pitch_gyro(gyro_y_scaled, pitch_gyro_old, deltaT);
		pitch_gyro = angle_pi(pitch_gyro);
		pitch_gyro_filtered = high_pass_filter(pitch_gyro, pitch_gyro_old, pitch_gyro_filtered_old, deltaT, tau);
		pitch_gyro_filtered = angle_pi(pitch_gyro_filtered);
	}else
	{
		/* if we reset the filter, we do not filter the accelero */		
		pitch_accelero_filtered = pitch_accelero;

		/* if we reset the filter, take pitch estimation for gyros*/
		pitch_gyro = pitch_accelero;
		pitch_gyro_filtered = 0;
		estimator->imu_lab_b.reset_filter = 0;
	}
	

	/* estimate pitch by fusing accelerometer and gyro data */
	pitch_fused = estimate_pitch_fused(pitch_accelero_filtered, pitch_gyro_filtered);
	pitch_fused = angle_pi(pitch_fused);

	/* ------------------------------------------------------------------------------*
	 *		         Here ends the interesting part of the code 					 *
	 * ------------------------------------------------------------------------------*/

	/* write values to estimator_pitch struct */
	estimator->pitch_accelero 	 		= pitch_accelero;
	estimator->pitch_accelero_filtered 	= pitch_accelero_filtered;
	estimator->pitch_gyro 	 			= pitch_gyro;
	estimator->pitch_gyro_filtered		= pitch_gyro_filtered;
	estimator->pitch_fused 				= pitch_fused;
	estimator->gyro_y_scaled			= gyro_y_scaled;
	estimator->timestamp 				= timestamp;
}