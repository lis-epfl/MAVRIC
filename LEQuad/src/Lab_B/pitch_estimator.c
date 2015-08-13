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


/**
 * \brief	Estimate pitch based on gyro data
 * 
 * \param	gyro_z 		Rate in z axis in rad/s
 * \param	gyro_z_old 	Rate in z axis of last iteration rad/s
 * \param 	pitch_old 	Last estimate pitch (by this function) in rad
 * \param 	deltaT		Time passed since the last estimate (by this function) in s
 *
 * \return 				estimated pitch in rad
 */
float estimate_pitch_gyro(float gyro_z, float gyro_z_old, float pitch_old, float deltaT);



float estimate_pitch_accelerometer(float acc_x, float acc_z)
{
	return atan2(acc_x, acc_z);
}

float estimate_pitch_gyro(float gyro_z, float gyro_z_old, float pitch_old, float deltaT)
{
	print_util_dbg_print("gyro avg: ");
	print_util_dbg_putfloat((gyro_z + gyro_z_old)/2,5);
	print_util_dbg_print("\r\n");
	float pitch =  (gyro_z + gyro_z_old)*deltaT/2.0f + pitch_old;
	return pitch;//fmod(pitch+PI,2*PI) - PI;
}

float estimate_pitch_fused(float acc_x, float acc_z)
{
	return 0;
}


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
	int i;
	for(i=0; i < 3; i++)
	{
		estimator->values[i] = 0;
		estimator->gyro_values[i] = 0;
		estimator->accelero_values[i] = 0;
	}
	estimator->timestamp = 0;
	return true;
}

void pitch_estimator_update(pitch_estimator_t* estimator){
	/* estimate pitch based on accelerometer data */
	float* accelero_values = estimator->imu->raw_accelero.data;
	float accelero_pitch = estimate_pitch_accelerometer(accelero_values[0], accelero_values[2]);
	while(accelero_pitch - estimator->values[0] > PI)
		accelero_pitch -= 2*PI;
	while(accelero_pitch - estimator->values[0] < -PI)
		accelero_pitch += 2*PI;
	estimator->values[0] = accelero_pitch;

	/* estimate pitch based on gyro data */
	float* gyro_values = estimator->imu->scaled_gyro.data;
	uint32_t timestamp = estimator->imu->last_update;
	float deltaT = time_keeper_ticks_to_seconds(timestamp - estimator->timestamp);
	if(timestamp > 0 && deltaT > 0)
	{
		estimator->values[1] = estimate_pitch_gyro(gyro_values[1], estimator->gyro_values[1], estimator->values[1], deltaT);
	}
	


	/* estimate pitch by fusing accelerometer and gyro data */
	estimator->values[2] = 2;//estimate_pitch_fused(raw_accelero[1], raw_accelero[2]);

	//memcpy(estimator->gyro_values, gyro_values, 3*sizeof(float));
	//memcpy(estimator->accelero_values, accelero_values, 3*sizeof(float));
	estimator->gyro_values[0] = gyro_values[0];
	estimator->gyro_values[1] = gyro_values[1];
	estimator->gyro_values[2] = gyro_values[2];
	estimator->accelero_values[0] = accelero_values[0];
	estimator->accelero_values[1] = accelero_values[1];
	estimator->accelero_values[2] = accelero_values[2];
	estimator->timestamp = timestamp;
}


