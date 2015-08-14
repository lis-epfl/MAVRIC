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
 * \file pitch_estimator.h
 * 
 * \author MAV'RIC Team
 * \author Basil Huber
 *   
 * \brief This file implements an estimator for the pitch angle (assuming 2D motion)
 * 			For use in TP only
 * 			This estimator can only be used if roll ~ 0 and yaw constant
 *
 ******************************************************************************/
#ifndef PITCH_ESTIMATOR_H_
#define PITCH_ESTIMATOR_H_


#ifdef __cplusplus
extern "C" {
	#endif

#include "imu.h"
#include "mavlink_stream.h"
#include "mavlink_message_handler.h"

/**
 * \brief Structure for pitch estimation
 */
typedef struct
{
	imu_t* imu; 								///< IMU containing measurement, biases, etc.
	float pitch_accelero_raw;					///< Estimated pitch based on raw accelerometer data [rad]
	float pitch_accelero_scaled;				///< Estimated pitch based on corrected accelerometer data [rad]
	float pitch_accelero_filtered;				///< Estimated pitch based on filtered accelerometer data [rad]
	float pitch_gyro_raw;						///< Estimated pitch based on raw gyroscope data [rad]
	float pitch_gyro_scaled;					///< Estimated pitch based on corrected gyroscope data [rad]
	float pitch_gyro_filtered;					///< Estimated pitch based on filtered gyroscope data [rad]
	float pitch_fused;							///< Estimated pitch by fusion of accelerometer and gyroscope data [rad]
	float gyro_y_raw;							///< Rate raw around y axis [rad]
	float gyro_y_scaled;						///< Rate scaled around y axis [rad]
	float timestamp;							///< time stamp of IMU measurement [ticks]
} pitch_estimator_t;

/**
 * \brief	Initialize the pitch estimator
 * 
 * \param	estimator	The pointer to the pitch estimator structure
 * \param 	imu 		Pointer to the imu structure
 *
 * \return 				returns true if init was successful
 */
bool pitch_estimator_init(pitch_estimator_t* estimator, imu_t* imu);

void pitch_estimator_update(pitch_estimator_t* estimator);

#ifdef __cplusplus
}
#endif

#endif /* PITCH_ESTIMATOR_H_ */