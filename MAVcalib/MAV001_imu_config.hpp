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
 * \file MAV001_imu_config.h
 * 
 * \author MAV'RIC Team
 *   
 * \brief This file defines the mapping between the IMU and the compass and the 
 * frames of the vehicles as well as the scales and the biaises. 
 * The NED frame is used.
 *
 ******************************************************************************/


#ifndef CONF_IMU_REV4_H_
#define CONF_IMU_REV4_H_

#include "imu.hpp"

static inline imu_conf_t imu_config(void)
{
	imu_conf_t conf	= imu_default_config();

	// Accelerometer Bias
	conf.accelerometer.bias[0] = 0.0275f;		// = 110.0 / 4000.0
	conf.accelerometer.bias[1] = 0.02375f;		// = 95.0  / 4000.0
	conf.accelerometer.bias[2] = 0.0825f;		// = 330.0 / 4000.0
	
	// Gyroscope Bias
	conf.gyroscope.bias[0] = -0.01219f;			// = -9.98    / 818.5111
	conf.gyroscope.bias[1] = -0.01626f;			// = -13.31   / 818.5111
	conf.gyroscope.bias[2] = -0.17729f;			// = -145.116 / 818.5111
	
	// Magnetometer Bias
	conf.magnetometer.bias[0] = -0.4925f;		// = -295.5 / 600.0
	conf.magnetometer.bias[1] = -0.1683f;		// = -101.0 / 600.0
	conf.magnetometer.bias[2] = 0.0691f;		// =  41.5  / 600.0
	

	/**
	 * Bellow should be common between megafly_rev4 boards
	 */

	// Accelerometer Scale
	conf.accelerometer.scale_factor[0] = 4000.0f;
	conf.accelerometer.scale_factor[1] = 4000.0f;
	conf.accelerometer.scale_factor[2] = 4000.0f;
	
	// Gyroscope Scale
	conf.gyroscope.scale_factor[0] = 818.5111f;
	conf.gyroscope.scale_factor[1] = 818.5111;
	conf.gyroscope.scale_factor[2] = 818.5111f;

	// Magnetometer Scale
	conf.magnetometer.scale_factor[0] = 600.0f;
	conf.magnetometer.scale_factor[1] = 600.0f;
	conf.magnetometer.scale_factor[2] = 600.0f;
	
	// Axis and sign
	conf.accelerometer.sign[0] = +1.0f;
	conf.accelerometer.sign[1] = -1.0f;
	conf.accelerometer.sign[2] = -1.0f;
	conf.accelerometer.axis[0] = 0;
	conf.accelerometer.axis[1] = 1;
	conf.accelerometer.axis[2] = 2;

	// Axis and sign
	conf.gyroscope.sign[0] = +1.0f;
	conf.gyroscope.sign[1] = -1.0f;
	conf.gyroscope.sign[2] = -1.0f;
	conf.gyroscope.axis[0] = 0;
	conf.gyroscope.axis[1] = 1;
	conf.gyroscope.axis[2] = 2;

	// Axis and sign
	conf.magnetometer.sign[0] = -1.0f;
	conf.magnetometer.sign[1] = -1.0f;
	conf.magnetometer.sign[2] = -1.0f;
	conf.magnetometer.axis[0] = 2;
	conf.magnetometer.axis[1] = 0;
	conf.magnetometer.axis[2] = 1;

	return conf;
}

// static imu_conf_t imu_config =
// {
// 	.accelerometer =
// 	{
// 		.axis =
// 		{
// 			0,					//ACC_AXIS_X
// 			1,					//ACC_AXIS_Y
// 			2					//ACC_AXIS_Z
// 		},
// 		.orientation =
// 		{
// 			1.0f,				//ACC_ORIENTATION_X
// 			-1.0f,				//ACC_ORIENTATION_Y
// 			-1.0f				//ACC_ORIENTATION_Z
// 		},
// 		.bias =
// 		{
// 			110.0f,				//ACC_BIAIS_X
// 			95.0f,				//ACC_BIAIS_Y
// 			330.0f				//ACC_BIAIS_Z
// 		},
// 		.scale_factor =
// 		{
// 			3924.0f,			//RAW_ACC_X_SCALE
// 			3844.8f,			//RAW_ACC_Y_SCALE
// 			4119.6f				//RAW_ACC_Z_SCALE
// 		}
// 	},
// 	.gyroscope =
// 	{
// 		.axis =
// 		{
// 			0,					//GYRO_AXIS_X
// 			1,					//GYRO_AXIS_Y
// 			2					//GYRO_AXIS_Z
// 		},
// 		.orientation =
// 		{
// 			1.0f,				//GYRO_ORIENTATION_X
// 			-1.0f,				//GYRO_ORIENTATION_Y
// 			-1.0f				//GYRO_ORIENTATION_Z
// 		},
// 		.bias =
// 		{
// 			-9.98f,				//GYRO_BIAIS_X
// 			-13.31f,			//GYRO_BIAIS_Y
// 			-145.116f			//GYRO_BIAIS_Z
// 		},
// 		.scale_factor =
// 		{
// 			818.5111f,			//RAW_GYRO_X_SCALE
// 			818.5111f,			//RAW_GYRO_Y_SCALE
// 			818.5111f			//RAW_GYRO_Z_SCALE
// 		}
// 	},
// 	.magnetometer =
// 	{
// 		.axis =
// 		{
// 			2,					//MAG_AXIS_X
// 			0,					//MAG_AXIS_Y
// 			1					//MAG_AXIS_Z
// 		},
// 		.orientation =
// 		{
// 			-1.0f,				//MAG_ORIENTATION_X
// 			-1.0f,				//MAG_ORIENTATION_Y
// 			-1.0f				//MAG_ORIENTATION_Z
// 		},
// 		.bias =
// 		{
// 			-295.5f,			//MAG_BIAIS_X
// 			-101.0f,			//MAG_BIAIS_Y
// 			41.5f				//MAG_BIAIS_Z
// 		},
// 		.scale_factor =
// 		{
// 			601.3117f,			//RAW_MAG_X_SCALE
// 			580.3974f,			//RAW_MAG_Y_SCALE
// 			513.8466f			//RAW_MAG_Z_SCALE
// 		}
// 	}
// };

#endif /* CONF_IMU_REV4_H_ */