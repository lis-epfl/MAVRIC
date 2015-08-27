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
 * \file MAV201_imu_config.h
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

#include "bias_scale.h"
#include "imu.h"
#include "conf_platform.h"	// To get the MAVLINK_SYS_ID

static imu_conf_t imu_config =
{
	.accelerometer =
	{
		.axis =
		{
			0,					//ACC_AXIS_X
			1,					//ACC_AXIS_Y
			2					//ACC_AXIS_Z
		},
		.orientation =
		{
			1.0f,				//ACC_ORIENTATION_X
			-1.0f,				//ACC_ORIENTATION_Y
			-1.0f				//ACC_ORIENTATION_Z
		},
		.bias =
		{
			ACC_BIAIS_X,			//ACC_BIAIS_X
			ACC_BIAIS_Y,			//ACC_BIAIS_Y
			ACC_BIAIS_Z				//ACC_BIAIS_Z
		},
		.scale_factor =
		{
			ACC_SCALE_X,			//RAW_ACC_X_SCALE
			ACC_SCALE_Y,			//RAW_ACC_Y_SCALE
			ACC_SCALE_Z				//RAW_ACC_Z_SCALE
		}
	},
	.gyroscope =
	{
		.axis =
		{
			0,					//GYRO_AXIS_X
			1,					//GYRO_AXIS_Y
			2					//GYRO_AXIS_Z
		},
		.orientation =
		{
			1.0f,				//GYRO_ORIENTATION_X
			-1.0f,				//GYRO_ORIENTATION_Y
			-1.0f				//GYRO_ORIENTATION_Z
		},
		.bias =
		{
			0.0f,				//GYRO_BIAIS_X
			GYRO_BIAIS_Y,		//GYRO_BIAIS_Y
			0.0f				//GYRO_BIAIS_Z
		},
		.scale_factor =
		{
			818.5111f,			//RAW_GYRO_X_SCALE
			818.5111f,			//RAW_GYRO_Y_SCALE
			818.5111f			//RAW_GYRO_Z_SCALE
		}
	},
	.magnetometer =
	{
		.axis =
		{
			2,					//MAG_AXIS_X
			0,					//MAG_AXIS_Y
			1					//MAG_AXIS_Z
		},
		.orientation =
		{
			-1.0f,				//MAG_ORIENTATION_X
			-1.0f,				//MAG_ORIENTATION_Y
			-1.0f				//MAG_ORIENTATION_Z
		},
		.bias =
		{
			-215.00f,			//MAG_BIAIS_X
			-165.00f,			//MAG_BIAIS_Y
			-357.72f			//MAG_BIAIS_Z
		},
		.scale_factor =
		{
			482.72f,			//RAW_MAG_X_SCALE
			573.55f,			//RAW_MAG_Y_SCALE
			500.13f				//RAW_MAG_Z_SCALE
		}
	}
};

#endif /* CONF_IMU_REV4_H_ */
