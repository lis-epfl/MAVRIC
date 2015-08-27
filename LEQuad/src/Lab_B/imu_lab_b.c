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
 * \file imu_lab_b.c
 * 
 * \author MAV'RIC Team
 * \author Basil Huber
 *   
 * \brief 
 *
 ******************************************************************************/

#include "imu_lab_b.h"
#include "time_keeper.h"
#include <math.h>


#define ALPHA 0.5f
#define ALPHA_MEAN 0.0002f

static float scale_value(float raw_value, float bias, float scale);
//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static float scale_value(float raw_value, float bias, float scale)
{
	return 0;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void imu_lab_b_init(imu_lab_b_t* imu_lab_b, imu_t* imu)
{
	imu_lab_b->imu = imu;
	imu_lab_b->measurement_count = 0;
	imu_lab_b->reset_filter = 1;
}

void imu_lab_b_update(imu_lab_b_t* imu_lab_b)
{
	uint32_t timestamp =  imu_lab_b->imu->last_update;

	if(timestamp <= 0)
		return;
	
	int i;
	for(i = 0; i < 6; i++)
	{
		float raw_value, scaled_value;
		if(i < 3)
		{
			raw_value = imu_lab_b->imu->oriented_accelero.data[i];
			/* scale the value */
			float bias = imu_lab_b->imu->calib_accelero.bias[i];
			float scale = imu_lab_b->imu->calib_accelero.scale_factor[i];
			scaled_value = scale_value(raw_value, bias, scale);
		}
		else
		{
			raw_value = imu_lab_b->imu->oriented_gyro.data[i-3];
			/* scale the value */
			float bias = imu_lab_b->imu->calib_gyro.bias[i-3];
			float scale = imu_lab_b->imu->calib_gyro.scale_factor[i-3];
			scaled_value = scale_value(raw_value, bias, scale);	
		}
		if(imu_lab_b->reset_filter <= 0)
		{
			float filtered = ALPHA*raw_value + ((1-ALPHA)* imu_lab_b->filtered[i]);
			//mean = (imu_lab_b->mean[i]*imu_lab_b->measurement_count + imu_lab_b->values[i])/(imu_lab_b->measurement_count+1);
			imu_lab_b->filtered[i] = filtered;
			imu_lab_b->mean[i] = ALPHA_MEAN*raw_value + ((1-ALPHA_MEAN) * imu_lab_b->mean[i]);
			imu_lab_b->min[i] = fmin(imu_lab_b->min[i], filtered);
			imu_lab_b->max[i] = fmax(imu_lab_b->max[i], filtered);
		} else {
			imu_lab_b->filtered[i] = raw_value;
			imu_lab_b->mean[i] = raw_value;
			imu_lab_b->min[i] = raw_value;
			imu_lab_b->max[i] = raw_value;
		}

		imu_lab_b->raw[i] = raw_value;
		imu_lab_b->scaled[i] = scaled_value;
	}
	imu_lab_b->measurement_count++;
	imu_lab_b->timestamp = timestamp;
}