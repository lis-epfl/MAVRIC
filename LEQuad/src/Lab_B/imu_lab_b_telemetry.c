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
 * \file imu_lab_b_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Basil Huber
 *   
 * \brief 
 *
 ******************************************************************************/

#include "imu_lab_b_telemetry.h"
#include "time_keeper.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void send_vect(const float* vect, const char* name, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,
									name,
									time_keeper_get_micros(),
									vect[0],
									vect[1],
									vect[2]);
}

void send_vect_scaled(const float* vect, const char* name, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,
									name,
									time_keeper_get_micros(),
									1000*vect[0],
									1000*vect[1],
									1000*vect[2]);
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void imu_lab_b_telemetry_send(const imu_lab_b_t* imu_lab_b, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
		
	send_vect(imu_lab_b->raw, "ACC RAW", mavlink_stream, msg);
	mavlink_stream_send(mavlink_stream, msg);

	send_vect_scaled(imu_lab_b->scaled, "ACC SCALED", mavlink_stream, msg);
	mavlink_stream_send(mavlink_stream, msg);

	time_keeper_delay_ms(10);
	
	send_vect(imu_lab_b->filtered, "ACC FILTERED", mavlink_stream, msg);
	mavlink_stream_send(mavlink_stream, msg);
	
	send_vect(imu_lab_b->mean, "ACC MEAN", mavlink_stream, msg);
	mavlink_stream_send(mavlink_stream, msg);
	
	time_keeper_delay_ms(10);

	send_vect(imu_lab_b->min, "ACC MIN", mavlink_stream, msg);
	mavlink_stream_send(mavlink_stream, msg);

	send_vect(imu_lab_b->max, "ACC MAX", mavlink_stream, msg);
	mavlink_stream_send(mavlink_stream, msg);

	time_keeper_delay_ms(10);
	
	send_vect(&imu_lab_b->raw[3], "GYRO RAW", mavlink_stream, msg);
	mavlink_stream_send(mavlink_stream, msg);

	send_vect(&imu_lab_b->scaled[3], "GYRO SCALED", mavlink_stream, msg);
	mavlink_stream_send(mavlink_stream, msg);	

	time_keeper_delay_ms(10);

	send_vect(&imu_lab_b->filtered[3], "GYRO FILTERED", mavlink_stream, msg);
	mavlink_stream_send(mavlink_stream, msg);

	send_vect(&imu_lab_b->mean[3], "GYRO MEAN", mavlink_stream, msg);
	mavlink_stream_send(mavlink_stream, msg);
	
	send_vect(&imu_lab_b->min[3], "GYRO MIN", mavlink_stream, msg);
	mavlink_stream_send(mavlink_stream, msg);

	send_vect(&imu_lab_b->max[3], "GYRO MAX", mavlink_stream, msg);
}