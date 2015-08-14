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
 * \file pitch_estimator_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Basil Huber
 *   
 * \brief pitch estimator telemetry functions
 *
 ******************************************************************************/

#include "pitch_estimator_telemetry.h"
#include "time_keeper.h"
/**
 * \brief 	Send the estimated pitch
 *
 * \param	estimator			Pointer to the pitch estimator structure
 * \param	mavlink_stream		Pointer to mavlink stream structure
 * \param	msg					Pointer to the message structure
 */
void pitch_estimator_telemetry_send (const pitch_estimator_t* estimator, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,
									"Pitch accelero",
									time_keeper_get_micros(),
									1000*estimator->pitch_accelero_raw,
									1000*estimator->pitch_accelero_scaled,
									1000*estimator->pitch_accelero_filtered);
	mavlink_stream_send(mavlink_stream, msg);
	
	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,
									"Pitch gyro",
									time_keeper_get_micros(),
									1000*estimator->pitch_gyro_raw,
									1000*estimator->pitch_gyro_scaled,
									1000*estimator->pitch_gyro_filtered);

	mavlink_stream_send(mavlink_stream, msg);
	
	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,
									"Pitch fused",
									time_keeper_get_micros(),
									1000*estimator->pitch_fused,
									0,
									0);
	
	/*mavlink_msg_named_value_int_pack(	mavlink_stream->sysid,
	 									mavlink_stream->compid,
	 									msg,
						       			time_keeper_get_micros(),
						       			"pa",
						       			estimator->pitch_accelero
						       			);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_named_value_int_pack(	mavlink_stream->sysid,
	 									mavlink_stream->compid,
	 									msg,
						       			time_keeper_get_micros()+1,
						       			"paf",
						       			estimator->pitch_accelero_filtered
						       			);
	mavlink_stream_send(mavlink_stream, msg);

	mavlink_msg_named_value_int_pack(	mavlink_stream->sysid,
	 									mavlink_stream->compid,
	 									msg,
						       			time_keeper_get_micros()+2,
						       			"pg",
						       			estimator->pitch_gyro
						       			);
	mavlink_stream_send(mavlink_stream, msg);


	mavlink_msg_named_value_int_pack(	mavlink_stream->sysid,
	 									mavlink_stream->compid,
	 									msg,
						       			time_keeper_get_micros()+2,
						       			"pgf",
						       			estimator->pitch_gyro_filtered
						       			);*/
}