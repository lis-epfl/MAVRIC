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
 * \file main.cpp
 * 
 * \author MAV'RIC Team
 *   
 * \brief Main file
 *
 ******************************************************************************/
 
#include "central_data.hpp"
#include "mavlink_telemetry.hpp"
#include "tasks.hpp"

#include "file_dummy.hpp"
#include "file_flash_avr32.hpp"
#include "serial_usb_avr32.hpp"

#include "dynamic_model_quad_diag.hpp"
#include "simulation.hpp"
#include "adc_dummy.hpp"

extern "C" 
{
	#include "led.h"
	#include "time_keeper.h"
	#include "print_util.h"
	#include "piezo_speaker.h"

	#include "servos.h"
	#include "servos_default_config.h"
}

#include "dbg.hpp"

void initialisation(Central_data& central_data, Megafly_rev4& board) 
{	
	bool init_success = true;

	// Board initialisation
	init_success &= board.init();

	// Init central data
	init_success &= central_data.init();

	init_success &= mavlink_telemetry_add_onboard_parameters(&central_data.mavlink_communication.onboard_parameters, &central_data);

	// Try to read from flash, if unsuccessful, write to flash
	if( onboard_parameters_read_parameters_from_storage(&central_data.mavlink_communication.onboard_parameters) == false )
	{
		onboard_parameters_write_parameters_to_storage(&central_data.mavlink_communication.onboard_parameters);
		init_success = false; 
	}

	init_success &= mavlink_telemetry_init(&central_data);

	central_data.state.mav_state = MAV_STATE_STANDBY;	
	
	init_success &= tasks_create_tasks(&central_data);	

	if (init_success)
	{
		piezo_speaker_quick_startup();
		
		// Switch off red LED
		LED_Off(LED2);
	}
	else
	{
		piezo_speaker_critical_error_melody();
	}

	print_util_dbg_print("[MAIN] OK. Starting up.\r\n");
}

#include <array>

int main (void)
{
	// -------------------------------------------------------------------------
	// Create board
	// -------------------------------------------------------------------------
	megafly_rev4_conf_t board_config = megafly_rev4_default_config();
	Megafly_rev4 board = Megafly_rev4( board_config );


	// -------------------------------------------------------------------------
	// Create simulation
	// -------------------------------------------------------------------------
	// Simulated servos
	servos_t sim_servos;
	servos_init(&sim_servos, servos_default_config() );
	servos_set_value_failsafe( &sim_servos );
	
	// Simulated dynamic model
	Dynamic_model_quad_diag sim_model 	= Dynamic_model_quad_diag(sim_servos);
	Simulation sim 						= Simulation(sim_model);
	
	// Simulated battery
	Adc_dummy 	sim_adc_battery = Adc_dummy(11.1f);
	Battery 	sim_battery 	= Battery(sim_adc_battery);

	// Simulated IMU
	Imu 		sim_imu 		= Imu(  sim.accelerometer(),
										sim.gyroscope(),
										sim.magnetometer() );


	// -------------------------------------------------------------------------
	// Create central data
	// -------------------------------------------------------------------------
	// Create central data using real sensors
	Central_data cd = Central_data( board.imu, 
									board.bmp085,
									board.gps_ublox, 
									// board.sonar_i2cxl,		// Warning:
									sim.sonar(),				// this is simulated
									board.uart0,
									board.spektrum_satellite,
									board.file_flash,
									board.battery,
									// sim_battery,
									board.servos );


	// Create central data with simulated sensors
	// Central_data cd = Central_data( sim_imu, 
	// 								sim.barometer(),
	// 								sim.gps(), 
	// 								sim.sonar(),
	// 								board.uart0, 				// mavlink serial
	// 								board.spektrum_satellite,
	// 								board.file_flash,
	// 								sim_battery,
	// 								sim_servos);

	initialisation(cd, board);

	while (1 == 1) 
	{
		scheduler_update(&cd.scheduler);
	}

	return 0;
}
