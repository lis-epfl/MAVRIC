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
 
#include "boardsupport.hpp"
#include "central_data.hpp"
#include "mavlink_telemetry.hpp"
#include "tasks.hpp"

#include "file_dummy.hpp"
#include "file_flash_avr32.hpp"
#include "serial_usb_avr32.hpp"

extern "C" 
{
	#include "led.h"
	#include "time_keeper.h"
	#include "print_util.h"
	#include "piezo_speaker.h"
}

#include "dbg.hpp"


void initialisation(Central_data& central_data, Megafly_rev4& board) 
{	
	bool init_success = true;
	
	// Legacy board initialisation (TODO: remove)
	init_success &= boardsupport_init(&central_data);



	// piezo_speaker_startup_bumblebot();
	// Link debug stream to USB serial
	
	// Serial_usb_avr32 usb({});
	// usb.init();
	// usb_serial = &usb;

	// // usb_serial->init();
	// debug_stream.get   = NULL; 
	// debug_stream.flush = NULL; 
	// debug_stream.buffer_empty = NULL; 
	// debug_stream.put = &serial_put_stream; 
	// debug_stream.data = NULL;
	// central_data.debug_out_stream = &debug_stream;

	// usb_serial = Serial_usb_avr32({});
	// usb_serial = usb;
	// usb_serial = Serial_usb_avr32({});
	// usb_serial.init();

	// debug_stream.get   = NULL; 
	// debug_stream.flush = NULL; 
	// debug_stream.buffer_empty = NULL; 
	// debug_stream.put = &serial_put_stream; 
	// debug_stream.data = NULL;
	// central_data.debug_out_stream = &debug_stream;


	// print_util_dbg_print_init(central_data.debug_out_stream);
	// print_util_dbg_print("Debug stream initialised\r\n");

	// piezo_speaker_startup_bumblebot();



	// New board initialisation
	init_success &= board.init();

	// Init central data
	init_success &= central_data.init(board.uart0, board.bmp085, board.spektrum_satellite);

	init_success &= mavlink_telemetry_add_onboard_parameters(&central_data.mavlink_communication.onboard_parameters, &central_data);

	bool read_from_flash_result = onboard_parameters_read_parameters_from_storage(&central_data.mavlink_communication.onboard_parameters);

	if (read_from_flash_result)
	{
		simulation_switch_from_reality_to_simulation(&central_data.sim_model);
	}

	init_success &= mavlink_telemetry_init(&central_data, &board);

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

	print_util_dbg_print("OK. Starting up.\r\n");
}


int main (void)
{
	// File_dummy file("flash.bin");
	File_flash_avr32 file("flash.bin");

	imu_t imu;
	Megafly_rev4 board 	= Megafly_rev4(imu);
	Central_data cd 	= Central_data( imu, 
										board.i2c1, 
										board.bmp085,
										board.lsm330dlc,
										board.magnetometer,
										file);
	
	initialisation(cd, board);

	/* create console */
	Console<Serial> console_usb(board.uart_usb);

	/* init debug facilities*/
	dbg::init(console_usb);

	/* examples how to use dbg: */
	dbg::print("we put an int: ");
	dbg::print(23);
	dbg::print(" or a float: ");
	dbg::print(1.4f);
	dbg::println(" this ends the line");
	dbg::dout() << "and we can do it all in a line: int " << 23 << " float " << 1.4f << "and even bools " << false;
	dbg::dout() << "and end the line " << endl;
	while (1 == 1) 
	{
		scheduler_update(&cd.scheduler);
	}

	return 0;
}
