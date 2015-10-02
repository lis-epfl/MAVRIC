#include "serial_udp.hpp"
#include "mavlink_communication.hpp"
#include "time_keeper.h"
#include "central_data_emu.hpp"

int main()
{
	// Init udp
	Serial_udp udp = Serial_udp();
	udp.init();

	// Init mavlink stream
	mavlink_stream_t 		mavlink_stream;
	mavlink_stream_conf_t 	mavlink_stream_config;
	mavlink_stream_config.sysid  = 123;
	mavlink_stream_config.compid = 50;
	mavlink_stream_init(&mavlink_stream, &mavlink_stream_config, &udp);

	Central_data cd = Central_data();


	while(1)
	{
		// Receive messages
		mavlink_stream_receive(&mavlink_stream);
		if( mavlink_stream.msg_available==true)
		{
			mavlink_message_t msg = mavlink_stream.rec.msg;
			printf("Rec msg with ID %i \n", msg.msgid);
			mavlink_stream.msg_available = false;
		}

		// Send hearbeat
		mavlink_message_t msg;
		mavlink_msg_heartbeat_pack( mavlink_stream.sysid, mavlink_stream.compid, &msg,
						       		0,  // uint8_t type, 
						       		0,  // uint8_t autopilot, 
						       		0,  // uint8_t base_mode, 
						       		0,  // uint32_t custom_mode, 
						       		0); //uint8_t system_status)
		mavlink_stream_send(&mavlink_stream, &msg);


		time_keeper_delay_ms(200);
	}

	return 0;
}