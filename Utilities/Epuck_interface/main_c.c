/*******************************/
/** Very simple LEDs blinking **/
/*******************************/

#include "p30f6014A.h"
#include "e_epuck_ports.h"
#include "e_init_port.h"

#include "uart\e_uart_char.h"
#include "motor_led\e_motors.h"

#include "mavlink\include\common\mavlink.h"
#include <string.h>
#include <stdio.h>

#define DELAY 500

// set configuration bits for MPLAB (e.g oscillator)
/*_FOSC(CSW_FSCM_OFF & XT_PLL8);
_FWDT(WDT_OFF);
_FBORPOR(PBOR_OFF & MCLR_EN);
_FGS(CODE_PROT_OFF);
*/

// Example variable, by declaring them static they're persistent
// and will thus track the system state
static int packet_drops = 0;
long timer = 0;

mavlink_message_t msg;
mavlink_status_t status;	
int16_t pitch = 0; //-10'000 to 10'000
int16_t roll = 0;	
	
/**
* @brief Receive communication packets and handle them
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/
static void communication_receive(void);

int main() {

	e_init_port();
	e_init_uart2();
	e_init_motors();
	
	//configure LEDs pin direction
	LED0_DIR = OUTPUT_PIN;
	LED0 = 0;
	LED4_DIR = OUTPUT_PIN;
	LED4 = 0;
	
	// Timer1 settings: TMR1 on, prescale 1:256
	T1CON = 0x8030;
	TMR1 = 0;
	
	while (1) {

		communication_receive(); 
		//for(timer = 0; timer < DELAY; timer++);
		
		if(pitch == 0)
		{
			LED0 = 0;
			LED4 = 0;
		}
		else if(pitch > 0)
		{
			LED0 = 1;
			LED4 = 0;
		}
		else
		{
			LED0 = 0;
			LED4 = 1;
		}
		
		if(TMR1 >= DELAY) //since e_set_speed_*() cannot be called too often
		{
			e_set_speed_left((pitch+roll)/2);
			e_set_speed_right((pitch-roll)/2);
			TMR1 = 0; //reset timer 1
		}	
	}								
}

static void communication_receive(void)
{
	// COMMUNICATION THROUGH EXTERNAL UART PORT (XBee serial)
 
	while(e_ischar_uart2())
	{
		char c;
		e_getchar_uart2(&c);			// read the character;
		
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
 			
			switch(msg.msgid)
			{
		        case MAVLINK_MSG_ID_HEARTBEAT:
		        {
			  		// E.g. read GCS heartbeat and go into
                    // comm lost mode if timer times out
                    //LED0 = !LED0; 
		        }
			        break;
				case MAVLINK_MSG_ID_COMMAND_LONG:
					// EXECUTE ACTION
					break;
				case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
		        {
			        if(mavlink_msg_rc_channels_scaled_get_port(&msg) == 0)
			  		{
			  			//get roll and pitch and scaled them from 10'000 range to a range of 1'000 for motors input
			  			roll = mavlink_msg_rc_channels_scaled_get_chan2_scaled(&msg)/10;
			  			pitch = mavlink_msg_rc_channels_scaled_get_chan3_scaled(&msg)/-10;
			  		}	 
		        }
			        break;
				default:
					//Do nothing
					break;
			}
		}
 
		// And get the next one
	}	
 
	// Update global packet drops counter
	packet_drops += status.packet_rx_drop_count;
}

