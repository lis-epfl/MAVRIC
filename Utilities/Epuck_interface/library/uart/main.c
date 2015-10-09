#include "e_uart_char.h"

int main(void)
{
 	char car;
  	e_init_uart1();
  	e_send_uart1_char("\f\a", 2);		//new page on HyperTerminal
  	while(1)
  	{
  		e_send_uart1_char("Give a character:\r\n", 19);
 		// do nothing while the text is not sent and while nothing is comming from the user
  		while(e_uart1_sending() || !e_ischar_uart1()) {} 
  		e_getchar_uart1(&car);			// read the character entered...
  		e_send_uart1_char("You have wrote: ", 16);
  		e_send_uart1_char(&car, 1);		//... and resend him to uart.
  		e_send_uart1_char("\r\n\r\n",4);
  	}
}