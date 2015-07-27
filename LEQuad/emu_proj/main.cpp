#include "serial_udp.hpp"
// #include <unistd.h>
#include "stdio.h"


#include "udp_client_server.hpp"

int main()
{

	Serial_udp udp = Serial_udp();

	udp.init();

	uint8_t i = 0; 
	while(1)
	{
		udp.write(&i);
		// printf("%i writeable \n", udp.writeable());
		// udp.flush();
		i +=1;


		// printf("%i, readable \n", udp.readable());
		if( udp.readable() > 50)
		{
			uint32_t n_bytes = udp.readable(); 
			uint8_t bytes[n_bytes];
			udp.read(bytes, n_bytes);
			
			for (int i = 0; i < n_bytes; ++i)
			{
				printf("%i ", bytes[i]);
			}
			printf("\n");
		}
	}


	// udp_server udp_rx("127.0.0.1", 14555);
	// udp_client udp_tx("127.0.0.1", 14550);

	// while(1)
	// {
	// 	uint8_t byte[17];
	// 	// udp.recv((char*)byte, 15);
	// 	// printf("Received: \n");
	// 	// for (int i = 0; i < 17; ++i)
	// 	// {
	// 	// 	printf("%i ", byte[i]);
	// 	// }
	// 	// printf("\n");


	// 	// change sys id
	// 	byte[3] = 123;
	// 	udp_tx.send((char*)byte, 17);
	// }

	return 0;
}