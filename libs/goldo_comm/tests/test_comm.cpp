#include<iostream>

#include "goldo_comm/comm.hpp"


#include <time.h>
#include <stdlib.h>
#include<algorithm>

uint8_t s_sent_buffer[1024*1024];
uint8_t s_recv_buffer[1024*1024];

uint8_t s_intermediate_buffer[128];

int main()
{
	srand(time(NULL));   // Initialization, should only be called once.
	for (int i = 0; i < sizeof(s_sent_buffer); i++)
	{
		s_sent_buffer[i] = rand();
	}


	uint32_t timestamp = 0;
	struct goldo_comm_state comm_state_1;
	struct goldo_comm_state comm_state_2;

	
	struct goldo_comm_sink_ops ops {
		NULL,
		&goldo_comm_alloc_buffer,
		&goldo_comm_write_data,
		&goldo_comm_event_callback
	};



	goldo_comm_init(&comm_state_1, &comm_state_2,timestamp);
	goldo_comm_init(&comm_state_2, &comm_state_1, timestamp);

	uint8_t* recv_ptr = s_recv_buffer;
	uint8_t* sent_ptr = s_sent_buffer;

	for(int i = 0; i < 10000; i++)
	{
		if (i % 10 == 0)
		{
			size_t sent_size = (*(uint16_t*)(sent_ptr) & 0x0000001fu) + 1;
			sent_size = 1;

			if (sent_ptr + sent_size + 2 <= s_sent_buffer + sizeof(s_sent_buffer))
			{
				goldo_comm_send(&comm_state_1, sent_ptr + 2, sent_size);
				*(uint16_t*)(sent_ptr) = sent_size;
				sent_ptr += sent_size + 2;
			};
		}

		goldo_comm_spin(&comm_state_1, timestamp);
		goldo_comm_spin(&comm_state_2, timestamp);

		int size_received = goldo_comm_recv(&comm_state_2,recv_ptr + 2, 512);
		if (size_received)
		{
			*(uint16_t*)(recv_ptr) = size_received;
			recv_ptr += size_received + 2;
		}

		for (int i = 0; i < sent_ptr - s_sent_buffer; i++)
		{
			if (s_sent_buffer[i] != s_recv_buffer[i])
			{
				int foo = 6;
			}
		}
		timestamp++;
	}
	return 0;
}