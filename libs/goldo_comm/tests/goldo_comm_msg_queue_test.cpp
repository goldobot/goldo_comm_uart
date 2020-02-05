#include<iostream>
extern "C"
{
#include "goldo_comm/msg_queue.h"
}

#include <time.h>
#include <stdlib.h>
#include<algorithm>

int main()
{
	srand(time(NULL));   // Initialization, should only be called once.
	struct goldo_comm_msg_queue msg_queue;
	goldo_comm_msg_queue_init(&msg_queue);
	goldo_comm_msg_queue_push_data(&msg_queue, (const uint8_t*)"goldobot", 8 );
	goldo_comm_msg_queue_end_msg(&msg_queue);
	return 0;
}