#include <stdint.h>
#include<algorithm>

extern "C"
{
#include "goldo_comm/cobs.h"
}

uint8_t s_encode_buffer[4096];

struct buff_decoder
{
	uint8_t* beg_ptr;
	uint8_t* end_ptr;
	uint8_t* ptr;
};

uint8_t* get_output_buffer(size_t* size, void* user_ptr)
{
	struct buff_decoder* d = (struct buff_decoder*)user_ptr;
	*size = std::min<size_t>(*size, d->end_ptr - d->ptr);
	return d->ptr;
}

void data_received(size_t size, void* user_ptr)
{
	struct buff_decoder* d = (struct buff_decoder*)user_ptr;
	d->ptr += size;
}

void end_packet_callback(enum GOLDO_COMM_SINK_EVENT, void*)
{
};

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size) {

	goldo_cobs_encoder_state encoder_state;

	struct goldo_cobs_sink_ops ops{ &get_output_buffer, &data_received, &end_packet_callback };
	buff_decoder d{ s_encode_buffer, s_encode_buffer + sizeof(s_encode_buffer), s_encode_buffer };

	goldo_cobs_encoder_init(&encoder_state, &ops, &d);
	
	const uint8_t* ptr = Data;
	const uint8_t* ptr_end = Data + Size;


	while (ptr+1 < ptr_end)
	{
		uint16_t size = (*(uint16_t*)ptr) % 512;
		ptr+=2;
		const uint8_t* ptr_end_msg = std::min(ptr + size, ptr_end);
		while (ptr != ptr_end_msg)
		{
			ptr += goldo_cobs_encoder_encode(&encoder_state, ptr, ptr_end_msg - ptr);
		}
		goldo_cobs_encoder_end_packet(&encoder_state);		
	}

  return 0;  // Non-zero return values are reserved for future use.
}
