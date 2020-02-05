#include <stdint.h>
#include<algorithm>

extern "C"
{
#include "goldo_cobs.h"
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

	goldo_cobs_decoder_state decoder_state;

	struct goldo_cobs_sink_ops ops{ &get_output_buffer, &data_received, &end_packet_callback };
	buff_decoder d{ s_encode_buffer, s_encode_buffer + sizeof(s_encode_buffer), s_encode_buffer };

	goldo_cobs_decoder_init(&decoder_state, &ops, &d);
	
	const uint8_t* ptr = Data;
	const uint8_t* ptr_end = Data + Size;


	while (ptr < ptr_end)
	{
		ptr+= goldo_cobs_decoder_decode(&decoder_state, ptr, ptr_end - ptr);	
	}

  return 0;  // Non-zero return values are reserved for future use.
}
