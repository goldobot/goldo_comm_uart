#include "goldo_comm/cobs.h"
#include <stdlib.h>

void goldo_cobs_decoder_init(
	struct goldo_cobs_decoder_state* s,
	void* sink_ptr
)
{
	s->vtable = NULL;
	s->counter = 0;
	s->code = 0;
	s->sink_ptr = sink_ptr;
}

size_t goldo_cobs_decoder_decode(
	struct goldo_cobs_decoder_state* s,
	const uint8_t* buffer,
	size_t size)
{
	const uint8_t* ptr = buffer;
	const uint8_t* end = ptr + size;

	/* Allocate output buffer.*/
	size_t out_size = size;
	uint8_t* out_ptr = (*(struct goldo_comm_sink_ops**)s->sink_ptr)->alloc_buffer(&out_size, s->sink_ptr);
	uint8_t* out_beg = out_ptr;
	uint8_t* out_end = out_ptr + out_size;


	for(;;)
	{
		/* Input buffer finished */
		if (ptr == end)
		{
			break;
		}
		/* Output buffer filled */
		if (out_ptr == out_end)
		{
			break;
		}
		/* Read until nonzero value */
		if (s->code == 0)
		{
			s->code = *ptr++;
			s->counter = s->code;
			continue;
		}

		if (s->counter == 1)
		{
			if (s->code != 255 && *ptr != 0)
			{
				*out_ptr++ = 0;
			}
			s->code = *ptr++;
			s->counter = s->code;
		}
		else
		{
			*out_ptr++ = *ptr++;
			s->counter--;
		}
		
		if (s->code == 0)
		{
			(*(struct goldo_comm_sink_ops**)s->sink_ptr)->write_data(out_ptr - out_beg, s->sink_ptr);
			(*(struct goldo_comm_sink_ops**)s->sink_ptr)->event_callback(GOLDO_COMM_SINK_EVENT_END_PACKET, s->sink_ptr);
			/* reallocate buffer */
			out_size = ptr - buffer;
			out_ptr = (*(struct goldo_comm_sink_ops**)s->sink_ptr)->alloc_buffer(&out_size, s->sink_ptr);
			out_beg = out_ptr;
			out_end = out_ptr + out_size;
		}		
	}
	(*(struct goldo_comm_sink_ops**)s->sink_ptr)->write_data(out_ptr - out_beg, s->sink_ptr);
	return ptr - buffer;
}