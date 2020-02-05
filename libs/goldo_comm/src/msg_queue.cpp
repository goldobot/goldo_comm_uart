#include "goldo_comm/msg_queue.hpp"

struct goldo_comm_sink_ops g_goldo_comm_msg_queue_sink_ops = {
	NULL,
	&goldo_comm_msg_queue_alloc_buffer,
	&goldo_comm_msg_queue_write_data,
	&goldo_comm_msg_queue_event_callback
};

void goldo_comm_msg_queue_init(struct goldo_comm_msg_queue* q)
{
	q->sink_ops = &g_goldo_comm_msg_queue_sink_ops;
	q->buffer_size = sizeof(q->buffer);
	q->read_idx = 0;
	q->write_idx = 0;
	q->msg_idx[0] = 0;
	q->msg_idx[1] = 0;
	q->buffer_read_idx = 0;
	q->buffer_write_idx = 0;
}

void goldo_comm_msg_queue_push_data(struct goldo_comm_msg_queue* q, const uint8_t* buffer, size_t size)
{
	const uint8_t* in_ptr = buffer;
	const uint8_t* in_end = in_ptr + size;	

	while (in_ptr != in_end)
	{
		q->buffer[q->buffer_write_idx++] = *in_ptr++;
		if (q->buffer_write_idx == q->buffer_size)
		{
			q->buffer_write_idx = 0;
		}
	}
	q->msg_idx[goldo_comm_msg_queue_next_index(q->write_idx)] = q->buffer_write_idx;
}

void goldo_comm_msg_queue_end_msg(struct goldo_comm_msg_queue* q)
{
	goldo_comm_msg_queue_advance_index(q, write_idx);
	q->msg_idx[q->write_idx] = q->buffer_write_idx;
}

uint8_t* goldo_comm_msg_queue_alloc_buffer(size_t* size, void* user_ptr)
{
	struct goldo_comm_msg_queue* q = (struct goldo_comm_msg_queue*) user_ptr;
	if (*size > q->buffer_size - q->buffer_write_idx)
	{
		*size = q->buffer_size - q->buffer_write_idx;
	}
	
	return q->buffer + q->buffer_write_idx;
}

void goldo_comm_msg_queue_write_data(size_t size, void* user_ptr)
{
	struct goldo_comm_msg_queue* q = (struct goldo_comm_msg_queue*) user_ptr;
	q->buffer_write_idx = (q->buffer_write_idx + size);
	if (q->buffer_write_idx == q->buffer_size)
	{
		q->buffer_write_idx = 0;
	}
}

void goldo_comm_msg_queue_event_callback(enum GOLDO_COMM_SINK_EVENT event, void* user_ptr)
{
	if (event == GOLDO_COMM_SINK_EVENT_END_PACKET)
	{
		struct goldo_comm_msg_queue* q = (struct goldo_comm_msg_queue*) user_ptr;
		goldo_comm_msg_queue_advance_index(q, write_idx);
		q->msg_idx[q->write_idx] = q->buffer_write_idx;
	}	
}


size_t goldo_comm_msg_queue_back_message_size(struct goldo_comm_msg_queue* q)
{
	if (q->read_idx == q->write_idx)
	{
		return 0;
	}
	else
	{
		return q->msg_idx[goldo_comm_msg_queue_next_index(q->read_idx)] - q->msg_idx[q->read_idx];
	}
}