#include<iostream>
extern "C"
{
#include "goldo_comm/cobs.h"
}

#include <time.h>
#include <stdlib.h>
#include<algorithm>

uint8_t s_data_buffer[1024 * 1024 * 10];
uint8_t s_encode_buffer[1024 * 1024 * 112];
uint8_t s_decode_buffer[1024 * 1024 * 10];

uint8_t s_message_buffer[512];
uint8_t s_encoded_message_buffer[520];
uint8_t s_decoded_message_buffer[512];

uint8_t data_1[] = { 0 };
uint8_t expected_1[] = { 1, 1, 0 };

uint8_t data_2[] = { 0, 0 };
uint8_t expected_2[] = { 1,1,1,0 };

uint8_t data_3[] = { 42 };
uint8_t expected_3[] = { 2, 42, 0 };

uint8_t data_4[] = { 42, 0 };
uint8_t expected_4[] = { 2, 42, 1,0 };

uint8_t data_5[] = { 0, 42 };
uint8_t expected_5[] = { 1, 2,42,0 };

uint8_t data_6[] = { 0, 42, 0 };
uint8_t expected_6[] = { 1, 2,42,1,0 };

uint8_t data_7[] = { 41, 42 ,0, 43 };
uint8_t expected_7[] = { 3, 41,42,2,43, 0 };

uint8_t data_8[] = { 41, 42 ,0, 43, 0 };
uint8_t expected_8[] = { 3, 41,42,2, 43, 1, 0 };

uint8_t data_9[254];
uint8_t expected_9[256];

uint8_t data_10[255];
uint8_t expected_10[258];

uint8_t data_11[270];
uint8_t expected_11[273];



/*
uint8_t data_5[255];
uint8_t expected_5[258];

uint8_t data_6[256];
uint8_t expected_6[260];*/



struct buffer_sink
{
	goldo_comm_sink_ops* vtable;
	uint8_t* beg_ptr;
	uint8_t* end_ptr;
	uint8_t* ptr;
};

struct msg_buffer_sink
{
	goldo_comm_sink_ops* vtable;
	uint8_t* beg_ptr;
	uint8_t* end_ptr;
	uint8_t* ptr;
	uint8_t* msg_ptr;
};

uint8_t* buffer_sink_alloc_buffer(size_t* size, void* user_ptr);
void buffer_sink_write_data(size_t size, void* user_ptr);
void buffer_sink_event_callback(enum GOLDO_COMM_SINK_EVENT evt, void*);

uint8_t* msg_buffer_sink_alloc_buffer(size_t* size, void* user_ptr);
void msg_buffer_sink_write_data(size_t size, void* user_ptr);
void msg_buffer_sink_event_callback(enum GOLDO_COMM_SINK_EVENT evt, void*);

struct goldo_comm_sink_ops g__vtable__buffer_sink = {
	NULL,
	&buffer_sink_alloc_buffer,
	&buffer_sink_write_data,
	&buffer_sink_event_callback
};

struct goldo_comm_sink_ops g__vtable__msg_buffer_sink = {
	NULL,
	&msg_buffer_sink_alloc_buffer,
	&msg_buffer_sink_write_data,
	&msg_buffer_sink_event_callback
};

void buffer_sink_init(struct buffer_sink* sink, uint8_t* buffer, size_t size)
{
	sink->vtable = &g__vtable__buffer_sink;
	sink->beg_ptr = buffer;
	sink->ptr = sink->beg_ptr;
	sink->end_ptr = sink->beg_ptr + size;
}

uint8_t* buffer_sink_alloc_buffer(size_t* size, void* user_ptr)
{
	struct buffer_sink* s = (buffer_sink*)user_ptr;
	*size = std::min<size_t>(*size, s->end_ptr - s->ptr);
	return s->ptr;
}

void buffer_sink_write_data(size_t size, void* user_ptr)
{
	struct buffer_sink* s = (buffer_sink*)user_ptr;
	s->ptr += size;
}

void buffer_sink_event_callback(enum GOLDO_COMM_SINK_EVENT evt, void*)
{

}

void msg_buffer_sink_init(struct msg_buffer_sink* sink, uint8_t* buffer, size_t size)
{
	sink->vtable = &g__vtable__msg_buffer_sink;
	sink->beg_ptr = buffer;
	sink->ptr = sink->beg_ptr + 2;
	sink->msg_ptr = sink->beg_ptr;
	sink->end_ptr = sink->beg_ptr + size;
}

uint8_t* msg_buffer_sink_alloc_buffer(size_t* size, void* user_ptr)
{
	struct msg_buffer_sink* s = (msg_buffer_sink*)user_ptr;
	*size = std::min<size_t>(*size, s->end_ptr - s->ptr - 2);
	return s->ptr;
}

void msg_buffer_sink_write_data(size_t size, void* user_ptr)
{
	struct msg_buffer_sink* s = (msg_buffer_sink*)user_ptr;
	s->ptr += size;
}

void msg_buffer_sink_event_callback(enum GOLDO_COMM_SINK_EVENT evt, void* user_ptr)
{
	struct msg_buffer_sink* s = (msg_buffer_sink*)user_ptr;
	size_t bs = s->ptr - s->beg_ptr;
	*(uint16_t*)(s->msg_ptr) = s->ptr - s->msg_ptr - 2;
	s->msg_ptr = s->ptr;
	s->ptr += 2;
}

bool test_encode(const char* test_name, const uint8_t* data, const uint8_t* expected, size_t data_size, size_t expected_size)
{
	goldo_cobs_encoder_state encoder_state;
	buffer_sink sink;


	goldo_cobs_encoder_init(&encoder_state, &sink);
	buffer_sink_init(&sink, s_encode_buffer, sizeof(s_encode_buffer));


	const uint8_t* ptr = data;
	const uint8_t* ptr_end = data + data_size;

	while (ptr != ptr_end)
	{
		ptr += goldo_cobs_encoder_encode(&encoder_state, ptr, ptr_end - ptr);
	}

	goldo_cobs_encoder_end_packet(&encoder_state);
	int status = memcmp(s_encode_buffer, expected, expected_size);

	if (status == 0)
	{
		std::cout << test_name << ": success\n";
		return true;
	}
	else
	{
		std::cout << test_name << ": fail\n";
		return false;
	}
};

bool test_decode(const char* test_name, const uint8_t* data, const uint8_t* expected, size_t data_size, size_t expected_size)
{
	goldo_cobs_decoder_state decoder_state;
	msg_buffer_sink sink;


	goldo_cobs_decoder_init(&decoder_state, &sink);
	msg_buffer_sink_init(&sink, s_encode_buffer, sizeof(s_encode_buffer));


	const uint8_t* ptr = expected;
	const uint8_t* ptr_end = expected + expected_size;

	while (ptr != ptr_end)
	{
		ptr += goldo_cobs_decoder_decode(&decoder_state, ptr, ptr_end - ptr);
	}

	int status = memcmp(s_encode_buffer + 2, data, data_size);

	if (status == 0 && *(uint16_t*)s_encode_buffer == data_size)
	{
		std::cout << test_name << ": success\n";
		return true;
	}
	else
	{
		std::cout << test_name << ": fail\n";
		return false;
	}
};

bool test_encode_decode()
{
	goldo_cobs_encoder_state encoder_state;
	goldo_cobs_decoder_state decoder_state;
	buffer_sink sink;
	msg_buffer_sink msg_sink;

	// Initialize encoder and decoder

	goldo_cobs_encoder_init(&encoder_state, &sink);
	goldo_cobs_decoder_init(&decoder_state, &msg_sink);
	buffer_sink_init(&sink, s_encode_buffer, sizeof(s_encode_buffer));
	msg_buffer_sink_init(&msg_sink, s_decode_buffer, sizeof(s_decode_buffer));

	// Generate random data
	srand(6245864);
	for(int i=0; i < sizeof(s_data_buffer); i++)
	{
		s_data_buffer[i] = rand();
	}

	// Encode each message, decode it and compare
	uint8_t* ptr = s_data_buffer;
	uint8_t* ptr_decode = sink.beg_ptr;
	size_t sent_size = (*(uint16_t*)(ptr) & 0x1ffu) + 1;

	while (ptr + sent_size + 2 < s_data_buffer + sizeof(s_data_buffer))
	{
		// Encode message size in first two bytes preceding it
		*(uint16_t*)(ptr) = sent_size;
		uint8_t* ptr_beg = ptr;
		ptr += 2;
		uint8_t* ptr_end = ptr + sent_size;		

		// Encode message		
		while (ptr != ptr_end)
		{
			ptr += goldo_cobs_encoder_encode(&encoder_state, ptr, ptr_end - ptr);
		}
		goldo_cobs_encoder_end_packet(&encoder_state);

		// Decode message
		while (ptr_decode != sink.ptr)
		{
			size_t s = goldo_cobs_decoder_decode(&decoder_state, ptr_decode, sink.ptr - ptr_decode);
			ptr_decode += s;
		};

		// Verify message
		for (int i = ptr_beg - s_data_buffer; i < ptr_end - s_data_buffer; i++)
		{
			if (s_data_buffer[i] != s_decode_buffer[i])
			{
				std::cout << "test_encode_decode" << ": fail\n";
				return false;
			}
		}
		sent_size = (*(uint16_t*)(ptr) & 0x1ffu) + 1;
	}

	size_t total_sent_size = ptr - s_data_buffer;
	size_t total_received_size = msg_sink.msg_ptr - s_decode_buffer;
	size_t encoded_size = sink.ptr - sink.beg_ptr;
	
	std::cout << "test_encode_decode" << ": success\n";
	return true;
}

int main()
{
	srand(time(NULL));   // Initialization, should only be called once.

	// 254 nonzero bytes 
	expected_9[0] = 255;
	for (int i = 0; i < 254; i++)
	{
		data_9[i] = i + 1;
		expected_9[i + 1] = i + 1;
	}
	expected_9[255] = 0;



	// 254 nonzero bytes followed by one 0
	expected_10[0] = 255;
	for (int i = 0; i < 254; i++)
	{
		data_10[i] = i + 1;
		expected_10[i + 1] = i + 1;
	}
	data_10[254] = 0;

	expected_10[255] = 1;
	expected_10[256] = 1;
	expected_10[257] = 0;

	// 254 nonzero bytes followed by 16 nonzero bytes
	expected_11[0] = 255;
	for (int i = 0; i < 254; i++)
	{
		data_11[i] = i + 1;
		expected_11[i + 1] = i + 1;
	}
	for (int i = 0; i < 16; i++)
	{
		data_11[i+254] = i + 1;
		expected_11[i + 256] = i + 1;
	}

	expected_11[255] = 17;
	expected_11[272] = 0;


	bool s = true;

	
	s &= test_encode("test_encode_1", data_1, expected_1, sizeof(data_1), sizeof(expected_1));
	s &= test_encode("test_encode_2", data_2, expected_2, sizeof(data_2), sizeof(expected_2));
	s &= test_encode("test_encode_3", data_3, expected_3, sizeof(data_3), sizeof(expected_3));
	s &= test_encode("test_encode_4", data_4, expected_4, sizeof(data_4), sizeof(expected_4));
	s &= test_encode("test_encode_5", data_5, expected_5, sizeof(data_5), sizeof(expected_5));	
	s &= test_encode("test_encode_6", data_6, expected_6, sizeof(data_6), sizeof(expected_6));	
	s &= test_encode("test_encode_7", data_7, expected_7, sizeof(data_7), sizeof(expected_7));
	s &= test_encode("test_encode_8", data_8, expected_8, sizeof(data_8), sizeof(expected_8));
	s &= test_encode("test_encode_9", data_9, expected_9, sizeof(data_9), sizeof(expected_9));
	s &= test_encode("test_encode_10", data_10, expected_10, sizeof(data_10), sizeof(expected_10));
	s &= test_encode("test_encode_11", data_11, expected_11, sizeof(data_11), sizeof(expected_11));

	s &= test_decode("test_decode_1", data_1, expected_1, sizeof(data_1), sizeof(expected_1));
	s &= test_decode("test_decode_2", data_2, expected_2, sizeof(data_2), sizeof(expected_2));
	s &= test_decode("test_decode_3", data_3, expected_3, sizeof(data_3), sizeof(expected_3));
	s &= test_decode("test_decode_4", data_4, expected_4, sizeof(data_4), sizeof(expected_4));
	s &= test_decode("test_decode_5", data_5, expected_5, sizeof(data_5), sizeof(expected_5));
	s &= test_decode("test_decode_6", data_6, expected_6, sizeof(data_6), sizeof(expected_6));	
	s &= test_decode("test_decode_7", data_7, expected_7, sizeof(data_7), sizeof(expected_7));
	s &= test_decode("test_decode_8", data_8, expected_8, sizeof(data_8), sizeof(expected_8));
	s &= test_decode("test_decode_9", data_9, expected_9, sizeof(data_9), sizeof(expected_9));
	s &= test_decode("test_decode_10", data_10, expected_10, sizeof(data_10), sizeof(expected_10));
	s &= test_decode("test_decode_11", data_11, expected_11, sizeof(data_11), sizeof(expected_11));

	s &= test_encode_decode();


	if (s)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}