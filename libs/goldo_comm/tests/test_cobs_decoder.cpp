#include<iostream>
extern "C"
{
#include "goldo_comm/cobs.h"
}

#include <time.h>
#include <stdlib.h>
#include<algorithm>

uint8_t s_encode_buffer[4096];

uint8_t data_1[] = { 0 };
uint8_t expected_1[] = { 1, 0 };

uint8_t data_2[] = { 42 };
uint8_t expected_2[] = { 2, 42, 0 };

uint8_t data_3[] = { 42, 0 };
uint8_t expected_3[] = { 2,42,1,0 };

uint8_t data_4[] = { 0, 42 };
uint8_t expected_4[] = { 1, 2,42,0 };

uint8_t data_5[] = { 42, 43 };
uint8_t expected_5[] = { 3,42,43,0 };

uint8_t data_6[] = { 0,1,0,1,2,3,0 };
uint8_t expected_6[] = { 1,2,1,4,1,2,3,1,0 };

uint8_t data_7[] = { 0,1,0,1,2,3 };
uint8_t expected_7[] = { 1,2,1,4,1,2,3,0 };

/*

uint8_t data_3[254];
uint8_t expected_3[256];

uint8_t data_4[255];
uint8_t expected_4[257];

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
	uint8_t* msg_ptr;
};

uint8_t* buffer_sink_alloc_buffer(size_t* size, void* user_ptr);
void buffer_sink_write_data(size_t size, void* user_ptr);
void buffer_sink_event_callback(enum GOLDO_COMM_SINK_EVENT evt, void*);

struct goldo_comm_sink_ops g__vtable__buffer_sink = {
	NULL,
	&buffer_sink_alloc_buffer,
    &buffer_sink_write_data,
	&buffer_sink_event_callback
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

	int foo = (uint8_t)(encoder_state.write_idx - encoder_state.code_idx);
	goldo_cobs_encoder_end_packet(&encoder_state);
	int status = memcmp(s_encode_buffer, expected, expected_size);
	
	if (status == 0)
	{
		std::cout << test_name << " " << (int)encoder_state.previous_block_type << " " << foo << ": success\n";
		return true;
	}
	else
	{
		std::cout << test_name << " " << (int)encoder_state.previous_block_type << " " << foo << ": fail\n";
		return false;
	}
};


int main()
{
	srand(time(NULL));   // Initialization, should only be called once.


	/*
	// 254 nonzero bytes 
	expected_3[0] = 255;
	for (int i = 0; i < 254; i++)
	{
		data_3[i] = i + 1;
		expected_3[i + 1] = i + 1;
	}
	expected_3[255] = 0;

	// 254 nonzero bytes followed by one 0
	expected_4[0] = 255;
	for (int i = 0; i < 254; i++)
	{
		data_4[i] = i + 1;
		expected_4[i + 1] = i + 1;
	}
	data_4[254] = 0;
	expected_4[255] = 1;
	expected_4[256] = 0;

	// 255 nonzero bytes
	expected_5[0] = 255;
	for (int i = 0; i < 254; i++)
	{
		data_5[i] = i + 1;
		expected_5[i + 1] = i + 1;
	}
	data_5[254] = 42;
	expected_5[255] = 2;
	expected_5[256] = 42;
	expected_5[257] = 0;

	// 254 nonzero bytes followed by one 0 followed by one nonzero
	expected_6[0] = 255;
	for (int i = 0; i < 254; i++)
	{
		data_6[i] = i + 1;
		expected_6[i + 1] = i + 1;
	}
	data_6[254] = 0;
	data_6[255] = 42;
	expected_6[255] = 1;
	expected_6[256] = 2;
	expected_6[257] = 42;
	expected_6[258] = 0;*/
	
	bool s = true;	
	s &= test_encode("test_encode_1", data_1, expected_1, sizeof(data_1), sizeof(expected_1));
	s &= test_encode("test_encode_2", data_2, expected_2, sizeof(data_2), sizeof(expected_2));
	s &= test_encode("test_encode_3", data_3, expected_3, sizeof(data_3), sizeof(expected_3));
	s &= test_encode("test_encode_4", data_4, expected_4, sizeof(data_4), sizeof(expected_4));
	s &= test_encode("test_encode_5", data_5, expected_5, sizeof(data_5), sizeof(expected_5));
	s &= test_encode("test_encode_6", data_6, expected_6, sizeof(data_6), sizeof(expected_6));
	s &= test_encode("test_encode_7", data_7, expected_7, sizeof(data_7), sizeof(expected_7));

	if (s)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}