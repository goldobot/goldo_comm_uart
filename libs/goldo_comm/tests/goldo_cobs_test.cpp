#include<iostream>
extern "C"
{
#include "goldo_comm/cobs.h"
}

#include <time.h>
#include <stdlib.h>
#include<algorithm>

uint8_t s_encode_buffer[4096];
uint8_t s_packet_buffer[4096];
uint8_t s_decode_buffer[4096];
bool s_end_packet;

uint8_t data_11[] = { 0 };
uint8_t expected_11[] = { 1, 0 };

uint8_t data_12[] = { 42 };
uint8_t expected_12[] = { 2,42,0 };

uint8_t data_13[] = { 42, 0 };
uint8_t expected_13[] = { 2,42,1,0 };

uint8_t data_14[] = { 0, 42 };
uint8_t expected_14[] = { 1, 2,42,0 };

uint8_t data_15[] = { 42, 43 };
uint8_t expected_15[] = { 3,42,43,0 };

uint8_t data_1[] = { 0,1,0,1,2,3,0 };
uint8_t expected_1[] = { 1,2,1,4,1,2,3,1,0 };

uint8_t data_2[] = { 0,1,0,1,2,3 };
uint8_t expected_2[] = { 1,2,1,4,1,2,3,0 };

uint8_t data_3[254];
uint8_t expected_3[256];

uint8_t data_4[255];
uint8_t expected_4[257];

uint8_t data_5[255];
uint8_t expected_5[258];

uint8_t data_6[256];
uint8_t expected_6[260];



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
	s_end_packet = false;
}

void end_packet_callback(enum GOLDO_COMM_SINK_EVENT,void*)
{
	s_end_packet = true;
};

bool test_encode(const char* test_name, const uint8_t* data, const uint8_t* expected, size_t data_size, size_t expected_size)
{
	goldo_cobs_encoder_state encoder_state;

	struct goldo_comm_sink_ops ops{NULL, &get_output_buffer, &data_received, &end_packet_callback };
	buff_decoder d{ s_encode_buffer, s_encode_buffer + sizeof(s_encode_buffer), s_encode_buffer };

	goldo_cobs_encoder_init(&encoder_state, &ops, &d);


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

bool test_decode(const char* test_name, const uint8_t* data, const uint8_t* expected, size_t data_size, size_t expected_size)
{
	goldo_cobs_decoder_state decoder_state;

	goldo_comm_sink_ops ops{ NULL, &get_output_buffer, &data_received, &end_packet_callback };
	buff_decoder d{ s_encode_buffer, s_encode_buffer + sizeof(s_encode_buffer), s_encode_buffer };

	goldo_cobs_decoder_init(&decoder_state, &ops, &d);


	const uint8_t* ptr = data;
	const uint8_t* ptr_end = data + data_size;

	while (ptr != ptr_end)
	{
		ptr += goldo_cobs_decoder_decode(&decoder_state, ptr, ptr_end - ptr);
	}

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


int main()
{
	srand(time(NULL));   // Initialization, should only be called once.


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
	expected_6[258] = 0;
	
	
	test_encode("test_encode_1.1", data_11, expected_11, sizeof(data_11), sizeof(expected_11));
	test_encode("test_encode_1.2", data_12, expected_12, sizeof(data_12), sizeof(expected_12));
	test_encode("test_encode_1.3", data_13, expected_13, sizeof(data_13), sizeof(expected_13));
	test_encode("test_encode_1.4", data_14, expected_14, sizeof(data_14), sizeof(expected_14));
	test_encode("test_encode_1.5", data_15, expected_15, sizeof(data_15), sizeof(expected_15));

	test_encode("test_encode_2", data_2, expected_2, sizeof(data_2), sizeof(expected_2));
	test_encode("test_encode_3", data_3, expected_3, sizeof(data_3), sizeof(expected_3));
	test_encode("test_encode_4", data_4, expected_4, sizeof(data_4), sizeof(expected_4));
	test_encode("test_encode_5", data_5, expected_5, sizeof(data_5), sizeof(expected_5));
	test_encode("test_encode_6", data_6, expected_6, sizeof(data_6), sizeof(expected_6));

	test_decode("test_decode_1.1", expected_11, data_11, sizeof(expected_11), sizeof(data_11));
	test_decode("test_decode_1.2", expected_12, data_12, sizeof(expected_12), sizeof(data_12));
	test_decode("test_decode_1.3", expected_13, data_13, sizeof(expected_13), sizeof(data_13));
	test_decode("test_decode_1.4", expected_14, data_14, sizeof(expected_14), sizeof(data_14));
	test_decode("test_decode_1.5", expected_15, data_15, sizeof(expected_15), sizeof(data_15));

	test_decode("test_decode_1", expected_1, data_1, sizeof(expected_1), sizeof(data_1));
	test_decode("test_decode_2", expected_2, data_2, sizeof(expected_2), sizeof(data_2));
	test_decode("test_decode_3", expected_3, data_3, sizeof(expected_3), sizeof(data_3));
	test_decode("test_decode_4", expected_4, data_4, sizeof(expected_4), sizeof(data_4));
	test_decode("test_decode_5", expected_5, data_5, sizeof(expected_5), sizeof(data_5));
	
	//Random encoding and decoding test
	goldo_cobs_encoder_state encoder_state;
	goldo_cobs_decoder_state decoder_state;

	goldo_comm_sink_ops ops_encode{ NULL, &get_output_buffer, &data_received, &end_packet_callback };
	goldo_comm_sink_ops ops_decode{ NULL, &get_output_buffer, &data_received, &end_packet_callback };

	buff_decoder bd_encode{ s_encode_buffer, s_encode_buffer + sizeof(s_encode_buffer), s_encode_buffer };
	buff_decoder bd_decode{ s_decode_buffer, s_decode_buffer + sizeof(s_decode_buffer), s_decode_buffer };

	goldo_cobs_encoder_init(&encoder_state, &ops_encode, &bd_encode);
	goldo_cobs_decoder_init(&decoder_state, &ops_decode, &bd_decode);
	bool ok = true;

	size_t prev_msg_size = 0;
	for (int i = 0; i < 100000; i++)
	{
		// Prepare message
		size_t msg_size = (rand() % 1024);
		for (int j = 0; j < msg_size; j++)
		{
			s_packet_buffer[j] = rand();
		}

		// Encode message
		bd_encode.ptr = bd_encode.beg_ptr;

		const uint8_t* ptr = s_packet_buffer;
		const uint8_t* ptr_end = s_packet_buffer + msg_size;

		while (ptr != ptr_end)
		{
			ptr += goldo_cobs_encoder_encode(&encoder_state, ptr, ptr_end - ptr);
		}
		goldo_cobs_encoder_end_packet(&encoder_state);
		prev_msg_size = msg_size;

		// Decode message
		bd_decode.ptr = bd_decode.beg_ptr;

		ptr = bd_encode.beg_ptr;
		ptr_end = bd_encode.ptr;

		s_end_packet = false;

		while (ptr != ptr_end)
		{
			ptr += goldo_cobs_decoder_decode(&decoder_state, ptr, ptr_end - ptr);
		}
		ok = ok && (memcmp(s_packet_buffer, s_decode_buffer, msg_size) == 0);// && s_end_packet;
		if (!ok)
		{
			int foo = 1;
		}
	}
	return 0;
}