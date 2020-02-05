#include "goldo_comm/cobs_encoder.hpp"
#include "gtest/gtest.h"

using namespace goldo_comm;

namespace {

	
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

	TEST(CobsEncoder, Test1) {
		CobsEncoder cobs_encoder;

		uint8_t data[] = { 0 };
		uint8_t expected[] = { 1, 1, 0 };
		uint8_t encoded[3];
		memset(encoded, 0, sizeof(encoded));

		size_t input_size = sizeof(data);
		size_t encoded_size = sizeof(encoded);

		cobs_encoder.encode_message(data, input_size, encoded, encoded_size);
		EXPECT_EQ(encoded_size, sizeof(encoded));
		EXPECT_EQ(memcmp(encoded, expected, sizeof(expected)), 0);		
	}

	TEST(CobsEncoder, Test2) {
		CobsEncoder cobs_encoder;

		uint8_t data[] = { 0, 0 };
		uint8_t expected[] = { 1,1,1,0 };
		uint8_t encoded[4];
		memset(encoded, 0, sizeof(encoded));

		size_t input_size = sizeof(data);
		size_t encoded_size = sizeof(encoded);

		cobs_encoder.encode_message(data, input_size, encoded, encoded_size);
		EXPECT_EQ(encoded_size, sizeof(encoded));
		EXPECT_EQ(memcmp(encoded, expected, sizeof(expected)), 0);
	}

	TEST(CobsEncoder, Test3) {
		CobsEncoder cobs_encoder;

		uint8_t data[] = { 42 };
		uint8_t expected[] = { 2, 42, 0 };
		uint8_t encoded[3];
		memset(encoded, 0, sizeof(encoded));

		size_t input_size = sizeof(data);
		size_t encoded_size = sizeof(encoded);

		cobs_encoder.encode_message(data, input_size, encoded, encoded_size);
		EXPECT_EQ(encoded_size, sizeof(encoded));
		EXPECT_EQ(memcmp(encoded, expected, sizeof(expected)), 0);
	}
}  // namespace

