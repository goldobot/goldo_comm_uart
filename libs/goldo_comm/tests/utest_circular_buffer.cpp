#include "goldo_comm/circular_buffer.hpp"
#include "gtest/gtest.h"
namespace {

	TEST(CircularBuffer, Test) {
		goldo_comm::CircularBuffer<8> circular_buffer;
		uint8_t buffer[16];
		memset(buffer, 0, sizeof(buffer));

		EXPECT_EQ(circular_buffer.size(), 0);
		EXPECT_EQ(circular_buffer.available_space(), 8);

		circular_buffer.push((uint8_t*)"test", 4);

		EXPECT_EQ(circular_buffer.size(), 4);
		EXPECT_EQ(circular_buffer.available_space(), 4);

		EXPECT_EQ(circular_buffer.pop(buffer, 6), 4);
		EXPECT_EQ(memcmp("test", buffer, 4), 0);
		EXPECT_EQ(circular_buffer.size(), 0);

		circular_buffer.push((uint8_t*)"deadbeef", 8);
		EXPECT_EQ(circular_buffer.size(), 8);
		EXPECT_EQ(circular_buffer.available_space(), 0);
		EXPECT_EQ(circular_buffer.full(), true);

		EXPECT_EQ(circular_buffer.pop(buffer, 6), 6);
		EXPECT_EQ(memcmp("deadbe", buffer, 6), 0);
		EXPECT_EQ(circular_buffer.size(), 2);

		EXPECT_EQ(circular_buffer.pop(buffer, 6), 2);
		EXPECT_EQ(memcmp("ef", buffer, 2), 0);
		EXPECT_EQ(circular_buffer.size(), 0);
	}
}  // namespace
