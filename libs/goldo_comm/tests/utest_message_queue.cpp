#include "goldo_comm/message_queue.hpp"
#include "gtest/gtest.h"
namespace {

	TEST(MessageQueue, Test) {
		goldo_comm::MessageQueue<12,4> message_queue;
		uint8_t buffer[16];
		memset(buffer, 0, sizeof(buffer));

		message_queue.push_message((uint8_t*)"test", 4);
		EXPECT_EQ(message_queue.message_size(), 4);
		EXPECT_EQ(message_queue.available_message_size(), 8);

		message_queue.push_message((uint8_t*)"foobar", 6);
		EXPECT_EQ(message_queue.message_size(), 4);
		EXPECT_EQ(message_queue.available_message_size(), 2);

		message_queue.pop_message(buffer, sizeof(buffer));
		EXPECT_EQ(message_queue.message_size(), 6);
		EXPECT_EQ(message_queue.available_message_size(), 6);

		message_queue.pop_message(buffer, 6);
		message_queue.pop_message();
		EXPECT_EQ(message_queue.available_message_size(), 12);

		message_queue.push_message((uint8_t*)"deadbeef", 8);
		int a = 0;

		
	}
}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}