#include "goldo_comm/comm.hpp"
#include "gtest/gtest.h"

using namespace goldo_comm;

namespace {

	TEST(Comm, Test) {
		Comm comm1;
		LoopbackCommHal hal;

		uint8_t buffer[16];

		comm1.setHal(&hal);

		comm1.send((void*)"test", 4);
		comm1.send((void*)"foo", 3);
		comm1.spin(chrono::milliseconds(1));
		comm1.send((void*)"deadbeef", 8);
		comm1.spin(chrono::milliseconds(1));
		comm1.recv(buffer, 16);
		comm1.recv(buffer, 16);
		comm1.recv(buffer, 16);
		size_t s = comm1.recv(buffer, 16);

		int a = 1;		
	}
}  // namespace

