add_executable(utest_goldo_comm 
utest_circular_buffer.cpp
utest_message_queue.cpp
utest_comm.cpp
utest_cobs_encoder.cpp)

target_link_libraries(utest_goldo_comm PRIVATE gtest_main goldo_comm)

#add_executable(test_cobs test_cobs.cpp)
#target_link_libraries(test_cobs PRIVATE goldo_comm)


#add_executable(goldo_comm_test_cobs goldo_cobs_test.cpp)
#target_link_libraries(goldo_comm_test_cobs PRIVATE comm)

#add_executable(test_comm test_comm.cpp)
#target_link_libraries(test_comm PRIVATE goldo_comm)

#add_executable(goldo_comm_test_msg_queue goldo_comm_msg_queue_test.cpp)
#target_link_libraries(goldo_comm_test_msg_queue PRIVATE comm)


#add_test(test_cobs test_cobs)
#add_test(goldo_comm_test_comm goldo_comm_test_comm)
#add_test(goldo_comm_test_msg_queue goldo_comm_test_msg_queue)