add_executable(fuzz_cobs fuzz_cobs.cpp)
target_link_libraries(fuzz_cobs PRIVATE goldo_comm)
target_compile_options(fuzz_cobs PRIVATE "-fsanitize=fuzzer")
target_link_options(fuzz_cobs PRIVATE "-fsanitize=fuzzer")

add_executable(fuzz_circular_buffer fuzz_circular_buffer.cpp)
target_link_libraries(fuzz_circular_buffer PRIVATE goldo_comm)
target_compile_options(fuzz_circular_buffer PRIVATE "-fsanitize=fuzzer")
target_link_options(fuzz_circular_buffer PRIVATE "-fsanitize=fuzzer")

add_executable(fuzz_message_queue fuzz_message_queue.cpp)
target_link_libraries(fuzz_message_queue PRIVATE goldo_comm)
target_compile_options(fuzz_message_queue PRIVATE "-fsanitize=fuzzer")
target_link_options(fuzz_message_queue PRIVATE "-fsanitize=fuzzer")
