cmake_minimum_required(VERSION 3.1...3.15)

project(goldo_comm VERSION 1.0 LANGUAGES C CXX)

option(GOLDO_COMM_BUILD_TESTS "goldo_comm build tests" OFF)
option(GOLDO_COMM_BUILD_FUZZERS "goldo_comm build tests" ON)

if(GOLDO_COMM_BUILD_TESTS EQUAL "ON")
  # Download and unpack googletest at configure time
  configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
  execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download"
  )
  execute_process(COMMAND "${CMAKE_COMMAND}" --build .
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download"
  )

  # Prevent GoogleTest from overriding our compiler/linker options
  # when building with Visual Studio
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  # Add googletest directly to our build. This adds the following targets:
  # gtest, gtest_main, gmock and gmock_main
  add_subdirectory("${CMAKE_BINARY_DIR}/googletest-src"
                   "${CMAKE_BINARY_DIR}/googletest-build"
  )
  
  enable_testing()
endif()





add_library(goldo_comm STATIC 
  include/goldo_comm/cobs.h
  include/goldo_comm/cobs_encoder.hpp
  include/goldo_comm/cobs_decoder.hpp
  include/goldo_comm/comm.hpp
  include/goldo_comm/circular_buffer.hpp
  include/goldo_comm/message_queue.hpp  
  include/goldo_comm/crc32c.hpp
  src/cobs_encoder.cpp
  src/cobs_decoder.cpp
  src/comm.cpp
  src/crc32c.cpp
  )
target_include_directories(goldo_comm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
  )
  
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND GOLDO_COMM_BUILD_FUZZERS)
  add_subdirectory(fuzzers)
endif()

if(GOLDO_COMM_BUILD_TESTS)
  add_subdirectory(tests)
endif()