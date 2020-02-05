#include "goldo_comm/cobs_encoder.hpp"
#include "goldo_comm/cobs_decoder.hpp"
#include "gen_messages.hpp"

#include <algorithm>
#include <vector>
#include <cassert>
#include <cstring>
#include <iostream>


extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size) {

  auto messages = genMessages(Data, Size, 512);  

  goldo_comm::CobsEncoder encoder;
  goldo_comm::CobsDecoder decoder;
  
  uint8_t msg_buffer[1024];
  uint8_t encoded_buffer[1024];
  uint8_t decoded_buffer[1024];
  
  for(auto msg : messages)
  {
	  size_t msg_size = msg.second;
      size_t input_size = msg_size;
      size_t encoded_size = sizeof(encoded_buffer);
      size_t decoded_size = sizeof(decoded_buffer);
      
      encoder.encode(msg.first, input_size, encoded_buffer, encoded_size);
      bool message_finished = decoder.decode(encoded_buffer, encoded_size, decoded_buffer, decoded_size);
      
      std::cout << msg_size <<" " << encoded_size << " "  << decoded_size << "\n";
      assert(message_finished);
	 
  };

  return 0;  // Non-zero return values are reserved for future use.
}