#include "goldo_comm/comm.hpp"
#include <fstream>

class SerialCommHal : public goldo_comm::CommHal
{
  public:
    ~SerialCommHal() override;
    void open(const char* port_name, int baudrate);
	
	uint8_t* lock_read(size_t& buffer_size) override;
    void unlock_read(size_t buffer_size) override;
	uint8_t* lock_write(size_t& buffer_size) override;
	void unlock_write(size_t buffer_size) override;
    
  private:
    int m_fd;
	uint8_t m_read_buffer[256];
	uint8_t m_write_buffer[256];
    
    unsigned m_read_buffer_read_idx{0};
    unsigned m_read_buffer_write_idx{0};
    
    std::ofstream m_recv_trace;
    
    void setInterfaceAttribs(int baudrate);    
};