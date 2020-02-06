#include "goldo_comm/comm.hpp"

class SerialCommHal
{
  public:
    void open(const char* port_name, int baudrate);
	void spinOnce();
    
  private:
    int m_fd;    
    void setInterfaceAttribs(int baudrate);    
};