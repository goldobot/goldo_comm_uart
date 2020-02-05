#include "goldo_comm/comm.hpp"

class SerialCommHal
{
  public:
    void open(const char* port_name, int baudrate);
    
  private:
    int m_fd;    
    void setInterfaceAttribs(int baudrate);    
};