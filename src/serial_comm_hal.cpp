#include "serial_comm_hal.hpp"
#include <ros/console.h>

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "goldo_comm/comm.hpp"

#include <iostream>
#include <algorithm>

SerialCommHal::~SerialCommHal()
{
}

uint8_t* SerialCommHal::lock_read(size_t& buffer_size)
{	
    size_t read_size = m_read_buffer_write_idx >= m_read_buffer_read_idx ? sizeof(m_read_buffer) - m_read_buffer_write_idx : m_read_buffer_read_idx - m_read_buffer_write_idx - 1;
    
	auto ret = read(m_fd, m_read_buffer + m_read_buffer_write_idx, read_size);
    
	if(ret > 0)
	{        
		m_read_buffer_write_idx += ret;
        if(m_read_buffer_write_idx == sizeof(m_read_buffer))
        {
            m_read_buffer_write_idx = 0;
        }
	};
    
    size_t max_size = m_read_buffer_write_idx >= m_read_buffer_read_idx ? m_read_buffer_write_idx - m_read_buffer_read_idx : sizeof(m_read_buffer) - m_read_buffer_read_idx;
    if(buffer_size > max_size)
    {
        buffer_size = max_size;
    }
	return m_read_buffer + m_read_buffer_read_idx;
}

void SerialCommHal::unlock_read(size_t buffer_size)
{
    m_recv_trace.write((const char*)m_read_buffer + m_read_buffer_read_idx, buffer_size);
    m_recv_trace.flush();
    m_read_buffer_read_idx += buffer_size;
    if(m_read_buffer_read_idx == sizeof(m_read_buffer))
    {
        m_read_buffer_read_idx = 0;
    };
}

uint8_t* SerialCommHal::lock_write(size_t& buffer_size)
{
	if(buffer_size > sizeof(m_write_buffer))
	{
		buffer_size = sizeof(m_write_buffer);
	};	
	return m_write_buffer;
}

void SerialCommHal::unlock_write(size_t buffer_size) 
{
	write(m_fd, m_write_buffer, buffer_size);
}

void SerialCommHal::open(const char* port_name, int baudrate)
{    
    ROS_INFO_STREAM("Opening serial port " << port_name);
    m_fd = ::open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
	if(m_fd == 0)
	{
		ROS_ERROR_STREAM("Failed to open serial port " << port_name);
	}
    setInterfaceAttribs(baudrate);
    m_recv_trace.open("/home/goldo/catkin_ws/trace", std::ios::binary);
}

void SerialCommHal::setInterfaceAttribs(int baudrate)
    {
        struct termios tty;

    if (tcgetattr(m_fd, &tty) < 0) {
        ROS_ERROR_STREAM("Error from tcgetattr: " << strerror(errno) << "\n");
        return;
    }
	
	speed_t baudrate_enum = B9600;
	switch(baudrate)
	{
		case 9600:
		  baudrate_enum = B9600;
		  break;
		case 19200:
		  baudrate_enum = B19200;
		  break;
		case 38400:
		  baudrate_enum = B38400;
		  break;
		case 57600:
		  baudrate_enum = B57600;
		  break;
		case 115200:
		  baudrate_enum = B115200;
		  break;
		case 230400:
		  baudrate_enum = B230400;
		  break;
		case 460800:
		  baudrate_enum = B460800;
		  break;
		case 500000:
		  baudrate_enum = B500000;
		  break;
		case 576000:
		  baudrate_enum = B576000;
		  break;
		case 921600:
		  baudrate_enum = B921600;
		  break;
		case 1000000:
		  baudrate_enum = B1000000;
		  break;
		case 1152000:
		  baudrate_enum = B1152000;
		  break;
		case 1500000:
		  baudrate_enum = B1500000;
		  break;
		default:
		  ROS_ERROR_STREAM("Invalid baudrate: " << baudrate <<", defaulting to 9600bps");
		  break;
	};		  

    cfsetospeed(&tty, baudrate_enum);
    cfsetispeed(&tty, baudrate_enum);
    
    tty.c_cflag &= ~(CSIZE|CSTOPB|HUPCL|PARENB|CRTSCTS);
    tty.c_cflag |= (CS8|PARODD|CLOCAL|CREAD);

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_iflag |= IGNBRK;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
       // printf("Error from tcsetattr: %s\n", strerror(errno));
        return;
    }        
    };