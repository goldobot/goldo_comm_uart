#include "serial_comm_hal.hpp"
#include <ros/console.h>

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "spdlog/spdlog.h"
#include "goldo_comm/comm.hpp"

#include <iostream>


void SerialCommHal::open(const char* port_name, int baudrate)
{    
    ROS_INFO_STREAM("Opening serial port " << port_name);
    m_fd = ::open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
	if(m_fd == 0)
	{
		ROS_ERROR_STREAM("Failed to open serial port " << port_name);
	}

    setInterfaceAttribs(baudrate);
    write(m_fd, "foobar", 6);
}

void SerialCommHal::setInterfaceAttribs(int baudrate)
    {
        struct termios tty;

    if (tcgetattr(m_fd, &tty) < 0) {
        //printf("Error from tcgetattr: %s\n", strerror(errno));
        return;
    }

    cfsetospeed(&tty, (speed_t)B230400);
    cfsetispeed(&tty, (speed_t)B230400);
    
    tty.c_cflag &= ~(CSIZE|CSTOPB|HUPCL|PARENB|CRTSCTS);
    tty.c_cflag |= (CS8|PARODD|CLOCAL|CREAD);

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
       // printf("Error from tcsetattr: %s\n", strerror(errno));
        return;
    }        
    };