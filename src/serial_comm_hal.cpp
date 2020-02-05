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
    ROS_INFO_STREAM("Opening serial port " << port_name << " baudrate: "<<baudrate);
    m_fd = ::open(port_name, O_RDWR | O_NOCTTY | O_SYNC);
    std::cout << m_fd;
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

    cfsetospeed(&tty, (speed_t)baudrate);
    cfsetispeed(&tty, (speed_t)baudrate);
    
    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

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