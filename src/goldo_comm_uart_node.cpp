#include "ros/ros.h"
#include "std_msgs/String.h"
#include "goldo_comm_uart/RawMessage.h"

#include "goldo_comm/comm.hpp"
#include "serial_comm_hal.hpp"

#include <sstream>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goldo_comm_uart");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("messages/raw", 1000);

  ros::Rate loop_rate(10);
  
  std::string port_name;
  int baudrate;
  n.getParam("/goldo/comm_uart/port_name", port_name);
  n.getParam("/goldo/comm_uart/baudrate", baudrate);

  
  SerialCommHal serial_comm_hal;
  serial_comm_hal.open(port_name.c_str(), baudrate);
  while (ros::ok())
  {
//    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
