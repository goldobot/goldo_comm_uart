#include "ros/ros.h"
#include "std_msgs/String.h"
#include "goldo_comm_uart/RawMessage.h"

#include "goldo_comm/comm.hpp"
#include "serial_comm_hal.hpp"

#include <sstream>
#include <iostream>


goldo_comm::Comm g_comm;

void chatterCallback(const goldo_comm_uart::RawMessage::ConstPtr& msg)
{
  ROS_INFO("I heard: ");
  g_comm.send((void*)msg->body.data(), msg->body.size());
}
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "goldo_comm_uart");

  ros::NodeHandle n("comm_uart");

  ros::Publisher chatter_pub = n.advertise<goldo_comm_uart::RawMessage>("stm32/out/raw", 1000);
  ros::Subscriber raw_message_sub = n.subscribe<goldo_comm_uart::RawMessage>("stm32/in/raw", 1000, chatterCallback);

  ros::Rate loop_rate(1000);
  
  std::string port_name;
  int baudrate;
  n.getParam("port", port_name);
  n.getParam("baudrate", baudrate);

  
  SerialCommHal serial_comm_hal;
  serial_comm_hal.open(port_name.c_str(), baudrate);
  g_comm.setHal(&serial_comm_hal);
  
  uint64_t i;
  while (ros::ok())
  {
    ros::spinOnce();
	g_comm.spin(std::chrono::milliseconds{i});
    loop_rate.sleep();
	i++;
  }

  return 0;
}
