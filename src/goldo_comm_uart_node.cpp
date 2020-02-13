#include "ros/ros.h"
#include "std_msgs/String.h"
#include "goldo_comm_uart/RawMessage.h"

#include "goldo_comm/comm.hpp"
#include "serial_comm_hal.hpp"

#include <sstream>
#include <iostream>


goldo_comm::Comm g_comm;
uint8_t g_message_buffer[1024];

void chatterCallback(const goldo_comm_uart::RawMessage::ConstPtr& msg)
{
	namespace ser = ros::serialization;	
	uint32_t serial_size = ros::serialization::serializationLength(*msg);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);    
    ser::OStream stream(buffer.get(), serial_size);
	ser::serialize(stream, *msg);
	
    g_comm.send((void*)buffer.get(), serial_size);
}

void consoleCallback(const std_msgs::String::ConstPtr& msg)
{
	*(uint16_t*)g_message_buffer = 4;
	for(int i=0; i < msg->data.size(); i++)
	{
		g_message_buffer[i+2] = msg->data[i];
	}
    g_comm.send(g_message_buffer, msg->data.size() + 2);
}
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "goldo_comm_uart");

  ros::NodeHandle n("comm_uart");

  ros::Publisher chatter_pub = n.advertise<goldo_comm_uart::RawMessage>("out/raw", 1000);  
  ros::Publisher console_pub = n.advertise<std_msgs::String>("out/console", 1000);
  
  ros::Subscriber raw_message_sub = n.subscribe<goldo_comm_uart::RawMessage>("in/raw", 1000, chatterCallback);  
  ros::Subscriber console_sub = n.subscribe<std_msgs::String>("in/console", 1000, consoleCallback);

  ros::Rate loop_rate(1000);
  
  std::string port_name;
  int baudrate;
  n.getParam("port", port_name);
  n.getParam("baudrate", baudrate);

  
  SerialCommHal serial_comm_hal;
  serial_comm_hal.open(port_name.c_str(), baudrate);
  g_comm.setHal(&serial_comm_hal);
  uint64_t i = 1;
  while (ros::ok())
  {
    ros::spinOnce();
	g_comm.spin(std::chrono::milliseconds{i});
	auto ret = g_comm.recv(g_message_buffer, sizeof(g_message_buffer));
	g_message_buffer[ret] = 0;
	if(ret > 0)
	{
		ROS_INFO_STREAM("Received message " << i << ": " << (char*)(g_message_buffer+2));
		i++;
	}
    loop_rate.sleep();
	
  }

  return 0;
}
