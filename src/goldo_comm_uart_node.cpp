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

void processMessage(uint16_t message_type, const uint8_t* payload, size_t size)
{
    ROS_INFO_STREAM("Received message, type: " << message_type << " payload: " << (char*)(payload));    
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goldo_comm_uart");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<goldo_comm_uart::RawMessage>("stm32/out/raw", 1000);  
  ros::Publisher console_pub = n.advertise<std_msgs::String>("stm32/out/console", 1000);
  
  ros::Subscriber raw_message_sub = n.subscribe<goldo_comm_uart::RawMessage>("stm32/in/raw", 1000, chatterCallback);  
  ros::Subscriber console_sub = n.subscribe<std_msgs::String>("stm32/in/console", 1000, consoleCallback);

  ros::Rate loop_rate(1000);
  
  std::string port_name;
  int baudrate;
  ros::param::get("~port", port_name);
  ros::param::get("~baudrate", baudrate);

  
  SerialCommHal serial_comm_hal;
  serial_comm_hal.open(port_name.c_str(), baudrate);
  g_comm.setHal(&serial_comm_hal);

  while (ros::ok())
  {
    ros::spinOnce();
	g_comm.spin(std::chrono::milliseconds{0});    
	auto ret = g_comm.recv(g_message_buffer, sizeof(g_message_buffer));

	while(ret > 0)
	{
        g_message_buffer[ret] = 0;
        processMessage(*(uint16_t*)g_message_buffer, g_message_buffer + 2, ret-2);       
        ret = g_comm.recv(g_message_buffer, sizeof(g_message_buffer));        
	}
    loop_rate.sleep();	
  }

  return 0;
}
