#include "ros/ros.h"
#include "std_msgs/String.h"
#include "goldo_comm_uart/RawMessage.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goldo_comm_uart");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
//    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
