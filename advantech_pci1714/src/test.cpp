#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "advantech_pci1714/Ping_received.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "advantechDriver_node");

	ros::NodeHandle n;

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */

		ROS_INFO("test");

		ros::spinOnce();

		loop_rate.sleep();
	}

  return 0;
}