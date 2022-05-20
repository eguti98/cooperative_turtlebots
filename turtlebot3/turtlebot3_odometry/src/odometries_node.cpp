#include "ros/ros.h"
#include "std_msgs/String.h"

void odometryCallback1(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Turtlebot 1");
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void odometryCallback2(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Turtlebot 2");
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void odometryCallback3(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Turtlebot 3");
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometries_node");
  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("/tb3_0/odom", 1000, odometryCallback1);
  ros::Subscriber sub2 = n.subscribe("tb3_1/odom", 1000, odometryCallback2);
  ros::Subscriber sub3 = n.subscribe("tb3_2/odom", 1000, odometryCallback3);

  ros::spin();
  return 0;
}
