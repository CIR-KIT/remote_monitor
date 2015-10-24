#include "ros/ros.h"
#include <nav_msgs/Odometry.h>			// odom
#include <tf/transform_broadcaster.h>
#include "third_robot_monitor/TeleportAbsolute.h"

bool getPos(third_robot_monitor::TeleportAbsolute::Request  &req,
        third_robot_monitor::TeleportAbsolute::Response &res)
{
  std::cout<<"pos: "<<req.x<<" "<<req.y<<" "<<req.theta<<std::endl;
  return true;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "third_robot_monitor_server");
  ros::NodeHandle n;

  ros::ServiceServer server = n.advertiseService("third_robot_monitor", getPos);
  ROS_INFO("Ready to serve getPos.");
  ros::spin();

  return 0;
}
