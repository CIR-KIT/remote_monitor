#include "ros/ros.h"
#include <nav_msgs/Odometry.h>			// odom
#include <tf/transform_broadcaster.h>
#include "third_robot_monitor/TeleportAbsolute.h"

class ThirdRobotMonitorServer
{
public:
	bool getPos(third_robot_monitor::TeleportAbsolute::Request  &req,
				third_robot_monitor::TeleportAbsolute::Response &res);
public:
	ThirdRobotMonitorServer()
	{
		ros::NodeHandle n("~");

		server_ = nh_.advertiseService<third_robot_monitor::TeleportAbsolute::Request,
									   third_robot_monitor::TeleportAbsolute::Response>
			("third_robot_monitor", boost::bind(&ThirdRobotMonitorServer::getPos, this, _1, _2));
	}
	
	
private:
	ros::NodeHandle nh_;
	ros::ServiceServer server_;
};


bool ThirdRobotMonitorServer::getPos(third_robot_monitor::TeleportAbsolute::Request  &req,
			third_robot_monitor::TeleportAbsolute::Response &res)
{
  ROS_INFO("Pos: [x] -> %6.2f, [y] -> %6.2f, [theta] -> %6.2f", req.x, req.y, req.theta);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "third_robot_monitor_server");
  ThirdRobotMonitorServer monitor_server;
  ros::spin();

  return 0;
}
