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

		monitor_pub_ = nh_.advertise<nav_msgs::Odometry>("/monitor_odom", 1);
		server_ = nh_.advertiseService<third_robot_monitor::TeleportAbsolute::Request,
									   third_robot_monitor::TeleportAbsolute::Response>
			("third_robot_monitor", boost::bind(&ThirdRobotMonitorServer::getPos, this, _1, _2));
	}
	
	
private:
	ros::NodeHandle nh_;
	ros::Publisher monitor_pub_;
	ros::ServiceServer server_;
};


bool ThirdRobotMonitorServer::getPos(third_robot_monitor::TeleportAbsolute::Request  &req,
			third_robot_monitor::TeleportAbsolute::Response &res)
{	
	std::cout<<"pos: "<<req.x<<" "<<req.y<<" "<<req.theta<<std::endl;
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "third_robot_monitor_server");
  ThirdRobotMonitorServer monitor_server;
  ros::spin();

  return 0;
}
