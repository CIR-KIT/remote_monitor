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

		odom_.header.frame_id = "monitor_odom";

		monitor_pub_ = nh_.advertise<nav_msgs::Odometry>("/monitor_odom", 1);
		server_ = nh_.advertiseService<third_robot_monitor::TeleportAbsolute::Request,
									   third_robot_monitor::TeleportAbsolute::Response>
			("third_robot_monitor", boost::bind(&ThirdRobotMonitorServer::getPos, this, _1, _2));
	}
	
	
private:
	ros::NodeHandle nh_;
	ros::Publisher monitor_pub_;
	ros::ServiceServer server_;
	nav_msgs::Odometry odom_;
	tf::TransformBroadcaster monitor_odom_broadcaster_;
};


bool ThirdRobotMonitorServer::getPos(third_robot_monitor::TeleportAbsolute::Request  &req,
			third_robot_monitor::TeleportAbsolute::Response &res)
{
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(req.theta);
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "monitor_odom";
	odom_trans.child_frame_id = "monitor_base_link";
	odom_trans.transform.translation.x = req.x;
	odom_trans.transform.translation.y = req.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	
	monitor_odom_broadcaster_.sendTransform(odom_trans);
	  
	odom_.header.stamp = ros::Time::now();
	odom_.pose.pose.position.x = req.x;
	odom_.pose.pose.position.y = req.y;
	odom_.pose.pose.position.z = 0.0;
	odom_.pose.pose.orientation = odom_quat;
	monitor_pub_.publish(odom_);
	
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
