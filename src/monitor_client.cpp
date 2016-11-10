#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"  // odom
#include "tf/transform_broadcaster.h"
#include "waypoint_navigator/TeleportAbsolute.h"


class ThirdRobotMonitorClient
{
public:
	void sendPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose);
	
public:
    ThirdRobotMonitorClient(const std::string _ns) : rate_(1.0), ns_(_ns)
	{
        ros::NodeHandle n(ns_);
        n.param("interval_dist", interval_dist_, 5.0);
        n.param("pose_topic", pose_topic_, std::string("/amcl_pose"));
        ROS_INFO("interval_dist = %.2f.", interval_dist_);
        ROS_INFO("pose_topic = %s.", pose_topic_.c_str());

        monitor_client_ = nh_.serviceClient<waypoint_navigator::TeleportAbsolute>("third_robot_monitor_robot_pose");
        odom_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_, 1, boost::bind(&ThirdRobotMonitorClient::sendPosition, this, _1));
	}

	

	double calculateDistance(geometry_msgs::PoseWithCovariance pose)
	{
		return sqrt(pow(pose.pose.position.x - last_pose_.pose.position.x, 2)
					+ pow(pose.pose.position.y - last_pose_.pose.position.y, 2));
	}

	void getRPY(const geometry_msgs::Quaternion &q,
				double &roll,double &pitch,double &yaw){
		tf::Quaternion tfq(q.x, q.y, q.z, q.w);
		tf::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
	}


private:
	ros::NodeHandle nh_;
	ros::Rate rate_;
	ros::ServiceClient monitor_client_;
	waypoint_navigator::TeleportAbsolute srv_;
	ros::Subscriber odom_sub_;

	geometry_msgs::PoseWithCovariance last_pose_;

    std::string ns_;
    double interval_dist_;
    std::string pose_topic_;
};

void ThirdRobotMonitorClient::sendPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose)
	{
		double dist = calculateDistance(amcl_pose->pose);
        if(dist >= interval_dist_)
		{
			double yaw, pitch, roll;
			getRPY(amcl_pose->pose.pose.orientation, roll, pitch, yaw);
			srv_.request.x = amcl_pose->pose.pose.position.x;
			srv_.request.y = amcl_pose->pose.pose.position.y;
			srv_.request.theta = yaw;
		
			if(monitor_client_.call(srv_))
			{
				ROS_INFO("Succeed to send robot position to server.");
			}
			else
			{
				ROS_INFO("Failed to send robot position to server.");
			}
			last_pose_ = amcl_pose->pose;
		}
	}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "third_robot_monitor_client");

    std::string ns;
    if(argc > 1)
        ns = argv[1];
    else
        ns = "third_robot_monitor_client";

    ThirdRobotMonitorClient client(ns);
	
	ros::spin();
	
	return 0;
}
