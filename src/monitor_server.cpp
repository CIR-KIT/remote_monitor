#include <stdio.h>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>			// odom
#include <tf/transform_broadcaster.h>
#include "third_robot_monitor/TeleportAbsolute.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

const static std::string RESIZE_RATIO_PARAM = "/ratio";
const static std::string MAP_PATH = "/map/";
const static std::string MAP_NAME_PARAM = "/image";
const static std::string MAP_WINDOW_NAME = "Map Monitor";

class ThirdRobotMonitorServer
{
public:
    bool getPos(third_robot_monitor::TeleportAbsolute::Request  &req,
                third_robot_monitor::TeleportAbsolute::Response &res);
public:
    ThirdRobotMonitorServer(const std::string i_image_path, const std::string i_name_space)
    {
        ros::NodeHandle n("~");

        server_ = nh_.advertiseService<third_robot_monitor::TeleportAbsolute::Request,
                third_robot_monitor::TeleportAbsolute::Response>
                ("third_robot_monitor", boost::bind(&ThirdRobotMonitorServer::getPos, this, _1, _2));

        this->LoadRosParam(i_image_path, i_name_space);
       // this->LoadMapImage();

        cv::Mat map_img_ori = cv::imread(image_path_);
/*
        map_img_ori_small_ = cv::Mat::zeros(static_cast<int>(map_img_ori_.rows * 0.2), static_cast<int>(map_img_ori_.cols * 0.2), CV_8U);
        cv::resize(map_img_ori_, map_img_ori_small_, map_img_ori_small_.size(), 0, 0, cv::INTER_LINEAR);
*/
        cv::resize(map_img_ori, map_img_ori_small_, cv::Size(), resize_ratio_, resize_ratio_, cv::INTER_LINEAR);

        //cv::imshow(MAP_WINDOW_NAME, map_img_ori_small_);
        //cv::waitKey();
    }

    void LoadRosParam(const std::string i_image_path, const std::string i_name_space)
    {
        image_path_ = i_image_path + MAP_PATH;
        std::string param_image_name = i_name_space + MAP_NAME_PARAM;
        nh_.getParam(param_image_name, image_name_);
        image_path_ += image_name_;
        printf("path is %s\n", image_path_.c_str());

        std::string param_ratio = i_name_space + RESIZE_RATIO_PARAM;
        nh_.getParam(param_ratio, resize_ratio_);
    }

    cv::Mat LoadMapImage()
    {
        cv::Mat map_img_ori = cv::imread(image_path_);
        printf("a\n");
        //cv::Mat map_img_ori_small_tmp = cv::Mat::zeros(static_cast<int>(map_img_ori_.rows * 0.2), static_cast<int>(map_img_ori_.cols * 0.2), CV_8U);
        //map_img_ori_small_ = map_img_ori_small_tmp.clone();
        //printf("b\n");
        //cv::resize(map_img_ori_, map_img_ori_small_, map_img_ori_small_.size(), 0, 0, cv::INTER_LINEAR);
        cv::resize(map_img_ori, map_img_ori_small_, cv::Size(), resize_ratio_, resize_ratio_, cv::INTER_LINEAR);
        printf("c\n");
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer server_;
    std::string image_path_;
    std::string image_name_;
    double resize_ratio_;
    // cv::Mat map_img_ori_;
    cv::Mat map_img_ori_small_;
    cv::Mat map_img_pos_;
};


bool ThirdRobotMonitorServer::getPos(third_robot_monitor::TeleportAbsolute::Request  &req,
                                     third_robot_monitor::TeleportAbsolute::Response &res)
{
    ROS_INFO("Pos: [x] -> %6.2f, [y] -> %6.2f, [theta] -> %6.2f", req.x, req.y, req.theta);

    cv::imshow(MAP_WINDOW_NAME, map_img_ori_small_);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "third_robot_monitor_server");
    ThirdRobotMonitorServer monitor_server(argv[1], argv[2]);
    ros::spin();

    return 0;
}
