#include <stdio.h>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>			// odom
#include <tf/transform_broadcaster.h>
#include "third_robot_monitor/TeleportAbsolute.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

const static std::string PARAM_NAME_RATIO_PARAM = "/ratio";
const static std::string MAP_PATH = "/map/";
const static std::string PARAM_NAME_MAP_NAME = "/image";
const static std::string PARAM_NAME_MAP_RESOLUTION = "/resolution";
const static std::string PARAM_NAME_MAP_ORIGIN = "/origin";
const static std::string MAP_WINDOW_NAME = "Map Monitor";
const static int ROS_SPIN_RATE = 100;
const static int CV_WAIT_KEY_RATE = 50;
const static cv::Scalar RED = cv::Scalar(0, 0, 255);
const static cv::Scalar GREEN = cv::Scalar(0, 255, 0);
const static cv::Scalar BLUE = cv::Scalar(255, 0, 0);
const static int ARROW_LENGTH = 10;

// default
const static std::string DEFAULT_MAP_NAME = "201510240538.pgm";
const static double DEFAULT_MAP_RESOLUTION = 0.1;
const static double DEFAULT_RESIZE_RATIO = 0.2;

enum WAIT_KEY_MODE
{
    CURR_POSITON,
    HISTORY_POSITION,
    QUIT
};

enum RESULT
{
    OK,
    NG
};

enum MAP_COORD
{
    INDEX_X,
    INDEX_Y,
    INDEX_Z
};


class ThirdRobotMonitorServer
{
public:
    bool getRobotPose(third_robot_monitor::TeleportAbsolute::Request  &req,
                      third_robot_monitor::TeleportAbsolute::Response &res);
    bool getHumanPose(third_robot_monitor::TeleportAbsolute::Request  &req,
                      third_robot_monitor::TeleportAbsolute::Response &res);

    ThirdRobotMonitorServer(const std::string i_image_path, const std::string i_name_space)
       : rate_(ROS_SPIN_RATE), state_(CURR_POSITON)
    {
        //ros::NodeHandle n("~");
        //rate_ = ROS_SPIN_RATE;
        map_origin_.clear();
        server_robot_pose_ = nh_.advertiseService<third_robot_monitor::TeleportAbsolute::Request,
                third_robot_monitor::TeleportAbsolute::Response>
                ("third_robot_monitor_robot_pose", boost::bind(&ThirdRobotMonitorServer::getRobotPose, this, _1, _2));
        server_human_pose_ = nh_.advertiseService<third_robot_monitor::TeleportAbsolute::Request,
                third_robot_monitor::TeleportAbsolute::Response>
                ("third_robot_monitor_human_pose", boost::bind(&ThirdRobotMonitorServer::getHumanPose, this, _1, _2));

        this->loadRosParam(i_image_path, i_name_space);
    }

    void drawRobotPoseOnMap(third_robot_monitor::TeleportAbsolute::Request &req)
    {
        // only current pos
        map_img_ori_small_.copyTo(map_img_pos_curr_);
        //-- center of the robot on the map
        double x_map_center = (req.x - map_origin_[INDEX_X]) * resize_ratio_curr_ / map_resolution_;
        double y_map_center = map_img_pos_curr_.rows - (req.y - map_origin_[INDEX_Y]) / map_resolution_ * resize_ratio_curr_;
        point_curr_ = cv::Point(x_map_center, y_map_center);
        //-- arrow tip that represents the robot orientation on the map
        double x_map_tip = x_map_center + ARROW_LENGTH * std::cos(req.theta);
        double y_map_tip = y_map_center - ARROW_LENGTH * std::sin(req.theta);
        point_tip_ = cv::Point(x_map_tip, y_map_tip);
        //-- draw
        cv::circle(map_img_pos_curr_, point_curr_, 2, RED, 3);
        cv::line(map_img_pos_curr_, point_curr_, point_tip_, RED, 2);

        // all pos history
        cv::circle(map_img_pos_hist_, point_curr_, 2, RED, 3);
        cv::line(map_img_pos_hist_, point_curr_, point_tip_, RED, 2);
    }

    void drawHumanPoseOnMap(third_robot_monitor::TeleportAbsolute::Request &req)
    {
        // only current pos
        map_img_ori_small_.copyTo(map_img_pos_curr_);
        //-- center of the robot on the map
        double x_map_center = (req.x - map_origin_[INDEX_X]) * resize_ratio_curr_ / map_resolution_;
        double y_map_center = map_img_pos_curr_.rows - (req.y - map_origin_[INDEX_Y]) / map_resolution_ * resize_ratio_curr_;
        point_curr_ = cv::Point(x_map_center, y_map_center);
        //-- arrow tip that represents the robot orientation on the map
        double x_map_tip = x_map_center + ARROW_LENGTH * std::cos(req.theta);
        double y_map_tip = y_map_center - ARROW_LENGTH * std::sin(req.theta);
        point_tip_ = cv::Point(x_map_tip, y_map_tip);
        //-- draw
        cv::circle(map_img_pos_curr_, point_curr_, 2, BLUE, 3);
        cv::line(map_img_pos_curr_, point_curr_, point_tip_, BLUE, 2);

        // all pos history
        cv::circle(map_img_pos_hist_, point_curr_, 2, BLUE, 3);
        cv::line(map_img_pos_hist_, point_curr_, point_tip_, BLUE, 2);
    }

    int waitKeyJudge(const int i_key)
    {
        int ret = 0;

        // current pose
        if(i_key == 'c' || i_key == 'C')
        {
            ret = CURR_POSITON;
        }
        // history of pose
        else if(i_key == 'h' || i_key == 'H')
        {
            ret = HISTORY_POSITION;
        }
        // reset history
        else if(i_key == 'r' || i_key == 'R')
        {
            // reset
            map_img_ori_small_.copyTo(map_img_pos_hist_);
            // redraw current position
            cv::circle(map_img_pos_hist_, point_curr_, 2, RED, 3);
            cv::line(map_img_pos_hist_, point_curr_, point_tip_, RED, 2);

            // keep original state
            ret = state_;
        }
        // zoom
        else if(i_key == 'p' || i_key == 'P')
        {
            resize_ratio_prev_ = resize_ratio_curr_;
            resize_ratio_curr_ += 0.05;
            ret = state_;

            // resize
            // resize
            cv::resize(map_img_ori_, map_img_ori_small_, cv::Size(), resize_ratio_curr_, resize_ratio_curr_, cv::INTER_LINEAR);
            map_img_pos_curr_ = map_img_ori_small_.clone();
            map_img_pos_hist_ = map_img_ori_small_.clone();

            drawRobotPoseOnMap(req_);
        }
        // fade
        else if(i_key == 'm' || i_key == 'M')
        {
            resize_ratio_prev_ = resize_ratio_curr_;
            resize_ratio_curr_ -= 0.05;
            ret = state_;

            // resize
            cv::resize(map_img_ori_, map_img_ori_small_, cv::Size(), resize_ratio_curr_, resize_ratio_curr_, cv::INTER_LINEAR);
            map_img_pos_curr_ = map_img_ori_small_.clone();
            map_img_pos_hist_ = map_img_ori_small_.clone();

            drawRobotPoseOnMap(req_);
        }
        // 'Esc' or 'q'が押された場合に終了
        else if(i_key == 27 || i_key == 'q' || i_key == 'Q')
        {
            ret = QUIT;
        }
        else
        {
            // do nothing
            ret = true;
        }

        return ret;
    }

    void showMap()
    {
        // current pose
        if(state_ == CURR_POSITON)
        {
            cv::imshow(MAP_WINDOW_NAME, map_img_pos_curr_);
        }
        // history of pose
        else if(state_ == HISTORY_POSITION)
        {
            cv::imshow(MAP_WINDOW_NAME, map_img_pos_hist_);
        }
        else
        {
            cv::imshow(MAP_WINDOW_NAME, map_img_pos_curr_);
        }
    }

    void runMainLoop()
    {
      while(nh_.ok())
      {
          //cv::imshow(MAP_WINDOW_NAME, map_img_pos_curr_);
          int key = cv::waitKey(CV_WAIT_KEY_RATE);
          if(key >= 0)
              state_ = waitKeyJudge(key);

          if(state_ == QUIT)
              break;
          else
              showMap();

          ros::spinOnce();
          rate_.sleep();
      }
    }

    int loadMapImage()
    {
        map_img_ori_ = cv::imread(image_path_);

        if(map_img_ori_.rows == 0 || map_img_ori_.cols == 0)
        {
          ROS_ERROR("image path is %s was not found.", image_path_.c_str());
          return NG;
        }

        ROS_INFO("image %s was successfully loaded.", image_path_.c_str());
        // resize
        cv::resize(map_img_ori_, map_img_ori_small_, cv::Size(), resize_ratio_curr_, resize_ratio_curr_, cv::INTER_LINEAR);
        map_img_pos_curr_ = map_img_ori_small_.clone();
        map_img_pos_hist_ = map_img_ori_small_.clone();
    }

private:
    void loadRosParam(const std::string i_image_path, const std::string i_name_space)
    {
        // map file name
        image_path_ = i_image_path + MAP_PATH;
        nh_.param<std::string>(i_name_space + PARAM_NAME_MAP_NAME, image_name_, DEFAULT_MAP_NAME);
        image_path_ += image_name_;
        ROS_INFO("image path is %s.", image_path_.c_str());

        // resize_ratio
        nh_.param(i_name_space + PARAM_NAME_RATIO_PARAM, resize_ratio_curr_, DEFAULT_RESIZE_RATIO);
        // resolution
        nh_.param(i_name_space + PARAM_NAME_MAP_RESOLUTION, map_resolution_, DEFAULT_MAP_RESOLUTION);
        // origin(array)
        XmlRpc::XmlRpcValue origin_list;
        nh_.getParam(i_name_space + PARAM_NAME_MAP_ORIGIN, origin_list);
        ROS_ASSERT(origin_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int32_t i = 0; i < origin_list.size(); ++i)
        {
            ROS_ASSERT(origin_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            map_origin_.push_back(static_cast<double>(origin_list[i]));
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Rate rate_;
    ros::ServiceServer server_robot_pose_;
    ros::ServiceServer server_human_pose_;
    std::string image_path_;
    std::string image_name_;
    double resize_ratio_curr_;
    double resize_ratio_prev_;
    double map_resolution_;
    std::vector<double> map_origin_;
    cv::Point point_curr_;
    cv::Point point_tip_;
    // images
    cv::Mat map_img_ori_;
    cv::Mat map_img_ori_small_;
    cv::Mat map_img_pos_curr_;
    cv::Mat map_img_pos_hist_;
    // state
    third_robot_monitor::TeleportAbsolute::Request req_;
    int state_;
};


bool ThirdRobotMonitorServer::getRobotPose(third_robot_monitor::TeleportAbsolute::Request  &req,
                                           third_robot_monitor::TeleportAbsolute::Response &res)
{
    ROS_INFO("RobotPose: [x] -> %6.2f, [y] -> %6.2f, [theta] -> %6.2f", req.x, req.y, req.theta);

    req_ = req;
    drawRobotPoseOnMap(req);

    return true;
}

bool ThirdRobotMonitorServer::getHumanPose(third_robot_monitor::TeleportAbsolute::Request  &req,
                                           third_robot_monitor::TeleportAbsolute::Response &res)
{
    ROS_INFO("HumanPose: [x] -> %6.2f, [y] -> %6.2f, [theta] -> %6.2f", req.x, req.y, req.theta);

    req_ = req;
    drawHumanPoseOnMap(req);

    return true;
}

int fillPolygonAndShow()
{
    int w = 800;
    cv::Mat img = cv::Mat::zeros( w, w, CV_8UC3 );
    int lineType = 8;

    /** Create some points */
    cv::Point rook_points[1][4];
    rook_points[0][0] = cv::Point(w*0.5, 0.0);
    rook_points[0][1] = cv::Point(0, w);
    rook_points[0][2] = cv::Point(w*0.5, w*0.7);
    rook_points[0][3] = cv::Point(w, w);

    const cv::Point* ppt[1] = { rook_points[0] };
    int npt[] = { 4 };

    cv::fillPoly( img, ppt, npt, 1, GREEN, lineType );

    cv::Point2f center(img.cols*0.5, img.rows*0.5);
    //const cv::Mat affine_matrix = cv::getRotationMatrix2D( center, angle, scale );

    cv::imshow("test", img);
    int key = cv::waitKey();

    if(key == 27 || key == 'q' || key == 'Q')
        return -1;
}

int main(int argc, char **argv)
{
    if (fillPolygonAndShow() == -1)
        return -1;

    ros::init(argc, argv, "third_robot_monitor_server");

    if(argc < 3)
    {
        ROS_ERROR("Short of arguments. map package path and namespace must be given.");
        ROS_ERROR("Aborting third_robot_monitor_server...");
        return -1;
    }

    std::string map_package_path = argv[1];
    std::string ns = argv[2];
    ThirdRobotMonitorServer monitor_server(map_package_path, ns);

    if(monitor_server.loadMapImage() == NG)
    {
        ROS_ERROR("Aborting third_robot_monitor_server...");
        return -1;
    }

    monitor_server.runMainLoop();

    return 0;
}
