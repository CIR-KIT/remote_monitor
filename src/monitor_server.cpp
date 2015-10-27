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
const static std::string MAP_WINDOW_NAME = "Map Monitor";
const static int ROS_SPIN_RATE = 100;
const static int CV_WAIT_KEY_RATE = 50;
const static cv::Scalar RED = cv::Scalar(0, 0, 255);
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


class ThirdRobotMonitorServer
{
public:
    bool getPos(third_robot_monitor::TeleportAbsolute::Request  &req,
                third_robot_monitor::TeleportAbsolute::Response &res);

    ThirdRobotMonitorServer(const std::string i_image_path, const std::string i_name_space)
       : rate_(ROS_SPIN_RATE), state_(CURR_POSITON)
    {
        //ros::NodeHandle n("~");
        //rate_ = ROS_SPIN_RATE;
        server_ = nh_.advertiseService<third_robot_monitor::TeleportAbsolute::Request,
                third_robot_monitor::TeleportAbsolute::Response>
                ("third_robot_monitor", boost::bind(&ThirdRobotMonitorServer::getPos, this, _1, _2));

        this->LoadRosParam(i_image_path, i_name_space);
        map_img_ori_ = cv::imread(image_path_);

        // resize
        cv::resize(map_img_ori_, map_img_ori_small_, cv::Size(), resize_ratio_curr_, resize_ratio_curr_, cv::INTER_LINEAR);
        map_img_pos_curr_ = map_img_ori_small_.clone();
        map_img_pos_hist_ = map_img_ori_small_.clone();
    }

    void DrawPosOnMap(third_robot_monitor::TeleportAbsolute::Request &req)
    {
        // only current pos
        map_img_ori_small_.copyTo(map_img_pos_curr_);
        //-- center of the robot on the map
        double x_map_center = req.x * resize_ratio_curr_ / map_resolution_;
        double y_map_center = map_img_pos_curr_.rows - req.y / map_resolution_ * resize_ratio_curr_;
        point_curr_ = cv::Point(x_map_center, y_map_center);
        //-- arrow tip that represents the robot orientation on the map
        double x_map_tip = x_map_center + ARROW_LENGTH * cv::cos(req.theta);
        double y_map_tip = y_map_center - ARROW_LENGTH * cv::sin(req.theta);
        point_tip_ = cv::Point(x_map_tip, y_map_tip);
        //-- draw
        cv::circle(map_img_pos_curr_, point_curr_, 2, RED, 3);
        cv::line(map_img_pos_curr_, point_curr_, point_tip_, RED, 2);

        // all pos history
        cv::circle(map_img_pos_hist_, point_curr_, 2, RED, 3);
        cv::line(map_img_pos_hist_, point_curr_, point_tip_, RED, 2);
    }

    int WaitKeyJudge(const int i_key)
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
            /*
            cv::resize(map_img_pos_curr_, map_img_pos_curr_, cv::Size(), resize_ratio_prev_/resize_ratio_curr_, resize_ratio_prev_/resize_ratio_curr_, cv::INTER_LINEAR);
            cv::resize(map_img_pos_hist_, map_img_pos_hist_, cv::Size(), resize_ratio_prev_/resize_ratio_curr_, resize_ratio_prev_/resize_ratio_curr_, cv::INTER_LINEAR);
            */
            cv::resize(map_img_ori_, map_img_ori_small_, cv::Size(), resize_ratio_curr_, resize_ratio_curr_, cv::INTER_LINEAR);
            map_img_pos_curr_ = map_img_ori_small_.clone();
            map_img_pos_hist_ = map_img_ori_small_.clone();

            DrawPosOnMap(req_);
        }
        // fade
        else if(i_key == 'm' || i_key == 'M')
        {
            resize_ratio_prev_ = resize_ratio_curr_;
            resize_ratio_curr_ -= 0.05;
            ret = state_;

            // resize
            /*
            cv::resize(map_img_pos_curr_, map_img_pos_curr_, cv::Size(), resize_ratio_prev_/resize_ratio_curr_, resize_ratio_prev_/resize_ratio_curr_, cv::INTER_LINEAR);
            cv::resize(map_img_pos_hist_, map_img_pos_hist_, cv::Size(), resize_ratio_prev_/resize_ratio_curr_, resize_ratio_prev_/resize_ratio_curr_, cv::INTER_LINEAR);
            */
            cv::resize(map_img_ori_, map_img_ori_small_, cv::Size(), resize_ratio_curr_, resize_ratio_curr_, cv::INTER_LINEAR);
            map_img_pos_curr_ = map_img_ori_small_.clone();
            map_img_pos_hist_ = map_img_ori_small_.clone();

            DrawPosOnMap(req_);
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

    void ShowMap()
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

    void RunMainLoop()
    {
      while(nh_.ok())
      {
          //cv::imshow(MAP_WINDOW_NAME, map_img_pos_curr_);
          int key = cv::waitKey(CV_WAIT_KEY_RATE);
          if(key >= 0)
              state_ = WaitKeyJudge(key);

          if(state_ == QUIT)
              break;
          else
              ShowMap();

          ros::spinOnce();
          rate_.sleep();
      }
    }

private:
    void LoadRosParam(const std::string i_image_path, const std::string i_name_space)
    {
        // map file name
        image_path_ = i_image_path + MAP_PATH;
        std::string param_name_image_name = i_name_space + PARAM_NAME_MAP_NAME;
        nh_.param<std::string>(param_name_image_name, image_name_, DEFAULT_MAP_NAME);
        image_path_ += image_name_;
        printf("path is %s\n", image_path_.c_str());

        // resize_ratio
        std::string param_name_ratio = i_name_space + PARAM_NAME_RATIO_PARAM;
        nh_.param(param_name_ratio, resize_ratio_curr_, DEFAULT_RESIZE_RATIO);

        // resolution
        std::string param_name_resolution = i_name_space + PARAM_NAME_MAP_RESOLUTION;
        nh_.param(param_name_resolution, map_resolution_, DEFAULT_MAP_RESOLUTION);
    }

    cv::Mat LoadMapImage()
    {
        cv::Mat map_img_ori = cv::imread(image_path_);
        cv::resize(map_img_ori, map_img_ori_small_, cv::Size(), resize_ratio_curr_, resize_ratio_curr_, cv::INTER_LINEAR);
    }

private:
    ros::NodeHandle nh_;
    ros::Rate rate_;
    ros::ServiceServer server_;
    std::string image_path_;
    std::string image_name_;
    double resize_ratio_curr_;
    double resize_ratio_prev_;
    double map_resolution_;
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


bool ThirdRobotMonitorServer::getPos(third_robot_monitor::TeleportAbsolute::Request  &req,
                                     third_robot_monitor::TeleportAbsolute::Response &res)
{
    ROS_INFO("Pos: [x] -> %6.2f, [y] -> %6.2f, [theta] -> %6.2f", req.x, req.y, req.theta);

    req_ = req;
    DrawPosOnMap(req);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "third_robot_monitor_server");

    ThirdRobotMonitorServer monitor_server(argv[1], argv[2]);
    monitor_server.RunMainLoop();
    //ros::spin();

    return 0;
}
