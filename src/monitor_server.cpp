#include "ros/ros.h"
#include "nav_msgs/Odometry.h"			// odom
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "sensor_msgs/image_encodings.h"
#include "tf/transform_broadcaster.h"
#include "cirkit_waypoint_navigator/TeleportAbsolute.h"

#include <stdio.h>

const static std::string PARAM_NAME_RATIO_PARAM = "/ratio";
const static std::string MAP_PATH = "/map/";
const static std::string PARAM_NAME_MAP_NAME = "/image";
const static std::string PARAM_NAME_MAP_RESOLUTION = "/resolution";
const static std::string PARAM_NAME_MAP_ORIGIN = "/origin";
const static std::string MAP_WINDOW_NAME = "Map Monitor";
const static int ROS_SPIN_RATE = 100;
const static int CV_WAIT_KEY_RATE = 50;
const static cv::Scalar RED = cv::Scalar(0, 0, 255);
const static cv::Scalar GREEN = cv::Scalar(0, 128, 0);
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


class RemoteMonitorServer
{
public:
    bool getRobotPose(cirkit_waypoint_navigator::TeleportAbsolute::Request  &req,
                      cirkit_waypoint_navigator::TeleportAbsolute::Response &res);
    bool getHumanPose(cirkit_waypoint_navigator::TeleportAbsolute::Request  &req,
                      cirkit_waypoint_navigator::TeleportAbsolute::Response &res);

    RemoteMonitorServer(const std::string i_image_path, const std::string i_name_space)
       : rate_(ROS_SPIN_RATE), state_(CURR_POSITON)
    {
        //ros::NodeHandle n("~");
        //rate_ = ROS_SPIN_RATE;
        map_origin_.clear();
        server_robot_pose_ = nh_.advertiseService<cirkit_waypoint_navigator::TeleportAbsolute::Request,
                cirkit_waypoint_navigator::TeleportAbsolute::Response>
                ("remote_monitor_robot_pose", boost::bind(&RemoteMonitorServer::getRobotPose, this, _1, _2));
        server_human_pose_ = nh_.advertiseService<cirkit_waypoint_navigator::TeleportAbsolute::Request,
                cirkit_waypoint_navigator::TeleportAbsolute::Response>
                ("remote_monitor_human_pose", boost::bind(&RemoteMonitorServer::getHumanPose, this, _1, _2));

        this->loadRosParam(i_image_path, i_name_space);
    }

    void drawRobotPoseOnMap(cirkit_waypoint_navigator::TeleportAbsolute::Request &req)
    {
        // only current point
        drawArrow(map_img_pos_curr_, req);
        // all pos history
        drawArrow(map_img_pos_hist_, req);
    }

    void drawArrow(cv::Mat &img, cirkit_waypoint_navigator::TeleportAbsolute::Request &req)
    {
        // only current pos
        map_img_ori_small_.copyTo(map_img_pos_curr_);
        //-- center of the robot on the map
        double x_map_center = (req.x - map_origin_[INDEX_X]) * resize_ratio_curr_ / map_resolution_;
        double y_map_center = map_img_pos_curr_.rows - (req.y - map_origin_[INDEX_Y]) / map_resolution_ * resize_ratio_curr_;

        int w = 10;
        int h = 15;
        int lineType = 8;

        /** Create some points */
        cv::Point rook_points[1][4];
        double p1_x_ori = (x_map_center-w*0.5) + w*0.5;
        double p1_y_ori = (y_map_center-h*0.5) + 0.0;
        double p2_x_ori = (x_map_center-w*0.5) + 0;
        double p2_y_ori = (y_map_center-h*0.5) + h;
        double p3_x_ori = (x_map_center-w*0.5) + w*0.5;
        double p3_y_ori = (y_map_center-h*0.5) + h*0.7;
        double p4_x_ori = (x_map_center-w*0.5) + w;
        double p4_y_ori = (y_map_center-h*0.5) + h;

        double cos_rot = cos(-req.theta + M_PI*0.5);
        double sin_rot = sin(-req.theta + M_PI*0.5);

        double p1_x_rot = cos_rot*(p1_x_ori-x_map_center) - sin_rot*(p1_y_ori-y_map_center);
        double p1_y_rot = sin_rot*(p1_x_ori-x_map_center) + cos_rot*(p1_y_ori-y_map_center);
        double p2_x_rot = cos_rot*(p2_x_ori-x_map_center) - sin_rot*(p2_y_ori-y_map_center);
        double p2_y_rot = sin_rot*(p2_x_ori-x_map_center) + cos_rot*(p2_y_ori-y_map_center);
        double p3_x_rot = cos_rot*(p3_x_ori-x_map_center) - sin_rot*(p3_y_ori-y_map_center);
        double p3_y_rot = sin_rot*(p3_x_ori-x_map_center) + cos_rot*(p3_y_ori-y_map_center);
        double p4_x_rot = cos_rot*(p4_x_ori-x_map_center) - sin_rot*(p4_y_ori-y_map_center);
        double p4_y_rot = sin_rot*(p4_x_ori-x_map_center) + cos_rot*(p4_y_ori-y_map_center);

        p1_x_rot += x_map_center;
        p1_y_rot += y_map_center;
        p2_x_rot += x_map_center;
        p2_y_rot += y_map_center;
        p3_x_rot += x_map_center;
        p3_y_rot += y_map_center;
        p4_x_rot += x_map_center;
        p4_y_rot += y_map_center;

        rook_points[0][0] = cv::Point(p1_x_rot, p1_y_rot);
        rook_points[0][1] = cv::Point(p2_x_rot, p2_y_rot);
        rook_points[0][2] = cv::Point(p3_x_rot, p3_y_rot);
        rook_points[0][3] = cv::Point(p4_x_rot, p4_y_rot);

        const cv::Point* ppt[1] = { rook_points[0] };
        int npt[] = { 4 };

        cv::fillPoly( img, ppt, npt, 1, GREEN, lineType );
    }

    void drawHumanPoseOnMap(cirkit_waypoint_navigator::TeleportAbsolute::Request &req)
    {
        // only current pos
        map_img_ori_small_.copyTo(map_img_pos_curr_);
        //-- center of the robot on the map
        double x_map_center = (req.x - map_origin_[INDEX_X]) * resize_ratio_curr_ / map_resolution_;
        double y_map_center = map_img_pos_curr_.rows - (req.y - map_origin_[INDEX_Y]) / map_resolution_ * resize_ratio_curr_;
        point_curr_ = cv::Point(x_map_center, y_map_center);

        // only current point
        drawHuman(map_img_pos_curr_, point_curr_);
        // all pos history
        drawHuman(map_img_pos_hist_, point_curr_);
    }

    void drawHuman(cv::Mat &img, const cv::Point2f pos)
    {
        const double body_len = 18.0;
        const double arm_len = 12.0;
        const double leg_len = 14.0;
        cv::Point point_body = cv::Point(pos.x, pos.y + body_len);
        cv::Point point_chest = cv::Point(pos.x, pos.y + body_len * 0.7);
        cv::Point point_arm_r = cv::Point(pos.x + arm_len, pos.y + arm_len * 0.25);
        cv::Point point_arm_l = cv::Point(pos.x - arm_len, pos.y + arm_len * 0.25);
        cv::Point point_leg_r = cv::Point(pos.x + leg_len * 0.25, pos.y + body_len + leg_len);
        cv::Point point_leg_l = cv::Point(pos.x - leg_len * 0.25, pos.y + body_len + leg_len);

        //-- draw
        cv::circle(img, point_curr_, 7, RED, -1, CV_AA);
        cv::line(img, point_curr_, point_body, RED, 2);
        cv::line(img, point_chest, point_arm_r, RED, 2);
        cv::line(img, point_chest, point_arm_l, RED, 2);
        cv::line(img, point_body, point_leg_r, RED, 2);
        cv::line(img, point_body, point_leg_l, RED, 2);
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
            drawArrow(map_img_pos_curr_, req_);

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
    cirkit_waypoint_navigator::TeleportAbsolute::Request req_;
    int state_;
};


bool RemoteMonitorServer::getRobotPose(cirkit_waypoint_navigator::TeleportAbsolute::Request  &req,
                                       cirkit_waypoint_navigator::TeleportAbsolute::Response &res)
{
    ROS_INFO("RobotPose: [x] -> %6.2f, [y] -> %6.2f, [theta] -> %6.2f", req.x, req.y, req.theta);

    req_ = req;
    drawRobotPoseOnMap(req);

    return true;
}

bool RemoteMonitorServer::getHumanPose(cirkit_waypoint_navigator::TeleportAbsolute::Request  &req,
                                       cirkit_waypoint_navigator::TeleportAbsolute::Response &res)
{
    ROS_INFO("HumanPose: [x] -> %6.2f, [y] -> %6.2f, [theta] -> %6.2f", req.x, req.y, req.theta);

    req_ = req;
    drawHumanPoseOnMap(req);

    return true;
}

int fillPolygonAndShow()
{
    int w = 20;
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

int humanDrawAndShow()
{

    int w = 20;
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
    /*
    if (fillPolygonAndShow() == -1)
        return -1;
        */

    ros::init(argc, argv, "remote_monitor_server");

    if(argc < 3)
    {
        ROS_ERROR("Short of arguments. map package path and namespace must be given.");
        ROS_ERROR("Aborting remote_monitor_server...");
        return -1;
    }

    std::string map_package_path = argv[1];
    std::string ns = argv[2];
    RemoteMonitorServer monitor_server(map_package_path, ns);

    if(monitor_server.loadMapImage() == NG)
    {
        ROS_ERROR("Aborting remote_monitor_server...");
        return -1;
    }

    monitor_server.runMainLoop();

    return 0;
}
