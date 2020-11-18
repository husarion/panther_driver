#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>

#ifndef DOCKING_CONTROLLER_H
#define DOCKING_CONTROLLER_H

struct trajectory_function
{
    tf::Vector3 start;
    tf::Vector3 end;
    // f = ax^2 + bx + c
    double curve_a;
    double curve_b;
    double curve_c;
    double alpha;
    double radius;
    double circle_x;
    double circle_z;
    double z0;
    double x0;
    double z2;
    double x2;
    double z1;
    double x1;
};

class DockingController
{
private:
    std::shared_ptr<ros::NodeHandle> node_handle;
    std::unique_ptr<ros::Timer> tf_timer;
    std::unique_ptr<ros::Timer> cmd_vel_timer;
    std::unique_ptr<ros::ServiceServer> enable_service;
    std::unique_ptr<ros::ServiceServer> begin_docking_service;
    ros::Publisher velocity_pub;
    ros::Publisher base_link_path_pub;
    ros::Publisher receiver_path_pub;
    geometry_msgs::Twist velocity_msg;
    tf::TransformListener *tf_listener;

    tf::StampedTransform _tf_Robot_Station;
    tf::StampedTransform _tf_Receiver_Robot;
    tf::StampedTransform _tf_Receiver_Station;
    std::string robot_frame;
    std::string power_receiver;
    std::string power_station;
    double max_angular_vel;
    double max_linear_vel;
    double heading_angle_threshold;
    double distance_threshold;
    bool output_enabled;
    double path_resolution;
    trajectory_function path_function;
    nav_msgs::Path robot_path;
    nav_msgs::Path receiver_path;

    bool update_transform();
    void compute_command();
    bool enableOutput(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    void enableOutput();
    void disableOutput();
    bool beginDocking(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void generatePath();
    trajectory_function generate_trajectory_function(tf::Vector3 start_point, tf::Vector3 end_point);
    nav_msgs::Path generate_base_link_path(trajectory_function f, double resolution);
    nav_msgs::Path generate_receiver_path(trajectory_function f, double resolution);
    bool initPath();
    void startPathFollow();
    void stopPathFollow();
    double func(double alpha, double x0, double z0, double z2);

public:
    DockingController(std::shared_ptr<ros::NodeHandle> nh);
    ~DockingController();
};

#endif // DOCKING_CONTROLLER_H