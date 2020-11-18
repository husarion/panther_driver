#include <DockingController.h>

DockingController::DockingController(std::shared_ptr<ros::NodeHandle> nh)
    : node_handle(nh)
{
    node_handle->param<std::string>("robot_frame", robot_frame, "base_link");
    node_handle->param<std::string>("power_receiver", power_receiver, "power_receiver");
    node_handle->param<std::string>("power_station", power_station, "power_station");
    node_handle->param<double>("max_angular_vel", max_angular_vel, 0.1);
    node_handle->param<double>("max_linear_vel", max_linear_vel, 0.1);
    node_handle->param<double>("heading_angle_threshold", heading_angle_threshold, 0.05);
    node_handle->param<double>("distance_threshold", distance_threshold, 0.05);
    node_handle->param<double>("path_resolution", path_resolution, 0.01);

    ROS_INFO("robot_frame is %s", robot_frame.c_str());
    ROS_INFO("power_station is %s", power_station.c_str());
    ROS_INFO("max_angular_vel is %f", max_angular_vel);
    ROS_INFO("max_linear_vel is %f", max_linear_vel);
    ROS_INFO("heading_angle_threshold is %f", heading_angle_threshold);

    output_enabled = false;
    enable_service = std::make_unique<ros::ServiceServer>(node_handle->advertiseService("enable_output", &DockingController::enableOutput, this));
    begin_docking_service = std::make_unique<ros::ServiceServer>(node_handle->advertiseService("begin_docking", &DockingController::beginDocking, this));

    velocity_msg.linear.x = 0;
    velocity_msg.linear.y = 0;
    velocity_msg.linear.z = 0;
    velocity_msg.angular.x = 0;
    velocity_msg.angular.y = 0;
    velocity_msg.angular.z = 0;

    velocity_pub = node_handle->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    base_link_path_pub = node_handle->advertise<nav_msgs::Path>("base_link_path", 1);
    receiver_path_pub = node_handle->advertise<nav_msgs::Path>("receiver_path", 1);

    tf_listener = new tf::TransformListener();
    tf_timer = std::make_unique<ros::Timer>(node_handle->createTimer(ros::Duration(1.0 / 50.0), std::bind(&DockingController::update_transform, this)));
    cmd_vel_timer = std::make_unique<ros::Timer>(node_handle->createTimer(ros::Duration(1.0 / 50.0), std::bind(&DockingController::compute_command, this)));
    cmd_vel_timer->stop();

    ROS_INFO("DockingController node [%s] started.", node_handle->getNamespace().c_str());
    ROS_INFO("To control output use service: %s", enable_service->getService().c_str());
    ROS_INFO("Enabling:\nrosservice call %s \"data: 1\"", enable_service->getService().c_str());
    ROS_INFO("Disabling:\nrosservice call %s \"data: 0\"", enable_service->getService().c_str());
}

DockingController::~DockingController()
{
}

bool DockingController::beginDocking(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    initPath();
    return true;
}

double DockingController::func(double R, double x0, double z0, double z2)
{
    double R_ = fabs(z0 - z2) / (sin(fabs(atan(fabs(z0 - z2) / (R - fabs(x0))))));
    return R_;
}

trajectory_function DockingController::generate_trajectory_function(tf::Vector3 start_point, tf::Vector3 end_point)
{
    trajectory_function f;
    f.start = start_point;
    f.end = end_point;
    f.z0 = f.start.z();
    f.x0 = f.start.x();
    f.z2 = f.end.z();
    f.x2 = f.end.x();

    double fun_val = fabs(func(f.z0 - f.z2, f.x0, f.z0, f.z2) - (f.z0 - f.z2));
    for (double R = (f.z0 - f.z2); R < (f.z0 * 250); R += 0.001)
    {
        if (fabs(func(R, f.x0, f.z0, f.z2) - R) < fun_val)
        {
            fun_val = func(R, f.x0, f.z0, f.z2) - R;
            f.radius = R;
        }
    }

    f.circle_z = f.z2;
    if (f.x0 > 0)
    {
        f.circle_x = f.radius;
    }
    else
    {
        f.circle_x = -f.radius;
    }

    f.alpha = atan((f.z0 - f.z2) / (f.radius - fabs(f.x0)));
    return f;
}

nav_msgs::Path DockingController::generate_base_link_path(trajectory_function f, double resolution)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = power_station;

    geometry_msgs::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = path.header.frame_id;
    stamped_pose.header.stamp = path.header.stamp;
    stamped_pose.pose.position.x = f.circle_x;
    stamped_pose.pose.position.y = 0;
    stamped_pose.pose.position.z = f.circle_z;
    stamped_pose.pose.orientation.x = 0;
    stamped_pose.pose.orientation.y = 0;
    stamped_pose.pose.orientation.z = 0;
    stamped_pose.pose.orientation.w = 1;
    path.poses.push_back(stamped_pose);

    for (float x = 0; x < f.alpha; x += f.alpha / 100)
    {
        if (f.circle_x > 0)
        {
            stamped_pose.pose.position.x = f.circle_x - f.radius * cos(x);
        }
        else
        {
            stamped_pose.pose.position.x = f.circle_x + f.radius * cos(x);
        }
        stamped_pose.pose.position.z = f.circle_z + f.radius * sin(x);
        path.poses.push_back(stamped_pose);
    }
    stamped_pose.pose.position.x = f.circle_x;
    stamped_pose.pose.position.z = f.circle_z;
    path.poses.push_back(stamped_pose);
    return path;
}

nav_msgs::Path DockingController::generate_receiver_path(trajectory_function f, double resolution)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = power_station;

    geometry_msgs::PoseStamped stamped_pose;
    stamped_pose.header.frame_id = path.header.frame_id;
    stamped_pose.header.stamp = path.header.stamp;
    stamped_pose.pose.position.x = 0;
    stamped_pose.pose.position.y = 0;
    stamped_pose.pose.position.z = 0;
    stamped_pose.pose.orientation.x = 0;
    stamped_pose.pose.orientation.y = 0;
    stamped_pose.pose.orientation.z = 0;
    stamped_pose.pose.orientation.w = 1;

    for (float x = 0; x < f.alpha; x += f.alpha / 100)
    {
        if (f.circle_x > 0)
        {
            stamped_pose.pose.position.x = f.circle_x - f.radius * cos(x) - f.circle_z * sin(x);
        }
        else
        {
            stamped_pose.pose.position.x = f.circle_x + f.radius * cos(x) + f.circle_z * sin(x);
        }
        stamped_pose.pose.position.z = f.circle_z + f.radius * sin(x) - f.circle_z * cos(x);
        path.poses.push_back(stamped_pose);
    }
    return path;
}

bool DockingController::initPath()
{
    if (!update_transform())
    {
        ROS_WARN("Could not update transforms");
        return false;
    }

    tf::Vector3 start_point = _tf_Robot_Station.getOrigin();
    tf::Vector3 end_point(0, 0, _tf_Receiver_Robot.getOrigin().x());

    path_function = generate_trajectory_function(start_point, end_point);
    robot_path = generate_base_link_path(path_function, path_resolution);
    receiver_path = generate_receiver_path(path_function, path_resolution);

    ROS_INFO("Publish base link path");
    base_link_path_pub.publish(robot_path);
    receiver_path_pub.publish(receiver_path);
    startPathFollow();
    return true;
}

void DockingController::stopPathFollow()
{
    tf_timer->stop();
    cmd_vel_timer->stop();
}

void DockingController::startPathFollow()
{
    tf_timer->start();
    cmd_vel_timer->start();
}

bool DockingController::enableOutput(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    // req.data [bool]
    // res.success [bool]
    // res.message [string]
    ROS_INFO("Request %b", req.data);
    if (req.data)
    {
        enableOutput();
        res.message = "Enabled output";
    }
    else
    {
        disableOutput();
        res.message = "Disabled output";
    }
    res.success = true;

    return true;
}

void DockingController::enableOutput()
{
    ROS_INFO("Enable output");
    output_enabled = true;
}

void DockingController::disableOutput()
{
    ROS_INFO("Disable output");
    output_enabled = false;
    velocity_msg.linear.x = 0;
    velocity_msg.angular.z = 0;
    velocity_pub.publish(velocity_msg);
}

bool DockingController::update_transform()
{
    try
    {
        tf_listener->lookupTransform(power_station, robot_frame, ros::Time(0), _tf_Robot_Station);
        tf_listener->lookupTransform(robot_frame, power_receiver, ros::Time(0), _tf_Receiver_Robot);
        tf_listener->lookupTransform(power_station, power_receiver, ros::Time(0), _tf_Receiver_Station);
    }
    catch (tf2::TransformException &ex)
    {
        return false;
    }
    catch (tf::LookupException &ex)
    {
        return false;
    }
    catch (tf::ConnectivityException &ex)
    {
        return false;
    }
    catch (tf::ExtrapolationException &ex)
    {
        return false;
    }
    return true;
}

void DockingController::compute_command()
{
    if (_tf_Robot_Station.child_frame_id_.compare(robot_frame) == 0 && _tf_Robot_Station.frame_id_.compare(power_station) == 0)
    {
        tf::Vector3 robot_vector = _tf_Robot_Station.getOrigin();
        tf::Vector3 receiver_vector = _tf_Receiver_Station.getOrigin();
        geometry_msgs::PoseStamped receiver_dest_pose;
        receiver_dest_pose = receiver_path.poses[receiver_path.poses.size() - 1];
        double receiver_distance = sqrt(
            pow(receiver_dest_pose.pose.position.x - receiver_vector.x(), 2) +
            pow(receiver_dest_pose.pose.position.z - receiver_vector.z(), 2));
        if (receiver_distance > distance_threshold)
        {
            tf::Vector3 receiver_direction = receiver_vector - robot_vector;
            receiver_direction.setY(0);
            tf::Vector3 dest_direction = tf::Vector3(
                receiver_dest_pose.pose.position.x - robot_vector.x(),
                0,
                receiver_dest_pose.pose.position.z - robot_vector.z());

            double robot_angle = 0;
            if (receiver_direction.cross(dest_direction).y() > 0)
            {
                robot_angle = acos(receiver_direction.dot(dest_direction) / (receiver_direction.length() * dest_direction.length()));
            }
            else
            {
                robot_angle = -acos(receiver_direction.dot(dest_direction) / (receiver_direction.length() * dest_direction.length()));
            }
            double dest_lin_vel = 0;
            double dest_ang_vel = 0;
            if (fabs(robot_angle) < heading_angle_threshold)
            {
                dest_lin_vel = max_linear_vel - (max_linear_vel * fabs(robot_angle) / heading_angle_threshold);
                dest_ang_vel = max_angular_vel * (robot_angle) / heading_angle_threshold;
            }
            else
            {
                if ((robot_angle) > 0)
                {
                    dest_ang_vel = max_angular_vel;
                }
                else
                {
                    dest_ang_vel = -max_angular_vel;
                }
            }
            velocity_msg.linear.x = dest_lin_vel;
            velocity_msg.angular.z = dest_ang_vel;
        }
        else if (receiver_path.poses.size() > 1)
        {
            receiver_path.poses.pop_back();
        }
        else
        {
            ROS_INFO("TARGET REACHED");
            velocity_msg.linear.x = 0;
            velocity_msg.angular.z = 0;
            velocity_pub.publish(velocity_msg);
            output_enabled = false;
            stopPathFollow();
        }
    }

    if (output_enabled)
    {
        velocity_pub.publish(velocity_msg);
        receiver_path_pub.publish(receiver_path);
    }
}
