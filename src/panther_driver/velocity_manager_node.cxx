#include <VelocityManager.hpp>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "vector"
#include <numeric>

#include "panther_driver/PantherDriverStatus.h"

// wrapper for fast change to new ros distro
void cmdCallback(const ros::MessageEvent<geometry_msgs::Twist const> &);
void joyCallback(const ros::MessageEvent<sensor_msgs::Joy const> &);

VelocityManager vm = VelocityManager();

bool sent_zeroes = 0; // joy publishes zeros all the time so to start timeout trigger - marker for last sent zeroes

void publishStatus(uint, std::string, const ros::Publisher &, uint32_t);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_manager_node");
  ros::NodeHandle n;

  ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 1, cmdCallback);
  ros::Subscriber sub_joy = n.subscribe("joy", 1, joyCallback);

  ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel_filtered", 1);
  ros::Publisher pub_status = n.advertise<panther_driver::PantherDriverStatus>("panther_driver/manager/status", 1);

  ros::Rate loop_rate(100);
  uint32_t counter = 0;
  while (ros::ok())
  {
    CmdVelInfo resp = vm.getVelocity();
    geometry_msgs::Twist msg = geometry_msgs::Twist();
    msg.linear.x = resp.linear.x;
    msg.linear.y = resp.linear.y;
    msg.linear.z = resp.linear.z;

    msg.angular.x = resp.angular.x;
    msg.angular.y = resp.angular.y;
    msg.angular.z = resp.angular.z;

    pub_cmd_vel.publish(msg);
    auto state_index = vm.getCurrentIndex();
    auto state_description = vm.getCurrentDescription();
    if (counter % 10 == 0)
    {
      publishStatus(state_index, state_description, pub_status, counter);
    }
    ros::spinOnce();
    vm.spin();
    loop_rate.sleep();
    counter++;
  }
  return 0;
}

void joyCallback(const ros::MessageEvent<sensor_msgs::Joy const> &event)
{
  const std::string &publisher_name = event.getPublisherName();
  const ros::M_string &header = event.getConnectionHeader();
  ros::Time receipt_time = event.getReceiptTime();

  const sensor_msgs::JoyConstPtr &msg = event.getMessage();

  if ((publisher_name.find("joy_node") != std::string::npos) && !(publisher_name.find("f710_joy_node") != std::string::npos))
  {
    //other joy node than f710
    vm.updateJoy(msg->buttons);
  }
  else
  {
    ; //ignore
  }
}

void cmdCallback(const ros::MessageEvent<geometry_msgs::Twist const> &event)
{
  const std::string &publisher_name = event.getPublisherName();
  const ros::M_string &header = event.getConnectionHeader();
  ros::Time receipt_time = event.getReceiptTime();

  const geometry_msgs::TwistConstPtr &msg = event.getMessage();

  if (publisher_name.find("f710_teleop_joy_node") != std::string::npos)
  {
    CmdVelInfo cmd_vel_info{};

    cmd_vel_info.publisher_name = "f710_teleop_joy_node";

    float sum_of_elems = msg->angular.x + msg->angular.y + msg->angular.z + msg->linear.x + msg->linear.y + msg->linear.z;

    if (sum_of_elems != 0.0)
    {
      cmd_vel_info.angular.x = msg->angular.x;
      cmd_vel_info.angular.y = msg->angular.y;
      cmd_vel_info.angular.z = msg->angular.z;

      cmd_vel_info.linear.x = msg->linear.x;
      cmd_vel_info.linear.y = msg->linear.y;
      cmd_vel_info.linear.z = msg->linear.z;
      vm.updateCmdVel(cmd_vel_info);
      sent_zeroes = 1;
    }
    else if (sent_zeroes && (sum_of_elems == 0.0))
    {
      ROS_INFO("setting cmd_vel to zero");
      //sent all zeroes
      cmd_vel_info.angular.x = 0.0;
      cmd_vel_info.angular.y = 0.0;
      cmd_vel_info.angular.z = 0.0;

      cmd_vel_info.linear.x = 0.0;
      cmd_vel_info.linear.y = 0.0;
      cmd_vel_info.linear.z = 0.0;
      vm.updateCmdVel(cmd_vel_info);
      sent_zeroes = 0;
    }
  }
  else if (publisher_name.find("move_base") != std::string::npos) //or publisher for autonomous in ros2
  {
    CmdVelInfo cmd_vel_info;

    cmd_vel_info.publisher_name = "autonomous";

    cmd_vel_info.angular.x = msg->angular.x;
    cmd_vel_info.angular.y = msg->angular.y;
    cmd_vel_info.angular.z = msg->angular.z;

    cmd_vel_info.linear.x = msg->linear.x;
    cmd_vel_info.linear.y = msg->linear.y;
    cmd_vel_info.linear.z = msg->linear.z;

    vm.updateCmdVel(cmd_vel_info);
  }
  else
  {
    CmdVelInfo cmd_vel_info;

    cmd_vel_info.publisher_name = "other";

    cmd_vel_info.angular.x = msg->angular.x;
    cmd_vel_info.angular.y = msg->angular.y;
    cmd_vel_info.angular.z = msg->angular.z;

    cmd_vel_info.linear.x = msg->linear.x;
    cmd_vel_info.linear.y = msg->linear.y;
    cmd_vel_info.linear.z = msg->linear.z;

    vm.updateCmdVel(cmd_vel_info);
  }
}

void publishStatus(uint status_index, std::string status_description, const ros::Publisher &pub, uint32_t index = 0)
{
  auto msg = panther_driver::PantherDriverStatus();
  msg.header.seq = index;
  msg.header.frame_id = "-";
  msg.header.stamp = ros::Time::now();
  msg.state = status_index;
  msg.description = status_description;
  pub.publish(msg);
}