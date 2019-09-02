// Headers in this package
#include <ros/ros.h>
#include <robotx_joy_control/robotx_joy_control.hpp>

RobotXJoyControl::RobotXJoyControl(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh),
  client_(nh,pnh,"control_state_machine_node")
{
  pnh_.param<int>("axis/left_thrust_cmd", left_thrust_axis_index_, 1);
  pnh_.param<int>("axis/right_thrust_cmd", right_thrust_axis_index_, 4);
  pnh_.param<int>("button/bringup_button_index", bringup_button_index_, 4);
  pnh_.param<std::string>("joy_topic", joy_topic_, "/joy");
  pnh_.param<std::string>("manual_command_topic", manual_command_topic_, "/manual_command");
  manual_command_pub_ = nh_.advertise<usv_control_msgs::AzimuthThrusterCatamaranDriveStamped>(manual_command_topic_,1);
  joy_sub_ = nh_.subscribe(joy_topic_,1,&RobotXJoyControl::joyCallback,this);
  client_.registerCallback(std::bind(&RobotXJoyControl::systemBringup, this),"RobotXJoyControl::systemBringup");
  client_.registerCallback(std::bind(&RobotXJoyControl::manualOverride, this),"RobotXJoyControl::manualOverride");
  client_.run();
}

RobotXJoyControl::~RobotXJoyControl()
{
  
}

void RobotXJoyControl::joyCallback(const sensor_msgs::Joy::ConstPtr msg)
{
  mtx_.lock();
  joy_ = *msg;
  usv_control_msgs::AzimuthThrusterCatamaranDriveStamped manual_command;
  ROS_ASSERT(msg->axes.size() > right_thrust_axis_index_);
  ROS_ASSERT(msg->axes.size() > left_thrust_axis_index_);
  manual_command.header = msg->header;
  manual_command.command.left_thrust_cmd = msg->axes[left_thrust_axis_index_];
  manual_command.command.right_thrust_cmd = msg->axes[right_thrust_axis_index_];
  manual_command_pub_.publish(manual_command);
  mtx_.unlock();
  return;
}

boost::optional<rostate_machine::Event> RobotXJoyControl::systemBringup()
{
  mtx_.lock();
  ROS_ASSERT(joy_.buttons.size() > bringup_button_index_);
  if(joy_.buttons[bringup_button_index_] == 1)
  {
    mtx_.unlock();
    rostate_machine::Event msg;
    msg.header = joy_.header;
    msg.trigger_event_name = "system_bringup";
    return msg;
  }
  mtx_.unlock();
  return boost::none;
}

boost::optional<rostate_machine::Event> RobotXJoyControl::manualOverride()
{
  mtx_.lock();
  ROS_ASSERT(joy_.buttons.size() > override_button_index_);
  if(joy_.buttons[override_button_index_] == 1)
  {
    mtx_.unlock();
    rostate_machine::Event msg;
    msg.header = joy_.header;
    msg.trigger_event_name = "manual_override";
    return msg;
  }
  mtx_.unlock();
  return boost::none;
}