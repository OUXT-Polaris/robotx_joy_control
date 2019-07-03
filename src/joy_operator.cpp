// Headers in this package
#include <ros/ros.h>
#include <vrx_joystick/joy_operator.hpp>

VrxJoystickOperator::VrxJoystickOperator(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh)
{
  std::string joy_topic = "joy";
  pnh_.subscribe(joy_topic,1,&VrxJoystickOperator::joyCallback,this);
}

VrxJoystickOperator::~VrxJoystickOperator()
{
  
}

void VrxJoystickOperator::run()
{
  boost::thread thread_calc(boost::bind(&VrxJoystickOperator::calcThruster, this));
  boost::thread thread_control(boost::bind(&VrxJoystickOperator::publishMotorCmd, this));
}

void VrxJoystickOperator::joyCallback(const sensor_msgs::Joy::ConstPtr msg)
{
  mtx_.lock();
  //axis_surge_ = msg->axes[joycon_map_.[0]];
  //axis_sway_ = msg->axes[joycon_map_.[1]];
  //axis_yaw_ = msg->axes[joycon_map_.[2]];
  axis_surge_ = msg->axes[0];
  axis_sway_ = msg->axes[1];
  axis_yaw_ = msg->axes[2];
  mtx_.unlock();
}

void VrxJoystickOperator::publishMotorCmd()
{
  ros::Rate rate(100);
  while(ros::ok())
  {
	mtx_.lock();
	pub_data_port_.data = cmd_port_;
	pub_data_stbd_.data = cmd_stbd_;
	mtx_.unlock();
  }
  rate.sleep();
}

void VrxJoystickOperator::calcThruster()
{
  mtx_.lock();
  cmd_port_ = axis_surge_;
  cmd_stbd_ = axis_sway_;
  mtx_.unlock();
}
