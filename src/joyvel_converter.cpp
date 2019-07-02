// Headers in this package
#include <vrx_joystick/joyvel_converter.hpp>

VrxJoystickOperator::VrxSpeedController(ros::NodeHandle nh):
  nh_(nh)
{
  joy_sub_ = nh_.subscribe("/Joy", 1, &VrxJoystickOperator::callback_joysub, this);
}

VrxJoystickOperator::~VrxSpeedController()
{
  
}

void VrxJoystickOperator::run()
{
  boost::thread thread_calc(boost::bind(&VrxJoystickOperator::calc_thruster, this));
  boost::thread thread_control(boost::bind(&VrxJoystickOperator::publish_motor_cmd, this));
}

void VrxJoystickOperator::callback_get_joysub(const sensor_msgs::Joy& joy_msg)
{
  mtx_.lock();
  //axis_surge_ = joy_msg.axes[joycon_map_.[0]];
  //axis_sway_ = joy_msg.axes[joycon_map_.[1]];
  //axis_yaw_ = joy_msg.axes[joycon_map_.[2]];
  axis_surge_ = joy_msg.axes[0];
  axis_sway_ = joy_msg.axes[1];
  axis_yaw_ = joy_msg.axes[2];
  mtx_.unlock();
}

void VrxJoystickOperator::publish_motor_cmd()
{
  ros::Rate rate(100);
  while(ros::ok())
  {
	mtx_lock();
	pub_data_port_.data = cmd_port_;
	pub_data_stbd_.data = cmd_stbd_;
	mtx_unlock();
  }
  rate.sleep();
}

void VrxJoystickOperator::calc_thruster()
{
  mtx_.lock();
  cmd_port_ = axis_surge;
  cmd_stbd_ = axis_sway;
  mtx_.unlock();
}
