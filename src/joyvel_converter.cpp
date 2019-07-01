// Headers in this package
#include <vrx_joystick/joyvel_converter.hpp>

VrxJoystickCommander::VrxSpeedController(ros::NodeHandle nh):
  nh_(nh)
{
  joy_sub_ = nh_.subscribe("/Joy", 1, &VrxJoystickCommander::callback_joysub, this);
}

void callback_joysub(const sensor_msgs::Joy& joy_msg)
{
  mtx_.lock();
  axis_surge_ = joy_msg.axes[joycon_map_.[0]];
  axis_sway_ = joy_msg.axes[joycon_map_.[1]];
  axis_yaw_ = joy_msg.axes[joycon_map_.[2]];
  mtx_.unlock();
}


