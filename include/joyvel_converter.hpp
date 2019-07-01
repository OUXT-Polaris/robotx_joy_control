#ifndef VRX_OPERATE_VRX_JOYSTICK_COMMANDER_H_INCLUDED
#define VRX_OPERATE_VRX_JOYSTICK_COMMANDER_H_INCLUDED

/* Headers in ROS */
#include <ros/ros.h>
#include "sensor_msgs/Joy.h"

/* Headers in STL */
#include <mutex>

/* Headers in Boost */
#include <boost/thread.hpp>

class VrxJoystickCommander
{
  
public:
  VrxSpeedController(ros::NodeHandle nh);
  ~VrxSpeedController();
  
private:
  const ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher operation_signal_pub_;
  std::mutex mtx_;
  
  std::vector<int> joycon_map_;
  float axis_surge_;
  float axis_sway_;
  float axis_yaw_;
  
  void callback_joysub(const sensor_msgs::Joy& joy_msg);
  void publishCurrentCmd();
  
};

#endif  /*VRX_OPERATE_VRX_JOYSTICK_COMMANDER_H_INCLUDED*/
