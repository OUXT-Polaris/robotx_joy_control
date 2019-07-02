#ifndef VRX_OPERATE_VRX_JOYSTICK_COMMANDER_H_INCLUDED
#define VRX_OPERATE_VRX_JOYSTICK_COMMANDER_H_INCLUDED

/* Headers in ROS */
#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32"
#include "yaml-cpp/yaml.h"

/* Headers in STL */
#include <mutex>

/* Headers in Boost */
#include <boost/thread.hpp>

class VrxJoystickOperator
{
  
public:
  VrxSpeedController(ros::NodeHandle nh);
  ~VrxSpeedController();
  void run();
  
private:
  const ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  ros::Publisher operation_signal_pub_;
  std::mutex mtx_;
  
  std::vector<int> joycon_map_;
  float axis_surge_, axis_sway_, axis_yaw_;
  float cmd_port_, cmd_stbd_;
  std_msgs::Float32 pub_data_port_, pub_data_stbd_;
  
  
  void callback_get_joysub(const sensor_msgs::Joy& joy_msg);
  void publish_motor_cmd();
  void calc_thruster();
  
};

#endif  /*VRX_OPERATE_VRX_JOYSTICK_COMMANDER_H_INCLUDED*/
