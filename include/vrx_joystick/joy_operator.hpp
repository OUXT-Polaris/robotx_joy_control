#ifndef VRX_OPERATE_VRX_JOYSTICK_OPERATOR_H_INCLUDED
#define VRX_OPERATE_VRX_JOYSTICK_OPERATORR_H_INCLUDED

/* Headers in ROS */
#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "yaml-cpp/yaml.h"

/* Headers in STL */
#include <mutex>

/* Headers in Boost */
#include <boost/thread.hpp>

class VrxJoystickOperator
{
  
public:
  VrxJoystickOperator(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~VrxJoystickOperator();
  void run();
  
private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber joy_sub_;
  ros::Publisher operation_signal_pub_;
  std::mutex mtx_;
  
  std::vector<int> joycon_map_;
  float axis_surge_, axis_sway_, axis_yaw_;
  float cmd_port_, cmd_stbd_;
  std_msgs::Float32 pub_data_port_, pub_data_stbd_;
  
  void joyCallback(const sensor_msgs::Joy::ConstPtr msg);
  void publishMotorCmd();
  void calcThruster();
  
};

#endif  /*VRX_OPERATE_VRX_JOYSTICK_OPERATOR_H_INCLUDED*/
