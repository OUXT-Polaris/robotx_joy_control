#ifndef VRX_OPERATE_VRX_JOYSTICK_OPERATOR_H_INCLUDED
#define VRX_OPERATE_VRX_JOYSTICK_OPERATORR_H_INCLUDED

/* Headers in ROS */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <usv_control_msgs/AzimuthThrusterCatamaranDriveStamped.h>
#include <rostate_machine/event_client.h>

/* Headers in STL */
#include <mutex>

/* Headers in Boost */
#include <boost/thread.hpp>

class RobotXJoyControl
{
  
public:
  RobotXJoyControl(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~RobotXJoyControl();
private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber joy_sub_;
  ros::Publisher manual_command_pub_;
  void joyCallback(const sensor_msgs::Joy::ConstPtr msg);
  void publishMotorCmd();
  void calcThruster();
  std::string joy_topic_;
  std::string manual_command_topic_;
  int left_thrust_axis_index_;
  int right_thrust_axis_index_;
  int bringup_button_index_;
  rostate_machine::EventClient client_;
  boost::optional<rostate_machine::Event> systemBringup();
  std::mutex mtx_;
  sensor_msgs::Joy joy_;
};

#endif  /*VRX_OPERATE_VRX_JOYSTICK_OPERATOR_H_INCLUDED*/
