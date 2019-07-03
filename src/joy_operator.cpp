// Headers in this package
#include <ros/ros.h>
#include <vrx_joystick/joy_operator.hpp>

VrxJoystickOperator::VrxJoystickOperator(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh)
{
  std::string joy_topic = "joy";
  joy_sub_ = nh_.subscribe(joy_topic,100,&VrxJoystickOperator::joyCallback,this);
  motor_port_pub_ = nh_.advertise<std_msgs::Float32>("left_thrust_cmd", 100);
  motor_stbd_pub_ = nh_.advertise<std_msgs::Float32>("right_thrust_cmd", 100);
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
  axis_surge_ = msg->axes[1];
  axis_sway_ = msg->axes[0];
  axis_yaw_ = msg->axes[3];
  mtx_.unlock();
}

void VrxJoystickOperator::publishMotorCmd()
{
  ros::Rate rate(10);
  while(ros::ok())
  {
	mtx_.lock();
	pub_data_port_.data = cmd_port_;
	pub_data_stbd_.data = cmd_stbd_;
	mtx_.unlock();

	motor_port_pub_.publish(pub_data_port_);
	motor_stbd_pub_.publish(pub_data_stbd_);

	ROS_INFO("Port:%.1f%%\tStbd:%.1f%%", pub_data_port_.data*100, pub_data_stbd_.data*100);
  }
  rate.sleep();
}

void VrxJoystickOperator::calcThruster()
{
  /*float thrust_x = 0.0;
  float thrust_y = 0.0;
  float x_fin = 0.0;
  float y_fin = 0.0;*/
  
  while(ros::ok())
  {	
	mtx_.lock();
	cmd_port_ = axis_surge_;
	cmd_stbd_ = axis_yaw_;
	mtx_.unlock();
  }
}
