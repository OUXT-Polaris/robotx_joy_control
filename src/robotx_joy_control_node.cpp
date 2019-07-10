/* Headers in ROS */
#include <ros/ros.h>

/* Include this package */
#include <robotx_joy_control/robotx_joy_control.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robotx_joy_control_node");
    ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
    RobotXJoyControl controller(nh, pnh);
    ros::spin();
    return 0;
}
