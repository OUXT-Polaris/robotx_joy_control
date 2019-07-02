/* Headers in ROS */
#include <ros/ros.h>

/* Include this package */
#include <vrx_joystick/joy_operator.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joy_operator_node");
    ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
    VrxJoystickOperator controller(nh, pnh);
    controller.run();
    ros::spin();
    return 0;
}
