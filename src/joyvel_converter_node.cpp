/* Headers in ROS */
#include <ros/ros.h>

/* Include this package */
#include <vrx_joystick/joyvel_converter.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joystick_operation_node");
    ros::NodeHandle nh;
    VrxJoystickOperator controller(nh);
    controller.run();
    ros::spin();
    return 0;
}
