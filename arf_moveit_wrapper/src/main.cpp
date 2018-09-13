#include <ros/ros.h>
#include <vector>

#include "arf_moveit_wrapper/moveit_wrapper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_moveit_wrapper");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    RobotMoveitWrapper robot;

    robot.printCurrentJointValues();

    std::vector<double> q1 = {0, 0, 0, 0, 0, 0};

    ROS_INFO_STREAM("Robot in collision? " << robot.isInCollision(q1)); 


    ros::Duration(1.0).sleep();

    ros::shutdown();
    return 0;
}