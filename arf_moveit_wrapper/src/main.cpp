#include <ros/ros.h>
#include <vector>
#include <Eigen/Geometry>

#include <moveit_visual_tools/moveit_visual_tools.h>

//#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_moveit_wrapper/redundant_robot.h"

namespace rvt = rviz_visual_tools;

class Rviz
{
public:
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  Rviz()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world","/demo_visuals"));
  }

  void plotPose(Eigen::Affine3d pose)
  {
    Eigen::Isometry3d pose_temp;
    pose_temp = pose.matrix();
    visual_tools_->publishAxis(pose_temp, rvt::LARGE);
    visual_tools_->trigger();
  }

  void clear()
  {
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();
  }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_moveit_wrapper");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    RedundantRobot robot;
    Rviz rviz;
    rviz.clear();

    // calculate forward kinematics to get reachable pose
    std::vector<double> q1 = {0.0, 0.0, 0, 0, 0, 0, 0, 0};
    auto fk_pose = robot.fk(q1, "tool_tip");

    // move the fk_pose closer to robot
    Eigen::Vector3d p1(0.3, 0.5, -0.3);
    fk_pose.translate(p1);

    // visualize fk_pose
    Eigen::Isometry3d pose_temp(fk_pose.matrix());
    rviz.visual_tools_->publishAxisLabeled(pose_temp, "fk_pose");
    rviz.visual_tools_->trigger();

    ros::Duration(1.0).sleep();

    // set values for fixed joints = y- and z-rail
    std::vector<double> q_fixed = {0, 0};
    
    //auto ik_sol = robot.redundantIk(fk_pose, q_fixed);
    auto ik_sol = robot.ikGridSamples(fk_pose);

    // show ik solutions
    if (ik_sol.size() > 0)
    {
        for (auto q_sol : ik_sol)
        {
            robot.plot(rviz.visual_tools_, q_sol);
            ros::Duration(0.4).sleep();
        }
    }
    else
    {
        ROS_INFO("No ik solutions found");
    }

    ros::Duration(1.0).sleep();

    ros::shutdown();
    return 0;
}