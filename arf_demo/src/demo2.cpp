#include <ros/ros.h>
#include <vector>
#include <Eigen/Geometry>
#include <exception>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>

//#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_moveit_wrapper/redundant_robot.h"
#include "arf_trajectory/trajectory.h"
#include "arf_graph/graph.h"
#include "arf_graph/util.h"
#include "arf_planning/planner.h"

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
    visual_tools_->publishAxis(pose, rvt::LARGE);
    visual_tools_->trigger();
  }

  void clear()
  {
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();
  }
  void animatePath(RedundantRobot& robot, std::vector<std::vector<double>> path)
  {
    for (auto q : path)
    {
      robot.plot(visual_tools_, q);
      ros::Duration(0.5).sleep();
    }
}
};

std::vector<TrajectoryPoint> createTrajectory()
{
  std::vector<TrajectoryPoint> ee_trajectory;
  for (int i = 0; i < 10; ++i)
  {
    TolerancedNumber x(1.0, 0.95, 1.05, 5);
    // Number y, z(0.5 + static_cast<double>(i) / 20);
    Number y(static_cast<double>(i) / 20), z(0.5);
    Number rx, ry(M_PI), rz;
    TrajectoryPoint tp(x, y, z, rx, ry, rz);
    ee_trajectory.push_back(tp);
  }
  return ee_trajectory;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_moveit_wrapper");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Demo 2");

    RedundantRobot robot;
    Rviz rviz;
    Planner planner;

    auto traj = createTrajectory();
    for (auto tp : traj)
    {
      tp.plot(rviz.visual_tools_);
    }

    planner.setTrajectory(traj);
    planner.createGraphData(robot);
    planner.calculateShortestPath(robot);

    auto shortest_path = planner.getShortestPath();
    //rviz.animatePath(robot, shortest_path);

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    move_group.setJointValueTarget(shortest_path[0]);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;

    bool success = (move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");

    //trajectory_msgs::JointTrajectory weld_trajectory;
    int N = plan1.trajectory_.joint_trajectory.points.size();
    ros::Duration start_time = plan1.trajectory_.joint_trajectory.points[N-1].time_from_start;
    ROS_INFO_STREAM("Start time welding: " <<  start_time);
    for (auto q_sol : shortest_path)
    {
      trajectory_msgs::JointTrajectoryPoint new_point;
      new_point.positions = q_sol;
      start_time += ros::Duration(0.1);
      new_point.time_from_start = start_time;
      plan1.trajectory_.joint_trajectory.points.push_back(new_point);
    }

    move_group.execute(plan1);

    ROS_INFO_STREAM("Plan: " << plan1.trajectory_);

    move_group.setNamedTarget("allZeros");
    success = (move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 22(joint space goal) %s", success ? "" : "FAILED");
    move_group.execute(plan1);


    

    // // calculate forward kinematics to get reachable pose
    // std::vector<double> q1 = {0.0, 0.0, 0, 0, 0, 0, 0, 0};
    // auto fk_pose = robot.fk(q1, "tool_tip");

    // // move the fk_pose closer to robot
    // Eigen::Vector3d p1(0.3, 0.5, -0.3);
    // fk_pose.translate(p1);

    // // visualize fk_pose
    // rviz.visual_tools_->publishAxisLabeled(fk_pose, "fk_pose");
    // rviz.visual_tools_->trigger();

    // ros::Duration(1.0).sleep();

    // // set values for fixed joints = y- and z-rail
    // std::vector<double> q_fixed = {0, 0};
    
    // //auto ik_sol = robot.redundantIk(fk_pose, q_fixed);
    // auto ik_sol = robot.ikGridSamples(fk_pose);

    // // show ik solutions
    // if (ik_sol.size() > 0)
    // {
    //     for (auto q_sol : ik_sol)
    //     {
    //         robot.plot(rviz.visual_tools_, q_sol);
    //         ros::Duration(0.4).sleep();
    //     }
    // }
    // else
    // {
    //     ROS_INFO("No ik solutions found");
    // }

    ros::Duration(1.0).sleep();

    ros::shutdown();
    
    return 0;
}