#include <ros/ros.h>
#include <vector>
#include <Eigen/Geometry>
#include <exception>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <eigen_conversions/eigen_msg.h>

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

    bool planner_success = planner.run(robot, traj);
    ROS_INFO_NAMED("tutorial", "Planner %s", planner_success ? "SUCCESS" : "FAILED");
    auto shortest_path = planner.getShortestPath();
    //rviz.animatePath(robot, shortest_path);

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    moveit::planning_interface::MoveGroupInterface::Plan plan1, plan2, plan3;

    // find approach point
    geometry_msgs::Pose  approach_pose;
    tf::poseEigenToMsg(robot.fk(shortest_path[0]), approach_pose);
    approach_pose.position.z -= 0.1;

    move_group.setJointValueTarget(approach_pose);
    bool success = (move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Go to approach pose %s", success ? "" : "FAILED");

    // std::vector<geometry_msgs::Pose> waypoints;
    // for (int i = 1; i < 11; ++i)
    // {
    //   geometry_msgs::Pose temp = approach_pose;
    //   temp.position.z += 0.01 * i;
    //   waypoints.push_back(temp); // mmm, should create new onces that keep existing...
    // }

    // moveit_msgs::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 0.01;
    // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // ROS_INFO_STREAM("Fraction: " << fraction);
    // ROS_INFO_STREAM("Plan: " << trajectory);

    int N = plan1.trajectory_.joint_trajectory.points.size();
    ros::Duration start_time = plan1.trajectory_.joint_trajectory.points[N-1].time_from_start;
    // for (auto& trajpt : trajectory.joint_trajectory.points)
    // {
    //   trajpt.time_from_start += start_time;
    //   plan1.trajectory_.joint_trajectory.points.push_back(trajpt);
    // }
  

    //trajectory_msgs::JointTrajectory weld_trajectory;
    N = plan1.trajectory_.joint_trajectory.points.size();
    start_time = plan1.trajectory_.joint_trajectory.points[N-1].time_from_start;
    ROS_INFO_STREAM("Start time welding: " <<  start_time);
    for (auto& q_sol : shortest_path)
    {
      trajectory_msgs::JointTrajectoryPoint new_point;
      new_point.positions = q_sol;
      start_time += ros::Duration(0.1);
      new_point.time_from_start = start_time;
      plan1.trajectory_.joint_trajectory.points.push_back(new_point);
    }

    move_group.execute(plan1);

    // ROS_INFO_STREAM("Plan: " << plan1.trajectory_);

    move_group.setNamedTarget("home");
    success = (move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 22(joint space goal) %s", success ? "" : "FAILED");
    move_group.execute(plan1);

    ros::Duration(1.0).sleep();

    ros::shutdown();
    
    return 0;
}