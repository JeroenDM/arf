#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_trajectory/trajectory.h"
#include "arf_graph/graph.h"
#include "arf_graph/util.h"

namespace rvt = rviz_visual_tools;

class Rviz
{
public:
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  Rviz()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link", "/rviz_visual_tools"));
  }

  void plotPose(Eigen::Affine3d pose);
  void clear();
};

class Demo1
{
  std::vector<std::vector<std::vector<double>>> graph_data_;
  std::vector<TrajectoryPoint> ee_trajectory_;
  std::vector<FreeOrientationPoint> ee_trajectory_2_;
  std::vector<std::vector<double>> shortest_path_;

public:
  void createAndShowTrajectory(Rviz& rviz);
  void createAndShowTrajectory2(Rviz& rviz);
  void createGraphData(Robot& robot);
  void calculateShortestPath(Robot& robot);
  void showShortestPath(Robot& robot, Rviz& rviz);
  void orientationFreeSampling(Robot& robot);
};

std::vector<TrajectoryPoint> createPath();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_moveit_wrapper");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Robot robot;
  Rviz rviz;
  Demo1 demo1;
  demo1.createAndShowTrajectory2(rviz);

  //demo1.createGraphData(robot);
  demo1.orientationFreeSampling(robot);

  demo1.calculateShortestPath(robot);
  demo1.showShortestPath(robot, rviz);
  ros::shutdown();
  return 0;
}

void Rviz::plotPose(Eigen::Affine3d pose)
{
  Eigen::Isometry3d pose_temp(pose.matrix());
  visual_tools_->publishAxis(pose_temp, rvt::LARGE);
  visual_tools_->trigger();
}

void Rviz::clear()
{
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
}

void Demo1::createAndShowTrajectory(Rviz& rviz)
{
  for (int i = 0; i < 10; ++i)
  {
    Number x(0.8);
    Number y(-0.2 + static_cast<double>(i) / 20);
    Number z(0.2);
    Number rx, ry(-M_PI);
    TolerancedNumber rz(0, -M_PI, M_PI, 20);
    TrajectoryPoint tp(x, y, z, rx, ry, rz);
    ee_trajectory_.push_back(tp);
  }
  for (auto tp : ee_trajectory_)
  {
    tp.plot(rviz.visual_tools_);
  }
}

void Demo1::createAndShowTrajectory2(Rviz& rviz)
{
  for (int i = 0; i < 10; ++i)
  {
    Number x(0.8);
    Number y(-0.2 + static_cast<double>(i) / 20);
    Number z(0.2);
    FreeOrientationPoint tp(x, y, z, 1000);
    ee_trajectory_2_.push_back(tp);
  }
  for (auto tp : ee_trajectory_)
  {
    tp.plot(rviz.visual_tools_);
  }
}

void Demo1::createGraphData(Robot& robot)
{
  // xyz="0.315 0 0.035" rpy="0 2.3560569232 0" -1.57079632679
  Transform tool0_to_tool_tip;
  tool0_to_tool_tip = Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(-1.57079632679, Eigen::Vector3d::UnitY());

  tool0_to_tool_tip =
      tool0_to_tool_tip * Eigen::Translation3d(0.315, 0.0, 0.035) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(2.3560569232, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  tool0_to_tool_tip = tool0_to_tool_tip.inverse();

  for (auto tp : ee_trajectory_)
  {
    std::vector<std::vector<double>> new_data;
    for (auto pose : tp.getGridSamples())
    {
      // convert tool frame to robot frame (TODO move this in robot class)
      Eigen::Isometry3d robot_pose;
      robot_pose = pose * tool0_to_tool_tip;

      for (auto q_sol : robot.ik(robot_pose))
      {
        if (!robot.isInCollision(q_sol))
          new_data.push_back(q_sol);
      }
    }
    graph_data_.push_back(new_data);
  }
}

void Demo1::orientationFreeSampling(Robot& robot)
{
  // xyz="0.315 0 0.035" rpy="0 2.3560569232 0" -1.57079632679
  auto temp = robot.getLinkFixedRelativeTransform("torch") * robot.getLinkFixedRelativeTransform("tool_tip");
  Transform tool0_to_tool_tip(temp.matrix());
  tool0_to_tool_tip = tool0_to_tool_tip.inverse();

  std::cout << "Trajectory length: " << ee_trajectory_2_.size() << std::endl;

  for (auto tp : ee_trajectory_2_)
  {
    std::vector<std::vector<double>> new_data;
    for (auto pose : tp.getGridSamples())
    {
      // convert tool frame to robot frame (TODO move this in robot class)
      Eigen::Isometry3d robot_pose;
      robot_pose = pose * tool0_to_tool_tip;

      for (auto q_sol : robot.ik(robot_pose))
      {
        if (!robot.isInCollision(q_sol))
          new_data.push_back(q_sol);
      }
    }
    graph_data_.push_back(new_data);
  }
}

void Demo1::calculateShortestPath(Robot& robot)
{
  Graph demo_graph(graph_data_);
  demo_graph.runMultiSourceDijkstra();
  std::vector<Node*> sp = demo_graph.getShortestPath();
  std::cout << "Shortest path \n";
  for (auto node : sp)
  {
    std::cout << (*node) << std::endl;
    shortest_path_.push_back(*(*node).jv);
  }
}

void Demo1::showShortestPath(Robot& robot, Rviz& rviz)
{
  for (auto q : shortest_path_)
  {
    robot.plot(rviz.visual_tools_, q);
    ros::Duration(0.5).sleep();
  }
}