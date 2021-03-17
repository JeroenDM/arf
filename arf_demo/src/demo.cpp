#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <simple_moveit_wrapper/industrial_robot.h>
#include "arf_trajectory/trajectory.h"
#include "arf_graph/graph.h"
#include "arf_graph/util.h"

namespace rvt = rviz_visual_tools;
namespace smw = simple_moveit_wrapper;

class Rviz
{
public:
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  Rviz()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link", "/visualization_marker_array"));
    visual_tools_->loadRobotStatePub("/display_robot_state");
  }

  void plotPose(Eigen::Affine3d pose);
  void clear();
};

class Demo1
{
  std::vector<std::vector<std::vector<double>>> graph_data_;
  std::vector<arf::TrajectoryPoint> ee_trajectory_;
  std::vector<std::vector<double>> shortest_path_;

public:
  void createAndShowTrajectory(Rviz& rviz);
  void createGraphData(smw::Robot& robot);
  void calculateShortestPath(smw::Robot& robot);
  void showShortestPath(smw::Robot& robot, Rviz& rviz);
};

std::vector<arf::TrajectoryPoint> createPath();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_moveit_wrapper");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // arf::Robot robot;
  smw::IndustrialRobot robot;
  Rviz rviz;
  Demo1 demo1;
  demo1.createAndShowTrajectory(rviz);
  demo1.createGraphData(robot);
  demo1.calculateShortestPath(robot);
  demo1.showShortestPath(robot, rviz);
  ros::shutdown();
  return 0;
}

std::vector<arf::TrajectoryPoint> createPath()
{
  const int num_path_points = 5;
  const std::vector<double> p1 = { 1, 0, 0.4 };
  const std::vector<double> p2 = { 1.5, 0, 0.5 };

  std::vector<arf::TrajectoryPoint> path;

  double s = 0;
  double ds = 1 / (num_path_points - 1);
  for (int i = 0; i < num_path_points; ++i)
  {
    arf::Number x(p1[0] * (1 - s) + p2[0] * s);
    arf::Number y(p1[1] * (1 - s) + p2[1] * s);
    arf::Number z(p1[2] * (1 - s) + p2[2] * s);
    arf::Number rx(0), ry(M_2_PI), rz(0);
    arf::TrajectoryPoint tp(x, y, z, rx, ry, rz);
    path.push_back(tp);
    s += ds;
  }
  return path;
}

void Rviz::plotPose(Eigen::Affine3d pose)
{
  Eigen::Isometry3d pose_temp(pose.matrix());
  visual_tools_->publishAxis(pose_temp, rvt::LARGE);
  visual_tools_->trigger();
  ros::Duration(0.1).sleep();
}

void Rviz::clear()
{
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
  ros::Duration(0.1).sleep();
}

void Demo1::createAndShowTrajectory(Rviz& rviz)
{
  for (int i = 0; i < 10; ++i)
  {
    arf::TolerancedNumber x(0.5, 0.45, 0.55, 5);
    arf::Number y, z(0.5 + static_cast<double>(i) / 20);
    arf::Number rx, ry(M_PI_2), rz;
    arf::TrajectoryPoint tp(x, y, z, rx, ry, rz);
    ee_trajectory_.push_back(tp);
  }
  for (auto tp : ee_trajectory_)
  {
    rviz.plotPose(tp.getNominalPose());
    // tp.plot(rviz.visual_tools_);
  }
}

void Demo1::createGraphData(smw::Robot& robot)
{
  for (auto tp : ee_trajectory_)
  {
    std::vector<std::vector<double>> new_data;
    for (auto pose : tp.getGridSamples())
    {
      for (auto q_sol : robot.ik(pose))
      {
        if (!robot.isColliding(q_sol))
          new_data.push_back(q_sol);
      }
    }
    graph_data_.push_back(new_data);
  }
}

void Demo1::calculateShortestPath(smw::Robot& robot)
{
  arf::Graph demo_graph(graph_data_);
  demo_graph.runMultiSourceDijkstra();
  std::vector<arf::Node*> sp = demo_graph.getShortestPath();
  std::cout << "Shortest path \n";
  for (auto node : sp)
  {
    std::cout << (*node) << std::endl;
    shortest_path_.push_back(*(*node).jv);
  }
}

void Demo1::showShortestPath(smw::Robot& robot, Rviz& rviz)
{
  for (auto q : shortest_path_)
  {
    robot.plot(rviz.visual_tools_, q);
    ros::Duration(0.5).sleep();
  }
}
