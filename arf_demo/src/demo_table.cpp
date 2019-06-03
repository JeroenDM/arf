#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <sstream>

#include "ros/package.h"
#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_trajectory/trajectory.h"
#include "arf_graph/graph.h"
#include "arf_graph/util.h"

#include "util.h"

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
  void orientationFreeSampling2(Robot& robot, Rviz& rviz);
  void sampleNearSolution(Robot& robot, Rviz& rviz, double dist);
  void showTrajectory(Rviz& rviz);
  void readTaskFromYaml(const std::string filename);
};

std::vector<TrajectoryPoint> createPath();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_table");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  

  Robot robot;
  Rviz rviz;
  Demo1 demo1;

  rviz.clear();

  std::string filename = ros::package::getPath("arf_demo") + "/config/table_task.csv";

  demo1.readTaskFromYaml(filename);
  demo1.showTrajectory(rviz);

  

  // demo1.createAndShowTrajectory2(rviz);

  //demo1.createGraphData(robot);
  demo1.orientationFreeSampling(robot);
  //demo1.orientationFreeSampling2(robot, rviz);

  demo1.calculateShortestPath(robot);
  demo1.showShortestPath(robot, rviz);

  std::vector<double> home = {0, 0, 0, 0, 0, 0};
  robot.plot(rviz.visual_tools_, home);

  demo1.sampleNearSolution(robot, rviz, 0.2);
  demo1.calculateShortestPath(robot);
  demo1.showShortestPath(robot, rviz);

  rviz.clear();
  demo1.showTrajectory(rviz);

  rviz.clear();

  demo1.sampleNearSolution(robot, rviz, 0.05);
  demo1.calculateShortestPath(robot);
  demo1.showShortestPath(robot, rviz);

  rviz.clear();
  demo1.showTrajectory(rviz);

  ros::shutdown();

  return 0;
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

void Demo1::showTrajectory(Rviz& rviz)
{
  for (auto tp : ee_trajectory_2_)
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
    Number rx(0.0), ry(M_PI), rz(0.0);
    FreeOrientationPoint tp(x, y, z, rx, ry, rz);
    ee_trajectory_2_.push_back(tp);
  }
  for (auto tp : ee_trajectory_2_)
  {
    tp.plot(rviz.visual_tools_);
  }
}

void Demo1::readTaskFromYaml(const std::string filename)
{
  ee_trajectory_2_.clear();

  std::ifstream data_file;
  data_file.open(filename);

  std::string line, number;
  std::vector<double> pt;
  int i = -1;
  while (std::getline(data_file, line))
  {
    if (i < 0)
    {
      // skip first line
      ++i;
      continue;
    }
    std::stringstream line_stream(line);
    pt.clear();
    while(std::getline(line_stream, number, ','))
    {
      pt.push_back(std::stod(number));
    }
    
    Number x(pt[0]), y(pt[1]), z(pt[2]);
    Number rx(pt[3]), ry(pt[4]), rz(pt[5]);
    FreeOrientationPoint tp(x, y, z, rx, ry, rz);
    ee_trajectory_2_.push_back(tp);
    ++i;
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
    for (auto pose : tp.sampleUniform(500))
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

void Demo1::sampleNearSolution(Robot& robot, Rviz& rviz, double dist)
{
  if (shortest_path_.size() < 1)
  {
    throw std::invalid_argument("There is no solution to sample around."); 
  }

  auto temp = robot.getLinkFixedRelativeTransform("torch") * robot.getLinkFixedRelativeTransform("tool_tip");
  Transform tool0_to_tool_tip(temp.matrix());
  tool0_to_tool_tip = tool0_to_tool_tip.inverse();

  graph_data_.clear();
  for (std::size_t i = 0; i < shortest_path_.size(); ++i)
  {
    auto fk_sol = robot.fk(shortest_path_[i]);
    Transform nom_pose(fk_sol.matrix());
    ee_trajectory_2_[i].setNominalPose(nom_pose);

    std::vector<std::vector<double>> new_data;
    for (auto pose : ee_trajectory_2_[i].sampleUniformNear(dist, 100))
    {
      rviz.plotPose(pose);

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

void Demo1::orientationFreeSampling2(Robot& robot, Rviz& rviz)
{
  // xyz="0.315 0 0.035" rpy="0 2.3560569232 0" -1.57079632679
  auto temp = robot.getLinkFixedRelativeTransform("torch") * robot.getLinkFixedRelativeTransform("tool_tip");
  Transform tool0_to_tool_tip(temp.matrix());
  tool0_to_tool_tip = tool0_to_tool_tip.inverse();

  std::cout << "Trajectory length: " << ee_trajectory_2_.size() << std::endl;

  for (auto tp : ee_trajectory_2_)
  {
    std::vector<std::vector<double>> new_data;

    Eigen::Isometry3d near_pose = tp.getNominalPose();

    for (auto pose :tp.sampleUniformNear(0.1, 500))
    {
      //pose = pose * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
      rviz.plotPose(pose);
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
  shortest_path_.clear();
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