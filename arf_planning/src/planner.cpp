#include "arf_planning/planner.h"

#include <ros/ros.h>
#include <vector>
//#include <Eigen/Geometry>

Planner::Planner()
{
  ROS_INFO("Created planner");
}

void Planner::createTrajectory()
{
  for (int i = 0; i < 10; ++i)
  {
    TolerancedNumber x(1.0, 0.95, 1.05, 5);
    // Number y, z(0.5 + static_cast<double>(i) / 20);
    Number y(static_cast<double>(i) / 20), z(0.5);
    Number rx, ry(M_PI), rz;
    TrajectoryPoint tp(x, y, z, rx, ry, rz);
    ee_trajectory_.push_back(tp);
  }
}

void Planner::createGraphData(RedundantRobot& robot)
{
  for (auto tp : ee_trajectory_)
  {
    std::vector<std::vector<double>> new_data;
    for (auto pose : tp.getGridSamples())
    {
      for (auto q_sol : robot.ikGridSamples(pose))
      {
        if (!robot.isInCollision(q_sol))
            new_data.push_back(q_sol);
      }
    }
    if (new_data.size() == 0)
    {
        ROS_ERROR("No collision free ik sol found for a tp");
        throw std::runtime_error("No ik found");
    }
    ROS_INFO_STREAM("Found collision free solutions: " << new_data.size());
    graph_data_.push_back(new_data);
  }
}

void Planner::calculateShortestPath(RedundantRobot& robot)
{
  Graph demo_graph(graph_data_);
  demo_graph.runMultiSourceDijkstra();
  std::vector<Node*> sp = demo_graph.getShortestPath();
  std::cout << "Shortest path \n";
  for (auto node : sp)
  {
    //std::cout << (*node) << std::endl;
    shortest_path_.push_back(*(*node).jv);
  }
}

void Planner::showShortestPath(RedundantRobot& robot, moveit_visual_tools::MoveItVisualToolsPtr vs)
{
  for (auto q : shortest_path_)
  {
    robot.plot(vs, q);
    ros::Duration(0.5).sleep();
  }
}