#ifndef _PLANNER_H_
#define _PLANNER_H_

#include <vector>

#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <simple_moveit_wrapper/robot.h>

#include "arf_trajectory/trajectory.h"
#include "arf_graph/graph.h"

namespace arf
{

class Planner
{
  using Robot = simple_moveit_wrapper::Robot;

  std::vector<std::vector<std::vector<double>>> graph_data_;
  std::vector<TrajectoryPoint> ee_trajectory_;
  std::vector<std::vector<double>> shortest_path_;

public:
  Planner() = default;
  ~Planner() = default;
  void setTrajectory(std::vector<TrajectoryPoint>& traj)
  {
    ee_trajectory_ = traj;
  }
  bool createGraphData(Robot& robot);
  void calculateShortestPath(Robot& robot);
  std::vector<std::vector<double>> getShortestPath()
  {
    return shortest_path_;
  }
  void showShortestPath(Robot& robot, moveit_visual_tools::MoveItVisualToolsPtr vs);

  const std::vector<std::vector<double>>& getTPData(int index)
  {
    return graph_data_.at(index);
  }

  bool run(Robot& robot, std::vector<TrajectoryPoint>& task);
};

}  // namespace arf

#endif
