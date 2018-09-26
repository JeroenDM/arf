#ifndef _PLANNER_H_
#define _PLANNER_H_

#include <vector>

#include "arf_moveit_wrapper/redundant_robot.h"
#include "arf_trajectory/trajectory.h"
#include "arf_graph/graph.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

class Planner {
  std::vector<std::vector<std::vector<double>>> graph_data_;
  std::vector<TrajectoryPoint> ee_trajectory_;
  std::vector<std::vector<double>> shortest_path_;
public:
  Planner();
  void createTrajectory();
  void createGraphData(RedundantRobot& robot);
  void calculateShortestPath(RedundantRobot& robot);
  std::vector<std::vector<double>> getShortestPath() { return shortest_path_; }
  void showShortestPath(RedundantRobot& robot, moveit_visual_tools::MoveItVisualToolsPtr vs);
};

#endif