#ifndef _ARF_DEMO_PLANNING_H_
#define _ARF_DEMO_PLANNING_H_

#include <vector>
#include <ros/ros.h>

#include "arf_graph/graph.h"
#include "arf_graph/util.h"  // enable std::cout << graph|node
#include "arf_trajectory/trajectory.h"
#include "arf_demo/visual_tools_wrapper.h"

namespace arf
{

bool DEBUG = false;
// ros::param::get<bool>("/arf_debug_flag", DEBUG, false);

// TODO move this to arf_trajectory
using Trajectory = std::vector<TrajectoryPoint>;

using GraphData = std::vector<std::vector<std::vector<double>>>;
using JointPose = std::vector<double>;
using JointPath = std::vector<JointPose>;


GraphData calculateValidJointPoses(Robot& robot, Trajectory& traj, Rviz& rviz)
{
  GraphData graph_data;

  for (auto tp : traj)
  {
    std::vector<std::vector<double>> new_data;
    for (auto pose : tp.getGridSamples())
    {
      if (DEBUG){
        rviz.plotPose(pose);
      }
      for (auto q_sol : robot.ik(pose))
      {
        if (!robot.isInCollision(q_sol))
        {
          new_data.push_back(q_sol);
          if (DEBUG){
            robot.plot(rviz.visual_tools_, q_sol);
          }
        }
      }
    }
    if (new_data.size() == 0)
    {
      throw std::runtime_error("No valid joint poses found for some pose.");
    }
    graph_data.push_back(new_data);
  }
  return graph_data;
}

JointPath calculateShortestPath(Robot& robot, GraphData& gd)
{
  std::cout << "Calculation shortest path..." << std::endl;

  JointPath solution;
  Graph demo_graph(gd);
  demo_graph.runMultiSourceDijkstra();
  std::vector<Node*> sp = demo_graph.getShortestPath();

  for (auto node : sp)
  {
    std::cout << (*node) << std::endl;
    solution.push_back(*(*node).jv);
  }

  return solution;
}

}

#endif