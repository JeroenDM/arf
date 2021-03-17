#include "arf_planning/planner.h"

namespace arf
{
bool Planner::createGraphData(smw::Robot& robot)
{
  // hardcoded redundant joint samples
  // this was implemented in the robot class in the past
  // we could add a sampler here, but I'm not sure how much I will use this planning class
  // TODO sample redundant joints here!
  assert(robot.getNumRedDof() == 1);  // hard coded for a single redundant dof;
  std::vector<std::vector<double>> q_red_samples;
  for (double v{ 0.0 }; v <= 2.0; v += 0.1)
    q_red_samples.push_back({ v });

  ROS_INFO_STREAM("Received trajectory of with points: " << ee_trajectory_.size());
  for (auto tp : ee_trajectory_)
  {
    std::vector<std::vector<double>> new_data;
    if (tp.is_joint_pose_specified_)
    {
      ROS_INFO_STREAM("Fixed joint pose tp.");
      new_data.push_back(tp.joint_pose_);
    }
    else
    {
      for (auto pose : tp.getGridSamples())
      {
        for (auto q_red : q_red_samples)
        {
          for (auto q_sol : robot.ik(pose, q_red))
          {
            if (!robot.isColliding(q_sol))
              new_data.push_back(q_sol);
          }
        }
      }
      if (new_data.size() == 0)
      {
        ROS_ERROR("No collision free ik sol found for a tp");
        // throw std::runtime_error("No ik found");
        return false;
      }
      ROS_INFO_STREAM("Found collision free solutions: " << new_data.size());
    }
    graph_data_.push_back(new_data);
  }
  return true;
}

void Planner::calculateShortestPath(smw::Robot& robot)
{
  Graph demo_graph(graph_data_);
  demo_graph.runMultiSourceDijkstra();
  std::vector<Node*> sp = demo_graph.getShortestPath();
  std::cout << "Shortest path \n";
  for (auto node : sp)
  {
    // std::cout << (*node) << std::endl;
    shortest_path_.push_back(*(*node).jv);
  }
}

void Planner::showShortestPath(smw::Robot& robot, moveit_visual_tools::MoveItVisualToolsPtr vs)
{
  for (auto q : shortest_path_)
  {
    robot.plot(vs, q);
    ros::Duration(0.5).sleep();
  }
}

bool Planner::run(smw::Robot& robot, std::vector<TrajectoryPoint>& task)
{
  setTrajectory(task);
  if (createGraphData(robot))
  {
    calculateShortestPath(robot);
    return true;
  }
  return false;
}

}  // namespace arf
