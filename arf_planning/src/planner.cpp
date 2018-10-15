#include "arf_planning/planner.h"

bool Planner::createGraphData(RedundantRobot& robot)
{
  ROS_INFO_STREAM("Received trajectory of with points: " << ee_trajectory_.size());
  for (auto tp : ee_trajectory_)
  {
    std::vector<std::vector<double>> new_data;
    if (tp.is_joint_pose_specified_)
    {
      ROS_INFO_STREAM("Fixed joint pose tp.");
      new_data.push_back(tp.joint_pose_);
    }
    else {
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
          //throw std::runtime_error("No ik found");
          return false;
      }
      ROS_INFO_STREAM("Found collision free solutions: " << new_data.size());
    }
    graph_data_.push_back(new_data);
  }
  return true;
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

bool Planner::run(RedundantRobot& robot, std::vector<TrajectoryPoint>& task)
{
  setTrajectory(task);
  if (createGraphData(robot))
  {
    calculateShortestPath(robot);
    return true;
  }
  return false;
}