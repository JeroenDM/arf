#include <ros/ros.h>
#include <vector>
#include <Eigen/Geometry>
#include <exception>

#include <moveit_visual_tools/moveit_visual_tools.h>

//#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_moveit_wrapper/redundant_robot.h"
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
};

class Demo2
{
  std::vector<std::vector<std::vector<double>>> graph_data_;
  std::vector<TrajectoryPoint> ee_trajectory_;
  std::vector<std::vector<double>> shortest_path_;
public:
  void createAndShowTrajectory(Rviz& rviz);
  void createGraphData(RedundantRobot& robot);
  void calculateShortestPath(RedundantRobot& robot);
  void showShortestPath(RedundantRobot& robot, Rviz& rviz);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_moveit_wrapper");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Demo 2");


    RedundantRobot robot;
    Rviz rviz;
    rviz.clear();

    Demo2 demo;
    demo.createAndShowTrajectory(rviz);
    demo.createGraphData(robot);
    demo.calculateShortestPath(robot);
    demo.showShortestPath(robot, rviz);

    // // calculate forward kinematics to get reachable pose
    // std::vector<double> q1 = {0.0, 0.0, 0, 0, 0, 0, 0, 0};
    // auto fk_pose = robot.fk(q1, "tool_tip");

    // // move the fk_pose closer to robot
    // Eigen::Vector3d p1(0.3, 0.5, -0.3);
    // fk_pose.translate(p1);

    // // visualize fk_pose
    // rviz.visual_tools_->publishAxisLabeled(fk_pose, "fk_pose");
    // rviz.visual_tools_->trigger();

    // ros::Duration(1.0).sleep();

    // // set values for fixed joints = y- and z-rail
    // std::vector<double> q_fixed = {0, 0};
    
    // //auto ik_sol = robot.redundantIk(fk_pose, q_fixed);
    // auto ik_sol = robot.ikGridSamples(fk_pose);

    // // show ik solutions
    // if (ik_sol.size() > 0)
    // {
    //     for (auto q_sol : ik_sol)
    //     {
    //         robot.plot(rviz.visual_tools_, q_sol);
    //         ros::Duration(0.4).sleep();
    //     }
    // }
    // else
    // {
    //     ROS_INFO("No ik solutions found");
    // }

    ros::Duration(1.0).sleep();

    ros::shutdown();
    
    return 0;
}

void Demo2::createAndShowTrajectory(Rviz& rviz)
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
  for (auto tp : ee_trajectory_)
  {
    tp.plot(rviz.visual_tools_);
  }
}

void Demo2::createGraphData(RedundantRobot& robot)
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

void Demo2::calculateShortestPath(RedundantRobot& robot)
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

void Demo2::showShortestPath(RedundantRobot& robot, Rviz& rviz)
{
  for (auto q : shortest_path_)
  {
    robot.plot(rviz.visual_tools_, q);
    ros::Duration(0.5).sleep();
  }
}