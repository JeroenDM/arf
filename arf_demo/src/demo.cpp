#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_trajectory/trajectory.h"
#include "arf_graph/graph.h"
#include "arf_graph/util.h"

namespace rvt = rviz_visual_tools;

class Demo
{
  

public:
moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  Demo()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link","/rviz_visual_tools"));
  }

  void demo1()
  {
    // Create pose
    Eigen::Affine3d pose;
    pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
    pose.translation() = Eigen::Vector3d( 0.1, 0.1, 0.1 ); // translate x,y,z

    // Publish arrow vector of pose
    ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
    visual_tools_->publishAxis(pose, rvt::LARGE);

    // Don't forget to trigger the publisher!
    visual_tools_->trigger();
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

std::vector<TrajectoryPoint> createPath();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_moveit_wrapper");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    RobotMoveitWrapper robot;
    Demo d;
    // robot.printCurrentJointValues();
    // d.demo1();
    // std::vector<double> q1 = {0.1, 0.2, 0.3, 0.5, 0.6, 0.7};
    // robot.plot(d.visual_tools_, q1);

    // create a path
    std::vector<TrajectoryPoint> demo_trajectory;
    for (int i = 0; i < 10; ++i)
    {
      TolerancedNumber x(0.5, 0.45, 0.55, 5);
      Number y, z(0.5 + static_cast<double>(i) / 20);
      Number rx, ry(M_PI_2), rz;
      TrajectoryPoint tp(x, y, z, rx, ry, rz);
      demo_trajectory.push_back(tp);
    }
    // plot path
    IKSolution sol;
    for (auto tp : demo_trajectory)
    {
      tp.plot(d.visual_tools_);
      // sol = robot.ik(tp.getNominalPose());
      // for (auto q_sol : sol)
      // {
      //   robot.plot(d.visual_tools_, q_sol);
      //   ros::Duration(0.1).sleep();
      // }
    }
    ros::Duration(1.0).sleep();

    // IKSolution sol;
    // for (auto tp : demo_trajectory)
    // {
    //   sol = robot.ik(tp.getNominalPose());
    //   for (auto q_sol : sol)
    //   {
    //     robot.plot(d.visual_tools_, q_sol);
    //     ros::Duration(0.5).sleep();
    //   }
    // }

    

    std::vector<std::vector<std::vector<double>>> graph_data;
    //IKSolution sol;

    for (auto tp : demo_trajectory)
    {
      std::vector<std::vector<double>> new_data;
      for (auto pose : tp.getGridSamples())
      {
        for (auto q_sol : robot.ik(pose))
        {
          new_data.push_back(q_sol);
        }
      }
      graph_data.push_back(new_data);
    }

    // create Graph
    Graph demo_graph(graph_data);

    

    demo_graph.runMultiSourceDijkstra();
    std::vector<Node*> sp = demo_graph.getShortestPath();
    std::cout << "Shortest path \n";
    std::vector<std::vector<double>> shortest_path;
    for (auto node : sp)
    {
      std::cout << (*node) << std::endl;
      shortest_path.push_back(*(*node).jv);
    }

    for (auto q : shortest_path)
    {
      robot.plot(d.visual_tools_, q);
      ros::Duration(0.5).sleep();
    }

    // std::vector<Eigen::Affine3d> discrete_poses;
    // discrete_poses = tp.getGridSamples();
    // for (auto p : discrete_poses)
    // {
    //   d.plotPose(p);
    // }

    // IKSolution sol;
    // sol = robot.ik(tp.getNominalPose());
    // for (auto q_sol : sol)
    // {
    //   if (!robot.isInCollision(q_sol))
    //   {
    //     robot.plot(d.visual_tools_, q_sol);
    //     ros::Duration(0.5).sleep();
    //   }
    // }

    ros::Duration(2.0).sleep();
    d.clear();

    // std::vector<double> q1 = {0, 0, 0, 0, 0, 0};
    // ROS_INFO_STREAM("Robot in collision? " << robot.isInCollision(q1)); 
    // ros::Duration(1.0).sleep();

    ros::shutdown();
    return 0;
}

std::vector<TrajectoryPoint> createPath()
{
  const int num_path_points = 5;
  const std::vector<double> p1 = {1, 0, 0.4};
  const std::vector<double> p2 = {1, 0, 0.5};

  std::vector<TrajectoryPoint> path;

  double s = 0;
  double ds = 1 / (num_path_points - 1);
  for (int i = 0; i < num_path_points; ++i)
  {
    Number x(p1[0] * (1 - s) + p2[0] * s);
    Number y(p1[1] * (1 - s) + p2[1] * s);
    Number z(p1[2] * (1 - s) + p2[2] * s);
    Number rx(0), ry(M_2_PI), rz(0);
    TrajectoryPoint tp(x, y, z, rx, ry, rz);
    path.push_back(tp);
    s += ds;
  }
  return path;
}
