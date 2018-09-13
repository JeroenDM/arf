#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_trajectory/trajectory.h"

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
    robot.printCurrentJointValues();

    Demo d;
    d.demo1();

    std::vector<double> q1 = {0.1, 0.2, 0.3, 0.5, 0.6, 0.7};
    robot.plot(d.visual_tools_, q1);

    // try plotting a trajectory point
    TolerancedNumber x(0.5, 0, 1);
    Number y, z(0.1);
    Number rx, ry(1.0), rz;
    TrajectoryPoint tp(x, y, z, rx, ry, rz);
    tp.plot(d.visual_tools_);

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
