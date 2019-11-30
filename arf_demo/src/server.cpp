#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_trajectory/trajectory.h"
#include "arf_demo/visual_tools_wrapper.h"
#include "arf_demo/planning.h"


arf::Trajectory createTrajectory()
{
  arf::Trajectory ee_trajectory_;
  for (int i = 0; i < 10; ++i)
  {
    Number x(0.98);
    Number y(-0.5 + static_cast<double>(i) / 9);
    Number z(0.02);
    Number rx(0.0), ry(135.0 * M_PI / 180.0); //, ry(-M_PI);
    //TolerancedNumber ry(-M_PI, -M_PI - 1.0, -M_PI + 1.0, 5);
    TolerancedNumber rz(0, -M_PI, M_PI, 10);
    TrajectoryPoint tp(x, y, z, rx, ry, rz);
    ee_trajectory_.push_back(tp);
  }
  return ee_trajectory_;
}


// void showShortestPath(Robot& robot, arf::Rviz& rviz, arf::JointPath& jp)
// {
//   for (auto q : jp)
//   {
//     robot.plot(rviz.visual_tools_, q);
//     ros::Duration(0.5).sleep();
//   }
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arf_cart_planning_server");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(3);
  spinner.start();

  Robot robot;
  arf::Rviz rviz;

  auto ee_traj = createTrajectory();
  auto gd = arf::calculateValidJointPoses(robot, ee_traj, rviz);
  std::cout << "Created graph data.\n";
  std::cout << gd << std::endl;
  auto jp = arf::calculateShortestPath(robot, gd);

  rviz.plotPath(robot, jp);

  ros::shutdown();
  return 0;
}