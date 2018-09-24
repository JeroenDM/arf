#include "arf_moveit_wrapper/redundant_robot.h"

IKSolution RedundantRobot::redundantIk(Eigen::Affine3d pose, std::vector<double>& q_fixed)
{

  // get base frame for 6dof robot ik
  std::vector<double> q_dummy = {q_fixed[0], q_fixed[1], 0, 0, 0, 0, 0, 0};
  auto base_link_pose = fk(q_dummy, "base_link");

  // get tool pose corrected for tool tip to tool0 frame
  auto tool0_to_tool_tip = getLinkFixedRelativeTransform("torch") * getLinkFixedRelativeTransform("tool_tip");

  // tranfrom pose to reference frame of 6dof robot
  auto sub_robot_pose = base_link_pose.inverse() * pose * tool0_to_tool_tip.inverse();

  // solve ik
  auto solution = ik(sub_robot_pose);

  for (auto& q_sol : solution)
  {
    q_sol.insert(q_sol.begin(), q_fixed[1]);
    q_sol.insert(q_sol.begin(), q_fixed[0]);
  }

  return solution;
}