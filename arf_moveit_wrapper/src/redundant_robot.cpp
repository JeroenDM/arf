#include "arf_moveit_wrapper/redundant_robot.h"

void printGrid(std::vector<std::vector<double>>& grid)
{
  if (grid.size() > 0)
  {
    std::cout << "===== GRID =======\n";
    for (auto jv : grid)
    {
      std::cout << "( ";
      for (auto val : jv)
      {
        std::cout << val << ", ";
      }
      std::cout << ")\n";
    }
  }
  else
  {
    std::cout << "===== NO GRID ======";
  }
  std::cout << std::endl;
}

RedundantRobot::RedundantRobot() : RobotMoveitWrapper()
{
  ROS_INFO("Configuring redundant joints");

  auto bnd1 = joint_model_group_->getJointModel("rail_base_to_carrier")->getVariableBounds();
  //auto bnd2 = joint_model_group_->getJointModel("z_rail_to_robot_mount")->getVariableBounds();

  sampler_.addDimension(bnd1[0].min_position_, bnd1[0].max_position_, 20);
  //sampler_.addDimension(bnd2[0].min_position_, bnd2[0].max_position_, 3);
}

// IKSolution RedundantRobot::redundantIk(const Eigen::Affine3d& pose, std::vector<double>& q_fixed)
// {

//   // get base frame for 6dof robot ik
//   std::vector<double> q_dummy = {q_fixed[0], q_fixed[1], 0, 0, 0, 0, 0, 0};
//   auto base_link_pose = fk(q_dummy, "base_link");

//   // get tool pose corrected for tool tip to tool0 frame
//   auto tool0_to_tool_tip = getLinkFixedRelativeTransform("torch") * getLinkFixedRelativeTransform("tool_tip");

//   // tranfrom pose to reference frame of 6dof robot
//   auto sub_robot_pose = base_link_pose.inverse() * pose * tool0_to_tool_tip.inverse();

//   // solve ik
//   auto solution = ik(sub_robot_pose);

//   for (auto& q_sol : solution)
//   {
//     q_sol.insert(q_sol.begin(), q_fixed[1]);
//     q_sol.insert(q_sol.begin(), q_fixed[0]);
//   }

//   return solution;
// }

IKSolution RedundantRobot::redundantIk(const Eigen::Affine3d& pose, std::vector<double>& q_fixed)
{

  // get base frame for 6dof robot ik
  std::vector<double> q_dummy = {q_fixed[0], 0, 0, 0, 0, 0, 0};
  auto base_link_pose = fk(q_dummy, "base_link");

  // get tool pose corrected for tool tip to tool0 frame
  //auto tool0_to_tool_tip = getLinkFixedRelativeTransform("torch") * getLinkFixedRelativeTransform("tool_tip");

  // tranfrom pose to reference frame of 6dof robot
  auto sub_robot_pose = base_link_pose.inverse() * pose; // * tool0_to_tool_tip.inverse();
  Eigen::Isometry3d temp_pose(sub_robot_pose.matrix());

  // solve ik
  auto solution = ik(temp_pose);

  for (auto& q_sol : solution)
  {
    q_sol.insert(q_sol.begin(), q_fixed[0]);
  }

  return solution;
}

IKSolution RedundantRobot::ikGridSamples(const Eigen::Affine3d& pose)
{
  IKSolution all_ik_sols, temp_ik_sol;
  auto fixed_joints_sampels = sampler_.getGridSamples();
  for (auto q_fixed : fixed_joints_sampels)
  {
    temp_ik_sol = redundantIk(pose, q_fixed);
    if (temp_ik_sol.size() > 0)
      all_ik_sols.insert(all_ik_sols.begin(), temp_ik_sol.begin(), temp_ik_sol.end());
  }
  return all_ik_sols;
}