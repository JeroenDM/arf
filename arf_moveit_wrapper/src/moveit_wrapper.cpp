#include "arf_moveit_wrapper/moveit_wrapper.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/link_model.h>

#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_utilities.h"

//#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

RobotMoveitWrapper::RobotMoveitWrapper()
{
    // load robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model_ = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());

    // load robot state
    kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
    kinematic_state_->setToDefaultValues();
    joint_model_group_ = kinematic_model_->getJointModelGroup("manipulator");

    // create planning scene to for collision checking
    planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    updatePlanningScene();
}

void RobotMoveitWrapper::updatePlanningScene()
{
    // I'm not sure yet how this works
    planning_scene_monitor_->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_);
    ps->getCurrentStateNonConst().update();
    planning_scene_ = ps->diff();
    planning_scene_->decoupleParent();
}

/**
 * @brief Get the relative position of a link frame relative the the previous link
 * when the joint value is zero.
 * This corresponds to the position of the joint in the urdf file that has the given frame
 * as a child link.
 * @param frame The name of the frame to get the relative pose for.
 */
const Eigen::Affine3d RobotMoveitWrapper::getLinkFixedRelativeTransform(const std::string & frame) const
{
    return kinematic_model_->getLinkModel(frame)->getJointOriginTransform();
}

/**
 * @brief Check whether given joint values satisfy the joint limits
 * @param q Joint values.
 */
bool RobotMoveitWrapper::isInJointLimits(const std::vector<double>& q) const
{
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    return kinematic_state_->satisfiesBounds();
}

bool RobotMoveitWrapper::isInCollision(const std::vector<double>& joint_pose) const
{
  bool in_collision = false;
  if (check_collisions_)
  {
    //ROS_INFO("Checking for collision.");

    //planning_scene_->printKnownObjects(std::cout);

    kinematic_state_->setJointGroupPositions(joint_model_group_, joint_pose);
    in_collision = planning_scene_->isStateColliding(*kinematic_state_);
  }
  return in_collision;
}

// bool MoveitStateAdapter::isInLimits(const std::vector<double> &joint_pose) const
// {
//   return joint_group_->satisfiesPositionBounds(joint_pose.data());
// }

/**
 * @brief Forward kinematics for a given frame, default "tool0" frame
 * @param q The joint values for which fk is calculated.
 * @param frame The name of the frame of which the pose is calculated.
 */
const Eigen::Affine3d RobotMoveitWrapper::fk(const std::vector<double>& q, const std::string& frame) const
{
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    return kinematic_state_->getGlobalLinkTransform(frame);
}

const IKSolution RobotMoveitWrapper::ik(const Transform pose)
{
      IKSolution joint_poses;

    // Transform input pose
    // needed if we introduce a tip frame different from tool0
    // or a different base frame
    // Eigen::Affine3d tool_pose = diff_base.inverse() * pose *
    // tip_frame.inverse();


    // convert pose from tool_tip to tool0 frame
    auto temp = getLinkFixedRelativeTransform("torch") * getLinkFixedRelativeTransform("tool_tip");
    Transform tool0_to_tool_tip(temp.matrix());
    tool0_to_tool_tip = tool0_to_tool_tip.inverse();
    Eigen::Isometry3d pose_temp;
    pose_temp = pose * tool0_to_tool_tip;

    std::array<double, 6 * 8> sols;
    opw_kinematics::inverse(robot_parameters_, pose_temp, sols.data());

    // Check the output
    std::vector<double> tmp(6);  // temporary storage for API reasons
    for (int i = 0; i < 8; i++)
    {
        double* sol = sols.data() + 6 * i;
        if (opw_kinematics::isValid(sol))
        {
        opw_kinematics::harmonizeTowardZero(sol);

        // TODO: make this better...
        std::copy(sol, sol + 6, tmp.data());
        // if (isValid(tmp))
        // {
        joint_poses.push_back(tmp);
        // }
        }
    }

    return joint_poses;
}

// print info functions
void RobotMoveitWrapper::printCurrentJointValues() const
{
    const std::vector<std::string>& joint_names = joint_model_group_->getVariableNames();
    std::vector<double> joint_values;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
}

void RobotMoveitWrapper::printJointFixedRelativeTransforms() const
{
    const std::vector< moveit::core::LinkModel* >  lms = kinematic_model_->getLinkModels();
    for (auto link : lms)
    {
        ROS_INFO_STREAM( "====== link " << link->getName() );
        Eigen::Vector3d pi = link->getJointOriginTransform().translation();
        ROS_INFO_STREAM( "Translation: " <<  pi.transpose());
    }
}

void RobotMoveitWrapper::plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, std::vector<double>& joint_pose)
{
    namespace rvt = rviz_visual_tools;
    kinematic_state_->setJointGroupPositions(joint_model_group_, joint_pose);
    //printCurrentJointValues();
    mvt->publishRobotState(kinematic_state_, rvt::DEFAULT);
    mvt->trigger();
}