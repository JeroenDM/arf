#ifndef _ROBOT_H
#define _ROBOT_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "opw_kinematics/opw_parameters.h"

#include <Eigen/Dense>
#include <string>
#include <vector>

using Transform = Eigen::Isometry3d;

template <typename T>
opw_kinematics::Parameters<T> makeKukaKr5()
{
  opw_kinematics::Parameters<T> p;
  p.a1 = T(0.180);
  p.a2 = T(-0.120);
  p.b =  T(0.000);
  p.c1 = T(0.400);
  p.c2 = T(0.600);
  p.c3 = T(0.620);
  p.c4 = T(0.115);

  p.offsets[1] = -M_PI / 2.0;
  p.sign_corrections[0] = -1;
  p.sign_corrections[3] = -1;
  p.sign_corrections[5] = -1;

  return p;
}

typedef std::vector<std::vector<double>> IKSolution;


class RobotMoveitWrapper
{
protected:
    robot_model::RobotModelPtr kinematic_model_; /**< Kinematic description of the robot. */
    robot_state::RobotStatePtr kinematic_state_; /**< Kinematic state of the robot */
    const robot_state::JointModelGroup* joint_model_group_; /**< Pointer to planning group */

    planning_scene::PlanningScenePtr planning_scene_; /**< Pointer to planning scene with collision objects */
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    opw_kinematics::Parameters<double> robot_parameters_ = makeKukaKr5<double>(); /**< Geometric parameters for ik */
    bool check_collisions_ = true;
public:
    RobotMoveitWrapper();
    ~RobotMoveitWrapper() = default;

    void updatePlanningScene();
    bool isInJointLimits(const std::vector<double>& q) const;
    bool isInCollision(const std::vector<double>& joint_pose) const;
    const Eigen::Affine3d fk(const std::vector<double>& q, const std::string& frame = "tool_tip") const;
    const IKSolution ik(const Transform pose);
    
    const Eigen::Affine3d getLinkFixedRelativeTransform(const std::string & name) const;

    // print info functions
    void printCurrentJointValues() const;
    void printJointFixedRelativeTransforms() const;

    void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, std::vector<double>& joint_pose);
};

class Robot : public RobotMoveitWrapper {};

#endif