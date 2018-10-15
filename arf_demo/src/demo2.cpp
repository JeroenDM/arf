#include <ros/ros.h>
#include <vector>
#include <Eigen/Geometry>
#include <exception>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <eigen_conversions/eigen_msg.h>

//#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_moveit_wrapper/redundant_robot.h"
#include "arf_trajectory/trajectory.h"
#include "arf_graph/graph.h"
#include "arf_graph/util.h"
#include "arf_planning/planner.h"

namespace rvt = rviz_visual_tools;
using MoveitPlan = moveit::planning_interface::MoveGroupInterface::Plan;

class Rviz
{
  public:
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    Rviz()
    {
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/demo_visuals"));
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
    void animatePath(RedundantRobot& robot, std::vector<JointPose> path)
    {
        for (auto q : path)
        {
            robot.plot(visual_tools_, q);
            ros::Duration(0.5).sleep();
        }
    }
};

std::vector<TrajectoryPoint> createTrajectory(double x_ref = 0.0, double y_ref = 0.0, double z_ref = 0.0)
{
    std::vector<TrajectoryPoint> ee_trajectory;
    for (int i = 0; i < 10; ++i)
    {
        // TolerancedNumber x(1.0 + x_ref, 0.95 + x_ref, 1.05 + x_ref, 5);
        // // Number y, z(0.5 + static_cast<double>(i) / 20);
        // Number y(static_cast<double>(i) / 20 + y_ref), z(0.5 + z_ref);
        // Number rx, ry(M_PI), rz;
        double s = static_cast<double>(i) / 9;
        double xi = -0.15 * (1-s) + 0.15 * s;
        Number x(xi + x_ref);
        Number y(1.45 + y_ref), z(0.2 + z_ref);
        Number rx, ry(M_PI);
        TolerancedNumber rz(0.0, -1.5, 1.5, 10);
        TrajectoryPoint tp(x, y, z, rx, ry, rz);
        ee_trajectory.push_back(tp);
    }
    return ee_trajectory;
}

struct WeldingTaskSolution
{
    bool success = false;
    geometry_msgs::Pose start_pose;
    geometry_msgs::Pose end_pose;
    JointPose start_joint_pose;
    JointPose end_joint_pose;
    std::vector<JointPose> joint_trajectory;
    MoveitPlan motion_plan;    
};

MoveitPlan jointTrajectoryToMoveitPlan(std::vector<JointPose> joint_trajectory)
{
    MoveitPlan plan;
    ros::Duration start_time(0.0);
    for (auto& q : joint_trajectory)
    {
        trajectory_msgs::JointTrajectoryPoint new_point;
        new_point.positions = q;
        new_point.time_from_start = start_time;
        plan.trajectory_.joint_trajectory.points.push_back(new_point);
        start_time += ros::Duration(0.2);
    }
    return plan;
}

void addApproachPath(std::vector<JointPose>& joint_trajectory, RedundantRobot& robot)
{
    ROS_INFO_STREAM("Calculating approach path.");
    Eigen::Affine3d pose = robot.fk(joint_trajectory[0]);
    Eigen::Vector3d position = pose.translation();
    Eigen::Vector3d angles   = pose.rotation().eulerAngles(2, 1, 0);
    ROS_INFO_STREAM("Angles: " << angles);
    std::vector<TrajectoryPoint> ee_trajectory;
    for (int i = 0; i < 5; ++i)
    {
        Number x(position[0]), y(position[1]), z(position[2]);
        //Number rx(angles[0]), ry(angles[1]), rz(angles[2]);
        Number rx, ry(M_PI), rz;
        TrajectoryPoint tp(x, y, z, rx, ry, rz);
        ee_trajectory.push_back(tp);
        position[2] += 0.025;
    }
    Planner planner;
    if (!planner.run(robot, ee_trajectory))
        ROS_ERROR_STREAM("Failed to find approach path.");
    auto jp = planner.getShortestPath();
    std::reverse(jp.begin(), jp.end());
    joint_trajectory.insert(joint_trajectory.begin(), jp.begin(), jp.end());

}

void addRetractPath(std::vector<JointPose>& joint_trajectory, RedundantRobot& robot)
{
    Eigen::Affine3d pose = robot.fk(joint_trajectory.back());
    Eigen::Vector3d position = pose.translation();
    Eigen::Vector3d angles   = pose.rotation().eulerAngles(2, 1, 0);
    ROS_INFO_STREAM("Angles: " << angles);
    std::vector<TrajectoryPoint> ee_trajectory;
    for (int i = 0; i < 5; ++i)
    {
        Number x(position[0]), y(position[1]), z(position[2]);
        //Number rx(angles[0]), ry(angles[1]), rz(angles[2]);
        Number rx, ry(M_PI), rz;
        TrajectoryPoint tp(x, y, z, rx, ry, rz);
        ee_trajectory.push_back(tp);
        position[2] += 0.025;
    }
    Planner planner;
    if (!planner.run(robot, ee_trajectory))
        ROS_ERROR_STREAM("Failed to find retract path.");
    auto jp = planner.getShortestPath();
    joint_trajectory.insert(joint_trajectory.end(), jp.begin(), jp.end());
}

WeldingTaskSolution planWeld(RedundantRobot robot, std::vector<TrajectoryPoint> ee_trajectory)
{
    WeldingTaskSolution solution;
    Planner planner;
    if (!planner.run(robot, ee_trajectory))
        return solution;
    solution.success = true;

    auto shortest_path = planner.getShortestPath();

    addApproachPath(shortest_path, robot);
    addRetractPath(shortest_path, robot);
    solution.joint_trajectory = shortest_path;
    solution.motion_plan = jointTrajectoryToMoveitPlan(shortest_path);

    solution.start_joint_pose = shortest_path.front();
    solution.end_joint_pose = shortest_path.back();
    tf::poseEigenToMsg(robot.fk(solution.start_joint_pose), solution.start_pose);
    tf::poseEigenToMsg(robot.fk(solution.end_joint_pose), solution.end_pose);

    ROS_INFO_STREAM("Plan size: " << solution.motion_plan.trajectory_.joint_trajectory.points.size());
    return solution;
}

void addPlans(MoveitPlan& plan_a, MoveitPlan& plan_b)
{
    //std::size_t Na = plan_a.trajectory_.joint_trajectory.points.size();
    ros::Duration last_time_a = plan_a.trajectory_.joint_trajectory.points.back().time_from_start;
    for (auto jtp : plan_b.trajectory_.joint_trajectory.points)
    {
        jtp.time_from_start += last_time_a;
        plan_a.trajectory_.joint_trajectory.points.push_back(jtp);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_moveit_wrapper");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Demo 2");

    moveit::planning_interface::MoveGroupInterface table("table");
    ROS_INFO("Reference frame: %s", table.getPlanningFrame().c_str());

    //moveit::core::RobotModelConstPtr table_model = table.getRobotModel();
    //const moveit::core::JointModelGroup* jmg = table_model->getJointModelGroup("table");
    robot_state::RobotState table_state(*table.getCurrentState());
    const Eigen::Affine3d wobj_frame = table_state.getGlobalLinkTransform("wobj");
    ROS_INFO_STREAM("===================================");
    ROS_INFO_STREAM("Work object frame: " << wobj_frame.translation().transpose());
    const Eigen::Vector3d offset = wobj_frame.translation();


    RedundantRobot robot;
    Rviz rviz;
    rviz.clear();

    auto traj = createTrajectory(offset[0], offset[1], offset[2]);
    for (auto tp : traj)
    {
        tp.plot(rviz.visual_tools_);
    }

    auto solution = planWeld(robot, traj);

    ROS_INFO_NAMED("tutorial", "Planner %s", solution.success ? "SUCCESS" : "FAILED");
    if (!solution.success)
    {
        ROS_INFO_STREAM("Planning the weld failed, stopping process.");
        ros::shutdown();
        return 0;
    }
    
    //rviz.animatePath(robot, solution.joint_trajectory);


    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    moveit::planning_interface::MoveGroupInterface::Plan plan1, plan2, plan3;

    move_group.setJointValueTarget(solution.start_joint_pose);
    bool success1 = (move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Go to approach pose %s", success1 ? "SUCCESS" : "FAILED");


    rviz.clear();
    std::vector<Eigen::Affine3d> final_ee_path;
    for (auto& q_sol : solution.joint_trajectory)
    {
        final_ee_path.push_back(robot.fk(q_sol));
    }

    for (auto& pose : final_ee_path)
    {
        rviz.plotPose(pose);
    }

    addPlans(plan1, solution.motion_plan);
    //ROS_INFO_STREAM("Sol plan: " << solution.motion_plan.trajectory_);
    //ROS_INFO_STREAM("plan plan: " << plan1.trajectory_);

    move_group.execute(plan1);
    ros::Duration(1.0).sleep();

    //move_group.execute(solution.motion_plan);
    //ros::Duration(1.0).sleep();

    // std::vector<double> group_variable_values;
    // move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), group_variable_values);

    // group_variable_values[3] = 0.0;
    // group_variable_values[4] = 0.0;
    // group_variable_values[5] = 0.0;
    // move_group.setJointValueTarget(group_variable_values);
    // move_group.move();
    // ros::Duration(1.0).sleep();

    move_group.setNamedTarget("home");
    move_group.move();
    ros::Duration(1.0).sleep();

    ros::shutdown();

    return 0;
}
