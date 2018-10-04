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

std::vector<TrajectoryPoint> createTrajectory()
{
    std::vector<TrajectoryPoint> ee_trajectory;
    for (int i = 0; i < 10; ++i)
    {
        TolerancedNumber x(1.0, 0.95, 1.05, 5);
        // Number y, z(0.5 + static_cast<double>(i) / 20);
        Number y(static_cast<double>(i) / 20), z(0.5);
        Number rx, ry(M_PI), rz;
        TrajectoryPoint tp(x, y, z, rx, ry, rz);
        ee_trajectory.push_back(tp);
    }
    return ee_trajectory;
}

struct WeldingTaskSolution
{
    bool success = false;
    //geometry_msgs::Pose start_pose;
    //geometry_msgs::Pose end_pose;
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
        start_time += ros::Duration(0.1);
    }
    return plan;
}

WeldingTaskSolution planWeld(RedundantRobot robot, std::vector<TrajectoryPoint> ee_trajectory)
{
    WeldingTaskSolution solution;
    Planner planner;
    if (!planner.run(robot, ee_trajectory))
        return solution;
    solution.success = true;

    auto shortest_path = planner.getShortestPath();
    solution.joint_trajectory = shortest_path;
    solution.motion_plan = jointTrajectoryToMoveitPlan(shortest_path);
    //tf::poseEigenToMsg(robot.fk(shortest_path[0]), solution.start_pose);
    //tf::poseEigenToMsg(robot.fk(shortest_path[shortest_path.size() - 1]), solution.end_pose);
    solution.start_joint_pose = shortest_path[0];
    solution.end_joint_pose = shortest_path[shortest_path.size() - 1];

    ROS_INFO_STREAM("Plan size: " << solution.motion_plan.trajectory_.joint_trajectory.points.size());
    return solution;
}

void addPlans(MoveitPlan& plan_a, MoveitPlan& plan_b)
{
    std::size_t Na = plan_a.trajectory_.joint_trajectory.points.size();
    ros::Duration last_time_a = plan_a.trajectory_.joint_trajectory.points[Na-1].time_from_start;
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

    RedundantRobot robot;
    Rviz rviz;
    rviz.clear();

    auto traj = createTrajectory();
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

    // trajectory_msgs::JointTrajectory weld_trajectory;
    // int N = plan1.trajectory_.joint_trajectory.points.size();
    // ros::Duration start_time = plan1.trajectory_.joint_trajectory.points[N - 1].time_from_start;
    // ROS_INFO_STREAM("Start time welding: " << start_time);
    // for (auto& q_sol : solution.joint_trajectory)
    // {
    //     trajectory_msgs::JointTrajectoryPoint new_point;
    //     new_point.positions = q_sol;
    //     start_time += ros::Duration(0.1);
    //     new_point.time_from_start = start_time;
    //     plan1.trajectory_.joint_trajectory.points.push_back(new_point);
    // }

    addPlans(plan1, solution.motion_plan);
    //ROS_INFO_STREAM("Sol plan: " << solution.motion_plan.trajectory_);
    ROS_INFO_STREAM("plan plan: " << plan1.trajectory_);

    move_group.execute(plan1);
    ros::Duration(1.0).sleep();

    //move_group.execute(solution.motion_plan);
    //ros::Duration(1.0).sleep();

    move_group.setNamedTarget("home");
    move_group.move();
    ros::Duration(1.0).sleep();

    ros::shutdown();

    return 0;
}
