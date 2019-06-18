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

#include "arf_demo/TrajOptPlanningServer.h"
#include "trajectory_msgs/JointTrajectory.h"

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
        Eigen::Isometry3d pose_temp(pose.matrix());
        visual_tools_->publishAxis(pose_temp, rvt::LARGE);
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
            ros::Duration(0.3).sleep();
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
        Number rx(M_PI_4), ry(M_PI);
        TolerancedNumber rz(0.0, -1.5, 1.5, 10);
        TrajectoryPoint tp(x, y, z, rx, ry, rz);
        ee_trajectory.push_back(tp);
    }
    return ee_trajectory;
}

std::vector<TrajectoryPoint> createWeldLine(const Eigen::Affine3d start_pose, const Eigen::Vector3d end_position, int num_points)
{
    Eigen::Vector3d position = start_pose.translation();
    Eigen::Vector3d angles   = start_pose.rotation().eulerAngles(0, 1, 2);

    std::vector<TrajectoryPoint> line;
    //double s;
    Eigen::Vector3d step = (end_position - position) / (num_points - 1);
    for (int i = 0; i < num_points; ++i)
    {
        Number x(position[0]), y(position[1]), z(position[2]);
        Number ry(angles[1]);
        Number rx(angles[0]);
        //TolerancedNumber rx(angles[0], angles[0] - 0.5, angles[0] + 0.5, 5);
        TolerancedNumber rz(angles[2], angles[2] - 1.5, angles[2] + 1.5, 10);
        TrajectoryPoint tp(x, y, z, rx, ry, rz);
        line.push_back(tp);
        //s = static_cast<double>(i) / (num_points - 1);
        //ROS_INFO_STREAM("s value: " << s);
        //position = (1.0 - s) * position + s * end_position;
        //position = position + s * (end_position - position);
        position += step;
    }
    return line;
}

std::vector<TrajectoryPoint> readTask(ros::NodeHandle& nh, const std::string task_name, Eigen::Vector3d wobj_pos)
{
    // read points from parameter server
    std::vector<double> p1, p2, angles;
    if (nh.hasParam(task_name))
    {
        nh.getParam(task_name + "/p1", p1);
        nh.getParam(task_name + "/p2", p2);
        nh.getParam(task_name + "/angles", angles);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to read task points from parameter server");
    }

    using Translation = Eigen::Translation3d;
    using AngleAxis = Eigen::AngleAxisd;
    using Vector = Eigen::Vector3d;

    // clang-format off
    Eigen::Affine3d start_pose;
    start_pose = Translation(wobj_pos[0] + p1[0], wobj_pos[1] + p1[1], wobj_pos[2] + p1[2]) *
                 AngleAxis(angles[0] * M_PI / 180.0, Vector::UnitX()) *
                 AngleAxis(angles[1] * M_PI / 180.0 , Vector::UnitY()) *
                 AngleAxis(angles[2] * M_PI / 180.0 , Vector::UnitZ());
    // clang-format on

    Vector end_position(wobj_pos[0] + p2[0], wobj_pos[1] + p2[1], wobj_pos[2] + p2[2]);

    return createWeldLine(start_pose, end_position, 10);
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

    // for debugging
    std::vector<JointPose> tp_data;    
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

void printJointTrajectory(std::vector<JointPose> joint_trajectory)
{
    for (auto jp : joint_trajectory)
    {
        ROS_INFO_STREAM(
            "( " << jp[0] << ", " << jp[1] << ", " <<
            jp[2] << ", " << jp[3] << ", " <<
            jp[4] << ", " << jp[5] << ", " <<
            jp[6] << ")"
        );
    }
}

std::vector<TrajectoryPoint> createLine(const Eigen::Affine3d start_pose, const Eigen::Vector3d direction, const double length)
{
    Eigen::Vector3d position = start_pose.translation();
    Eigen::Vector3d angles   = start_pose.rotation().eulerAngles(0, 1, 2);

    std::vector<TrajectoryPoint> line;
    //double s;
    Eigen::Vector3d step = length * direction / (4.0);
    for (int i = 0; i < 5; ++i)
    {
        Number x(position[0]), y(position[1]), z(position[2]);
        Number rx(angles[0]), ry(angles[1]), rz(angles[2]);
        TrajectoryPoint tp(x, y, z, rx, ry, rz);
        line.push_back(tp);
        //s = static_cast<double>(i) / 4 * 0.1;
        //position = position + s * direction;
        position += step;
    }
    return line;
}

void addApproachPath(std::vector<JointPose>& joint_trajectory, RedundantRobot& robot)
{
    ROS_INFO_STREAM("Calculating approach path.");
    Eigen::Affine3d pose = robot.fk(joint_trajectory[0]);
    Eigen::Vector3d direction = - pose.rotation() * Eigen::Vector3d::UnitZ();
    auto ee_trajectory = createLine(pose, direction, 0.1);
    TrajectoryPoint jtp(joint_trajectory.front());
    //ee_trajectory.push_back(jtp);
    ee_trajectory.insert(ee_trajectory.begin(), jtp);

    Planner planner;
    if (!planner.run(robot, ee_trajectory))
        ROS_ERROR_STREAM("Failed to find approach path.");
    auto jp = planner.getShortestPath();
    std::reverse(jp.begin(), jp.end());
    joint_trajectory.insert(joint_trajectory.begin(), jp.begin(), jp.end());
}

void addRetractPath(std::vector<JointPose>& joint_trajectory, RedundantRobot& robot)
{
    ROS_INFO_STREAM("Calculating retract path.");
    Eigen::Affine3d pose = robot.fk(joint_trajectory.back());
    Eigen::Vector3d direction = - pose.rotation() * Eigen::Vector3d::UnitZ();
    std::vector<TrajectoryPoint> ee_trajectory = createLine(pose, direction, 0.1);
    TrajectoryPoint jtp(joint_trajectory.back());
    ee_trajectory.insert(ee_trajectory.begin(), jtp);

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

    // for debugging
    solution.tp_data = planner.getTPData(0);

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

std::vector<JointPose> planPTP(std::vector<double> start, std::vector<double> goal, ros::NodeHandle& node_handle)
{
    ros::ServiceClient client = node_handle.serviceClient<arf_demo::TrajOptPlanningServer>("trajopt_planning_server");
    arf_demo::TrajOptPlanningServer srv;

    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.num_path_points = 20;
    srv.request.type = "joint_goal";

    if (client.call(srv))
    {
        ROS_INFO_STREAM("Result of planning request: " << std::to_string(srv.response.success));
        ROS_INFO_STREAM(srv.response);
    }
    else
    {
        ROS_ERROR("Failed to call service trajopt_planning_server");
    }
    std::vector<JointPose> solution;
    for (auto pt : srv.response.trajectory)
    {
        solution.push_back(pt.positions);
    }
    return solution;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo2");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Demo 2");

    // de verschillende configuraties voor de drie laslijnen maken dat de tussenin paden moeilijk gaan
    // dus ideaal gezien
    // 1) trajopt joint_pose -> next start pose
    // 2) descartes joint_pose at first tp -> cart_pose.
    // REPEAT

    // test planning server
    // ======================================================================================
    // std::vector<double> zeros = {0, 0, 0, 0, 0, 0, 0};
    // std::vector<double> goal = {0.984902, -2.70526, -0.336027, 0.191903, 2.40635, 1.70897, 1.92218};

    // auto ptp_solution = planPTP(zeros, goal, node_handle);

    // case
    // ======================================================================================

    moveit::planning_interface::MoveGroupInterface table("rotation_table");
    robot_state::RobotState table_state(*table.getCurrentState());
    const Eigen::Affine3d wobj_frame = table_state.getGlobalLinkTransform("wobj");
    ROS_INFO_STREAM("===================================");
    ROS_INFO_STREAM("Work object frame: " << wobj_frame.translation().transpose());
    const Eigen::Vector3d offset = wobj_frame.translation();


    RedundantRobot robot;
    Rviz rviz;
    rviz.clear();

    //std::vector<std::string> task_names = {"/task1", "/task2", "/task3"};
    std::vector<std::string> task_names;
    node_handle.getParam("/task_names", task_names);
    std::vector<std::vector<TrajectoryPoint>> trajectories;
    std::vector<WeldingTaskSolution> solutions;

    for (auto task_name : task_names)
    {
        trajectories.push_back(readTask(node_handle, task_name, offset));
        for (auto tp : trajectories.back()) tp.plot(rviz.visual_tools_);
        solutions.push_back(planWeld(robot, trajectories.back()));
    }

    // animate cartesian trajectories that where planned
    // for (auto solution : solutions)
    // {
    //     ROS_INFO("Planner %s", solution.success ? "SUCCESS" : "FAILED");
    //     if (solution.success) rviz.animatePath(robot, solution.joint_trajectory);
    // }

    std::vector<std::vector<JointPose>> in_between_sols;
    // plan from home to start
    std::vector<double> zeros = {0, 0, -1.5708, 1.5708, 0, 0, 0};
    auto ptp_sol_1 = planPTP(zeros, solutions[0].joint_trajectory.front(), node_handle);
    in_between_sols.push_back(ptp_sol_1);


    auto ptp_sol_2 = planPTP(
        solutions[0].joint_trajectory.back(),
        solutions[1].joint_trajectory[0],
        node_handle);
    in_between_sols.push_back(ptp_sol_2);

    auto ptp_sol_3 = planPTP(
        solutions[1].joint_trajectory.back(),
        solutions[2].joint_trajectory[0],
        node_handle);
    in_between_sols.push_back(ptp_sol_3);

    // plan to home
    auto ptp_sol_4 = planPTP(solutions[2].joint_trajectory.back(), zeros, node_handle);
    in_between_sols.push_back(ptp_sol_4);

    // for (int i = 1; i < solutions.size(); ++i)
    // {
    //     auto ptp_sol = planPTP(
    //         solutions[i - 1].joint_trajectory.back(),
    //         solutions[i].joint_trajectory.front(),
    //         node_handle
    //     );
    //     if (ptp_sol.size() > 0)
    //     {
    //         in_between_sols.push_back(ptp_sol);
    //     }
    //     else
    //     {
    //         ROS_ERROR_STREAM("Failed to find in between solution");
    //     } 
    // }

    for (int i = 0; i < 3; ++i)
    {
        rviz.animatePath(robot, in_between_sols[i]);
        rviz.animatePath(robot, solutions[i].joint_trajectory);
    }
    rviz.animatePath(robot, in_between_sols.back());
 
    // ======================================================================================

    //printJointTrajectory(solution.joint_trajectory);
    
    // show joint solutions of one trajectory point
    // ROS_INFO_STREAM("Number of js for tp 0: " << solution.tp_data.size());
    // ROS_INFO("Showing the first 20.");
    // for (auto q : solution.tp_data)
    // {
    //     robot.plot(rviz.visual_tools_, q);
    //     ros::Duration(0.1).sleep();
    // }


    // moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    // moveit::planning_interface::MoveGroupInterface::Plan plan1, plan2, plan3;

    // move_group.setJointValueTarget(solutions[0].start_joint_pose);
    // bool success1 = (move_group.plan(plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Go to approach pose %s", success1 ? "SUCCESS" : "FAILED");


    // rviz.clear();
    // std::vector<Eigen::Affine3d> final_ee_path;
    // for (auto& q_sol : solution.joint_trajectory)
    // {
    //     final_ee_path.push_back(robot.fk(q_sol));
    // }

    // for (auto& pose : final_ee_path)
    // {
    //     rviz.plotPose(pose);
    // }

    // addPlans(plan1, solutions[0].motion_plan);
    //ROS_INFO_STREAM("Sol plan: " << solution.motion_plan.trajectory_);
    //ROS_INFO_STREAM("plan plan: " << plan1.trajectory_);

    // move_group.execute(plan1);
    // ros::Duration(1.0).sleep();
    // rviz.animatePath(robot, solutions[0].joint_trajectory);

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

    //move_group.setNamedTarget("home");
    //move_group.move();
    //ros::Duration(1.0).sleep();

    ros::shutdown();

    return 0;
}
