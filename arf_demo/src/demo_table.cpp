#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

#include "arf_graph/graph.h"
#include "arf_graph/util.h"
#include "arf_moveit_wrapper/moveit_wrapper.h"
#include "arf_trajectory/trajectory.h"
#include "ros/package.h"

#include "util.h"

std::string FILE_NAME_SINGLE_RUN = "/data/results_case_1_single_run_l1.csv";
std::string FILE_NAME_ITERATIVE = "/data/results_case_1_l1.csv";

class Demo1
{
  std::vector<std::vector<std::vector<double>>> graph_data_;
  std::vector<TrajectoryPoint> ee_trajectory_;
  std::vector<FreeOrientationPoint> ee_trajectory_2_;
  std::vector<std::vector<double>> shortest_path_;

public:
  double last_path_cost_;
  void createTrajectory();
  void createGraphData(Robot& robot, Rviz& rviz);
  void calculateShortestPath(Robot& robot);
  void showShortestPath(Robot& robot, Rviz& rviz);
  void orientationFreeSampling(Robot& robot, int num_samples);
  void sampleNearSolution(Robot& robot, Rviz& rviz, double dist, int n);
  void showTrajectory(Rviz& rviz);
  void readTaskFromYaml(const std::string filename);
  void readTask1(ros::NodeHandle& nh);
  void reset();
};

void Demo1::reset()
{
  graph_data_.clear();
  shortest_path_.clear();
  last_path_cost_ = 0;
}

std::vector<TrajectoryPoint> createPath();

void showResults(std::vector<std::chrono::duration<double>>& times, std::vector<double> costs)
{
  ROS_INFO_STREAM("========================================");
  ROS_INFO_STREAM("Timing results");
  for (std::size_t i = 0; i < times.size(); ++i)
  {
    ROS_INFO_STREAM("time: " << times[i].count() << " cost: " << costs[i]);
  }
}

void run_case(Demo1& demo1, Robot& robot, Rviz& rviz, int num_samples, std::ofstream& file, int run_index)
{
  ROS_INFO_STREAM("Started running planning case 1");
  // parameters
  double reduction_factor = 2.0;
  double initial_quat_dist = 0.25 * M_PI;  // same as free
  int num_iterations = 4;                  // including first run

  std::vector<std::chrono::duration<double>> times;
  std::vector<double> run_time;
  std::vector<double> costs;

  // first run without reduced tolerance
  auto start = std::chrono::high_resolution_clock::now();
  demo1.orientationFreeSampling(robot, num_samples);
  demo1.calculateShortestPath(robot);
  auto stop = std::chrono::high_resolution_clock::now();

  times.push_back(stop - start);
  costs.push_back(demo1.last_path_cost_);

  double dist = initial_quat_dist;
  for (int i = 0; i < num_iterations - 1; ++i)
  {
    dist = dist / reduction_factor;
    ROS_INFO_STREAM("Sampling iteration " << i << " with dist: " << dist);

    start = std::chrono::high_resolution_clock::now();
    demo1.sampleNearSolution(robot, rviz, dist, num_samples);
    demo1.calculateShortestPath(robot);
    stop = std::chrono::high_resolution_clock::now();

    times.push_back(stop - start);
    costs.push_back(demo1.last_path_cost_);

    // demo1.showShortestPath(robot, rviz);
    // rviz.clear();
  }

  // print results
  for (std::size_t i = 0; i < costs.size(); ++i)
  {
    file << run_index << ",";
    file << i << ",";  // iteration
    file << num_samples << ",";
    file << costs[i] << ",";
    file << times[i].count();
    file << std::endl;
  }
}

void run_multiple_cases(Demo1& demo, Robot& robot, Rviz& rviz, std::vector<int> num_samples, int num_runs)
{
  std::string header = "run,iteration,samples,cost,time";
  std::string filename = ros::package::getPath("arf_demo") + FILE_NAME_ITERATIVE;

  std::ofstream data_file;
  data_file.open(filename);

  if (data_file.is_open())
  {
    data_file << header << std::endl;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to open file for data output.");
  }

  for (int k = 0; k < num_runs; ++k)
  {
    for (auto ns : num_samples)
    {
      demo.reset();
      run_case(demo, robot, rviz, ns, data_file, k);
    }
  }

  data_file.close();
}

void single_run(Demo1& demo1, Robot& robot, Rviz& rviz, int num_samples, std::ofstream& file, int run_index)
{
  ROS_INFO_STREAM("Started running single run, " << num_samples << " samples.");

  std::vector<std::chrono::duration<double>> times;
  std::vector<double> run_time;
  std::vector<double> costs;

  // first run without reduced tolerance
  auto start = std::chrono::high_resolution_clock::now();
  demo1.orientationFreeSampling(robot, num_samples);
  demo1.calculateShortestPath(robot);
  auto stop = std::chrono::high_resolution_clock::now();

  times.push_back(stop - start);
  costs.push_back(demo1.last_path_cost_);

  // print results
  for (std::size_t i = 0; i < costs.size(); ++i)
  {
    file << run_index << ",";
    file << num_samples << ",";
    file << costs[i] << ",";
    file << times[i].count();
    file << std::endl;

    std::cout << "Cost: " << costs[i] << " Time: " << times[i].count() << std::endl;
  }
}

void run_multiple_cases_single_run(Demo1& demo, Robot& robot, Rviz& rviz, std::vector<int> num_samples, int num_runs)
{
  std::string header = "run,samples,cost,time";
  std::string filename = ros::package::getPath("arf_demo") + FILE_NAME_SINGLE_RUN;

  std::ofstream data_file;
  data_file.open(filename);

  if (data_file.is_open())
  {
    data_file << header << std::endl;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to open file for data output.");
  }

  for (int k = 0; k < num_runs; ++k)
  {
    for (auto ns : num_samples)
    {
      demo.reset();
      single_run(demo, robot, rviz, ns, data_file, k);
    }
  }

  data_file.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_table");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Robot robot;
  Rviz rviz;
  Demo1 demo1;

  rviz.clear();

  // task orientation free
  std::string filename = ros::package::getPath("arf_demo") + "/config/table_task.csv";
  demo1.readTaskFromYaml(filename);
  demo1.showTrajectory(rviz);

  std::vector<double> home = { 0, -1.5, 1.5, 0, 0, 0 };
  robot.plot(rviz.visual_tools_, home);

  std::vector<int> num_samples = { 300, 600, 900 };
  run_multiple_cases(demo1, robot, rviz, num_samples, 10);

  std::vector<int> num_samples_2 = { 300, 600, 900, 1200, 1800, 2500, 3000, 3600 };
  run_multiple_cases_single_run(demo1, robot, rviz, num_samples_2, 10);

  // std::ofstream ftest;
  // ftest.open("test.csv");
  // single_run(demo1, robot, rviz, 1000, ftest, 0);

  demo1.showShortestPath(robot, rviz);
  rviz.clear();
  demo1.showTrajectory(rviz);

  ros::shutdown();

  return 0;
}

TrajectoryPoint createPointFromParameters(ros::NodeHandle& nh, const std::string absolute_path)
{
  std::vector<double> pose, ld, ud;
  std::vector<int> ns;

  // nh.getParam(absolute_path + "/type", type);
  nh.getParam(absolute_path + "/pose", pose);
  nh.getParam(absolute_path + "/num_samples", ns);
  nh.getParam(absolute_path + "/lower_delta", ld);
  nh.getParam(absolute_path + "/upper_delta", ud);

  std::vector<std::shared_ptr<Number>> values;
  for (int i = 0; i < 6; ++i)
  {
    if (ns[i] == 0)
    {
      values.push_back(std::make_shared<Number>(pose[i]));
    }
    else
    {
      values.push_back(std::make_shared<TolerancedNumber>(pose[i], pose[i] - ld[i], pose[i] + ud[i], ns[i]));
    }
  }

  // TolerancedNumber x(pose[0], pose[0] - ld[0], pose[0] + ud[0], ns[0]);
  // Number x(pose[0]);
  // Number y(pose[1]);
  // Number z(pose[2]);
  // Number rx(pose[3]);
  // TolerancedNumber ry(pose[4], pose[4] - ld[4], pose[4] + ud[4], ns[4]);
  // TolerancedNumber rz(pose[5], pose[5] - ld[5], pose[5] + ud[5], ns[5]);

  // TrajectoryPoint tp(x, y, z, rx, ry, rz);

  TrajectoryPoint tp(*values[0], *values[1], *values[2], *values[3], *values[4], *values[5]);
  return tp;
}

void Demo1::readTask1(ros::NodeHandle& nh)
{
  ee_trajectory_.clear();
  std::vector<double> ud, ld;
  std::string task_name = "path";
  if (nh.hasParam(task_name))
  {
    int num_steps = 0;
    nh.getParam(task_name + "/num_steps", num_steps);
    ROS_INFO_STREAM("Found path of length: " << num_steps);

    for (int i = 0; i < num_steps; ++i)
    {
      ee_trajectory_.push_back(createPointFromParameters(nh, task_name + "/point_" + std::to_string(i)));
    }
  }
  else
  {
    ROS_ERROR_STREAM("Failed to read a path from parameter server");
  }

  // for (int i = 0; i < ud.size(); ++i)
  // {
  //   std::cout << "ud: " << ud[i] << "  ld: " << ld[i] << std::endl;
  // }
  // for (int i = 0; i < 10; ++i)
  // {
  //   Number x(0.8);
  //   Number y(-0.2 + static_cast<double>(i) / 20);
  //   Number z(0.2);
  //   Number rx; //, ry(-M_PI);
  //   TolerancedNumber ry(-M_PI, -M_PI - ld[4], -M_PI + ud[4], 5);
  //   TolerancedNumber rz(0, -ld[5], ud[5], 20);
  //   TrajectoryPoint tp(x, y, z, rx, ry, rz);
  //   ee_trajectory_.push_back(tp);
  // }
}

void Demo1::createTrajectory()
{
  for (int i = 0; i < 10; ++i)
  {
    Number x(0.8);
    Number y(-0.2 + static_cast<double>(i) / 20);
    Number z(0.2);
    Number rx;  //, ry(-M_PI);
    TolerancedNumber ry(-M_PI, -M_PI - 1.0, -M_PI + 1.0, 5);
    TolerancedNumber rz(0, -M_PI, M_PI, 20);
    TrajectoryPoint tp(x, y, z, rx, ry, rz);
    ee_trajectory_.push_back(tp);
  }
}

void Demo1::showTrajectory(Rviz& rviz)
{
  if (ee_trajectory_.size() > 0)
  {
    for (auto tp : ee_trajectory_)
      tp.plot(rviz.visual_tools_);
  }
  else
  {
    for (auto tp : ee_trajectory_2_)
      tp.plot(rviz.visual_tools_);
  }
}

void Demo1::readTaskFromYaml(const std::string filename)
{
  ee_trajectory_2_.clear();

  std::ifstream data_file;
  data_file.open(filename);

  std::string line, number;
  std::vector<double> pt;
  int i = -1;
  while (std::getline(data_file, line))
  {
    if (i < 0)
    {
      // skip first line
      ++i;
      continue;
    }
    std::stringstream line_stream(line);
    pt.clear();
    while (std::getline(line_stream, number, ','))
    {
      pt.push_back(std::stod(number));
    }

    Number x(pt[0]), y(pt[1]), z(pt[2]);
    Number rx(pt[3]), ry(pt[4]), rz(pt[5]);
    FreeOrientationPoint tp(x, y, z, rx, ry, rz);
    ee_trajectory_2_.push_back(tp);
    ++i;
  }
}

void Demo1::createGraphData(Robot& robot, Rviz& rviz)
{
  for (auto tp : ee_trajectory_)
  {
    std::vector<std::vector<double>> new_data;
    for (auto pose : tp.getGridSamples())
    {
      rviz.plotPose(pose);
      for (auto q_sol : robot.ik(pose))
      {
        if (!robot.isInCollision(q_sol))
          new_data.push_back(q_sol);
      }
    }
    graph_data_.push_back(new_data);
  }
}

void Demo1::orientationFreeSampling(Robot& robot, int num_samples = 500)
{
  std::cout << "Trajectory length: " << ee_trajectory_2_.size() << std::endl;
  graph_data_.clear();

  for (auto tp : ee_trajectory_2_)
  {
    std::vector<std::vector<double>> new_data;
    for (auto pose : tp.sampleUniform(num_samples))
    {
      for (auto q_sol : robot.ik(pose))
      {
        if (!robot.isInCollision(q_sol))
          new_data.push_back(q_sol);
      }
    }
    graph_data_.push_back(new_data);
  }
}

void Demo1::sampleNearSolution(Robot& robot, Rviz& rviz, double dist, int n = 500)
{
  if (shortest_path_.size() < 1)
  {
    throw std::invalid_argument("There is no solution to sample around.");
  }

  graph_data_.clear();
  for (std::size_t i = 0; i < shortest_path_.size(); ++i)
  {
    auto fk_sol = robot.fk(shortest_path_[i]);
    Transform nom_pose(fk_sol.matrix());
    ee_trajectory_2_[i].setNominalPose(nom_pose);

    std::vector<std::vector<double>> new_data;
    for (auto pose : ee_trajectory_2_[i].sampleUniformNear(dist, n))
    {
      // rviz.plotPose(pose);
      for (auto q_sol : robot.ik(pose))
      {
        if (!robot.isInCollision(q_sol))
          new_data.push_back(q_sol);
      }
    }
    graph_data_.push_back(new_data);
  }
}

void Demo1::calculateShortestPath(Robot& robot)
{
  Graph demo_graph(graph_data_);
  demo_graph.runMultiSourceDijkstra();
  std::vector<Node*> sp = demo_graph.getShortestPath();
  // std::cout << "Shortest path \n";

  shortest_path_.clear();
  for (auto node : sp)
  {
    // std::cout << (*node) << std::endl;
    shortest_path_.push_back(*(*node).jv);
  }

  last_path_cost_ = demo_graph.last_path_cost;
}

void Demo1::showShortestPath(Robot& robot, Rviz& rviz)
{
  for (auto q : shortest_path_)
  {
    robot.plot(rviz.visual_tools_, q);
    ros::Duration(0.5).sleep();
  }
}