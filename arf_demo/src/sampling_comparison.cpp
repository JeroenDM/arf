#include <ros/ros.h>

#include <chrono>

#include <arf_moveit_wrapper/moveit_wrapper.h>
#include <arf_tsr/task_space_region.h>
#include <arf_sampling/random_sampler.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
namespace rvt = rviz_visual_tools;

// #include "util.h"

class Rviz
{
public:
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  Rviz()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link", "/visualization_marker_array"));
    visual_tools_->loadMarkerPub(true);
    visual_tools_->loadRobotStatePub("/display_robot_state", true);
  }

  void plotPose(Eigen::Isometry3d pose);
  void clear();
};

std::vector<std::vector<double>> rejectionSampling(const arf::RobotMoveitWrapperPtr& robot, const arf::TSR& tsr,
                                                   const std::size_t min_samples = 20);

std::vector<std::vector<double>> ikSampling(const arf::RobotMoveitWrapperPtr& robot, const arf::TSR& tsr,
                                            const std::size_t min_samples = 20);

void run_experiment(const arf::RobotMoveitWrapperPtr& robot, const arf::TSR& tsr, const int min_number_samples)
{
  double time_ik{ 0.0 };
  double time_rej{ 0.0 };
  std::size_t num_s_ik{ 0 };
  std::size_t num_s_rej{ 0 };
  {
    // std::chrono::duration<double> time;
    auto start = std::chrono::high_resolution_clock::now();
    auto samples1 = ikSampling(robot, tsr, min_number_samples);
    auto stop = std::chrono::high_resolution_clock::now();
    time_ik = std::chrono::duration<double>(stop - start).count();
    num_s_ik = samples1.size();
  }

  {
    // std::chrono::duration<double> time;
    auto start = std::chrono::high_resolution_clock::now();
    auto samples = rejectionSampling(robot, tsr, min_number_samples);
    auto stop = std::chrono::high_resolution_clock::now();
    time_rej = std::chrono::duration<double>(stop - start).count();
    num_s_rej = samples.size();
  }
  std::cout << tsr.bounds_[0].range() << "," << tsr.volume() << ",";
  std::cout << num_s_ik << "," << num_s_rej << ",";
  std::cout << time_ik << "," << time_rej << "\n";
}

const std::string HEADER{ "box_size,tsr_volume,num_s_ik,num_s_rej,time_ik,time_rej" };

// bool project(const arf::RobotMoveitWrapper& robot, Eigen::VectorXd& q, double tolerance = 1e-6, int max_iterations =
// 50)
// {
//   // Newton's method
//   unsigned int iter = 0;
//   double norm = 0;
//   Eigen::VectorXd f(robot.getNumDof());
//   Eigen::MatrixXd;

//   const double squaredTolerance = tolerance * tolerance;

//   return norm < squaredTolerance;
// }

std::vector<std::vector<double>> projectionSampling(const arf::RobotMoveitWrapperPtr& robot, const arf::TSR& tsr,
                                                    const std::size_t min_samples = 20)
{
  const std::size_t max_iters{ 20000 };

  std::vector<std::vector<double>> samples;
  samples.reserve(min_samples + 10);  // could be more because of IK solutions

  std::size_t num_samples{ 0 };
  for (std::size_t iter{ 0 }; iter < max_iters; ++iter)
  {
    Eigen::VectorXd joint_values(robot->getNumDof());
    robot->getRandomPosition(joint_values);

    Eigen::Isometry3d tf;
    Eigen::Vector6d d;
    Eigen::MatrixXd jac(6, robot->getNumDof());
    const int max_iters_2{ 0 };
    int iters_2{ 0 };
    const double tolerance{ 1e-3 };

    tf = robot->fk(joint_values);
    d = tsr.distanceVector(tf);
    while (d.squaredNorm() < tolerance && iters_2++ < max_iters_2)
    {
      jac = robot->jac(joint_values);
      joint_values -= jac.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(d);
      tf = robot->fk(joint_values);
      d = tsr.distanceVector(tf);
    }

    num_samples++;
    if (num_samples >= min_samples)
    {
      break;
    }
  }
  samples.shrink_to_fit();
  return samples;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_table");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // double BOX_SIZE{ 0.2 };

  arf::RobotMoveitWrapperPtr robot = std::make_shared<arf::Robot>();
  Rviz rviz;
  rviz.clear();

  std::vector<double> home = { 0, -1.5, 1.5, 0, 0, 0 };
  robot->plot(rviz.visual_tools_, home);

  arf::Transform tf_nominal(robot->fk(home));
  rviz.plotPose(tf_nominal);

  // arf::TSRBounds bounds{
  //   { { -BOX_SIZE, BOX_SIZE }, { -BOX_SIZE, BOX_SIZE }, { -BOX_SIZE, BOX_SIZE }, { 0, 0 }, { 0, 0 }, { 0, 0 } }
  // };
  // std::vector<int> ns{ 3, 3, 3, 1, 1, 1 };
  // arf::TSR tsr(tf_nominal, bounds, std::make_shared<arf::RandomSampler>(), ns);

  std::cout << HEADER << std::endl;
  double sbox{ 1.0 };
  // const int num_runs{ 1 };
  // for (double sbox{ 0.2 }; sbox < 2.0; sbox += 0.22)
  // {
  // ROS_INFO_STREAM("-------------------------------------");
  // ROS_INFO_STREAM("BOX_SIZE: " << sbox);

  arf::TSRBounds bounds_2{ { { -sbox / 2, sbox / 2 },
                             { -sbox / 2, sbox / 2 },
                             { -sbox / 2, sbox / 2 },
                             { -M_PI, M_PI },
                             { -M_PI, M_PI },
                             { -M_PI, M_PI } } };
  std::vector<int> ns_2{ 3, 3, 3, 2, 2, 2 };

  arf::TSR tsr_2(tf_nominal, bounds_2, std::make_shared<arf::RandomSampler>(), ns_2);

  //   for (int run{ 0 }; run < num_runs; ++run)
  //   {
  //     run_experiment(robot, tsr_2, 50);
  //   }
  // }

  // std::size_t num_samples = 100;

  // auto samples1 = ikSampling(robot, tsr_2, 200);
  // ROS_INFO_STREAM("Found " << samples1.size() << " samples");
  // for (auto& jp : samples1)
  // {
  //   robot->plot(rviz.visual_tools_, jp);
  //   ros::Duration(0.05).sleep();
  //   rviz.plotPose(robot->fk(jp));
  // }

  std::size_t num_samples = 10;

  auto samples1 = projectionSampling(robot, tsr_2, num_samples);
  ROS_INFO_STREAM("Found " << samples1.size() << " samples");
  for (auto& jp : samples1)
  {
    robot->plot(rviz.visual_tools_, jp);
    ros::Duration(0.05).sleep();
    rviz.plotPose(robot->fk(jp));
  }

  // ros::Duration(1.5).sleep();
  // rviz.clear();
  // ros::Duration(0.5).sleep();

  // robot->plot(rviz.visual_tools_, home);

  // auto samples = rejectionSampling(robot, tsr_2, num_samples);
  // ROS_INFO_STREAM("Found " << samples.size() << " samples");
  // for (auto& jp : samples)
  // {
  //   robot->plot(rviz.visual_tools_, jp);
  //   ros::Duration(0.05).sleep();
  //   rviz.plotPose(robot->fk(jp));
  // }

  // robot->plot(rviz.visual_tools_, home);

  // ros::Duration(0.1).sleep();

  ros::shutdown();

  return 0;
}

void Rviz::plotPose(Eigen::Isometry3d pose)
{
  visual_tools_->publishAxis(pose, rvt::LARGE);
  visual_tools_->trigger();
}

void Rviz::clear()
{
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
}

std::vector<std::vector<double>> rejectionSampling(const arf::RobotMoveitWrapperPtr& robot, const arf::TSR& tsr,
                                                   const std::size_t min_samples)
{
  const std::size_t max_iters{ 2000 };

  std::vector<std::vector<double>> samples;
  samples.reserve(min_samples);

  std::size_t num_samples{ 0 };
  for (std::size_t iter{ 0 }; iter < max_iters; ++iter)
  {
    auto jp = robot->getRandomPosition();
    auto tf = robot->fk(jp);
    double d = tsr.distance(tf);

    if (d < 0.001)
    {
      samples.push_back(jp);
      num_samples++;
    }

    if (num_samples >= min_samples)
    {
      // ROS_INFO_STREAM("Rejection sampler found " << num_samples << " in " << iter << " iterations.");
      break;
    }
  }

  if (num_samples < min_samples)
  {
    // ROS_INFO_STREAM("Rejection sampler only found " << num_samples << " in " << max_iters << " iterations.");
  }

  samples.shrink_to_fit();
  return samples;
}

std::vector<std::vector<double>> ikSampling(const arf::RobotMoveitWrapperPtr& robot, const arf::TSR& tsr,
                                            const std::size_t min_samples)
{
  const std::size_t max_iters{ 2000 };

  std::vector<std::vector<double>> samples;
  samples.reserve(min_samples + 10);  // could be more because of IK solutions

  std::size_t num_samples{ 0 };
  for (std::size_t iter{ 0 }; iter < max_iters; ++iter)
  {
    auto tsamples = tsr.getSamples();
    for (auto& pose : tsamples)
    {
      auto iksols = robot->ik(pose);
      for (auto& js : iksols)
      {
        samples.push_back(js);
      }
      num_samples += iksols.size();
      if (num_samples >= min_samples)
      {
        break;
      }
    }

    if (num_samples >= min_samples)
    {
      // ROS_INFO_STREAM("IK sampler found " << num_samples << " in " << iter << " iterations.");
      break;
    }
  }
  samples.shrink_to_fit();
  return samples;
}
