#include <ros/ros.h>

#include <arf_moveit_wrapper/moveit_wrapper.h>
#include <arf_tsr/task_space_region.h>
#include <arf_sampling/sampling.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
namespace rvt = rviz_visual_tools;

// #include "util.h"

constexpr double BOX_SIZE{ 0.5 };

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
                                                   const std::size_t min_samples = 20)
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
      ROS_INFO_STREAM("Found " << num_samples << " in " << iter << " iterations.");
      break;
    }
  }

  samples.shrink_to_fit();
  return samples;
}

std::vector<std::vector<double>> ikSampling(const arf::RobotMoveitWrapperPtr& robot, const arf::TSR& tsr,
                                            const std::size_t min_samples = 20)
{
  std::vector<std::vector<double>> samples;
  samples.reserve(min_samples + 8);

  auto tsamples = tsr.getSamples();
  ROS_INFO_STREAM("Number of samples: " << tsamples.size());

  std::size_t num_samples{ 0 };
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
  samples.shrink_to_fit();
  return samples;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_table");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  arf::RobotMoveitWrapperPtr robot = std::make_shared<arf::Robot>();
  Rviz rviz;
  rviz.clear();

  std::vector<double> home = { 0, -1.5, 1.5, 0, 0, 0 };
  robot->plot(rviz.visual_tools_, home);

  arf::Transform tf_nominal(robot->fk(home));
  rviz.plotPose(tf_nominal);

  arf::TSRBounds bounds{
    { { -BOX_SIZE, BOX_SIZE }, { -BOX_SIZE, BOX_SIZE }, { -BOX_SIZE, BOX_SIZE }, { 0, 0 }, { 0, 0 }, { 0, 0 } }
  };
  std::vector<int> ns{ 3, 3, 3, 1, 1, 1 };
  arf::TSR tsr(tf_nominal, bounds, std::make_shared<arf::Sampler>(), ns);

  arf::TSRBounds bounds_2{ { { -BOX_SIZE, BOX_SIZE },
                             { -BOX_SIZE, BOX_SIZE },
                             { -BOX_SIZE, BOX_SIZE },
                             { -M_PI, M_PI },
                             { -M_PI, M_PI },
                             { -M_PI, M_PI } } };
  std::vector<int> ns_2{ 3, 3, 3, 2, 2, 2 };

  arf::TSR tsr_2(tf_nominal, bounds_2, std::make_shared<arf::Sampler>(), ns_2);

  // auto tsamples = tsr.getSamples();
  // ROS_INFO_STREAM("Number of samples: " << tsamples.size());
  // for (auto& pose : tsamples)
  // {
  //   rviz.plotPose(pose);
  //   ROS_INFO_STREAM("Distance to centre: " << tsr.distanceVector(pose).transpose());
  //   ROS_INFO_STREAM("Distance to TSR: " << tsr.distance(pose));
  //   // auto iksols = robot.ik(pose);
  //   // ROS_INFO_STREAM("Ik solutions: " << iksols.size());
  //   // for (auto& iksol : iksols)
  //   // {
  //   //   robot.plot(rviz.visual_tools_, iksol);
  //   //   ros::Duration(0.1).sleep();
  //   // }
  // }

  // const std::size_t max_iters{ 500 };
  // const std::size_t min_samples{ 20 };
  // std::size_t num_samples{ 0 };
  // for (std::size_t iter{ 0 }; iter < max_iters; ++iter)
  // {
  //   auto jp = robot.getRandomPosition();
  //   auto tf = robot.fk(jp);
  //   double d = tsr_2.distance(tf);
  //   // ROS_INFO_STREAM("Distance: " << d << "---------------------------------");

  //   // ROS_INFO_STREAM("Angles b: " << tsr_2.poseToValues(tf).transpose());
  //   // ROS_INFO_STREAM("Angles a: " << tsr_2.applyBounds(tsr_2.poseToValues(tf)).transpose());

  //   if (d < 0.001)
  //   {
  //     rviz.plotPose(tf);
  //     robot.plot(rviz.visual_tools_, jp);
  //     ros::Duration(0.1).sleep();
  //     num_samples++;
  //   }

  //   if (num_samples >= min_samples)
  //   {
  //     ROS_INFO_STREAM("Found " << num_samples << " in " << iter << " iterations.");
  //     break;
  //   }
  // }

  std::size_t num_samples = 100;

  auto samples1 = ikSampling(robot, tsr_2, 200);
  ROS_INFO_STREAM("Found " << samples1.size() << " samples");
  for (auto& jp : samples1)
  {
    robot->plot(rviz.visual_tools_, jp);
    ros::Duration(0.05).sleep();
    rviz.plotPose(robot->fk(jp));
  }

  ros::Duration(1.5).sleep();
  rviz.clear();
  ros::Duration(0.5).sleep();

  robot->plot(rviz.visual_tools_, home);

  auto samples = rejectionSampling(robot, tsr_2, num_samples);
  ROS_INFO_STREAM("Found " << samples.size() << " samples");
  for (auto& jp : samples)
  {
    robot->plot(rviz.visual_tools_, jp);
    ros::Duration(0.05).sleep();
    rviz.plotPose(robot->fk(jp));
  }

  robot->plot(rviz.visual_tools_, home);

  ros::Duration(0.1).sleep();

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