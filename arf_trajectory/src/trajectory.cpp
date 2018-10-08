#include <iostream>
#include <stdexcept>
#include "arf_trajectory/trajectory.h"

void TolerancedNumber::checkInput()
{
  if (lower_bound_ >= upper_bound_)
  {
    throw std::invalid_argument("Lower bound should be strictly smaller than upper bound.");
  }
  if (nominal_ < lower_bound_ || nominal_ > upper_bound_)
  {
    throw std::invalid_argument("Nominal value should be withing bounds.");
  }
}

TrajectoryPoint::TrajectoryPoint(Number& x, Number& y, Number& z, Number& rx, Number& ry, Number& rz, double timing)
{
  using namespace Eigen;
  raw_numbers_[0] = &x;
  raw_numbers_[1] = &y;
  raw_numbers_[2] = &z;
  raw_numbers_[3] = &rx;
  raw_numbers_[4] = &ry;
  raw_numbers_[5] = &rz;
  std::vector<double> nominal_values = {x, y, z, rx, ry, rz};
  nominal_pose_ = valuesToPose(nominal_values);
  
  for (auto n : raw_numbers_)
  {
    sampler_.addDimension(n->lower_bound_, n->upper_bound_, n->num_samples_);
  }
  time_from_previous_point_ = timing;
}

Eigen::Affine3d TrajectoryPoint::valuesToPose(std::vector<double>& values)
{
  using namespace Eigen;
  Affine3d t;
    t = Translation3d(values[0], values[1], values[2]) *
        AngleAxisd(values[3], Vector3d::UnitZ()) *
        AngleAxisd(values[4], Vector3d::UnitY()) *
        AngleAxisd(values[5], Vector3d::UnitZ());
    return t;
}

std::vector<Eigen::Affine3d> TrajectoryPoint::getGridSamples()
{
  std::vector<Eigen::Affine3d> poses;

  for (auto val : sampler_.getGridSamples())
  {
    poses.push_back(valuesToPose(val));
  }
  return poses;
}

void TrajectoryPoint::plot(moveit_visual_tools::MoveItVisualToolsPtr mvt)
{
    namespace rvt = rviz_visual_tools;
    mvt->publishAxis(nominal_pose_, rvt::LARGE);
    mvt->trigger();
}