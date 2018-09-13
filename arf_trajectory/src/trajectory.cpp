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

TrajectoryPoint::TrajectoryPoint(Number& x, Number& y, Number& z, Number& rx, Number& ry, Number& rz)
{
  using namespace Eigen;
  raw_numbers_[0] = &x;
  raw_numbers_[1] = &y;
  raw_numbers_[2] = &z;
  raw_numbers_[3] = &rx;
  raw_numbers_[4] = &ry;
  raw_numbers_[5] = &rz;
  nominal_pose_ = Translation3d(x, y, z) *
        AngleAxisd(rx, Vector3d::UnitZ()) *
        AngleAxisd(ry, Vector3d::UnitY()) *
        AngleAxisd(rz, Vector3d::UnitZ());
  
  for (auto n : raw_numbers_)
  {
    sampler_.addDimension(n->getNumSamples(), n->getLowerBound(), n->getUpperBound());
  }
}

std::vector<std::vector<double>> TrajectoryPoint::getGridSamples()
{
  return sampler_.getGridSamples();
}

std::vector<Eigen::Affine3d> TrajectoryPoint::getPoses()
{
  using namespace Eigen;
  std::vector<Affine3d> poses;

  for (auto val : sampler_.getGridSamples())
  {
    Affine3d t;
    t = Translation3d(val[0], val[1], val[2]) *
        AngleAxisd(val[3], Vector3d::UnitZ()) *
        AngleAxisd(val[4], Vector3d::UnitY()) *
        AngleAxisd(val[5], Vector3d::UnitZ());
    poses.push_back(t);
  }
  return poses;
}

void TrajectoryPoint::plot(moveit_visual_tools::MoveItVisualToolsPtr mvt)
{
    namespace rvt = rviz_visual_tools;
    mvt->publishAxis(nominal_pose_, rvt::LARGE);
    mvt->trigger();
}