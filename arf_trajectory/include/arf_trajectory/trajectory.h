#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#include <Eigen/Dense>
#include <vector>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "arf_sampling/sampling.h"

struct Number
{
  double nominal_;
  double lower_bound_, upper_bound_;
  int num_samples_;

  Number(double n = 0.0)
    : nominal_(n), lower_bound_(n), upper_bound_(n), num_samples_(1) {}

  operator double() const
  {
    return nominal_;
  }

  protected:
  Number(double n, double l, double u, int s)
    : nominal_(n), lower_bound_(l), upper_bound_(u), num_samples_(s) {}
};

struct TolerancedNumber : public Number
{
  TolerancedNumber(double n, double l, double u, int s = 10)
    : Number(n, l, u, s)
  {
    checkInput();
  }

  private:
  void checkInput();
};

class TrajectoryPoint
{
  std::array<Number*, 6> raw_numbers_; /* x, y, z, rx, ry, rz */
  Eigen::Affine3d nominal_pose_;
  Sampler sampler_;

  Eigen::Affine3d valuesToPose(std::vector<double>& values);

  public:
  TrajectoryPoint(Number& x, Number& y, Number& z, Number& rx, Number& ry, Number& rz);
  ~TrajectoryPoint() = default;

  std::vector<Eigen::Affine3d> getGridSamples();
  std::vector<Eigen::Affine3d> getPoses();
  void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt);
};

#endif