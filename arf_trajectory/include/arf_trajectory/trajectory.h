#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#include <Eigen/Dense>
#include <vector>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "arf_sampling/sampling.h"

class Number
{
protected:
  double nominal_;

public:
  Number(double n = 0.0) : nominal_(n) {}
  ~Number() = default;

  virtual bool hasTolerance()
  {
    return false;
  }
  virtual double getNominal() { return nominal_; }
  virtual int getNumSamples() { return 1; }
  virtual double getLowerBound() { return nominal_; }
  virtual double getUpperBound() { return nominal_; }

  operator double() const
  {
    return nominal_;
  }
};

class TolerancedNumber : public Number
{
  double upper_bound_, lower_bound_;
  int num_samples_;

  void checkInput();

public:
  TolerancedNumber(double n, double l, double u, int s = 10)
    : Number(n), lower_bound_(l), upper_bound_(u), num_samples_(s)
  {
    checkInput();
  }
  ~TolerancedNumber() = default;

  bool hasTolerance() { return true; }
  int getNumSamples() { return num_samples_; }
  virtual double getLowerBound() { return lower_bound_; }
  virtual double getUpperBound() { return upper_bound_; }
};

class TrajectoryPoint
{
  std::array<Number*, 6> raw_numbers_; /* x, y, z, rx, ry, rz */
  Eigen::Affine3d nominal_pose_;
  Sampler sampler_;

public:
  TrajectoryPoint(Number& x, Number& y, Number& z, Number& rx, Number& ry, Number& rz);
  ~TrajectoryPoint() = default;

  std::vector<std::vector<double>> getGridSamples();
  std::vector<Eigen::Affine3d> getPoses();
  void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt);
};

#endif