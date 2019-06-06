#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "arf_sampling/sampling.h"

using Transform = Eigen::Isometry3d;
using Quaternion = Eigen::Quaterniond;

struct Number
{
  double nominal_;
  double lower_bound_, upper_bound_;
  int num_samples_;

  Number(double n = 0.0) : nominal_(n), lower_bound_(n), upper_bound_(n), num_samples_(1)
  {
  }

  operator double() const
  {
    return nominal_;
  }

protected:
  Number(double n, double l, double u, int s) : nominal_(n), lower_bound_(l), upper_bound_(u), num_samples_(s)
  {
  }
};

struct TolerancedNumber : public Number
{
  TolerancedNumber(double n, double l, double u, int s = 10) : Number(n, l, u, s)
  {
    checkInput();
  }

private:
  void checkInput();
};

// class TrajectoryPointBase
// {
//   std::array<Number*, 6> raw_numbers_; /* x, y, z, rx, ry, rz */
//   Transform nominal_pose_;
//   Sampler sampler_;
//   double time_from_previous_point_;

//   Transform valuesToPose(const std::vector<double>& values);

// public:
//   TrajectoryPoint(Number& x, Number& y, Number& z, Number& rx, Number& ry, Number& rz, double timing = 0.1);
//   TrajectoryPoint(std::vector<double> joint_pose) : is_joint_pose_specified_(true), joint_pose_(joint_pose)
//   {
//   }
//   ~TrajectoryPoint() = default;

//   bool is_joint_pose_specified_ = false;
//   std::vector<double> joint_pose_;
//   virtual std::vector<Transform> sample() = 0;
//   Transform getNominalPose()
//   {
//     return nominal_pose_;
//   }
//   void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt);
// };

class TrajectoryPoint
{
  std::array<Number*, 6> raw_numbers_; /* x, y, z, rx, ry, rz */
  Transform nominal_pose_;
  Sampler sampler_;
  double time_from_previous_point_;

  Transform valuesToPose(std::vector<double>& values);

public:
  TrajectoryPoint(Number& x, Number& y, Number& z, Number& rx, Number& ry, Number& rz, double timing = 0.1);
  TrajectoryPoint(std::vector<double> joint_pose) : is_joint_pose_specified_(true), joint_pose_(joint_pose)
  {
  }
  ~TrajectoryPoint() = default;

  bool is_joint_pose_specified_ = false;
  std::vector<double> joint_pose_;
  std::vector<Transform> getGridSamples();
  std::vector<Transform> getPoses();
  Transform getNominalPose()
  {
    return nominal_pose_;
  }
  void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt);
};

class FreeOrientationPoint
{
  std::array<Number*, 3> raw_numbers_; /* x, y, z */

  Sampler sampler_;
  double time_from_previous_point_;

  Transform valuesToPose(std::vector<double>& values);

public:
  FreeOrientationPoint(Number& x, Number& y, Number& z, Number& rx, Number& ry, Number& rz, double timing = 0.1);
  ~FreeOrientationPoint() = default;

  std::vector<Transform> sampleUniform(int n = 100);
  std::vector<Transform> sampleUniformNear(double dist, int n = 100);
  Transform getNominalPose()
  {
    return nominal_pose_;
  }
  void setNominalPose(Transform& new_pose)
  {
    nominal_pose_ = new_pose;
  }
  void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt);

  Transform nominal_pose_;
};

#endif