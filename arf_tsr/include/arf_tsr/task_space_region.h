#ifndef _ARF_TASK_SPACE_REGIONS_
#define _ARF_TASK_SPACE_REGIONS_

#include <vector>
#include <Eigen/Geometry>
#include <memory>
#include <numeric>
#include <algorithm>

#include <arf_sampling/sampling.h>

namespace Eigen
{
typedef Matrix<double, 6, 1> Vector6d;
}

namespace arf
{
typedef Eigen::Isometry3d Transform;

// namespace Eigen{
//   typedef Matrix<double, 6, 1> Vector6d;
// }

Eigen::Vector3d minNormEquivalent(const Eigen::Vector3d& angles);
Eigen::Vector6d poseDistance(const Transform& tf_ref, const Transform& tf);

struct Bound
{
  double lower;
  double upper;

  double distance(double value) const
  {
    if (value < lower)
      return lower - value;
    else if (value > upper)
      return value - upper;
    else
      return 0.0;
  }

  double range() const
  {
    assert(lower <= upper);
    return upper - lower;
  }
};

typedef std::array<Bound, 6> TSRBounds;
// idea that would be nice to implement below
// struct TSRBounds
// {
//   std::array<Bound, 6> bounds_;

//   TSRBounds() = delete;
//   TSRBounds(const std::array<Bound, 6> bounds) : bounds_(bounds)
//   {
//   }

//   arf::Bound& x()
//   {
//     return bounds_[0];
//   }
//   arf::Bound& y()
//   {
//     return bounds_[1];
//   }
//   arf::Bound& z()
//   {
//     return bounds_[2];
//   }
//   arf::Bound& rx()
//   {
//     return bounds_[3];
//   }
//   arf::Bound& ry()
//   {
//     return bounds_[4];
//   }
//   arf::Bound& rz()
//   {
//     return bounds_[5];
//   }
// };

class TSR
{
  Eigen::Isometry3d tf_nominal_;
  std::array<Bound, 6> bounds_;
  arf::SamplerPtr sampler_;

public:
  TSR(Transform tf, std::array<Bound, 6> bounds, arf::SamplerPtr sampler, const std::vector<int>& num_samples);
  ~TSR() = default;

  /** \brief sample poses from within the task space regions.
   *
   * This only implements grid sampling at the moment, so `n` is ignored.
   * */
  std::vector<Transform> getSamples(const int n = 1) const;

  /** \brief Turn a six vector of position and euler angle values into an end-effector pose.
   *
   * This 'values' vector expresses the local diviation from the nominal task space region frame.
   * Therefore it is pre-multiplied with this nominal pose after conversion, before returning it.
   * */
  Transform valuesToPose(const Eigen::Vector6d& values) const;

  /** \brief Express a pose in the nominal TSR frame and convert it to a six vector of position and euler angles.
   *
   * The reverse of `valuesToPose`.
   * */
  Eigen::Vector6d poseToValues(const Transform& tf) const;

  Eigen::Vector6d applyBounds(const Eigen::Vector6d& distance) const;

  Eigen::Vector6d distanceVector(const Transform& tf) const
  {
    return applyBounds(poseToValues(tf));
  }

  double distance(const Transform& tf) const
  {
    return distanceVector(tf).norm();
  }

  /** \brief Specific volume metric from the paper
   *
   * Position in meter and angles in randians are weighted equally by default.
   * */
  double volume(double angle_weight = 1.0)
  {
    double volume;
    // position part
    volume += bounds_[0].range();
    volume += bounds_[1].range();
    volume += bounds_[2].range();
    // angular part
    volume += angle_weight * bounds_[3].range();
    volume += angle_weight * bounds_[4].range();
    volume += angle_weight * bounds_[5].range();
    return volume;
  }
};
}  // namespace arf

#endif  // _ARF_TASK_SPACE_REGIONS_