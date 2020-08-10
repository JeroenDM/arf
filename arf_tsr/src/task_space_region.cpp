#include "arf_tsr/task_space_region.h"

#include <vector>
#include <Eigen/Geometry>
#include <memory>

#include <arf_sampling/sampler.h>

namespace arf
{
Eigen::Vector3d minNormEquivalent(const Eigen::Vector3d& angles)
{
  Eigen::Matrix<double, 9, 3> m;
  double x(angles.x()), y(angles.y()), z(angles.z());
  // clang-format off
   m <<  x,  y, z,
        x - M_PI, -y - M_PI, z - M_PI,
        x - M_PI, -y - M_PI, z + M_PI,
        x - M_PI, -y + M_PI, z - M_PI,
        x - M_PI, -y + M_PI, z + M_PI,
        x + M_PI, -y - M_PI, z - M_PI,
        x + M_PI, -y - M_PI, z + M_PI,
        x + M_PI, -y + M_PI, z - M_PI,
        x + M_PI, -y + M_PI, z + M_PI;
  // clang-format on
  // get the index of the row with the lowest norm
  Eigen::VectorXd::Index index;
  m.rowwise().norm().minCoeff(&index);
  return m.row(index);
}

Eigen::Vector6d poseDistance(const Transform& tf_ref, const Transform& tf)
{
  const Transform tf_diff = tf_ref.inverse() * tf;
  Eigen::Vector3d angles = minNormEquivalent(tf_diff.rotation().eulerAngles(0, 1, 2));
  Eigen::Vector6d d;
  d << tf_diff.translation(), angles;
  return d;
}

/*********************
 * Task Space Region
 * *******************/
TSR::TSR(Transform tf, std::array<Bound, 6> bounds, arf::SamplerPtr sampler, const std::vector<int>& num_samples)
  : tf_nominal_(tf), bounds_(bounds), sampler_(sampler)
{
  sampler_->addDimension(bounds_[0].lower, bounds_[0].upper, num_samples[0]);
  sampler_->addDimension(bounds_[1].lower, bounds_[1].upper, num_samples[1]);
  sampler_->addDimension(bounds_[2].lower, bounds_[2].upper, num_samples[2]);
  sampler_->addDimension(bounds_[3].lower, bounds_[3].upper, num_samples[3]);
  sampler_->addDimension(bounds_[4].lower, bounds_[4].upper, num_samples[4]);
  sampler_->addDimension(bounds_[5].lower, bounds_[5].upper, num_samples[5]);
}

std::vector<Transform> TSR::getSamples(const int n) const
{
  std::vector<Transform> samples;
  auto tsr_samples = sampler_->getSamples(n);
  for (auto& tsr_sample : tsr_samples)
  {
    Eigen::Vector6d v(tsr_sample.data());
    samples.push_back(valuesToPose(v));
  }
  return samples;
}

Transform TSR::valuesToPose(const Eigen::Vector6d& values) const
{
  using Translation = Eigen::Translation3d;
  using AngleAxis = Eigen::AngleAxisd;
  using Vector = Eigen::Vector3d;

  // clang-format off
    Transform t;
      t = Translation(values[0], values[1], values[2]) *
          AngleAxis(values[3], Vector::UnitX()) *
          AngleAxis(values[4], Vector::UnitY()) *
          AngleAxis(values[5], Vector::UnitZ());
  // clang-format on

  // the values are expressed in the nominal tsr frame,
  // convert them to the world frame
  return tf_nominal_ * t;
}

Eigen::Vector6d TSR::poseToValues(const Transform& tf) const
{
  // values are calculated in nominal frame of this tsr
  Eigen::Isometry3d tf_diff = tf_nominal_.inverse() * tf;
  // Eigen documentation:
  //    The returned angles are in the ranges [0:pi]x[-pi:pi]x[-pi:pi].
  Eigen::Vector3d angles = tf_diff.rotation().eulerAngles(0, 1, 2);

  Eigen::Vector6d values;
  values << tf_diff.translation(), angles;
  return values;
}

Eigen::Vector6d TSR::applyBounds(const Eigen::Vector6d& values) const
{
  Eigen::Vector6d distance;
  for (std::size_t i{ 0 }; i < 6; ++i)
  {
    distance[i] = bounds_[i].distance(values[i]);
  }
  return distance;
}

}  // namespace arf