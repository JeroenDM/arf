#include "arf_tsr/task_space_region.h"

#include <vector>
#include <Eigen/Geometry>
#include <memory>

#include <arf_sampling/sampling.h>

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

TSR::TSR(Transform tf, TSRBounds bounds, arf::SamplerPtr sampler, const std::vector<int>& num_samples)
  : tf_nominal_(tf), bounds_(bounds), sampler_(sampler)
{
  sampler_->addDimension(bounds.x.lower, bounds.x.upper, num_samples[0]);
  sampler_->addDimension(bounds.y.lower, bounds.y.upper, num_samples[1]);
  sampler_->addDimension(bounds.z.lower, bounds.z.upper, num_samples[2]);
  sampler_->addDimension(bounds.rx.lower, bounds.rx.upper, num_samples[3]);
  sampler_->addDimension(bounds.ry.lower, bounds.ry.upper, num_samples[4]);
  sampler_->addDimension(bounds.rz.lower, bounds.rz.upper, num_samples[5]);
}

std::vector<Transform> TSR::getSamples(const int n)
{
  std::vector<Transform> samples;
  auto tsr_samples = sampler_->getGridSamples();
  for (auto& tsr_sample : tsr_samples)
  {
    samples.push_back(valuesToPose(tsr_sample));
  }
  return samples;
}

Transform TSR::valuesToPose(std::vector<double>& values)
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
  return tf_nominal_ * t;
}

}  // namespace arf