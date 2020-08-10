#ifndef _ARF_TASK_SPACE_REGIONS_
#define _ARF_TASK_SPACE_REGIONS_

#include <vector>
#include <Eigen/Geometry>
#include <memory>

#include <arf_sampling/sampling.h>

namespace arf
{
typedef Eigen::Isometry3d Transform;

Eigen::Vector3d minNormEquivalent(const Eigen::Vector3d& angles);

struct Bounds
{
  double lower;
  double upper;
};

struct TSRBounds
{
  Bounds x, y, z;
  Bounds rx, ry, rz;
};

class TSR
{
  Eigen::Isometry3d tf_nominal_;
  TSRBounds bounds_;
  arf::SamplerPtr sampler_;

public:
  TSR(Transform tf, TSRBounds bounds, arf::SamplerPtr sampler, const std::vector<int>& num_samples);
  ~TSR() = default;
  std::vector<Transform> getSamples(const int n = 1);
  Transform valuesToPose(std::vector<double>& values);
};
}  // namespace arf

#endif  // _ARF_TASK_SPACE_REGIONS_