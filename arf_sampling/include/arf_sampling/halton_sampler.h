#ifndef _HALTON_SAMPLING_H_
#define _HALTON_SAMPLING_H_

#include "arf_sampling/sampler.h"

#include <vector>
namespace arf
{
class HaltonSampler : public Sampler
{
  std::vector<int> primes_;
  int vdc_count_{ 1 };

public:
  HaltonSampler() = default;
  ~HaltonSampler() = default;
  void addDimension(double lower_bound, double upper_bound) override;
  std::vector<std::vector<double>> getSamples(const int n = 1) override;
};
}  // namespace arf

#endif