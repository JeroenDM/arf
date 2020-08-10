#include "arf_sampling/random_sampler.h"

namespace arf
{
std::vector<std::vector<double>> RandomSampler::getSamples(const int n)
{
  std::vector<std::vector<double>> samples(n);
  for (int i = 0; i < n; ++i)
  {
    for (int dim = 0; dim < dimensions_; ++dim)
    {
      double u = randu_(random_engine_);
      samples[i].push_back(scale(u, dim));
    }
  }

  return samples;
}
}  // namespace arf