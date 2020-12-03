#ifndef _GRID_SAMPLING_H_
#define _GRID_SAMPLING_H_

#include <vector>

#include "arf_sampling/sampler.h"

namespace arf
{
typedef unsigned int uint;

/**
 * Convert a decimal number to a number into a mixed radix number,
 * with variable base, specified in the bases vector.
 * **/
std::vector<uint> convertBase(uint n, std::vector<uint>& bases)
{
  std::vector<uint> digits(bases.size());
  for (std::size_t i{ bases.size() - 1 }; i >= 0; --i)
  {
    digits[i] = n % bases[i];
    n /= bases[i];
  }
  return digits;
}

class GridSampler : public Sampler
{
  std::vector<int> num_samples_;

  void recursiveGridSampling(int index, std::vector<double> prev_values, std::vector<std::vector<double>>& grid);

public:
  GridSampler() = default;
  ~GridSampler() = default;
  void addDimension(double lower_bound, double upper_bound) override;
  void addDimension(double lower_bound, double upper_bound, int num_samples) override;
  std::vector<std::vector<double>> getSamples(const int n = 1) override;

  static std::vector<double> range(double lower_bound, double upper_bound, int num_samples);
};
}  // namespace arf

#endif
