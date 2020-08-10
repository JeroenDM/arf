/** van der Corput and Halton sampling
 * based on
 * https://en.wikipedia.org/wiki/Van_der_Corput_sequence
 * */

#include "arf_sampling/halton_sampler.h"

#include <array>
#include <vector>
#include <stdexcept>

namespace arf
{
double vdc(int n, int base)
{
  double q{};
  double bk{ 1.0 / base };

  while (n > 0)
  {
    q += (n % base) * bk;
    n /= base;
    bk /= base;
  }
  return q;
}

int next_prime()
{
  static std::size_t index{ 0 };
  static const std::size_t NUM_PRIMES{ 48 };
  static const std::array<int, NUM_PRIMES> primes{ 41,  43,  47,  53,  59,  61,  67,  71,  73,  79,  83,  89,
                                                   97,  101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151,
                                                   157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223,
                                                   227, 229, 233, 239, 241, 251, 257, 263, 269, 271, 277, 281 };

  if (index >= NUM_PRIMES)
  {
    throw std::range_error("Maximum number of dimensions Halton sampler reached.");
  }
  return primes[index++];
}

void HaltonSampler::addDimension(double lower_bound, double upper_bound)
{
  Sampler::addDimension(lower_bound, upper_bound);
  primes_.push_back(next_prime());
}

std::vector<std::vector<double>> HaltonSampler::getSamples(const int n)
{
  std::vector<std::vector<double>> samples(n);
  for (int i = 0; i < n; ++i)
  {
    for (int dim = 0; dim < dimensions_; ++dim)
    {
      samples[i].push_back(scale(vdc(vdc_count_, primes_[dim]), dim));
      vdc_count_++;
    }
  }

  return samples;
}
}  // namespace arf