#ifndef _RANDOM_SAMPLING_H_
#define _RANDOM_SAMPLING_H_

#include "arf_sampling/sampler.h"

#include <vector>
#include <random>

namespace arf
{
/** separate function because the random engine cannot be instantiated as a class attribute.
 * Are seeds generated using std::seed_seq better?
 * https://stackoverflow.com/questions/16841939/share-random-number-engine-between-different-methods-within-a-class-in-c11
 **/
std::mt19937 make_random_engine()
{
  std::random_device seed_generator;
  return std::mt19937(seed_generator());
}

class RandomSampler : public Sampler
{
  std::mt19937 random_engine_{ make_random_engine() };
  std::uniform_real_distribution<double> randu_;  // between 0 an 1;

public:
  RandomSampler() = default;
  RandomSampler(std::uint32_t seed) : random_engine_(seed)
  {
  }
  ~RandomSampler() = default;
  std::vector<std::vector<double>> getSamples(const int n = 1) override;
};
}  // namespace arf

#endif