#ifndef _SAMPLING_H_
#define _SAMPLING_H_

#include <iostream>
#include <vector>

class Sampler
{
  int dimensions_ = 0;
  std::vector<int> num_samples_;
  std::vector<double> lower_bounds_;
  std::vector<double> upper_bounds_;
  std::vector<std::vector<double>> sampled_ranges_;
public:
  Sampler() = default;

  void addDimension(int num_samples, double lower_bound, double upper_bound);
  std::vector<double> range(double lower_bound, double upper_bound, int num_samples);

  void recursiveGridSampling(int index, std::vector<double> prev_values, std::vector<std::vector<double>>& grid);
  std::vector<std::vector<double>> getGridSamples();
};

#endif