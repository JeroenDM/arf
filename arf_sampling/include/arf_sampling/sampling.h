#ifndef _SAMPLING_H_
#define _SAMPLING_H_

#include <iostream>
#include <stdexcept>
#include <vector>

std::vector<double> range(double lower_bound, double upper_bound, int num_samples);

class Sampler
{
    int dimensions_ = 0;
    std::vector<double> lower_bounds_;
    std::vector<double> upper_bounds_;
    std::vector<int> num_samples_;

    void recursiveGridSampling(int index, std::vector<double> prev_values, std::vector<std::vector<double>>& grid);

  public:
    Sampler() = default;
    void addDimension(double lower_bound, double upper_bound, int num_samples);
    std::vector<std::vector<double>> getGridSamples();
};

#endif