#include "arf_sampling/sampling.h"

std::vector<double> range(double lower_bound, double upper_bound, int num_samples)
{
    std::vector<double> range;
    if (num_samples == 1)
    {      
      range = { lower_bound };
    }
    else
    {
        range.resize(num_samples);
        double increment = (upper_bound - lower_bound) / (num_samples - 1);
        for (int i = 0; i < num_samples; ++i)
        {
            range[i] = lower_bound + i * increment;
        }
    }
    return range;
}

void Sampler::recursiveGridSampling(int index, std::vector<double> prev_values, std::vector<std::vector<double>>& grid)
{
    auto sample_range = range(lower_bounds_[index], upper_bounds_[index], num_samples_[index]);
    if (index < (dimensions_ - 1))
    {
        for (auto value : sample_range)
        {
            std::vector<double> pvi(prev_values);
            pvi.push_back(value);
            recursiveGridSampling(index + 1, pvi, grid);
        }
    }
    else
    {
        for (auto value : sample_range)
        {
            std::vector<double> pvi(prev_values);
            pvi.push_back(value);
            std::vector<double> new_element(pvi);
            grid.push_back(new_element);
        }
    }
}

void Sampler::addDimension(double lower_bound, double upper_bound, int num_samples)
{
    if (num_samples == 1 and lower_bound != upper_bound)
        throw std::invalid_argument("Lower bound should equal upper bound if number of samples is 1.");
    dimensions_ += 1;
    lower_bounds_.push_back(lower_bound);
    upper_bounds_.push_back(upper_bound);
    num_samples_.push_back(num_samples);
}

std::vector<std::vector<double>> Sampler::getGridSamples()
{
    std::vector<std::vector<double>> grid;
    std::vector<double> empty_vector;
    recursiveGridSampling(0, empty_vector, grid);

    return grid;
}
