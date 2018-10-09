# Sampler
This class bundles different sampling techniques to generate samples over an n-dimensional space.

## Usage
Every dimension is specified with the `addDimension` method like this:

```c++
Sampler my_sampler;
// specify lower bound, upper bound and number of samples (for this dimension)
my_sampler.addDimension(-1, 2, 4);
```

The above code will result in samples `[-1, 0, 1, 2]` for that dimension. The number of samples for a dimension is not relevant for incremental sampling techniques, only the bounds are used, and assumed included.

After we have specified the different dimension we can sample the space. Only grid sampling is currently implemented.
```c++
Sampler my_sampler;
my_sampler.addDimension(0, 1, 2);
my_sampler.addDimension(3, 3, 1);
my_sampler.addDimension(2, 3, 3);
std::vector<std::vector<double>> grid;
grid = my_sampler.getGridSamples();
// the 2D vector grid will contain:
// {0, 3, 2.0}
// {1, 3, 2.0}
// {0, 3, 2.5}
// {1, 3, 2.5}
// {0, 3, 3.0}
// {1, 3, 3.0}
```

When the number of samples equals 1, the lower bound should equal the upper bound. The lower bound is used as the sampled value for grid sampling (in the `range()` function).

## Internals
The grid samples are not saved inside the class, but generated when the function `getGridSamples()` is called.

The grid is calculated using a recursive function `recursiveGridSampling()`. This is not that easy to understand but works. An better implementation is welcome.

