#include "arf_sampling/sampling.h"

#include <gtest/gtest.h>
#include <iostream>

void printGrid(std::vector<std::vector<double>>& grid)
{
  if (grid.size() > 0)
  {
    std::cout << "===== GRID =======\n";
    for (auto jv : grid)
    {
      std::cout << "( ";
      for (auto val : jv)
      {
        std::cout << val << ", ";
      }
      std::cout << ")\n";
    }
  }
  else
  {
    std::cout << "===== NO GRID ======";
  }
  std::cout << std::endl;
}

TEST(Sampler, addDimensions)
{
  Sampler s;
  s.addDimension(1, 0, 0);
  s.addDimension(2, 1, 2);
  s.addDimension(1, 99, 99);
  s.addDimension(3, 5, 7);
  std::vector<std::vector<double>> grid = s.getGridSamples();
  printGrid(grid);
}

// TEST(TolerancedNumber, range)
// {
//   TolerancedNumber x(0.5, -1, 2, 4);
//   std::vector<double> r = x.getRange();
//   ASSERT_EQ(r.size(), 4);
//   EXPECT_NEAR(r[0], -1, TOLERANCE);
//   EXPECT_NEAR(r[1],  0, TOLERANCE);
//   EXPECT_NEAR(r[2],  1, TOLERANCE);
//   EXPECT_NEAR(r[3],  2, TOLERANCE);

// }

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}