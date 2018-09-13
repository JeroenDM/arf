
#include <gtest/gtest.h>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

#include "arf_trajectory/trajectory.h"

const double TOLERANCE = 1e-8;

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

TEST(test_setup, works)
{

  ASSERT_TRUE(true);
}

TEST(Number, constructor)
{
  Number x(3.0);
  Number y;
}

TEST(Number, casting)
{
  Number x(3.0);
  double y = x;
  EXPECT_FALSE(x.hasTolerance());
  EXPECT_EQ(y, 3.0);
}

TEST(TolerancedNumber, constructor)
{
  TolerancedNumber x(0.5, -1, 2);
  TolerancedNumber y(0.5, -1, 2, 6);
  EXPECT_TRUE(x.hasTolerance());
}

TEST(TolerancedNumber, wrongInput)
{
  try
  {
    TolerancedNumber x(0, 1, -1);
  }
  catch(std::invalid_argument const & err)
  {
    EXPECT_EQ(err.what(), std::string("Lower bound should be strictly smaller than upper bound."));
  }
  try
  {
    TolerancedNumber x(-2, -1, 1);
  }
  catch(std::invalid_argument const & err)
  {
    EXPECT_EQ(err.what(), std::string("Nominal value should be withing bounds."));
  }
  try
  {
    TolerancedNumber x(5, -1, 1);
  }
  catch(std::invalid_argument const & err)
  {
    EXPECT_EQ(err.what(), std::string("Nominal value should be withing bounds."));
  }
}

TEST(TrajectoryPoint, constructor)
{
  TolerancedNumber x(1, 1, 2, 2);
  Number y(5);
  TolerancedNumber z(1, 1, 3, 3);
  Number rx, ry, rz;
  TrajectoryPoint tp(x, y, z, rx, ry, rz);
}

TEST(TrajectoryPoint, grid)
{
  TolerancedNumber x(1, 1, 2, 2);
  Number y(5);
  TolerancedNumber z(1, 1, 3, 3);
  Number rx, ry, rz;

  EXPECT_TRUE(x.hasTolerance());

  TrajectoryPoint tp(x, y, z, rx, ry, rz);
  std::vector<std::vector<double>> grid = tp.getGridSamples();
  printGrid(grid);
}

// TEST(TrajectoryPoint, getPoses)
// {
//   TolerancedNumber x(1, 1, 2, 2);
//   Number y(5);
//   TolerancedNumber z(1, 1, 3, 3);
//   Number rx, ry, rz;
//   TrajectoryPoint tp(&x, &y, &z, &rx, &ry, &rz);
//   tp.discretize();

//   std::vector<Eigen::Affine3d> p;
//   p = tp.getPoses();

//   for (auto t : p)
//   {
//     std::cout << "Translation: \n" << t.translation() << std::endl;
//     std::cout << "Rotation: \n" << t.rotation() << std::endl;
//   }
// }

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}