#include "arf_planning/planner.h"

#include <gtest/gtest.h>

std::vector<TrajectoryPoint> createTrajectory()
{
  std::vector<TrajectoryPoint> ee_trajectory;
  for (int i = 0; i < 10; ++i)
  {
    TolerancedNumber x(1.0, 0.95, 1.05, 5);
    // Number y, z(0.5 + static_cast<double>(i) / 20);
    Number y(static_cast<double>(i) / 20), z(0.5);
    Number rx, ry(M_PI), rz;
    TrajectoryPoint tp(x, y, z, rx, ry, rz);
    ee_trajectory.push_back(tp);
  }
  return ee_trajectory;
}

TEST(planner, constructor)
{
  Planner planner;
  EXPECT_TRUE(true);
}

TEST(planner, setTrajectory)
{
  std::vector<TrajectoryPoint> trajectory = createTrajectory();
  Planner planner;
  planner.setTrajectory(trajectory);
  EXPECT_TRUE(true);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}