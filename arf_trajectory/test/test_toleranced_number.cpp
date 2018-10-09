#include <iostream>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "arf_trajectory/trajectory.h"

const double TOLERANCE = 1e-8;
using Transform = Eigen::Isometry3d;
void comparePoses(const Transform& Ta, const Transform& Tb);

TEST(Number, constructor)
{
    Number x(3.0);
    Number y;
}

TEST(Number, casting)
{
    Number x(3.0);
    double y = x;
    EXPECT_EQ(y, 3.0);
}

TEST(TolerancedNumber, constructor)
{
    TolerancedNumber x(0.5, -1, 2);
    TolerancedNumber y(0.5, -1, 2, 6);
}

TEST(TolerancedNumber, wrongInput)
{
    try
    {
        TolerancedNumber x(0, 1, -1);
    }
    catch (std::invalid_argument const& err)
    {
        EXPECT_EQ(err.what(), std::string("Lower bound should be strictly smaller than upper bound."));
    }
    try
    {
        TolerancedNumber x(-2, -1, 1);
    }
    catch (std::invalid_argument const& err)
    {
        EXPECT_EQ(err.what(), std::string("Nominal value should be withing bounds."));
    }
    try
    {
        TolerancedNumber x(5, -1, 1);
    }
    catch (std::invalid_argument const& err)
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

TEST(TrajectoryPoint, gridSampling)
{
    TolerancedNumber x(1, 1, 2, 2);
    Number y(5);
    TolerancedNumber z(1, 1, 3, 3);
    Number rx, ry, rz;
    TrajectoryPoint tp(x, y, z, rx, ry, rz);

    std::vector<Transform> actual;
    actual = tp.getGridSamples();

    std::size_t N = actual.size();

    std::vector<Transform> expected;
    expected.resize(N);
    for (std::size_t i = 0; i < N; ++i)
        expected[i] = Eigen::Isometry3d::Identity();
    expected[0].translation() << 1, 5, 1;
    expected[1].translation() << 1, 5, 2;
    expected[2].translation() << 1, 5, 3;
    expected[3].translation() << 2, 5, 1;
    expected[4].translation() << 2, 5, 2;
    expected[5].translation() << 2, 5, 3;

    for (std::size_t i = 0; i < N; ++i)
        comparePoses(expected[i], actual[i]);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

void comparePoses(const Transform& Ta, const Transform& Tb)
{
    using Matrix = Eigen::Matrix3d;
    using Vector = Eigen::Vector3d;

    Matrix Ra = Ta.rotation(), Rb = Tb.rotation();
    for (int i = 0; i < Ra.rows(); ++i)
    {
        for (int j = 0; j < Ra.cols(); ++j)
        {
            EXPECT_NEAR(Ra(i, j), Rb(i, j), TOLERANCE);
        }
    }

    Vector pa = Ta.translation(), pb = Tb.translation();
    EXPECT_NEAR(pa[0], pb[0], TOLERANCE);
    EXPECT_NEAR(pa[1], pb[1], TOLERANCE);
    EXPECT_NEAR(pa[2], pb[2], TOLERANCE);
}