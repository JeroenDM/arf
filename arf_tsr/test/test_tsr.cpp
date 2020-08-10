#include <arf_tsr/task_space_region.h>
#include <arf_sampling/sampling.h>

#include <Eigen/Geometry>

#include <gtest/gtest.h>
#include <iostream>

using namespace arf;

const double TOLERANCE = 1e-8;
void comparePoses(const Transform& Ta, const Transform& Tb);

TEST(TestConstructor, TestPositionSamples)
{
  EXPECT_TRUE(true);

  // create all the input for the constructor
  auto tf = Eigen::Isometry3d::Identity();
  TSRBounds tsr_bounds;
  tsr_bounds.x = { -1, 1 };
  tsr_bounds.y = { 0, 0 };
  tsr_bounds.z = { 0, 0 };
  tsr_bounds.rx = { 0, 0 };
  tsr_bounds.ry = { 0, 0 };
  tsr_bounds.rz = { 0, 0 };
  SamplerPtr sampler = std::make_shared<Sampler>();
  std::vector<int> num_samples = { 3, 1, 1, 1, 1, 1 };

  TSR tsr(tf, tsr_bounds, sampler, num_samples);

  auto samples = tsr.getSamples();
  EXPECT_EQ(samples.size(), 3);

  // create the expected reference frames to check the output
  auto tf_1 = Eigen::Isometry3d::Identity();
  tf_1.translation().x() = -1.0;
  auto tf_2 = Eigen::Isometry3d::Identity();
  auto tf_3 = Eigen::Isometry3d::Identity();
  tf_3.translation().x() = 1.0;

  comparePoses(samples[0], tf_1);
  comparePoses(samples[1], tf_2);
  comparePoses(samples[2], tf_3);
}

TEST(TestConstructor, TestRotationSamples)
{
  EXPECT_TRUE(true);

  // create all the input for the constructor
  auto tf = Eigen::Isometry3d::Identity();
  TSRBounds tsr_bounds;
  tsr_bounds.x = { 0, 0 };
  tsr_bounds.y = { 0, 0 };
  tsr_bounds.z = { 0, 0 };
  tsr_bounds.rx = { -1, 1 };
  tsr_bounds.ry = { 0, 0 };
  tsr_bounds.rz = { 0, 0 };
  SamplerPtr sampler = std::make_shared<Sampler>();
  std::vector<int> num_samples = { 1, 1, 1, 3, 1, 1 };

  TSR tsr(tf, tsr_bounds, sampler, num_samples);

  auto samples = tsr.getSamples();
  EXPECT_EQ(samples.size(), 3);

  // create the expected reference frames to check the output
  auto tf_1 = Eigen::Isometry3d::Identity();
  tf_1 *= Eigen::AngleAxisd(-1, Eigen::Vector3d::UnitX());
  auto tf_2 = Eigen::Isometry3d::Identity();
  auto tf_3 = Eigen::Isometry3d::Identity();
  tf_3 *= Eigen::AngleAxisd(1, Eigen::Vector3d::UnitX());

  comparePoses(samples[0], tf_1);
  comparePoses(samples[1], tf_2);
  comparePoses(samples[2], tf_3);
}

TEST(TestConstructor, TestPosAndRotSamples)
{
  EXPECT_TRUE(true);

  // create all the input for the constructor
  auto tf = Eigen::Isometry3d::Identity();
  TSRBounds tsr_bounds;
  tsr_bounds.x = { -1, 1 };
  tsr_bounds.y = { 0, 0 };
  tsr_bounds.z = { 0, 0 };
  tsr_bounds.rx = { -1, 1 };
  tsr_bounds.ry = { 0, 0 };
  tsr_bounds.rz = { 0, 0 };
  SamplerPtr sampler = std::make_shared<Sampler>();
  std::vector<int> num_samples = { 3, 1, 1, 2, 1, 1 };

  TSR tsr(tf, tsr_bounds, sampler, num_samples);

  auto samples = tsr.getSamples();
  EXPECT_EQ(samples.size(), 6);

  // create the expected reference frames to check the output
  auto tf_1 = Eigen::Isometry3d::Identity();
  tf_1 *= Eigen::AngleAxisd(-1, Eigen::Vector3d::UnitX());
  tf_1.translation().x() = -1.0;

  auto tf_2 = Eigen::Isometry3d::Identity();
  tf_2 *= Eigen::AngleAxisd(-1, Eigen::Vector3d::UnitX());

  auto tf_3 = Eigen::Isometry3d::Identity();
  tf_3 *= Eigen::AngleAxisd(-1, Eigen::Vector3d::UnitX());
  tf_3.translation().x() = 1.0;

  auto tf_4 = Eigen::Isometry3d::Identity();
  tf_4 *= Eigen::AngleAxisd(1, Eigen::Vector3d::UnitX());
  tf_4.translation().x() = -1.0;

  auto tf_5 = Eigen::Isometry3d::Identity();
  tf_5 *= Eigen::AngleAxisd(1, Eigen::Vector3d::UnitX());

  auto tf_6 = Eigen::Isometry3d::Identity();
  tf_6 *= Eigen::AngleAxisd(1, Eigen::Vector3d::UnitX());
  tf_6.translation().x() = 1.0;

  comparePoses(samples[0], tf_1);
  comparePoses(samples[1], tf_4);
  comparePoses(samples[2], tf_2);
  comparePoses(samples[3], tf_5);
  comparePoses(samples[4], tf_3);
  comparePoses(samples[5], tf_6);
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