#include <arf_tsr/task_space_region.h>
#include <arf_sampling/grid_sampler.h>

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
  TSRBounds tsr_bounds{ { { -1, 1 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } } };
  SamplerPtr sampler = std::make_shared<GridSampler>();
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

TEST(TestConstructor, TestVolume)
{
  EXPECT_TRUE(true);

  // create all the input for the constructor
  auto tf = Eigen::Isometry3d::Identity();
  TSRBounds tsr_bounds{ { { -1, 1 }, { 0, 0.3 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 1 } } };
  SamplerPtr sampler = std::make_shared<GridSampler>();
  std::vector<int> num_samples = { 3, 2, 1, 1, 1, 2 };

  TSR tsr(tf, tsr_bounds, sampler, num_samples);

  EXPECT_DOUBLE_EQ(tsr.volume(), 3.3);
  // multiply the angular part by 2
  EXPECT_DOUBLE_EQ(tsr.volume(2.0), 4.3);
}

TEST(TestConstructor, TestRotationSamples)
{
  EXPECT_TRUE(true);

  // create all the input for the constructor
  auto tf = Eigen::Isometry3d::Identity();
  TSRBounds tsr_bounds{ { { 0, 0 }, { 0, 0 }, { 0, 0 }, { -1, 1 }, { 0, 0 }, { 0, 0 } } };
  SamplerPtr sampler = std::make_shared<GridSampler>();
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
  TSRBounds tsr_bounds{ { { -1, 1 }, { 0, 0 }, { 0, 0 }, { -1, 1 }, { 0, 0 }, { 0, 0 } } };
  SamplerPtr sampler = std::make_shared<GridSampler>();
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

TEST(TestConversions, TestValuesAndPose)
{
  // create all the input for the constructor
  auto tf = Eigen::Isometry3d::Identity();
  TSRBounds tsr_bounds{ { { -1, 1 }, { 0, 0 }, { 0, 0 }, { -1, 1 }, { 0, 0 }, { 0, 0 } } };
  SamplerPtr sampler = std::make_shared<GridSampler>();
  std::vector<int> num_samples = { 3, 1, 1, 2, 1, 1 };

  // create the task space region
  TSR tsr(tf, tsr_bounds, sampler, num_samples);

  Eigen::Vector6d v;
  v << -0.1, -0.2, -0.3, 0.1, 0.2, 0.3;
  auto tf_v = tsr.valuesToPose(v);
  auto new_v = tsr.poseToValues(tf_v);

  for (std::size_t i{ 0 }; i < v.size(); ++i)
  {
    EXPECT_NEAR(v[i], new_v[i], TOLERANCE) << "Values not equal at dimension " << i;
  }

  // v = { -0.1, -0.2, -0.3, -0.1, 0.2, 0.3 };
  // tf_v = tsr.valuesToPose(v);
  // new_v = tsr.poseToValues(tf_v);

  // for (std::size_t i{ 0 }; i < v.size(); ++i)
  // {
  //   EXPECT_NEAR(v.at(i), new_v.at(i), TOLERANCE) << "Values not equal at dimension " << i;
  // }

  // std::cout << tsr.ep_ << std::endl;
}

TEST(TestEulerStuff, Stuff)
{
  // create all the input for the constructor
  auto tf = Eigen::Isometry3d::Identity();
  TSRBounds tsr_bounds{ { { -1, 1 }, { 0, 0 }, { 0, 0 }, { -1, 1 }, { 0, 0 }, { 0, 0 } } };
  SamplerPtr sampler = std::make_shared<GridSampler>();
  std::vector<int> num_samples = { 3, 1, 1, 2, 1, 1 };

  // create the task space region
  TSR tsr(tf, tsr_bounds, sampler, num_samples);

  Eigen::Vector3d a1(0.1, 0.2, 0.3);
  auto res1 = minNormEquivalent(a1);
  std::cout << res1.transpose() << std::endl;

  EXPECT_NEAR(a1[0], res1[0], TOLERANCE);
  EXPECT_NEAR(a1[1], res1[1], TOLERANCE);
  EXPECT_NEAR(a1[2], res1[2], TOLERANCE);

  Eigen::Vector3d a2(3.0, -3.0, 0.3);
  Eigen::Vector3d expected2(3.0 - M_PI, 3.0 - M_PI, 0.3 - M_PI);
  auto res2 = minNormEquivalent(a2);
  std::cout << res2.transpose() << std::endl;
  std::cout << expected2.transpose() << std::endl;

  EXPECT_NEAR(expected2[0], res2[0], TOLERANCE);
  EXPECT_NEAR(expected2[1], res2[1], TOLERANCE);
  EXPECT_NEAR(expected2[2], res2[2], TOLERANCE);

  Eigen::Vector6d v_before, v_after;
  v_before << 0, 0, 0, a2;
  v_after << 0, 0, 0, res2;
  auto tf2_before = tsr.valuesToPose(v_before);
  auto tf2_after = tsr.valuesToPose(v_after);
  comparePoses(tf2_before, tf2_after);
}

TEST(TestMath, TestPoseDistanceSimple)
{
  auto tf = Eigen::Isometry3d::Identity();
  tf.translation() << 1, 2, 3;

  auto tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() << 1, 2, 3.987;
  tf2 *= Eigen::AngleAxisd(1.123, Eigen::Vector3d::UnitX());

  auto d = poseDistance(tf, tf2);
  EXPECT_DOUBLE_EQ(d[0], 0.0);
  EXPECT_DOUBLE_EQ(d[1], 0.0);
  EXPECT_DOUBLE_EQ(d[2], 0.987);

  EXPECT_DOUBLE_EQ(d[3], 1.123);
  EXPECT_DOUBLE_EQ(d[4], 0.0);
  EXPECT_DOUBLE_EQ(d[5], 0.0);
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