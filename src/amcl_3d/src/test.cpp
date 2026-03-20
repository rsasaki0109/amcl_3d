#include <memory>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "amcl_3d/mcl.hpp"
#include "amcl_3d/time.hpp"
#include "amcl_3d/type.hpp"

namespace
{
amcl_3d::AmclParam makeDefaultParam()
{
  amcl_3d::AmclParam p;
  p.augmented_mcl.alpha_fast = 0.95;
  p.augmented_mcl.alpha_slow = 0.05;
  p.augmented_mcl.w_fast = 1.0;
  p.augmented_mcl.w_slow = 1.0;
  p.augmented_mcl.noise_x_var = 1.0;
  p.augmented_mcl.noise_y_var = 1.0;
  p.augmented_mcl.noise_z_var = 0.2;
  p.augmented_mcl.noise_roll_var = 0.1;
  p.augmented_mcl.noise_pitch_var = 0.3;
  p.augmented_mcl.noise_yaw_var = 1.0;
  p.resample_timing.ess_ratio_threshold = 0.95;
  p.kld_sampling.min_particle_num = 10;
  p.kld_sampling.max_particle_num = 2000;
  p.kld_sampling.delta = 0.5;
  p.kld_sampling.epsilon = 0.5;
  p.kld_sampling.x_bin_width = 0.1;
  p.kld_sampling.y_bin_width = 0.1;
  p.kld_sampling.z_bin_width = 0.1;
  p.init_pose.initial_particle_num = 100;
  return p;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr makeSimpleCloud()
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->push_back(pcl::PointXYZ(0, 1, 0));
  cloud->push_back(pcl::PointXYZ(1, 0, 0));
  return cloud;
}
}  // namespace

TEST(Amcl3dTest, SetParamAndMap)
{
  amcl_3d::Amcl amcl(makeDefaultParam());
  EXPECT_TRUE(amcl.setMap(makeSimpleCloud()));
}

TEST(Amcl3dTest, SetInitialPose)
{
  amcl_3d::Amcl amcl(makeDefaultParam());
  amcl.setMap(makeSimpleCloud());

  Position position(0, 0, 0);
  Quat quat = Quat::Identity();
  PoseCovariance covariance = PoseCovariance::Zero();
  covariance(0, 0) = 1.0;
  covariance(1, 1) = 1.0;
  covariance(2, 2) = 0.5;

  EXPECT_TRUE(amcl.setInitialPose(position, quat, covariance));
}

TEST(Amcl3dTest, MeasureLidarAndGetMMSE)
{
  amcl_3d::Amcl amcl(makeDefaultParam());
  amcl.setMap(makeSimpleCloud());

  Position position(0, 0, 0);
  Quat quat = Quat::Identity();
  PoseCovariance covariance = PoseCovariance::Zero();
  covariance(0, 0) = 1.0;
  covariance(1, 1) = 1.0;
  covariance(2, 2) = 0.5;
  amcl.setInitialPose(position, quat, covariance);

  amcl_3d::Time t;
  t.sec = 1;
  t.nsec = 0;
  EXPECT_TRUE(amcl.measureLidar(t, makeSimpleCloud()));

  const auto mmse = amcl.getMMSE();
  // Map and measurement are identical at origin, so MMSE should stay near origin
  EXPECT_NEAR(mmse.position.x(), 0.0, 2.0);
  EXPECT_NEAR(mmse.position.y(), 0.0, 2.0);
  EXPECT_NEAR(mmse.position.z(), 0.0, 2.0);
}

TEST(Amcl3dTest, GetParticles)
{
  amcl_3d::Amcl amcl(makeDefaultParam());
  amcl.setMap(makeSimpleCloud());

  Position position(0, 0, 0);
  Quat quat = Quat::Identity();
  PoseCovariance covariance = PoseCovariance::Zero();
  covariance(0, 0) = 0.5;
  covariance(1, 1) = 0.5;
  covariance(2, 2) = 0.5;
  amcl.setInitialPose(position, quat, covariance);

  std::shared_ptr<const amcl_3d::Particles> particles;
  EXPECT_TRUE(amcl.getParticles(particles));
  EXPECT_GT(particles->size(), 0u);
}
