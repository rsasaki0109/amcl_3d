#pragma once
#include "amcl_3d/type.hpp"
#include "amcl_3d/time.hpp"
#include "amcl_3d/particle_filter.hpp"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <memory>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

namespace amcl_3d
{
class Amcl
{
public:
  Amcl();
  Amcl(const AmclParam &param);
  ~Amcl(){};
  bool setParam(const AmclParam &param);
  AmclParam getParam();
  bool setMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map);
  bool getParticles(std::shared_ptr<const Particles> &particles_ptr);
  bool measureLidar(const Time &time, const pcl::PointCloud<pcl::PointXYZ>::Ptr measuement);
  bool measureNdtPose(std::shared_ptr<const Particles> particles_ptr,
                      const Position &position,
                      const Quat &quat,
                      const PoseCovariance &covariance);
  bool predict(std::shared_ptr<PredictionModelInterface> model);
  bool predict(std::shared_ptr<PredictionModelInterface> model, const Time &time);
  bool predict(std::shared_ptr<Particles> particles_ptr,
               std::shared_ptr<PredictionModelInterface> model,
               const Time &time,
               const bool update_time = true);
  bool setInitialPose(const Position &position, const Quat &quat, const PoseCovariance &covariance);
  bool setInitialPose(const Position &position, const Quat &quat, const PoseCovariance &covariance, const size_t particle_num);
  State getMMSE();
  State getMAP();

private:
  bool checkResample(const MeasurementState &measurement_state);

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_map_ptr_;
  std::shared_ptr<ParticleFilter> pf_ptr_;
  AmclParam param_;
};
} // namespace amcl_3d
