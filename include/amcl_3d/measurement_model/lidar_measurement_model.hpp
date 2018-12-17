#pragma once
#include "amcl_3d/measurement_model/measurement_model_interface.hpp"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "amcl_3d/xorshift128.hpp"

namespace amcl_3d
{
class LidarMeasurementModel : public MeasurementModelInterface
{
public:
  LidarMeasurementModel(pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_map_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr measurement_ptr,
                        const size_t random_sample_num,
                        /* max limit[m] to calc likelihood */ const double max_dist,
                        /* sigma[m] to calc likelihood */ const double sigma);
  bool measure(std::shared_ptr<Particles> particles_ptr, MeasurementState &measuremnt_state) override;
  void setKdMap(pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_map_ptr);
  void setMeasuement(pcl::PointCloud<pcl::PointXYZ>::Ptr measurement_ptr);
  bool isKdMap();
  void setRandomSampleNum(const size_t random_sample_num);
  size_t getRandomSampleNum();

private:
  void updateRandomSampleIndexVec(const size_t random_sample_num);
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_map_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr measurement_ptr_;
  std::vector<size_t> v_random_sample_index_;
  static XorShift128 rand_;

private:
  double max_dist_;
  double sigma_;
};

} // namespace amcl_3d