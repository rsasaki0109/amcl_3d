#pragma once
#include "amcl_3d/prediction_model/prediction_model_interface.hpp"
#include "amcl_3d/xorshift128.hpp"
#include "amcl_3d/mcl.hpp"
#include <ros/ros.h>

namespace amcl_3d
{
class FooPredictionModel : public PredictionModelInterface
{
public:
  FooPredictionModel();
  FooPredictionModel(const Eigen::Vector3d &vel, const Eigen::Vector3d &omega);
  bool predict(State &state, bool rising_edge = false, bool falling_edge = false) override;
  bool predict(State &state, const double dt_sec);
  bool measumentLinearVelocity(const Eigen::Vector3d &vel);
  bool measumentAngularVelocity(const Eigen::Vector3d &omega);
  Eigen::Vector3d getLinearVelocity();
  Eigen::Vector3d getAngularVelocity();

private:
  ros::Time current_time_;
  ros::Time last_prediction_time_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d omega_;
  static XorShift128 rand_;
};

} // namespace amcl_3d