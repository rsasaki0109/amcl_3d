#pragma once
#include "amcl_3d/prediction_model/prediction_model_interface.hpp"
#include "amcl_3d/xorshift128.hpp"
#include "amcl_3d/mcl.hpp"

namespace amcl_3d
{
class FooPredictionModel : public PredictionModelInterface
{
public:
  FooPredictionModel();
  FooPredictionModel(const Eigen::Vector3d &vel, const Eigen::Vector3d &omega);
  bool predict(State &state, const double dt_sec) override;
  bool measumentLinearVelocity(const Eigen::Vector3d &vel);
  bool measumentAngularVelocity(const Eigen::Vector3d &omega);
  Eigen::Vector3d getLinearVelocity();
  Eigen::Vector3d getAngularVelocity();

private:
  Eigen::Vector3d vel_;
  Eigen::Vector3d omega_;
  static XorShift128 rand_;
};

} // namespace amcl_3d