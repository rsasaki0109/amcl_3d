#pragma once
#include "amcl_3d/measurement_model/measurement_model_interface.hpp"

namespace amcl_3d
{
class NdtPoseMeasurementModel : public MeasurementModelInterface
{
public:
  NdtPoseMeasurementModel(const Position &position, const Quat &quat, const PoseCovariance &covariance);
  bool measure(std::shared_ptr<const Particles> mesurement_point_particles_ptr,
               std::shared_ptr<Particles> particles_ptr,
               MeasurementState &measurement_state) override;

private:
  Position position_;
  Quat quat_;
  PoseCovariance covariance_;
};

} // namespace amcl_3d