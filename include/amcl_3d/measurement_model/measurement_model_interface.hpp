#pragma once
#include "amcl_3d/type.hpp"
#include <memory>
namespace amcl_3d
{
class MeasurementModelInterface
{
public:
  virtual bool measure(std::shared_ptr<const Particles> mesurement_point_particles_ptr,
                       std::shared_ptr<Particles> particles_ptr,
                       MeasurementState &measurement_state) = 0; // don't normalize weight because of augmented mcl
};

} // namespace amcl_3d