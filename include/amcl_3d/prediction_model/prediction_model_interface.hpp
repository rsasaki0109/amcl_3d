#pragma once
#include "amcl_3d/type.hpp"

namespace amcl_3d
{
class PredictionModelInterface
{
public:
  virtual bool predict(State &state, bool rising_edge = false, bool falling_edge = false) = 0;
};
} // namespace amcl_3d