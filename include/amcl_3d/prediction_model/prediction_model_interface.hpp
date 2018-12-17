#pragma once
#include "amcl_3d/type.hpp"

namespace amcl_3d
{
class PredictionModelInterface
{
public:
  virtual bool predict(State &state) = 0;
};
} // namespace amcl_3d