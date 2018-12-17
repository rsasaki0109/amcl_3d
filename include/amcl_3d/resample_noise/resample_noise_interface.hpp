#pragma once
#include "amcl_3d/xorshift128.hpp"

namespace amcl_3d
{
class ResampleNoiseInterface
{
public:
  virtual double getNoise(XorShift128& rand) = 0;
};

} // namespace amcl_3d