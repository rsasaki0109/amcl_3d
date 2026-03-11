#pragma once
#include "amcl_3d/resample_noise/resample_noise_interface.hpp"
#include <random>

namespace amcl_3d
{
class NormalDistribution : public ResampleNoiseInterface
{
  public:
    NormalDistribution(const double avg, const double var);
    double getNoise(XorShift128& rand) override;
    void setVar(const double var) ;
    void setAvg(const double avg) ;
    double getVar() ;
    double getAvg() ;

  private:
    double avg_;
    double var_;
    std::normal_distribution<double> noise_;
};
} // namespace amcl_3d