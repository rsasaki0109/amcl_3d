#include "amcl_3d/resample_noise/normal_distribution.hpp"

namespace amcl_3d
{
NormalDistribution::NormalDistribution(const double avg, const double var) : avg_(avg), var_(var), noise_(avg_, var_) {}
double NormalDistribution::getNoise(XorShift128& rand)
{
    return noise_(rand);
}
void NormalDistribution::setVar(const double var)
{
    var_ = var;
    noise_ = std::normal_distribution<double>(avg_, var_);
}
void NormalDistribution::setAvg(const double avg)
{
    avg_ = avg;
    noise_ = std::normal_distribution<double>(avg_, var_);
}
double NormalDistribution::getVar() { return var_; }
double NormalDistribution::getAvg() { return avg_; }
} // namespace amcl_3d