#pragma once
#include "amcl_3d/particle_filter_interface.hpp"
#include "amcl_3d/xorshift128.hpp"
#include <random>
#include <limits>

namespace amcl_3d
{
class ParticleFilter : public ParticleFilterInterface
{
public:
  ParticleFilter();

  virtual ~ParticleFilter(){};
  bool init(const Position &position, const Quat &quat, const NoiseGenerators &noise_ge, const size_t particle_num) override;
  bool resample(const KLDSamplingParam &param, const double random_particle_ratio,
                const NoiseGenerators &random_particle_noise_gen); // kld resample and random noised resample
  bool resample(const size_t particle_num,
                const double random_particle_ratio, const NoiseGenerators &random_particle_noise_gen); // random noised resample
  bool resample(const size_t particle_num);                                                            // normal resample
  bool predict(std::shared_ptr<PredictionModelInterface> model) override;
  bool measure(std::shared_ptr<MeasurementModelInterface> model, MeasurementState &measuremnt_state) override;
  bool getParticles(std::shared_ptr<const Particles> &particles_ptr) override;
  void normalizeWeight();
  State getMMSE(); // minimum mean square error
  State getMAP(); // maximum a posteriori
  size_t getParticleNum();
  double getESS(const bool normalized = false); // effective sample size

private:
  bool resample(std::shared_ptr<Particles> new_particles_ptr, const size_t particle_num,
                const double random_particle_ratio, const NoiseGenerators &random_particle_noise_gen);
  void normalizeWeight(std::shared_ptr<Particles> particles_ptr);
  double getNormalCDFQuantile(const double u); 
  XorShift128 rand_;
};
} // namespace amcl_3d