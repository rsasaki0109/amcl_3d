#pragma once
#include <memory>
#include "amcl_3d/type.hpp"
#include "amcl_3d/resample_noise/resample_noise_interface.hpp"
#include "amcl_3d/prediction_model/prediction_model_interface.hpp"
#include "amcl_3d/measurement_model/measurement_model_interface.hpp"

namespace amcl_3d
{
class ParticleFilterInterface
{
public:
  struct NoiseGenerators
  {
    NoiseGenerators(std::shared_ptr<ResampleNoiseInterface> x_noise_gen,
                    std::shared_ptr<ResampleNoiseInterface> y_noise_gen,
                    std::shared_ptr<ResampleNoiseInterface> z_noise_gen,
                    std::shared_ptr<ResampleNoiseInterface> roll_noise_gen,
                    std::shared_ptr<ResampleNoiseInterface> pitch_noise_gen,
                    std::shared_ptr<ResampleNoiseInterface> yaw_noise_gen) : x(x_noise_gen), y(y_noise_gen), z(z_noise_gen), roll(roll_noise_gen), pitch(pitch_noise_gen), yaw(yaw_noise_gen) {}
    std::shared_ptr<ResampleNoiseInterface> x;
    std::shared_ptr<ResampleNoiseInterface> y;
    std::shared_ptr<ResampleNoiseInterface> z;
    std::shared_ptr<ResampleNoiseInterface> roll;
    std::shared_ptr<ResampleNoiseInterface> pitch;
    std::shared_ptr<ResampleNoiseInterface> yaw;
  };
  virtual bool init(const Position &position, const Quat &quat, const NoiseGenerators &noise_ge, const size_t particle_num) = 0;
  virtual bool resample(const size_t particle_num,
                const double random_particle_ratio, const NoiseGenerators &random_particle_noise_gen) = 0;
  virtual bool resample(const size_t particle_num) = 0;
  virtual bool predict(std::shared_ptr<PredictionModelInterface> model) = 0;
  virtual bool measure(std::shared_ptr<MeasurementModelInterface> model, MeasurementState &measuremnt_state) = 0;
  virtual bool getParticles(std::shared_ptr<const Particles> &particles_ptr) = 0;
  ParticleFilterInterface() : particles_ptr_(std::make_shared<Particles>()) {}
  virtual ~ParticleFilterInterface() {}

protected:
  std::shared_ptr<Particles> particles_ptr_;
};

} // namespace amcl_3d