#include "amcl_3d/prediction_model/foo_prediction_model.hpp"
#include <iostream>
namespace amcl_3d
{
XorShift128 FooPredictionModel::rand_;
FooPredictionModel::FooPredictionModel()
{
    vel_ << 0.0, 0.0, 0.0;
    omega_ << 0.0, 0.0, 0.0;
}
FooPredictionModel::FooPredictionModel(const Eigen::Vector3d &vel,
                                       const Eigen::Vector3d &omega)
    : vel_(vel), omega_(omega)
{
}

bool FooPredictionModel::predict(State &state, const double dt_sec)
{
    Position local_position;
    Quat local_quat;
    std::normal_distribution<double> vel_scale_noise(1.0, 1.0),
        vel_bias_noise(0.0, 1.0),
        omega_scale_noise(1.0, 0.5),
        omega_bias_noise(0.0, 0.5);
    double vel = vel_.x() * vel_scale_noise(rand_) + vel_bias_noise(rand_);
    double omega = omega_.z() * omega_scale_noise(rand_) + omega_bias_noise(rand_);
    const double r = vel / omega;
    local_position.x() = r * std::sin(omega * dt_sec);
    local_position.y() = -r * std::cos(omega * dt_sec) + r;
    local_position.z() = 0.0;
    local_quat = Quat(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())                // roll
                      * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())              // pitch
                      * Eigen::AngleAxisd(omega * dt_sec, Eigen::Vector3d::UnitZ())); // yaw

    Eigen::Affine3d mat_global2local;
    Eigen::Translation<double, 3> translation =
        Eigen::Translation<double, 3>(state.position.x(), state.position.y(), state.position.z());
    mat_global2local = translation * state.quat.normalized();
    state.position = mat_global2local * local_position;

    state.quat = (state.quat * local_quat).normalized();
    return true;
}

bool FooPredictionModel::measumentLinearVelocity(const Eigen::Vector3d &vel)
{
    vel_ = vel;
    return true;
}
bool FooPredictionModel::measumentAngularVelocity(const Eigen::Vector3d &omega)
{
    omega_ = omega;
    return true;
}

Eigen::Vector3d FooPredictionModel::getLinearVelocity() { return vel_; }
Eigen::Vector3d FooPredictionModel::getAngularVelocity() { return omega_; }

} // namespace amcl_3d
