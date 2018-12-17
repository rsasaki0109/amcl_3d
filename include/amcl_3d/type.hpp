#pragma once
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Matrix6d PoseCovariance;             // x,y,z,roll,pitch,yaw
typedef Eigen::Matrix3d PosisitonCovariance; // x,y,z
typedef Eigen::Vector3d Position;            // x,y,z
typedef Eigen::Quaterniond Quat;             // x,y,z,w
typedef Eigen::Vector3d Euler;               // roll,pitch,yaw

namespace amcl_3d
{
struct State
{
    Position position; // x,y,z
    Quat quat;         // x,y,z,w
    double weight;
};

struct MeasurementState
{
    double raw_weight_avg;
};

struct AugmentedMCLParam
{
    double w_fast;
    double w_slow;
    double alpha_fast;
    double alpha_slow;
    double noise_x_var;
    double noise_y_var;
    double noise_z_var;
    double noise_roll_var;
    double noise_pitch_var;
    double noise_yaw_var;
};
struct ResampleTimingParam
{
    double ess_ratio_threshold;
};

struct KLDSamplingParam
{
    size_t min_particle_num;
    double delta;
    double epsilon;
    double x_bin_width;
    double y_bin_width;
    double z_bin_width;
};

struct AmclParam
{
    AugmentedMCLParam augmented_mcl;
    ResampleTimingParam resample_timing;
    KLDSamplingParam kld_sampling;
};

typedef std::vector<State> Particles;

} // namespace amcl_3d