#include "amcl_3d/measurement_model/ndt_pose_measurement_model.hpp"
#include <iostream>

namespace amcl_3d
{
NdtPoseMeasurementModel::NdtPoseMeasurementModel(const Position &position, const Quat &quat, const PoseCovariance &covariance)
    : position_(position), quat_(quat), covariance_(covariance)
{
}

bool NdtPoseMeasurementModel::measure(std::shared_ptr<const Particles> mesurement_point_particles_ptr,
                                      std::shared_ptr<Particles> particles_ptr,
                                      MeasurementState &measurement_state)
{
    if (mesurement_point_particles_ptr->empty() || particles_ptr->empty())
        return false;

    PosisitonCovariance position_covariance;
    position_covariance = covariance_.block<3, 3>(0, 0);
    constexpr double sigma = 0.1;
    // calc log likelihood each particlle
    std::vector<double> v_log_likelihood;
    v_log_likelihood.reserve(particles_ptr->size());
    for (const auto &state : *mesurement_point_particles_ptr)
    {
        /*
         * likelihood = exp(-1/2*(x-mu)sigma_inv(x-mu)), x is variable, mu is avg
         * log likelihood = -1/2*(x-mu)sigma_inv(x-mu)
         */

        double log_likelihood = 0.0;
        // position likelihood
        log_likelihood = -1.0 / 2.0 * (state.position - position_).transpose() * position_covariance.inverse() * (state.position - position_);
        // quaternion likelihood
        const double distance = (1.0 - state.quat.dot(quat_) * state.quat.dot(quat_));
        log_likelihood += -1.0 / 2.0 * (distance * distance) / (sigma * sigma);

        v_log_likelihood.push_back(log_likelihood);
    }
    // just to be sure
    if (particles_ptr->size() != v_log_likelihood.size())
    {
        std::cerr << "Error:" << __FILE__ << ", line:" << __LINE__ << std::endl; // there is no way this run
        return false;
    }
    // calc weight and avoid numerical underflow
    // http://www.maths.lu.se/fileadmin/maths/forskning_research/InferPartObsProcess/particlemethods.pdf
    double max_log_likelihood = *std::max_element(v_log_likelihood.begin(), v_log_likelihood.end());
    for (size_t i = 0; i < particles_ptr->size(); ++i)
    {
        particles_ptr->at(i).weight *= std::exp(v_log_likelihood.at(i) - max_log_likelihood);
    }
    // calc raw weight average for augmented mcl
    double weight_sum = 0.0;
    for (size_t i = 0; i < particles_ptr->size(); ++i)
    {
        weight_sum += std::exp(v_log_likelihood.at(i));
    }
    measurement_state.raw_weight_avg = weight_sum / (double)particles_ptr->size();
    return true;
}
} // namespace amcl_3d