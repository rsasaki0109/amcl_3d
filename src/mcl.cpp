#include "amcl_3d/mcl.hpp"
#include "amcl_3d/measurement_model/lidar_measurement_model.hpp"
#include "amcl_3d/measurement_model/ndt_pose_measurement_model.hpp"
#include "amcl_3d/resample_noise/normal_distribution.hpp"

namespace amcl_3d
{
Amcl::Amcl() : pf_ptr_(std::make_shared<ParticleFilter>()){};
Amcl::Amcl(const AmclParam &param)
    : pf_ptr_(std::make_shared<ParticleFilter>()), param_(param){};

bool Amcl::setParam(const AmclParam &param)
{
    param_ = param;
    return true;
}

AmclParam Amcl::getParam() { return param_; }

bool Amcl::setMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map)
{
    if (map == nullptr)
        return false;

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_map_ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    kd_map_ptr_ = kd_map_ptr;
    kd_map_ptr_->setInputCloud(map);
    return true;
}

bool Amcl::getParticles(std::shared_ptr<const Particles> &particles_ptr)
{
    pf_ptr_->getParticles(particles_ptr);
    return true;
}

bool Amcl::measureLidar(const Time &time, const pcl::PointCloud<pcl::PointXYZ>::Ptr measuement)
{
#if 0
    if (kd_map_ptr_ == nullptr || measuement == nullptr)
        return false;
    MeasurementState measurement_state;
    const size_t random_sample_num = 100;
    const double max_dist = 1.0;
    const double sigma = 1.0;
    std::shared_ptr<MeasurementModelInterface> model =
        std::make_shared<LidarMeasurementModel>(kd_map_ptr_, measuement, random_sample_num, max_dist, sigma);
    pf_ptr_->measure(model, time, measurement_state);

    checkResample(measurement_state);
#endif
    return true;
}

bool Amcl::measureNdtPose(std::shared_ptr<const Particles> particles_ptr, const Position &position, const Quat &quat, const PoseCovariance &covariance)
{
    MeasurementState measurement_state;
    std::shared_ptr<MeasurementModelInterface> model =
        std::make_shared<NdtPoseMeasurementModel>(position, quat, covariance);
    pf_ptr_->measure(particles_ptr, model, measurement_state);
    checkResample(measurement_state);
    return true;
}

bool Amcl::checkResample(const MeasurementState &measurement_state)
{
    param_.augmented_mcl.w_fast =
        param_.augmented_mcl.w_fast +
        param_.augmented_mcl.alpha_fast * (measurement_state.raw_weight_avg - param_.augmented_mcl.w_fast);
    param_.augmented_mcl.w_slow =
        param_.augmented_mcl.w_slow +
        param_.augmented_mcl.alpha_slow * (measurement_state.raw_weight_avg - param_.augmented_mcl.w_slow);
    const double random_sampling_ratio = std::max(0.0, 1.0 - (param_.augmented_mcl.w_fast / param_.augmented_mcl.w_slow));
    // std::cout << "measurement_state.raw_weight_avg: " << measurement_state.raw_weight_avg << std::endl;
    // std::cout << "param_.augmented_mcl.w_fast: " << param_.augmented_mcl.w_fast << std::endl;
    // std::cout << "param_.augmented_mcl.w_slow: " << param_.augmented_mcl.w_slow << std::endl;
    // std::cout << " 1.0 - (param_.augmented_mcl.w_fast / param_.augmented_mcl.w_slow): " <<  1.0 - (param_.augmented_mcl.w_fast / param_.augmented_mcl.w_slow) << std::endl;
    // std::cout << "random_sampling_ratio: " << random_sampling_ratio << std::endl;
    pf_ptr_->normalizeWeight();

    double checked_ess_ratio_threshold = std::max(param_.resample_timing.ess_ratio_threshold, 1.0 / (double)pf_ptr_->getParticleNum());
    // The smaller the ess ratio, the larger the variation of the weight
    const double ess_ratio = pf_ptr_->getESS(/*normalized*/ true) / (double)pf_ptr_->getParticleNum();
    // std::cout << "ess_ratio: " << ess_ratio << "(<= checked_ess_ratio_threshold:" << checked_ess_ratio_threshold << ")" << std::endl;

    if (ess_ratio <= checked_ess_ratio_threshold)
    {
        std::shared_ptr<ResampleNoiseInterface> x_noise_ptr =
            std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ param_.augmented_mcl.noise_x_var);
        std::shared_ptr<ResampleNoiseInterface> y_noise_ptr =
            std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ param_.augmented_mcl.noise_y_var);
        std::shared_ptr<ResampleNoiseInterface> z_noise_ptr =
            std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ param_.augmented_mcl.noise_z_var);
        std::shared_ptr<ResampleNoiseInterface> roll_noise_ptr =
            std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ param_.augmented_mcl.noise_roll_var);
        std::shared_ptr<ResampleNoiseInterface> pitch_noise_ptr =
            std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ param_.augmented_mcl.noise_pitch_var);
        std::shared_ptr<ResampleNoiseInterface> yaw_noise_ptr =
            std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ param_.augmented_mcl.noise_yaw_var);
        ParticleFilter::NoiseGenerators
            noise_gens(x_noise_ptr, y_noise_ptr, z_noise_ptr, roll_noise_ptr, pitch_noise_ptr, yaw_noise_ptr);
        // pf_ptr_->resample(pf_ptr_->getParticleNum(), 0.3, noise_gens); // random resample
        // pf_ptr_->resample(pf_ptr_->getParticleNum(), random_sampling_ratio, noise_gens); // random resample
        pf_ptr_->resample(param_.kld_sampling, random_sampling_ratio, noise_gens); // random resample and kld sample
    }
}

bool Amcl::predict(std::shared_ptr<PredictionModelInterface> model)
{
    return predict(model, Time::getTimeNow());
}

bool Amcl::predict(std::shared_ptr<PredictionModelInterface> model, const Time &time)
{
    pf_ptr_->predict(model, time);
    return true;
}

bool Amcl::predict(std::shared_ptr<Particles> particles_ptr, std::shared_ptr<PredictionModelInterface> model, const Time &time)
{
    pf_ptr_->predict(particles_ptr, model, time);
    return true;
}

bool Amcl::setInitialPose(const Position &position, const Quat &quat, const PoseCovariance &covariance)
{
    return setInitialPose(position, quat, covariance, param_.init_pose.initial_particle_num);
}

bool Amcl::setInitialPose(const Position &position, const Quat &quat, const PoseCovariance &covariance, const size_t particle_num)
{
    std::shared_ptr<ResampleNoiseInterface> x_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ covariance(0, 0));
    std::shared_ptr<ResampleNoiseInterface> y_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ covariance(1, 1));
    std::shared_ptr<ResampleNoiseInterface> z_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ covariance(2, 2));
    std::shared_ptr<ResampleNoiseInterface> roll_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ covariance(3, 3));
    std::shared_ptr<ResampleNoiseInterface> pitch_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ covariance(4, 4));
    std::shared_ptr<ResampleNoiseInterface> yaw_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ covariance(5, 5));
    ParticleFilter::NoiseGenerators
        noise_gens(x_noise_ptr, y_noise_ptr, z_noise_ptr, roll_noise_ptr, pitch_noise_ptr, yaw_noise_ptr);
    return pf_ptr_->init(position, quat, noise_gens, particle_num);
}

State Amcl::getMMSE()
{
    return pf_ptr_->getMMSE();
}

State Amcl::getMAP()
{
    return pf_ptr_->getMAP();
}

} // namespace amcl_3d