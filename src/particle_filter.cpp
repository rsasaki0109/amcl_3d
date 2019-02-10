#include "amcl_3d/particle_filter.hpp"
// #include <iostream>
namespace amcl_3d
{
ParticleFilter::ParticleFilter() : particles_ptr_(std::make_shared<Particles>()) {}

bool ParticleFilter::init(const Position &position, const Quat &quat, const NoiseGenerators &noise_gen, const size_t particle_num)
{
    // Initialize particle
    particles_ptr_->clear();
    particles_ptr_->reserve(particle_num);

    for (size_t i = 0; i < particle_num; ++i)
    {
        State noised_state;
        // Add noise to position
        {
            noised_state.position.x() = position.x() + noise_gen.x->getNoise(rand_);
            noised_state.position.y() = position.y() + noise_gen.y->getNoise(rand_);
            noised_state.position.z() = position.z() + noise_gen.z->getNoise(rand_);
        }
        // Add noise to quaternion
        {
            // Euler euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
            // const double &roll = euler[0];
            // const double &pitch = euler[1];
            // const double &yaw = euler[2];
            // Quat noised_quat(Eigen::AngleAxisd(roll + noise_gen.roll->getNoise(rand_), Eigen::Vector3d::UnitX())     // roll
            //                  * Eigen::AngleAxisd(pitch + noise_gen.pitch->getNoise(rand_), Eigen::Vector3d::UnitY()) // pitch
            //                  * Eigen::AngleAxisd(yaw + noise_gen.yaw->getNoise(rand_), Eigen::Vector3d::UnitZ()));   // yaw
            // noised_state.quat = noised_quat;
            Quat noise_quat(Eigen::AngleAxisd(noise_gen.roll->getNoise(rand_), Eigen::Vector3d::UnitX())    // roll
                            * Eigen::AngleAxisd(noise_gen.pitch->getNoise(rand_), Eigen::Vector3d::UnitY()) // pitch
                            * Eigen::AngleAxisd(noise_gen.yaw->getNoise(rand_), Eigen::Vector3d::UnitZ())); // yaw
            noised_state.quat = noise_quat * quat;
        }
        // Initialize weight
        noised_state.weight = 1.0;

        particles_ptr_->push_back(noised_state);
    }
    normalizeWeight(particles_ptr_);

    last_prediction_time_ = Time::getTimeNow();
    return true;
}

bool ParticleFilter::resample(const size_t particle_num)
{
    std::shared_ptr<Particles> new_particles_ptr = std::make_shared<Particles>();
    std::shared_ptr<ResampleNoiseInterface> x_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ 0.0);
    std::shared_ptr<ResampleNoiseInterface> y_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ 0.0);
    std::shared_ptr<ResampleNoiseInterface> z_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ 0.0);
    std::shared_ptr<ResampleNoiseInterface> roll_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ 0.0);
    std::shared_ptr<ResampleNoiseInterface> pitch_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ 0.0);
    std::shared_ptr<ResampleNoiseInterface> yaw_noise_ptr =
        std::make_shared<NormalDistribution>(/*avg*/ 0.0, /*var*/ 0.0);
    ParticleFilter::NoiseGenerators
        noise_gens(x_noise_ptr, y_noise_ptr, z_noise_ptr, roll_noise_ptr, pitch_noise_ptr, yaw_noise_ptr);
    resample(new_particles_ptr, particle_num, 0.0, noise_gens);
    particles_ptr_ = new_particles_ptr;
    normalizeWeight(particles_ptr_);

    return true;
}

bool ParticleFilter::resample(const size_t particle_num,
                              const double random_particle_ratio, const NoiseGenerators &random_particle_noise_gen)
{
    std::shared_ptr<Particles> new_particles_ptr = std::make_shared<Particles>();
    resample(new_particles_ptr, particle_num, random_particle_ratio, random_particle_noise_gen);
    particles_ptr_ = new_particles_ptr;
    normalizeWeight(particles_ptr_);

    return true;
}

bool ParticleFilter::resample(const KLDSamplingParam &param, const double random_particle_ratio,
                              const NoiseGenerators &random_particle_noise_gen)
{
    std::shared_ptr<Particles> new_particles_ptr = std::make_shared<Particles>();
    // minimum resample
    resample(new_particles_ptr, param.min_particle_num, random_particle_ratio, random_particle_noise_gen);
    // calc bin size for kld boundary
    std::vector<Eigen::Vector3i> v_true_bin;
    // std::cout << "new_particles_ptr->size():" << new_particles_ptr->size() << std::endl;
    for (const auto &state : *new_particles_ptr)
    {
        Eigen::Vector3i bin;
        bin << (int)(state.position.x() / param.x_bin_width),
            (int)(state.position.y() / param.y_bin_width),
            (int)(state.position.z() / param.z_bin_width);
        bool find(false);
        // std::cout << "bin:" << bin << std::endl;
        for (const auto &true_bin : v_true_bin)
        {
            if (true_bin == bin)
            {
                find = true;
                // std::cout << "find:" << find << std::endl;
            }
        }
        if (!find)
        {
            v_true_bin.push_back(bin);
        }
    }
    // std::cout << "v_true_bin.size():" << v_true_bin.size() << std::endl;
    // add sample based on kld boundary
    if (1 < v_true_bin.size())
    {
        const double z = getNormalCDFQuantile(param.delta);
        double k = (double)v_true_bin.size();
        double chi = 1.0 - 2.0 / (9.0 * (k - 1)) + std::sqrt(2.0 / (9.0 * (k - 1))) * z;
        double kld_bound_particle_num = (k - 1) / (2.0 * param.epsilon) * chi * chi * chi;
        std::uniform_real_distribution<double> random_particle_ratio_dist(0.0, 1.0);
        // std::cout << "kld_bound_particle_num:" << kld_bound_particle_num << std::endl;

        while (true)
        {
            // check kld boundary
            if ((size_t)(kld_bound_particle_num + 0.5) <= new_particles_ptr->size() || param.max_particle_num < new_particles_ptr->size())
            {
                break;
            }

            // add particle
            State state;
            std::uniform_int_distribution<int> random_sample_index_dist(1, (particles_ptr_->size() - 1));
            int random_sample_index = random_sample_index_dist(rand_);
            if (random_particle_ratio_dist(rand_) < random_particle_ratio)
            {
                State &noised_state = state;
                // Add noise to position
                {
                    const Position &position = particles_ptr_->at(random_sample_index).position;
                    noised_state.position.x() = position.x() + random_particle_noise_gen.x->getNoise(rand_);
                    noised_state.position.y() = position.y() + random_particle_noise_gen.y->getNoise(rand_);
                    noised_state.position.z() = position.z() + random_particle_noise_gen.z->getNoise(rand_);
                }
                // Add noise to quaternion
                {
                    const Quat &quat = particles_ptr_->at(random_sample_index).quat;
                    // Euler euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
                    // const double &roll = euler[0];
                    // const double &pitch = euler[1];
                    // const double &yaw = euler[2];
                    // Quat noised_quat(Eigen::AngleAxisd(roll + random_particle_noise_gen.roll->getNoise(rand_), Eigen::Vector3d::UnitX())     // roll
                    //                  * Eigen::AngleAxisd(pitch + random_particle_noise_gen.pitch->getNoise(rand_), Eigen::Vector3d::UnitY()) // pitch
                    //                  * Eigen::AngleAxisd(yaw + random_particle_noise_gen.yaw->getNoise(rand_), Eigen::Vector3d::UnitZ()));   // yaw
                    // noised_state.quat = noised_quat;
                    Quat noise_quat(Eigen::AngleAxisd(random_particle_noise_gen.roll->getNoise(rand_), Eigen::Vector3d::UnitX())    // roll
                                    * Eigen::AngleAxisd(random_particle_noise_gen.pitch->getNoise(rand_), Eigen::Vector3d::UnitY()) // pitch
                                    * Eigen::AngleAxisd(random_particle_noise_gen.yaw->getNoise(rand_), Eigen::Vector3d::UnitZ())); // yaw
                    noised_state.quat = noise_quat * quat;
                }
                // Initialize weight
                noised_state.weight = 1.0;
            }
            else
            {
                state = particles_ptr_->at(random_sample_index);
            }
            new_particles_ptr->push_back(state);
            // update bin and kld_bound_particle_num
            Eigen::Vector3i bin;
            bin << (int)(state.position.x() / param.x_bin_width),
                (int)(state.position.y() / param.y_bin_width),
                (int)(state.position.z() / param.z_bin_width);
            bool find(false);
            for (const auto &true_bin : v_true_bin)
            {
                if (true_bin == bin)
                {
                    find = true;
                }
            }
            if (!find)
            {
                v_true_bin.push_back(bin);
                k = (double)v_true_bin.size();
                chi = 1.0 - 2.0 / (9.0 * (k - 1)) + std::sqrt(2.0 / (9.0 * (k - 1))) * z;
                kld_bound_particle_num = (k - 1) / (2.0 * param.epsilon) * chi * chi * chi;
                // std::cout << "v_true_bin.size():" << v_true_bin.size() << std::endl;
                // std::cout << "kld_bound_particle_num:" << kld_bound_particle_num << std::endl;
            }
        }
    }
    // std::cout << "kld sampled particle size:" << new_particles_ptr->size() << std::endl;
    particles_ptr_ = new_particles_ptr;
    normalizeWeight(particles_ptr_);

    return true;
}

bool ParticleFilter::resample(std::shared_ptr<Particles> new_particles_ptr, const size_t particle_num,
                              const double random_particle_ratio, const NoiseGenerators &random_particle_noise_gen)
{
    double weight_interval;
    {
        double weight_sum = 0.0;
        for (const auto &particle : *particles_ptr_)
        {
            weight_sum += particle.weight;
        }
        weight_interval = weight_sum / (double)particle_num;
    }
    Particles::iterator particle_itr = particles_ptr_->begin();
    double weight_sum = 0.0;
    double weight_interval_sum = 0.0;
    // resampling from legacy particles
    const size_t resample_particle_num = (size_t)((double)particle_num * (1.0 - random_particle_ratio) + 0.5);
    for (size_t i = 0; i < resample_particle_num; ++i)
    {
        weight_interval_sum += weight_interval;
        while (weight_interval_sum - (weight_interval / 2.0) >= weight_sum)
        {
            if (particle_itr == particles_ptr_->end())
                break;
            weight_sum += particle_itr->weight;
            ++particle_itr;
        }
        State new_state = *(particle_itr - 1);
        // Initialize weight
        new_state.weight = 1.0;

        new_particles_ptr->push_back(new_state);
    }
    // add random particles
    for (size_t i = new_particles_ptr->size(); i < particle_num; ++i)
    {
        weight_interval_sum += weight_interval;
        while (weight_interval_sum - (weight_interval / 2.0) >= weight_sum)
        {
            if (particle_itr == particles_ptr_->end())
                break;
            weight_sum += particle_itr->weight;
            ++particle_itr;
        }
        State noised_state;
        // Add noise to position
        {
            const Position &position = (particle_itr - 1)->position;
            noised_state.position.x() = position.x() + random_particle_noise_gen.x->getNoise(rand_);
            noised_state.position.y() = position.y() + random_particle_noise_gen.y->getNoise(rand_);
            noised_state.position.z() = position.z() + random_particle_noise_gen.z->getNoise(rand_);
        }
        // Add noise to quaternion
        {
            const Quat &quat = (particle_itr - 1)->quat;
            Euler euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
            const double &roll = euler[0];
            const double &pitch = euler[1];
            const double &yaw = euler[2];
            Quat noised_quat(Eigen::AngleAxisd(roll + random_particle_noise_gen.roll->getNoise(rand_), Eigen::Vector3d::UnitX())     // roll
                             * Eigen::AngleAxisd(pitch + random_particle_noise_gen.pitch->getNoise(rand_), Eigen::Vector3d::UnitY()) // pitch
                             * Eigen::AngleAxisd(yaw + random_particle_noise_gen.yaw->getNoise(rand_), Eigen::Vector3d::UnitZ()));   // yaw
            noised_state.quat = noised_quat;
        }
        // Initialize weight
        noised_state.weight = 1.0;
        new_particles_ptr->push_back(noised_state);
    }
}

State ParticleFilter::getMMSE()
{
    State mmse_state;
    mmse_state.position.x() = 0.0;
    mmse_state.position.y() = 0.0;
    mmse_state.position.z() = 0.0;
    double weight_sum = 0.0;
    for (const auto &particle : *particles_ptr_)
    {
        weight_sum += particle.weight;
    }
    mmse_state.weight = weight_sum;
    for (const auto &particle : *particles_ptr_)
    {
        mmse_state.position.x() += particle.position.x() * (particle.weight / weight_sum);
        mmse_state.position.y() += particle.position.y() * (particle.weight / weight_sum);
        mmse_state.position.z() += particle.position.z() * (particle.weight / weight_sum);
    }

    Eigen::Vector2d vec_roll, vec_pitch, vec_yaw;
    vec_roll.x() = 0.0;
    vec_roll.y() = 0.0;
    vec_pitch.x() = 0.0;
    vec_pitch.y() = 0.0;
    vec_yaw.x() = 0.0;
    vec_yaw.y() = 0.0;
    for (const auto &particle : *particles_ptr_)
    {
        Euler euler = particle.quat.toRotationMatrix().eulerAngles(0, 1, 2);
        vec_roll.x() += std::cos(euler[0]) * particle.weight / weight_sum;
        vec_roll.y() += std::sin(euler[0]) * particle.weight / weight_sum;
        vec_pitch.x() += std::cos(euler[1]) * particle.weight / weight_sum;
        vec_pitch.y() += std::sin(euler[1]) * particle.weight / weight_sum;
        vec_yaw.x() += std::cos(euler[2]) * particle.weight / weight_sum;
        vec_yaw.y() += std::sin(euler[2]) * particle.weight / weight_sum;
    }
    double roll, pitch, yaw;
    roll = std::atan2(vec_roll.y(), vec_roll.x());
    pitch = std::atan2(vec_pitch.y(), vec_pitch.x());
    yaw = std::atan2(vec_yaw.y(), vec_yaw.x());

    mmse_state.quat = Quat(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())    // roll
                           * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) // pitch
                           * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())); // yaw

    return mmse_state;
}

State ParticleFilter::getMAP()
{
    State map_state;
    double max_weight = 0.0;
    for (const auto &particle : *particles_ptr_)
    {
        if (max_weight < particle.weight)
        {
            max_weight = particle.weight;
            map_state = particle;
        }
    }
    return map_state;
}
bool ParticleFilter::predict(std::shared_ptr<PredictionModelInterface> model, const Time &time)
{
    return predict(particles_ptr_, model, time);
}

bool ParticleFilter::predict(std::shared_ptr<Particles> particles_ptr, std::shared_ptr<PredictionModelInterface> model, const Time &time, bool update_time)
{
    double dt_sec = Time::getDiff(last_prediction_time_, time);
    for (size_t i = 0; i < particles_ptr->size(); ++i)
    {
        model->predict(particles_ptr->at(i), dt_sec);
    }
    if (update_time)
        last_prediction_time_ = time;
    return true;
}

bool ParticleFilter::measure(std::shared_ptr<MeasurementModelInterface> model, MeasurementState &measuremnt_state)
{
    return measure(particles_ptr_, model, measuremnt_state);
}

bool ParticleFilter::measure(std::shared_ptr<const Particles> measurement_point_particles_ptr, std::shared_ptr<MeasurementModelInterface> model, MeasurementState &measuremnt_state)
{
    return model->measure(measurement_point_particles_ptr, particles_ptr_, measuremnt_state);
}

bool ParticleFilter::getParticles(std::shared_ptr<const Particles> &particles_ptr)
{
    particles_ptr = particles_ptr_;
    return true;
}

void ParticleFilter::normalizeWeight()
{
    normalizeWeight(particles_ptr_);
}

void ParticleFilter::normalizeWeight(std::shared_ptr<Particles> particles_ptr)
{
    double weight_sum = 0.0;
    for (const auto &particle : *particles_ptr)
    {
        weight_sum += particle.weight;
    }
    for (auto &particle : *particles_ptr)
    {
        particle.weight /= weight_sum;
    }
}

size_t ParticleFilter::getParticleNum()
{
    return particles_ptr_->size();
}

double ParticleFilter::getESS(const bool normalized) // effective sample size
{
    // in order to calc effective sample size, particles must be normalized weight
    double weight_square_sum = 0.0;
    if (!normalized)
    {
        double weight_sum = 0.0;
        for (const auto &particle : *particles_ptr_)
        {
            weight_sum += particle.weight;
        }
        const double denominator = weight_sum * weight_sum;
        for (const auto &particle : *particles_ptr_)
        {
            weight_square_sum += particle.weight * particle.weight / denominator;
        }
    }
    else
    {
        for (const auto &particle : *particles_ptr_)
        {
            weight_square_sum += particle.weight * particle.weight;
        }
    }
    if (weight_square_sum <= 0.0)
        return std::numeric_limits<double>::max();
    return 1.0 / weight_square_sum;
}

double ParticleFilter::getNormalCDFQuantile(const double u)
{
    constexpr double a[9] = {1.24818987e-4, -1.075204047e-3, 5.198775019e-3,
                             -0.019198292004, 0.059054035642, -0.151968751364,
                             0.319152932694, -0.5319230073, 0.797884560593};
    constexpr double b[15] = {-4.5255659e-5, 1.5252929e-4, -1.9538132e-5,
                              -6.76904986e-4, 1.390604284e-3, -7.9462082e-4,
                              -2.034254874e-3, 6.549791214e-3, -0.010557625006,
                              0.011630447319, -9.279453341e-3, 5.353579108e-3,
                              -2.141268741e-3, 5.35310549e-4, 0.999936657524};
    double w, y, z;
    int i;

    if (u == 0.)
        return (0.5);
    y = u / 2.0;
    if (y < -3.)
        return (0.0);
    if (y > 3.)
        return (1.0);
    if (y < 0.0)
        y = -y;
    if (y < 1.0)
    {
        w = y * y;
        z = a[0];
        for (i = 1; i < 9; i++)
            z = z * w + a[i];
        z *= (y * 2.0);
    }
    else
    {
        y -= 2.0;
        z = b[0];
        for (i = 1; i < 15; i++)
            z = z * y + b[i];
    }

    if (u < 0.0)
        return ((1. - z) / 2.0);
    return ((1. + z) / 2.0);
}

} // namespace amcl_3d