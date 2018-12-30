#include "amcl_3d/prediction_model/foo_prediction_model.hpp"
#include <iostream>
namespace amcl_3d
{
XorShift128 FooPredictionModel::rand_;
FooPredictionModel::FooPredictionModel()
{
    //http://wiki.ros.org/roscpp/Overview/Time
    /*
    When using simulated Clock time, now() returns time 0 
    until first message has been received on /clock, so 0 
    means essentially that the client does not know clock 
    time yet. A value of 0 should therefore be treated 
    differently, such as looping over now() until non-zero 
    is returned.
    */
    ros::Duration d = ros::Duration(0.1);
    while (true)
    {
        if (ros::Time::now().toSec() != 0.0)
            break;
        d.sleep();
    }
    last_prediction_time_ = ros::Time::now();
    vel_ << 0.0, 0.0, 0.0;
    omega_ << 0.0, 0.0, 0.0;
}
FooPredictionModel::FooPredictionModel(const Eigen::Vector3d &vel,
                                       const Eigen::Vector3d &omega)
    : vel_(vel), omega_(omega)
{
    //http://wiki.ros.org/roscpp/Overview/Time
    /*
    When using simulated Clock time, now() returns time 0 
    until first message has been received on /clock, so 0 
    means essentially that the client does not know clock 
    time yet. A value of 0 should therefore be treated 
    differently, such as looping over now() until non-zero 
    is returned.
    */
    ros::Duration d = ros::Duration(0.1);
    while (true)
    {
        if (ros::Time::now().toSec() != 0.0)
            break;
        d.sleep();
    }
    last_prediction_time_ = ros::Time::now();
}

bool FooPredictionModel::predict(State &state, bool rising_edge, bool falling_edge)
{
    if (rising_edge)
        current_time_ = ros::Time::now();

    const double dt_sec =
        (current_time_ - last_prediction_time_).toSec();
    if (dt_sec > 0.0)
    {
        predict(state, dt_sec);
        if (falling_edge)
            last_prediction_time_ = current_time_;
    }
    return true;
}

bool FooPredictionModel::predict(State &state, const double dt_sec)
{
    const Eigen::Matrix3d rot_map2base_link = state.quat.normalized().toRotationMatrix();
    const Eigen::Vector3d vel_map = rot_map2base_link.inverse() * vel_;
    /* linear */
    {
        std::normal_distribution<double> noise(0.0, 1);
        state.position.x() += (vel_map.x() + noise(rand_)) * dt_sec;
        state.position.y() += (vel_map.y() + noise(rand_)) * dt_sec;
        state.position.z() += (vel_map.z() + noise(rand_)) * dt_sec;
    }
    /* rotation */
    {
        std::normal_distribution<double> noise(0.0, 0.4);
        Eigen::Matrix3d skew_omega;
        skew_omega << 0.0, -omega_.z() + noise(rand_), omega_.y() + noise(rand_),
            omega_.z() + noise(rand_), 0.0, -omega_.x() + noise(rand_),
            -omega_.y() + noise(rand_), omega_.x() + noise(rand_), 0.0;
        Eigen::Matrix3d rot_map2base_link_next = rot_map2base_link + rot_map2base_link * skew_omega * dt_sec;
        rot_map2base_link_next.col(0) = rot_map2base_link_next.col(0).normalized();
        rot_map2base_link_next.col(1) = rot_map2base_link_next.col(1).normalized();
        rot_map2base_link_next.col(2) = rot_map2base_link_next.col(2).normalized();
        state.quat = Quat(rot_map2base_link_next);
    }
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
