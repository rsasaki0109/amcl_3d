#include "amcl_3d/prediction_model/foo_prediction_model_node.hpp"

namespace amcl_3d
{
FooPredictionModelNode::FooPredictionModelNode(std::shared_ptr<Amcl> amcl)
    : nh_(""), pnh_("~"), amcl_(amcl)
{
    prediction_model_ = std::make_shared<FooPredictionModel>();
    odom_sub_ = nh_.subscribe("odom", 100, &FooPredictionModelNode::odomCallback, this);
    imu_sub_ = nh_.subscribe("imu", 100, &FooPredictionModelNode::imuCallback, this);
}

std::shared_ptr<PredictionModelInterface> FooPredictionModelNode::getPredictionModel()
{
    return prediction_model_;
}

void FooPredictionModelNode::odomCallback(const nav_msgs::Odometry::ConstPtr &input_odom_msg)
{
    // Eigen::Vector3d vel;
    // Eigen::Vector3d omega;
    // vel << input_odom_msg->twist.twist.linear.x,
    //     input_odom_msg->twist.twist.linear.y,
    //     input_odom_msg->twist.twist.linear.z;
    // omega << input_odom_msg->twist.twist.angular.x,
    //     input_odom_msg->twist.twist.angular.y,
    //     input_odom_msg->twist.twist.angular.z;
    // vel = 0.5 * (vel + prediction_model_->getLinearVelocity());
    // omega = 0.5 * (omega + prediction_model_->getAngularVelocity());
    // prediction_model_->measumentLinearVelocity(vel);
    // prediction_model_->measumentAngularVelocity(omega);
    // amcl_->predict(prediction_model_);
    Eigen::Vector3d vel;
    Eigen::Vector3d omega;
    vel << input_odom_msg->twist.twist.linear.x,
        input_odom_msg->twist.twist.linear.y,
        input_odom_msg->twist.twist.linear.z;
    vel = 0.5 * (vel + prediction_model_->getLinearVelocity());
    prediction_model_->measumentLinearVelocity(vel);
}

void FooPredictionModelNode::imuCallback(const sensor_msgs::Imu::ConstPtr &input_imu_msg)
{
    Eigen::Vector3d omega;
    omega << input_imu_msg->angular_velocity.x,
        input_imu_msg->angular_velocity.y,
        input_imu_msg->angular_velocity.z;
    omega = 0.5 * (omega + prediction_model_->getAngularVelocity());
    prediction_model_->measumentAngularVelocity(omega);
}

} // namespace amcl_3d
