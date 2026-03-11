#include "amcl_3d/prediction_model/foo_prediction_model_node.hpp"

#include <functional>

namespace amcl_3d
{
namespace
{
Time messageTimeOrNow(const builtin_interfaces::msg::Time & stamp, const rclcpp::Node & node)
{
    if (stamp.sec == 0 && stamp.nanosec == 0)
    {
        return Time::fromRclcppTime(node.get_clock()->now());
    }
    return Time::fromROSTime(stamp);
}
} // namespace

FooPredictionModelNode::FooPredictionModelNode(rclcpp::Node & node, std::shared_ptr<Amcl> amcl)
    : node_(node), amcl_(std::move(amcl))
{
    prediction_model_ = std::make_shared<FooPredictionModel>();
    odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        rclcpp::SensorDataQoS(),
        std::bind(&FooPredictionModelNode::odomCallback, this, std::placeholders::_1));
    imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        rclcpp::SensorDataQoS(),
        std::bind(&FooPredictionModelNode::imuCallback, this, std::placeholders::_1));
}

std::shared_ptr<PredictionModelInterface> FooPredictionModelNode::getPredictionModel() const
{
    return prediction_model_;
}

void FooPredictionModelNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr input_odom_msg)
{
    Eigen::Vector3d vel;
    Eigen::Vector3d omega;
    vel << input_odom_msg->twist.twist.linear.x,
        input_odom_msg->twist.twist.linear.y,
        input_odom_msg->twist.twist.linear.z;
    omega << input_odom_msg->twist.twist.angular.x,
        input_odom_msg->twist.twist.angular.y,
        input_odom_msg->twist.twist.angular.z;
    vel = 0.5 * (vel + prediction_model_->getLinearVelocity());
    omega = 0.5 * (omega + prediction_model_->getAngularVelocity());
    prediction_model_->measumentLinearVelocity(vel);
    prediction_model_->measumentAngularVelocity(omega);
    amcl_->predict(prediction_model_, messageTimeOrNow(input_odom_msg->header.stamp, node_));
}

void FooPredictionModelNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr input_imu_msg)
{
    Eigen::Vector3d omega;
    omega << input_imu_msg->angular_velocity.x,
        input_imu_msg->angular_velocity.y,
        input_imu_msg->angular_velocity.z;
    omega = 0.5 * (omega + prediction_model_->getAngularVelocity());
    prediction_model_->measumentAngularVelocity(omega);
}

} // namespace amcl_3d
