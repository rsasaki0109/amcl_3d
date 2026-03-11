#pragma once

#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "amcl_3d/mcl.hpp"
#include "amcl_3d/prediction_model/foo_prediction_model.hpp"
#include "amcl_3d/prediction_model/prediction_model_interface.hpp"

namespace amcl_3d
{
class FooPredictionModelNode
{
  public:
    FooPredictionModelNode(rclcpp::Node & node, std::shared_ptr<Amcl> amcl);
    std::shared_ptr<PredictionModelInterface> getPredictionModel() const;

  private:
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr input_odom_msg);
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr input_imu_msg);

    rclcpp::Node & node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::shared_ptr<Amcl> amcl_;
    std::shared_ptr<FooPredictionModel> prediction_model_;
};

} // namespace amcl_3d
