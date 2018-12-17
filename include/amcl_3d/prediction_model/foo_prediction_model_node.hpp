#pragma once
#include "amcl_3d/mcl.hpp"
#include "amcl_3d/prediction_model/foo_prediction_model.hpp"
#include "amcl_3d/prediction_model/prediction_model_interface.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

namespace amcl_3d
{
class FooPredictionModelNode
{
  public:
    FooPredictionModelNode(std::shared_ptr<Amcl> amcl);
    std::shared_ptr<PredictionModelInterface> getPredictionModel();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber odom_sub_; // odometry
    ros::Subscriber imu_sub_;  // imu
    void odomCallback(const nav_msgs::Odometry::ConstPtr &input_odom_msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &input_imu_msg);

    std::shared_ptr<Amcl> amcl_;
    std::shared_ptr<FooPredictionModel> prediction_model_;
};

} // namespace amcl_3d