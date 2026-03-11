#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "amcl_3d/mcl.hpp"
#include "amcl_3d/prediction_model/foo_prediction_model_node.hpp"
#include "amcl_3d/time.hpp"
#include "amcl_3d/type.hpp"

namespace amcl_3d
{
namespace
{
std::string sanitizeFrameId(std::string frame_id)
{
  if (!frame_id.empty() && frame_id.front() == '/')
  {
    frame_id.erase(0, 1);
  }
  return frame_id;
}

Time stampOrNow(const builtin_interfaces::msg::Time & stamp, const rclcpp::Node & node)
{
  if (stamp.sec == 0 && stamp.nanosec == 0)
  {
    return Time::fromRclcppTime(node.get_clock()->now());
  }
  return Time::fromROSTime(stamp);
}

rclcpp::Time stampOrZero(const builtin_interfaces::msg::Time & stamp, const rclcpp::Node & node)
{
  if (stamp.sec == 0 && stamp.nanosec == 0)
  {
    return rclcpp::Time(0, 0, node.get_clock()->get_clock_type());
  }
  return rclcpp::Time(stamp);
}
} // namespace

class Amcl3dNode : public rclcpp::Node
{
public:
  Amcl3dNode();

private:
  void mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_map_msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr input_init_pose_msg);
  void pc2Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pc2_msg);
  void publishTimerCallback();

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pf_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<Amcl> amcl_;
  std::string world_frame_id_;
  std::string map_frame_id_;
  std::string base_link_frame_id_;
  std::string odom_frame_id_;
  std::shared_ptr<FooPredictionModelNode> prediction_model_node_;
};

Amcl3dNode::Amcl3dNode() : rclcpp::Node("amcl_3d")
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  AmclParam amcl_param;
  amcl_param.augmented_mcl.alpha_fast =
      this->declare_parameter<double>("amcl_param.augmented_mcl.alpha_fast", 0.95);
  amcl_param.augmented_mcl.alpha_slow =
      this->declare_parameter<double>("amcl_param.augmented_mcl.alpha_slow", 0.05);
  amcl_param.augmented_mcl.w_fast =
      this->declare_parameter<double>("amcl_param.augmented_mcl.w_fast", 0.5);
  amcl_param.augmented_mcl.w_slow =
      this->declare_parameter<double>("amcl_param.augmented_mcl.w_slow", 0.5);
  amcl_param.augmented_mcl.noise_x_var =
      this->declare_parameter<double>("amcl_param.augmented_mcl.noise_x_var", 1.0);
  amcl_param.augmented_mcl.noise_y_var =
      this->declare_parameter<double>("amcl_param.augmented_mcl.noise_y_var", 1.0);
  amcl_param.augmented_mcl.noise_z_var =
      this->declare_parameter<double>("amcl_param.augmented_mcl.noise_z_var", 0.2);
  amcl_param.augmented_mcl.noise_roll_var =
      this->declare_parameter<double>("amcl_param.augmented_mcl.noise_roll_var", 0.1);
  amcl_param.augmented_mcl.noise_pitch_var =
      this->declare_parameter<double>("amcl_param.augmented_mcl.noise_pitch_var", 0.1);
  amcl_param.augmented_mcl.noise_yaw_var =
      this->declare_parameter<double>("amcl_param.augmented_mcl.noise_yaw_var", 0.1);
  amcl_param.resample_timing.ess_ratio_threshold =
      this->declare_parameter<double>("amcl_param.resample_timing.ess_ratio_threshold", 0.7);
  amcl_param.init_pose.initial_particle_num = static_cast<size_t>(
      this->declare_parameter<int64_t>("amcl_param.init_pose.initial_particle_num", 100));
  amcl_param.kld_sampling.min_particle_num = static_cast<size_t>(
      this->declare_parameter<int64_t>("amcl_param.kld_sampling.min_particle_num", 100));
  amcl_param.kld_sampling.max_particle_num = static_cast<size_t>(
      this->declare_parameter<int64_t>("amcl_param.kld_sampling.max_particle_num", 2000));
  amcl_param.kld_sampling.delta =
      this->declare_parameter<double>("amcl_param.kld_sampling.delta", 0.5);
  amcl_param.kld_sampling.epsilon =
      this->declare_parameter<double>("amcl_param.kld_sampling.epsilon", 0.5);
  amcl_param.kld_sampling.x_bin_width =
      this->declare_parameter<double>("amcl_param.kld_sampling.x_bin_width", 0.2);
  amcl_param.kld_sampling.y_bin_width =
      this->declare_parameter<double>("amcl_param.kld_sampling.y_bin_width", 0.2);
  amcl_param.kld_sampling.z_bin_width =
      this->declare_parameter<double>("amcl_param.kld_sampling.z_bin_width", 0.2);

  world_frame_id_ = this->declare_parameter<std::string>("world_frame_id", "world");
  map_frame_id_ = this->declare_parameter<std::string>("map_frame_id", "map");
  base_link_frame_id_ = this->declare_parameter<std::string>("base_link_frame_id", "base_link");
  odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
  const double publish_rate = this->declare_parameter<double>("publish_rate", 10.0);
  if (publish_rate <= 0.0)
  {
    throw std::invalid_argument("publish_rate must be positive");
  }

  amcl_ = std::make_shared<Amcl>(amcl_param);
  prediction_model_node_ = std::make_shared<FooPredictionModelNode>(*this, amcl_);

  const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  pf_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particles", latched_qos);
  current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", latched_qos);
  pc2_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "map",
      latched_qos,
      std::bind(&Amcl3dNode::mapCallback, this, std::placeholders::_1));
  init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose",
      rclcpp::QoS(100),
      std::bind(&Amcl3dNode::initialPoseCallback, this, std::placeholders::_1));
  pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pc2",
      rclcpp::SensorDataQoS(),
      std::bind(&Amcl3dNode::pc2Callback, this, std::placeholders::_1));

  publish_timer_ = this->create_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate)),
      std::bind(&Amcl3dNode::publishTimerCallback, this));
}

void Amcl3dNode::mapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_map_msg)
{
  RCLCPP_INFO(this->get_logger(), "received map");

  auto ros_map = input_map_msg;
  const std::string source_frame_id = sanitizeFrameId(ros_map->header.frame_id);
  if (source_frame_id != world_frame_id_)
  {
    try
    {
      const auto transform_stamped = tf_buffer_->lookupTransform(
          world_frame_id_,
          source_frame_id,
          stampOrZero(ros_map->header.stamp, *this),
          rclcpp::Duration::from_seconds(0.2));
      sensor_msgs::msg::PointCloud2 transformed_ros_map;
      tf2::doTransform(*ros_map, transformed_ros_map, transform_stamped);
      ros_map = std::make_shared<sensor_msgs::msg::PointCloud2>(std::move(transformed_ros_map));
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
  }

  auto pc_map = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*ros_map, *pc_map);
  amcl_->setMap(pc_map);
}

void Amcl3dNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr input_init_pose_msg)
{
  RCLCPP_INFO(this->get_logger(), "received initial pose");

  geometry_msgs::msg::PoseWithCovarianceStamped init_transformed_pose = *input_init_pose_msg;
  const std::string source_frame_id = sanitizeFrameId(input_init_pose_msg->header.frame_id);

  if (source_frame_id != world_frame_id_)
  {
    try
    {
      const auto transform_stamped = tf_buffer_->lookupTransform(
          world_frame_id_,
          source_frame_id,
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(1.0));
      geometry_msgs::msg::PoseStamped raw_pose;
      raw_pose.header = input_init_pose_msg->header;
      raw_pose.pose = input_init_pose_msg->pose.pose;

      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(raw_pose, transformed_pose, transform_stamped);
      init_transformed_pose.pose.pose = transformed_pose.pose;
      init_transformed_pose.header.frame_id = world_frame_id_;
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
  }

  const auto & ros_position = init_transformed_pose.pose.pose.position;
  const auto & ros_quat = init_transformed_pose.pose.pose.orientation;
  const auto & ros_covariance = init_transformed_pose.pose.covariance;
  Position position(ros_position.x, ros_position.y, ros_position.z);
  Quat quat(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z);

  PoseCovariance covariance = PoseCovariance::Zero();
  covariance(0, 0) = ros_covariance.at(0 * 6 + 0);
  covariance(1, 1) = ros_covariance.at(1 * 6 + 1);
  covariance(2, 2) = ros_covariance.at(2 * 6 + 2);
  covariance(3, 3) = ros_covariance.at(3 * 6 + 3);
  covariance(4, 4) = ros_covariance.at(4 * 6 + 4);
  covariance(5, 5) = ros_covariance.at(5 * 6 + 5);

  if (covariance(0, 0) == 0.0 && covariance(1, 1) == 0.0 &&
      covariance(2, 2) == 0.0 && covariance(3, 3) == 0.0 &&
      covariance(4, 4) == 0.0 && covariance(5, 5) == 0.0)
  {
    RCLCPP_WARN(this->get_logger(), "covariance is all zeros; applying fallback defaults");
    covariance(0, 0) = 0.5;
    covariance(1, 1) = 0.5;
    covariance(2, 2) = 0.5;
    covariance(3, 3) = 0.1;
    covariance(4, 4) = 0.1;
    covariance(5, 5) = 0.1;
  }

  if (!amcl_->setInitialPose(position, quat, covariance))
  {
    RCLCPP_ERROR(this->get_logger(), "failed to set initial pose");
  }
}

void Amcl3dNode::pc2Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pc2_msg)
{
  auto ros_pc2 = input_pc2_msg;
  const Time measurement_time = stampOrNow(ros_pc2->header.stamp, *this);
  amcl_->predict(prediction_model_node_->getPredictionModel(), measurement_time);

  const std::string source_frame_id = sanitizeFrameId(ros_pc2->header.frame_id);
  if (source_frame_id != base_link_frame_id_)
  {
    try
    {
      const auto transform_stamped = tf_buffer_->lookupTransform(
          base_link_frame_id_,
          source_frame_id,
          stampOrZero(ros_pc2->header.stamp, *this),
          rclcpp::Duration::from_seconds(0.2));
      sensor_msgs::msg::PointCloud2 transformed_ros_pc2;
      tf2::doTransform(*ros_pc2, transformed_ros_pc2, transform_stamped);
      ros_pc2 = std::make_shared<sensor_msgs::msg::PointCloud2>(std::move(transformed_ros_pc2));
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
  }

  auto pc_measurement = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*ros_pc2, *pc_measurement);
  amcl_->measureLidar(measurement_time, pc_measurement);
}

void Amcl3dNode::publishTimerCallback()
{
  const rclcpp::Time current_time = this->get_clock()->now();
  amcl_->predict(prediction_model_node_->getPredictionModel(), Time::fromRclcppTime(current_time));

  geometry_msgs::msg::PoseArray output_particle_msg;
  output_particle_msg.header.stamp = current_time;
  output_particle_msg.header.frame_id = world_frame_id_;

  std::shared_ptr<const Particles> particles_ptr;
  amcl_->getParticles(particles_ptr);
  for (const State & state : *particles_ptr)
  {
    geometry_msgs::msg::Pose particle;
    particle.position.x = state.position.x();
    particle.position.y = state.position.y();
    particle.position.z = state.position.z();
    particle.orientation.x = state.quat.x();
    particle.orientation.y = state.quat.y();
    particle.orientation.z = state.quat.z();
    particle.orientation.w = state.quat.w();
    output_particle_msg.poses.push_back(particle);
  }
  pf_pub_->publish(output_particle_msg);

  const State world2base_link = amcl_->getMMSE();
  geometry_msgs::msg::TransformStamped ros_world2base_link;
  ros_world2base_link.header.frame_id = world_frame_id_;
  ros_world2base_link.child_frame_id = base_link_frame_id_;
  ros_world2base_link.header.stamp = current_time;
  ros_world2base_link.transform.translation.x = world2base_link.position.x();
  ros_world2base_link.transform.translation.y = world2base_link.position.y();
  ros_world2base_link.transform.translation.z = world2base_link.position.z();
  ros_world2base_link.transform.rotation.x = world2base_link.quat.x();
  ros_world2base_link.transform.rotation.y = world2base_link.quat.y();
  ros_world2base_link.transform.rotation.z = world2base_link.quat.z();
  ros_world2base_link.transform.rotation.w = world2base_link.quat.w();
  tf_broadcaster_->sendTransform(ros_world2base_link);

  geometry_msgs::msg::PoseStamped output_current_pose_msg;
  output_current_pose_msg.header.frame_id = world_frame_id_;
  output_current_pose_msg.header.stamp = current_time;
  output_current_pose_msg.pose.position.x = world2base_link.position.x();
  output_current_pose_msg.pose.position.y = world2base_link.position.y();
  output_current_pose_msg.pose.position.z = world2base_link.position.z();
  output_current_pose_msg.pose.orientation.x = world2base_link.quat.x();
  output_current_pose_msg.pose.orientation.y = world2base_link.quat.y();
  output_current_pose_msg.pose.orientation.z = world2base_link.quat.z();
  output_current_pose_msg.pose.orientation.w = world2base_link.quat.w();
  current_pose_pub_->publish(output_current_pose_msg);
}

} // namespace amcl_3d

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amcl_3d::Amcl3dNode>());
  rclcpp::shutdown();
  return 0;
}
