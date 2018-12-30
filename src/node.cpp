#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <memory>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/array.hpp>
#include "amcl_3d/mcl.hpp"
#include "amcl_3d/prediction_model/foo_prediction_model_node.hpp"
#include "amcl_3d/prediction_model/foo_prediction_model.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

#include <Eigen/Geometry>
namespace amcl_3d
{

class Amcl3dNode
{
public:
  Amcl3dNode();
  ~Amcl3dNode(){};

private: // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pf_pub_;
  ros::Subscriber init_pose_sub_; // initial pose
  ros::Subscriber pc2_map_sub_;   // pointcloud2 map data
  ros::Subscriber pc2_sub_;       // pointcloud2 measurement data
  ros::Timer publish_timer_;      // publish timer
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &input_map_msg);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input_init_pose_msg);
  void pc2Callback(const sensor_msgs::PointCloud2::ConstPtr &input_pc2_msg);
  void publishTimerCallback(const ros::TimerEvent &e);

private:
  std::shared_ptr<Amcl> amcl_;
  std::string map_frame_id_;
  std::string base_link_frame_id_;
  std::string odom_frame_id_;
  std::shared_ptr<FooPredictionModelNode> prediction_model_node_;
};

Amcl3dNode::Amcl3dNode() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  // amcl param
  {
    AmclParam amcl_param;
    pnh_.param<double>("amcl_param/augmented_mcl/alpha_fast", amcl_param.augmented_mcl.alpha_fast, double(0.9));
    pnh_.param<double>("amcl_param/augmented_mcl/alpha_slow", amcl_param.augmented_mcl.alpha_slow, double(0.1));
    pnh_.param<double>("amcl_param/augmented_mcl/w_fast", amcl_param.augmented_mcl.w_fast, double(0.5));
    pnh_.param<double>("amcl_param/augmented_mcl/w_slow", amcl_param.augmented_mcl.w_slow, double(0.5));
    pnh_.param<double>("amcl_param/augmented_mcl/noise_x_var", amcl_param.augmented_mcl.noise_x_var, double(1.0));
    pnh_.param<double>("amcl_param/augmented_mcl/noise_y_var", amcl_param.augmented_mcl.noise_y_var, double(1.0));
    pnh_.param<double>("amcl_param/augmented_mcl/noise_z_var", amcl_param.augmented_mcl.noise_z_var, double(0.2));
    pnh_.param<double>("amcl_param/augmented_mcl/noise_roll_var", amcl_param.augmented_mcl.noise_roll_var, double(0.1));
    pnh_.param<double>("amcl_param/augmented_mcl/noise_pitch_var", amcl_param.augmented_mcl.noise_pitch_var, double(0.3));
    pnh_.param<double>("amcl_param/augmented_mcl/noise_yaw_var", amcl_param.augmented_mcl.noise_yaw_var, double(1.0));
    pnh_.param<double>("amcl_param/resample_timing/ess_ratio_threshold", amcl_param.resample_timing.ess_ratio_threshold, double(0.9));
    int initial_particle_num;
    pnh_.param<int>("amcl_param/init_pose/initial_particle_num", initial_particle_num, int(10));
    amcl_param.init_pose.initial_particle_num = (size_t)(initial_particle_num);
    int min_particle_num;
    pnh_.param<int>("amcl_param/kld_sampling/min_particle_num", min_particle_num, int(10));
    amcl_param.kld_sampling.min_particle_num = (size_t)(min_particle_num);
    int max_particle_num;
    pnh_.param<int>("amcl_param/kld_sampling/max_particle_num", max_particle_num, int(10));
    amcl_param.kld_sampling.max_particle_num = (size_t)(max_particle_num);
    pnh_.param<double>("amcl_param/kld_sampling/delta", amcl_param.kld_sampling.delta, double(0.5));
    pnh_.param<double>("amcl_param/kld_sampling/epsilon", amcl_param.kld_sampling.epsilon, double(0.5));
    pnh_.param<double>("amcl_param/kld_sampling/x_bin_width", amcl_param.kld_sampling.x_bin_width, double(0.2));
    pnh_.param<double>("amcl_param/kld_sampling/y_bin_width", amcl_param.kld_sampling.y_bin_width, double(0.2));
    pnh_.param<double>("amcl_param/kld_sampling/z_bin_width", amcl_param.kld_sampling.z_bin_width, double(0.2));
    amcl_ = std::make_shared<Amcl>(amcl_param);
    prediction_model_node_ = std::make_shared<FooPredictionModelNode>(amcl_);
  }
  // ros param
  pnh_.param<std::string>("map_frame_id", map_frame_id_, std::string("map"));
  pnh_.param<std::string>("base_link_frame_id", base_link_frame_id_, std::string("base_link"));
  pnh_.param<std::string>("odom_frame_id", odom_frame_id_, std::string("odom"));
  double publish_rate;
  pnh_.param<double>("publish_rate", publish_rate, double(100.0));
  pf_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particles", 1, true);
  pc2_map_sub_ = nh_.subscribe("map", 10, &Amcl3dNode::mapCallback, this);
  init_pose_sub_ = nh_.subscribe("initialpose", 100, &Amcl3dNode::initialPoseCallback, this);
  pc2_sub_ = nh_.subscribe("pc2", 1, &Amcl3dNode::pc2Callback, this);
  publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate), &Amcl3dNode::publishTimerCallback, this);
}

void Amcl3dNode::mapCallback(const sensor_msgs::PointCloud2::ConstPtr &input_map_msg)
{
  sensor_msgs::PointCloud2::ConstPtr ros_map = input_map_msg;
  // transform map data to map frame id coordinate
  if (ros_map->header.frame_id != map_frame_id_)
  {
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer_.lookupTransform(/*target*/ map_frame_id_, /*src*/ ros_map->header.frame_id,
                                                     ros_map->header.stamp);
      sensor_msgs::PointCloud2::Ptr transformed_ros_map(new sensor_msgs::PointCloud2());
      tf2::doTransform(*ros_map, *transformed_ros_map, transform_stamped);
      ros_map = transformed_ros_map;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
  }
  // convert ros to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*ros_map, *pc_map);
  // set map
  amcl_->setMap(pc_map);
}

void Amcl3dNode::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input_init_pose_msg)
{

  // make data (position, quat, covariance) to set amcl
  const geometry_msgs::Point &ros_position = input_init_pose_msg->pose.pose.position;
  const geometry_msgs::Quaternion &ros_quat = input_init_pose_msg->pose.pose.orientation;
  const boost::array<double, 36ul> &ros_covariance = input_init_pose_msg->pose.covariance;
  Position position(ros_position.x, ros_position.y, ros_position.z);
  Quat quat(ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w);
  PoseCovariance covariance;
  covariance(/*x*/ 0, /*x*/ 0) = ros_covariance.at(/*x*/ 1 * 1 - 1);             // x var
  covariance(/*y*/ 1, /*y*/ 1) = ros_covariance.at(/*y*/ 2 * 2 - 1);             // y var
  covariance(/*z*/ 2, /*z*/ 2) = ros_covariance.at(/*z*/ 3 * 3 - 1);             // z var
  covariance(/*roll*/ 3, /*roll*/ 3) = ros_covariance.at(/*roll*/ 4 * 4 - 1);    // roll var
  covariance(/*pitch*/ 4, /*pitch*/ 4) = ros_covariance.at(/*pitch*/ 5 * 5 - 1); // pitch var
  covariance(/*yaw*/ 5, /*yaw*/ 5) = ros_covariance.at(/*yaw*/ 6 * 6 - 1);       // yaw var
  // warn wrong covariance
  if (covariance(/*x*/ 0, /*x*/ 0) == 0.0 || covariance(/*y*/ 1, /*y*/ 1) == 0.0 ||
      covariance(/*z*/ 2, /*z*/ 2) == 0.0 || covariance(/*roll*/ 3, /*roll*/ 3) == 0.0 ||
      covariance(/*pitch*/ 4, /*pitch*/ 4) == 0.0 || covariance(/*yaw*/ 5, /*yaw*/ 5) == 0.0)
  {
    ROS_WARN("Covariance is 0. Please check & set covariance");
    covariance(/*x*/ 0, /*x*/ 0) = 0.5;         // x var
    covariance(/*y*/ 1, /*y*/ 1) = 0.5;         // y var
    covariance(/*z*/ 2, /*z*/ 2) = 0.5;         // z var
    covariance(/*roll*/ 3, /*roll*/ 3) = 0.5;   // roll var
    covariance(/*pitch*/ 4, /*pitch*/ 4) = 0.1; // pitch var
    covariance(/*yaw*/ 5, /*yaw*/ 5) = 0.5;     // yaw var
  }
  // set initial pose
  if (!amcl_->setInitialPose(position, quat, covariance))
    ROS_ERROR("Failed set initial pose");
}

void Amcl3dNode::pc2Callback(const sensor_msgs::PointCloud2::ConstPtr &input_pc2_msg)
{
  amcl_->predict(prediction_model_node_->getPredictionModel());
  sensor_msgs::PointCloud2::ConstPtr ros_pc2 = input_pc2_msg;
  // transform sensor data to base link frame id coordinate
  if (ros_pc2->header.frame_id != base_link_frame_id_)
  {
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer_.lookupTransform(/*target*/ base_link_frame_id_, /*src*/ ros_pc2->header.frame_id,
                                                     ros_pc2->header.stamp);
      sensor_msgs::PointCloud2::Ptr transformed_ros_pc2(new sensor_msgs::PointCloud2());
      tf2::doTransform(*ros_pc2, *transformed_ros_pc2, transform_stamped);
      ros_pc2 = transformed_ros_pc2;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
  }
  // convert ros to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_measuement(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*ros_pc2, *pc_measuement);
  // calculate likelihood and resample particle
  amcl_->measureLidar(pc_measuement);
}

void Amcl3dNode::publishTimerCallback(const ros::TimerEvent &e)
{
  const ros::Time current_time = ros::Time::now();
  amcl_->predict(prediction_model_node_->getPredictionModel());
  geometry_msgs::PoseArray output_msg;

  output_msg.header.stamp = current_time;
  output_msg.header.frame_id = map_frame_id_;
  std::shared_ptr<const Particles> particles_ptr;
  amcl_->getParticles(particles_ptr);
  for (const State &state : *particles_ptr)
  {
    geometry_msgs::Pose particle;
    particle.position.x = state.position.x();
    particle.position.y = state.position.y();
    particle.position.z = state.position.z();
    particle.orientation.x = state.quat.x();
    particle.orientation.y = state.quat.y();
    particle.orientation.z = state.quat.z();
    particle.orientation.w = state.quat.w();
    output_msg.poses.push_back(particle);
  }
  pf_pub_.publish(output_msg);

  // tf publish
  tf2::Transform tf_odom2base_link;
  tf2::Transform tf_map2odom;
  tf2::Transform tf_map2base_link;
  try {
    geometry_msgs::TransformStamped ros_odom2base_link;
    ros_odom2base_link = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
    tf2::fromMsg(ros_odom2base_link.transform, tf_odom2base_link);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  State map2base_link = amcl_->getMAP();
  {
    geometry_msgs::Pose ros_map2base_link;
    ros_map2base_link.position.x = map2base_link.position.x();
    ros_map2base_link.position.y = map2base_link.position.y();
    ros_map2base_link.position.z = map2base_link.position.z();
    ros_map2base_link.orientation.x = map2base_link.quat.x();
    ros_map2base_link.orientation.y = map2base_link.quat.y();
    ros_map2base_link.orientation.z = map2base_link.quat.z();
    ros_map2base_link.orientation.w = map2base_link.quat.w();
    tf2::fromMsg(ros_map2base_link, tf_map2base_link);
  }
  tf_map2odom = tf_map2base_link * tf_odom2base_link.inverse();
  geometry_msgs::TransformStamped ros_map2odom;
  ros_map2odom.header.frame_id = "map";
  ros_map2odom.child_frame_id = "odom";
  ros_map2odom.header.stamp = current_time;
  ros_map2odom.transform = tf2::toMsg(tf_map2odom);
  tf_broadcaster_.sendTransform(ros_map2odom);
}

} // namespace amcl_3d

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "amcl_3d");

  amcl_3d::Amcl3dNode node;
  ros::spin();

  return 0;
}
