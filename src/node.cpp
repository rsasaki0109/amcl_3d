#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
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
#include "amcl_3d/type.hpp"
#include "amcl_3d/time.hpp"
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
  ros::Publisher current_pose_pub_;
  ros::Subscriber init_pose_sub_; // initial pose
  ros::Subscriber pc2_map_sub_;   // pointcloud2 map data
  ros::Subscriber pc2_sub_;       // pointcloud2 measurement data
  ros::Subscriber ndt_pose_sub_;  // ndt pose measurement data
  ros::Timer publish_timer_;      // publish timer
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  void mapCallback(const sensor_msgs::PointCloud2::ConstPtr &input_map_msg);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input_init_pose_msg);
  void ndtPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &input_ndt_pose_msg);
  void pc2Callback(const sensor_msgs::PointCloud2::ConstPtr &input_pc2_msg);
  void publishTimerCallback(const ros::TimerEvent &e);

private:
  std::shared_ptr<Amcl> amcl_;
  std::string world_frame_id_;
  // std::string map_frame_id_;
  // std::string base_link_frame_id_;
  // std::string odom_frame_id_;
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
  pnh_.param<std::string>("world_frame_id", world_frame_id_, std::string("world"));
  // pnh_.param<std::string>("map_frame_id", map_frame_id_, std::string("map"));
  // pnh_.param<std::string>("base_link_frame_id", base_link_frame_id_, std::string("base_link"));
  // pnh_.param<std::string>("odom_frame_id", odom_frame_id_, std::string("odom"));
  double publish_rate;
  pnh_.param<double>("publish_rate", publish_rate, double(100.0));
  pf_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particles", 1, true);
  current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_pose", 1, true);
  //  pc2_map_sub_ = nh_.subscribe("map", 10, &Amcl3dNode::mapCallback, this);
  init_pose_sub_ = nh_.subscribe("initialpose", 100, &Amcl3dNode::initialPoseCallback, this);
  ndt_pose_sub_ = nh_.subscribe("ndt_pose", 1, &Amcl3dNode::ndtPoseCallback, this);
  //  pc2_sub_ = nh_.subscribe("pc2", 1, &Amcl3dNode::pc2Callback, this);
  publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate), &Amcl3dNode::publishTimerCallback, this);
}

void Amcl3dNode::mapCallback(const sensor_msgs::PointCloud2::ConstPtr &input_map_msg)
{
#if 0
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
#endif
}

void Amcl3dNode::ndtPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &input_ndt_pose_msg)
{
  std::shared_ptr<const Particles> particles_ptr;
  amcl_->getParticles(particles_ptr);
  std::shared_ptr<Particles> copied_particles_ptr(new Particles(*particles_ptr)); // for time back
  amcl_->predict(copied_particles_ptr, prediction_model_node_->getPredictionModel(), Time::fromROSTime(input_ndt_pose_msg->header.stamp));

  // transform map data to world frame id coordinate
  geometry_msgs::PoseStamped::Ptr ndt_pose(new geometry_msgs::PoseStamped(*input_ndt_pose_msg));
  if (input_ndt_pose_msg->header.frame_id != world_frame_id_)
  {
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
      std::string src_frame_id = input_ndt_pose_msg->header.frame_id;
      if (src_frame_id.front() == '/')
      {
        src_frame_id.erase(0, 1);
      }
      transform_stamped = tf_buffer_.lookupTransform(/*target*/ world_frame_id_, /*src*/ src_frame_id,
                                                     input_ndt_pose_msg->header.stamp);
      geometry_msgs::PoseStamped::Ptr transformed_ndt_pose(new geometry_msgs::PoseStamped());
      tf2::doTransform(*input_ndt_pose_msg, *transformed_ndt_pose, transform_stamped);
      ndt_pose = transformed_ndt_pose;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
  }

  const geometry_msgs::Point &ros_position = ndt_pose->pose.position;
  const geometry_msgs::Quaternion &ros_quat = ndt_pose->pose.orientation;
  Position position(ros_position.x, ros_position.y, ros_position.z);
  Quat quat(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z);
  PoseCovariance covariance;
  covariance(/*x*/ 0, /*x*/ 0) = 1.0;         // x var
  covariance(/*y*/ 1, /*y*/ 1) = 1.0;         // y var
  covariance(/*z*/ 2, /*z*/ 2) = 1.0;         // z var
  covariance(/*roll*/ 3, /*roll*/ 3) = 0.5;   // roll var
  covariance(/*pitch*/ 4, /*pitch*/ 4) = 0.1; // pitch var
  covariance(/*yaw*/ 5, /*yaw*/ 5) = 0.5;     // yaw var
  // std::cout << position.x() << ", "
  //           << position.y() << ", "
  //           << position.z() << ", "
  //           << quat.x() << ", "
  //           << quat.y() << ", "
  //           << quat.z() << ", "
  //           << quat.w() << std::endl;

  amcl_->measureNdtPose(copied_particles_ptr, position, quat, covariance);
}

void Amcl3dNode::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input_init_pose_msg)
{

  // make data (position, quat, covariance) to set amcl
  const geometry_msgs::Point &ros_position = input_init_pose_msg->pose.pose.position;
  const geometry_msgs::Quaternion &ros_quat = input_init_pose_msg->pose.pose.orientation;
  const boost::array<double, 36ul> &ros_covariance = input_init_pose_msg->pose.covariance;
  Position position(ros_position.x, ros_position.y, ros_position.z);
  Quat quat(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z);

  PoseCovariance covariance;
  covariance(/*x*/ 0, /*x*/ 0) = ros_covariance.at(/*x*/ 0 * 6 + 0);             // x var
  covariance(/*y*/ 1, /*y*/ 1) = ros_covariance.at(/*y*/ 1 * 6 + 1);             // y var
  covariance(/*z*/ 2, /*z*/ 2) = ros_covariance.at(/*z*/ 2 * 6 + 2);             // z var
  covariance(/*roll*/ 3, /*roll*/ 3) = ros_covariance.at(/*roll*/ 3 * 6 + 3);    // roll var
  covariance(/*pitch*/ 4, /*pitch*/ 4) = ros_covariance.at(/*pitch*/ 4 * 6 + 4); // pitch var
  covariance(/*yaw*/ 5, /*yaw*/ 5) = ros_covariance.at(/*yaw*/ 5 * 6 + 5);       // yaw var
  // warn wrong covariance
  if (covariance(/*x*/ 0, /*x*/ 0) == 0.0 && covariance(/*y*/ 1, /*y*/ 1) == 0.0 &&
      covariance(/*z*/ 2, /*z*/ 2) == 0.0 && covariance(/*roll*/ 3, /*roll*/ 3) == 0.0 &&
      covariance(/*pitch*/ 4, /*pitch*/ 4) == 0.0 && covariance(/*yaw*/ 5, /*yaw*/ 5) == 0.0)
  {
    ROS_WARN("Covariance is 0. Please check & set covariance");
    covariance(/*x*/ 0, /*x*/ 0) = 0.5;         // x var
    covariance(/*y*/ 1, /*y*/ 1) = 0.5;         // y var
    covariance(/*z*/ 2, /*z*/ 2) = 0.5;         // z var
    covariance(/*roll*/ 3, /*roll*/ 3) = 0.1;   // roll var
    covariance(/*pitch*/ 4, /*pitch*/ 4) = 0.1; // pitch var
    covariance(/*yaw*/ 5, /*yaw*/ 5) = 0.1;     // yaw var
  }
  // set initial pose
  if (!amcl_->setInitialPose(position, quat, covariance))
    ROS_ERROR("Failed set initial pose");
}

void Amcl3dNode::pc2Callback(const sensor_msgs::PointCloud2::ConstPtr &input_pc2_msg)
{
#if 0
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
#endif
}

void Amcl3dNode::publishTimerCallback(const ros::TimerEvent &e)
{
  // particles
  const ros::Time current_time = ros::Time::now();
  amcl_->predict(prediction_model_node_->getPredictionModel());
  geometry_msgs::PoseArray output_particle_msg;

  output_particle_msg.header.stamp = current_time;
  output_particle_msg.header.frame_id = world_frame_id_;
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
    output_particle_msg.poses.push_back(particle);
  }
  pf_pub_.publish(output_particle_msg);

  // tf publish
  tf2::Transform tf_world2base_link;
  tf2::Transform tf_world2map;
  tf2::Transform tf_map2base_link;
  try
  {
    geometry_msgs::TransformStamped ros_world2map;
    ros_world2map = tf_buffer_.lookupTransform("world", "map", ros::Time(0));
    tf2::fromMsg(ros_world2map.transform, tf_world2map);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  State world2base_link = amcl_->getMMSE();
  {
    geometry_msgs::Pose ros_world2base_link;
    ros_world2base_link.position.x = world2base_link.position.x();
    ros_world2base_link.position.y = world2base_link.position.y();
    ros_world2base_link.position.z = world2base_link.position.z();
    ros_world2base_link.orientation.x = world2base_link.quat.x();
    ros_world2base_link.orientation.y = world2base_link.quat.y();
    ros_world2base_link.orientation.z = world2base_link.quat.z();
    ros_world2base_link.orientation.w = world2base_link.quat.w();
    tf2::fromMsg(ros_world2base_link, tf_world2base_link);
  }
  tf_map2base_link = tf_world2map.inverse() * tf_world2base_link;
  geometry_msgs::TransformStamped ros_map2base_link;
  ros_map2base_link.header.frame_id = "map";
  ros_map2base_link.child_frame_id = "base_link";
  ros_map2base_link.header.stamp = current_time;
  ros_map2base_link.transform = tf2::toMsg(tf_map2base_link);
  //  tf_broadcaster_.sendTransform(ros_map2base_link);

  // current pose
  geometry_msgs::PoseStamped output_current_pose_msg;
  output_current_pose_msg.header.frame_id = "map";
  output_current_pose_msg.header.stamp = current_time;
  output_current_pose_msg.pose.position.x = ros_map2base_link.transform.translation.x;
  output_current_pose_msg.pose.position.y = ros_map2base_link.transform.translation.y;
  output_current_pose_msg.pose.position.z = ros_map2base_link.transform.translation.z;
  output_current_pose_msg.pose.orientation = ros_map2base_link.transform.rotation;
  current_pose_pub_.publish(output_current_pose_msg);
}

} // namespace amcl_3d

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "amcl_3d");

  amcl_3d::Amcl3dNode node;
  ros::spin();

  return 0;
}
