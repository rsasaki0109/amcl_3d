#include <iostream>
#include <memory>
#include "amcl_3d/mcl.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

#include <Eigen/Geometry>
int main(int argc, char *argv[])
{

    amcl_3d::Amcl amcl;
    // amcl param
    {
        amcl_3d::AmclParam amcl_param;
        amcl_param.augmented_mcl.alpha_fast = 0.95;
        amcl_param.augmented_mcl.alpha_slow = 0.05;
        amcl_param.augmented_mcl.w_fast = 1.0;
        amcl_param.augmented_mcl.w_slow = 1.0;
        amcl_param.augmented_mcl.noise_x_var = 1.0;
        amcl_param.augmented_mcl.noise_y_var = 1.0;
        amcl_param.augmented_mcl.noise_z_var = 0.2;
        amcl_param.augmented_mcl.noise_roll_var = 0.1;
        amcl_param.augmented_mcl.noise_pitch_var = 0.3;
        amcl_param.augmented_mcl.noise_yaw_var = 1.0;
        amcl_param.resample_timing.ess_ratio_threshold = 0.95;
        amcl_param.kld_sampling.min_particle_num = 10;
        amcl_param.kld_sampling.delta = 0.5;
        amcl_param.kld_sampling.epsilon = 0.5;
        amcl_param.kld_sampling.x_bin_width = 0.1;
        amcl_param.kld_sampling.y_bin_width = 0.1;
        amcl_param.kld_sampling.z_bin_width = 0.1;
        amcl.setParam(amcl_param);
        // prediction_model_ = std::make_shared<>(); // todo
    }
    std::cout << "test" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ map_point(0, 1, 0);
    pc_map->push_back(map_point);pcl::PointXYZ map_point2(1, 0, 0);
    pc_map->push_back(map_point2);
    amcl.setMap(pc_map);

    Position position(0, 0, 0);
    Quat quat;
    quat = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
    PoseCovariance covariance;
    covariance(/*x*/ 0, /*x*/ 0) = 1.0;         // x var
    covariance(/*y*/ 1, /*y*/ 1) = 0.0;         // y var
    covariance(/*z*/ 2, /*z*/ 2) = 0.0;         // z var
    covariance(/*roll*/ 3, /*roll*/ 3) = 0.0;   // roll var
    covariance(/*pitch*/ 4, /*pitch*/ 4) = 0.0; // pitch var
    covariance(/*yaw*/ 5, /*yaw*/ 5) = 0.0;     // yaw var

    // set initial pose
    amcl.setInitialPose(position, quat, covariance);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_measurement(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ measurement_point(0, 1, 0);
    pc_measurement->push_back(measurement_point); pcl::PointXYZ measurement_point2(1, 0, 0);
    pc_measurement->push_back(measurement_point2);
    amcl.measureLidar(pc_measurement);
    return 0;
}