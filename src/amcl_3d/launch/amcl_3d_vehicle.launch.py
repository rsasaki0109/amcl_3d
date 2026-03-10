from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("amcl_3d")
    base_launch = PathJoinSubstitution([package_share, "launch", "amcl_3d.launch.py"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([package_share, "config", "amcl_3d.params.yaml"])),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("world_frame_id", default_value="world"),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            DeclareLaunchArgument("base_link_frame_id", default_value="base_link"),
            DeclareLaunchArgument("odom_frame_id", default_value="odom"),
            DeclareLaunchArgument("publish_rate", default_value="10.0"),
            DeclareLaunchArgument("input_map", default_value="/mapcloud"),
            DeclareLaunchArgument("input_odom", default_value="/vehicle/odom"),
            DeclareLaunchArgument("input_imu", default_value="/imu/data"),
            DeclareLaunchArgument("input_initialpose", default_value="/initialpose"),
            DeclareLaunchArgument("input_pc2", default_value="/cloud"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "world_frame_id": LaunchConfiguration("world_frame_id"),
                    "map_frame_id": LaunchConfiguration("map_frame_id"),
                    "base_link_frame_id": LaunchConfiguration("base_link_frame_id"),
                    "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                    "publish_rate": LaunchConfiguration("publish_rate"),
                    "input_map": LaunchConfiguration("input_map"),
                    "input_odom": LaunchConfiguration("input_odom"),
                    "input_imu": LaunchConfiguration("input_imu"),
                    "input_initialpose": LaunchConfiguration("input_initialpose"),
                    "input_pc2": LaunchConfiguration("input_pc2"),
                }.items(),
            ),
        ]
    )
