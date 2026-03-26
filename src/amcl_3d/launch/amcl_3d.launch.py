from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("amcl_3d")
    default_params = PathJoinSubstitution([package_share, "config", "amcl_3d.params.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            DeclareLaunchArgument("base_link_frame_id", default_value="base_link"),
            DeclareLaunchArgument("odom_frame_id", default_value="odom"),
            DeclareLaunchArgument("pc2_frame_id_filter", default_value=""),
            DeclareLaunchArgument("publish_rate", default_value="10.0"),
            DeclareLaunchArgument("input_map", default_value="map"),
            DeclareLaunchArgument("input_odom", default_value="odom"),
            DeclareLaunchArgument("input_imu", default_value="imu"),
            DeclareLaunchArgument("input_initialpose", default_value="initialpose"),
            DeclareLaunchArgument("input_pc2", default_value="pc2"),
            Node(
                package="amcl_3d",
                executable="amcl_3d_node",
                name="amcl_3d",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "map_frame_id": LaunchConfiguration("map_frame_id"),
                        "base_link_frame_id": LaunchConfiguration("base_link_frame_id"),
                        "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                        "pc2_frame_id_filter": LaunchConfiguration("pc2_frame_id_filter"),
                        "publish_rate": LaunchConfiguration("publish_rate"),
                    },
                ],
                remappings=[
                    ("map", LaunchConfiguration("input_map")),
                    ("odom", LaunchConfiguration("input_odom")),
                    ("imu", LaunchConfiguration("input_imu")),
                    ("initialpose", LaunchConfiguration("input_initialpose")),
                    ("pc2", LaunchConfiguration("input_pc2")),
                ],
            ),
        ]
    )
