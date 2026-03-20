from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _bag_play_action(context):
    bag_path = LaunchConfiguration("bag_path").perform(context).strip()
    if not bag_path:
        return [LogInfo(msg="bag_path is empty, skipping ros2 bag play")]

    cmd = [
        "ros2",
        "bag",
        "play",
        bag_path,
        "--clock",
        LaunchConfiguration("clock_publish_hz").perform(context),
        "--rate",
        LaunchConfiguration("bag_rate").perform(context),
        "--start-offset",
        LaunchConfiguration("bag_start_offset").perform(context),
    ]

    if LaunchConfiguration("bag_loop").perform(context).lower() == "true":
        cmd.append("--loop")
    if LaunchConfiguration("bag_start_paused").perform(context).lower() == "true":
        cmd.append("--start-paused")

    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    package_share = FindPackageShare("amcl_3d")
    rviz_config = PathJoinSubstitution([package_share, "rviz", "ros2_demo.rviz"])
    amcl_launch = PathJoinSubstitution([package_share, "launch", "amcl_3d.launch.py"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([package_share, "config", "amcl_3d.params.yaml"])),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map_frame_id", default_value="map"),
            DeclareLaunchArgument("base_link_frame_id", default_value="base_link"),
            DeclareLaunchArgument("odom_frame_id", default_value="odom"),
            DeclareLaunchArgument("publish_rate", default_value="10.0"),
            DeclareLaunchArgument("input_map", default_value="/mapcloud"),
            DeclareLaunchArgument("input_odom", default_value="/odom"),
            DeclareLaunchArgument("input_imu", default_value="/imu/data"),
            DeclareLaunchArgument("input_initialpose", default_value="/initialpose"),
            DeclareLaunchArgument("input_pc2", default_value="/cloud"),
            DeclareLaunchArgument("bag_path", default_value=""),
            DeclareLaunchArgument("bag_rate", default_value="0.5"),
            DeclareLaunchArgument("bag_loop", default_value="true"),
            DeclareLaunchArgument("bag_start_paused", default_value="false"),
            DeclareLaunchArgument("bag_start_offset", default_value="0.0"),
            DeclareLaunchArgument("clock_publish_hz", default_value="100.0"),
            DeclareLaunchArgument("open_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_config),
            DeclareLaunchArgument("publish_map_to_static_tf", default_value="false"),
            DeclareLaunchArgument("static_child_frame_id", default_value="cad"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(amcl_launch),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
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
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_static_frame_publisher",
                condition=IfCondition(LaunchConfiguration("publish_map_to_static_tf")),
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    LaunchConfiguration("map_frame_id"),
                    "--child-frame-id",
                    LaunchConfiguration("static_child_frame_id"),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(LaunchConfiguration("open_rviz")),
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                output="screen",
            ),
            OpaqueFunction(function=_bag_play_action),
        ]
    )
