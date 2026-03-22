# amcl_3d for ROS 2

[![Jazzy CI](https://github.com/rsasaki0109/amcl_3d/actions/workflows/jazzy-ci.yml/badge.svg)](https://github.com/rsasaki0109/amcl_3d/actions/workflows/jazzy-ci.yml)

3D AMCL (Adaptive Monte Carlo Localization) ported from ROS 1 to ROS 2 Jazzy.

See the top-level [README](../../README.md) for demo results.

## Build

```bash
cd ~/workspace/amcl_3d_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

`amcl_3d.launch.py` is the base launch file using internal topic names by default.

```bash
ros2 launch amcl_3d amcl_3d.launch.py
```

Default topic names:

- `map`
- `odom`
- `imu`
- `initialpose`
- `pc2`

### Custom Remap

Input topics can be remapped via launch arguments.

```bash
ros2 launch amcl_3d amcl_3d.launch.py \
  input_map:=/mapcloud \
  input_odom:=/vehicle/odom \
  input_pc2:=/cloud \
  input_initialpose:=/initialpose
```

Default parameters are in [config/amcl_3d.params.yaml](config/amcl_3d.params.yaml).

## Demo With Rosbag2

A rosbag2 playback launch is provided for demos.

Convert a ROS 1 bag first if needed:

```bash
cd ~/workspace/amcl_3d_ros2_ws/src/amcl_3d
./tools/convert_ros1_bag_to_ros2.sh /path/to/short_test.bag /path/to/short_test_ros2
```

Then run the demo launch:

```bash
cd ~/workspace/amcl_3d_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch amcl_3d amcl_3d_rosbag.launch.py bag_path:=/path/to/short_test_ros2
```

Default demo topic mappings:

- `input_map:=/mapcloud`
- `input_odom:=/odom`
- `input_imu:=/imu/data`
- `input_initialpose:=/initialpose`
- `input_pc2:=/cloud`
- `publish_map_to_static_tf:=false`

Additional arguments:

- `open_rviz:=false` to skip RViz
- `bag_rate:=0.5` to change playback speed
- `bag_loop:=false` to disable looping
- Use `publish_map_to_static_tf:=true` and `static_child_frame_id:=cad` only when the bag frame tree needs an extra static TF under the map frame

Headless smoke test:

```bash
ros2 launch amcl_3d amcl_3d_rosbag.launch.py \
  bag_path:=/path/to/short_test_ros2 \
  open_rviz:=false
```

### PCD map publisher

The rosbag launch can also load a PCD map file and publish it as a PointCloud2 topic
with `transient_local` QoS (matching amcl_3d's map subscription):

```bash
ros2 launch amcl_3d amcl_3d_rosbag.launch.py \
  bag_path:=/path/to/bag \
  map_pcd_file:=/path/to/map.pcd \
  map_pcd_frame:=map
```

### QoS overrides for bag playback

Some bags require QoS overrides (e.g., `/tf_static` with `transient_local` durability).
Pass a YAML file via `qos_overrides_path`:

```bash
ros2 launch amcl_3d amcl_3d_rosbag.launch.py \
  bag_path:=/path/to/bag \
  qos_overrides_path:=/path/to/qos_override.yaml
```

A default override file is included at `config/qos_override.yaml`.

## Demo: Kinematic-ICP Husky Bag

Full demo using a Kinematic-ICP Husky bag (2 min, Hesai lidar + wheel odom).

### Step 1: Generate 3D map with lidarslam_ros2

Terminal 1 — lidarslam:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros/slam_ros2_ws/install/setup.bash

PARAMS_FILE=$(ros2 pkg prefix lidarslam)/share/lidarslam/param/lidarslam.yaml

ros2 run scanmatcher scanmatcher_node --ros-args \
  -r /input_cloud:=/lidar_points \
  --params-file "$PARAMS_FILE" \
  -p use_sim_time:=true &

ros2 run graph_based_slam graph_based_slam_node --ros-args \
  --params-file "$PARAMS_FILE" \
  -p use_sim_time:=true &
```

Terminal 2 — bag playback:

```bash
ros2 bag play bags/2024-08-23-11-05-41_0_clipped.mcap \
  --clock 100 --rate 1.0 \
  --qos-profile-overrides-path src/amcl_3d/config/qos_override.yaml
```

After bag finishes, save the map:

```bash
ros2 service call /map_save std_srvs/srv/Empty
# Output: map.pcd in the current directory
mkdir -p maps && mv map.pcd maps/kinematic_icp_husky.pcd
```

### Step 2: Run amcl_3d with the generated map

One-liner using the rosbag launch:

```bash
ros2 launch amcl_3d amcl_3d_rosbag.launch.py \
  bag_path:=bags/2024-08-23-11-05-41_0_clipped.mcap \
  map_pcd_file:=$(pwd)/maps/kinematic_icp_husky.pcd \
  input_odom:=/husky_velocity_controller/odom \
  input_pc2:=/lidar_points \
  qos_overrides_path:=$(ros2 pkg prefix amcl_3d)/share/amcl_3d/config/qos_override.yaml \
  bag_rate:=0.5 \
  bag_loop:=true
```

In a separate terminal, set the initial pose:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: "map"},
    pose: {pose: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1}},
           covariance: [0.5,0,0,0,0,0, 0,0.5,0,0,0,0, 0,0,0.5,0,0,0,
                        0,0,0,0.1,0,0, 0,0,0,0,0.1,0, 0,0,0,0,0,0.1]}}'
```

### Verification

- `map → odom` TF: `ros2 run tf2_ros tf2_echo map odom`
- Particles: `ros2 topic echo /particles --no-arr`
- Current pose: `ros2 topic echo /current_pose`

## Notes

- Ported to `ament_cmake` / `rclcpp` / ROS 2 launch.
- Added `amcl_3d_rosbag.launch.py` for rosbag2 playback.
- Publishes `map -> odom` TF following the ROS nav convention (`odom -> base_link` is provided by the odometry source).
