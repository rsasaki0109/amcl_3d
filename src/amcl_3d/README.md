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

## Notes

- Ported to `ament_cmake` / `rclcpp` / ROS 2 launch.
- Added `amcl_3d_rosbag.launch.py` for rosbag2 playback.
- Publishes `map -> odom` TF following the ROS nav convention (`odom -> base_link` is provided by the odometry source).
