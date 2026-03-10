# amcl_3d for ROS 2

ROS1版 `amcl_3d` を ROS 2 Jazzy 向けに移植した作業ツリーです。

## Build

```bash
cd ~/workspace/amcl_3d_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch amcl_3d amcl_3d.launch.py
```

主要な入出力トピックは launch 引数で差し替えられます。

```bash
ros2 launch amcl_3d amcl_3d.launch.py \
  input_map:=/mapcloud \
  input_odom:=/vehicle/odom \
  input_pc2:=/cloud \
  input_initialpose:=/initialpose
```

パラメータの既定値は [config/amcl_3d.params.yaml](config/amcl_3d.params.yaml) にあります。

## Notes

- この移植では `ament_cmake` / `rclcpp` / ROS 2 launch に置き換えています。
- 旧 `rosbag` デモ launch はそのままでは ROS 2 で使えないため削除しています。
- `map -> odom` 推定ではなく、元実装どおり `world -> base_link` の推定姿勢を publish します。
