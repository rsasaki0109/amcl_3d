# amcl_3d for ROS 2

[![Jazzy CI](https://github.com/rsasaki0109/amcl_3d/actions/workflows/jazzy-ci.yml/badge.svg)](https://github.com/rsasaki0109/amcl_3d/actions/workflows/jazzy-ci.yml)

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

`amcl_3d.launch.py` は低レベルの基本 launch で、既定ではノード内部名そのままのトピックを使います。

```bash
ros2 launch amcl_3d amcl_3d.launch.py
```

この場合の既定値は次です。

- `map`
- `odom`
- `imu`
- `initialpose`
- `pc2`

### Custom Remap

主要な入出力トピックは launch 引数で差し替えられます。

```bash
ros2 launch amcl_3d amcl_3d.launch.py \
  input_map:=/mapcloud \
  input_odom:=/vehicle/odom \
  input_pc2:=/cloud \
  input_initialpose:=/initialpose
```

パラメータの既定値は [config/amcl_3d.params.yaml](config/amcl_3d.params.yaml) にあります。

## Demo With Rosbag2

元リポジトリのデモは ROS1 `.bag` 前提だったので、ROS2 側では rosbag2 を再生する launch を追加しています。

ROS1 bag を持っている場合は先に rosbag2 へ変換します。

```bash
cd ~/workspace/amcl_3d_ros2_ws/src/amcl_3d
./tools/convert_ros1_bag_to_ros2.sh /path/to/short_test.bag /path/to/short_test_ros2
```

その後、デモ launch を使います。

```bash
cd ~/workspace/amcl_3d_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch amcl_3d amcl_3d_rosbag.launch.py bag_path:=/path/to/short_test_ros2
```

このデモ launch の topic 既定値は次です。

- `input_map:=/mapcloud`
- `input_odom:=/odom`
- `input_imu:=/imu/data`
- `input_initialpose:=/initialpose`
- `input_pc2:=/cloud`
- `world_frame_id:=map`
- `publish_world_to_static_tf:=false`

主な引数は次の通りです。

- `open_rviz:=false` で RViz を起動しません
- `bag_rate:=0.5` で再生速度を変更できます
- `bag_loop:=false` でループ再生を止めます
- bag の frame 構成に追加で world 系 TF が必要なときだけ `publish_world_to_static_tf:=true` と `static_child_frame_id:=cad` を使います

headless 環境で起動確認だけしたい場合はこうします。

```bash
ros2 launch amcl_3d amcl_3d_rosbag.launch.py \
  bag_path:=/path/to/short_test_ros2 \
  open_rviz:=false
```

## Notes

- この移植では `ament_cmake` / `rclcpp` / ROS 2 launch に置き換えています。
- ROS2 用の `amcl_3d_rosbag.launch.py` を追加し、rosbag2 再生に寄せています。
- `map -> odom` 推定ではなく、元実装どおり `world -> base_link` の推定姿勢を publish します。
