# short_test rosbag2 odom vs AMCL report

`amcl_3d_rosbag.launch.py` で `short_test_ros2` を再生し、`/current_pose` と bag 内の `/odom` を XY で比較した結果です。

## 実行コマンド

```bash
cd ~/workspace/amcl_3d_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

timeout 90s ros2 topic echo /current_pose geometry_msgs/msg/PoseStamped --csv \
  --qos-durability transient_local \
  --qos-reliability reliable \
  > /tmp/short_test_current_pose_full.csv &

timeout 90s ros2 launch amcl_3d amcl_3d_rosbag.launch.py \
  bag_path:=~/workspace/amcl_3d_ros2_ws/demo_data/short_test_ros2 \
  open_rviz:=false \
  bag_loop:=false \
  bag_rate:=4.0

python3 reports/generate_short_test_trajectory.py \
  /tmp/short_test_current_pose_full.csv \
  --bag-path ~/workspace/amcl_3d_ros2_ws/demo_data/short_test_ros2 \
  --compare-odom-output-png reports/assets/short_test_amcl_vs_odom_xy.png \
  --compare-title "amcl_3d ROS 2 vs odom XY (short_test_ros2)"
```

## 結果

- `/odom` は初期 pose と初期 yaw で `map` 側に合わせてから比較
- AMCL 軌跡長: `28.7 m`
- odom 軌跡長: `15.8 m`
- 平均 XY 差分: `0.29 m`
- XY RMSE: `0.35 m`
- 最大 XY 差分: `1.12 m`
- 終端 XY 差分: `0.20 m`

![amcl vs odom xy](assets/short_test_amcl_vs_odom_xy.png)

## メモ

- ずれが大きかった主因は lidar measurement 側の座標変換順で、`translate -> rotate` になっていた点です。`rotate -> translate` に修正すると、`odom` との XY 差分は大きく改善しました。
- 再生直後の数フレームは TF がまだ揃っておらず、`hokuyo3d_front` / `hokuyo3d_rear` に対する外挿警告が出ますが、その後 `received map` まで進み、`/current_pose` は継続して publish されます。
- `demo_data/` 配下の bag はサイズが大きいので git には含めていません。
- `reports/generate_short_test_trajectory.py` は `--compare-odom-output-png` を使うと、`/odom` を初期 pose / yaw で AMCL 側にそろえて比較図を出します。
