# amcl_3d

[![Jazzy CI](https://github.com/rsasaki0109/amcl_3d/actions/workflows/jazzy-ci.yml/badge.svg)](https://github.com/rsasaki0109/amcl_3d/actions/workflows/jazzy-ci.yml)

ROS1 `amcl_3d` を ROS 2 Jazzy 向けに移植したワークスペースです。

`short_test.bag` を rosbag2 に変換して流した `odom` と AMCL の XY 比較結果は下です。

![short_test amcl vs odom xy](reports/assets/short_test_amcl_vs_odom_xy.png)

- `/odom` は初期 pose と初期 yaw で `map` 側にそろえて比較
- AMCL 軌跡長: `17.9 m`
- odom 軌跡長: `15.8 m`
- 平均 XY 差分: `0.22 m`
- XY RMSE: `0.25 m`
- 最大 XY 差分: `0.46 m`
- 終端 XY 差分: `0.32 m`
- 修正点: lidar measurement の座標変換順を `rotate -> translate` に修正
- 修正点: 予測更新は callback 実行時の sim time を使い、非単調な `dt` は無視
- パッケージの build / run 手順: [src/amcl_3d/README.md](src/amcl_3d/README.md)
- 比較図の再生成スクリプト: [reports/generate_short_test_trajectory.py](reports/generate_short_test_trajectory.py)
- ROS1 branch: https://github.com/rsasaki0109/amcl_3d/tree/ros1
