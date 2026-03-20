# amcl_3d Plan

## Current Branch Policy

- `https://github.com/rsasaki0109/amcl_3d` を正本にする
- `amcl_3d` は public repo
- default branch は `main`
- `main` は ROS 2
- `ros2` も ROS 2 の退避 branch として残す
- `ros1` は旧 ROS1 系の branch として残す
- `devel` など既存の ROS1 branch は削除せず保持する

## Current Remote Policy

- `origin` = `https://github.com/rsasaki0109/amcl_3d.git` (整理済み)

## Current Local Workspaces

- `/home/sasaki/workspace/amcl_3d_ros2_ws`
  - ROS 2 の作業用 workspace
- `/home/sasaki/workspace/amcl_3d_ws/src/amcl_3d`
  - ROS1 元 repo の参照用 checkout

## Current ROS 2 State

- ROS2 側を `main` として公開済み
- README には `odom` と AMCL の XY 比較図を掲載済み
- lidar measurement の座標変換順を修正済み
- 予測更新は callback 実行時の sim time 基準に修正済み
- 非単調な `dt` は無視するように修正済み
- `reports/short_test_demo.md` は削除済み
- 比較結果は README に集約済み
- TF publish を `map -> odom` に変更済み (`world_frame_id` パラメータ削除)
- README タイトル・CI badge を `amcl_3d` に統一済み
- ブランチ説明を README に追加済み

## Next Candidates

(なし)
