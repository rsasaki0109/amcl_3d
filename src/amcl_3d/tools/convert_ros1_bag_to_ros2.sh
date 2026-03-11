#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <src_ros1_bag> <dst_ros2_bag_dir>" >&2
  exit 1
fi

if ! command -v rosbags-convert >/dev/null 2>&1; then
  echo "rosbags-convert not found. Install it with: pip install --user rosbags" >&2
  exit 1
fi

src_bag="$1"
dst_bag="$2"

if [[ ! -e "$src_bag" ]]; then
  echo "Source bag does not exist: $src_bag" >&2
  exit 1
fi

if [[ -e "$dst_bag" ]]; then
  echo "Destination already exists: $dst_bag" >&2
  exit 1
fi

rosbags-convert --src "$src_bag" --dst "$dst_bag"

echo "Converted ROS1 bag to ROS2 bag: $dst_bag"
echo "Inspect it with: ros2 bag info $dst_bag"
