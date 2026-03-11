#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib
from rosbags.highlevel import AnyReader

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


@dataclass
class PoseSample:
    stamp: float
    frame_id: str
    x: float
    y: float
    z: float
    yaw: float


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_csv(csv_path: Path) -> list[PoseSample]:
    samples: list[PoseSample] = []
    for line in csv_path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split(",")
        if len(parts) != 10:
            continue

        try:
            sec = int(parts[0])
            nanosec = int(parts[1])
            x = float(parts[3])
            y = float(parts[4])
            z = float(parts[5])
            qx = float(parts[6])
            qy = float(parts[7])
            qz = float(parts[8])
            qw = float(parts[9])
        except ValueError:
            continue

        samples.append(
            PoseSample(
                stamp=sec + nanosec * 1e-9,
                frame_id=parts[2],
                x=x,
                y=y,
                z=z,
                yaw=yaw_from_quaternion(qx, qy, qz, qw),
            )
        )

    if not samples:
        raise ValueError(f"no pose samples parsed from {csv_path}")

    return samples


def parse_pointcloud2_xy(points_msg: object) -> np.ndarray:
    fields = {field.name: field for field in points_msg.fields}
    if "x" not in fields or "y" not in fields:
        raise ValueError("pointcloud must contain x and y fields")

    endian_prefix = ">" if points_msg.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [f"{endian_prefix}f4", f"{endian_prefix}f4", f"{endian_prefix}f4"],
            "offsets": [
                fields["x"].offset,
                fields["y"].offset,
                fields.get("z", fields["y"]).offset if "z" in fields else fields["y"].offset,
            ],
            "itemsize": points_msg.point_step,
        }
    )
    structured = np.frombuffer(bytes(points_msg.data), dtype=dtype, count=points_msg.width * points_msg.height)
    points = np.column_stack((structured["x"], structured["y"], structured["z"]))
    return points[np.isfinite(points).all(axis=1)]


def load_pose_samples_from_bag(bag_path: Path, topic: str) -> list[PoseSample]:
    samples: list[PoseSample] = []
    with AnyReader([bag_path]) as reader:
        target_connections = [connection for connection in reader.connections if connection.topic == topic]
        if not target_connections:
            raise ValueError(f"topic {topic} not found in {bag_path}")

        for connection, _, rawdata in reader.messages(connections=target_connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.msgtype == "nav_msgs/msg/Odometry":
                pose = msg.pose.pose
                header = msg.header
            elif connection.msgtype == "geometry_msgs/msg/PoseStamped":
                pose = msg.pose
                header = msg.header
            else:
                raise ValueError(f"unsupported pose message type: {connection.msgtype}")

            samples.append(
                PoseSample(
                    stamp=header.stamp.sec + header.stamp.nanosec * 1e-9,
                    frame_id=header.frame_id,
                    x=pose.position.x,
                    y=pose.position.y,
                    z=pose.position.z,
                    yaw=yaw_from_quaternion(
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ),
                )
            )

    if not samples:
        raise ValueError(f"no pose samples found for topic {topic} in {bag_path}")

    return samples


def load_map_points(bag_path: Path, topic: str) -> tuple[np.ndarray, str]:
    with AnyReader([bag_path]) as reader:
        target_connections = [connection for connection in reader.connections if connection.topic == topic]
        if not target_connections:
            raise ValueError(f"topic {topic} not found in {bag_path}")

        for connection, _, rawdata in reader.messages(connections=target_connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            return parse_pointcloud2_xy(msg), msg.header.frame_id

    raise ValueError(f"no messages found for topic {topic} in {bag_path}")


def trajectory_arrays(samples: list[PoseSample]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    return (
        np.array([sample.stamp for sample in samples], dtype=float),
        np.array([sample.x for sample in samples], dtype=float),
        np.array([sample.y for sample in samples], dtype=float),
        np.array([sample.yaw for sample in samples], dtype=float),
    )


def compute_path_length(x: np.ndarray, y: np.ndarray) -> float:
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))


def align_samples_to_reference(samples: list[PoseSample], reference_sample: PoseSample) -> list[PoseSample]:
    origin = samples[0]
    yaw_delta = reference_sample.yaw - origin.yaw
    cos_yaw = math.cos(yaw_delta)
    sin_yaw = math.sin(yaw_delta)

    aligned_samples: list[PoseSample] = []
    for sample in samples:
        dx = sample.x - origin.x
        dy = sample.y - origin.y
        aligned_samples.append(
            PoseSample(
                stamp=sample.stamp,
                frame_id=reference_sample.frame_id,
                x=reference_sample.x + cos_yaw * dx - sin_yaw * dy,
                y=reference_sample.y + sin_yaw * dx + cos_yaw * dy,
                z=sample.z,
                yaw=sample.yaw + yaw_delta,
            )
        )

    return aligned_samples


def compute_xy_error_stats(reference_samples: list[PoseSample], compared_samples: list[PoseSample]) -> dict[str, float]:
    ref_t, ref_x, ref_y, _ = trajectory_arrays(reference_samples)
    cmp_t, cmp_x, cmp_y, _ = trajectory_arrays(compared_samples)

    start_time = max(ref_t[0], cmp_t[0])
    end_time = min(ref_t[-1], cmp_t[-1])
    mask = (ref_t >= start_time) & (ref_t <= end_time)
    if not np.any(mask):
        raise ValueError("no overlapping timestamps between compared trajectories")

    ref_t = ref_t[mask]
    ref_x = ref_x[mask]
    ref_y = ref_y[mask]

    cmp_t, unique_idx = np.unique(cmp_t, return_index=True)
    cmp_x = cmp_x[unique_idx]
    cmp_y = cmp_y[unique_idx]

    interp_x = np.interp(ref_t, cmp_t, cmp_x)
    interp_y = np.interp(ref_t, cmp_t, cmp_y)
    delta = np.hypot(ref_x - interp_x, ref_y - interp_y)

    return {
        "sample_count": float(len(delta)),
        "mean_xy_error_m": float(np.mean(delta)),
        "rmse_xy_error_m": float(np.sqrt(np.mean(delta ** 2))),
        "max_xy_error_m": float(np.max(delta)),
        "end_xy_error_m": float(delta[-1]),
    }


def build_plot(samples: list[PoseSample], output_path: Path, title: str, map_points: np.ndarray | None = None) -> None:
    t, x, y, yaw = trajectory_arrays(samples)
    elapsed = t - t[0]

    path_length = compute_path_length(x, y)
    frame_id = samples[0].frame_id

    fig, ax = plt.subplots(figsize=(8.5, 8.0), constrained_layout=True)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#f8fafc")
    ax.grid(True, color="#cbd5e1", linewidth=0.8, alpha=0.7)

    if map_points is not None and len(map_points) > 0:
        stride = max(1, len(map_points) // 18000)
        map_xy = map_points[::stride, :2]
        ax.scatter(
            map_xy[:, 0],
            map_xy[:, 1],
            s=2.0,
            color="#cbd5e1",
            alpha=0.55,
            linewidths=0,
            label="mapcloud",
            zorder=0,
        )

    ax.plot(x, y, color="#94a3b8", linewidth=1.4, alpha=0.9, zorder=1)
    scatter = ax.scatter(
        x,
        y,
        c=elapsed,
        s=14,
        cmap="viridis",
        linewidths=0,
        alpha=0.95,
        zorder=2,
    )

    ax.scatter(x[0], y[0], s=110, color="#16a34a", edgecolors="white", linewidths=0.9, label="start", zorder=3)
    ax.scatter(x[-1], y[-1], s=130, marker="X", color="#dc2626", edgecolors="white", linewidths=0.9, label="end", zorder=3)

    arrow_step = max(1, len(samples) // 24)
    ax.quiver(
        x[::arrow_step],
        y[::arrow_step],
        np.cos(yaw[::arrow_step]),
        np.sin(yaw[::arrow_step]),
        angles="xy",
        scale_units="xy",
        scale=8.5,
        width=0.0024,
        color="#0f172a",
        alpha=0.35,
        zorder=2,
    )

    stats_text = "\n".join(
        [
            f"samples   : {len(samples)}",
            f"duration  : {elapsed[-1]:.1f} s",
            f"path len  : {path_length:.1f} m",
            f"frame_id  : {frame_id}",
        ]
    )
    ax.text(
        0.02,
        0.98,
        stats_text,
        transform=ax.transAxes,
        ha="left",
        va="top",
        family="monospace",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "white", "edgecolor": "#cbd5e1", "alpha": 0.95},
    )

    ax.set_title(title, fontsize=15)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.legend(loc="lower left", frameon=True)

    colorbar = fig.colorbar(scatter, ax=ax, shrink=0.88)
    colorbar.set_label("elapsed time [s]")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)

    print(f"samples={len(samples)}")
    print(f"duration_sec={elapsed[-1]:.6f}")
    print(f"path_length_m={path_length:.6f}")
    print(f"frame_id={frame_id}")
    print(f"output={output_path}")


def build_comparison_plot(
    amcl_samples: list[PoseSample],
    odom_samples: list[PoseSample],
    output_path: Path,
    title: str,
    map_points: np.ndarray | None = None,
) -> None:
    amcl_t, amcl_x, amcl_y, _ = trajectory_arrays(amcl_samples)
    odom_t, odom_x, odom_y, _ = trajectory_arrays(odom_samples)
    amcl_elapsed = amcl_t - amcl_t[0]
    amcl_path_length = compute_path_length(amcl_x, amcl_y)
    odom_path_length = compute_path_length(odom_x, odom_y)
    error_stats = compute_xy_error_stats(amcl_samples, odom_samples)

    fig, ax = plt.subplots(figsize=(8.8, 8.0), constrained_layout=True)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("#f8fafc")
    ax.grid(True, color="#cbd5e1", linewidth=0.8, alpha=0.7)

    if map_points is not None and len(map_points) > 0:
        stride = max(1, len(map_points) // 18000)
        map_xy = map_points[::stride, :2]
        ax.scatter(
            map_xy[:, 0],
            map_xy[:, 1],
            s=2.0,
            color="#cbd5e1",
            alpha=0.55,
            linewidths=0,
            label="mapcloud",
            zorder=0,
        )

    ax.plot(odom_x, odom_y, color="#f97316", linewidth=1.5, alpha=0.9, label="odom aligned", zorder=1)
    amcl_scatter = ax.scatter(
        amcl_x,
        amcl_y,
        c=amcl_elapsed,
        s=14,
        cmap="viridis",
        linewidths=0,
        alpha=0.95,
        label="amcl current_pose",
        zorder=3,
    )
    ax.plot(amcl_x, amcl_y, color="#2563eb", linewidth=1.2, alpha=0.9, zorder=2)

    ax.scatter(amcl_x[0], amcl_y[0], s=110, color="#16a34a", edgecolors="white", linewidths=0.9, label="shared start", zorder=4)
    ax.scatter(amcl_x[-1], amcl_y[-1], s=130, marker="X", color="#1d4ed8", edgecolors="white", linewidths=0.9, label="amcl end", zorder=4)
    ax.scatter(odom_x[-1], odom_y[-1], s=100, marker="D", color="#ea580c", edgecolors="white", linewidths=0.9, label="odom end", zorder=4)

    stats_text = "\n".join(
        [
            f"amcl len   : {amcl_path_length:.1f} m",
            f"odom len   : {odom_path_length:.1f} m",
            f"mean err   : {error_stats['mean_xy_error_m']:.2f} m",
            f"rmse err   : {error_stats['rmse_xy_error_m']:.2f} m",
            f"max err    : {error_stats['max_xy_error_m']:.2f} m",
            f"end err    : {error_stats['end_xy_error_m']:.2f} m",
        ]
    )
    ax.text(
        0.02,
        0.98,
        stats_text,
        transform=ax.transAxes,
        ha="left",
        va="top",
        family="monospace",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "white", "edgecolor": "#cbd5e1", "alpha": 0.95},
    )

    ax.set_title(title, fontsize=15)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.legend(loc="lower left", frameon=True)

    colorbar = fig.colorbar(amcl_scatter, ax=ax, shrink=0.88)
    colorbar.set_label("AMCL elapsed time [s]")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)

    print(f"compare_samples={int(error_stats['sample_count'])}")
    print(f"compare_mean_xy_error_m={error_stats['mean_xy_error_m']:.6f}")
    print(f"compare_rmse_xy_error_m={error_stats['rmse_xy_error_m']:.6f}")
    print(f"compare_max_xy_error_m={error_stats['max_xy_error_m']:.6f}")
    print(f"compare_end_xy_error_m={error_stats['end_xy_error_m']:.6f}")
    print(f"compare_output={output_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate trajectory plots from /current_pose CSV output.")
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("--trajectory-output-png", type=Path, default=None)
    parser.add_argument("--title", default="amcl_3d ROS 2 trajectory")
    parser.add_argument("--bag-path", type=Path, default=None, help="Optional rosbag2 directory used to overlay /mapcloud.")
    parser.add_argument("--map-topic", default="/mapcloud")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--compare-odom-output-png", type=Path, default=None)
    parser.add_argument("--compare-title", default="amcl_3d ROS 2 vs odom XY")
    args = parser.parse_args()

    if args.trajectory_output_png is None and args.compare_odom_output_png is None:
        raise ValueError("either --trajectory-output-png or --compare-odom-output-png must be set")

    samples = parse_csv(args.input_csv)
    map_points = None
    if args.bag_path is not None:
        map_points, map_frame_id = load_map_points(args.bag_path, args.map_topic)
        print(f"map_points={len(map_points)}")
        print(f"map_frame_id={map_frame_id}")

    if args.trajectory_output_png is not None:
        build_plot(samples, args.trajectory_output_png, args.title, map_points)

    if args.compare_odom_output_png is not None:
        if args.bag_path is None:
            raise ValueError("--compare-odom-output-png requires --bag-path")
        odom_samples = load_pose_samples_from_bag(args.bag_path, args.odom_topic)
        aligned_odom_samples = align_samples_to_reference(odom_samples, samples[0])
        build_comparison_plot(samples, aligned_odom_samples, args.compare_odom_output_png, args.compare_title, map_points)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
