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
                yaw=2.0 * math.atan2(qz, qw),
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


def load_map_points(bag_path: Path, topic: str) -> tuple[np.ndarray, str]:
    with AnyReader([bag_path]) as reader:
        target_connections = [connection for connection in reader.connections if connection.topic == topic]
        if not target_connections:
            raise ValueError(f"topic {topic} not found in {bag_path}")

        for connection, _, rawdata in reader.messages(connections=target_connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            return parse_pointcloud2_xy(msg), msg.header.frame_id

    raise ValueError(f"no messages found for topic {topic} in {bag_path}")


def build_plot(samples: list[PoseSample], output_path: Path, title: str, map_points: np.ndarray | None = None) -> None:
    t = np.array([sample.stamp for sample in samples], dtype=float)
    x = np.array([sample.x for sample in samples], dtype=float)
    y = np.array([sample.y for sample in samples], dtype=float)
    yaw = np.array([sample.yaw for sample in samples], dtype=float)
    elapsed = t - t[0]

    path_length = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a trajectory plot from /current_pose CSV output.")
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_png", type=Path)
    parser.add_argument("--title", default="amcl_3d ROS 2 trajectory")
    parser.add_argument("--bag-path", type=Path, default=None, help="Optional rosbag2 directory used to overlay /mapcloud.")
    parser.add_argument("--map-topic", default="/mapcloud")
    args = parser.parse_args()

    samples = parse_csv(args.input_csv)
    map_points = None
    if args.bag_path is not None:
        map_points, map_frame_id = load_map_points(args.bag_path, args.map_topic)
        print(f"map_points={len(map_points)}")
        print(f"map_frame_id={map_frame_id}")
    build_plot(samples, args.output_png, args.title, map_points)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
