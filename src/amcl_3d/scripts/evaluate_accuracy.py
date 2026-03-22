#!/usr/bin/env python3
"""
Record current_pose (AMCL) and odom, then plot trajectory comparison.

Usage:
1. Launch amcl_3d with a bag
2. In another terminal: python3 evaluate_accuracy.py
3. Press Ctrl+C when done -- plots are saved to the working directory.
"""

import sys
import signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityType, ReliabilityType
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib is required: pip install matplotlib", file=sys.stderr)
    sys.exit(1)


class AccuracyRecorder(Node):
    def __init__(self):
        super().__init__("accuracy_recorder")
        self.amcl_times = []
        self.amcl_xy = []
        self.odom_times = []
        self.odom_xy = []

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityType.RELIABLE,
            durability=DurabilityType.TRANSIENT_LOCAL,
        )
        self.create_subscription(PoseStamped, "/current_pose", self._amcl_cb, latched)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.get_logger().info("Recording /current_pose and /odom ...")

    def _amcl_cb(self, msg: PoseStamped):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.amcl_times.append(t)
        self.amcl_xy.append((msg.pose.position.x, msg.pose.position.y))

    def _odom_cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_times.append(t)
        self.odom_xy.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def plot(self, out_dir: str = "."):
        if not self.amcl_xy and not self.odom_xy:
            self.get_logger().warn("No data recorded — nothing to plot.")
            return

        amcl = np.array(self.amcl_xy) if self.amcl_xy else np.empty((0, 2))
        odom = np.array(self.odom_xy) if self.odom_xy else np.empty((0, 2))

        # --- 1. Trajectory comparison ---
        fig1, ax1 = plt.subplots(figsize=(10, 8))
        if odom.size:
            ax1.plot(odom[:, 0], odom[:, 1], "b-", alpha=0.5, linewidth=1, label="Odom")
        if amcl.size:
            ax1.plot(amcl[:, 0], amcl[:, 1], "r-", linewidth=1.5, label="AMCL")
        ax1.set_xlabel("X [m]")
        ax1.set_ylabel("Y [m]")
        ax1.set_title("Trajectory: AMCL vs Odometry")
        ax1.legend()
        ax1.set_aspect("equal")
        ax1.grid(True, alpha=0.3)
        path1 = f"{out_dir}/trajectory_comparison.png"
        fig1.savefig(path1, dpi=150, bbox_inches="tight")
        self.get_logger().info(f"Saved {path1}")

        # --- 2. Position difference over time ---
        if amcl.size and odom.size:
            amcl_t = np.array(self.amcl_times)
            odom_t = np.array(self.odom_times)
            # Interpolate odom to amcl timestamps
            odom_x_interp = np.interp(amcl_t, odom_t, odom[:, 0])
            odom_y_interp = np.interp(amcl_t, odom_t, odom[:, 1])
            diff = np.sqrt((amcl[:, 0] - odom_x_interp) ** 2 + (amcl[:, 1] - odom_y_interp) ** 2)
            t_rel = amcl_t - amcl_t[0]

            fig2, ax2 = plt.subplots(figsize=(10, 4))
            ax2.plot(t_rel, diff, "g-", linewidth=1)
            ax2.set_xlabel("Time [s]")
            ax2.set_ylabel("Position difference [m]")
            ax2.set_title("AMCL vs Odom — Position Difference")
            ax2.grid(True, alpha=0.3)
            path2 = f"{out_dir}/position_difference.png"
            fig2.savefig(path2, dpi=150, bbox_inches="tight")
            self.get_logger().info(f"Saved {path2}")

        plt.close("all")
        n_amcl = len(self.amcl_xy)
        n_odom = len(self.odom_xy)
        self.get_logger().info(f"Recorded {n_amcl} AMCL, {n_odom} odom samples")


def main():
    rclpy.init()
    node = AccuracyRecorder()

    def shutdown_handler(*_):
        node.get_logger().info("Shutting down — generating plots ...")
        node.plot()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        rclpy.spin(node)
    except Exception:
        shutdown_handler()


if __name__ == "__main__":
    main()
