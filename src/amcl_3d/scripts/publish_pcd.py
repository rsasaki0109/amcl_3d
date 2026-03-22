#!/usr/bin/env python3
"""Publish a PCD file as a PointCloud2 message with transient_local QoS."""

import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def load_pcd(path: str) -> PointCloud2:
    """Load an ASCII PCD file and return a PointCloud2 message."""
    with open(path) as f:
        lines = f.readlines()

    header_end = 0
    fields_str = ""
    sizes_str = ""
    types_str = ""
    counts_str = ""
    width = 0
    height = 1
    data_format = ""

    for i, line in enumerate(lines):
        parts = line.strip().split()
        if not parts:
            continue
        key = parts[0]
        if key == "FIELDS":
            fields_str = parts[1:]
        elif key == "SIZE":
            sizes_str = parts[1:]
        elif key == "TYPE":
            types_str = parts[1:]
        elif key == "COUNT":
            counts_str = parts[1:]
        elif key == "WIDTH":
            width = int(parts[1])
        elif key == "HEIGHT":
            height = int(parts[1])
        elif key == "DATA":
            data_format = parts[1]
            header_end = i + 1
            break

    if data_format != "ascii":
        raise ValueError(f"Only ASCII PCD supported, got: {data_format}")

    # Build PointField list
    pf_list = []
    offset = 0
    for name, size, dtype, count in zip(fields_str, sizes_str, types_str, counts_str):
        size_val = int(size)
        count_val = int(count)
        if dtype == "F" and size_val == 4:
            datatype = PointField.FLOAT32
        elif dtype == "F" and size_val == 8:
            datatype = PointField.FLOAT64
        elif dtype == "U" and size_val == 4:
            datatype = PointField.UINT32
        elif dtype == "U" and size_val == 2:
            datatype = PointField.UINT16
        elif dtype == "U" and size_val == 1:
            datatype = PointField.UINT8
        elif dtype == "I" and size_val == 4:
            datatype = PointField.INT32
        elif dtype == "I" and size_val == 2:
            datatype = PointField.INT16
        elif dtype == "I" and size_val == 1:
            datatype = PointField.INT8
        else:
            datatype = PointField.FLOAT32
        pf_list.append(PointField(name=name, offset=offset, datatype=datatype, count=count_val))
        offset += size_val * count_val

    point_step = offset
    data_lines = lines[header_end:]
    num_points = min(len(data_lines), width * height)

    # Pack binary data
    fmt_map = {
        PointField.FLOAT32: "f",
        PointField.FLOAT64: "d",
        PointField.UINT8: "B",
        PointField.UINT16: "H",
        PointField.UINT32: "I",
        PointField.INT8: "b",
        PointField.INT16: "h",
        PointField.INT32: "i",
    }
    pack_fmt = "<"
    for pf in pf_list:
        pack_fmt += fmt_map.get(pf.datatype, "f") * pf.count

    data = bytearray()
    actual_points = 0
    for line in data_lines:
        vals = line.strip().split()
        if not vals:
            continue
        packed_vals = []
        for pf, val_str in zip(pf_list, vals):
            if pf.datatype in (PointField.FLOAT32, PointField.FLOAT64):
                packed_vals.append(float(val_str))
            else:
                packed_vals.append(int(float(val_str)))
        data.extend(struct.pack(pack_fmt, *packed_vals))
        actual_points += 1

    msg = PointCloud2()
    msg.height = 1
    msg.width = actual_points
    msg.fields = pf_list
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * actual_points
    msg.data = bytes(data)
    msg.is_dense = True
    return msg


class PcdPublisher(Node):
    def __init__(self, pcd_path: str, topic: str, frame_id: str):
        super().__init__("pcd_publisher")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PointCloud2, topic, qos)
        self.get_logger().info(f"Loading PCD: {pcd_path}")
        self.msg = load_pcd(pcd_path)
        self.msg.header.frame_id = frame_id
        self.get_logger().info(f"Loaded {self.msg.width} points, publishing on '{topic}' (frame: {frame_id})")
        self.pub.publish(self.msg)
        # Re-publish periodically for late subscribers
        self.timer = self.create_timer(5.0, self._republish)

    def _republish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    pcd_path = sys.argv[1] if len(sys.argv) > 1 else "map.pcd"
    topic = sys.argv[2] if len(sys.argv) > 2 else "/mapcloud"
    frame_id = sys.argv[3] if len(sys.argv) > 3 else "map"
    node = PcdPublisher(pcd_path, topic, frame_id)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
