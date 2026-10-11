#!/usr/bin/env python3
"""Make Gazebo LiDAR scans FAST-LIO friendly (simulation only).

1. Drop non-finite points. gpu_lidar reports beams that hit nothing as +inf; FAST-LIO
   keeps them (inf passes its blind-distance check) and they break its voxel filter.
2. Add an exact per-point `time` field. Gazebo captures a whole scan at one instant but
   publishes no per-point time, so FAST-LIO would fall back to guessing times from the
   azimuth (a spinning-LiDAR heuristic) and can stall waiting for IMU data. Instead the
   header is moved back one scan period and every point gets time = scan period, i.e. all
   points sit exactly at the capture instant: correct timing, no undistortion error.
   Use with `preprocess.timestamp_unit: 0` (seconds) in the FAST-LIO config.
"""
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

OUT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'time', 'ring'],
    'formats': [np.float32, np.float32, np.float32, np.float32, np.float32, np.uint16],
    'offsets': [0, 4, 8, 12, 16, 20],
    'itemsize': 24,
})
OUT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
]


class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')
        self.declare_parameter('scan_period', 0.1)
        self.period = float(self.get_parameter('scan_period').value)
        self.pub = self.create_publisher(PointCloud2, '/lidar/points_clean', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/lidar/points', self._on_cloud, qos_profile_sensor_data)

    def _on_cloud(self, msg):
        pts = point_cloud2.read_points(msg)
        ok = np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
        src = pts[ok]
        out_pts = np.zeros(src.shape[0], dtype=OUT_DTYPE)
        for name in ('x', 'y', 'z', 'intensity'):
            out_pts[name] = src[name]
        out_pts['ring'] = src['ring'] if 'ring' in src.dtype.names else 0
        out_pts['time'] = self.period

        t_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec - int(self.period * 1e9)
        out = PointCloud2()
        out.header.frame_id = msg.header.frame_id
        out.header.stamp = Time(sec=t_ns // 10**9, nanosec=t_ns % 10**9)
        out.height = 1
        out.width = int(out_pts.shape[0])
        out.fields = OUT_FIELDS
        out.is_bigendian = False
        out.point_step = OUT_DTYPE.itemsize
        out.row_step = out.point_step * out.width
        out.is_dense = True
        out.data = out_pts.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = LidarFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
