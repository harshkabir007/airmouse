#!/usr/bin/env python3
"""Live 2D occupancy map from the 3D SLAM scans.

Each scan (already in the map frame) is sliced to a height band above the floor
(walls, standing people; not floor, not the ceiling net). In-band returns mark
cells occupied; the nearest return per bearing is ray-traced to mark free space.
Log-odds keep the map stable against noise. Published as nav_msgs/OccupancyGrid.
"""
import math
from array import array

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from airmouse.arena import declare_arena
from airmouse.params import declare


class Mapper(Node):
    def __init__(self):
        super().__init__('mapper')
        self.arena = declare_arena(self)
        self.map_frame = declare(self, 'map_frame', 'map')
        self.res = float(declare(self, 'resolution', 0.05))
        margin = float(declare(self, 'margin', 2.0))
        self.floor_z = float(declare(self, 'floor_z', -0.2))
        self.band_min = float(declare(self, 'band_min', 0.35))
        self.band_max = float(declare(self, 'band_max', 2.1))
        self.min_r = float(declare(self, 'min_range', 0.4))
        self.max_r = float(declare(self, 'max_range', 10.0))
        self.bins = int(declare(self, 'angular_bins', 720))
        self.l_hit = float(declare(self, 'log_odds_hit', 0.85))
        self.l_miss = float(declare(self, 'log_odds_miss', -0.4))
        self.l_min, self.l_max = -2.0, 3.5
        self.occ_t = float(declare(self, 'occupied_threshold', 0.6))
        self.free_t = float(declare(self, 'free_threshold', -0.4))
        rate = float(declare(self, 'publish_rate', 2.0))

        # Map covers the arena plus the start/exit point outside it.
        xmin, xmax, ymin, ymax = self.arena.bounds()
        xmin, xmax = min(xmin, 0.0) - margin, max(xmax, 0.0) + margin
        ymin, ymax = min(ymin, 0.0) - margin, max(ymax, 0.0) + margin
        self.ox, self.oy = xmin, ymin
        self.nx = int(math.ceil((xmax - xmin) / self.res))
        self.ny = int(math.ceil((ymax - ymin) / self.res))
        self.L = np.zeros((self.ny, self.nx), np.float32)
        self.pose = None

        self.pub = self.create_publisher(OccupancyGrid, '/airmouse/map', 1)
        self.create_subscription(Odometry, '/airmouse/odom', self._on_odom, 20)
        self.create_subscription(PointCloud2, '/airmouse/cloud', self._on_cloud, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f'map {self.nx}x{self.ny} cells @ {self.res} m')

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def _index(self, x, y):
        ix = np.floor((x - self.ox) / self.res).astype(np.int64)
        iy = np.floor((y - self.oy) / self.res).astype(np.int64)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return np.unique(iy[ok] * self.nx + ix[ok])

    def _on_cloud(self, msg):
        if self.pose is None:
            return
        px, py, _ = self.pose
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            return
        z = pts[:, 2]
        band = (z > self.floor_z + self.band_min) & (z < self.floor_z + self.band_max)
        dx = pts[band, 0] - px
        dy = pts[band, 1] - py
        r = np.hypot(dx, dy)
        keep = (r > self.min_r) & (r < self.max_r)
        dx, dy, r = dx[keep], dy[keep], r[keep]
        if r.size == 0:
            return

        hits = self._index(px + dx, py + dy)

        # Free space: ray-trace to the nearest in-band return of each bearing bin.
        b = ((np.arctan2(dy, dx) + math.pi) / (2 * math.pi) * self.bins).astype(np.int64) % self.bins
        order = np.argsort(r)
        _, first = np.unique(b[order], return_index=True)
        sel = order[first]
        rr = r[sel]
        ca, sa = dx[sel] / rr, dy[sel] / rr
        t = np.arange(0.0, rr.max(), self.res * 0.5)
        along = t[None, :] < (rr[:, None] - self.res)
        fx = (px + ca[:, None] * t[None, :])[along]
        fy = (py + sa[:, None] * t[None, :])[along]
        free = np.setdiff1d(self._index(fx, fy), hits, assume_unique=True)

        flat = self.L.reshape(-1)
        flat[free] += self.l_miss
        flat[hits] += self.l_hit
        np.clip(self.L, self.l_min, self.l_max, out=self.L)

    def _publish(self):
        data = np.full(self.L.shape, -1, np.int8)
        data[self.L >= self.occ_t] = 100
        data[self.L <= self.free_t] = 0
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.res
        msg.info.width = self.nx
        msg.info.height = self.ny
        msg.info.origin.position.x = self.ox
        msg.info.origin.position.y = self.oy
        msg.info.origin.orientation.w = 1.0
        msg.data = array('b', data.tobytes())
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
