#!/usr/bin/env python3
"""Detect survivors in the camera image and place them on the map / rescue grid.

Localisation per detection:
  1. Project the current LiDAR scan (map frame) into the image; take the nearest
     cluster of points inside the bounding box -> 3D position.
  2. Fallback (person lying low / no LiDAR points): intersect the ray through the
     bottom-centre of the box with the floor plane.
Detections are merged into tracks; a track becomes a confirmed survivor after
`confirm_hits` consistent detections inside the arena.

Publishes
  /airmouse/survivors          std_msgs/String (JSON list)
  /airmouse/detections         std_msgs/String (JSON: latest boxes, for the GCS overlay)
  /airmouse/survivor_markers   visualization_msgs/MarkerArray (RViz)
"""
import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from airmouse.arena import declare_arena
from airmouse.detectors import make_detector
from airmouse.geometry import quat_to_rot, rot_y
from airmouse.imaging import image_to_bgr
from airmouse.params import declare


class SurvivorDetector(Node):
    def __init__(self):
        super().__init__('survivor_detector')
        self.arena = declare_arena(self)
        self.map_frame = declare(self, 'map_frame', 'map')
        backend = declare(self, 'backend', 'ultralytics')
        model = declare(self, 'model', '/opt/models/yolo11n.pt')
        conf = float(declare(self, 'confidence', 0.45))
        imgsz = int(declare(self, 'imgsz', 416))
        rate = float(declare(self, 'detect_rate_hz', 4.0))
        self.cam_xyz = np.array(declare(self, 'camera_mount_xyz', [0.12, 0.0, 0.03]), float)
        self.cam_pitch = float(declare(self, 'camera_pitch', 0.26))   # rad, positive = tilted down
        self.hfov = float(declare(self, 'camera_hfov', 1.57))
        self.floor_z = float(declare(self, 'floor_z', -0.2))
        self.max_range = float(declare(self, 'max_range', 6.0))
        self.merge_r = float(declare(self, 'merge_radius', 1.0))
        self.confirm_hits = int(declare(self, 'confirm_hits', 3))
        self.min_box_px = float(declare(self, 'min_box_px', 20))

        self.R_mount = rot_y(-self.cam_pitch)   # base -> camera-mount (x fwd, y left, z up)
        self.R_mount_inv = rot_y(self.cam_pitch)
        self.detector = make_detector(backend, model, imgsz, conf, self.get_logger())
        self.get_logger().info(f'detector backend: {type(self.detector).__name__}')

        self.K = None
        self.img = None
        self.img_done = None
        self.cloud = None
        self.pose = None
        self.tracks = []
        self.next_id = 1

        sd = qos_profile_sensor_data
        self.create_subscription(Image, declare(self, 'image_topic', '/camera/image_raw'), self._on_img, sd)
        self.create_subscription(CameraInfo, declare(self, 'camera_info_topic', '/camera/camera_info'),
                                 self._on_info, sd)
        self.create_subscription(PointCloud2, '/airmouse/cloud', self._on_cloud, sd)
        self.create_subscription(Odometry, '/airmouse/odom', self._on_odom, 20)
        self.pub_surv = self.create_publisher(String, '/airmouse/survivors', 10)
        self.pub_det = self.create_publisher(String, '/airmouse/detections', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/airmouse/survivor_markers', 10)
        self.create_timer(1.0 / rate, self._process)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_img(self, msg):
        self.img = msg

    def _on_info(self, msg):
        if msg.k[0] > 0:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height)

    def _on_cloud(self, msg):
        self.cloud = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (quat_to_rot(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z]))

    def _intrinsics(self, w, h):
        if self.K is not None:
            fx, fy, cx, cy, kw, kh = self.K
            sx, sy = w / kw, h / kh
            return fx * sx, fy * sy, cx * sx, cy * sy
        f = (w / 2.0) / math.tan(self.hfov / 2.0)
        return f, f, w / 2.0, h / 2.0

    # ------------------------------------------------------------------
    def _process(self):
        msg = self.img
        if msg is None or msg is self.img_done:
            return
        self.img_done = msg
        bgr = image_to_bgr(msg)
        if bgr is None:
            self.get_logger().warn(f'unsupported image encoding {msg.encoding}', throttle_duration_sec=10)
            return
        h, w = bgr.shape[:2]
        boxes = [b for b in self.detector.detect(bgr) if (b[3] - b[1]) >= self.min_box_px]
        now = self._now()
        self.pub_det.publish(String(data=json.dumps(
            {'t': now, 'w': w, 'h': h, 'boxes': [[round(v, 1) for v in b[:4]] + [round(b[4], 2)] for b in boxes]})))
        if self.pose is None or not boxes:
            return
        R, t = self.pose
        K = self._intrinsics(w, h)
        for b in boxes:
            p = self._localize(b, K, R, t)
            if p is not None:
                self._merge(p, b[4], now)
        self._publish()

    def _localize(self, box, K, R, t):
        fx, fy, cx, cy = K
        x1, y1, x2, y2, _ = box
        cam_w = t + R @ self.cam_xyz
        pts = self.cloud
        if pts is not None and len(pts):
            pm = ((pts - t) @ R - self.cam_xyz) @ self.R_mount.T     # map -> base -> camera mount
            fwd = pm[:, 0]
            ok = (fwd > 0.2) & (fwd < self.max_range)
            u = cx - fx * pm[:, 1] / np.where(ok, fwd, 1.0)
            v = cy - fy * pm[:, 2] / np.where(ok, fwd, 1.0)
            bw, bh = x2 - x1, y2 - y1
            ins = ok & (u > x1 + 0.2 * bw) & (u < x2 - 0.2 * bw) & (v > y1 + 0.15 * bh) & (v < y2 - 0.15 * bh)
            if ins.sum() >= 5:
                depth = fwd[ins]
                d0 = np.percentile(depth, 30)
                sel = np.nonzero(ins)[0][np.abs(depth - d0) < 0.35]
                p = pts[sel].mean(axis=0)
                if self.floor_z - 0.3 < p[2] < self.floor_z + 2.3:
                    return p
        # Fallback: ray through the bottom-centre of the box onto the floor.
        u, v = (x1 + x2) / 2.0, y2
        d_w = R @ (self.R_mount_inv @ np.array([1.0, -(u - cx) / fx, -(v - cy) / fy]))
        if d_w[2] > -1e-3:
            return None
        s = (self.floor_z - cam_w[2]) / d_w[2]
        if s * np.linalg.norm(d_w) > self.max_range:
            return None
        return cam_w + s * d_w

    def _merge(self, p, conf, now):
        for tr in self.tracks:
            if math.hypot(tr['x'] - p[0], tr['y'] - p[1]) < self.merge_r:
                n = tr['hits']
                tr['x'] = (tr['x'] * n + p[0]) / (n + 1)
                tr['y'] = (tr['y'] * n + p[1]) / (n + 1)
                tr['z'] = (tr['z'] * n + p[2]) / (n + 1)
                tr['hits'] = n + 1
                tr['conf'] = max(tr['conf'], conf)
                tr['last'] = now
                break
        else:
            self.tracks.append({'id': None, 'x': float(p[0]), 'y': float(p[1]), 'z': float(p[2]),
                                'hits': 1, 'conf': conf, 'first': now, 'last': now, 'confirmed': False})
        self._fuse_tracks()
        for tr in self.tracks:
            tr['cell'] = self.arena.label(tr['x'], tr['y'])
            if not tr['confirmed'] and tr['hits'] >= self.confirm_hits and tr['cell'] != 'OUT':
                tr['confirmed'] = True
                tr['id'] = self.next_id
                self.next_id += 1
                self.get_logger().info(f"SURVIVOR {tr['id']} confirmed in cell {tr['cell']} "
                                       f"({tr['x']:.2f}, {tr['y']:.2f})")

    def _fuse_tracks(self):
        """Merge tracks whose estimates converged onto the same person (first sightings from
        far away can land >merge_radius apart, then drift together as the drone gets closer)."""
        def rank(t):   # keep confirmed tracks first (lowest id), then the most-observed one
            return (not t['confirmed'], t['id'] if t['id'] is not None else 1 << 30, -t['hits'])

        fused = True
        while fused:
            fused = False
            for i, a in enumerate(self.tracks):
                for b in self.tracks[i + 1:]:
                    if math.hypot(a['x'] - b['x'], a['y'] - b['y']) >= self.merge_r:
                        continue
                    keep, drop = sorted((a, b), key=rank)
                    n, m = keep['hits'], drop['hits']
                    for k in ('x', 'y', 'z'):
                        keep[k] = (keep[k] * n + drop[k] * m) / (n + m)
                    keep['hits'] = n + m
                    keep['conf'] = max(keep['conf'], drop['conf'])
                    keep['first'] = min(keep['first'], drop['first'])
                    keep['last'] = max(keep['last'], drop['last'])
                    if drop['confirmed']:
                        self.get_logger().warn(f"survivor {drop['id']} merged into {keep['id']} (duplicate)")
                    self.tracks.remove(drop)
                    fused = True
                    break
                if fused:
                    break

    def _publish(self):
        out = [{'id': tr['id'], 'x': round(tr['x'], 2), 'y': round(tr['y'], 2), 'z': round(tr['z'], 2),
                'cell': tr['cell'], 'conf': round(tr['conf'], 2), 'hits': tr['hits'],
                'confirmed': tr['confirmed']} for tr in self.tracks]
        self.pub_surv.publish(String(data=json.dumps(out)))
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, tr in enumerate(t for t in self.tracks if t['confirmed']):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.map_frame, stamp
            m.ns, m.id, m.type, m.action = 'survivors', i, Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = tr['x'], tr['y'], self.floor_z + 0.9
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.5
            m.scale.z = 1.8
            m.color.r, m.color.a = 1.0, 0.8
            arr.markers.append(m)
        self.pub_mk.publish(arr)


def main():
    rclpy.init()
    node = SurvivorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
