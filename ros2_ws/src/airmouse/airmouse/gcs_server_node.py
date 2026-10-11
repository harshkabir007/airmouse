#!/usr/bin/env python3
"""Web Ground Control Station served from the drone (or the sim container).

Open http://<drone-ip>:8080 on the GCS laptop over the team's own Wi-Fi link.
No internet needed: page, map, video and state all come from this node.

  GET  /              dashboard (web/index.html)
  GET  /api/state     JSON: mission status, pose, survivors, path, map meta, arena grid
  GET  /map.png       occupancy map, rotated so +x (into the maze) is up
  GET  /video.mjpg    live camera (MJPEG) with detection boxes drawn on it
  POST /api/cmd       {"cmd": "start" | "recall" | "abort"}  (the only operator inputs)
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from airmouse.arena import declare_arena
from airmouse.geometry import quat_to_rot, yaw_of
from airmouse.imaging import image_to_bgr
from airmouse.params import declare

# unknown, free, occupied (BGR)
MAP_LUT = {-1: (54, 47, 43), 0: (236, 236, 236), 100: (30, 30, 30)}


class GcsServer(Node):
    def __init__(self):
        super().__init__('gcs_server')
        self.arena = declare_arena(self)
        port = int(declare(self, 'port', 8080))
        bind = declare(self, 'bind', '0.0.0.0')
        self.jpeg_q = int(declare(self, 'jpeg_quality', 70))
        self.video_period = 1.0 / float(declare(self, 'video_max_fps', 15.0))
        self.video_width = int(declare(self, 'video_width', 640))
        self.index_html = os.path.join(get_package_share_directory('airmouse'), 'web', 'index.html')

        self.lock = threading.Lock()
        self.status = {}
        self.pose = None
        self.survivors = []
        self.path = []
        self.slam_ok = False
        self.detections = None
        self.det_rx = 0.0
        self.map_png = None
        self.map_meta = None
        self.map_seq = 0
        self.frame = None
        self.frame_seq = 0
        self.frame_t = 0.0
        self.last_video = 0.0
        self.frame_cond = threading.Condition()

        sd = qos_profile_sensor_data
        self.create_subscription(String, '/airmouse/status', self._on_status, 10)
        self.create_subscription(String, '/airmouse/survivors', self._on_survivors, 10)
        self.create_subscription(String, '/airmouse/detections', self._on_detections, 10)
        self.create_subscription(Odometry, '/airmouse/odom', self._on_odom, 20)
        self.create_subscription(Path, '/airmouse/path', self._on_path, 10)
        self.create_subscription(Bool, '/airmouse/slam_ok', self._on_slam, 10)
        self.create_subscription(OccupancyGrid, '/airmouse/map', self._on_map, 1)
        self.create_subscription(Image, declare(self, 'image_topic', '/camera/image_raw'), self._on_image, sd)
        self.pub_cmd = self.create_publisher(String, '/airmouse/cmd', 10)

        self.httpd = ThreadingHTTPServer((bind, port), make_handler(self))
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'GCS dashboard on http://{bind}:{port}')

    # ------------------------------------------------------------------ ROS callbacks
    def _on_status(self, msg):
        with self.lock:
            self.status = json.loads(msg.data)

    def _on_survivors(self, msg):
        with self.lock:
            self.survivors = json.loads(msg.data)

    def _on_detections(self, msg):
        with self.lock:
            self.detections = json.loads(msg.data)
            self.det_rx = time.monotonic()

    def _on_slam(self, msg):
        self.slam_ok = msg.data

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self.lock:
            self.pose = {'x': p.x, 'y': p.y, 'z': p.z, 'yaw': yaw_of(quat_to_rot(q.x, q.y, q.z, q.w))}

    def _on_path(self, msg):
        with self.lock:
            self.path = [[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses]

    def _on_map(self, msg):
        nx, ny = msg.info.width, msg.info.height
        grid = np.asarray(msg.data, dtype=np.int8).reshape(ny, nx)
        img = np.empty((ny, nx, 3), np.uint8)
        img[:] = MAP_LUT[-1]
        img[(grid >= 0) & (grid < 50)] = MAP_LUT[0]
        img[grid >= 50] = MAP_LUT[100]
        # rows = iy, cols = ix  ->  screen: up = +x, left = +y
        disp = np.ascontiguousarray(img.transpose(1, 0, 2)[::-1, ::-1])
        ok, png = cv2.imencode('.png', disp)
        if not ok:
            return
        with self.lock:
            self.map_png = png.tobytes()
            self.map_seq += 1
            self.map_meta = {'seq': self.map_seq, 'res': msg.info.resolution, 'nx': nx, 'ny': ny,
                             'ox': msg.info.origin.position.x, 'oy': msg.info.origin.position.y}

    def _on_image(self, msg):
        now = time.monotonic()
        if now - self.last_video < self.video_period:
            return
        self.last_video = now
        bgr = image_to_bgr(msg)
        if bgr is None:
            return
        with self.lock:
            det = self.detections if now - self.det_rx < 1.0 else None
        if det:
            sx, sy = bgr.shape[1] / det['w'], bgr.shape[0] / det['h']
            for x1, y1, x2, y2, c in det['boxes']:
                p1, p2 = (int(x1 * sx), int(y1 * sy)), (int(x2 * sx), int(y2 * sy))
                cv2.rectangle(bgr, p1, p2, (40, 40, 255), 2)
                cv2.putText(bgr, f'person {c:.2f}', (p1[0], max(12, p1[1] - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 40, 255), 1, cv2.LINE_AA)
        if bgr.shape[1] > self.video_width:
            s = self.video_width / bgr.shape[1]
            bgr = cv2.resize(bgr, (self.video_width, int(bgr.shape[0] * s)))
        ok, jpg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_q])
        if ok:
            with self.frame_cond:
                self.frame = jpg.tobytes()
                self.frame_seq += 1
                self.frame_t = now
                self.frame_cond.notify_all()

    # ------------------------------------------------------------------ HTTP helpers
    def state_json(self):
        with self.lock:
            return json.dumps({
                'status': self.status, 'pose': self.pose, 'survivors': self.survivors,
                'path': self.path, 'map': self.map_meta, 'arena': self.arena.to_dict(),
                'slam_ok': self.slam_ok,
                'video_age': round(time.monotonic() - self.frame_t, 1) if self.frame_t else None,
            }).encode()

    def command(self, cmd):
        if cmd not in ('start', 'recall', 'abort'):
            return False
        self.get_logger().warn(f'operator command: {cmd}')
        self.pub_cmd.publish(String(data=cmd))
        return True


def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            route = self.path.split('?')[0]
            if route in ('/', '/index.html'):
                with open(node.index_html, 'rb') as f:
                    self._send(200, 'text/html; charset=utf-8', f.read())
            elif route == '/api/state':
                self._send(200, 'application/json', node.state_json())
            elif route == '/map.png':
                with node.lock:
                    png = node.map_png
                if png is None:
                    self._send(404, 'text/plain', b'no map yet')
                else:
                    self._send(200, 'image/png', png)
            elif route == '/video.mjpg':
                self._stream()
            else:
                self._send(404, 'text/plain', b'not found')

        def do_POST(self):
            if self.path != '/api/cmd':
                self._send(404, 'text/plain', b'not found')
                return
            try:
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
                ok = node.command(str(body.get('cmd', '')))
            except ValueError:
                ok = False
            self._send(200 if ok else 400, 'application/json', json.dumps({'ok': ok}).encode())

        def _stream(self):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            last = -1
            while True:
                with node.frame_cond:
                    node.frame_cond.wait_for(lambda: node.frame_seq != last, timeout=2.0)
                    jpg, last = node.frame, node.frame_seq
                if jpg is None:
                    continue
                try:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                except (BrokenPipeError, ConnectionResetError):
                    return

    return Handler


def main():
    rclpy.init()
    node = GcsServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.httpd.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
