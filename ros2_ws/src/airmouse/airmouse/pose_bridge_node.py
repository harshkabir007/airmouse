#!/usr/bin/env python3
"""FAST-LIO -> drone-centric pose, cloud and ArduPilot external navigation.

FAST-LIO reports its IMU ("body") frame in "camera_init" (the IMU frame at start-up).
The LiDAR is usually mounted upside-down under the drone, so everything is
re-expressed in a level, drone-centred "map" frame:

    map = base_link at start-up (x forward into the maze, y left, z up)
    M   = T_base_body  (LiDAR/IMU mount pose measured on the drone)
    T_map_base(t) = M @ T_camera_init_body(t) @ inv(M)
    p_map         = M @ p_camera_init

Publishes
    /airmouse/odom            nav_msgs/Odometry   base_link pose in map
    /airmouse/cloud           PointCloud2         current scan in map frame
    /mavros/vision_pose/pose  PoseStamped         pose for the ArduPilot EKF3
    /airmouse/slam_ok         std_msgs/Bool       false when SLAM is stale or jumps
Vision poses are withheld while SLAM is unhealthy so the EKF failsafe can act.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from airmouse.geometry import make_tf, quat_to_rot, rot_to_quat, rpy_to_rot
from airmouse.params import declare


class PoseBridge(Node):
    def __init__(self):
        super().__init__('pose_bridge')
        mount_xyz = declare(self, 'lidar_mount_xyz', [0.0, 0.0, -0.08])
        mount_rpy = declare(self, 'lidar_mount_rpy', [math.pi, 0.0, 0.0])
        self.map_frame = declare(self, 'map_frame', 'map')
        self.base_frame = declare(self, 'base_frame', 'base_link')
        self.stale_s = declare(self, 'stale_timeout_s', 0.5)
        self.max_speed = declare(self, 'max_plausible_speed', 4.0)

        self.M = make_tf(rpy_to_rot(*mount_rpy), mount_xyz)
        self.M_inv = np.linalg.inv(self.M)

        self.last_t = None
        self.last_pos = None
        self.bad_until = -1.0

        self.pub_odom = self.create_publisher(Odometry, '/airmouse/odom', 10)
        self.pub_vision = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        self.pub_cloud = self.create_publisher(PointCloud2, '/airmouse/cloud', qos_profile_sensor_data)
        self.pub_ok = self.create_publisher(Bool, '/airmouse/slam_ok', 10)
        self.tf = TransformBroadcaster(self)

        self.create_subscription(Odometry, declare(self, 'odom_in', '/Odometry'), self._on_odom, 50)
        self.create_subscription(PointCloud2, declare(self, 'cloud_in', '/cloud_registered'),
                                 self._on_cloud, qos_profile_sensor_data)
        self.create_timer(0.1, self._health)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _healthy(self, now):
        return self.last_t is not None and now - self.last_t < self.stale_s and now > self.bad_until

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        T = self.M @ make_tf(quat_to_rot(q.x, q.y, q.z, q.w), [p.x, p.y, p.z]) @ self.M_inv
        pos = T[:3, 3]
        now = self._now()
        if self.last_pos is not None:
            dt = now - self.last_t
            if dt > 1e-3 and np.linalg.norm(pos - self.last_pos) / dt > self.max_speed:
                self.bad_until = now + 2.0
                self.get_logger().warn('SLAM pose jump detected - withholding vision for 2 s')
        self.last_t, self.last_pos = now, pos
        qx, qy, qz, qw = rot_to_quat(T[:3, :3])

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, pos)
        o = odom.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, pos)
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)

        if self._healthy(now):
            vis = PoseStamped()
            vis.header = odom.header
            vis.pose = odom.pose.pose
            self.pub_vision.publish(vis)

    def _on_cloud(self, msg):
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            return
        pts = pts.astype(np.float64) @ self.M[:3, :3].T + self.M[:3, 3]
        msg.header.frame_id = self.map_frame
        self.pub_cloud.publish(point_cloud2.create_cloud_xyz32(msg.header, pts.astype(np.float32)))

    def _health(self):
        self.pub_ok.publish(Bool(data=bool(self._healthy(self._now()))))


def main():
    rclpy.init()
    node = PoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
