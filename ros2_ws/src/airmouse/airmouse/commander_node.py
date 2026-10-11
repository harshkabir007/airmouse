#!/usr/bin/env python3
"""Mission commander: state machine, frontier exploration and return to the exit.

The operator may only START, RECALL (go back to the exit) or ABORT (land now).
Everything else is autonomous, as the NIDAR rules require.

IDLE -> PREFLIGHT -> ARMING -> TAKEOFF -> EXPLORE -> RETURN -> LANDING -> DONE
                                   any flying state -> LANDING (failsafe / abort)
"""
import json
import math
from enum import Enum

import numpy as np
import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State as MavState
from mavros_msgs.msg import StatusText
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy import ndimage
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

from airmouse import planning
from airmouse.arena import declare_arena
from airmouse.geometry import quat_to_rot, wrap_angle, yaw_of, yaw_to_quat
from airmouse.params import declare


class S(str, Enum):
    IDLE = 'IDLE'
    PREFLIGHT = 'PREFLIGHT'
    ARMING = 'ARMING'
    TAKEOFF = 'TAKEOFF'
    EXPLORE = 'EXPLORE'
    RETURN = 'RETURN'
    LANDING = 'LANDING'
    DONE = 'DONE'
    ABORTED = 'ABORTED'


FLYING = (S.TAKEOFF, S.EXPLORE, S.RETURN)


class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        self.arena = declare_arena(self)
        self.map_frame = declare(self, 'map_frame', 'map')
        self.takeoff_alt = float(declare(self, 'takeoff_alt', 1.2))
        self.wp_tol = float(declare(self, 'waypoint_tolerance', 0.25))
        self.inflation = float(declare(self, 'inflation_radius', 0.30))
        self.plan_res = float(declare(self, 'plan_resolution', 0.10))
        self.max_segment = float(declare(self, 'max_segment_length', 3.0))
        self.min_frontier = float(declare(self, 'min_frontier_size', 0.4))
        self.goal_reach_r = float(declare(self, 'goal_reach_radius', 0.6))
        self.visited_r = float(declare(self, 'visited_radius', 0.8))
        self.size_gain = float(declare(self, 'frontier_size_gain', 1.0))
        self.switch_margin = float(declare(self, 'goal_switch_margin', 1.5))
        self.goal_timeout = float(declare(self, 'goal_timeout_s', 40.0))
        self.spin_enabled = bool(declare(self, 'spin_at_goals', True))
        self.spin_dwell = float(declare(self, 'spin_dwell_s', 1.0))
        # Yaw only where the whole rotor disc fits: clearance >= this (half prop-tip span + margin)
        self.turn_clearance = float(declare(self, 'turn_clearance', 0.35))
        self.max_yaw_rate = math.radians(float(declare(self, 'max_yaw_rate_deg', 45.0)))
        self.turn_in_place = math.radians(float(declare(self, 'turn_in_place_deg', 45.0)))
        self.max_explore_s = float(declare(self, 'max_explore_time_s', 1260.0))
        self.hard_limit_s = float(declare(self, 'hard_time_limit_s', 1680.0))
        self.batt_return = float(declare(self, 'battery_return_fraction', 0.30))
        self.batt_land = float(declare(self, 'battery_land_fraction', 0.18))
        self.slam_grace = float(declare(self, 'slam_loss_grace_s', 1.0))
        self.fence_margin = float(declare(self, 'fence_margin', 1.5))
        self.exit_xy = tuple(declare(self, 'exit_xy', [0.0, 0.0]))
        self.origin_lla = list(declare(self, 'ekf_origin_lla', [12.9716, 77.5946, 900.0]))
        self.use_fcu_offset = bool(declare(self, 'use_fcu_offset', True))
        self.auto_start = bool(declare(self, 'auto_start', False))

        self.state = S.IDLE
        self.state_since = self._now()
        self.msg = 'ready - press START'
        self.complete = False
        self.mission_t0 = None
        self.pose = None            # (x, y, z, yaw) of base_link in map
        self.mav_state = MavState()
        self.fcu_offset = np.zeros(3)
        self.slam_ok = False
        self.slam_bad_since = None
        self.battery = None
        self.map_msg = None
        self.grid = None            # last PlanGrid (for clearance lookups between plans)
        self.ground_z = 0.0
        self.cruise_z = self.takeoff_alt
        self.setpoint = None        # (x, y, z, yaw)
        self.waypoints = []
        self.hold_xy = None         # where to hover while turning on the spot
        self.goal = None
        self.visited = []
        self.spin_targets = []
        self.spin_xy = None
        self.spin_t = None
        self.no_frontier = 0
        self.frontiers = 0
        self.explored_m2 = 0.0
        self.announced = set()
        self.last_req = {}
        self.origin_sent = 0

        sd = qos_profile_sensor_data
        self.create_subscription(Odometry, '/airmouse/odom', self._on_odom, 20)
        self.create_subscription(OccupancyGrid, '/airmouse/map', self._on_map, 1)
        self.create_subscription(Bool, '/airmouse/slam_ok', self._on_slam_ok, 10)
        self.create_subscription(String, '/airmouse/cmd', self._on_cmd, 10)
        self.create_subscription(String, '/airmouse/survivors', self._on_survivors, 10)
        self.create_subscription(MavState, '/mavros/state', self._on_mav_state, sd)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_mav_pose, sd)
        self.create_subscription(BatteryState, '/mavros/battery', self._on_battery, sd)

        self.pub_sp = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.pub_origin = self.create_publisher(GeoPointStamped, '/mavros/global_position/set_gp_origin', 10)
        self.pub_text = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.pub_status = self.create_publisher(String, '/airmouse/status', 10)
        self.pub_path = self.create_publisher(Path, '/airmouse/path', 10)

        self.cli_mode = self.create_client(SetMode, '/mavros/set_mode')
        self.cli_arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cli_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.cli_stream = self.create_client(StreamRate, '/mavros/set_stream_rate')

        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._plan)
        if self.auto_start:
            self.create_timer(5.0, self._auto_start_once)

    # ------------------------------------------------------------------ helpers
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _every(self, key, period):
        t = self._now()
        if t - self.last_req.get(key, -1e9) >= period:
            self.last_req[key] = t
            return True
        return False

    def _set_state(self, s, msg=None):
        if s != self.state:
            self.get_logger().info(f'{self.state.value} -> {s.value}' + (f': {msg}' if msg else ''))
        self.state = s
        self.state_since = self._now()
        if msg:
            self.msg = msg

    def _call(self, client, req, what):
        if not client.service_is_ready():
            self.get_logger().warn(f'{what}: MAVROS service not ready')
            return
        client.call_async(req).add_done_callback(
            lambda f: f.exception() and self.get_logger().error(f'{what} failed: {f.exception()}'))

    def _set_mode(self, mode):
        self._call(self.cli_mode, SetMode.Request(custom_mode=mode), f'set_mode {mode}')

    def _land(self, reason):
        self.get_logger().warn(f'LAND: {reason}')
        self._set_mode('LAND')
        self.waypoints, self.spin_targets, self.goal = [], [], None
        self._set_state(S.LANDING, reason)

    def _start_return(self, reason):
        if self.state != S.EXPLORE:
            return
        self.waypoints, self.spin_targets, self.goal = [], [], None
        self._set_state(S.RETURN, f'returning to exit ({reason})')
        self._plan()

    def _inside_fence(self, x, y):
        xmin, xmax, ymin, ymax = self.arena.bounds()
        ex, ey = self.exit_xy
        m = self.fence_margin
        return (min(xmin, ex, 0.0) - m <= x <= max(xmax, ex, 0.0) + m
                and min(ymin, ey, 0.0) - m <= y <= max(ymax, ey, 0.0) + m)

    # ------------------------------------------------------------------ callbacks
    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, p.z, yaw_of(quat_to_rot(q.x, q.y, q.z, q.w)))

    def _on_mav_pose(self, msg):
        if self.pose is None or not self.use_fcu_offset:
            return
        p = msg.pose.position
        raw = np.array([p.x - self.pose[0], p.y - self.pose[1], p.z - self.pose[2]])
        self.fcu_offset = 0.9 * self.fcu_offset + 0.1 * raw

    def _on_map(self, msg):
        self.map_msg = msg

    def _on_slam_ok(self, msg):
        self.slam_ok = msg.data

    def _on_mav_state(self, msg):
        self.mav_state = msg

    def _on_battery(self, msg):
        if msg.percentage is not None and 0.0 <= msg.percentage <= 1.0:
            self.battery = msg.percentage

    def _on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'start' and self.state in (S.IDLE, S.ABORTED):
            self.complete = False
            self.origin_sent = 0
            self.visited, self.announced = [], set()
            self._set_state(S.PREFLIGHT, 'preflight checks')
        elif cmd == 'abort':
            if self.mav_state.armed or self.state in FLYING:
                self._land('ABORT by operator')
            else:
                self._set_state(S.ABORTED, 'aborted by operator')
        elif cmd == 'recall':
            self._start_return('RECALL by operator')
        else:
            self.get_logger().warn(f'ignored command "{cmd}" in state {self.state.value}')

    def _auto_start_once(self):
        if self.state == S.IDLE and not self.complete and self.mission_t0 is None:
            self._on_cmd(String(data='start'))

    def _on_survivors(self, msg):
        try:
            survivors = json.loads(msg.data)
        except ValueError:
            return
        for s in survivors:
            if s.get('confirmed') and s['id'] not in self.announced:
                self.announced.add(s['id'])
                st = StatusText()
                st.severity = StatusText.NOTICE
                st.text = f"SURVIVOR {s['id']} at {s['cell']}"[:50]
                self.pub_text.publish(st)

    # ------------------------------------------------------------------ main loop
    def _tick(self):
        now = self._now()
        if self.state in FLYING and self.pose is not None:
            self._check_failsafes(now)
        handler = {
            S.PREFLIGHT: self._preflight, S.ARMING: self._arming, S.TAKEOFF: self._takeoff,
            S.EXPLORE: self._follow, S.RETURN: self._follow, S.LANDING: self._landing,
        }.get(self.state)
        if handler:
            handler(now)
        if self.state in (S.EXPLORE, S.RETURN) and self.setpoint is not None:
            self._publish_setpoint()
        if self._every('status', 0.5):
            self._publish_status(now)

    def _check_failsafes(self, now):
        if self.mav_state.mode and self.mav_state.mode != 'GUIDED' and now - self.state_since > 3.0:
            self._set_state(S.ABORTED, f'FC switched to {self.mav_state.mode} - autonomy stopped')
            return
        if not self.slam_ok:
            if self.slam_bad_since is None:
                self.slam_bad_since = now
            elif now - self.slam_bad_since > self.slam_grace:
                self._land('SLAM lost')
                return
        else:
            self.slam_bad_since = None
        elapsed = now - self.mission_t0
        if elapsed > self.hard_limit_s:
            self._land('hard time limit reached')
            return
        if self.battery is not None:
            if self.battery < self.batt_land:
                self._land('battery critical')
                return
            if self.battery < self.batt_return:
                self._start_return('battery low')
        if elapsed > self.max_explore_s:
            self._start_return('exploration time budget used')
        if not self._inside_fence(self.pose[0], self.pose[1]):
            self._land('geofence breach')

    def _preflight(self, now):
        if now - self.state_since > 60.0:
            self._set_state(S.IDLE, 'preflight timeout - check FCU link and SLAM')
            return
        if not self.mav_state.connected:
            self.msg = 'waiting for flight controller link'
            return
        if self.origin_sent < 5 and self._every('origin', 1.0):
            self._send_origin()
            self._call(self.cli_stream, StreamRate.Request(stream_id=0, message_rate=10, on_off=True),
                       'set_stream_rate')
            self.origin_sent += 1
        if not self.slam_ok or self.pose is None:
            self.msg = 'waiting for SLAM'
            return
        if self.origin_sent < 3:
            return
        if self.mav_state.mode != 'GUIDED':
            self.msg = 'switching to GUIDED'
            if self._every('mode', 1.5):
                self._set_mode('GUIDED')
            return
        self._set_state(S.ARMING, 'arming')

    def _send_origin(self):
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position.latitude, msg.position.longitude, msg.position.altitude = map(float, self.origin_lla)
        self.pub_origin.publish(msg)

    def _arming(self, now):
        if self.mav_state.armed:
            self.ground_z = self.pose[2]
            self.cruise_z = self.ground_z + self.takeoff_alt
            self._call(self.cli_takeoff, CommandTOL.Request(altitude=float(self.takeoff_alt)), 'takeoff')
            self.mission_t0 = now
            self._set_state(S.TAKEOFF, 'taking off')
            return
        if now - self.state_since > 30.0:
            self._set_state(S.IDLE, 'arming failed - check Mission Planner messages')
            return
        if self._every('arm', 2.0):
            self._call(self.cli_arm, CommandBool.Request(value=True), 'arm')

    def _takeoff(self, now):
        x, y, z, yaw = self.pose
        if z - self.ground_z >= self.takeoff_alt - 0.15:
            self.setpoint = (x, y, self.cruise_z, yaw)
            self._set_state(S.EXPLORE, 'exploring')
            self._plan()
        elif now - self.state_since > 20.0:
            self._land('takeoff timeout')

    def _landing(self, now):
        if not self.mav_state.armed and now - self.state_since > 2.0:
            self._set_state(S.DONE, 'MISSION COMPLETE - landed at exit' if self.complete else 'landed')

    def _follow(self, now):
        x, y, _, yaw = self.pose
        if self.spin_targets:
            target = self.spin_targets[0]
            self.setpoint = (self.spin_xy[0], self.spin_xy[1], self.cruise_z, self._ramp_yaw(target))
            if abs(wrap_angle(yaw - target)) < math.radians(15):
                if self.spin_t is None:
                    self.spin_t = now
                elif now - self.spin_t > self.spin_dwell:
                    self.spin_targets.pop(0)
                    self.spin_t = None
                    if not self.spin_targets:
                        self._plan()
            return
        if self.waypoints:
            wx, wy = self.waypoints[0]
            if math.hypot(wx - x, wy - y) < self.wp_tol:
                self.waypoints.pop(0)
                if not self.waypoints:
                    self._goal_reached()
                    return
                wx, wy = self.waypoints[0]
            d = math.hypot(wx - x, wy - y)
            desired = math.atan2(wy - y, wx - x) if d > 0.3 else self.setpoint[3]
            if self._clearance(x, y) >= self.turn_clearance:
                new_yaw = self._ramp_yaw(desired)
                if abs(wrap_angle(desired - yaw)) > self.turn_in_place:
                    # big heading change: rotate on the spot first, then translate
                    if self.hold_xy is None:
                        self.hold_xy = (x, y)
                    self.setpoint = (self.hold_xy[0], self.hold_xy[1], self.cruise_z, new_yaw)
                    return
            else:
                new_yaw = self.setpoint[3]   # too tight to rotate safely: translate without yawing
            self.hold_xy = None
            self.setpoint = (wx, wy, self.cruise_z, new_yaw)
        if self.state == S.EXPLORE and self.goal and now - self.goal['t0'] > self.goal_timeout:
            self.get_logger().warn('goal timeout - blacklisting frontier')
            self.visited.append(self.goal['centroid'])
            self.goal, self.waypoints = None, []

    def _goal_reached(self):
        x, y, _, yaw = self.pose
        if self.state == S.RETURN:
            if math.hypot(x - self.exit_xy[0], y - self.exit_xy[1]) < 0.5:
                self.complete = True
                self._land('reached exit')
            return
        if self.goal:
            self.visited.append(self.goal['centroid'])
            self.goal = None
        if self.spin_enabled and self._clearance(x, y) >= self.turn_clearance:
            self.spin_xy = (x, y)
            self.spin_targets = [wrap_angle(yaw + k * math.pi / 2) for k in range(1, 5)]
            self.spin_t = None
        else:
            self._plan()

    def _clearance(self, x, y):
        return self.grid.clearance_at(x, y) if self.grid is not None else 0.0

    def _ramp_yaw(self, target, dt=0.1):
        """Step the commanded yaw toward target at max_yaw_rate (big yaw steps make the drone drift)."""
        cur = self.setpoint[3]
        step = self.max_yaw_rate * dt
        return wrap_angle(cur + float(np.clip(wrap_angle(target - cur), -step, step)))

    def _publish_setpoint(self):
        x, y, z, yaw = self.setpoint
        dx, dy, dz = self.fcu_offset if self.use_fcu_offset else (0.0, 0.0, 0.0)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x + dx, y + dy, z + dz
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = yaw_to_quat(yaw)
        self.pub_sp.publish(msg)

    # ------------------------------------------------------------------ planning
    def _plan(self):
        if self.state not in (S.EXPLORE, S.RETURN) or self.map_msg is None or self.pose is None:
            return
        if self.spin_targets:
            return
        x, y = self.pose[0], self.pose[1]
        xmin, xmax, ymin, ymax = self.arena.bounds()
        ex, ey = self.exit_xy
        crop = (min(xmin, ex, x) - 1.0, max(xmax, ex, x) + 1.0, min(ymin, ey, y) - 1.0, max(ymax, ey, y) + 1.0)
        g = planning.PlanGrid(self.map_msg, self.plan_res, self.inflation, crop)
        self.grid = g
        search = int(math.ceil(0.6 / g.res))
        start = planning.nearest_true(g.trav, g.cell(x, y), search)
        if start is None:
            self.msg = 'no free space around drone - holding'
            return
        dist = planning.wavefront(g.trav, start)

        if self.state == S.EXPLORE:
            goal = self._select_frontier(g, dist)
            if goal is None:
                self.no_frontier += 1
                self.waypoints = []
                if self.no_frontier >= 3:
                    self._start_return('exploration complete')
                return
            self.no_frontier = 0
            goal_cell = goal['cell']
        else:
            goal_cell = planning.nearest_true(dist >= 0, g.cell(ex, ey), search)
            if goal_cell is None:
                self.msg = 'exit not reachable in map yet - holding'
                return

        path = planning.descend(dist, goal_cell)
        if path is None:
            return
        pts = planning.simplify(path, g.trav, max(2, int(self.max_segment / g.res)))
        self.waypoints = [g.world(c) for c in pts[1:]]
        if not self.waypoints:
            self._goal_reached()
        self._publish_path(g.world(start))

    def _select_frontier(self, g, dist):
        xmin, xmax, ymin, ymax = self.arena.bounds()
        xs, ys = g.centers()
        inside = ((ys >= ymin) & (ys <= ymax))[:, None] & ((xs >= xmin) & (xs <= xmax))[None, :]
        self.explored_m2 = float((g.free & inside).sum()) * g.res * g.res
        fr = planning.frontier_mask(g.free, g.unknown, g.occ, max(1, int(math.ceil(0.15 / g.res)))) & inside
        labels, n = ndimage.label(fr, structure=planning.SQUARE)
        self.frontiers = n
        if n == 0:
            return None
        idx = np.arange(1, n + 1)
        sizes = ndimage.sum(fr, labels, index=idx)
        cents = ndimage.center_of_mass(fr, labels, index=idx)
        grow = planning.disk(max(1, int(round(self.goal_reach_r / g.res))))
        pad = grow.shape[0] // 2
        reach = dist >= 0
        H, W = g.shape
        best = current = None
        for i, sl in enumerate(ndimage.find_objects(labels)):
            size_m = float(sizes[i]) * g.res
            if sl is None or size_m < self.min_frontier:
                continue
            cw = g.world(cents[i])
            if any(math.hypot(cw[0] - v[0], cw[1] - v[1]) < self.visited_r for v in self.visited):
                continue
            r0, r1 = max(0, sl[0].start - pad), min(H, sl[0].stop + pad)
            c0, c1 = max(0, sl[1].start - pad), min(W, sl[1].stop + pad)
            near = ndimage.binary_dilation(labels[r0:r1, c0:c1] == i + 1, structure=grow) & reach[r0:r1, c0:c1]
            if not near.any():
                continue
            yy, xx = np.nonzero(near)
            dd = dist[r0:r1, c0:c1][yy, xx]
            k = int(np.argmin(dd))
            cell = (int(yy[k] + r0), int(xx[k] + c0))
            cand = {'cell': cell, 'xy': g.world(cell), 'centroid': cw,
                    'cost': float(dd[k]) * g.res - self.size_gain * min(size_m, 3.0)}
            if self.goal and math.hypot(cw[0] - self.goal['centroid'][0], cw[1] - self.goal['centroid'][1]) < 1.0:
                current = cand
            if best is None or cand['cost'] < best['cost']:
                best = cand
        if current is not None and best['cost'] > current['cost'] - self.switch_margin:
            best = current
        if best is not None:
            keep_t0 = self.goal is not None and best is current
            best['t0'] = self.goal['t0'] if keep_t0 else self._now()
        self.goal = best
        return best

    def _publish_path(self, start_xy):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for px, py in [start_xy] + self.waypoints:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = px, py, self.cruise_z
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def _publish_status(self, now):
        status = {
            'state': self.state.value,
            'msg': self.msg,
            'elapsed': round(now - self.mission_t0, 1) if self.mission_t0 is not None else 0.0,
            'complete': self.complete,
            'mode': self.mav_state.mode,
            'armed': self.mav_state.armed,
            'fcu_connected': self.mav_state.connected,
            'battery': self.battery,
            'slam_ok': self.slam_ok,
            'frontiers': self.frontiers,
            'explored_m2': round(self.explored_m2, 1),
            'goal': list(self.goal['xy']) if self.goal else None,
            'waypoints': [list(w) for w in self.waypoints],
            'exit': list(self.exit_xy),
        }
        self.pub_status.publish(String(data=json.dumps(status)))


def main():
    rclpy.init()
    node = Commander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
