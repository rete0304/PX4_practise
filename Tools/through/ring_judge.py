#!/usr/bin/env python3
"""Judge ring passes from the gz ground truth (THROUGH_MODE_SPEC.md 13.3).

Subscribes to /world/<world>/pose/info (gz transport, ~50 Hz) and detects the
moment the vehicle crosses the ring plane. The crossing point (linearly
interpolated between two pose samples) is classified by its distance r to the
ring center:

    r < inner_diameter/2 - body_radius   -> PASS
    r < inner_diameter/2 + tube_diameter -> HIT
    otherwise                            -> MISS   (crossings with r > --max-distance are ignored)

Only crossings in the dive direction are judged: from the approach side (the side the
ring normal points to) to the far side. Crossings back are listed as REVERSE in
crossings.csv but not counted (use --both-directions to judge them too).

Outputs (in --csv-dir):
    crossings.csv  one row per crossing: result, r, crossing point, speed and the
                   angle between the velocity and the ring normal
    los_truth.csv  true LOS from the camera to the ring center (NED unit vector,
                   distance) at the pose rate, to validate through_status.los_ned (M4)

Times are gz sim time in microseconds, which equals the PX4 SITL clock (VERIFICATION.md V7),
so they line up with the ULog timestamps. World positions are converted from gz ENU to NED.
"""

import argparse
import csv
import math
import os
import signal
import threading
import time

import numpy as np
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


def quat_to_matrix(q):
    """gz Quaternion message -> rotation matrix."""
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def pose_to_rt(pose):
    p = pose.position
    return quat_to_matrix(pose.orientation), np.array([p.x, p.y, p.z])


def enu_to_ned(v):
    return np.array([v[1], v[0], -v[2]])


class RingJudge:

    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.ring = None             # (center, normal) in ENU
        self.prev = None             # (t_us, position ENU, signed distance)
        self.prev_velocity = np.zeros(3)
        self.counts = {'PASS': 0, 'HIT': 0, 'MISS': 0}
        self.last_los_t = 0

        os.makedirs(args.csv_dir, exist_ok=True)
        self.crossings_file = open(os.path.join(args.csv_dir, 'crossings.csv'), 'w', newline='')
        self.crossings = csv.writer(self.crossings_file)
        self.crossings.writerow(['time_us', 'result', 'direction', 'r_m', 'x_n', 'x_e', 'x_d', 'speed_mps',
                                 'angle_to_normal_deg', 'v_n', 'v_e', 'v_d'])
        self.los_file = open(os.path.join(args.csv_dir, 'los_truth.csv'), 'w', newline='')
        self.los = csv.writer(self.los_file)
        self.los.writerow(['time_us', 'cam_n', 'cam_e', 'cam_d', 'los_n', 'los_e', 'los_d', 'range_m',
                           'v_n', 'v_e', 'v_d'])

        self.r_pass = args.inner_diameter / 2.0 - args.body_radius
        self.r_hit = args.inner_diameter / 2.0 + args.tube_diameter

    def close(self):
        with self.lock:
            self.crossings_file.close()
            self.los_file.close()

    def on_poses(self, msg):
        t_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nsec // 1000
        vehicle = camera = None
        for pose in msg.pose:
            if pose.name == self.args.ring and self.ring is None:
                r, c = pose_to_rt(pose)
                self.ring = (c, r[:, 0])  # ring normal = model x axis (gen_through_ring_world.py)
                print(f'ring center (ENU) {np.round(c, 3)}, normal {np.round(r[:, 0], 3)}', flush=True)
            elif pose.name == self.args.vehicle:
                vehicle = pose_to_rt(pose)
            elif pose.name == 'camera_link':
                camera = pose_to_rt(pose)  # relative to the vehicle model
        if vehicle is None or self.ring is None:
            return
        with self.lock:
            self.update(t_us, vehicle, camera)

    def update(self, t_us, vehicle, camera):
        r_wb, p = vehicle
        center, normal = self.ring
        d = float(np.dot(p - center, normal))

        velocity = self.prev_velocity
        if self.prev is not None and t_us > self.prev[0]:
            velocity = (p - self.prev[1]) / ((t_us - self.prev[0]) * 1e-6)
            self.prev_velocity = velocity

        # true LOS from the camera to the ring center
        if t_us - self.last_los_t >= self.args.los_period_us:
            cam = p if camera is None else p + r_wb @ camera[1]
            los = center - cam
            rng = float(np.linalg.norm(los))
            los_ned = enu_to_ned(los / rng)
            self.los.writerow([t_us, *np.round(enu_to_ned(cam), 4), *np.round(los_ned, 6), round(rng, 4),
                               *np.round(enu_to_ned(velocity), 4)])
            self.last_los_t = t_us

        if self.prev is not None and (self.prev[2] > 0.0) != (d > 0.0) and self.prev[2] != d:
            t_prev, p_prev, d_prev = self.prev
            a = d_prev / (d_prev - d)
            x = p_prev + a * (p - p_prev)
            t_x = int(t_prev + a * (t_us - t_prev))
            radial = (x - center) - np.dot(x - center, normal) * normal
            r = float(np.linalg.norm(radial))
            if r <= self.args.max_distance:
                forward = d_prev > 0.0  # approach side -> far side
                self.report(t_x, r, x, velocity, normal, forward)

        self.prev = (t_us, p, d)

    def report(self, t_us, r, x, velocity, normal, forward):
        result = 'PASS' if r < self.r_pass else 'HIT' if r < self.r_hit else 'MISS'
        if not forward and not self.args.both_directions:
            result = 'REVERSE'
        else:
            self.counts[result] += 1
        speed = float(np.linalg.norm(velocity))
        angle = math.degrees(math.acos(min(1.0, abs(float(np.dot(velocity, normal))) / speed))) \
            if speed > 1e-3 else float('nan')
        self.crossings.writerow([t_us, result, 'forward' if forward else 'reverse', round(r, 4), *np.round(enu_to_ned(x), 4), round(speed, 3),
                                 round(angle, 2), *np.round(enu_to_ned(velocity), 3)])
        self.crossings_file.flush()
        print(f'[{t_us * 1e-6:9.3f} s] {result}: r = {r:.3f} m (pass < {self.r_pass:.3f}, hit < {self.r_hit:.3f}), '
              f'speed {speed:.2f} m/s, angle to normal {angle:.1f} deg   '
              f'totals {self.counts}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--world', default='through_ring')
    parser.add_argument('--vehicle', default='x500_through_0')
    parser.add_argument('--ring', default='through_ring')
    parser.add_argument('--inner-diameter', type=float, default=2.0)
    parser.add_argument('--tube-diameter', type=float, default=0.15)
    parser.add_argument('--body-radius', type=float, default=0.35)
    parser.add_argument('--max-distance', type=float, default=10.0,
                        help='ignore plane crossings farther than this from the ring center [m]')
    parser.add_argument('--both-directions', action='store_true',
                        help='also judge crossings from the far side back to the approach side')
    parser.add_argument('--los-rate', type=float, default=50.0, help='los_truth.csv rate [Hz]')
    parser.add_argument('--csv-dir', default='.')
    args = parser.parse_args()
    args.los_period_us = int(1e6 / args.los_rate) if args.los_rate > 0 else 0

    judge = RingJudge(args)
    node = Node()
    topic = f'/world/{args.world}/pose/info'
    if not node.subscribe(Pose_V, topic, judge.on_poses):
        raise SystemExit(f'cannot subscribe to {topic}')
    print(f'ring_judge: listening on {topic}, writing to {os.path.abspath(args.csv_dir)}', flush=True)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    while not stop.is_set():
        time.sleep(0.2)
    node.unsubscribe(topic)
    judge.close()
    print(f'ring_judge: totals {judge.counts}', flush=True)


if __name__ == '__main__':
    main()
