#!/usr/bin/env python3
"""M3 checks for the THROUGH simulation (vehicle + camera + vision node).

Run with the stack from run_through_sim.sh (real vision node). Uses MAVLink 14540 for
the vehicle and ROS 2 for /fmu/in/vision_target and the ~/set_lock service, so the
ROS 2 workspace must be sourced.

  1. take off to --alt (default 25 m, D8) and hover: hover thrust estimate ~0.60 (D7)
  2. ring detected from the hover point
  3. pan/tilt signs (spec 6) when moving the vehicle:
       move left (north, the vehicle faces east to the ring) -> target right   -> pan  increases
       yaw right (clockwise)                                -> target left    -> pan  decreases
       descend                                              -> target higher  -> tilt decreases
  4. ~/set_lock: lock accepted while detected, uncage follows lock / unlock
"""

import argparse
import math
import os
import subprocess
import sys
import time

import rclpy
from px4_msgs.msg import VisionTarget
from rclpy.qos import qos_profile_sensor_data

from m2_acceptance import Vehicle, log, mavutil


class VisionListener:

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('m3_check')
        self.last = None
        self.node.create_subscription(VisionTarget, '/fmu/in/vision_target', self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        self.last = msg

    def spin(self, duration):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def average(self, duration=2.0):
        """Mean pan/tilt/size over valid samples."""
        samples = []
        end = time.time() + duration
        while time.time() < end:
            self.last = None
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.last is not None and self.last.target_valid:
                samples.append((self.last.pan_error, self.last.tilt_error, self.last.target_size))
        if not samples:
            return None
        n = len(samples)
        return tuple(sum(s[i] for s in samples) / n for i in range(3))


def set_lock(locked):
    data = 'true' if locked else 'false'
    out = subprocess.run(['ros2', 'service', 'call', '/through_vision/set_lock', 'std_srvs/srv/SetBool',
                          f'{{data: {data}}}'], capture_output=True, text=True, timeout=30)
    return 'success=True' in out.stdout, out.stdout.strip().splitlines()[-1] if out.stdout else out.stderr


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--url', default='udpin:0.0.0.0:14540')
    parser.add_argument('--alt', type=float, default=25.0)
    args = parser.parse_args()

    results = {}
    v = Vehicle(args.url)
    vis = VisionListener()
    v.m.mav.request_data_stream_send(v.sys, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    v.m.mav.command_long_send(v.sys, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                              mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000, 0, 0, 0, 0, 0)
    v.set_param('MIS_TAKEOFF_ALT', args.alt)

    def fly(seconds):
        end = time.time() + seconds
        while time.time() < end:
            v.pump(0.05)
            vis.spin(0.05)

    def global_pos():
        msg = None
        while msg is None:
            msg = v.pump(1.0, want='GLOBAL_POSITION_INT')
        return msg

    def reposition(north=0.0, east=0.0, up=0.0, yaw_deg=math.nan):
        g = global_pos()
        lat = g.lat * 1e-7 + north / 6378137.0 * 180.0 / math.pi
        lon = g.lon * 1e-7 + east / (6378137.0 * math.cos(math.radians(g.lat * 1e-7))) * 180.0 / math.pi
        alt = g.alt * 1e-3 + up
        # MAV_CMD_DO_REPOSITION: param2 bit 1 = change to guided mode (Hold in PX4).
        # PX4 takes param4 (yaw) in radians (navigator_main.cpp), unlike the MAVLink spec (degrees).
        yaw = math.remainder(math.radians(yaw_deg), 2.0 * math.pi) if math.isfinite(yaw_deg) else math.nan
        return v.command(mavutil.mavlink.MAV_CMD_DO_REPOSITION, -1, 1, 0, yaw, lat, lon, alt)

    log('arm and take off')
    v.wait(lambda: False, 3.0)
    if v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1) != 0:
        log('arming failed')
        return report(results)
    v.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
    if not v.wait(lambda: v.local_pos is not None and -v.local_pos.z > args.alt - 0.5, 90):
        log('takeoff did not reach altitude')
        return report(results)
    fly(15)

    # 1. hover thrust (read from the PX4 shell)
    out = subprocess.run(['./bin/px4-listener', 'hover_thrust_estimate'], capture_output=True, text=True,
                         cwd=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      '../../build/px4_sitl_default'))
    hte = None
    for line in out.stdout.splitlines():
        if line.strip().startswith('hover_thrust:'):
            hte = float(line.split(':')[1])
    log(f'1. hover_thrust_estimate = {hte}')
    results['1 hover thrust 0.55..0.65'] = hte is not None and 0.55 <= hte <= 0.65

    # 2. detection from the hover point
    base = vis.average()
    log(f'2. pan/tilt/size at hover: {fmt(base)}')
    results['2 ring detected at hover'] = base is not None
    if base is None:
        return report(results)

    # 3. signs
    def step(name, key, expect_sign, north=0.0, up=0.0, yaw_delta=0.0):
        heading = global_pos().hdg * 0.01
        before = vis.average()
        reposition(north=north, up=up, yaw_deg=heading + yaw_delta)
        fly(8)
        after = vis.average()
        reposition(north=-north, up=-up, yaw_deg=heading)
        fly(8)
        if before is None or after is None:
            log(f'3. {name}: target lost')
            results[f'3 {name}'] = False
            return
        delta = after[key] - before[key]
        ok = math.copysign(1, delta) == expect_sign and abs(delta) > math.radians(1.0)
        log(f'3. {name}: {"pan" if key == 0 else "tilt"} {math.degrees(before[key]):+.2f} -> '
            f'{math.degrees(after[key]):+.2f} deg ({"ok" if ok else "WRONG"})')
        results[f'3 {name}'] = ok

    step('move left (north 3 m) -> pan +', 0, +1, north=3.0)
    step('yaw right 10 deg -> pan -', 0, -1, yaw_delta=10.0)
    step('descend 3 m -> tilt -', 1, -1, up=-3.0)

    # 4. lock service
    ok_lock, msg = set_lock(True)
    vis.spin(0.5)
    locked = vis.last is not None and vis.last.uncage
    ok_unlock, _ = set_lock(False)
    vis.spin(0.5)
    unlocked = vis.last is not None and not vis.last.uncage
    log(f'4. set_lock(true): {msg} -> uncage {locked}; set_lock(false) -> uncage {not unlocked}')
    results['4 set_lock service'] = ok_lock and locked and ok_unlock and unlocked

    log('land')
    v.command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
    return report(results)


def fmt(sample):
    if sample is None:
        return 'no target'
    return f'pan {math.degrees(sample[0]):+.2f} deg, tilt {math.degrees(sample[1]):+.2f} deg, size {sample[2]:.3f}'


def report(results):
    print()
    print('M3 check summary')
    for k, ok in results.items():
        print(f'  [{"PASS" if ok else "FAIL"}] {k}')
    return 0 if results and all(results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
