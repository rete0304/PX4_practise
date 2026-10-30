#!/usr/bin/env python3
"""M4 acceptance for the THROUGH mode: gimbal tracking + LOS reconstruction (ALIGN).

Run with the stack from run_through_sim.sh (real vision node and ring_judge.py, which
writes los_truth.csv into the log directory). Uses MAVLink 14540 and the
/through_vision/set_lock service, so the ROS 2 workspace must be sourced.

Flight:
  1. take off to --alt (25 m, D8), turn --yaw-offset deg away from the ring
  2. lock the target and switch to THROUGH
  3. ALIGN turns towards the LOS azimuth and tracks with the gimbal, then -> RAMP
     (RAMP only holds in M4), stay --hold s in RAMP, switch to Hold and land

Checks (from the ULog and los_truth.csv):
  a. ALIGN -> RAMP happened (yaw aligned)
  b. |tilt_error| in steady state (from 1 s before RAMP until the end) < 1 deg
  c. angle between through_status.los_ned and the true LOS (camera -> ring center) < 1.5 deg
"""

import argparse
import csv
import glob
import math
import os
import subprocess
import sys
import time

import numpy as np

from m2_acceptance import Vehicle, log, mavutil

THROUGH_CUSTOM_MODE = (11 << 24) | (4 << 16)  # AUTO / EXTERNAL1
HOLD_CUSTOM_MODE = (3 << 24) | (4 << 16)      # AUTO / LOITER
PX4_BUILD = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../build/px4_sitl_default')


def set_lock(locked):
    data = 'true' if locked else 'false'
    out = subprocess.run(['ros2', 'service', 'call', '/through_vision/set_lock', 'std_srvs/srv/SetBool',
                          f'{{data: {data}}}'], capture_output=True, text=True, timeout=30)
    return 'success=True' in out.stdout


def fly(args):
    v = Vehicle(args.url)
    v.m.mav.command_long_send(v.sys, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                              mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000, 0, 0, 0, 0, 0)
    v.set_param('MIS_TAKEOFF_ALT', args.alt)
    set_lock(False)
    v.wait(lambda: False, 3.0)

    log('arm and take off')
    if v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1) != 0:
        raise SystemExit('arming failed')
    v.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
    if not v.wait(lambda: v.local_pos is not None and -v.local_pos.z > args.alt - 0.5, 90):
        raise SystemExit('takeoff did not reach altitude')
    v.wait(lambda: False, 5)

    g = v.pump(2.0, want='GLOBAL_POSITION_INT')
    yaw = math.radians(g.hdg * 0.01 + args.yaw_offset)
    log(f'turn {args.yaw_offset:+.0f} deg away from the ring')
    # PX4 takes the DO_REPOSITION yaw in radians; NaN position = stay
    v.command(mavutil.mavlink.MAV_CMD_DO_REPOSITION, -1, 1, 0, math.remainder(yaw, 2 * math.pi),
              math.nan, math.nan, math.nan)
    v.wait(lambda: False, 8)

    log('lock and switch to THROUGH')
    if not set_lock(True):
        raise SystemExit('lock refused (target not detected?)')
    v.wait(lambda: False, 1.0)
    v.status_texts.clear()
    v.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 11)
    if not v.wait(lambda: v.custom_mode == THROUGH_CUSTOM_MODE, 3.0):
        raise SystemExit('THROUGH not entered')
    t_enter = time.time()

    ramp = v.wait(lambda: any('ALIGN -> RAMP' in s for s in v.status_texts), 15.0)
    log(f'RAMP reached: {ramp} after {time.time() - t_enter:.1f} s')
    if ramp:
        v.wait(lambda: False, args.hold)

    log('switch to Hold and land')
    v.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 3)
    v.wait(lambda: v.custom_mode == HOLD_CUSTOM_MODE, 3.0)
    set_lock(False)
    v.command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)

    # the ULog is closed on disarm
    end = time.time() + 90
    while time.time() < end:
        hb = v.pump(1.0, want='HEARTBEAT')
        if hb is not None and hb.get_srcComponent() == 1 and \
                not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            log('landed and disarmed')
            break
    return v.status_texts


def latest_ulog():
    logs = glob.glob(os.path.join(PX4_BUILD, 'rootfs/log/*/*.ulg'))
    return max(logs, key=os.path.getmtime) if logs else None


def load_truth(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([int(r['time_us']) for r in rows])
    los = np.array([[float(r['los_n']), float(r['los_e']), float(r['los_d'])] for r in rows])
    return t, los


def analyze(ulog_path, truth_path):
    from pyulog import ULog

    ulog = ULog(ulog_path, ['through_status'])
    st = ulog.data_list[0].data
    t = st['timestamp']
    phase = st['phase']
    los = np.stack([st['los_ned[0]'], st['los_ned[1]'], st['los_ned[2]']], axis=1)
    tilt = st['tilt_error']
    pan = st['pan_error']

    results = {}
    align = np.where(phase == 1)[0]
    ramp = np.where(phase == 2)[0]
    if len(align) == 0:
        print('no ALIGN phase in the log')
        return {'a ALIGN -> RAMP': False}
    t_align = t[align[0]]
    print(f'ALIGN from {t_align * 1e-6:.2f} s, '
          f'{"RAMP from %.2f s (%.2f s in ALIGN)" % (t[ramp[0]] * 1e-6, (t[ramp[0]] - t_align) * 1e-6) if len(ramp) else "no RAMP"}')
    results['a ALIGN -> RAMP'] = len(ramp) > 0

    active = (phase == 1) | (phase == 2)
    if len(ramp):
        steady = active & (t >= t[ramp[0]] - 1_000_000)
    else:
        steady = active & (t >= t_align + 5_000_000)
    tilt_s = np.abs(np.degrees(tilt[steady & np.isfinite(tilt)]))
    pan_s = np.abs(np.degrees(pan[steady & np.isfinite(pan)]))
    print(f'steady state ({steady.sum()} samples): |tilt| mean {tilt_s.mean():.3f} max {tilt_s.max():.3f} deg, '
          f'|pan| mean {pan_s.mean():.3f} max {pan_s.max():.3f} deg')
    results['b |tilt_error| steady < 1 deg'] = bool(len(tilt_s)) and tilt_s.max() < 1.0

    t_truth, los_truth = load_truth(truth_path)
    valid = active & np.all(np.isfinite(los), axis=1)
    idx = np.clip(np.searchsorted(t_truth, t[valid]), 0, len(t_truth) - 1)
    dt = np.abs(t_truth[idx] - t[valid])
    dots = np.clip(np.sum(los[valid] * los_truth[idx], axis=1), -1.0, 1.0)
    err = np.degrees(np.arccos(dots))[dt < 50_000]
    err_steady = np.degrees(np.arccos(dots))[(dt < 50_000) & steady[valid]]
    print(f'LOS error vs truth: whole ALIGN/RAMP mean {err.mean():.3f} max {err.max():.3f} deg ({len(err)} samples); '
          f'steady mean {err_steady.mean():.3f} max {err_steady.max():.3f} deg')
    results['c LOS error < 1.5 deg'] = bool(len(err)) and err.max() < 1.5

    gc, gm = st['gimbal_pitch_cmd'], st['gimbal_pitch_meas']
    g = steady & np.isfinite(gc) & np.isfinite(gm)
    if g.any():
        print(f'gimbal (+ = down): cmd {np.degrees(gc[g]).mean():.2f} deg, meas {np.degrees(gm[g]).mean():.2f} deg, '
              f'LOS elevation {np.degrees(st["los_elevation"][g]).mean():.2f} deg')
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--url', default='udpin:0.0.0.0:14540')
    parser.add_argument('--alt', type=float, default=25.0)
    parser.add_argument('--yaw-offset', type=float, default=30.0, help='initial heading offset from the ring [deg]')
    parser.add_argument('--hold', type=float, default=5.0, help='time to stay in RAMP [s]')
    parser.add_argument('--log-dir', required=True, help='run_through_sim.sh log dir (los_truth.csv)')
    parser.add_argument('--ulog', help='analyze this ULog instead of flying')
    args = parser.parse_args()

    if args.ulog:
        ulog = args.ulog
    else:
        fly(args)
        time.sleep(3)  # let the logger flush
        ulog = latest_ulog()
    print(f'ULog: {ulog}')
    results = analyze(ulog, os.path.join(args.log_dir, 'los_truth.csv'))

    print()
    print('M4 summary')
    for k, ok in results.items():
        print(f'  [{"PASS" if ok else "FAIL"}] {k}')
    return 0 if results and all(results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
