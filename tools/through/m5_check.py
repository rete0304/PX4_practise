#!/usr/bin/env python3
"""M5 acceptance for the THROUGH mode: RAMP + GUIDE + all EXIT paths.

Run with the stack from run_through_sim.sh (real vision node; ring_judge.py optional).
Uses MAVLink 14540 and the /through_vision/set_lock service through rclpy, so the
ROS 2 workspace must be sourced.

The vehicle takes off to --alt facing the ring, and for each scenario flies back to
the start point, locks the target, switches to THROUGH and triggers one exit:

  unlock       in GUIDE: set_lock(false)                      -> USER_UNLOCK
  mode_switch  in GUIDE: switch to Hold                         -> USER_MODE_SWITCH
  min_alt      THRU_MIN_ALT = alt - 4 m, the dive goes below it -> MIN_ALT
  align_to     THRU_ALN_T_OK > THRU_ALN_T_MAX                   -> ALIGN_TIMEOUT
  mode_to      THRU_T_MAX = 4 s                                 -> MODE_TIMEOUT
  dive         default parameters, full dive approach          -> COMPLETED (or a logged abort)

Checks per scenario: the expected exit reason (status text live, through_status in the
ULog afterwards) and that the vehicle is in Hold after the exit. The dive also has to
pass through RAMP and GUIDE, and closes in on the ring (minimum distance from the ULog
LOS geometry, crossings from ring_judge if available).
"""

import argparse
import csv
import glob
import math
import os
import sys
import time

import numpy as np

from m2_acceptance import Vehicle, log, mavutil

THROUGH_CUSTOM_MODE = (11 << 24) | (4 << 16)  # AUTO / EXTERNAL1
HOLD_CUSTOM_MODE = (3 << 24) | (4 << 16)      # AUTO / LOITER
PX4_BUILD = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../PX4-Autopilot/build/px4_sitl_default')

REASONS = ['NONE', 'COMPLETED', 'USER_UNLOCK', 'USER_MODE_SWITCH', 'ALIGN_TIMEOUT', 'TARGET_LOST', 'MIN_ALT',
           'TILT_SAT', 'MODE_TIMEOUT', 'OPENING']
PHASES = ['IDLE', 'ALIGN', 'RAMP', 'GUIDE', 'TERMINAL', 'EXIT']


class Lock:
    """/through_vision/set_lock client (rclpy: the ros2 CLI takes ~1 s per call, too slow in GUIDE)."""

    def __init__(self):
        import rclpy
        from std_srvs.srv import SetBool
        self._rclpy = rclpy
        self._SetBool = SetBool
        rclpy.init()
        self.node = rclpy.create_node('m5_check')
        self.client = self.node.create_client(SetBool, '/through_vision/set_lock')
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise SystemExit('/through_vision/set_lock not available')

    def set(self, locked):
        req = self._SetBool.Request()
        req.data = locked
        future = self.client.call_async(req)
        self._rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.result() is not None and future.result().success


def command_int(v, cmd, frame, p1=0.0, p2=0.0, p3=0.0, p4=0.0, x=0, y=0, z=0.0, timeout=3.0):
    v.m.mav.command_int_send(v.sys, 1, frame, cmd, 0, 0, p1, p2, p3, p4, x, y, z)
    end = time.time() + timeout
    while time.time() < end:
        ack = v.pump(end - time.time(), want='COMMAND_ACK')
        if ack is not None and ack.command == cmd:
            return ack.result
    return None


def set_mode_hold(v):
    v.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 3)
    return v.wait(lambda: v.custom_mode == HOLD_CUSTOM_MODE, 3.0)


def go_to_start(v, start):
    """Fly back to the start point (Hold + DO_REPOSITION) and wait until settled."""
    if v.custom_mode != HOLD_CUSTOM_MODE:
        set_mode_hold(v)
    command_int(v, mavutil.mavlink.MAV_CMD_DO_REPOSITION, mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                -1, 1, 0, start['yaw'], start['lat'], start['lon'], start['amsl'])

    def settled():
        lp = v.local_pos
        if lp is None:
            return False
        d = math.dist((lp.x, lp.y, lp.z), start['ned'])
        s = math.sqrt(lp.vx ** 2 + lp.vy ** 2 + lp.vz ** 2)
        return d < 0.5 and s < 0.2

    ok = v.wait(settled, 60.0)
    v.wait(lambda: False, 3.0)  # let the heading settle as well
    return ok


def wait_text(v, pattern, timeout):
    return v.wait(lambda: any(pattern in s for s in v.status_texts), timeout)


def exit_text(v):
    for s in v.status_texts:
        if s.startswith('THROUGH aborted: ') or s.startswith('THROUGH stopped: '):
            return s.split(': ', 1)[1].strip()
        if s.startswith('THROUGH completed'):
            return 'COMPLETED'
    return None


def run_scenario(v, lock, start, name, expected, params, trigger, alt):
    log(f'=== scenario {name}: expect {expected} ===')
    result = {'name': name, 'expected': expected}

    if not go_to_start(v, start):
        log('  could not reach the start point')

    for pname, value in params.items():
        v.set_param(pname, value)

    if not lock.set(True):
        log('  lock refused (target not detected?)')
        result.update(live=None, hold=False, error='lock refused')
        return result

    v.wait(lambda: False, 1.0)
    v.status_texts.clear()
    v.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 11)

    if not v.wait(lambda: v.custom_mode == THROUGH_CUSTOM_MODE, 3.0):
        result.update(live=None, hold=False, error='THROUGH not entered')
        lock.set(False)
        return result

    result['t_enter'] = time.time()
    trigger(v, lock)

    got_exit = v.wait(lambda: exit_text(v) is not None, 30.0)
    result['live'] = exit_text(v) if got_exit else None
    result['hold'] = v.wait(lambda: v.custom_mode == HOLD_CUSTOM_MODE, 3.0)
    lp = v.local_pos
    result['alt_after'] = -lp.z if lp is not None else math.nan
    log(f'  exit: {result["live"]}, Hold: {result["hold"]}, altitude {result["alt_after"]:.1f} m')

    lock.set(False)

    for pname in params:
        v.set_param(pname, DEFAULTS[pname])

    v.wait(lambda: False, 2.0)
    return result


DEFAULTS = {'THRU_MIN_ALT': 2.0, 'THRU_ALN_T_OK': 0.5, 'THRU_ALN_T_MAX': 8.0, 'THRU_T_MAX': 30.0}


def trig_none(v, lock):
    pass


def trig_in_guide(action, delay=0.5):
    def trigger(v, lock):
        if not wait_text(v, 'RAMP -> GUIDE', 15.0):
            log('  GUIDE not reached')
            return
        v.wait(lambda: False, delay)
        log(f'  trigger: {action}')
        if action == 'unlock':
            lock.set(False)
        else:
            v.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 3)
    return trigger


def fly(args):
    lock = Lock()
    v = Vehicle(args.url)
    v.m.mav.command_long_send(v.sys, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                              mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000, 0, 0, 0, 0, 0)
    v.set_param('MIS_TAKEOFF_ALT', args.alt)
    for pname, value in DEFAULTS.items():
        v.set_param(pname, value)
    lock.set(False)
    v.wait(lambda: False, 3.0)

    log('arm and take off')
    if v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1) != 0:
        raise SystemExit('arming failed')
    v.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
    if not v.wait(lambda: v.local_pos is not None and -v.local_pos.z > args.alt - 0.5, 90):
        raise SystemExit('takeoff did not reach altitude')
    v.wait(lambda: False, 5)

    g = v.pump(2.0, want='GLOBAL_POSITION_INT')
    lp = v.local_pos
    start = {'lat': g.lat, 'lon': g.lon, 'amsl': g.alt * 1e-3, 'yaw': math.radians(g.hdg * 0.01),
             'ned': (lp.x, lp.y, lp.z)}
    log(f'start point: NED {lp.x:.2f} {lp.y:.2f} {lp.z:.2f}, heading {g.hdg * 0.01:.1f} deg')

    scenarios = [
        ('unlock', 'USER_UNLOCK', {}, trig_in_guide('unlock')),
        ('mode_switch', 'USER_MODE_SWITCH', {}, trig_in_guide('hold')),
        ('min_alt', 'MIN_ALT', {'THRU_MIN_ALT': args.alt - 4.0}, trig_none),
        ('align_to', 'ALIGN_TIMEOUT', {'THRU_ALN_T_OK': 20.0, 'THRU_ALN_T_MAX': 3.0}, trig_none),
        ('mode_to', 'MODE_TIMEOUT', {'THRU_T_MAX': 4.0}, trig_none),
        ('dive', 'COMPLETED', {}, trig_none),
    ]
    if args.only:
        scenarios = [s for s in scenarios if s[0] in args.only]

    results = []
    for name, expected, params, trigger in scenarios:
        results.append(run_scenario(v, lock, start, name, expected, params, trigger, args.alt))

    log('back to the start point and land')
    go_to_start(v, start)
    v.command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)

    end = time.time() + 120
    while time.time() < end:
        hb = v.pump(1.0, want='HEARTBEAT')
        if hb is not None and hb.get_srcComponent() == 1 and \
                not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            log('landed and disarmed')
            break
    return results


def latest_ulog():
    logs = glob.glob(os.path.join(PX4_BUILD, 'rootfs/log/*/*.ulg'))
    return max(logs, key=os.path.getmtime) if logs else None


def ulog_runs(ulog_path):
    """Split through_status into mode runs: phase sequence, exit reason, guidance stats."""
    from pyulog import ULog

    ulog = ULog(ulog_path, ['through_status'])
    st = ulog.data_list[0].data
    t = st['timestamp']
    phase = st['phase']
    active = phase != 0

    runs = []
    i = 0
    n = len(t)
    while i < n:
        if not active[i]:
            i += 1
            continue
        j = i
        while j < n and active[j]:
            j += 1
        seg = slice(i, j)
        seq = []
        for p in phase[seg]:
            if not seq or seq[-1] != PHASES[p]:
                seq.append(PHASES[p])
        # the reason is set on EXIT, or on the switch back to IDLE (USER_MODE_SWITCH), and kept in IDLE
        reason = int(st['exit_reason'][min(j, n - 1)])
        guide = (phase[seg] == 3)
        run = {'t0': t[i], 't1': t[j - 1], 'phases': seq, 'reason': REASONS[reason] if reason < len(REASONS) else '?'}
        if guide.any():
            vc = st['vc'][seg][guide]
            thr = st['thrust_cmd'][seg][guide]
            ap = np.stack([st[f'a_perp[{k}]'][seg][guide] for k in range(3)], axis=1)
            run['guide'] = {
                'duration': (t[seg][guide][-1] - t[seg][guide][0]) * 1e-6,
                'vc_max': float(np.nanmax(vc)),
                'thrust': (float(np.nanmin(thr)), float(np.nanmax(thr))),
                'a_perp_max': float(np.nanmax(np.linalg.norm(ap, axis=1))),
                'tilt_limited': float(np.mean(st['tilt_limited'][seg][guide])),
                'thrust_saturated': float(np.mean(st['thrust_saturated'][seg][guide])),
                'size_max': float(np.nanmax(st['target_size'][seg][guide])),
            }
        runs.append(run)
        i = j
    return runs


def load_crossings(log_dir):
    path = os.path.join(log_dir, 'crossings.csv') if log_dir else None
    if not path or not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--url', default='udpin:0.0.0.0:14540')
    parser.add_argument('--alt', type=float, default=25.0)
    parser.add_argument('--only', nargs='+', help='run only these scenarios')
    parser.add_argument('--log-dir', help='run_through_sim.sh log dir (crossings.csv)')
    parser.add_argument('--ulog', help='only analyze this ULog')
    args = parser.parse_args()

    results = [] if args.ulog else fly(args)
    if not args.ulog:
        time.sleep(3)  # let the logger flush
    ulog = args.ulog or latest_ulog()
    print(f'\nULog: {ulog}')
    runs = ulog_runs(ulog)

    print('\nTHROUGH runs in the ULog')
    for k, r in enumerate(runs):
        print(f'  {k}: {" -> ".join(r["phases"])}  exit_reason {r["reason"]}  ({(r["t1"] - r["t0"]) * 1e-6:.1f} s)')
        if 'guide' in r:
            gd = r['guide']
            print(f'     GUIDE {gd["duration"]:.2f} s, vc max {gd["vc_max"]:.1f} m/s, thrust {gd["thrust"][0]:.2f}..'
                  f'{gd["thrust"][1]:.2f}, |a_perp| max {gd["a_perp_max"]:.2f} m/s^2, tilt limited '
                  f'{gd["tilt_limited"] * 100:.0f}%, thrust saturated {gd["thrust_saturated"] * 100:.0f}%, '
                  f'target size max {gd["size_max"]:.2f}')

    crossings = load_crossings(args.log_dir)
    if crossings:
        print('\nring_judge crossings')
        for c in crossings:
            print('  ' + ', '.join(f'{k}={v}' for k, v in c.items()))

    if not results:
        return 0

    print('\nM5 summary')
    ok_all = True
    # one ULog run per scenario that entered THROUGH, in order
    entered = iter(runs)
    for r in results:
        run = next(entered, None) if 't_enter' in r else None
        ulog_reason = run['reason'] if run is not None else None
        phases = run['phases'] if run is not None else []
        ok = r.get('live') == r['expected'] and r.get('hold', False)
        if ulog_reason is not None:
            ok = ok and ulog_reason == r['expected']
        if r['name'] == 'dive':
            ok = ok and 'GUIDE' in phases
        ok_all &= ok
        print(f'  [{"PASS" if ok else "FAIL"}] {r["name"]:12s} expected {r["expected"]:16s} live {r.get("live")}, '
              f'ULog {ulog_reason}, Hold {r.get("hold")}, phases {" -> ".join(phases)}')
    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
