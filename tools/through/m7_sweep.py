#!/usr/bin/env python3
"""M7 tuning sweep for the THROUGH mode.

Run with the stack from run_through_sim.sh (real vision node and ring_judge.py). Flies one
dive per case and compares them: each case sets PX4 parameters (reset to the defaults
afterwards), optionally through_vision parameters, and a start point offset from the
standard hover point (25 m above the start, D8):

  std_*   standard start (spec 15.1)
  lat_*   start 5 m north of the standard start (spec 15.2, the PN has to correct)
  alt_*   start 10 m higher (spec 15.3, steeper dive)

Per dive: ring_judge result and miss distance, GUIDE metrics (LOS rate, a_perp, limiter
shares) and the LOS bias against the ring_judge truth at 2-5 m range.

Usage:
  m7_sweep.py --log-dir LOG_DIR [--cases std_base lat_n5 ...]
  m7_sweep.py --log-dir LOG_DIR --ulog LOG.ulg --cases ...   (analysis only, same case list)
"""

import argparse
import csv
import math
import os
import subprocess
import sys
import time
import warnings

import numpy as np

from analyze_through_log import Log, run_metrics, vec
from m2_acceptance import Vehicle, log, mavutil
from m5_check import HOLD_CUSTOM_MODE, THROUGH_CUSTOM_MODE, Lock, command_int, exit_text, latest_ulog, set_mode_hold

warnings.filterwarnings('ignore', message='Unable to import Axes3D')

DEFAULTS = {'THRU_PN_N': 3.5, 'THRU_LOS_LPF': 8.0, 'THRU_TERM_SZ': 0.6}
VISION_DEFAULTS = {'center_method': 'circle'}

# name: (PX4 params, vision params, start offset north/east/up [m])
CASES = {
    'std_base': ({}, {}, (0, 0, 0)),
    'std_n2.5': ({'THRU_PN_N': 2.5}, {}, (0, 0, 0)),
    'std_n5': ({'THRU_PN_N': 5.0}, {}, (0, 0, 0)),
    'std_lpf4': ({'THRU_LOS_LPF': 4.0}, {}, (0, 0, 0)),
    'std_lpf15': ({'THRU_LOS_LPF': 15.0}, {}, (0, 0, 0)),
    'std_tsz0.45': ({'THRU_TERM_SZ': 0.45}, {}, (0, 0, 0)),
    'std_tsz0.75': ({'THRU_TERM_SZ': 0.75}, {}, (0, 0, 0)),
    'std_moments': ({}, {'center_method': 'moments'}, (0, 0, 0)),
    'lat_base': ({}, {}, (5, 0, 0)),
    'lat_n2.5': ({'THRU_PN_N': 2.5}, {}, (5, 0, 0)),
    'lat_n5': ({'THRU_PN_N': 5.0}, {}, (5, 0, 0)),
    'lat_lpf4': ({'THRU_LOS_LPF': 4.0}, {}, (5, 0, 0)),
    'lat_lpf15': ({'THRU_LOS_LPF': 15.0}, {}, (5, 0, 0)),
    'latS_base': ({}, {}, (-5, 0, 0)),
    'alt_base': ({}, {}, (0, 0, 10)),
    'alt_n2.5': ({'THRU_PN_N': 2.5}, {}, (0, 0, 10)),
    'alt_n5': ({'THRU_PN_N': 5.0}, {}, (0, 0, 10)),
}

EARTH_RADIUS = 6378137.0


def offset_point(start, dn, de, du):
    lat = start['lat'] * 1e-7
    p = dict(start)
    p['lat'] = int(round((lat + math.degrees(dn / EARTH_RADIUS)) * 1e7))
    p['lon'] = int(round((start['lon'] * 1e-7 + math.degrees(de / (EARTH_RADIUS * math.cos(math.radians(lat))))) * 1e7))
    p['amsl'] = start['amsl'] + du
    p['ned'] = (start['ned'][0] + dn, start['ned'][1] + de, start['ned'][2] - du)
    return p


def reposition(v, p, timeout=60.0):
    command_int(v, mavutil.mavlink.MAV_CMD_DO_REPOSITION, mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                -1, 1, 0, p['yaw'], p['lat'], p['lon'], p['amsl'])

    def settled():
        lp = v.local_pos
        if lp is None:
            return False
        return math.dist((lp.x, lp.y, lp.z), p['ned']) < 0.5 and math.sqrt(lp.vx ** 2 + lp.vy ** 2 + lp.vz ** 2) < 0.2

    ok = v.wait(settled, timeout)
    v.wait(lambda: False, 3.0)  # heading and gimbal settle
    return ok


def go_to(v, start, p):
    """Climb in place to the higher of the two altitudes, fly over, then descend: never through the ring."""
    if v.custom_mode != HOLD_CUSTOM_MODE:
        set_mode_hold(v)
    g = v.m.messages.get('GLOBAL_POSITION_INT')
    safe_amsl = max(start['amsl'], p['amsl'])
    command_int(v, mavutil.mavlink.MAV_CMD_DO_REPOSITION, mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                -1, 1, 0, p['yaw'], g.lat, g.lon, safe_amsl)
    v.wait(lambda: v.local_pos is not None and v.local_pos.z < start['ned'][2] + 1.0 - (safe_amsl - start['amsl']), 30.0)
    return reposition(v, p)


RING_FROM_START = (0.0, 25.0, 15.0)  # ring center from the standard start: north, east, below [m]


def point_gimbal(v, offset):
    """Tilt the camera at the ring before locking: from a high start the ring is below the
    vertical field of view of the neutral gimbal. THROUGH takes the gimbal over on entry."""
    dn, de, du = offset
    horizontal = math.hypot(RING_FROM_START[0] - dn, RING_FROM_START[1] - de)
    pitch = -math.degrees(math.atan2(RING_FROM_START[2] + du, horizontal))  # MAVLink: - = down
    # primary control for this GCS (explicit ids: -2 would mean the autopilot itself), then pitch / yaw
    # relative to the vehicle
    v.command(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, v.m.source_system, v.m.source_component, -1, -1)
    v.command(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, pitch, 0, math.nan, math.nan, 0)
    v.wait(lambda: False, 2.0)


def set_vision_params(params):
    for name, value in params.items():
        subprocess.run(['ros2', 'param', 'set', '/through_vision', name, str(value)], capture_output=True, timeout=30)


def dive(v, lock, start, name):
    params, vparams, offset = CASES[name]
    log(f'=== {name} ===')
    result = {'case': name}
    p = offset_point(start, *offset)
    if not go_to(v, start, p):
        log('  start point not reached')

    for pname, value in params.items():
        v.set_param(pname, value)
    set_vision_params(vparams)
    if offset[2] > 0:
        point_gimbal(v, offset)

    if lock.set(True):
        v.wait(lambda: False, 1.0)
        v.status_texts.clear()
        v.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 11)
        if v.wait(lambda: v.custom_mode == THROUGH_CUSTOM_MODE, 3.0):
            result['entered'] = True
            v.wait(lambda: exit_text(v) is not None, 40.0)
            result['live'] = exit_text(v)
            v.wait(lambda: v.custom_mode == HOLD_CUSTOM_MODE, 3.0)
            log(f'  exit: {result["live"]}')
    else:
        log('  lock refused')
    lock.set(False)

    for pname in params:
        v.set_param(pname, DEFAULTS[pname])
    set_vision_params({k: VISION_DEFAULTS[k] for k in vparams})
    v.wait(lambda: False, 3.0)
    return result


def fly(args, cases):
    lock = Lock()
    v = Vehicle(args.url)
    v.m.mav.command_long_send(v.sys, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                              mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000, 0, 0, 0, 0, 0)
    v.set_param('MIS_TAKEOFF_ALT', args.alt)
    for pname, value in DEFAULTS.items():
        v.set_param(pname, value)
    set_vision_params(VISION_DEFAULTS)
    lock.set(False)
    v.wait(lambda: False, 3.0)

    log('arm and take off')
    if v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1) != 0:
        raise SystemExit('arming failed')
    v.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
    if not v.wait(lambda: v.local_pos is not None and -v.local_pos.z > args.alt - 0.5, 90):
        raise SystemExit('takeoff did not reach altitude')
    # the EKF heading is ~5 deg off right after takeoff and needs ~60 s in air to converge;
    # until then ALIGN cannot get |pan| below THRU_ALN_ERR (VERIFICATION.md 17)
    log(f'hover {args.settle:.0f} s for the EKF heading to converge')
    v.wait(lambda: False, args.settle)
    g = v.pump(2.0, want='GLOBAL_POSITION_INT')
    lp = v.local_pos
    start = {'lat': g.lat, 'lon': g.lon, 'amsl': g.alt * 1e-3, 'yaw': math.radians(g.hdg * 0.01),
             'ned': (lp.x, lp.y, lp.z)}

    results = [dive(v, lock, start, name) for name in cases]

    log('back to the start point and land')
    go_to(v, start, start)
    v.command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
    end = time.time() + 120
    while time.time() < end:
        hb = v.pump(1.0, want='HEARTBEAT')
        if hb is not None and hb.get_srcComponent() == 1 and \
                not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            log('landed and disarmed')
            break
    return results


def los_bias(ulg, t0, t1, truth):
    """Mean elevation / azimuth error of los_ned against the truth at 2-5 m range in GUIDE [deg]."""
    st = ulg.d['through_status']
    t, ph = st['timestamp'], st['phase']
    u = vec(st, 'los_ned')
    sel = (t >= t0) & (t <= t1) & (ph == 3) & st['target_valid'].astype(bool) & np.all(np.isfinite(u), axis=1)
    if not sel.any():
        return math.nan, math.nan
    tt, lt, rng = truth
    idx = np.clip(np.searchsorted(tt, t[sel]), 0, len(tt) - 1)
    near = (rng[idx] >= 2.0) & (rng[idx] <= 5.0)
    if not near.any():
        return math.nan, math.nan
    us, ls = u[sel][near], lt[idx][near]
    e_el = np.degrees(np.arcsin(-us[:, 2]) - np.arcsin(-ls[:, 2]))
    e_az = np.degrees(np.arctan2(us[:, 1], us[:, 0]) - np.arctan2(ls[:, 1], ls[:, 0]))
    return float(e_el.mean()), float(e_az.mean())


def analyze(ulog_path, log_dir, cases, results=None):
    ulg = Log(ulog_path)
    runs = ulg.runs()
    with open(os.path.join(log_dir, 'crossings.csv')) as f:
        crossings = list(csv.DictReader(f))
    with open(os.path.join(log_dir, 'los_truth.csv')) as f:
        rows = list(csv.DictReader(f))
    truth = (np.array([int(r['time_us']) for r in rows]),
             np.array([[float(r[k]) for k in ('los_n', 'los_e', 'los_d')] for r in rows]),
             np.array([float(r['range_m']) for r in rows]))

    # one ULog run per case that entered the mode
    entered = [c for k, c in enumerate(cases) if results is None or results[k].get('entered')]
    rows_out = []
    print(f'\n{"case":13s} {"ring":6s} {"r[m]":>6s} {"exit":10s} {"GUIDE s":>7s} {"w p95":>6s} {"ap mean":>7s} '
          f'{"ap max":>6s} {"tilt%":>5s} {"thr%":>4s} {"TERM":>11s} {"bias el/az @2-5m":>17s}')
    for name, (a, b) in zip(entered, runs):
        m = run_metrics(ulg, a, b, crossings)
        g = m.get('guide', {})
        c = m.get('crossing')
        el, az = los_bias(ulg, int(ulg.d['through_status']['timestamp'][a]),
                          int(ulg.d['through_status']['timestamp'][b]), truth)
        term = f'{m["term"]["by"]} {m["term"]["size"]:.2f}' if 'term' in m else '-'
        row = {
            'case': name, 'ring': c['result'] if c else 'NONE', 'r_m': float(c['r_m']) if c else math.nan,
            'exit': m['reason'], 'guide_s': g.get('duration', math.nan), 'omega_p95': g.get('omega_p95', math.nan),
            'a_perp_mean': g.get('a_perp_mean', math.nan), 'a_perp_max': g.get('a_perp_max', math.nan),
            'tilt_limited': g.get('tilt_limited', math.nan), 'thrust_saturated': g.get('thrust_saturated', math.nan),
            'term': term, 'bias_el': el, 'bias_az': az,
        }
        rows_out.append(row)
        print(f'{name:13s} {row["ring"]:6s} {row["r_m"]:6.3f} {row["exit"][:10]:10s} {row["guide_s"]:7.2f} '
              f'{row["omega_p95"]:6.2f} {row["a_perp_mean"]:7.2f} {row["a_perp_max"]:6.2f} '
              f'{row["tilt_limited"] * 100:5.0f} {row["thrust_saturated"] * 100:4.0f} {term:>11s} '
              f'{el:8.2f}/{az:6.2f}')

    out = os.path.join(log_dir, 'm7_sweep.csv')
    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows_out[0].keys()) if rows_out else ['case'])
        w.writeheader()
        w.writerows(rows_out)
    print(f'\nwritten {out}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--url', default='udpin:0.0.0.0:14540')
    parser.add_argument('--alt', type=float, default=25.0)
    parser.add_argument('--settle', type=float, default=60.0, help='hover time after takeoff [s]')
    parser.add_argument('--cases', nargs='+', default=list(CASES.keys()), choices=list(CASES.keys()))
    parser.add_argument('--log-dir', required=True)
    parser.add_argument('--ulog', help='only analyze this ULog')
    args = parser.parse_args()

    results = None
    if not args.ulog:
        results = fly(args, args.cases)
        time.sleep(3)
    analyze(args.ulog or latest_ulog(), args.log_dir, args.cases, results)
    return 0


if __name__ == '__main__':
    sys.exit(main())
