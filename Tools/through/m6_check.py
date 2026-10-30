#!/usr/bin/env python3
"""M6 acceptance for the THROUGH mode: TERMINAL + full ring passes.

Run with the stack from run_through_sim.sh (real vision node and ring_judge.py, which
writes crossings.csv into the log directory). Uses MAVLink 14540 and the
/through_vision/set_lock service through rclpy, so the ROS 2 workspace must be sourced.

Standard scene (spec 15.1, D8): hover at --alt (25 m) facing the ring, default parameters.
Each of the --runs dives: fly back to the start point, lock, switch to THROUGH, wait for
the exit. Between dives the vehicle first climbs straight up to --alt and only then flies
back, so the return path does not cross the ring.

Pass criterion: at least --min-pass of --runs dives with a ring_judge PASS (forward
crossing inside the ring opening minus the vehicle radius).
"""

import argparse
import csv
import math
import os
import sys
import time

import numpy as np

from m2_acceptance import Vehicle, log, mavutil
from m5_check import (HOLD_CUSTOM_MODE, PHASES, REASONS, THROUGH_CUSTOM_MODE, Lock, command_int, exit_text,
                      go_to_start, latest_ulog, set_mode_hold)


def climb_in_place(v, start):
    """Climb vertically to the start altitude before flying back (avoid the ring)."""
    if v.custom_mode != HOLD_CUSTOM_MODE:
        set_mode_hold(v)
    g = v.m.messages.get('GLOBAL_POSITION_INT')
    if g is None:
        return False
    command_int(v, mavutil.mavlink.MAV_CMD_DO_REPOSITION, mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                -1, 1, 0, start['yaw'], g.lat, g.lon, start['amsl'])
    return v.wait(lambda: v.local_pos is not None and abs(v.local_pos.z - start['ned'][2]) < 1.0, 30.0)


def dive(v, lock, start, k):
    log(f'=== dive {k + 1} ===')
    result = {'run': k + 1}
    climb_in_place(v, start)
    if not go_to_start(v, start):
        log('  could not reach the start point')

    if not lock.set(True):
        result['error'] = 'lock refused'
        log('  lock refused')
        return result

    v.wait(lambda: False, 1.0)
    v.status_texts.clear()
    v.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 11)
    if not v.wait(lambda: v.custom_mode == THROUGH_CUSTOM_MODE, 3.0):
        result['error'] = 'THROUGH not entered'
        lock.set(False)
        return result

    result['entered'] = True
    got_exit = v.wait(lambda: exit_text(v) is not None, 40.0)
    result['live'] = exit_text(v) if got_exit else None
    result['hold'] = v.wait(lambda: v.custom_mode == HOLD_CUSTOM_MODE, 3.0)
    log(f'  exit: {result["live"]}, Hold: {result["hold"]}')
    lock.set(False)
    v.wait(lambda: False, 3.0)  # let the vehicle stop before climbing
    return result


def fly(args):
    lock = Lock()
    v = Vehicle(args.url)
    v.m.mav.command_long_send(v.sys, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                              mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000, 0, 0, 0, 0, 0)
    v.set_param('MIS_TAKEOFF_ALT', args.alt)
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

    results = [dive(v, lock, start, k) for k in range(args.runs)]

    log('back to the start point and land')
    climb_in_place(v, start)
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


def ulog_runs(ulog_path):
    """Mode runs from through_status, with TERMINAL entry details and the low point after the exit."""
    from pyulog import ULog

    ulog = ULog(ulog_path, ['through_status', 'vehicle_local_position'])
    d = {x.name: x.data for x in ulog.data_list}
    st, lp = d['through_status'], d['vehicle_local_position']
    t, phase = st['timestamp'], st['phase']
    act = (phase != 0).astype(int)
    starts = np.where(np.diff(act) == 1)[0] + 1
    ends = np.where(np.diff(act) == -1)[0] + 1

    runs = []
    for a, b in zip(starts, ends):
        seq = []
        for p in phase[a:b]:
            if not seq or seq[-1] != PHASES[p]:
                seq.append(PHASES[p])
        run = {'t0': int(t[a]), 't1': int(t[b]), 'phases': seq, 'reason': REASONS[int(st['exit_reason'][b])]}

        term = np.where(phase[a:b] == 4)[0]
        if len(term):
            i = a + term[0]
            # TERMINAL by size (target valid and above THRU_TERM_SZ) or after losing a near target
            run['term_by'] = 'size' if st['target_valid'][i] else 'lost'
            run['term_size'] = float(np.nanmax(st['target_size'][a:i + 1]))

        w = (lp['timestamp'] > t[b]) & (lp['timestamp'] < t[b] + 8_000_000)
        run['alt_min_after'] = float(-lp['z'][w].max()) if w.any() else math.nan
        runs.append(run)
    return runs


def load_crossings(path):
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--url', default='udpin:0.0.0.0:14540')
    parser.add_argument('--alt', type=float, default=25.0)
    parser.add_argument('--runs', type=int, default=10)
    parser.add_argument('--min-pass', type=int, default=8)
    parser.add_argument('--log-dir', required=True, help='run_through_sim.sh log dir (crossings.csv)')
    parser.add_argument('--ulog', help='only analyze this ULog')
    args = parser.parse_args()

    if not args.ulog:
        fly(args)
        time.sleep(3)  # let the logger and ring_judge flush
    ulog = args.ulog or latest_ulog()
    print(f'\nULog: {ulog}')
    runs = ulog_runs(ulog)
    crossings = [c for c in load_crossings(os.path.join(args.log_dir, 'crossings.csv'))
                 if c['direction'] == 'forward']

    print('\nM6 dives')
    n_pass = 0
    for k, r in enumerate(runs):
        # forward crossing while the mode still runs (the switch back to IDLE lags the exit a little);
        # later crossings are Hold drifting through the ring, not the dive
        hits = [c for c in crossings if r['t0'] <= int(c['time_us']) <= r['t1'] + 500_000]
        c = hits[0] if hits else None
        result = c['result'] if c else 'NO CROSSING'
        n_pass += result == 'PASS'
        term = f'TERMINAL by {r["term_by"]} (size max {r["term_size"]:.2f})' if 'term_by' in r else 'no TERMINAL'
        cross = (f'r {float(c["r_m"]):.3f} m, {float(c["speed_mps"]):.1f} m/s, '
                 f'{float(c["angle_to_normal_deg"]):.1f} deg to normal') if c else ''
        print(f'  {k + 1:2d}: {result:11s} {cross:42s} | {" -> ".join(r["phases"])}, exit {r["reason"]}, {term}, '
              f'lowest {r["alt_min_after"]:.1f} m after exit')

    ok = len(runs) > 0 and n_pass >= args.min_pass
    print(f'\nM6 summary: {n_pass}/{len(runs)} PASS (need {args.min_pass}) -> {"PASS" if ok else "FAIL"}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
