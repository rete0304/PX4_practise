#!/usr/bin/env python3
"""Analyze THROUGH mode runs in a PX4 ULog (spec 14, M7).

For each mode run (through_status.phase != IDLE) plots, against the time since mode
entry with the phases shaded:

  phase and target validity | LOS elevation / azimuth | LOS rate | PN lateral
  acceleration a_perp and along-LOS s | thrust | tilt (setpoint, actual) |
  pan / tilt error and target size | closing velocity, speed and gimbal pitch

and prints per-run metrics used for tuning (LOS rate noise in ALIGN, PN demand
in GUIDE, limiter / saturation shares, TERMINAL entry, ring crossing).

Usage:
  analyze_through_log.py LOG.ulg [--run N ...] [--crossings crossings.csv] [-o OUT_DIR]

Without --run every run is analyzed. Figures are written as OUT_DIR/run_<N>.png
(default: next to the ULog).
"""

import argparse
import csv
import math
import os
import sys
import warnings

import numpy as np

# matplotlib in ~/.local next to the system one cannot load Axes3D; only 2D plots are used here
warnings.filterwarnings('ignore', message='Unable to import Axes3D')

PHASES = ['IDLE', 'ALIGN', 'RAMP', 'GUIDE', 'TERMINAL', 'EXIT']
REASONS = ['NONE', 'COMPLETED', 'USER_UNLOCK', 'USER_MODE_SWITCH', 'ALIGN_TIMEOUT', 'TARGET_LOST', 'MIN_ALT',
           'TILT_SAT', 'MODE_TIMEOUT', 'OPENING']
PHASE_COLORS = {1: '#dbe9f6', 2: '#fde8c8', 3: '#d9f0d3', 4: '#f6d5d5', 5: '#e6e6e6'}


def vec(data, name):
    return np.stack([data[f'{name}[{k}]'] for k in range(3)], axis=1)


def tilt_of_quat(q):
    """Angle between the body z axis and the world down axis for FRD->NED quaternions [w x y z]."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    r22 = 1.0 - 2.0 * (x * x + y * y)
    return np.degrees(np.arccos(np.clip(r22, -1.0, 1.0)))


class Log:

    def __init__(self, path):
        from pyulog import ULog

        topics = ['through_status', 'vehicle_attitude', 'vehicle_attitude_setpoint', 'vehicle_local_position']
        ulog = ULog(path, topics)
        self.path = path
        self.d = {x.name: x.data for x in ulog.data_list if x.multi_id == 0}
        for data in self.d.values():
            # signed: differences to the mode entry time must be able to go negative
            data['timestamp'] = data['timestamp'].astype(np.int64)
        if 'through_status' not in self.d:
            raise SystemExit(f'{path}: no through_status')

        # parameters at the start, updated by later changes (latest value before the run is used)
        self.params0 = dict(ulog.initial_parameters)
        self.param_changes = sorted(ulog.changed_parameters)

    def param(self, name, t, default=math.nan):
        value = self.params0.get(name, default)
        for tc, n, v in self.param_changes:
            if tc > t:
                break
            if n == name:
                value = v
        return value

    def runs(self):
        st = self.d['through_status']
        act = (st['phase'] != 0).astype(int)
        starts = list(np.where(np.diff(act) == 1)[0] + 1)
        ends = list(np.where(np.diff(act) == -1)[0] + 1)
        if act[0]:
            starts.insert(0, 0)
        if len(ends) < len(starts):
            ends.append(len(act) - 1)
        return list(zip(starts, ends))


def interp(t_src, y_src, t):
    return np.interp(t, t_src, y_src, left=np.nan, right=np.nan)


def run_metrics(log, a, b, crossings):
    st = log.d['through_status']
    t = st['timestamp']
    ph = st['phase'][a:b + 1]
    tt = t[a:b + 1]
    m = {'t0': int(t[a]), 'reason': REASONS[int(st['exit_reason'][b])]}

    seq = []
    for k in range(len(ph)):
        p = int(ph[k])
        if p == 0:
            continue
        if not seq or seq[-1][0] != p:
            seq.append([p, tt[k], tt[k]])
        seq[-1][2] = tt[k]
    m['phases'] = [(PHASES[p], (t1 - t0) * 1e-6) for p, t0, t1 in seq]

    omega = np.degrees(np.linalg.norm(vec(st, 'los_rate')[a:b + 1], axis=1))
    align = ph == 1
    guide = ph == 3
    if align.any():
        late_align = align & (tt > tt[align][0] + 500_000)  # filter warm-up
        if late_align.any():
            m['omega_align'] = (np.nanmean(omega[late_align]), np.nanpercentile(omega[late_align], 95))
    if guide.any():
        ap = np.linalg.norm(vec(st, 'a_perp')[a:b + 1], axis=1)
        m['guide'] = {
            'duration': (tt[guide][-1] - tt[guide][0]) * 1e-6,
            'omega_mean': float(np.nanmean(omega[guide])),
            'omega_p95': float(np.nanpercentile(omega[guide], 95)),
            'a_perp_mean': float(np.nanmean(ap[guide])),
            'a_perp_max': float(np.nanmax(ap[guide])),
            'a_perp_limited': float(np.mean(st['a_perp_limited'][a:b + 1][guide])),
            'tilt_limited': float(np.mean(st['tilt_limited'][a:b + 1][guide])),
            'thrust_saturated': float(np.mean(st['thrust_saturated'][a:b + 1][guide])),
            'vc_max': float(np.nanmax(st['vc'][a:b + 1][guide])),
        }

    term = np.where(ph == 4)[0]
    if len(term):
        i = a + term[0]
        m['term'] = {'by': 'size' if st['target_valid'][i] else 'lost',
                     'size': float(np.nanmax(st['target_size'][a:i + 1])), 't': int(t[i])}

    for c in crossings:
        tc = int(c['time_us'])
        if c['direction'] == 'forward' and m['t0'] <= tc <= int(t[b]) + 500_000:
            m['crossing'] = c
            if 'term' in m:
                # distance flown in TERMINAL before the ring plane
                m['term']['dist'] = float(c['speed_mps']) * (tc - m['term']['t']) * 1e-6
            break
    return m


def print_metrics(k, m):
    print(f'run {k}: ' + ' -> '.join(f'{n} {d:.2f}s' for n, d in m['phases']) + f', exit {m["reason"]}')
    if 'omega_align' in m:
        print(f'   ALIGN |omega_LOS| mean {m["omega_align"][0]:.2f} deg/s, p95 {m["omega_align"][1]:.2f} deg/s '
              f'(noise at rest)')
    if 'guide' in m:
        g = m['guide']
        print(f'   GUIDE {g["duration"]:.2f} s: |omega_LOS| mean {g["omega_mean"]:.2f} p95 {g["omega_p95"]:.2f} deg/s, '
              f'|a_perp| mean {g["a_perp_mean"]:.2f} max {g["a_perp_max"]:.2f} m/s^2, vc max {g["vc_max"]:.1f} m/s')
        print(f'   limited: a_perp {g["a_perp_limited"] * 100:.0f}%, tilt {g["tilt_limited"] * 100:.0f}%, '
              f'thrust {g["thrust_saturated"] * 100:.0f}%')
    if 'term' in m:
        tm = m['term']
        dist = f', {tm["dist"]:.2f} m before the ring' if 'dist' in tm else ''
        print(f'   TERMINAL by {tm["by"]}, target size {tm["size"]:.2f}{dist}')
    if 'crossing' in m:
        c = m['crossing']
        print(f'   ring: {c["result"]} r {float(c["r_m"]):.3f} m, {float(c["speed_mps"]):.1f} m/s, '
              f'{float(c["angle_to_normal_deg"]):.1f} deg to normal')


def plot_run(log, k, a, b, m, out_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    st = log.d['through_status']
    t_all = st['timestamp']
    t0 = t_all[a]
    # 1 s of context before entry and after the exit
    sel = (t_all >= t0 - 1_000_000) & (t_all <= t_all[b] + 1_000_000)
    t = (t_all[sel] - t0) * 1e-6
    s = {key: val[sel] for key, val in st.items()}
    ph = s['phase']

    att = log.d.get('vehicle_attitude')
    att_sp = log.d.get('vehicle_attitude_setpoint')
    lp = log.d.get('vehicle_local_position')

    fig, axes = plt.subplots(8, 1, figsize=(11, 20), sharex=True)

    def shade(ax):
        start = 0
        for i in range(1, len(ph) + 1):
            if i == len(ph) or ph[i] != ph[start]:
                p = int(ph[start])
                if p in PHASE_COLORS:
                    ax.axvspan(t[start], t[min(i, len(t) - 1)], color=PHASE_COLORS[p], lw=0)
                start = i
        ax.grid(True, alpha=0.3)

    for ax in axes:
        shade(ax)

    ax = axes[0]
    ax.step(t, ph, where='post', color='k', label='phase')
    ax.plot(t, s['target_valid'] * 0.8, color='tab:green', lw=1, label='target_valid (0.8)')
    ax.plot(t, s['uncage'] * 0.6, color='tab:purple', lw=1, label='uncage (0.6)')
    ax.set_yticks(range(6))
    ax.set_yticklabels(PHASES)
    ax.set_title(f'{os.path.basename(log.path)} run {k}: ' + ' -> '.join(n for n, _ in m['phases'])
                 + f', exit {m["reason"]}' + (f', ring {m["crossing"]["result"]}' if 'crossing' in m else ''))
    ax.legend(loc='upper left', fontsize=8)

    ax = axes[1]
    ax.plot(t, np.degrees(s['los_elevation']), label='LOS elevation (+ up)')
    ax.plot(t, np.degrees(s['los_azimuth']), label='LOS azimuth')
    ax.set_ylabel('deg')
    ax.legend(loc='upper left', fontsize=8)

    ax = axes[2]
    w = np.degrees(vec(s, 'los_rate'))
    for c, n in enumerate('NED'):
        ax.plot(t, w[:, c], lw=0.8, label=f'omega_LOS {n}')
    ax.plot(t, np.linalg.norm(w, axis=1), color='k', label='|omega_LOS|')
    ax.set_ylabel('deg/s')
    ax.legend(loc='upper left', fontsize=8)

    ax = axes[3]
    ap = vec(s, 'a_perp')
    for c, n in enumerate('NED'):
        ax.plot(t, ap[:, c], lw=0.8, label=f'a_perp {n}')
    ax.plot(t, np.linalg.norm(ap, axis=1), color='k', label='|a_perp|')
    ax.plot(t, s['s_along'], color='tab:brown', label='s (along LOS)')
    a_max = log.param('THRU_A_MAX', t0)
    ax.axhline(a_max, color='r', ls='--', lw=0.8, label='THRU_A_MAX')
    lim = s['a_perp_limited'].astype(bool)
    if lim.any():
        ax.plot(t[lim], np.full(lim.sum(), a_max), 'r|', label='a_perp limited')
    ax.set_ylabel('m/s^2')
    ax.legend(loc='upper left', fontsize=8, ncol=2)

    ax = axes[4]
    ax.plot(t, s['thrust_cmd'], color='k', label='thrust cmd')
    ax.plot(t, s['hover_thrust'], color='tab:gray', ls=':', label='T_h (latched)')
    for name in ('THRU_THR_MIN', 'THRU_THR_BASE', 'THRU_THR_MAX'):
        ax.axhline(log.param(name, t0), ls='--', lw=0.8, color='tab:red' if name != 'THRU_THR_BASE' else 'tab:blue',
                   label=name)
    sat = s['thrust_saturated'].astype(bool)
    if sat.any():
        ax.plot(t[sat], s['thrust_cmd'][sat], 'rx', ms=3, label='saturated')
    ax.set_ylabel('norm')
    ax.legend(loc='upper left', fontsize=8, ncol=2)

    ax = axes[5]
    if att_sp is not None:
        tsp = (att_sp['timestamp'] - t0) * 1e-6
        w_sp = (tsp >= t[0]) & (tsp <= t[-1])
        ax.plot(tsp[w_sp], tilt_of_quat(np.stack([att_sp[f'q_d[{i}]'] for i in range(4)], axis=1))[w_sp],
                label='tilt setpoint')
    if att is not None:
        ta = (att['timestamp'] - t0) * 1e-6
        w_a = (ta >= t[0]) & (ta <= t[-1])
        ax.plot(ta[w_a], tilt_of_quat(np.stack([att[f'q[{i}]'] for i in range(4)], axis=1))[w_a], label='tilt')
    ax.axhline(log.param('THRU_TILT_MAX', t0), color='r', ls='--', lw=0.8, label='THRU_TILT_MAX')
    tl = s['tilt_limited'].astype(bool)
    if tl.any():
        ax.plot(t[tl], np.full(tl.sum(), log.param('THRU_TILT_MAX', t0)), 'r|', label='tilt limited')
    ax.set_ylabel('deg')
    ax.legend(loc='upper left', fontsize=8)

    ax = axes[6]
    ax.plot(t, np.degrees(s['pan_error']), label='pan error')
    ax.plot(t, np.degrees(s['tilt_error']), label='tilt error')
    ax.set_ylabel('deg')
    ax.legend(loc='upper left', fontsize=8)
    ax2 = ax.twinx()
    ax2.plot(t, s['target_size'], color='tab:red', lw=1, label='target size')
    ax2.axhline(log.param('THRU_TERM_SZ', t0), color='tab:red', ls='--', lw=0.8, label='THRU_TERM_SZ')
    ax2.set_ylabel('size')
    ax2.legend(loc='upper right', fontsize=8)

    ax = axes[7]
    ax.plot(t, s['vc'], label='vc (v . u)')
    if lp is not None:
        tl_ = (lp['timestamp'] - t0) * 1e-6
        w_l = (tl_ >= t[0]) & (tl_ <= t[-1])
        speed = np.sqrt(lp['vx'] ** 2 + lp['vy'] ** 2 + lp['vz'] ** 2)
        ax.plot(tl_[w_l], speed[w_l], label='speed')
        ax.plot(tl_[w_l], -lp['z'][w_l], label='altitude [m]')
    ax.axhline(log.param('THRU_V_MAX', t0), color='r', ls='--', lw=0.8, label='THRU_V_MAX')
    ax.set_ylabel('m/s, m')
    ax.legend(loc='upper left', fontsize=8)
    ax2 = ax.twinx()
    ax2.plot(t, np.degrees(s['gimbal_pitch_cmd']), color='tab:gray', lw=0.8, ls='--', label='gimbal cmd (+ down)')
    ax2.plot(t, np.degrees(s['gimbal_pitch_meas']), color='tab:gray', lw=0.8, label='gimbal meas')
    ax2.set_ylabel('gimbal deg')
    ax2.legend(loc='upper right', fontsize=8)
    ax.set_xlabel('time since mode entry [s]')

    fig.tight_layout()
    fig.savefig(out_path, dpi=90)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('ulog')
    parser.add_argument('--run', type=int, nargs='+', help='run indices (default: all)')
    parser.add_argument('--crossings', help='ring_judge crossings.csv')
    parser.add_argument('-o', '--out-dir', help='figure directory (default: next to the ULog)')
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args()

    log = Log(args.ulog)
    crossings = []
    if args.crossings and os.path.exists(args.crossings):
        with open(args.crossings) as f:
            crossings = list(csv.DictReader(f))

    runs = log.runs()
    if not runs:
        print('no THROUGH runs in the log')
        return 1

    out_dir = args.out_dir or os.path.splitext(args.ulog)[0] + '_through'
    if not args.no_plot:
        os.makedirs(out_dir, exist_ok=True)

    for k, (a, b) in enumerate(runs):
        if args.run and k not in args.run:
            continue
        m = run_metrics(log, a, b, crossings)
        print_metrics(k, m)
        if not args.no_plot:
            path = os.path.join(out_dir, f'run_{k}.png')
            plot_run(log, k, a, b, m, path)
            print(f'   figure: {path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
