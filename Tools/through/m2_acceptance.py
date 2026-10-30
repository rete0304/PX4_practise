#!/usr/bin/env python3
"""M2 acceptance test for the THROUGH mode (SITL).

Uses the same MAVLink path as QGroundControl (AVAILABLE_MODES + MAV_CMD_DO_SET_MODE):

  1. THROUGH appears in the dynamic mode list
  2. uncage = false  -> mode switch rejected (ack + status text)
  3. uncage = true   -> mode switch accepted
  4. hover in THROUGH (ALIGN) for 30 s, horizontal drift < 1 m
  5. switching away  -> mode leaves THROUGH (module stops publishing)

Prerequisites (each in its own terminal):
  PX4 SITL (gz_x500_through), MicroXRCEAgent udp4 -p 8888,
  ros_gz_bridge /clock, through_vision fake_vision_target (use_sim_time:=true).
This script toggles the lock through the fake node's ~/set_lock service, so the
ROS 2 workspace must be sourced in the shell that runs it.

Connects to the PX4 onboard MAVLink link (UDP 14540), so it can run while QGC
is connected on 14550.
"""

import argparse
import math
import subprocess
import sys
import os
import time

os.environ.setdefault('MAVLINK20', '1')  # EVENT messages need MAVLink 2
from pymavlink import mavutil  # noqa: E402

HOLD_MAIN, HOLD_SUB = 4, 3  # PX4_CUSTOM_MAIN_MODE_AUTO / PX4_CUSTOM_SUB_MODE_AUTO_LOITER

# events::ID("through_target_not_locked") (sub ID within the PX4 component, from all_events.json)
EVENT_THROUGH_TARGET_NOT_LOCKED = 0x05ca83


def log(msg):
    print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)


class Vehicle:

    def __init__(self, url):
        self.m = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        # wait for the autopilot itself (other components may also send heartbeats)
        end = time.time() + 30
        while time.time() < end:
            hb = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb is not None and hb.get_srcSystem() > 0 and hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                break
        else:
            raise RuntimeError('no PX4 heartbeat')
        self.sys = hb.get_srcSystem()
        self.m.target_system = self.sys
        self.m.target_component = 1
        self.status_texts = []
        self.event_ids = []
        self.custom_mode = None
        self.local_pos = None
        self.last_heartbeat = 0.0
        log(f'connected to system {self.sys}')

    def send_heartbeat(self):
        """Act as a GCS: without a GCS heartbeat PX4 refuses to arm (NAV_DLL_ACT)."""
        now = time.time()
        if now - self.last_heartbeat >= 1.0:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_heartbeat = now

    def pump(self, timeout=0.1, want=None):
        """Process incoming messages, return the first message of type `want`."""
        end = time.time() + timeout
        while True:
            self.send_heartbeat()
            msg = self.m.recv_match(blocking=True, timeout=max(0.0, end - time.time()))
            if msg is None:
                return None
            t = msg.get_type()
            if t == 'HEARTBEAT' and msg.get_srcComponent() == 1:
                self.custom_mode = msg.custom_mode
            elif t == 'STATUSTEXT':
                self.status_texts.append(msg.text)
                log(f'  STATUSTEXT: {msg.text}')
            elif t == 'EVENT':
                self.event_ids.append(msg.id & 0xFFFFFF)
            elif t == 'LOCAL_POSITION_NED':
                self.local_pos = msg
            if want is not None and t == want:
                return msg
            if time.time() >= end:
                return None

    def command(self, cmd, *params, timeout=3.0, settle=0.5):
        """Send COMMAND_LONG and return the result of the last ack received.

        PX4 mavlink_receiver acks DO_SET_MODE with ACCEPTED as soon as it forwards
        it, and the commander then sends its own (possibly rejecting) ack, so keep
        listening for `settle` seconds after the first ack.
        """
        p = list(params) + [0.0] * (7 - len(params))
        self.m.mav.command_long_send(self.sys, 1, cmd, 0, *p)
        result = None
        end = time.time() + timeout
        while time.time() < end:
            ack = self.pump(end - time.time(), want='COMMAND_ACK')
            if ack is not None and ack.command == cmd:
                if result is None:
                    end = min(end, time.time() + settle)
                result = ack.result
        return result

    def set_param(self, name, value, is_int=False):
        ptype = mavutil.mavlink.MAV_PARAM_TYPE_INT32 if is_int else mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        if is_int:
            # PX4 expects the int32 bits reinterpreted as float
            import struct
            value_f = struct.unpack('<f', struct.pack('<i', int(value)))[0]
        else:
            value_f = float(value)
        for _ in range(3):
            self.m.mav.param_set_send(self.sys, 1, name.encode(), value_f, ptype)
            msg = self.pump(2.0, want='PARAM_VALUE')
            if msg is not None and msg.param_id.rstrip('\x00') == name:
                log(f'  param {name} = {value}')
                return True
        log(f'  failed to set {name}')
        return False

    def wait(self, cond, timeout, period=0.1):
        end = time.time() + timeout
        while time.time() < end:
            self.pump(period)
            if cond():
                return True
        return False

    def available_modes(self):
        """Request AVAILABLE_MODES (id 435) for all modes."""
        modes = {}
        self.m.mav.command_long_send(self.sys, 1, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                     435, 0, 0, 0, 0, 0, 0)
        end = time.time() + 5.0
        total = None
        while time.time() < end:
            msg = self.pump(end - time.time(), want='AVAILABLE_MODES')
            if msg is None:
                break
            name = msg.mode_name.rstrip('\x00') if isinstance(msg.mode_name, str) else bytes(msg.mode_name).decode(errors='ignore').rstrip('\x00')
            modes[msg.mode_index] = (name, msg.custom_mode, msg.properties)
            total = msg.number_modes
            if total is not None and len(modes) >= total:
                break
        return modes

    def set_mode(self, custom_mode):
        main = (custom_mode >> 16) & 0xFF
        sub = (custom_mode >> 24) & 0xFF
        return self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub)


LOCK_SERVICE = '/fake_vision_target/set_lock'


def set_lock(locked):
    data = 'true' if locked else 'false'
    out = subprocess.run(['ros2', 'service', 'call', LOCK_SERVICE,
                          'std_srvs/srv/SetBool', f'{{data: {data}}}'],
                         capture_output=True, text=True, timeout=30)
    ok = 'success=True' in out.stdout
    log(f'  set_lock({locked}) -> {"ok" if ok else out.stdout.strip() or out.stderr.strip()}')
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--url', default='udpin:0.0.0.0:14540')
    parser.add_argument('--alt', type=float, default=10.0, help='takeoff altitude [m]')
    parser.add_argument('--hover', type=float, default=30.0, help='hover test duration [s]')
    parser.add_argument('--lock-service', default='/fake_vision_target/set_lock',
                        help='SetBool lock service (/through_vision/set_lock with the real vision node)')
    args = parser.parse_args()

    global LOCK_SERVICE
    LOCK_SERVICE = args.lock_service
    results = {}
    v = Vehicle(args.url)
    v.m.mav.request_data_stream_send(v.sys, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # ALIGN would time out (THRU_ALN_T_MAX = 8 s) before the 30 s hover test finishes:
    # extend the timeouts for this test only (M2 has no ALIGN -> RAMP transition yet).
    log('configure test parameters')
    v.set_param('THRU_ALN_T_MAX', args.hover + 15.0)
    v.set_param('THRU_T_MAX', args.hover + 30.0)
    v.set_param('MIS_TAKEOFF_ALT', args.alt)

    # 1. dynamic mode list
    log('1. dynamic mode list')
    modes = v.available_modes()
    through = [(idx, cm) for idx, (name, cm, _) in modes.items() if name == 'THROUGH']
    for idx in sorted(modes):
        log(f'  mode {idx}: {modes[idx][0]!r} custom_mode=0x{modes[idx][1]:08x}')
    results['1 THROUGH in mode list'] = bool(through)
    if not through:
        log('THROUGH not found, aborting')
        return report(results)
    through_mode = through[0][1]

    # take off
    log('arm and take off')
    set_lock(False)
    v.wait(lambda: False, 3.0)  # let the health checks see the GCS heartbeat
    # The commander sends the arming check events (incl. our failure reason) only when the
    # results change, e.g. at THRU_MIN_ALT during takeoff; QGC keeps them and appends them to
    # the "currently not possible" message. So collect them from takeoff on.
    v.event_ids.clear()
    if v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1) != 0:
        log('arming failed, aborting')
        return report(results)
    v.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
    if not v.wait(lambda: v.local_pos is not None and -v.local_pos.z > args.alt - 0.5, 60):
        log('takeoff did not reach altitude, aborting')
        return report(results)
    v.wait(lambda: False, 5)  # settle in Hold

    # 2. not locked -> rejected
    log('2. switch with uncage = false')
    v.status_texts.clear()
    v.wait(lambda: False, 1.0)  # let the next arming check (300 ms period) see the unlocked state
    r = v.set_mode(through_mode)
    v.wait(lambda: False, 1.0)
    rejected = (r != 0) and v.custom_mode != through_mode
    log(f'  ack result {r}, status texts: {v.status_texts}')
    results['2 rejected when not locked'] = rejected
    if hasattr(mavutil.mavlink, 'MAVLINK_MSG_ID_EVENT'):
        reason_sent = EVENT_THROUGH_TARGET_NOT_LOCKED in v.event_ids
        log(f'  "target not locked" event received since takeoff: {reason_sent}')
        results['2b rejection reason event sent'] = reason_sent
    else:
        # check the uORB 'event' topic in the ULog instead (through_target_not_locked)
        log('  SKIP reason event check: this pymavlink has no EVENT message definition')

    # 3. locked -> accepted
    log('3. switch with uncage = true')
    set_lock(True)
    v.wait(lambda: False, 1.0)
    r = v.set_mode(through_mode)
    entered = v.wait(lambda: v.custom_mode == through_mode, 3.0)
    log(f'  ack result {r}, in THROUGH: {entered}')
    results['3 accepted when locked'] = (r == 0) and entered
    if not entered:
        return report(results)

    # 4. hover
    log(f'4. hover {args.hover:.0f} s in THROUGH (ALIGN)')
    p0 = v.local_pos
    max_xy, max_dz, left = 0.0, 0.0, False
    end = time.time() + args.hover
    while time.time() < end:
        v.pump(0.2)
        if v.custom_mode != through_mode:
            left = True
            break
        p = v.local_pos
        max_xy = max(max_xy, math.hypot(p.x - p0.x, p.y - p0.y))
        max_dz = max(max_dz, abs(p.z - p0.z))
    log(f'  max horizontal drift {max_xy:.2f} m, max altitude change {max_dz:.2f} m, left mode early: {left}')
    results['4 hover drift < 1 m'] = (not left) and max_xy < 1.0

    # 5. switch away
    log('5. switch to Hold')
    v.status_texts.clear()
    t0 = time.time()
    v.set_mode((HOLD_SUB << 24) | (HOLD_MAIN << 16))
    left = v.wait(lambda: v.custom_mode != through_mode, 3.0)
    v.wait(lambda: False, 0.5)
    stopped = any('THROUGH stopped' in s for s in v.status_texts)
    log(f'  left THROUGH after {time.time() - t0:.2f} s, module reported stop: {stopped}')
    results['5 switching away stops the mode'] = left and stopped

    # land
    log('land')
    set_lock(False)
    v.command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
    return report(results)


def report(results):
    print()
    print('M2 acceptance summary')
    for k, ok in results.items():
        print(f'  [{"PASS" if ok else "FAIL"}] {k}')
    return 0 if results and all(results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
