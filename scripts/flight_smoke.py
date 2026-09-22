#!/usr/bin/env python3
"""Milestone 4: high-level MAVLink arm/takeoff/land against the real PX4 SITL.

Run without the ROS mission controller. This diagnostic never controls motors.
Normal autonomous missions use the ROS DDS interface instead.
"""
import argparse
import json
import math
from pathlib import Path
import time
from pymavlink import mavutil


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=14540)
    parser.add_argument('--altitude', type=float, default=3.)
    parser.add_argument('--output', type=Path, default=Path('logs/flight-smoke.json'))
    args = parser.parse_args()
    link = mavutil.mavlink_connection(f'udpin:127.0.0.1:{args.port}', source_system=244, source_component=190)
    latest = {}
    evidence = {'transport': 'MAVLink diagnostic only', 'events': [], 'passed': False}
    last_heartbeat = 0.

    def pump():
        nonlocal last_heartbeat
        if time.monotonic() - last_heartbeat > 1:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_heartbeat = time.monotonic()
        msg = link.recv_match(blocking=True, timeout=.1)
        if msg:
            latest[msg.get_type()] = msg
            if msg.get_type() == 'STATUSTEXT':
                print(msg.text, flush=True)
        return msg

    def wait(label, condition, timeout=60):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pump()
            if condition():
                evidence['events'].append({'event': label, 'time': time.time(),
                    'local_position': latest['LOCAL_POSITION_NED'].to_dict() if 'LOCAL_POSITION_NED' in latest else None})
                print(f'PASS {label}', flush=True)
                return
        raise TimeoutError(label)

    def command(command_id, *params):
        params = [*params] + [0.] * (7 - len(params))
        link.mav.command_long_send(link.target_system, link.target_component, command_id, 0, *params)

    try:
        wait('PX4 heartbeat', lambda: 'HEARTBEAT' in latest, 30)
        link.target_system = latest['HEARTBEAT'].get_srcSystem()
        link.target_component = 1
        for message_id in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                           mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                           mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
                           mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION):
            command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, message_id, 200000)
        wait('local and global position and home', lambda: all(k in latest for k in
             ('LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'HOME_POSITION')), 60)
        # Let the EKF settle using sensor data; never force-arm around checks.
        settle = time.monotonic() + 5
        while time.monotonic() < settle:
            pump()
        command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.)
        wait('armed', lambda: bool(latest['HEARTBEAT'].base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED), 15)
        # HOME_POSITION can precede EKF vertical-origin convergence. Use the
        # current estimator global altitude, and verify a relative local climb.
        start_z = latest['LOCAL_POSITION_NED'].z
        amsl = latest['GLOBAL_POSITION_INT'].alt / 1000 + args.altitude
        evidence['takeoff_request'] = {'start_ned_z': start_z, 'target_amsl': amsl,
                                      'relative_climb': args.altitude}
        command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0., 0., 0., math.nan, math.nan, math.nan, amsl)
        wait('takeoff altitude', lambda: latest.get('LOCAL_POSITION_NED') is not None and
             latest['LOCAL_POSITION_NED'].z < start_z-args.altitude+.3, 90)
        command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0., 0., 0., math.nan, math.nan, math.nan, math.nan)
        wait('landed and disarmed', lambda: 'EXTENDED_SYS_STATE' in latest and
             latest['EXTENDED_SYS_STATE'].landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND and
             not (latest['HEARTBEAT'].base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED), 90)
        evidence['passed'] = True
    except Exception as exc:
        evidence['error'] = str(exc)
        if 'HEARTBEAT' in latest and latest['HEARTBEAT'].base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            command(mavutil.mavlink.MAV_CMD_NAV_LAND, 0., 0., 0., math.nan, math.nan, math.nan, math.nan)
        raise
    finally:
        evidence['final_messages'] = {k: v.to_dict() for k, v in latest.items()
            if k in {'HEARTBEAT', 'LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'EXTENDED_SYS_STATE', 'COMMAND_ACK'}}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(evidence, indent=2))
        link.close()


if __name__ == '__main__':
    main()
