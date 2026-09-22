#!/usr/bin/env python3
"""Inject a selected local process failure during SITL takeoff; verify PX4 lands.

The diagnostic subscribes to PX4 output directly so loss of the gateway cannot
hide the result. It never publishes a flight setpoint or a force-arm command.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleLandDetected
from drone_core.ros_support import SENSOR_QOS


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--target', choices=['mission', 'gateway', 'pause'], required=True)
    p.add_argument('--pid', type=int, required=True, help='Exact PID of this workspace node')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    root = Path(__file__).resolve().parents[1]
    command = Path(f'/proc/{args.pid}/cmdline').read_bytes().replace(b'\0', b' ').decode()
    package = 'drone_mission_manager' if args.target == 'mission' else 'drone_px4_interface'
    expected_path = (root / 'isaac_sim/scripts/run_simulation.py' if args.target == 'pause'
                     else root / 'ros2_ws/install' / package)
    if str(expected_path) not in command:
        raise SystemExit('PID does not belong to the requested local workspace node')
    # Require this project's actual SITL binary, not merely any ROS/PX4 link.
    binaries = [x / 'exe' for x in Path('/proc').iterdir() if x.name.isdigit()]
    sitl = root / 'vendor/PX4-Autopilot/build/px4_sitl_default/bin/px4'
    def is_project_sitl(path):
        try:
            return path.resolve(strict=True) == sitl
        except OSError:
            return False
    if not any(is_project_sitl(x) for x in binaries):
        raise SystemExit('This project SITL process is not running')
    rclpy.init(args=[])
    node = Node('sitl_process_failure_trial')
    status, position, landed = None, None, None
    seen = {'status': 0., 'position': 0., 'landed': 0.}
    def on_status(msg):
        nonlocal status
        status, seen['status'] = msg, time.monotonic()
    def on_position(msg):
        nonlocal position
        position, seen['position'] = msg, time.monotonic()
    def on_landed(msg):
        nonlocal landed
        landed, seen['landed'] = msg, time.monotonic()
    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', on_status, SENSOR_QOS)
    node.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', on_position, SENSOR_QOS)
    node.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', on_landed, SENSOR_QOS)
    start = node.create_client(Trigger, '/drone/start_mission')
    data = {'target': args.target, 'pid': args.pid, 'passed': False, 'samples': []}
    future, killed, saw_land, last_sample, resume_at = None, None, False, 0., None
    begin = time.monotonic()
    try:
        if not start.wait_for_service(timeout_sec=5.):
            raise RuntimeError('Mission service unavailable')
        future = start.call_async(Trigger.Request())
        while time.monotonic()-begin < 180.:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if resume_at is not None and now >= resume_at:
                os.kill(args.pid, signal.SIGCONT)
                resume_at = None
                data['resumed_at_elapsed'] = now-begin
                print('Resumed simulation after 3 s wall-time pause', flush=True)
            if future is not None and future.done():
                reply = future.result()
                if not reply.success:
                    raise RuntimeError(reply.message)
                future = None
            if not all((status, position, landed)) or any(now-t > 5. for t in seen.values()):
                continue
            armed = status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            point = [position.y, position.x, -position.z]
            if now-last_sample > .2:
                data['samples'].append({'wall_elapsed': now-begin, 'position_enu': point,
                    'armed': armed, 'landed': landed.landed, 'nav_state': status.nav_state,
                    'failsafe': status.failsafe})
                last_sample = now
            if killed is None and armed and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and point[2] >= 1.5:
                current = Path(f'/proc/{args.pid}/cmdline').read_bytes().replace(b'\0', b' ').decode()
                if current != command:
                    raise RuntimeError('Selected PID changed identity before injection')
                os.kill(args.pid, signal.SIGSTOP if args.target == 'pause' else signal.SIGKILL)
                if args.target == 'pause':
                    resume_at = now+3.
                killed = now
                data['injection'] = {'elapsed': now-begin, 'position_enu': point}
                print(f'Injected {args.target} failure into PID {args.pid} at {point}', flush=True)
            if killed is not None:
                saw_land |= status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
                if now-killed > 5. and landed.landed and not armed:
                    data['passed'] = saw_land
                    data['land_after_seconds'] = now-killed
                    data['final_position_enu'] = point
                    data['final_state'] = {'armed': armed, 'landed': landed.landed,
                        'nav_state': status.nav_state, 'failsafe': status.failsafe,
                        'position_enu': point, 'wall_elapsed': now-begin}
                    data['horizontal_displacement_m'] = math.dist(point[:2], data['injection']['position_enu'][:2])
                    break
        else:
            data['error'] = 'Timed out; inspect native PX4 state before restarting'
    except KeyboardInterrupt:
        data['error'] = 'Observer interrupted; inspect PX4 state'
    except Exception as exc:
        data['error'] = str(exc)
    finally:
        if resume_at is not None:
            os.kill(args.pid, signal.SIGCONT)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(data, indent=2, allow_nan=False))
        print(f'Evidence: {args.output}; passed={data["passed"]}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if data['passed'] else 1)


if __name__ == '__main__':
    main()
