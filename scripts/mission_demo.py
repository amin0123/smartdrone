#!/usr/bin/env python3
"""Start/observe a real ROS/PX4 mission and save evidence. Never generates flight setpoints."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState
from drone_interfaces.msg import VehicleState, MissionStatus, DockObservation
from drone_interfaces.srv import SetBattery
from drone_core.ros_support import SENSOR_QOS, LATCHED_QOS


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', choices=['normal', 'low-battery', 'emergency-battery'], default='normal')
    p.add_argument('--no-start', action='store_true', help='Observe the current mission only')
    p.add_argument('--timeout', type=float, default=600.)
    p.add_argument('--output', type=Path, default=Path('logs/mission-evidence.json'))
    args = p.parse_args()
    rclpy.init(args=[])
    node = Node('mission_demonstration')
    data = {'scenario': args.scenario, 'transitions': [], 'samples': [], 'visible_marker_count': 0,
            'battery_injected': False, 'passed': False}
    state, battery, status = None, None, None
    last_sample, last_state, terminal_at = 0., '', None
    def vehicle(msg):
        nonlocal state
        state = msg
    def power(msg):
        nonlocal battery
        battery = msg.percentage*100 if msg.present else None
    def mission(msg):
        nonlocal status, last_state, terminal_at
        status = msg
        if msg.mission_id and (msg.mission_id, msg.state) != last_state:
            data['mission_id'] = msg.mission_id
            data['transitions'].append({'state': msg.state, 'reason': msg.reason,
                'simulation_time': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, 'wall_time': time.time()})
            print(f'{msg.state}: {msg.reason}', flush=True)
            last_state = (msg.mission_id, msg.state)
            if msg.state in {'READY', 'MISSION_ABORTED'}:
                terminal_at = time.monotonic()
    def marker(msg):
        if msg.visible:
            data['visible_marker_count'] += 1
    node.create_subscription(VehicleState, '/drone/state', vehicle, SENSOR_QOS)
    node.create_subscription(BatteryState, '/drone/battery', power, SENSOR_QOS)
    node.create_subscription(MissionStatus, '/drone/mission_status', mission, LATCHED_QOS)
    node.create_subscription(DockObservation, '/drone/dock_observation', marker, SENSOR_QOS)
    start = node.create_client(Trigger, '/drone/start_mission')
    injection = node.create_client(SetBattery, '/drone/sim/set_battery')
    begin, started, inject_future = time.monotonic(), args.no_start, None
    start_future = None
    try:
        while time.monotonic()-begin < args.timeout:
            rclpy.spin_once(node, timeout_sec=.05)
            if not started and start.service_is_ready():
                start_future, started = start.call_async(Trigger.Request()), True
                # Ignore a latched status from an earlier sortie until start replies.
                data['transitions'], data['samples'], terminal_at = [], [], None
            if start_future is not None and start_future.done():
                response = start_future.result()
                if not response.success:
                    raise RuntimeError(response.message)
                start_future = None
            if status and not data['battery_injected'] and args.scenario != 'normal':
                phase = 'INSPECTION' if args.scenario == 'low-battery' else 'TAKEOFF'
                high_enough = state is not None and state.pose.position.z >= 1.
                if status.state == phase and high_enough and injection.service_is_ready():
                    value = 25. if args.scenario == 'low-battery' else 10.
                    inject_future = injection.call_async(SetBattery.Request(percentage=value))
                    data['battery_injected'] = True
                    print(f'Injecting simulated battery: {value}%', flush=True)
            if inject_future is not None and inject_future.done():
                if not inject_future.result().accepted:
                    raise RuntimeError(inject_future.result().message)
                inject_future = None
            if state and time.monotonic()-last_sample >= .2:
                point = state.pose.position
                data['samples'].append({'sim_time': state.header.stamp.sec+state.header.stamp.nanosec*1e-9,
                    'position_enu': [point.x, point.y, point.z], 'armed': state.armed,
                    'landed': state.landed, 'offboard': state.offboard, 'connected': state.connected,
                    'battery_percent': battery if battery is not None and math.isfinite(battery) else None})
                last_sample = time.monotonic()
            if terminal_at and start_future is None and time.monotonic()-terminal_at > 1.:
                settled = bool(state and state.connected and state.landed and not state.armed)
                expected = 'MISSION_ABORTED' if args.scenario == 'emergency-battery' else 'READY'
                data['passed'] = bool(status.state == expected and settled)
                if args.scenario == 'normal' and not args.no_start:
                    data['passed'] = data['passed'] and any(t['reason'] == 'coverage_complete' for t in data['transitions'])
                if args.scenario == 'low-battery':
                    data['passed'] = data['passed'] and data['battery_injected'] and any(
                        t['state'] == 'RETURN_HOME' and 'battery' in t['reason'] for t in data['transitions'])
                break
        else:
            data['error'] = 'Observer timeout; inspect mission state and abort service if required'
    except KeyboardInterrupt:
        data['error'] = 'Observer interrupted; mission continues until completion or abort request'
    except Exception as exc:
        data['error'] = str(exc)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(data, indent=2, allow_nan=False))
        print(f'Evidence: {args.output}; passed={data["passed"]}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if data['passed'] else 1)


if __name__ == '__main__':
    main()
