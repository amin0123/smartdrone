#!/usr/bin/env python3
"""Run the physical Pegasus aircraft. No ROS code controls rotor forces."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--headless', action='store_true')
parser.add_argument('--no-px4', action='store_true', help='Spawn-only milestone; aircraft cannot fly')
parser.add_argument('--external-px4', action='store_true')
parser.add_argument('--duration', type=float, default=0., help='Simulation seconds; zero runs until closed')
parser.add_argument('--world', type=Path, default=ROOT / 'config/world.yaml')
parser.add_argument('--cameras', action='store_true')
args, kit_args = parser.parse_known_args()

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless, 'width': 1280, 'height': 720,
                     'anti_aliasing': 0, 'renderer': 'RaytracedLighting'})

# Isaac imports must follow SimulationApp initialization.
import numpy as np
import yaml
import omni.timeline
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.viewports import set_camera_view

enable_extension('omni.isaac.dynamic_control')
enable_extension('isaacsim.replicator.agent.core')
enable_extension('isaacsim.ros2.bridge')
app.update()

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.params import ROBOTS

sys.path.insert(0, str(ROOT / 'isaac_sim'))
from environments.field import create_field


def main() -> None:
    settings = yaml.safe_load(args.world.read_text())
    pg = PegasusInterface()
    world = World(physics_dt=settings['physics_dt'], rendering_dt=1 / 60,
                  stage_units_in_meters=1., device='cpu')
    # Pegasus exposes world read-only; its documented standalone example assigns
    # this backing member. Keep this upstream-version-specific use here only.
    pg._world = world
    labels = create_field(world, settings)
    (ROOT / 'logs/scene_labels.json').write_text(json.dumps(labels, indent=2))
    cfg = MultirotorConfig()
    cfg.backends = [] if args.no_px4 else [PX4MavlinkBackend(PX4MavlinkBackendConfig({
        'vehicle_id': 0, 'px4_autolaunch': False, 'enable_lockstep': True,
        'connection_ip': '127.0.0.1', 'px4_dir': str(ROOT / 'vendor/PX4-Autopilot'),
        'update_rate': 1 / settings['physics_dt'],
    }))]
    dock = settings['dock']
    vehicle = Multirotor('/World/quadrotor', ROBOTS['Iris'], 0,
        [dock[0], dock[1], dock[2] + settings['spawn_height']],
        [0., 0., 0., 1.], config=cfg)
    set_camera_view(eye=np.array([16., -18., 14.]), target=np.array([5., 0., 0.]))
    from sensors.ros_camera import RosCamera
    sensors = RosCamera(world, settings['camera'], enabled=args.cameras)
    process, log = None, None
    timeline = omni.timeline.get_timeline_interface()
    try:
        world.reset()
        if sensors:
            sensors.initialize()
        if not args.no_px4 and not args.external_px4:
            log = (ROOT / 'logs/px4-runtime.log').open('w')
            process = subprocess.Popen(['bash', str(ROOT / 'scripts/run_px4.sh'), '-d'],
                                       stdout=log, stderr=subprocess.STDOUT)
        timeline.play()
        print('DRONE_SIM_READY: physical Iris and agricultural field spawned', flush=True)
        start = time.monotonic()
        steps = 0
        while app.is_running():
            if args.duration and world.current_time >= args.duration:
                break
            if process and process.poll() is not None:
                raise RuntimeError(f'PX4 exited with {process.returncode}; see logs/px4-runtime.log')
            render = steps % settings['render_every'] == 0
            world.step(render=render)
            if sensors:
                sensors.step(world.current_time, render)
            steps += 1
            if settings.get('realtime', True):
                remaining = start + steps * settings['physics_dt'] - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
        position = vehicle.state.position.tolist()
        print('DRONE_SIM_FINISHED ' + json.dumps({'steps': steps, 'simulation_seconds': world.current_time,
              'position_enu': position}), flush=True)
    finally:
        timeline.stop()
        if sensors:
            sensors.close()
        if process and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if log:
            log.close()
        app.close()


if __name__ == '__main__':
    main()
