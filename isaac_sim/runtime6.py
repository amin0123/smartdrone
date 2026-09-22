"""Small explicit adapter for upstream Pegasus's unreleased Isaac 6 port."""
import omni.usd
import omni.timeline
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.rendering_manager import RenderingManager


class World6:
    def __init__(self, app, dt):
        self.app = app
        self.stage = omni.usd.get_context().get_stage()
        SimulationManager.setup_simulation(dt=dt, device='cpu')

    @property
    def current_time(self):
        return SimulationManager.get_simulation_time()

    def reset(self):
        omni.timeline.get_timeline_interface().play()
        self.app.update()

    def step(self, render=True):
        SimulationManager.step(steps=1)
        if render:
            RenderingManager.render()
