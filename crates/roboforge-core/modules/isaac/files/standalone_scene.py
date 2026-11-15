"""Minimal Isaac Sim standalone app. Runs in the dedicated Isaac environment."""
from isaacsim import SimulationApp

app = SimulationApp({'headless': False})

# Isaac modules can only be imported after SimulationApp exists.
import numpy as np  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid  # noqa: E402

world = World()
world.scene.add_default_ground_plane()
world.scene.add(
    DynamicCuboid(
        prim_path='/World/cube',
        name='cube',
        position=np.array([0.0, 0.0, 1.0]),
        scale=np.array([0.2, 0.2, 0.2]),
        color=np.array([0.9, 0.4, 0.1]),
    )
)
world.reset()

while app.is_running():
    world.step(render=True)

app.close()
