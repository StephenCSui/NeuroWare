"""Opens the dedicated Robot1 + Cube_03 isolated test world in an
interactive Isaac Sim GUI window for visual confirmation -- not headless,
no autonomous script logic running."""
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": False,
    "renderer": "MinimalRendering",
    "minimal_shading_mode": 3,
    "width": 1280,
    "height": 800,
    "window_width": 1320,
    "window_height": 900,
})

import os
import isaacsim.core.experimental.utils.stage as stage_utils
from omni.kit.viewport.utility import get_active_viewport, frame_viewport_prims
import omni.timeline

USD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test", "robot1_cube03_test.usd")
success, stage = stage_utils.open_stage(USD_PATH)
print(f"[open robot1+cube03 world] opened {USD_PATH} -> {success}")

for _ in range(10):
    simulation_app.update()

frame_viewport_prims(get_active_viewport(), ["/World", "/Cube_03"])
omni.timeline.get_timeline_interface().play()

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
