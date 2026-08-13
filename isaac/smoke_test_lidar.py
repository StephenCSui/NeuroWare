"""Lidar VRAM smoke test -- follow-up to smoke_test_depth_sensor.py (single
depth camera, 882MiB total). Measures whether an RTX Lidar sensor
(isaacsim.sensors.experimental.rtx.LidarSensor) is lighter than the depth
camera we're already using, before investing any effort in live
visualization or wiring it into the real robot script. If it's heavier,
there's no point switching.

Headless first, per the established pattern this session -- VRAM numbers
don't need a window to measure.
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={
    "headless": True,
    "renderer": "MinimalRendering",
    "minimal_shading_mode": 3,
})

import subprocess
import numpy as np
from pxr import UsdGeom, UsdLux
import omni.usd
import omni.timeline
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data


def gpu_mem():
    out = subprocess.run(
        ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
        capture_output=True, text=True,
    ).stdout.strip()
    return int(out.splitlines()[0])


print(f"[lidar smoke test] GPU mem after SimulationApp init: {gpu_mem()} MiB")

stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Cube.Define(stage, "/World/TestCube")
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(300)
GroundPlane("/World/GroundPlane", sizes=1000, colors="gray", templates=None)

for _ in range(5):
    simulation_app.update()

print(f"[lidar smoke test] GPU mem after scene setup: {gpu_mem()} MiB")

omni.timeline.get_timeline_interface().play()
for _ in range(5):
    simulation_app.update()

print("[lidar smoke test] creating Lidar prim (authoring only, no render product yet)...")
try:
    lidar_prim = Lidar.create(
        path="/World/Lidar",
        config="Example_Rotary_2D",
        translations=np.array([[0.0, 0.0, 0.3]]),
    )
    print("[lidar smoke test] Lidar prim created")
except Exception as e:
    print(f"[lidar smoke test] Lidar.create FAILED: {type(e).__name__}: {e}")
    lidar_prim = None

for _ in range(10):
    simulation_app.update()
print(f"[lidar smoke test] GPU mem after Lidar PRIM only (no render product/annotator): {gpu_mem()} MiB")

sensor = None
if lidar_prim is not None:
    try:
        sensor = LidarSensor(lidar_prim, annotators=["generic-model-output"])
        print("[lidar smoke test] LidarSensor (render product + annotator) created successfully")
    except Exception as e:
        print(f"[lidar smoke test] LidarSensor FAILED: {type(e).__name__}: {e}")

for _ in range(10):
    simulation_app.update()

print(f"[lidar smoke test] GPU mem after lidar created (render product + annotator): {gpu_mem()} MiB")

if sensor is not None:
    for _ in range(30):
        simulation_app.update()

    try:
        data, info = sensor.get_data("generic-model-output")
        print(f"[lidar smoke test] get_data -- data is None: {data is None}, info keys: {list(info.keys()) if info else None}")
        if data is not None:
            raw_bytes = int(np.prod(data.shape)) if getattr(data, "shape", None) else None
            print(f"[lidar smoke test] raw buffer: shape={getattr(data, 'shape', None)} ({raw_bytes} bytes)")
            gmo = parse_generic_model_output_data(data)
            print(f"[lidar smoke test] parsed GenericModelOutput -- numElements (actual points this frame): {gmo.numElements}")
            if raw_bytes and gmo.numElements:
                print(f"[lidar smoke test] bytes per point (buffer/numElements): {raw_bytes / gmo.numElements:.1f}")
            print(f"[lidar smoke test] our depth camera frame for comparison: 128*128*4 = {128*128*4} bytes, 16384 pixels")
    except Exception as e:
        print(f"[lidar smoke test] get_data FAILED: {type(e).__name__}: {e}")

print(f"[lidar smoke test] GPU mem after 30 more ticks (lidar live): {gpu_mem()} MiB")
print("[lidar smoke test] DONE")
simulation_app.close()
