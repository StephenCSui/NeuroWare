from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True, "renderer": "MinimalRendering"})

import isaacsim.core.experimental.utils.app as app_utils
app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

print("[diag ros2] importing rclpy...")
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
print(f"[diag ros2] rclpy imported OK, from: {rclpy.__file__}")

rclpy.init()
class DiagNode(Node):
    def __init__(self):
        super().__init__("diag_interop_node")
        self.got_msg = False
        self.sub = self.create_subscription(Empty, "diag_topic", self._cb, 10)
    def _cb(self, msg):
        self.got_msg = True
        print("[diag ros2] RECEIVED message on diag_topic!")

node = DiagNode()
print("[diag ros2] node created, spinning for 30s waiting for external ros2 topic pub...")
import time
start = time.time()
while time.time() - start < 30:
    simulation_app.update()
    rclpy.spin_once(node, timeout_sec=0.0)
    if node.got_msg:
        break
    time.sleep(0.05)

print(f"[diag ros2] FINAL got_msg={node.got_msg}")
node.destroy_node()
rclpy.shutdown()
simulation_app.close()
