import rclpy
from rclpy.node import Node
from kerfur_msgs.msg import Detection
import serial

class NeckBridge(Node):
    def __init__(self):
        super().__init__("neck_bridge")
        # Serial to the Pico (adjust port to your actual device)
        self.pico = serial.Serial("/dev/ttyACM0", 115200, timeout=0.1)

        # Gaze smoothing state (the tracker logic, host-side)
        self.cur_yaw = 0.0
        self.cur_pitch = 0.0
        self.tgt_yaw = 0.0
        self.tgt_pitch = 0.0
        self.last_detection = self.get_clock().now()

        self.create_subscription(Detection, "/head/detection", self.on_detection, 10)
        # Timer drives the smoothing + serial send at a steady rate
        self.create_timer(0.03, self.tick)   # ~33Hz

        # Tunables (later: ROS params per CONVENTIONS; hardcoded for PoC)
        self.dead_zone = 0.05
        self.smooth = 0.15
        self.max_yaw = 0.6
        self.max_pitch = 0.6
        self.stale_sec = 1.0

    def on_detection(self, msg: Detection):
        # label "" = nothing salient -> don't update target, let it idle
        if msg.label == "":
            return
        # dead zone: only accept meaningfully-changed targets
        if abs(msg.x - self.tgt_yaw) > self.dead_zone:
            self.tgt_yaw = max(-self.max_yaw, min(self.max_yaw, msg.x))
        if abs(msg.y - self.tgt_pitch) > self.dead_zone:
            self.tgt_pitch = max(-self.max_pitch, min(self.max_pitch, msg.y))
        self.last_detection = self.get_clock().now()

    def tick(self):
        # Fail soft: if gaze is stale, ease toward neutral
        age = (self.get_clock().now() - self.last_detection).nanoseconds / 1e9
        if age > self.stale_sec:
            self.tgt_yaw = 0.0
            self.tgt_pitch = 0.0

        # Smooth toward target
        self.cur_yaw += (self.tgt_yaw - self.cur_yaw) * self.smooth
        self.cur_pitch += (self.tgt_pitch - self.cur_pitch) * self.smooth

        # Send pose to the Pico as a simple line: "yaw,pitch\n"
        line = "%.4f,%.4f\n" % (self.cur_yaw, self.cur_pitch)
        try:
            self.pico.write(line.encode())
        except Exception as e:
            self.get_logger().warn("Pico serial write failed: %s" % e)

def main():
    rclpy.init()
    node = NeckBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
