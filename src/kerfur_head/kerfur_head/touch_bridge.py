"""Touch-bridge: receives debounced touch events from the native GPIO handler
over a local socket and publishes reflex PADNudges.

This is the reflex layer for touch - fast, no LLM. A pat produces an immediate
mood nudge. The native handler (outside the container) feeds this node.
"""

import json
import socket
import threading

import rclpy
from rclpy.node import Node

from kerfur_msgs.msg import PADNudge


class TouchBridge(Node):
    def __init__(self):
        super().__init__("touch_bridge")

        # Parameters - reflex nudge magnitudes, tunable via global config later
        self.declare_parameter("listen_host", "127.0.0.1")
        self.declare_parameter("listen_port", 8765)
        self.declare_parameter("pat_d_pleasure", 0.25)
        self.declare_parameter("pat_d_arousal", -0.05)
        self.declare_parameter("pat_d_dominance", -0.05)

        self.host = self.get_parameter("listen_host").value
        self.port = self.get_parameter("listen_port").value
        self.dp = self.get_parameter("pat_d_pleasure").value
        self.da = self.get_parameter("pat_d_arousal").value
        self.dd = self.get_parameter("pat_d_dominance").value

        self.nudge_pub = self.create_publisher(PADNudge, "/kerfur/pad_nudge", 10)

        # Socket server runs in a background thread so it doesn't block ROS spin
        self.sock_thread = threading.Thread(target=self._serve, daemon=True)
        self.sock_thread.start()

        self.get_logger().info(
            f"TouchBridge listening on {self.host}:{self.port}, "
            f"pat nudge=[{self.dp:+.2f},{self.da:+.2f},{self.dd:+.2f}]"
        )

    def _serve(self):
        """Accept a connection from the native handler and process its event stream."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)

        while rclpy.ok():
            try:
                conn, addr = srv.accept()
                self.get_logger().info(f"Touch handler connected from {addr}")
                self._handle_connection(conn)
            except OSError as e:
                self.get_logger().warn(f"Socket accept error: {e}")

    def _handle_connection(self, conn):
        """Read newline-delimited JSON events until the connection drops."""
        buf = ""
        with conn:
            while rclpy.ok():
                try:
                    data = conn.recv(1024)
                except OSError:
                    break
                if not data:
                    break  # handler disconnected
                buf += data.decode("utf-8")
                # Process complete lines
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line:
                        self._process_event(line)
        self.get_logger().info("Touch handler disconnected")

    def _process_event(self, line):
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Bad event JSON: {line!r}")
            return

        if event.get("type") == "touch":
            nudge = PADNudge()
            nudge.d_pleasure = float(self.dp)
            nudge.d_arousal = float(self.da)
            nudge.d_dominance = float(self.dd)
            nudge.mode_change = ""
            nudge.reason = "pat (reflex)"
            self.nudge_pub.publish(nudge)
            self.get_logger().info("Pat -> reflex nudge published")


def main():
    rclpy.init()
    node = TouchBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
