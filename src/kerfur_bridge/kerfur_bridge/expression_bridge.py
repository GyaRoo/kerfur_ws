import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
import websockets

from kerfur_msgs.msg import Expression


class ExpressionBridge(Node):
    """Bridges ROS2 /kerfur/expression to the FastAPI hub WebSocket."""

    def __init__(self):
        super().__init__('expression_bridge')

        # Parameters - configurable at launch time
        self.declare_parameter('hub_url', 'ws://localhost:8000/ws')
        self.hub_url = self.get_parameter('hub_url').value

        # ROS2 subscription
        self.subscription = self.create_subscription(
            Expression,
            '/kerfur/expression',
            self.on_expression,
            10
        )

        # Async loop for WebSocket - runs in its own thread
        self.loop = asyncio.new_event_loop()
        self.ws = None
        self.ws_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.ws_thread.start()

        self.get_logger().info(f'ExpressionBridge started, connecting to {self.hub_url}')

    def _run_async_loop(self):
        """Run the asyncio loop in a background thread."""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._maintain_connection())

    async def _maintain_connection(self):
        """Keep reconnecting to the hub if the connection drops."""
        while True:
            try:
                async with websockets.connect(self.hub_url) as ws:
                    self.ws = ws
                    self.get_logger().info('Connected to hub')
                    # Keep connection alive by reading incoming messages
                    async for message in ws:
                        # We don't currently use inbound messages but consume them
                        pass
            except Exception as e:
                self.get_logger().warn(f'Hub connection failed: {e}, retrying in 2s')
                self.ws = None
                await asyncio.sleep(2.0)

    def on_expression(self, msg: Expression):
        """Called when a ROS2 Expression message arrives."""
        self.get_logger().info(
            f'Expression: {msg.name} intensity={msg.intensity:.2f} duration={msg.duration_sec:.2f}'
        )

        # Translate to your existing browser protocol
        payload = {
            'type': 'setExpression',
            'name': msg.name,
            'intensity': float(msg.intensity),
            'duration_sec': float(msg.duration_sec)
        }

        # Forward over WebSocket (thread-safely)
        if self.ws is not None:
            asyncio.run_coroutine_threadsafe(
                self.ws.send(json.dumps(payload)),
                self.loop
            )
        else:
            self.get_logger().warn('Hub not connected, dropping expression')


def main():
    rclpy.init()
    node = ExpressionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
