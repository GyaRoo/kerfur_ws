import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
import websockets

from kerfur_msgs.msg import Expression, GazeTarget, Speech, ConsciousnessTrigger


class ExpressionBridge(Node):
    """Bridges ROS2 face topics to the FastAPI hub WebSocket.

    Forwards three message types to the browser over ONE hub connection:
      /kerfur/expression -> {type: "setExpression", ...}
      /kerfur/gaze       -> {type: "lookAt", x, y}
      /kerfur/speech     -> {type: "say", ...score...}
    The browser multiplexes by msg.type (see face.js handleCommand), so a single
    socket carrying all of them is exactly what the face expects. One connection
    avoids nodes competing over the hub socket - this node is the sole writer.
    """

    def __init__(self):
        super().__init__('expression_bridge')

        # Parameters - configurable at launch time
        self.declare_parameter('hub_url', 'ws://localhost:8000/ws')
        self.hub_url = self.get_parameter('hub_url').value
        self.wake_pub = self.create_publisher(ConsciousnessTrigger, '/kerfur/wake', 10)
        # ROS2 subscriptions
        self.expr_sub = self.create_subscription(
            Expression,
            '/kerfur/expression',
            self.on_expression,
            10
        )
        self.gaze_sub = self.create_subscription(
            GazeTarget,
            '/kerfur/gaze',
            self.on_gaze,
            10
        )
        self.speech_sub = self.create_subscription(
            Speech,
            '/kerfur/speech',
            self.on_speech,
            10
        )

        # Async loop for WebSocket - runs in its own thread
        self.loop = asyncio.new_event_loop()
        self.ws = None
        self.ws_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.ws_thread.start()

        self.get_logger().info(
            f'ExpressionBridge started (expression + gaze + speech), '
            f'connecting to {self.hub_url}'
        )

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
                    async for message in ws:
                        self._on_hub_message(message)                   
              
            except Exception as e:
                self.get_logger().warn(f'Hub connection failed: {e}, retrying in 2s')
                self.ws = None
                await asyncio.sleep(2.0)

    def _send(self, payload):
        """Forward a payload dict to the hub over the websocket (thread-safely)."""
        if self.ws is not None:
            asyncio.run_coroutine_threadsafe(
                self.ws.send(json.dumps(payload)),
                self.loop
            )
            return True
        return False

    def on_expression(self, msg: Expression):
        """Called when a ROS2 Expression message arrives."""
        self.get_logger().info(
            f'Expression: {msg.name} intensity={msg.intensity:.2f} duration={msg.duration_sec:.2f}'
        )
        payload = {
            'type': 'setExpression',
            'name': msg.name,
            'intensity': float(msg.intensity),
            'duration_sec': float(msg.duration_sec)
        }
        if not self._send(payload):
            self.get_logger().warn('Hub not connected, dropping expression')

    def on_gaze(self, msg: GazeTarget):
        """Called when a ROS2 GazeTarget message arrives. Forward as lookAt.

        Gaze arrives at the selector's publish rate (~20Hz). We do NOT log every
        one - that would flood the console. The browser smooths pupil motion.
        """
        payload = {
            'type': 'lookAt',
            'x': float(msg.x),
            'y': float(msg.y),
        }
        # Silently drop if the hub isn't connected - gaze is a continuous stream,
        # a dropped frame is invisible, and warning on every one would spam.
        self._send(payload)


    def on_speech(self, msg: Speech):
        """Called when a ROS2 Speech score arrives. Forward as a flat copy.

        Field names in Speech.msg match the JSON keys speech.js expects, so
        this is a copy, not a translation. ROS array fields arrive as
        array.array, which json.dumps rejects - hence the explicit list()s.
        """
        self.get_logger().info(
            f'Speech: "{msg.text}" ({len(msg.syllables)} syl, timbre={msg.timbre})'
        )
        payload = {
            'type': 'say',
            'text': msg.text,
            'syllables': list(msg.syllables),
            'pitch_hz': [float(x) for x in msg.pitch_hz],
            'duration_sec': [float(x) for x in msg.duration_sec],
            'gap_sec': [float(x) for x in msg.gap_sec],
            'gain': float(msg.gain),
            'timbre': msg.timbre,
        }
        if not self._send(payload):
            self.get_logger().warn('Hub not connected, dropping speech')
            

    def _on_hub_message(self, raw):
        """Inbound hub traffic. Only 'address' (typed text from /talk) is
        acted on; anything else on the socket is ignored."""
        try:
            msg = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(msg, dict) or msg.get('type') != 'address':
            return
        text = str(msg.get('text', '')).strip()
        if not text:
            return

    trig = ConsciousnessTrigger()
    trig.trigger_type = 'address'
    trig.content = text[:500]   # cap what a text box can push into the prompt
    trig.source = 'user'
    self.wake_pub.publish(trig)
    self.get_logger().info(f'Address from hub: "{trig.content}"')
    



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
