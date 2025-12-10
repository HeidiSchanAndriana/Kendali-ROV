import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from importlib import import_module
import json

def load_class(path: str):
    mod, cls = path.split(':')
    return getattr(import_module(mod), cls)

class AlgRunner(Node):
    def __init__(self):
        super().__init__('alg_runner')
        self.declare_parameter('algorithm_class', 'rov_ctrl_alg_template.algorithms.pid_hold_depth:PIDHoldDepth')
        self.declare_parameter('cmd_topic', '/control/cmd')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('params', '{}')

        alg_class_path = self.get_parameter('algorithm_class').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter('rate_hz').get_parameter_value().double_value)
        params_str = self.get_parameter('params').get_parameter_value().string_value or "{}"
        try:
            alg_params = json.loads(params_str)
        except Exception:
            alg_params = {}

        AlgClass = load_class(alg_class_path)
        self.alg = AlgClass(alg_params)

        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        self.sub_state = self.create_subscription(State, '/mavros/mavros_uas1/state', self.on_state, 10)

        period = 1.0 / max(1e-3, self.rate_hz)
        self.timer = self.create_timer(period, self.tick)
        self.t0 = self.get_clock().now()

        self.get_logger().info(f'AlgRunner using {alg_class_path} at {self.rate_hz} Hz → {self.cmd_topic}')

    def on_state(self, msg: State):
        if hasattr(self.alg, 'on_state'):
            self.alg.on_state(msg)

    def tick(self):
        now = self.get_clock().now()
        now_sec = (now - self.t0).nanoseconds * 1e-9
        cmd = self.alg.update(now_sec)
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init()
    node = AlgRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
