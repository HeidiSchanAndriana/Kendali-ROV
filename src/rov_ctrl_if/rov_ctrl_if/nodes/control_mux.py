import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import ManualControl
from mavros_msgs.msg import State

def scale_unit_to_1000(u: float) -> int:
    # u in [-1.0, 1.0] -> [0, 1000], 0 -> 500
    val = int(500 + 500 * max(-1.0, min(1.0, u)))
    return max(0, min(1000, val))

class ControlMux(Node):
    def __init__(self):
        super().__init__('control_mux')
        self.declare_parameter('ns', '/mavros/mavros_uas1')
        self.declare_parameter('cmd_topic', '/control/cmd')  # geometry_msgs/Twist
        self.declare_parameter('enable_deadman', True)
        self.declare_parameter('deadman_timeout', 1.0)

        self.ns = self.get_parameter('ns').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        self.enable_deadman = self.get_parameter('enable_deadman').get_parameter_value().bool_value
        self.deadman_timeout = self.get_parameter('deadman_timeout').get_parameter_value().double_value

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.last_cmd_time = self.get_clock().now()
        self.connected = False

        self.sub_state = self.create_subscription(State, f'{self.ns}/state', self.on_state, qos)
        self.sub_cmd = self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.pub_mc = self.create_publisher(ManualControl, f'{self.ns}/manual_control/send', 10)
        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz

        self.get_logger().info(f'ControlMux ready: listening {cmd_topic}, publishing {self.ns}/manual_control/send')

    def on_state(self, msg: State):
        self.connected = msg.connected

    def on_cmd(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.publish_mc(msg)

    def publish_mc(self, msg: Twist):
        out = ManualControl()
        out.x = float(scale_unit_to_1000(msg.linear.x))
        out.y = float(scale_unit_to_1000(msg.linear.y))
        out.z = float(scale_unit_to_1000(-msg.linear.z))  # heave: +up -> invert
        out.r = float(scale_unit_to_1000(msg.angular.z))
        out.buttons = 0
        self.pub_mc.publish(out)

    def neutral(self):
        out = ManualControl()
        out.x = 500.0; out.y = 0.0; out.z = 500.0; out.r = 0.0; out.buttons = 0
        self.pub_mc.publish(out)

    def tick(self):
        if not self.connected:
            self.neutral()
            return
        if self.enable_deadman:
            dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if dt > self.deadman_timeout:
                self.neutral()

def main():
    rclpy.init()
    node = ControlMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
