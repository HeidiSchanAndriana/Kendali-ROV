import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool
class ModeArmingNode(Node):
    def __init__(self):
        super().__init__('mode_arming_node')
        ns=self.get_namespace()
        self.cli_mode=self.create_client(SetMode, f'{ns}/set_mode')
        self.cli_arm=self.create_client(CommandBool, f'{ns}/cmd/arming')
        self.get_logger().info('ModeArmingNode ready.')
def main():
    rclpy.init(); n=ModeArmingNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
