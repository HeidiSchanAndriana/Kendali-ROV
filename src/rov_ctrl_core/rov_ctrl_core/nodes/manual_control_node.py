import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        ns=self.get_namespace()
        self.pub=self.create_publisher(ManualControl,f'{ns}/manual_control/send',10)
        self.timer=self.create_timer(0.1,self.tick); self._x=500.0
    def tick(self):
        m=ManualControl(); m.x=self._x; m.y=0.0; m.z=500.0; m.r=0.0; m.buttons=0; self.pub.publish(m)
def main():
    rclpy.init(); n=ManualControlNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
