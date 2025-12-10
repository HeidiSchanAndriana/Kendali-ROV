import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
class SimpleForwardDemo(Node):
    def __init__(self):
        super().__init__('simple_forward_demo')
        self.pub=self.create_publisher(ManualControl,'/mavros/mavros_uas1/manual_control/send',10)
        self.timer=self.create_timer(0.1,self.tick); self.count=0
    def tick(self):
        m=ManualControl(); m.x=700.0 if self.count<20 else 500.0; m.y=0.0; m.z=500.0; m.r=0.0; m.buttons=0.0
        self.pub.publish(m); self.count=(self.count+1)%40
def main():
    rclpy.init(); n=SimpleForwardDemo(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
