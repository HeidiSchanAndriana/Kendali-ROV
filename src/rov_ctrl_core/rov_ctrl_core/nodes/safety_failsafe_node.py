import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
from mavros_msgs.msg import State, ManualControl
class SafetyFailsafeNode(Node):
    def __init__(self):
        super().__init__('safety_failsafe_node')
        ns=self.get_namespace(); qos=QoSProfile(depth=10); qos.reliability=ReliabilityPolicy.BEST_EFFORT; qos.history=HistoryPolicy.KEEP_LAST
        self.last=self.get_clock().now(); self.timeout=2.0
        self.sub=self.create_subscription(State,f'{ns}/state',self.on_state,qos)
        self.pub=self.create_publisher(ManualControl,f'{ns}/manual_control/send',10)
        self.timer=self.create_timer(0.1,self.tick)
    def on_state(self,_): self.last=self.get_clock().now()
    def tick(self):
        if (self.get_clock().now()-self.last).nanoseconds*1e-9>self.timeout:
            m=ManualControl(); m.x=500.0; m.y=0.0; m.z=500.0; m.r=0.0; m.buttons=0.0; self.pub.publish(m)
def main():
    rclpy.init(); n=SafetyFailsafeNode(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
