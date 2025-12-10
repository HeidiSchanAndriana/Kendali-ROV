import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ExampleSquareSweep(Node):
    def __init__(self):
        super().__init__('example_square_sweep')
        self.pub = self.create_publisher(Twist, '/control/cmd', 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.step = 0
        self.count = 0

    def tick(self):
        msg = Twist()
        if self.step == 0:
            msg.linear.x = 0.6
        elif self.step == 1:
            msg.linear.y = 0.6
        elif self.step == 2:
            msg.linear.x = -0.6
        elif self.step == 3:
            msg.linear.y = -0.6
        self.pub.publish(msg)
        self.count += 1
        if self.count >= 30:
            self.count = 0
            self.step = (self.step + 1) % 4

def main():
    rclpy.init()
    node = ExampleSquareSweep()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
