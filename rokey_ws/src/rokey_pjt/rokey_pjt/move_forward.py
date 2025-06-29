# move_forward_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveForwardNode(Node):
    def __init__(self):
        super().__init__('move_forward_node')
        self.pub = self.create_publisher(Twist, '/robot3/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_forward)
        self.start_time = self.get_clock().now()
        self.duration = 2.5  # 초
        self.speed = 0.2     # m/s

    def move_forward(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        twist = Twist()
        if elapsed < self.duration:
            twist.linear.x = self.speed
        else:
            twist.linear.x = 0.0
            self.get_logger().info('목표 거리 이동 완료')
            self.destroy_timer(self.timer)

        self.pub.publish(twist)

def main():
    rclpy.init()
    node = MoveForwardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
