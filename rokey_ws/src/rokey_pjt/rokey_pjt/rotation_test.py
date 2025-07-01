#!/usr/bin/env python3
# rotate_180.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Rotate180(Node):
    def __init__(self):
        super().__init__('rotate_180_node')

        # ✅ 네임스페이스 포함된 cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/robot3/cmd_vel', 10)

    def rotate(self):
        twist = Twist()
        twist.angular.z = 0.5  # rad/s, 반시계 방향

        duration = 3.141592 / 0.5  # π rad / 속도

        self.get_logger().info(f"제자리 180도 회전 시작 (예상 {duration:.2f}초)")

        start_time = self.get_clock().now().seconds_nanoseconds()[0]

        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        # 정지
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info("✅ 회전 완료, 로봇 정지!")

def main():
    rclpy.init()
    node = Rotate180()
    node.rotate()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
