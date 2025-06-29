# move_object_front.py

import rclpy
from rclpy.node import Node
import json

from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

class AutoParkingNode(Node):
    def __init__(self):
        super().__init__('auto_parking_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            String, '/detect/object_info', self.object_info_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/robot3/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.target_point = None
        self.is_parking = False

    def object_info_callback(self, msg):
        if self.is_parking:
            self.get_logger().info("이미 주차 진행 중 → 새 객체 무시")
            return

        try:
            objects = json.loads(msg.data)
            if not objects:
                return

            obj = objects[0]  # 첫 번째 객체만 사용
            u, v, distance = obj['center_x'], obj['center_y'], obj['distance']
            frame_id = obj['frame_id']

            # 픽셀 + 거리 → OAK-D 카메라 좌표 → PointStamped
            x, y, z = self.pixel_to_3d(u, v, distance)

            point_camera = PointStamped()
            point_camera.header.frame_id = frame_id
            point_camera.point.x = x
            point_camera.point.y = y
            point_camera.point.z = z

            # TF: base_link로 변환
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                point_base = tf2_geometry_msgs.do_transform_point(point_camera, tf)

                self.get_logger().info(f"객체 base_link: ({point_base.point.x:.2f}, {point_base.point.y:.2f})")

                # ➡️ 50cm 앞 좌표 계산 (base_link X축)
                target_x = point_base.point.x - 0.5
                target_y = point_base.point.y

                self.target_point = (target_x, target_y)
                self.is_parking = True

                self.get_logger().info(f"타겟 목표: base_link ({target_x:.2f}, {target_y:.2f})")

            except TransformException as e:
                self.get_logger().error(f"TF 변환 실패: {e}")

        except Exception as e:
            self.get_logger().error(f"JSON 파싱 실패: {e}")

    def pixel_to_3d(self, u, v, depth):
        # 카메라 내부 파라미터는 /camera_info 로 받아서 초기화돼 있다고 가정
        fx, fy = 600.0, 600.0  # 예시값, 실제 CameraInfo 사용!
        cx, cy = 320.0, 240.0

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z

    def control_loop(self):
        if not self.is_parking or self.target_point is None:
            return

        target_x, target_y = self.target_point

        distance = (target_x**2 + target_y**2)**0.5

        self.get_logger().info(f"남은 거리: {distance:.3f} m")

        twist = Twist()
        if distance > 0.05:
            twist.linear.x = 0.2  # 단순 전진 속도
            twist.angular.z = -1.0 * target_y  # 간단한 방향 보정
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.is_parking = False
            self.get_logger().info("목표점 도달! 이동 종료")

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutoParkingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
