## yolo_tf

import rclpy
from rclpy.node import Node
import numpy as np
import json

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

class YOLOTFNode(Node):
    def __init__(self):
        super().__init__('yolo_tf_node')

        self.K = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, '/robot3/oakd/stereo/camera_info', self.camera_info_callback, 10)
        self.create_subscription(String, '/detect/object_info', self.object_info_callback, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/object_markers', 10)

        self.marker_id = 0

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("CameraInfo 수신 완료")

    def object_info_callback(self, msg):
        if self.K is None:
            self.get_logger().warn("CameraInfo 미수신. 변환 불가.")
            return

        try:
            objects = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"객체 정보 파싱 실패: {e}")
            return

        marker_array = MarkerArray()

        for obj in objects:
            try:
                u = obj['center_x']
                v = obj['center_y']
                depth = obj['distance']
                class_name = obj['class_name']
                frame_id = obj['frame_id']

                self.get_logger().info(f"수신된 객체 정보 frame_id: '{frame_id}'")

                x, y, z = self.pixel_to_3d(u, v, depth)

                self.get_logger().info(
                    f"OAK-D 광학 프레임 원시 좌표 ({class_name}): x={x:.3f}, y={y:.3f}, z={z:.3f}"
                ) # yolo_detect.py에 있던 이 로그를 yolo_tf.py로 옮겨옴 (확인용)


                point_camera = PointStamped()
                point_camera.header.frame_id = frame_id
                
                # 🚀 새로운 매핑 시도: 로봇 오른쪽으로 가는 문제 해결
                # OAK-D 원시: X-right, Y-down, Z-forward
                # ROS 표준 로봇: X-forward, Y-left, Z-up
                
                # ROS X (전방) <-> OAK-D Z (깊이)
                point_camera.point.x = z 
                
                # ROS Y (왼쪽) <-> OAK-D X (오른쪽)
                # OAK-D X가 양수일 때 ROS Y가 양수가 되어야 왼쪽으로 감
                # OAK-D X가 음수일 때 ROS Y가 음수가 되어야 오른쪽으로 감
                # 현재 문제가 "정면에 있는데 마커가 오른쪽으로 감" (base_link Y음수) 이므로
                # OAK-D x가 음수일 때 (카메라 기준 왼쪽) ROS Y도 음수가 되게 x를 그대로 씁니다.
                point_camera.point.y = x  # <-- 이 부분 변경 (이전에 시도했던 조합이지만, 진단 기반으로 다시 시도)
                
                # ROS Z (위쪽) <-> OAK-D Y (아래쪽)
                # OAK-D Y는 아래쪽이 양수, 위쪽이 음수이므로 부호 반전
                point_camera.point.z = -y 

                try:
                    tf = self.tf_buffer.lookup_transform(
                        'map',
                        frame_id,
                        rclpy.time.Time(), 
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    point_transformed = tf2_geometry_msgs.do_transform_point(point_camera, tf)
                    out_frame_id = 'map'
                except TransformException as e:
                    self.get_logger().warn(f"map 프레임 변환 실패 → base_link fallback: {e}")
                    tf = self.tf_buffer.lookup_transform(
                        'base_link',
                        frame_id,
                        rclpy.time.Time(), 
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    point_transformed = tf2_geometry_msgs.do_transform_point(point_camera, tf)
                    out_frame_id = 'base_link'

                marker = Marker()
                marker.header.frame_id = out_frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "objects"
                marker.id = self.marker_id
                self.marker_id += 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position = point_transformed.point
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2

                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                marker.lifetime.sec = 1
                marker_array.markers.append(marker)

                self.get_logger().info(
                    f"마커: {class_name} → ({marker.pose.position.x:.2f}, {marker.pose.position.y:.2f}, {marker.pose.position.z:.2f}) in {out_frame_id}"
                )

            except Exception as e:
                self.get_logger().error(f"객체 처리 실패: {e}")

        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    def pixel_to_3d(self, u, v, depth):
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z

def main(args=None):
    rclpy.init(args=args)
    node = YOLOTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()