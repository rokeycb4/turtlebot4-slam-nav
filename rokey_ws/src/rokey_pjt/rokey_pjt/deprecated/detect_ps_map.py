# yolo_tf_node.py

import rclpy
from rclpy.node import Node
import numpy as np
import json

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
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

        # 기존 String JSON 퍼블리셔 → 주석 처리
        # self.object_map_coordinates_pub = self.create_publisher(String, '/detect/object_map_coordinates', 10)

        # 새로 추가: PoseStamped 퍼블리셔
        self.pose_pub = self.create_publisher(PoseStamped, '/detect/object_map_pose', 10)

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

                point_camera = PointStamped()
                point_camera.header.frame_id = frame_id
                point_camera.point.x = x
                point_camera.point.y = y
                point_camera.point.z = z

                out_frame_id = 'map'
                point_transformed = PointStamped()

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
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            'base_link',
                            frame_id,
                            rclpy.time.Time(), 
                            timeout=rclpy.duration.Duration(seconds=1.0)
                        )
                        point_transformed = tf2_geometry_msgs.do_transform_point(point_camera, tf)
                        out_frame_id = 'base_link'
                    except TransformException as e_fallback:
                        self.get_logger().error(f"base_link 프레임 변환도 실패: {e_fallback}. 마커/포즈 발행 불가.")
                        continue

                # 마커 생성
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

                # PoseStamped 발행
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = out_frame_id
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.pose.position = point_transformed.point
                pose_msg.pose.orientation.w = 1.0  # 방향은 기본값
                self.pose_pub.publish(pose_msg)

            except Exception as e:
                self.get_logger().error(f"객체 처리 실패: {e}")

        if marker_array.markers:
            self.marker_pub.publish(marker_array)

        # 기존 JSON String 발행 → 주석 처리
        # if objects_for_publish:
        #     try:
        #         json_str = json.dumps(objects_for_publish)
        #         msg_to_publish = String()
        #         msg_to_publish.data = json_str
        #         self.object_map_coordinates_pub.publish(msg_to_publish)
        #     except Exception as e:
        #         self.get_logger().error(f"객체 맵 좌표 발행 실패: {e}")

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
