#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped, PointStamped
from std_msgs.msg import ColorRGBA, String
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import json

class ObjectPositionNode(Node):
    def __init__(self):
        super().__init__('object_position_node')

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_transform()

        self.K = None
        self.rgb_image = None
        self.depth_image = None
        self.object_info = None

        self.marker_id = 0
        self.target_frame = "map"
        self.use_map_frame = True

        # 구독
        # self.create_subscription(Image, '/robot3/oakd/rgb/image_raw', self.rgb_callback, 10)
        self.create_subscription(CompressedImage, '/robot3/oakd/rgb/image_raw/compressed', self.rgb_compressed_callback, 10)
        self.create_subscription(Image, '/robot3/oakd/stereo/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/robot3/oakd/stereo/camera_info', self.camera_info_callback, 10)
        self.create_subscription(String, '/detect/object_info', self.object_info_callback, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        self.timer = self.create_timer(0.1, self.process_objects)
        self.check_map_frame_timer = self.create_timer(5.0, self.check_map_frame)

    def check_map_frame(self):
        try:
            self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
            if not self.use_map_frame:
                self.use_map_frame = True
                self.target_frame = "map"
                self.get_logger().info("map 프레임 발견. map 프레임으로 마커를 발행합니다.")
        except TransformException:
            if self.use_map_frame:
                self.use_map_frame = False
                self.target_frame = "base_link"
                self.get_logger().warn("map 프레임을 찾을 수 없습니다. base_link 프레임으로 마커를 발행합니다.")

    def publish_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "base_link"
        static_transform.child_frame_id = "camera_link"
        static_transform.transform.translation.x = -0.1
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.2
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info("정적 TF 발행: base_link -> camera_link")

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("카메라 내부 파라미터 수신 완료")

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB 변환 에러: {e}')

    def rgb_compressed_callback(self, msg):
        try:
            self.rgb_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'압축 RGB 변환 에러: {e}')

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth 변환 에러: {e}')
            
    def object_info_callback(self, msg):
        try:
            self.object_info = json.loads(msg.data)
            self.get_logger().debug(f"객체 정보 수신: {len(self.object_info)}개 객체")
        except Exception as e:
            self.get_logger().error(f"객체 정보 파싱 에러: {e}")

    def process_objects(self):
        if self.K is None or self.rgb_image is None or self.depth_image is None or self.object_info is None:
            return

        marker_array = MarkerArray()
        
        for obj in self.object_info:
            try:
                class_name = obj['class_name']
                if "enemy" not in class_name and "fr" not in class_name:
                    continue
                u, v = obj['center_x'], obj['center_y']
                distance_m = obj['distance']
                x, y, z = self.pixel_to_3d(u, v, distance_m)
                self.add_marker(marker_array, x, y, z, class_name)
            except Exception as e:
                self.get_logger().error(f"객체 처리 에러: {e}")
        
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

    def add_marker(self, marker_array, x, y, z, class_name):
        try:
            point_camera = PointStamped()
            point_camera.header.stamp = self.get_clock().now().to_msg()
            point_camera.header.frame_id = "camera_link"
            point_camera.point.x = z
            point_camera.point.y = -x
            point_camera.point.z = -y
            target_frame = self.target_frame if self.use_map_frame else "base_link"

            try:
                point_transformed = self.tf_buffer.transform(
                    point_camera, target_frame, timeout=rclpy.duration.Duration(seconds=0.5))
            except TransformException as e:
                self.get_logger().warn(f"{target_frame}으로 변환 실패, base_link 사용: {e}")
                target_frame = "base_link"
                point_transformed = self.tf_buffer.transform(
                    point_camera, "base_link", timeout=rclpy.duration.Duration(seconds=0.5))

            marker = Marker()
            marker.header.frame_id = target_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "objects"
            marker.id = self.marker_id
            self.marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point_transformed.point.x
            marker.pose.position.y = point_transformed.point.y
            marker.pose.position.z = point_transformed.point.z
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            color = ColorRGBA()
            if "enemy" in class_name:
                color.r = 1.0
                color.g = 0.0
                color.b = 0.0
                color.a = 1.0
            elif "fr" in class_name:
                color.r = 0.0
                color.g = 0.0
                color.b = 1.0
                color.a = 1.0
            marker.color = color
            marker.lifetime.sec = 1
            marker_array.markers.append(marker)

            self.get_logger().info(f"마커 추가: {class_name} at ({marker.pose.position.x:.2f}, {marker.pose.position.y:.2f}, {marker.pose.position.z:.2f}) in {target_frame}")
        except Exception as e:
            self.get_logger().error(f"마커 생성 에러: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
