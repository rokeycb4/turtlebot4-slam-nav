#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
from ultralytics import YOLO

class DetectObjectsWithTF(Node):
    def __init__(self):
        super().__init__('detect_objects_with_tf')

        self.model = YOLO('/home/rokey/rokey_ws/park_area.pt')
        self.get_logger().info("YOLO 모델 로드 완료")

        self.bridge = CvBridge()
        self.K = None
        self.rgb_image = None
        self.depth_image = None
        self.class_names = getattr(self.model, 'names', [])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_transform()

        self.create_subscription(CompressedImage, '/robot3/oakd/rgb/image_raw/compressed', self.rgb_compressed_callback, 10)
        self.create_subscription(Image, '/robot3/oakd/stereo/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/robot3/oakd/stereo/camera_info', self.camera_info_callback, 10)
        
        self.pub_point_base = self.create_publisher(PointStamped, '/detect/point_base', 10)
        self.pub_point_map = self.create_publisher(PointStamped, '/detect/point_map', 10)
        self.pub_marker = self.create_publisher(MarkerArray, '/detect/object_markers', 10)

        self.marker_id = 0

        self.timer = self.create_timer(0.1, self.process_image)

    def publish_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'
        static_transform.child_frame_id = 'camera_link'
        static_transform.transform.translation.x = -0.1
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.2
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info("정적 TF 발행: base_link → camera_link")

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("CameraInfo 수신 완료")

    def rgb_compressed_callback(self, msg):
        try:
            self.rgb_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB 압축 해제 실패: {e}')

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth 해제 실패: {e}')

    def process_image(self):
        if self.rgb_image is None or self.depth_image is None or self.K is None:
            return

        image = self.rgb_image.copy()
        depth = self.depth_image.copy()
        results = self.model.predict(source=image, conf=0.5, verbose=False)[0]

        marker_array = MarkerArray()

        for box, conf, class_id in zip(
            results.boxes.xyxy.cpu().numpy(),
            results.boxes.conf.cpu().numpy(),
            results.boxes.cls.cpu().numpy()
        ):
            x1, y1, x2, y2 = map(int, box)
            u, v = (x1 + x2) // 2, (y1 + y2) // 2

            if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
                continue

            z = depth[v, u]
            if z == 0 or np.isnan(z):
                continue
            z = z / 1000.0 if depth.dtype == np.uint16 else float(z)

            fx, fy = self.K[0, 0], self.K[1, 1]
            cx, cy = self.K[0, 2], self.K[1, 2]
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            # ✅ ROS 축 변환: camera_link → base_link
            point_cam = PointStamped()
            point_cam.header.frame_id = 'camera_link'
            point_cam.header.stamp = self.get_clock().now().to_msg()
            point_cam.point.x = z         # 전방 → X
            point_cam.point.y = -x        # 오른쪽 → Y(-)
            point_cam.point.z = -y        # 아래 → Z(-)

            try:
                point_base = self.tf_buffer.transform(point_cam, 'base_link', timeout=rclpy.duration.Duration(seconds=0.5))
                self.pub_point_base.publish(point_base)
                
                class_name = self.class_names[int(class_id)]
                log_message = f"[{class_name}] base_link({point_base.point.x:.2f}, {point_base.point.y:.2f}, {point_base.point.z:.2f})"

                try:
                    tf_base_to_map = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
                    point_map = tf2_geometry_msgs.do_transform_point(point_base, tf_base_to_map)
                    self.pub_point_map.publish(point_map)

                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = 'objects'
                    marker.id = self.marker_id
                    self.marker_id += 1
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x = point_map.point.x
                    marker.pose.position.y = point_map.point.y
                    marker.pose.position.z = point_map.point.z
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.2
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                    marker.lifetime.sec = 1
                    marker_array.markers.append(marker)

                    log_message += f" → map({point_map.point.x:.2f}, {point_map.point.y:.2f}, {point_map.point.z:.2f})"

                except TransformException:
                    self.get_logger().warn("map 프레임 없음: base_link만 사용")

                self.get_logger().info(log_message)

            except Exception as e:
                self.get_logger().warn(f"TF 실패: {e}")
                continue

        if marker_array.markers:
            self.pub_marker.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = DetectObjectsWithTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
