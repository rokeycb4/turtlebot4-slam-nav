#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import ColorRGBA
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
from ultralytics import YOLO

class YoloWithMarkers(Node):
    def __init__(self):
        super().__init__('yolo_with_markers')

        # YOLO 모델
        self.model = YOLO('/home/rokey/rokey_ws/src/yolov8_ros/yolov8_ros/best.pt')
        self.class_names = getattr(self.model, 'names', [])
        self.get_logger().info("YOLO 로딩 완료")

        # 기본값
        self.bridge = CvBridge()
        self.K = None
        self.rgb_image = None
        self.depth_image = None
        self.marker_id = 0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # 구독
        self.create_subscription(CompressedImage, '/robot3/oakd/rgb/image_raw/compressed', self.rgb_cb, 10)
        self.create_subscription(Image, '/robot3/oakd/stereo/image_raw', self.depth_cb, 10)
        self.create_subscription(CameraInfo, '/robot3/oakd/stereo/camera_info', self.camera_cb, 10)

        # 발행
        self.pub_image = self.create_publisher(Image, '/detect/yolo_distance_image', 10)
        self.pub_marker = self.create_publisher(MarkerArray, '/object_markers', 10)

        # 타이머
        self.timer = self.create_timer(0.1, self.process)

    def publish_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "base_link"
        static_transform.child_frame_id = "camera_link"
        static_transform.transform.translation.x = -0.1
        static_transform.transform.translation.z = 0.2
        static_transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info("정적 TF: base_link → camera_link")

    def camera_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("CameraInfo 수신")

    def rgb_cb(self, msg):
        self.rgb_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')

    def depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def process(self):
        if self.rgb_image is None or self.depth_image is None or self.K is None:
            return

        image = self.rgb_image.copy()
        depth = self.depth_image.copy()
        results = self.model.predict(source=image, conf=0.5, verbose=False)[0]

        marker_array = MarkerArray()
        self.marker_id = 0

        for box, conf, class_id in zip(
            results.boxes.xyxy.cpu().numpy(),
            results.boxes.conf.cpu().numpy(),
            results.boxes.cls.cpu().numpy()
        ):
            x1, y1, x2, y2 = map(int, box)
            u, v = (x1 + x2)//2, (y1 + y2)//2

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

            point_cam = PointStamped()
            point_cam.header.frame_id = "camera_link"
            point_cam.header.stamp = self.get_clock().now().to_msg()
            point_cam.point.x, point_cam.point.y, point_cam.point.z = z, -x, -y

            try:
                tf_point = self.tf_buffer.transform(point_cam, "base_link", timeout=rclpy.duration.Duration(seconds=0.5))
                marker = Marker()
                marker.header.frame_id = "base_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "objects"
                marker.id = self.marker_id
                self.marker_id += 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = tf_point.point.x
                marker.pose.position.y = tf_point.point.y
                marker.pose.position.z = tf_point.point.z
                marker.scale.x = marker.scale.y = marker.scale.z = 0.2
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                marker_array.markers.append(marker)

                self.get_logger().info(f"[{self.class_names[int(class_id)]}] {conf:.2f} {tf_point.point.x:.2f},{tf_point.point.y:.2f},{tf_point.point.z:.2f}")

                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            except Exception as e:
                self.get_logger().warn(f"TF 변환 실패: {e}")

        self.pub_image.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        if marker_array.markers:
            self.pub_marker.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = YoloWithMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
