#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

class DetectObjectsWithTF(Node):
    def __init__(self):
        super().__init__('detect_objects_with_tf')

        self.model_path = '/home/rokey/rokey_ws/src/yolov8_ros/yolov8_ros/best.pt'
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"YOLO 모델 로딩 완료: {self.model_path}")

        self.bridge = CvBridge()
        self.K = None
        self.rgb_image = None
        self.depth_image = None
        self.class_names = getattr(self.model, 'names', [])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CompressedImage, 
            '/robot3/oakd/rgb/image_raw/compressed', 
            self.rgb_compressed_callback, 
            10
        )

        self.create_subscription(
            Image, 
            '/robot3/oakd/stereo/image_raw', 
            self.depth_callback, 
            10
        )

        self.create_subscription(
            CameraInfo, 
            '/robot3/oakd/stereo/camera_info', 
            self.camera_info_callback, 
            10
        )

        self.pub_image = self.create_publisher(Image, '/detect/yolo_distance_image', 10)
        self.timer = self.create_timer(0.1, self.process_image)

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("CameraInfo 수신 완료")

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

    def process_image(self):
        if self.rgb_image is None or self.depth_image is None or self.K is None:
            return

        image = self.rgb_image.copy()
        depth = self.depth_image.copy()
        results = self.model.predict(source=image, conf=0.5, verbose=False)[0]

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

            point_cam = PointStamped()
            point_cam.header.frame_id = 'camera_link'
            point_cam.header.stamp = self.get_clock().now().to_msg()
            point_cam.point.x = x
            point_cam.point.y = y
            point_cam.point.z = z

            try:
                # camera_link -> base_link
                point_base = self.tf_buffer.transform(
                    point_cam, 
                    'base_link', 
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

                self.get_logger().info(
                    f"[{self.class_names[int(class_id)]}] conf={conf:.2f} | "
                    f"{point_cam.header.frame_id}({x:.2f}, {y:.2f}, {z:.2f}) → "
                    f"base_link({point_base.point.x:.2f}, {point_base.point.y:.2f}, {point_base.point.z:.2f})"
                )

                # base_link -> map
                tf_base_to_map = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                point_map = tf2_geometry_msgs.do_transform_point(point_base, tf_base_to_map)

                self.get_logger().info(
                    f"[base_link → map] (x={point_map.point.x:.2f}, y={point_map.point.y:.2f}, z={point_map.point.z:.2f})"
                )

            except Exception as e:
                self.get_logger().warn(
                    f"TF 변환 실패: {point_cam.header.frame_id} → base_link/map | 이유: {e}"
                )
                continue

            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                image, 
                f"{self.class_names[int(class_id)]} {conf:.2f}", 
                (x1, y1 - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.6, 
                (0, 0, 255), 
                2
            )
            cv2.putText(
                image, 
                f"{z:.2f}m", 
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.6, 
                (255, 0, 0), 
                2
            )

        self.publish_image(image)

    def publish_image(self, image):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.pub_image.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"이미지 퍼블리시 에러: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectObjectsWithTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
