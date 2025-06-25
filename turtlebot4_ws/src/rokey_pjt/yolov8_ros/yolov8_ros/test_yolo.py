#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

class DetectEnemyYolo(Node):
    def __init__(self):
        super().__init__('detect_enemy_yolo')

        model_path = '/home/kiwi/rokey_ws/src/yolov8_ros/yolov8_ros/detect_mu2.pt'
        self.get_logger().info(f"모델 로딩 중: {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO 모델 로딩 완료")

        self.bridge = CvBridge()

        # 클래스 매핑
        self.class_mapping = {
            "0": "enemy",
            "1": "friendly"
        }

        self.subscription = self.create_subscription(
            Image,
            '/robot3/oakd/rgb/image_raw',
            self.image_callback,
            10
        )

        # 이미지만 퍼블리시
        self.pub_image = self.create_publisher(Image, '/detect/yolo_image', 10)

        self.last_image = None
        self.image_received = False
        self.timer = self.create_timer(0.1, self.process_image)  # 10Hz

    def image_callback(self, msg):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'CV Bridge 에러: {e}')
            self.image_received = False

    def process_image(self):
        if not self.image_received or self.last_image is None:
            return

        self.image_received = False
        cv_image = self.last_image.copy()

        # YOLO 추론 (confidence 0.2)
        results = self.model.predict(source=cv_image, conf=0.2, verbose=False)
        result = results[0]

        boxes = result.boxes.xyxy.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy()

        annotated_image = cv_image.copy()

        for idx in range(len(boxes)):
            x1, y1, x2, y2 = boxes[idx].astype(int)
            conf = confidences[idx]
            class_idx = int(class_ids[idx])
            class_name = self.class_mapping.get(str(class_idx), 'Unknown')

            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label_text = f"{class_name} {conf:.2f}"
            cv2.putText(annotated_image, label_text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        self.publish_image(annotated_image)

    def publish_image(self, image):
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.pub_image.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DetectEnemyYolo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
