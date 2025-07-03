# yolo_detect.py

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import String
import json

class DetectAllObjectsWithDistance(Node):
    def __init__(self):
        super().__init__('detect_all_objects_with_distance')

        self.model_path = '/home/rokey/rokey_ws/park_area.pt'
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"YOLO 모델 로딩 완료: {self.model_path}")

        self.bridge = CvBridge()
        self.K = None  # CameraInfo 내부 파라미터
        self.rgb_image = None
        self.depth_image = None

        self.rgb_frame_id = None  # RGB 이미지의 frame_id 저장

        self.class_names = getattr(self.model, 'names', [])

        self.create_subscription(CompressedImage, '/robot3/oakd/rgb/image_raw/compressed', self.rgb_compressed_callback, 10)
        self.create_subscription(Image, '/robot3/oakd/stereo/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/robot3/oakd/stereo/camera_info', self.camera_info_callback, 10)

        self.pub_image = self.create_publisher(Image, '/detect/yolo_distance_image', 10)
        self.pub_objects = self.create_publisher(String, '/detect/object_info', 10)
        self.timer = self.create_timer(0.1, self.process_image)  # 10Hz

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("CameraInfo 수신 완료")

    def rgb_compressed_callback(self, msg):
        try:
            self.rgb_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rgb_frame_id = msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f'압축 RGB 변환 에러: {e}')

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth 변환 에러: {e}')

    def process_image(self):
        if self.rgb_image is None or self.depth_image is None or self.K is None or self.rgb_frame_id is None:
            return

        image = self.rgb_image.copy()
        depth = self.depth_image.copy()
        results = self.model.predict(source=image, conf=0.7, verbose=False)[0]

        boxes = results.boxes.xyxy.cpu().numpy()
        confidences = results.boxes.conf.cpu().numpy()
        class_ids = results.boxes.cls.cpu().numpy()
        
        object_info = []

        for idx in range(len(boxes)):
            x1, y1, x2, y2 = boxes[idx].astype(int)
            conf = confidences[idx]
            class_id = int(class_ids[idx])
            class_name = self.class_names[class_id] if class_id < len(self.class_names) else str(class_id)

            u, v = (x1 + x2) // 2, (y1 + y2) // 2
            if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
                continue

            # ROI 기반 거리 추정 (3x3)
            roi_size = 3  
            x_start = max(0, u - roi_size // 2)
            x_end = min(depth.shape[1], u + roi_size // 2 + 1)
            y_start = max(0, v - roi_size // 2)
            y_end = min(depth.shape[0], v + roi_size // 2 + 1)

            roi = depth[y_start:y_end, x_start:x_end].astype(np.float32)

            # 무효값 제외 (0 또는 NaN)
            roi_valid = roi[(roi > 0) & np.isfinite(roi)]

            if roi_valid.size > 0:
                distance_m = np.median(roi_valid) / 1000.0 if depth.dtype == np.uint16 else float(np.median(roi_valid))
            else:
                self.get_logger().warn(f"[{class_name}] 유효한 Depth 없음 → 건너뜀: ({u},{v})")
                continue

            # 유효 거리 범위 검사 (예: 0.3~4.0m)
            if distance_m < 0.3 or distance_m > 4.0:
                self.get_logger().warn(f"[{class_name}] 거리 범위 초과: {distance_m:.2f}m")
                continue

            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, f"{class_name} {conf:.2f}", (x1, y1 - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(image, f"{distance_m:.2f}m", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            object_info.append({
                'class_name': class_name,
                'confidence': float(conf),
                'center_x': int(u),
                'center_y': int(v),
                'distance': float(distance_m),
                'frame_id': self.rgb_frame_id
            })

            self.get_logger().info(f"[{class_name}] conf={conf:.2f} at ({u},{v}) → {distance_m:.2f}m frame_id={self.rgb_frame_id}")

        self.publish_image(image)
        
        if object_info:
            self.publish_object_info(object_info)

    def publish_image(self, image):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.pub_image.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"이미지 퍼블리시 에러: {e}")
    
    def publish_object_info(self, object_info):
        try:
            json_str = json.dumps(object_info)
            msg = String()
            msg.data = json_str
            self.pub_objects.publish(msg)
        except Exception as e:
            self.get_logger().error(f"객체 정보 퍼블리시 에러: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectAllObjectsWithDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
