import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import threading
import os
import sys

# ========================
# 상수 정의
# ========================
# YOLO_MODEL_PATH = '/home/mi/rokey_ws/model/yolov8n.pt'
YOLO_MODEL_PATH = '/home/kiwi/rokey_ws/src/yolov8_ros/yolov8_ros/detect_mu2.pt'  # YOLO 모델 경로
RGB_TOPIC = 'cropped/rgb/image_raw'
DEPTH_TOPIC = 'cropped/depth/image_raw'
CAMERA_INFO_TOPIC = 'cropped/camera_info'

# ROBOT_NAMESPACE = 'robot4'
# RGB_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/rgb/preview/image_raw'
# DEPTH_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/stereo/image_raw'
# CAMERA_INFO_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/stereo/camera_info'
TARGET_CLASS_ID = 0  # 예: car
# ========================

class YoloDepthDistance(Node):
    def __init__(self):
        super().__init__('yolo_depth_distance')
        self.get_logger().info("YOLO + Depth 거리 출력 노드 시작")

        # YOLO 모델 로드
        if not os.path.exists(YOLO_MODEL_PATH):
            self.get_logger().error(f"YOLO 모델이 존재하지 않습니다: {YOLO_MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(YOLO_MODEL_PATH)
        self.class_names = getattr(self.model, 'names', [])

        self.bridge = CvBridge()
        self.K = None
        self.rgb_image = None
        self.depth_image = None
        self.lock = threading.Lock()

        # ROS 구독자 설정
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 1)
        self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 1)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 1)

        # YOLO + 거리 출력 루프 실행
        threading.Thread(target=self.processing_loop, daemon=True).start()

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("CameraInfo 수신 완료")

    def rgb_callback(self, msg):
        with self.lock:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        with self.lock:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def processing_loop(self):
        cv2.namedWindow("YOLO Distance View", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            with self.lock:
                if self.rgb_image is None or self.depth_image is None or self.K is None:
                    continue
                rgb = self.rgb_image.copy()
                depth = self.depth_image.copy()

            results = self.model(rgb, stream=True)

            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    if cls != TARGET_CLASS_ID:
                        continue

                    # 중심 좌표
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2

                    if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
                        continue

                    # 거리 계산 (mm → m)
                    val = depth[v, u]
                    if depth.dtype == np.uint16:
                        distance_m = val / 1000.0
                    else:
                        distance_m = float(val)

                    label = self.class_names[cls] if cls < len(self.class_names) else f'class_{cls}'
                    self.get_logger().info(f"{label} at ({u},{v}) → {distance_m:.2f}m")

                    # RGB 이미지 위 시각화
                    cv2.rectangle(rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(rgb, (u, v), 4, (0, 0, 255), -1)
                    cv2.putText(rgb, f"{distance_m:.2f}m", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            cv2.imshow("YOLO Distance View", rgb)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

# ========================
# 메인 함수
# ========================
def main():
    rclpy.init()
    node = YoloDepthDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
