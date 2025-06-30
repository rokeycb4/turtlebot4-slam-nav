## carplate_ocr.py
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import pytesseract
from ultralytics import YOLO

import os
import sys

# ========================
# 상수 정의
# ========================

MODEL_PATH = '/home/kiwi/rokey_ws/car_plate1.pt'  # 👉 네가 준 경로로 변경
RGB_TOPIC = '/robot2/oakd/rgb/preview/image_raw'
CONF = 0.5  # YOLO 탐지 신뢰도 임계값

class CarPlateOCRNode(Node):
    def __init__(self):
        super().__init__('carplate_ocr_node')

        self.bridge = CvBridge()

        # YOLO 모델 로드
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)

        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f"YOLO 모델 로드 완료: {MODEL_PATH}")

        # Tesseract OCR 설정 (한글+영어)
        self.ocr_config = '--oem 3 --psm 7 -l kor+eng'

        # RGB 이미지 구독
        self.rgb_sub = self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 10)

        # OCR 결과 퍼블리셔
        self.ocr_pub = self.create_publisher(String, '/carplate/ocr_text', 10)

    def rgb_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(img, stream=True)

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < CONF:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # 번호판 ROI Crop
                plate_roi = img[y1:y2, x1:x2]

                if plate_roi.size == 0:
                    continue

                # Tesseract OCR 실행
                ocr_text = pytesseract.image_to_string(plate_roi, config=self.ocr_config)
                ocr_text = ocr_text.strip().replace('\n', '').replace(' ', '')

                if ocr_text:
                    msg_out = String()
                    msg_out.data = ocr_text
                    self.ocr_pub.publish(msg_out)
                    self.get_logger().info(f"OCR 결과: {ocr_text}")

                # 디버그: 박스 + OCR 결과 표시
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, ocr_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 디버그 창 출력
        cv2.imshow("Car Plate OCR", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CarPlateOCRNode()
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
