## carplate_ocr.py

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import easyocr
from ultralytics import YOLO

import os
import sys

MODEL_PATH = '/home/kiwi/rokey_ws/car_plate1.pt'
MODEL_PATH = '/home/kiwi/rokey_ws/car_plate1.pt'

RGB_TOPIC = '/robot3/oakd/rgb/image_raw'
CONF = 0.5
PROCESS_INTERVAL_SEC = 0.2

class CarPlateOCRNode(Node):
    def __init__(self):
        super().__init__('carplate_ocr_node')

        self.bridge = CvBridge()
        self.latest_image = None

        # YOLO 모델 로드
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)

        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f"YOLO 모델 로드 완료: {MODEL_PATH}")

        # ✅ EasyOCR 리더 초기화
        self.reader = easyocr.Reader(['ko', 'en'])
        self.get_logger().info(f"EasyOCR 리더 로드 완료 (kor+eng)")

        # RGB 이미지 구독
        self.rgb_sub = self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 10)

        # OCR 결과 퍼블리셔
        self.ocr_pub = self.create_publisher(String, '/carplate/ocr_text', 10)

        # 이진화된 이미지 디버그 퍼블리셔 추가
        self.binary_img_pub = self.create_publisher(Image, '/carplate/debug/binary_image', 10)
        self.get_logger().info(f"이진화된 이미지 퍼블리셔 준비: {self.binary_img_pub.topic_name}")

        # 이미지 처리 타이머 생성
        self.timer = self.create_timer(PROCESS_INTERVAL_SEC, self.process_image_callback)
        self.get_logger().info(f"이미지 처리 타이머 시작: 매 {PROCESS_INTERVAL_SEC}초마다 실행")

    def rgb_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def process_image_callback(self):
        if self.latest_image is None:
            return

        img_display = self.latest_image.copy()
        img_process = self.latest_image.copy()
        
        results = self.model(img_process, stream=True)

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < CONF:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                plate_roi = img_process[y1:y2, x1:x2]
                if plate_roi.size == 0:
                    continue

                # --- 번호판 이미지 전처리 ---
                gray_roi = cv2.cvtColor(plate_roi, cv2.COLOR_BGR2GRAY)
                denoised_roi = cv2.medianBlur(gray_roi, 3)
                _, binary_roi = cv2.threshold(denoised_roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                ocr_image = binary_roi

                # 디버그 이진화 이미지 퍼블리시
                try:
                    binary_msg = self.bridge.cv2_to_imgmsg(binary_roi, encoding='mono8')
                    binary_msg.header.stamp = self.get_clock().now().to_msg()
                    self.binary_img_pub.publish(binary_msg)
                except Exception as e:
                    self.get_logger().error(f"Failed to publish binary image: {e}")

                # ✅ EasyOCR 실행
                result = self.reader.readtext(ocr_image, detail=0)
                ocr_text = ''.join(result).strip().replace(' ', '')

                # 특수문자 제거 (선택)
                for c in [':', '|', '(', ')', '[', ']', '{', '}', ';', '\'', '"', '~', '!']:
                    ocr_text = ocr_text.replace(c, '')

                if ocr_text:
                    msg_out = String()
                    msg_out.data = ocr_text
                    self.ocr_pub.publish(msg_out)
                    self.get_logger().info(f"OCR 결과: {ocr_text}")

                cv2.rectangle(img_display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img_display, ocr_text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("Car Plate OCR", img_display)
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
