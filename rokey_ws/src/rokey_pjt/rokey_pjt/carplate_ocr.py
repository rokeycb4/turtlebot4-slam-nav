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
import time

# ========================
# 상수 정의
# ========================

MODEL_PATH = '/home/rokey/rokey_ws/car_plate1.pt'
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

        # Tesseract OCR 설정 (한글+영어)
        # 모든 숫자 (0-9)와 한글 '가', '주'만 포함하는 화이트리스트
        whitelist_chars = "가주0123456789"
        self.ocr_config = f'--oem 1 --psm 13 -l eng+kor -c tessedit_char_whitelist="{whitelist_chars}"'


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

                # 번호판 ROI Crop
                plate_roi = img_process[y1:y2, x1:x2]

                if plate_roi.size == 0:
                    continue

                # --- 번호판 이미지 전처리 시작 ---
                # 1. 그레이스케일 변환
                gray_roi = cv2.cvtColor(plate_roi, cv2.COLOR_BGR2GRAY)

                # 2. 노이즈 제거 (Median Blur)
                denoised_roi = cv2.medianBlur(gray_roi, 3)

                # 3. 이진화 (OTSU 알고리즘)
                _, binary_roi = cv2.threshold(denoised_roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                
                # 전처리된 이미지를 Tesseract에 전달
                ocr_image = binary_roi
                # --- 번호판 이미지 전처리 끝 ---

                # --- 이진화된 이미지 토픽 발행 (디버그용) ---
                try:
                    binary_msg = self.bridge.cv2_to_imgmsg(binary_roi, encoding='mono8')
                    binary_msg.header.stamp = self.get_clock().now().to_msg()
                    self.binary_img_pub.publish(binary_msg)
                except Exception as e:
                    self.get_logger().error(f"Failed to publish binary image: {e}")
                # --- 이진화된 이미지 토픽 발행 끝 ---


                # Tesseract OCR 실행
                ocr_text = pytesseract.image_to_string(ocr_image, config=self.ocr_config)
                ocr_text = ocr_text.strip().replace('\n', '').replace(' ', '')
                
                # 특수문자 제거 (이 부분은 화이트리스트가 적용되면 덜 필요할 수 있습니다)
                ocr_text = ocr_text.replace(':', '').replace('|', '').replace('(', '').replace(')', '')
                ocr_text = ocr_text.replace('[', '').replace(']', '').replace('{', '').replace('}', '')
                ocr_text = ocr_text.replace(';', '').replace('\'', '').replace('"', '').replace('~', '').replace('!', '')


                if ocr_text:
                    msg_out = String()
                    msg_out.data = ocr_text
                    self.ocr_pub.publish(msg_out)
                    self.get_logger().info(f"OCR 결과: {ocr_text}")

                # 디버그: 박스 + OCR 결과 표시 (원본 이미지에 그리기)
                cv2.rectangle(img_display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img_display, ocr_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 디버그 창 출력
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