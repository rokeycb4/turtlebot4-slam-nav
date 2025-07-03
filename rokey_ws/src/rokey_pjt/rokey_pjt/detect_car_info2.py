# detect_car_info2.py
# ros2 run rokey_pjt detect_car_info2 --ros-args -- --mode raw
# ros2 run rokey_pjt detect_car_info2 --ros-args -- --mode compressed
# ros2 run rokey_pjt detect_car_info2 --mode raw
# ros2 run rokey_pjt detect_car_info2 --mode compressed

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import easyocr
from ultralytics import YOLO

from PIL import Image as PILImage, ImageDraw, ImageFont

import numpy as np
import os
import sys
import json
import re
import argparse

MODEL_PATH = '/home/rokey/rokey_ws/car_plate2.pt'

RAW_TOPIC = '/robot2/oakd/rgb/image_raw'
COMPRESSED_TOPIC = '/robot2/oakd/rgb/image_raw/compressed'

CONF = 0.5
PROCESS_INTERVAL_SEC = 0.2

class CarPlateOCRNode(Node):
    def __init__(self, mode):
        super().__init__('carplate_ocr_node')

        self.bridge = CvBridge()
        self.latest_image = None
        self.mode = mode

        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)

        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f"YOLO 모델 로드 완료: {MODEL_PATH}")

        self.reader = easyocr.Reader(['ko'])
        self.get_logger().info(f"EasyOCR 리더 로드 완료 (kor)")

        if self.mode == 'compressed':
            self.rgb_sub = self.create_subscription(CompressedImage, COMPRESSED_TOPIC, self.compressed_rgb_callback, 10)
            self.get_logger().info(f"압축 이미지 구독: {COMPRESSED_TOPIC}")
        else:
            self.rgb_sub = self.create_subscription(Image, RAW_TOPIC, self.raw_rgb_callback, 10)
            self.get_logger().info(f"RAW 이미지 구독: {RAW_TOPIC}")

        self.ocr_result_pub = self.create_publisher(String, '/carplate/ocr_result', 10)
        self.binary_img_pub = self.create_publisher(Image, '/carplate/debug/binary_image', 10)
        self.result_img_pub = self.create_publisher(Image, '/carplate/debug/result_image', 10)

        self.get_logger().info(f"이진화 이미지 퍼블리셔: {self.binary_img_pub.topic_name}")
        self.get_logger().info(f"결과 이미지 퍼블리셔: {self.result_img_pub.topic_name}")
        self.get_logger().info(f"OCR JSON 퍼블리셔: {self.ocr_result_pub.topic_name}")

        self.timer = self.create_timer(PROCESS_INTERVAL_SEC, self.process_image_callback)
        self.get_logger().info(f"이미지 처리 타이머 시작: 매 {PROCESS_INTERVAL_SEC}초마다 실행")

        self.font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
        if not os.path.exists(self.font_path):
            self.get_logger().warn(f"폰트 경로 확인 필요: {self.font_path}")

    def raw_rgb_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def compressed_rgb_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def process_image_callback(self):
        if self.latest_image is None:
            return

        img_display = self.latest_image.copy()
        img_process = self.latest_image.copy()

        results = self.model(img_process, stream=True)

        is_disabled = False
        detected_type = None
        final_ocr_text = ""

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < CONF:
                    continue

                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id] if hasattr(self.model, 'names') else str(cls_id)

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                if class_name == "disabled":
                    is_disabled = True
                    self.get_logger().info("장애인 스티커 검출됨")
                elif class_name in ["normal", "electric", "plate"]:
                    roi = img_process[y1:y2, x1:x2]
                    if roi.size == 0:
                        continue

                    # ✅ 좌측 8% crop
                    crop_x = int(roi.shape[1] * 0.08)
                    roi = roi[:, crop_x:]

                    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    denoised_roi = cv2.medianBlur(gray_roi, 3)
                    _, binary_roi = cv2.threshold(denoised_roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

                    try:
                        binary_msg = self.bridge.cv2_to_imgmsg(binary_roi, encoding='mono8')
                        binary_msg.header.stamp = self.get_clock().now().to_msg()
                        self.binary_img_pub.publish(binary_msg)
                    except Exception as e:
                        self.get_logger().error(f"Failed to publish binary image: {e}")

                    result = self.reader.readtext(binary_roi, detail=0)
                    ocr_text = ''.join(result).strip().replace(' ', '')

                    for c in [':', '|', '(', ')', '[', ']', '{', '}', ';', '\'', '"', '~', '!', '-']:
                        ocr_text = ocr_text.replace(c, '')

                    # ✅ 정규식 필터링
                    if not re.fullmatch(r"\d{2,3}[가-힣]\d{4}", ocr_text):
                        self.get_logger().info(f"OCR 결과 무시됨 (정규식 불일치): {ocr_text}")
                        continue

                    final_ocr_text = ocr_text
                    self.get_logger().info(f"OCR 결과 ({class_name}): {final_ocr_text}")

                    cv2.rectangle(img_display, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    img_pil = PILImage.fromarray(cv2.cvtColor(img_display, cv2.COLOR_BGR2RGB))
                    draw = ImageDraw.Draw(img_pil)
                    try:
                        font = ImageFont.truetype(self.font_path, 30)
                        draw.text((x1, y1 - 40), f"{class_name}: {final_ocr_text}", font=font, fill=(255, 0, 0, 0))
                    except Exception as e:
                        self.get_logger().error(f"PIL 텍스트 렌더링 실패: {e}")

                    img_display = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

                    if detected_type is None and class_name in ["normal", "electric"]:
                        detected_type = class_name

        if is_disabled:
            result_type = "disabled"
        elif detected_type:
            result_type = detected_type
        else:
            result_type = "unknown"

        if final_ocr_text:
            result_json = {
                "type": result_type,
                "car_plate": final_ocr_text
            }
            msg_out = String()
            msg_out.data = json.dumps(result_json, ensure_ascii=False)
            self.ocr_result_pub.publish(msg_out)
            self.get_logger().info(f"OCR JSON 결과: {msg_out.data}")

        try:
            result_msg = self.bridge.cv2_to_imgmsg(img_display, encoding='bgr8')
            result_msg.header.stamp = self.get_clock().now().to_msg()
            self.result_img_pub.publish(result_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish result image: {e}")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['raw', 'compressed'], default='raw', help='이미지 입력 모드: raw 또는 compressed')
    cli_args = parser.parse_args()

    rclpy.init(args=args)
    node = CarPlateOCRNode(cli_args.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
