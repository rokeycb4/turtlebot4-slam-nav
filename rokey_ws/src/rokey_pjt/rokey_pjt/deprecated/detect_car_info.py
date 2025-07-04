# detect_car_info.py

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
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

# MODEL_PATH = '/home/kiwi/github_package/turtlebot4-slam-nav/rokey_ws/car_plate2.pt'
MODEL_PATH = '/home/rokey/rokey_ws/car_plate2.pt'

# RGB_TOPIC = '/robot3/oakd/rgb/image_raw'
RGB_TOPIC = '/robot2/oakd/rgb/image_raw'

CONF = 0.5
PROCESS_INTERVAL_SEC = 0.2

class CarPlateOCRNode(Node):
    def __init__(self):
        super().__init__('carplate_ocr_node')

        self.bridge = CvBridge()
        self.latest_image = None

        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model not found: {MODEL_PATH}")
            sys.exit(1)

        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f"YOLO 모델 로드 완료: {MODEL_PATH}")

        self.reader = easyocr.Reader(['ko'])
        self.get_logger().info(f"EasyOCR 리더 로드 완료 (kor)")

        self.rgb_sub = self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 10)

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

    def rgb_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

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
                    # disabled만 OCR 제외
                    roi = img_process[y1:y2, x1:x2]
                    if roi.size == 0:
                        continue

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

                    for c in [':', '|', '(', ')', '[', ']', '{', '}', ';', '\'', '"', '~', '!']:
                        ocr_text = ocr_text.replace(c, '')

                    if ocr_text:
                        final_ocr_text = ocr_text
                        self.get_logger().info(f"OCR 결과 ({class_name}): {final_ocr_text}")

                    cv2.rectangle(img_display, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    img_pil = PILImage.fromarray(cv2.cvtColor(img_display, cv2.COLOR_BGR2RGB))
                    draw = ImageDraw.Draw(img_pil)
                    try:
                        font = ImageFont.truetype(self.font_path, 30)
                        draw.text((x1, y1 - 40), final_ocr_text, font=font, fill=(255, 0, 0, 0))
                    except Exception as e:
                        self.get_logger().error(f"PIL 텍스트 렌더링 실패: {e}")

                    img_display = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

                    if detected_type is None and class_name in ["normal", "electric"]:
                        detected_type = class_name

        # 최종 type 결정
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
    rclpy.init(args=args)
    node = CarPlateOCRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
