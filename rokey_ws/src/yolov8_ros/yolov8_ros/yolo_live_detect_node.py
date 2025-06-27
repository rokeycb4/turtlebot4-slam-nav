import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

MODEL_PATH = "/home/fred/rokey_ws/detect_mu3.pt"
IMAGE_TOPIC = "/robot3/oakd/rgb/preview/image_raw"
OUTPUT_TOPIC = "/yolo/detected_image"

class YoloLiveNode(Node):
    def __init__(self):
        super().__init__('yolo_live_node')

        self.bridge = CvBridge()
        self.yolo_model = YOLO(MODEL_PATH)

        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.listener_callback,
            10
        )
        self.pub = self.create_publisher(Image, OUTPUT_TOPIC, 10)

    def listener_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.yolo_model.predict(source=frame, conf=0.25, verbose=False)
        result_frame = results[0].plot()

        out_msg = self.bridge.cv2_to_imgmsg(result_frame, encoding='bgr8')
        out_msg.header = msg.header  # 원본 헤더 유지(타임스탬프 등)
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    node = YoloLiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import os
# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from ultralytics import YOLO
# import numpy as np

# # ==============================
# # 사용자 설정
# # ==============================
# MODEL_PATH = "/home/fred/rokey_ws/detect_mu2.pt"  # YOLO 모델 경로
# IMAGE_TOPIC = "/robot3/oakd/rgb/preview/image_raw"

# class YoloLiveNode(Node):
#     def __init__(self):
#         super().__init__('yolo_live_node')

#         self.bridge = CvBridge()
#         self.yolo_model = YOLO(MODEL_PATH)

#         self.subscription = self.create_subscription(
#             Image,
#             IMAGE_TOPIC,
#             self.listener_callback,
#             10
#         )

#     def listener_callback(self, msg: Image):
#         # ROS 이미지 → OpenCV 이미지
#         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

#         # YOLO 예측 수행
#         results = self.yolo_model.predict(source=frame, conf=0.25, verbose=False)

#         # 결과 시각화 이미지 추출
#         result_frame = results[0].plot()  # 탐지 박스 포함된 이미지 반환

#         # 결과 이미지 보여주기
#         cv2.imshow("YOLO Live Detection", result_frame)
#         key = cv2.waitKey(1) & 0xFF
#         if key == ord('q'):
#             rclpy.shutdown()
#             cv2.destroyAllWindows()

# def main():
#     rclpy.init()
#     node = YoloLiveNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()
