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
        self.K = None
        self.rgb_image = None
        self.depth_image = None

        self.rgb_frame_id = None

        self.class_names = getattr(self.model, 'names', [])

        self.create_subscription(CompressedImage, '/robot3/oakd/rgb/image_raw/compressed', self.rgb_compressed_callback, 10)
        self.create_subscription(Image, '/robot3/oakd/stereo/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/robot3/oakd/stereo/camera_info', self.camera_info_callback, 10)

        self.pub_image = self.create_publisher(Image, '/detect/yolo_distance_image', 10)
        self.pub_objects = self.create_publisher(String, '/detect/object_info', 10)
        self.timer = self.create_timer(0.1, self.process_image)

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

            val = depth[v, u]
            distance_m = val / 1000.0 if depth.dtype == np.uint16 else float(val)

            # 픽셀 좌표와 깊이를 3D 카메라 좌표로 변환
            x, y, z = self.pixel_to_3d(u, v, distance_m)

            # 🚀 추가된 로그: OAK-D 카메라 광학 프레임 기준의 x, y, z 값 출력
            self.get_logger().info(
                f"OAK-D 광학 프레임 원시 좌표 ({class_name}): x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )
            # -------------------------------------------------------------

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

    def pixel_to_3d(self, u, v, depth):
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z

def main(args=None):
    rclpy.init(args=args)
    node = DetectAllObjectsWithDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()