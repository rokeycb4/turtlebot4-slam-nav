# tf_ref2
import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge

import cv2

import numpy as np

import os

import sys

import math

from ultralytics import YOLO

from geometry_msgs.msg import PointStamped 



# ========================

# 상수 정의

# ========================

MODEL_PATH = '/home/rokey/rokey_ws/src/rokey_pjt/rokey_pjt/yolo_detect_2/best.pt'  # YOLO 모델 경로

RGB_TOPIC = '/robot2/oakd/rgb/preview/image_raw'

DEPTH_TOPIC = '/robot2/oakd/stereo/image_raw'

CAMERA_INFO_TOPIC = '/robot2/oakd/stereo/camera_info'

TARGET_CLASS_IDS = [0, 1, 2, 3]  # YOLO에서 탐지할 클래스

NORMALIZE_DEPTH_RANGE = 3.0  # 정규화 최대 깊이 (m)

CONF=0.7



class MultiSensorViewerNode(Node):

    def __init__(self):

        super().__init__('multi_sensor_viewer')

        self.bridge = CvBridge()

        self.K = None

        self.should_shutdown = False



        # YOLO 모델 로드

        if not os.path.exists(MODEL_PATH):

            self.get_logger().error(f"Model not found: {MODEL_PATH}")

            sys.exit(1)

        self.model = YOLO(MODEL_PATH)

        self.class_names = getattr(self.model, 'names', [])



        # Subscribe

        self.depth_point_pub = self.create_publisher(PointStamped, 'depth_point', 10)

        self.rgb_sub = self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, 10)

        self.depth_sub = self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 10)

        self.camera_info_sub = self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, 10)



        # 최근 depth frame

        self.latest_depth_mm = None



    def camera_info_callback(self, msg):

        if self.K is None:

            self.K = np.array(msg.k).reshape(3, 3)

            self.get_logger().info(f"CameraInfo received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")



    def depth_callback(self, msg):

        self.latest_depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # depth_colored 시각화

        '''depth_vis = np.nan_to_num(self.latest_depth_mm, nan=0.0)

        depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)

        depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)

        depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)



        # 중심 십자선

        if self.K is not None:

            u = int(self.K[0, 2])

            v = int(self.K[1, 2])

            cv2.circle(depth_colored, (u, v), 5, (0, 0, 0), -1)

            cv2.line(depth_colored, (0, v), (depth_colored.shape[1], v), (0, 0, 0), 1)

            cv2.line(depth_colored, (u, 0), (u, depth_colored.shape[0]), (0, 0, 0), 1)



        cv2.imshow("Depth Image", depth_colored)

        '''

    def rgb_callback(self, msg):

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        display_img = img.copy()



        object_count = 0

        results = self.model(img, stream=True)



        for r in results:

            for box in r.boxes:

                cls = int(box.cls[0])

                if cls not in TARGET_CLASS_IDS:

                    continue



                x1, y1, x2, y2 = map(int, box.xyxy[0])

                conf = math.ceil(box.conf[0] * 100) / 100

                label = self.class_names[cls] if cls < len(self.class_names) else f"class_{cls}"



                # 중심 좌표

                u = (x1 + x2) // 2

                v = (y1 + y2) // 2



                # 거리 정보 가져오기

                if self.K is not None and self.latest_depth_mm is not None:

                    h, w = self.latest_depth_mm.shape

                    if 0 <= v < h and 0 <= u < w:

                        distance_m = self.latest_depth_mm[v, u] / 1000.0

                        label += f" ({distance_m:.2f}m)"

                        # fx, fy, cx, cy를 이용해 3D 좌표계 변환

                        fx, fy = self.K[0, 0], self.K[1, 1]

                        cx, cy = self.K[0, 2], self.K[1, 2]

                        x = (u - cx) * distance_m / fx

                        y = (v - cy) * distance_m / fy

                        z = distance_m



                        # PointStamped 메시지 생성 및 publish

                        point_msg = PointStamped()

                        point_msg.header.stamp = self.get_clock().now().to_msg()

                        point_msg.header.frame_id = msg.header.frame_id  # 예: 'oakd_rgb_optical_frame'

                        point_msg.point.x = x

                        point_msg.point.y = y

                        point_msg.point.z = z



                if conf> CONF:

                    # 박스 + 라벨 시각화

                    cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 0, 255), 2)

                    cv2.putText(display_img, f"{label}: {conf}", (x1, y1 - 10),

                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

                    object_count += 1

                    self.depth_point_pub.publish(point_msg)



        # 탐지 수 출력

        cv2.putText(display_img, f"Objects: {object_count}", (10, 30),

                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)



        # 출력

        cv2.imshow("YOLO + Depth Viewer", cv2.resize(display_img, (img.shape[1]*2, img.shape[0]*2)))

        key = cv2.waitKey(1)

        if key == ord('q'):

            self.should_shutdown = True

            self.get_logger().info("Q pressed. Shutting down...")



# ========================

# 메인 함수

# ========================

def main():

    rclpy.init()

    node = MultiSensorViewerNode()



    try:

        while rclpy.ok() and not node.should_shutdown:

            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()

        cv2.destroyAllWindows()

        print("Shutdown complete.")

        sys.exit(0)



if __name__ == '__main__':

    main()