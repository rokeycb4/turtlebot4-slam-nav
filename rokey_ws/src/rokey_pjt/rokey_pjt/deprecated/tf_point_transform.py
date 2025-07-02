import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge

# ================================
# 설정 상수
# ================================
DEPTH_TOPIC = '/robot3/oakd/stereo/image_raw'  # Depth 이미지 토픽
CAMERA_INFO_TOPIC = '/robot3/oakd/stereo/camera_info'  # CameraInfo 토픽
MAX_DEPTH_METERS = 5.0                 # 시각화 시 최대 깊이 값 (m)
NORMALIZE_DEPTH_RANGE = 3.0            # 시각화 정규화 범위 (m)
WINDOW_NAME = 'Depth Image (Click to get distance)'
# ================================

class DepthChecker(Node):
    def __init__(self):
        super().__init__('depth_checker')
        self.bridge = CvBridge()
        self.K = None
        self.should_exit = False
        self.depth_mm = None  # 최신 depth 이미지 저장
        self.depth_colored = None  # 시각화 이미지 저장

        self.subscription = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_callback,
            10)

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            10)

        # OpenCV 마우스 콜백 설정
        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"CameraInfo received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

    def depth_callback(self, msg):
        if self.should_exit:
            return

        if self.K is None:
            self.get_logger().warn('Waiting for CameraInfo...')
            return

        self.depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        height, width = self.depth_mm.shape

        # 시각화 이미지 생성
        depth_vis = np.nan_to_num(self.depth_mm, nan=0.0)
        depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)
        depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)
        self.depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        # 중심점 표시
        cx = int(self.K[0, 2])
        cy = int(self.K[1, 2])
        cv2.circle(self.depth_colored, (cx, cy), 5, (0, 0, 0), -1)
        cv2.line(self.depth_colored, (0, cy), (width, cy), (0, 0, 0), 1)
        cv2.line(self.depth_colored, (cx, 0), (cx, height), (0, 0, 0), 1)

        cv2.imshow(WINDOW_NAME, self.depth_colored)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.should_exit = True

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.depth_mm is not None:
            distance_mm = self.depth_mm[y, x]
            distance_m = distance_mm / 1000.0  # mm → m
            self.get_logger().info(f"Clicked at (u={x}, v={y}) → Distance = {distance_m:.2f} meters")

def main():
    rclpy.init()
    node = DepthChecker()

    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
