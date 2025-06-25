import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import os

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')

        self.bridge = CvBridge()
        self.frame = None  # 최신 이미지를 저장할 변수

        # 저장 설정
        self.save_dir = "img_capture_1"
        self.capture_interval = 1.0  # 초
        os.makedirs(self.save_dir, exist_ok=True)

        # 파일 접두어 입력
        prefix_input = input("저장할 파일의 접두어를 입력하세요: ").strip()
        self.file_prefix = f"{prefix_input}_"
        self.image_count = 0

        # QoS 설정 (센서 스트림용)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 이미지 토픽 구독 (절대 경로 사용)
        self.subscription = self.create_subscription(
            Image,
            '/robot3/oakd/rgb/image_raw',
            self.image_callback,
            qos_profile
        )

        # 주기적으로 이미지 저장
        self.timer = self.create_timer(self.capture_interval, self.capture_callback)

        self.get_logger().info("이미지 캡처 노드 시작됨. 'q'를 누르면 종료됩니다.")

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge 변환 실패: {e}")

    def capture_callback(self):
        if self.frame is None:
            self.get_logger().warn("아직 수신한 이미지가 없습니다.")
            return

        cv2.imshow("ROS Image", self.frame)
        key = cv2.waitKey(1)

        if key == ord('q'):
            self.get_logger().info("종료 요청됨.")
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
            return

        filename = os.path.join(self.save_dir, f"{self.file_prefix}img_{self.image_count}.jpg")
        cv2.imwrite(filename, self.frame)
        self.get_logger().info(f"이미지 저장 완료: {filename}")
        self.image_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C 감지됨. 종료 중...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
