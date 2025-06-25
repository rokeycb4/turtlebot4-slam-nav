import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from geometry_msgs.msg import Pose2D
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D


class YoloLive(Node):
    def __init__(self):
        super().__init__('yolo_live_node')
        self.model = YOLO('/home/kiwi/rokey_ws/src/yolov8_ros/yolov8_ros/detect_mu2.pt')
        self.model.model.names = {0: 'enemy', 1: 'friendly'}

        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/robot3/oakd/rgb/image_raw',
            self.image_callback,
            10
        )

        cv2.namedWindow("YOLOv8 Detection", cv2.WINDOW_NORMAL)
        self.get_logger().info('YOLOv8 Live Node Started')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 실패: {e}')
            return

        results = self.model(frame, conf=0.2)[0]

        for det in results.boxes:
            x1, y1, x2, y2 = map(int, det.xyxy[0])
            cls_id = int(det.cls[0])
            score = float(det.conf[0])
            label = self.model.model.names.get(cls_id, f'class_{cls_id}')

            # 바운딩박스 및 라벨 시각화
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            text = f'{label} {score:.2f}'
            (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw, y1), (0, 255, 0), -1)
            cv2.putText(frame, text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        cv2.imshow("YOLOv8 Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloLive()
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
