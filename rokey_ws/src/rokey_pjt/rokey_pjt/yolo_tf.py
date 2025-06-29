import rclpy
from rclpy.node import Node
import numpy as np
import json

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String # String 메시지 타입을 사용합니다.
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

class YOLOTFNode(Node):
    def __init__(self):
        super().__init__('yolo_tf_node')

        self.K = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, '/robot3/oakd/stereo/camera_info', self.camera_info_callback, 10)
        self.create_subscription(String, '/detect/object_info', self.object_info_callback, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        # 🚀 추가: 검출된 객체의 클래스 이름과 맵 좌표를 발행할 퍼블리셔
        self.object_map_coordinates_pub = self.create_publisher(String, '/detect/object_map_coordinates', 10)

        self.marker_id = 0

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("CameraInfo 수신 완료")

    def object_info_callback(self, msg):
        if self.K is None:
            self.get_logger().warn("CameraInfo 미수신. 변환 불가.")
            return

        try:
            objects = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"객체 정보 파싱 실패: {e}")
            return

        marker_array = MarkerArray()
        # 🚀 추가: 퍼블리싱할 객체 좌표 정보를 담을 리스트
        objects_for_publish = [] 

        for obj in objects:
            try:
                u = obj['center_x']
                v = obj['center_y']
                depth = obj['distance']
                class_name = obj['class_name']
                frame_id = obj['frame_id']

                self.get_logger().info(f"수신된 객체 정보 frame_id: '{frame_id}'")

                x, y, z = self.pixel_to_3d(u, v, depth)

                self.get_logger().info(
                    f"OAK-D 광학 프레임 원시 좌표 ({class_name}): x={x:.3f}, y={y:.3f}, z={z:.3f}"
                )

                point_camera = PointStamped()
                point_camera.header.frame_id = frame_id
                
                # 🚀 이전 대화에서 확인된 올바른 매핑 (x, y, z 그대로 사용)
                point_camera.point.x = x
                point_camera.point.y = y
                point_camera.point.z = z

                out_frame_id = 'map' # 기본적으로 map 프레임으로 변환 시도
                point_transformed = PointStamped() # 변환된 포인트를 저장할 객체

                try:
                    tf = self.tf_buffer.lookup_transform(
                        'map',
                        frame_id,
                        rclpy.time.Time(), 
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    point_transformed = tf2_geometry_msgs.do_transform_point(point_camera, tf)
                    out_frame_id = 'map'
                except TransformException as e:
                    self.get_logger().warn(f"map 프레임 변환 실패 → base_link fallback: {e}")
                    # map 변환 실패 시 base_link로 폴백
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            'base_link',
                            frame_id,
                            rclpy.time.Time(), 
                            timeout=rclpy.duration.Duration(seconds=1.0)
                        )
                        point_transformed = tf2_geometry_msgs.do_transform_point(point_camera, tf)
                        out_frame_id = 'base_link'
                    except TransformException as e_fallback:
                        self.get_logger().error(f"base_link 프레임 변환도 실패: {e_fallback}. 마커 생성 불가.")
                        continue # 이 객체는 마커 생성 및 정보 발행을 건너뛰고 다음 객체로 넘어감

                # 🚀 추가: 발행할 객체 정보에 맵 좌표 추가
                objects_for_publish.append({
                    'class_name': class_name,
                    'x': float(point_transformed.point.x),
                    'y': float(point_transformed.point.y),
                    'z': float(point_transformed.point.z),
                    'frame_id': out_frame_id # 실제로 변환된 프레임 ID 기록
                })

                marker = Marker()
                marker.header.frame_id = out_frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "objects"
                marker.id = self.marker_id
                self.marker_id += 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position = point_transformed.point
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2

                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                marker.lifetime.sec = 1
                marker_array.markers.append(marker)

                # self.get_logger().info(
                #     f"마커: {class_name} → ({marker.pose.position.x:.2f}, {marker.pose.position.y:.2f}, {marker.pose.position.z:.2f}) in {out_frame_id}"
                # )

            except Exception as e:
                self.get_logger().error(f"객체 처리 실패: {e}")

        if marker_array.markers:
            self.marker_pub.publish(marker_array)
        
        # 🚀 추가: 검출된 객체 클래스 + 맵 좌표 발행
        if objects_for_publish:
            try:
                json_str = json.dumps(objects_for_publish)
                msg_to_publish = String()
                msg_to_publish.data = json_str
                self.object_map_coordinates_pub.publish(msg_to_publish)
                # self.get_logger().info(f"객체 맵 좌표 발행 완료: {json_str}")
            except Exception as e:
                self.get_logger().error(f"객체 맵 좌표 발행 실패: {e}")

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
    node = YOLOTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()