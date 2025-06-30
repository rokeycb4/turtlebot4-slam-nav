import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration # Duration 임포트

import numpy as np
import json
import math # atan2 사용을 위해 math 모듈 임포트

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped # PoseStamped 추가
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray # 마커 발행용 추가

import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

from nav2_msgs.action import NavigateToPose # NavigateToPose 액션 메시지 임포트


class ObjectToNavGoalNode(Node):
    def __init__(self):
        super().__init__('object_to_nav_goal_node')

        self.get_logger().info("ObjectToNavGoalNode 초기화 시작.")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.K = None  # CameraInfo로 채움

        # 1. 객체 검출 정보를 구독
        self.create_subscription(
            CameraInfo, '/robot3/oakd/stereo/camera_info', self.camera_info_callback, 10)
        self.create_subscription(
            String, '/detect/object_info', self.object_info_callback, 10)

        # 4. 테스트 용으로 계산된 타겟 마커 발행
        self.target_marker_pub = self.create_publisher(MarkerArray, '/navigation_target_markers', 10)

        # 5. 내비게이션 액션 클라이언트 생성
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("내비게이션 액션 클라이언트 대기 중...")
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info("내비게이션 액션 서버 연결됨.")

        self.is_navigating = False # 현재 내비게이션 중인지 나타내는 플래그
        self.goal_handle = None # 현재 전송된 골의 핸들

        self.marker_id = 0

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("CameraInfo 수신 완료")

    def object_info_callback(self, msg):
        if self.K is None:
            self.get_logger().warn("CameraInfo 미수신. 변환 불가.")
            return

        if self.is_navigating:
            self.get_logger().info("이미 내비게이션 진행 중 → 새 객체 무시")
            return

        try:
            objects = json.loads(msg.data)
            if not objects:
                return

            obj = objects[0]  # 첫 번째 객체만 사용
            u, v, depth = obj['center_x'], obj['center_y'], obj['distance']
            frame_id = obj['frame_id']

            x_cam, y_cam, z_cam = self.pixel_to_3d(u, v, depth)

            # 1. 객체 검출 정보를 구독 (이미 완료)
            # 2. 검출된 좌표를 base_link로 변경
            point_camera = PointStamped()
            point_camera.header.frame_id = frame_id
            point_camera.header.stamp = self.get_clock().now().to_msg() # 최신 타임스탬프

            point_camera.point.x = x_cam
            point_camera.point.y = y_cam
            point_camera.point.z = z_cam

            self.get_logger().info(
                f"OAK-D 광학 프레임 원시 좌표: ({point_camera.point.x:.3f}, {point_camera.point.y:.3f}, {point_camera.point.z:.3f})"
            )

            point_base_link = None
            try:
                # 카메라 좌표를 base_link로 변환
                # TF 타임스탬프는 rclpy.time.Time() (최신 변환) 또는 point_camera.header.stamp를 사용
                # 여기서는 최신 변환을 사용하되, 가급적 센서 데이터의 타임스탬프를 사용하는 것이 좋음
                transform_cam_to_base = self.tf_buffer.lookup_transform(
                    'base_link', frame_id,
                    rclpy.time.Time(), # 또는 point_camera.header.stamp
                    timeout=Duration(seconds=1.0)
                )
                point_base_link = tf2_geometry_msgs.do_transform_point(point_camera, transform_cam_to_base)
                self.get_logger().info(f"객체 base_link (변환): ({point_base_link.point.x:.2f}, {point_base_link.point.y:.2f})")

            except TransformException as e:
                self.get_logger().error(f"카메라->base_link TF 변환 실패: {e}")
                return

            if point_base_link is None:
                return

            # 3. base_link 기준 50cm 앞에 좌표를 타겟 좌표로 구함
            # 로봇의 base_link 기준 50cm 앞에 객체가 오도록 목표 설정
            # 즉, 로봇은 point_base_link.x - 0.5 만큼 앞으로 가서 멈추려 함
            # 그리고 point_base_link.y 만큼 옆으로 틀어져있으면 그만큼 움직여서 정렬함
            
            # 여기서 중요한 점: 'base_link' 기준 목표점은 항상 (0.5, 0.0)이 되도록 만드는 것이 목표입니다.
            # 하지만 객체의 현재 위치가 (point_base_link.x, point_base_link.y)이므로,
            # 로봇이 이 위치로 이동하면 객체가 base_link 기준 (0.5, 0.0)에 위치하게 됩니다.
            
            # 즉, 로봇이 현재 base_link 기준으로 (point_base_link.x - 0.5, point_base_link.y)로 이동해야 함
            target_point_in_base_link = PointStamped()
            target_point_in_base_link.header.frame_id = 'base_link'
            target_point_in_base_link.header.stamp = self.get_clock().now().to_msg()
            target_point_in_base_link.point.x = point_base_link.point.x - 0.5 # 50cm 떨어진 곳
            target_point_in_base_link.point.y = point_base_link.point.y        # Y축은 객체와 동일하게
            target_point_in_base_link.point.z = 0.0 # 맵 위에서 움직이므로 Z는 0으로 가정

            self.get_logger().info(f"계산된 base_link 기준 상대 목표: ({target_point_in_base_link.point.x:.2f}, {target_point_in_base_link.point.y:.2f})")

            # 4. 타겟 좌표를 맵좌표로 변경
            target_pose_in_map = PoseStamped()
            target_pose_in_map.header.frame_id = 'map'
            target_pose_in_map.header.stamp = self.get_clock().now().to_msg() # 최신 타임스탬프

            try:
                # base_link 기준 목표점을 map 프레임으로 변환
                # TF 타임스탬프는 rclpy.time.Time() (최신 변환) 또는 target_point_in_base_link.header.stamp를 사용
                transform_base_to_map = self.tf_buffer.lookup_transform(
                    'map', 'base_link',
                    rclpy.time.Time(), # 또는 target_point_in_base_link.header.stamp
                    timeout=Duration(seconds=1.0)
                )
                
                # PointStamped를 map으로 변환
                transformed_point = tf2_geometry_msgs.do_transform_point(target_point_in_base_link, transform_base_to_map)
                
                target_pose_in_map.pose.position = transformed_point.point
                
                # 목표 방향 설정 (쿼터니언)
                # 객체를 바라보는 방향으로 설정
                # target_point_in_base_link는 base_link 기준의 목표점 (x,y)
                # 이 점을 향하는 각도 (yaw)를 계산하여 쿼터니언으로 변환
                # Note: Nav2는 목표 '포즈'로 이동하므로, 목표 위치와 함께 목표 방향도 중요합니다.
                # 로봇이 객체 정면을 바라보도록 하는 것이 목표라면,
                # 이 yaw는 map 프레임에서 해당 목표 위치에 로봇이 도달했을 때
                # 객체를 바라보는 방향이 되어야 합니다.
                # 현재는 base_link 기준 목표점을 향하는 방향을 사용하고 있습니다.
                # 이 방식은 로봇이 객체에 정렬된 상태로 접근하는 데 도움이 됩니다.
                yaw = math.atan2(target_point_in_base_link.point.y, target_point_in_base_link.point.x)
                
                # Yaw 값을 map 프레임에서의 절대 방향으로 변환해야 함
                # 현재 로봇의 map 프레임에서의 yaw + base_link 기준 목표 yaw
                # transform_base_to_map.transform.rotation 에서 현재 로봇의 yaw 추출
                current_robot_yaw = self.quaternion_to_euler(
                    transform_base_to_map.transform.rotation.x,
                    transform_base_to_map.transform.rotation.y,
                    transform_base_to_map.transform.rotation.z,
                    transform_base_to_map.transform.rotation.w
                )[2] # roll, pitch, yaw 중 yaw 가져옴

                final_target_yaw_in_map = current_robot_yaw + yaw # 로봇의 현재 방향 + 객체를 향한 상대 방향
                
                q = self.euler_to_quaternion(0, 0, final_target_yaw_in_map) # roll, pitch는 0, yaw만 적용
                target_pose_in_map.pose.orientation.x = q[0]
                target_pose_in_map.pose.orientation.y = q[1]
                target_pose_in_map.pose.orientation.z = q[2]
                target_pose_in_map.pose.orientation.w = q[3]

                self.get_logger().info(
                    f"최종 맵 좌표 목표: ({target_pose_in_map.pose.position.x:.2f}, {target_pose_in_map.pose.position.y:.2f}) "
                    f"목표 방향 (Yaw): {math.degrees(final_target_yaw_in_map):.2f}도 in '{target_pose_in_map.header.frame_id}'"
                )

            except TransformException as e:
                self.get_logger().error(f"base_link->map TF 변환 실패: {e}. 내비게이션 목표 전송 불가.")
                return

            # 4.1 테스트 용으로 계산된 타겟 마커 발행
            self.publish_target_marker(target_pose_in_map)

            # 5. 내비게이션으로 타겟 좌표까지 이동
            self.send_nav_goal(target_pose_in_map)

        except Exception as e:
            self.get_logger().error(f"객체 정보 처리 또는 JSON 파싱 실패: {e}")

    def pixel_to_3d(self, u, v, depth):
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        # 카메라 내부 파라미터를 사용하여 2D 픽셀을 3D 공간으로 역투영
        # X는 수평 방향, Y는 수직 방향, Z는 깊이(카메라 전방)
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth # depth는 보통 z값으로 사용됨
        return x, y, z

    def euler_to_quaternion(self, roll, pitch, yaw):
        # 오일러 각(roll, pitch, yaw)을 쿼터니언으로 변환
        # ROS 2에서 쿼터니언 순서는 (x, y, z, w)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = sr * cp * cy - cr * sp * sy # x
        q[1] = cr * sp * cy + sr * cp * sy # y
        q[2] = cr * cp * sy - sr * sp * cy # z
        q[3] = cr * cp * cy + sr * sp * sy # w
        return q
    
    def quaternion_to_euler(self, x, y, z, w):
        # 쿼터니언을 오일러 각(roll, pitch, yaw)으로 변환
        # ROS 2에서 쿼터니언 순서는 (x, y, z, w)
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def publish_target_marker(self, pose: PoseStamped):
        marker_array = MarkerArray()
        
        marker = Marker()
        marker.header.frame_id = pose.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nav_targets"
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.ARROW # 화살표 타입으로 방향까지 표시
        marker.action = Marker.ADD
        marker.pose = pose.pose # PoseStamped의 pose를 그대로 사용
        marker.scale.x = 0.5 # 화살표 길이
        marker.scale.y = 0.1 # 화살표 폭
        marker.scale.z = 0.1 # 화살표 높이

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0 # 불투명도

        marker.lifetime = Duration(seconds=5.0).to_msg() # 5초 후 사라짐
        marker_array.markers.append(marker)
        
        self.target_marker_pub.publish(marker_array)
        self.get_logger().info(f"내비게이션 목표 마커 발행 완료: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")

    def send_nav_goal(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info("내비게이션 목표를 Nav2로 전송 중...")
        
        # 비동기적으로 목표 전송
        self.future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True # 내비게이션 시작 플래그 설정

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('목표가 거부되었습니다.')
            self.is_navigating = False
            return

        self.get_logger().info('목표가 수락되었습니다.')
        # 목표가 수락되면, 결과를 기다리는 Future를 추가합니다.
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # 목표 결과를 처리하는 콜백 함수
        result = future.result().result
        status = future.result().status
        if status == 4: # GoalStatus.SUCCEEDED (Nav2 메시지 참조)
            self.get_logger().info('내비게이션 목표 달성!')
        else:
            self.get_logger().info(f'내비게이션 목표 실패 (상태: {status})')
        self.is_navigating = False # 내비게이션 완료 (성공/실패 무관)
        self.goal_handle = None # 다음 목표를 위해 핸들 초기화


def main(args=None):
    rclpy.init(args=args)
    node = ObjectToNavGoalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()