#!/usr/bin/env python3
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
import time
from std_msgs.msg import String

def create_pose(x, y, yaw_deg, navigator):
    """x, y, yaw(도 단위) → PoseStamped 생성"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y

    yaw_rad = yaw_deg * 3.141592 / 180.0
    q = quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


class LocationSubscriber(Node):
    def __init__(self):
        super().__init__('parking_location_listener')
        self.subscription = self.create_subscription(
            String,
            '/parking/location',
            self.location_callback,
            10
        )
        self.get_logger().info('Subscribed to /parking/location')

    def location_callback(self, msg):
        location = msg.data
        self.get_logger().info(f"[LOCATION RECEIVED] Parking location: {location}")

def main():
    rclpy.init()

    # 도킹 및 경로 이동용 Navigator
    dock_navigator = TurtleBot4Navigator()
    nav_navigator = BasicNavigator()

    # 1. 초기 위치 설정
    initial_pose = create_pose(-0.02, -0.02, 0.0, nav_navigator)
    nav_navigator.setInitialPose(initial_pose)
    nav_navigator.get_logger().info(f'초기 위치 설정 중...')
    time.sleep(1.0) #AMCL이 초기 pose 처리 시 필요한 시간과 TF를 얻을 수 있게 됨
    
    nav_navigator.waitUntilNav2Active()

    # 2. 도킹 상태면 언도킹
    if dock_navigator.getDockedStatus():
        dock_navigator.get_logger().info('도킹 상태 → 언도킹')
        dock_navigator.undock()

    # 3. 개별 Pose 생성 (경유지 명시)
    waypoints = [
        create_pose(-0.02, -1.20, 180.0, nav_navigator), #수레위치
        create_pose(-2.28, -5.01, 90.0, nav_navigator), #A-1
        create_pose(-1.32, -5.15, 90.0, nav_navigator), #A-2
        create_pose(1.03, -2.06, 0.0, nav_navigator),# B-1
        create_pose(0.94, -1.10, 0.0, nav_navigator),# B-2
        create_pose(-2.95, -3.54, 180.0, nav_navigator), #C-1
        create_pose(-3.04, -4.59, 180.0, nav_navigator), #C-2
    ]

    # 4. Waypoint 경로 이동 시작
    nav_start = nav_navigator.get_clock().now()
    nav_navigator.followWaypoints(waypoints)

    # 5. 이동 중 피드백 확인
    while not nav_navigator.isTaskComplete():
        feedback = nav_navigator.getFeedback()
        if feedback:
            elapsed = nav_navigator.get_clock().now() - nav_start
            nav_navigator.get_logger().info(
                f'현재 waypoint: {feedback.current_waypoint + 1}/{len(waypoints)}, '
                f'경과 시간: {elapsed.nanoseconds / 1e9:.1f}초'
            )

    # 6. 도달한 waypoint 인덱스 확인
    result_index = nav_navigator.getResult()
    nav_navigator.get_logger().info(f'Waypoint {result_index} 까지 도달 완료')

    # 7. 자동 도킹 요청
    #dock_navigator.dock()
    #dock_navigator.get_logger().info('도킹 요청 완료')

    # 8. 종료 처리
    dock_navigator.destroy_node()
    nav_navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
'''
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler
import time

def create_pose(x, y, yaw_deg, navigator):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y

    yaw_rad = yaw_deg * 3.141592 / 180.0
    q = quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


class ParkingLocationCommander(Node):
    def __init__(self):
        super().__init__('parking_location_commander')
        self.navigator = BasicNavigator()

        # Location map
        self.location_map = {
            "A-1": (-2.28, -5.01, 90.0),
            "A-2": (-1.32, -5.15, 90.0),
            "B-1": (1.03, -2.06, 0.0),
            "B-2": (0.94, -1.10, 0.0),
            "C-1": (-2.95, -3.54, 180.0),
            "C-2": (-3.04, -4.59, 180.0),
        }

        # Initialize pose (AMCL + 실제 이동)
        self.go_to_initial_pose()

        # After arriving, start listening
        self.subscription = self.create_subscription(
            String,
            '/parking/location',
            self.location_callback,
            10
        )
        self.get_logger().info('✅ Subscribed to /parking/location - Ready for commands!')

    def go_to_initial_pose(self):
        # 1. Set initial pose in AMCL
        initial = create_pose(-0.02, -0.02, 0.0, self.navigator)
        self.navigator.setInitialPose(initial)
        self.get_logger().info('✅ 초기 위치 AMCL 설정 완료')
        time.sleep(1.0)
        self.navigator.waitUntilNav2Active()

        # 2. Actually navigate to initial location
        self.get_logger().info('🚀 초기 위치로 이동 시작')
        self.navigator.goToPose(initial)

        start_time = self.navigator.get_clock().now()

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                elapsed = self.navigator.get_clock().now() - start_time
                self.get_logger().info(
                    f"➡️ 초기 이동 중 - 남은 거리: {feedback.distance_remaining:.2f} m, "
                    f"경과 시간: {elapsed.nanoseconds / 1e9:.1f} s"
                )
            time.sleep(0.2)

        result = self.navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            self.get_logger().info('✅ 초기 위치 도착 완료. 명령 대기 시작.')
        else:
            self.get_logger().warn(f"⚠️ 초기 위치 이동 실패 (결과 코드: {result})")


    def location_callback(self, msg):
        location = msg.data.strip()
        self.get_logger().info(f"📌 Received location command: {location}")

        if location not in self.location_map:
            self.get_logger().warn(f"❗ Unknown location: {location}")
            return

        x, y, yaw = self.location_map[location]
        target_pose = create_pose(x, y, yaw, self.navigator)

        self.get_logger().info(f"🚗 이동 시작: {location} → 좌표 ({x}, {y}, {yaw})")
        self.navigator.goToPose(target_pose)

        start_time = self.navigator.get_clock().now()

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                elapsed = self.navigator.get_clock().now() - start_time
                self.get_logger().info(
                    f"➡️ 진행 중 - 남은 거리: {feedback.distance_remaining:.2f} m, "
                    f"경과 시간: {elapsed.nanoseconds / 1e9:.1f} s"
                )
            time.sleep(0.2)

        result = self.navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            self.get_logger().info(f"✅ 도착 완료: {location}")
        else:
            self.get_logger().warn(f"⚠️ 이동 실패 (결과 코드: {result})")


def main():
    rclpy.init()
    commander = ParkingLocationCommander()

    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        commander.get_logger().info('🛑 종료 요청 감지')
    finally:
        commander.navigator.lifecycleShutdown()
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

'''

#!/usr/bin/env python3
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
import time

def create_pose(x, y, yaw_deg, navigator):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y

    yaw_rad = yaw_deg * 3.141592 / 180.0
    q = quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


class ParkingLocationCommander(Node):
    def __init__(self):
        super().__init__('parking_location_commander')

        # Nav2 Navigator (이동용)
        self.navigator = BasicNavigator()
        # Docking 전용
        self.dock_navigator = TurtleBot4Navigator()

        # 좌표 맵
        self.location_map = {
            """
            "A-1": (-2.28, -5.01, 90.0),
            "A-2": (-1.32, -5.15, 90.0),
            "B-1": (1.03, -2.06, 0.0),
            "B-2": (0.94, -1.10, 0.0),
            "C-1": (-2.95, -3.54, 180.0),
            "C-2": (-3.04, -4.59, 180.0),
            """
            "A-1": (-3.21, -5.34, 90.0),
            "A-2": (-2.17, -5.40, 90.0),
            "B-1": (0.55, -2.09, 180.0),
            "B-2": (0.57, -1.12, 180.0),
            "C-1": (-4.25, -3.44, 0.0),
            "C-2": (-4.22, -4.48, 0.0),
        }

        # 위치 상수
        self.initial_xyyaw = (-0.02, -0.02, 90.0)
        #self.wait_xyyaw = (-1.03, -0.02, 180.0)
        self.final_xyyaw=(-2.38,-4.38,180.0)
        # Nav2 활성화 대기
        #self.navigator.waitUntilNav2Active()

        # 초기 위치로 이동 (AMCL 초기화 + 실제 이동)
        self.go_to_initial_pose()

        # 도킹 상태면 언도킹
        if self.dock_navigator.getDockedStatus():
            self.get_logger().info('🚦 현재 도킹 상태 → 언도킹 수행')
            self.dock_navigator.undock()
            time.sleep(2.0)

        # 대기 지점으로 이동
        #self.go_to_wait_pose()

        # 명령 구독 시작
        self.subscription = self.create_subscription(
            String,
            '/parking/location',
            self.location_callback,
            10
        )
        self.get_logger().info('✅ Subscribed to /parking/location - Ready for commands!')

    def go_to_pose_blocking(self, pose, description):
        self.get_logger().info(f"🚀 이동 시작: {description}")
        self.navigator.goToPose(pose)
        start_time = self.navigator.get_clock().now()

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                elapsed = self.navigator.get_clock().now() - start_time
                self.get_logger().info(
                    f"➡️ 진행 중 - 남은 거리: {feedback.distance_remaining:.2f} m, "
                    f"경과 시간: {elapsed.nanoseconds / 1e9:.1f} s"
                )
            time.sleep(0.2)
        '''
        result = self.navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            self.get_logger().info(f"✅ 도착 완료: {description}")
        else:
            self.get_logger().warn(f"⚠️ 이동 실패 (결과 코드: {result})")
        '''

    def go_to_initial_pose(self):
        initial = create_pose(*self.initial_xyyaw, self.navigator)
        self.navigator.setInitialPose(initial)
        self.get_logger().info('✅ AMCL 초기 Pose 세팅 완료')
        time.sleep(1.0)
        self.navigator.waitUntilNav2Active()

        #self.go_to_pose_blocking(initial, "초기 위치 (-0.02, -0.02)")

    def go_to_wait_pose(self):
        wait_pose = create_pose(*self.wait_xyyaw, self.navigator)
        self.go_to_pose_blocking(wait_pose, "대기 지점 (-0.02, -1.20)")

    def location_callback(self, msg):
        location = msg.data.strip()
        self.get_logger().info(f"📌 Received parking command: {location}")

        if location not in self.location_map:
            self.get_logger().warn(f"❗ Unknown location: {location}")
            return

        # 1️⃣ 지정 주차 위치로 이동
        x, y, yaw = self.location_map[location]
        target_pose = create_pose(x, y, yaw, self.navigator)
        self.go_to_pose_blocking(target_pose, f"주차 위치: {location}")
        time.sleep(5.0)

        # 2️⃣ 최종 위치
        final_pose = create_pose(*self.final_xyyaw, self.navigator)
        self.go_to_pose_blocking(final_pose, "최종위치")
        time.sleep(3.0)

        # 2️⃣ 초기위치
        initial_pose = create_pose(*self.initial_xyyaw, self.navigator)
        self.go_to_pose_blocking(initial_pose, "원래위치")
        # 3️⃣ 초기 위치에서 도킹 요청
        self.get_logger().info('🚀 초기 위치 도착 → 도킹 요청 시작')
        self.dock_navigator.dock()
        self.get_logger().info('✅ 도킹 요청 완료')

        self.get_logger().info('✅ Ready for next /parking/location command!')

def main():
    rclpy.init()
    commander = ParkingLocationCommander()

    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        commander.get_logger().info('🛑 종료 요청 감지')
    finally:
        commander.navigator.lifecycleShutdown()
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
