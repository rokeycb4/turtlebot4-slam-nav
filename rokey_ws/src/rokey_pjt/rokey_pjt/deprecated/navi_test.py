#!/usr/bin/env python3

# navi_test.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
import time
# from rclpy.qos import qos_profile_sensor_data # 더 이상 사용되지 않아 주석 처리


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
        self.dock_navigator = TurtleBot4Navigator()

        self.location_map = {
            "A-1": (-2.28, -5.01, -90.0),
            "A-2": (-1.32, -5.15, -90.0),
            "B-1": (1.03, -2.06, 0.0),
            "B-2": (0.94, -1.10, 0.0),
            "C-1": (-2.95, -3.54, 180.0),
            "C-2": (-3.04, -4.59, 180.0),
        }

        self.initial_xyyaw = (-0.02, -0.02, 90.0)
        # self.wait_xyyaw = (-1.03, -0.02, 0.0) # 더 이상 사용되지 않아 주석 처리

        self.go_to_initial_pose()

        # 언독 로직
        if self.dock_navigator.getDockedStatus():
            self.get_logger().info('🚦 현재 도킹 상태 → 언도킹 수행')
            self.dock_navigator.undock()
            time.sleep(2.0)
        else:
            self.get_logger().info('🚦 현재 도킹 상태가 아님 → 언도킹 건너뜀')

        # --- A-2 위치로 이동 및 180도 회전 자동 실행 로직 ---
        # 이 함수는 로봇을 A-2 위치로 보내고 2초 대기 후 180도 회전합니다.
        self.perform_a1_and_rotate() # 함수 이름은 그대로 두지만 실제 동작은 A-2
        # --- 자동 실행 로직 끝 ---

        # /parking/location 토픽 구독: 자동화된 동작 후에도 수동 명령 테스트를 위해 유지합니다.
        self.subscription = self.create_subscription(
            String,
            '/parking/location',
            self.location_callback,
            10
        )
        self.get_logger().info('✅ /parking/location 토픽 구독 시작 (테스트 모드)')
        self.get_logger().info('➡️ 자동 언독, A-2 이동, 180도 회전 후 수동 명령 테스트 가능.')

        # self.parking_coord = None # 더 이상 사용되지 않아 주석 처리

        # /detect/object_map_pose 구독 제거
        # self.create_subscription(
        #     PoseStamped,
        #     '/detect/object_map_pose',
        #     self.object_map_pose_callback,
        #     qos_profile_sensor_data
        # )
        # self.get_logger().info('✅ Subscribed to /detect/object_map_pose') # 관련 로그도 제거

        # /cmd_vel 토픽 퍼블리셔는 180도 회전에 필요하므로 유지합니다.
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot3/cmd_vel', 10) 

    # --- A-2 위치로 이동 및 180도 회전 메서드 ---
    def perform_a1_and_rotate(self): # 함수 이름은 그대로 유지합니다.
        self.get_logger().info("🗺️ A-2 위치로 이동 시작...") 
        if "A-2" in self.location_map: 
            x, y, yaw = self.location_map["A-2"]
            target_pose_a1 = create_pose(x, y, yaw, self.navigator)
            self.go_to_pose_blocking(target_pose_a1, "A-2 주차 위치")
            
            time.sleep(2.0) # 도착 후 2초 대기
            self.get_logger().info("⏳ 2초 대기 완료.")

            # 180도 회전 로직
            twist_msg = Twist()
            twist_msg.angular.z = 0.5  # rad/s

            duration = 3.141592 / 0.5  # π rad / 속도 ≒ 6.28초 (180도 회전을 위한 시간)
            self.get_logger().info(f"↪️ 제자리 180도 회전 시작 (예상 {duration:.2f}초)")

            start_time = self.get_clock().now().seconds_nanoseconds()[0]
            while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
                self.cmd_vel_pub.publish(twist_msg)
                time.sleep(0.1)

            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            self.get_logger().info("✅ 180도 회전 완료, 로봇 정지!")
            
        else:
            self.get_logger().error("❌ A-2 위치가 location_map에 정의되어 있지 않습니다.") 
    # --- 메서드 끝 ---

    # object_map_pose_callback 함수 전체를 삭제합니다.
    # def object_map_pose_pose_callback(self, msg):
    #    ...

    def go_to_pose_blocking(self, pose, description):
        self.get_logger().info(f"🚗 이동 시작: {description}")
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
        result = self.navigator.getResult()
        # 수정된 부분
        if result == self.navigator.TaskResult.SUCCEEDED:
            self.get_logger().info(f"✅ {description} 이동 완료!")
        else:
            self.get_logger().error(f"❌ {description} 이동 실패: {result}")


    def go_to_initial_pose(self):
        initial = create_pose(*self.initial_xyyaw, self.navigator)
        self.navigator.setInitialPose(initial)
        self.get_logger().info('✅ AMCL 초기 Pose 설정 완료')
        time.sleep(1.0)
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('✅ Nav2 활성화 완료')

        # ✅ 추가: AMCL 초기 pose로 실제 이동해야 현재 pose가 확정됨
        self.go_to_pose_blocking(initial, "초기 위치 이동")





    def location_callback(self, msg):
        # go_to_wait_pose() 호출을 제거합니다.
        # self.go_to_wait_pose() 

        location = msg.data.strip()
        self.get_logger().info(f"📌 Received parking command: {location}")

        if location not in self.location_map:
            self.get_logger().warn(f"❗ Unknown location: {location}")
            return

        x, y, yaw = self.location_map[location]
        target_pose = create_pose(x, y, yaw, self.navigator)
        self.go_to_pose_blocking(target_pose, f"지정 주차 위치: {location}")
        time.sleep(3.0)


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