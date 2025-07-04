
# ros2 run rokey_pjt sc_follow_waypoints2  --ros-args -r __ns:=/robot2


# sc_follow_waypoints2.py

#!/usr/bin/env python3
# 주차
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
from rclpy.qos import qos_profile_sensor_data

import time
import threading

# === Audio 관련 메시지 ===
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration


# === Pose 생성 ===
def create_pose(x, y, yaw_deg, navigator):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    q = quaternion_from_euler(0, 0, yaw_deg * 3.141592 / 180.0)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


# === 본체 클래스 ===
class ParkingLocationCommander(Node):
    def __init__(self):
        super().__init__('parking_location_commander')

        self.navigator = BasicNavigator()
        self.dock_navigator = TurtleBot4Navigator()

        self.location_map = {
            "A-1": (-2.35, -5.01, -90.0),
            "A-2": (-1.32, -5.15, -90.0),
            "B-1": (1.03, -2.06, 0.0),
            "B-2": (0.94, -1.10, 0.0),
            "C-1": (-2.95, -3.54, 180.0),
            "C-2": (-3.04, -4.59, 180.0),
        }

        self.initial_xyyaw = (-0.02, -0.02, 0.0)
        self.wait_xyyaw = (-1.03, -0.02, 0.0)

        self.go_to_initial_pose()

        if self.dock_navigator.getDockedStatus():
            self.get_logger().info('🚦 도킹 상태 → 언도킹 수행')
            self.dock_navigator.undock()
            time.sleep(2.0)

        self.create_subscription(String, '/parking/location', self.location_callback, 10)
        self.get_logger().info('✅ /parking/location 구독 중')

        self.parking_coord = None

        self.create_subscription(
            PoseStamped,
            '/detect/object_map_pose',
            self.object_map_pose_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('✅ /detect/object_map_pose 구독 중')

        self.cmd_vel_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.audio_pub = self.create_publisher(AudioNoteVector, '/robot2/cmd_audio', 10)

    def play_audio(self, mode="here"):
        if mode == "here":
            notes = [
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=100, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
            ]
        elif mode == "here2":
            notes = [
                AudioNote(frequency=660, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=990, max_runtime=Duration(nanosec=300_000_000)),
                AudioNote(frequency=1320, max_runtime=Duration(nanosec=300_000_000)),
            ]
        else:
            self.get_logger().warn(f"❗ 알 수 없는 사운드 모드: {mode}")
            return

        msg = AudioNoteVector()
        msg.append = False
        msg.notes = notes
        self.audio_pub.publish(msg)
        self.get_logger().info(f'🔊 Audio "{mode}" published')
        time.sleep(0.3)


    def object_map_pose_callback(self, msg):
        self.parking_coord = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.orientation
        )
        self.get_logger().info(f"📍 객체 좌표 수신: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")

        if self.parking_coord:
            self.get_logger().warn("🅿️ 주차 시작")
            time.sleep(1.0)
            # 🎵 주차 시작 시 삐삐삐 사운드
            print("오디오1 출력")
            #self.get_logger().info('✅ 오디오 1 출력 ')
            self.play_audio("here")
            time.sleep(1.0)
            obj_pose = create_pose(*self.parking_coord[:2], 0.0, self.navigator)
            self.go_to_pose_blocking(obj_pose, "객체 기반 주차 위치")

            time.sleep(2)
            self.get_logger().warn("✅ 주차 완료")

        else:
            self.get_logger().warn("⚠️ 객체 맵 좌표 없음")

        # ↪️ 180도 회전
        twist_msg = Twist()
        twist_msg.angular.z = 0.5
        duration = 3.141592 / 0.5
        self.get_logger().info(f"↪️ 제자리 회전 시작 ({duration:.1f}초 예상)")

        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while self.get_clock().now().seconds_nanoseconds()[0] - start_time < duration:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)

        self.cmd_vel_pub.publish(Twist())  # 정지
        self.get_logger().info("✅ 회전 완료")
        time.sleep(1.0)
        # 🎵 띠리링 사운드
        print("오디오 2 출력 ")
        #self.get_logger().info('✅ 오디오 2 출력 ')
        self.play_audio("here2")

        # 복귀
        self.go_to_pose_blocking(create_pose(*self.initial_xyyaw, self.navigator), "초기 위치 복귀")

        self.get_logger().info('🚀 도킹 시작')
        self.dock_navigator.dock()
        self.get_logger().info('✅ 도킹 완료')

    def go_to_pose_blocking(self, pose, description):
        self.get_logger().info(f"➡️ 이동: {description}")
        self.navigator.goToPose(pose)
        start_time = self.navigator.get_clock().now()

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                elapsed = self.navigator.get_clock().now() - start_time
                self.get_logger().info(
                    f"➡️ 남은 거리: {feedback.distance_remaining:.2f} m, "
                    f"경과: {elapsed.nanoseconds / 1e9:.1f} s"
                )
            time.sleep(0.2)

    def go_to_initial_pose(self):
        initial = create_pose(*self.initial_xyyaw, self.navigator)
        self.navigator.setInitialPose(initial)
        self.get_logger().info('✅ 초기 Pose 설정 완료')
        time.sleep(1.0)
        self.navigator.waitUntilNav2Active()

    def go_to_wait_pose(self):
        wait_pose = create_pose(*self.wait_xyyaw, self.navigator)
        self.go_to_pose_blocking(wait_pose, "대기 지점")
        time.sleep(5.0)

    def location_callback(self, msg):
        self.go_to_wait_pose()
        location = msg.data.strip()
        self.get_logger().info(f"📌 주차 명령 수신: {location}")

        if location not in self.location_map:
            self.get_logger().warn(f"❗ 잘못된 명령: {location}")
            return

        target_pose = create_pose(*self.location_map[location], self.navigator)
        self.go_to_pose_blocking(target_pose, f"지정 위치: {location}")
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