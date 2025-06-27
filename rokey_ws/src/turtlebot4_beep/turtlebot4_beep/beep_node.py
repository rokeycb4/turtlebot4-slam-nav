#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration

class AudioSignalNode(Node):
    def __init__(self):
        super().__init__('audio_signal_node')
        self.publisher = self.create_publisher(AudioNoteVector, '/robot3/cmd_audio', 10)

        # 타이머 예: 1초 후에 한 번 신호음 발행(1초 후 콜백 실행) 노드가 생성되고 퍼블리셔 토픽이 연결이 되었는데 왜 1초를 쉬어야할까?
        # 만약 시스템아키텍처가 ssh로 로봇안에서 실행이 되어질 경우 내부보드안에서 실행하면 필요없을 수 있음. + 시뮬에서도 필요없음
        # 그러나 실물 로봇 기반 노트북(네트워크) 상에서 토픽 메시지를 날리는 과정에서 삐뽀삐뽀 토픽을 호출시 아직 미완성커넥션(불안정) 상태로 컴 또는 로봇에서 오류발생(리붓을 해야할 경우도 발생)
        self.timer = self.create_timer(1.0, self.publish_audio_notes)

        self.published = False  # 한번만 발행하도록 플래그

    def publish_audio_notes(self): # 콜백 함수
        if self.published:
            return

        notes = [
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 삐
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 뽀
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 삐
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 뽀
        ]

        msg = AudioNoteVector()
        msg.append = False
        msg.notes = notes

        self.publisher.publish(msg)
        self.get_logger().info('AudioNoteVector message published!')

        self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = AudioSignalNode()
    # rclpy.spin(node)
    rclpy.spin_once(node)  # 한 번만 돌고 종료
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
