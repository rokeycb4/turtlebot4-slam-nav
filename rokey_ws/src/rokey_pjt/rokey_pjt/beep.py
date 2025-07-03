import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool

class Beep(Node):
    def __init__(self):
        super().__init__('beep_node')
        self.pub = self.create_publisher(AudioNoteVector, '/robot3/cmd_audio', 10)
        self.danger_sub = self.create_subscription(Bool, '/danger_state', self.danger_callback, 10)
        self.alarm_on = False
        self.beep_timer = None 
        self.get_logger().info("beep")
        # 1초 후에 소리 전송
        #self.timer = self.create_timer(1, self.timer_callback)
    
    '''
    def danger_callback(self, msg):
        if msg.data and not self.alarm_on:
            #self.play_beep()
            self.alarm_on = True
            self.get_logger().info("alarm 시작")
            self.count+=1
            if self.count<3:
                self.start_beep_timer()
        elif not msg.data and self.alarm_on:
            self.get_logger().info("Danger cleared. Stopping beep.")
            self.alarm_on = False
            self.stop_beep_timer()
    '''
    def danger_callback(self, msg):
        if msg.data:
            if not self.alarm_on:
                self.alarm_on = True
                self.count = 0
                self.get_logger().info("alarm 시작")
                self.start_beep_timer()
        else:
            if self.alarm_on:
                self.get_logger().info("Danger cleared. Stopping beep.")
                self.alarm_on = False
                self.stop_beep_timer()

    def start_beep_timer(self):
        if self.beep_timer is None:
            self.beep_timer = self.create_timer(1.0, self.play_beep)
            self.get_logger().info("Beep timer started.")

    def stop_beep_timer(self):
        if self.beep_timer is not None:
            self.beep_timer.cancel()
            self.beep_timer = None
            self.get_logger().info("Beep timer stopped.")
    '''
    def play_beep(self):
        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 삐
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 뽀
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 삐
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 뽀
        ]

        self.get_logger().info('삐뽀삐보 소리 전송 중...')
        self.pub.publish(msg)
        self.get_logger().info('삐뽀삐보 소리 전송 완료...')
        #self.timer.cancel()  # 타이머 종료
    '''
    def play_beep(self):
        if self.count >= 3:
            self.get_logger().info("Beep count limit reached. Stopping beep.")
            self.stop_beep_timer()
            return

        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),
        ]

        self.pub.publish(msg)
        self.count += 1
        self.get_logger().info(f'삐뽀삐보 소리 전송 완료... (count={self.count})')


def main(args=None):
    rclpy.init(args=args)
    node = Beep()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()
