# tf_ref1.py

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PointStamped

import tf2_ros

import tf2_geometry_msgs  # 중요!

from tf2_geometry_msgs import do_transform_point



class TfPointTransform(Node):

    def __init__(self):

        super().__init__('tf_point_transform')



        # TF Buffer/Listener

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)



        # YOLO 노드로부터 PointStamped 메시지 구독

        self.create_subscription(PointStamped, 'depth_point', self.point_callback, 10)

        self.get_logger().info("Waiting for PointStamped messages on 'depth_point'...")



    def point_callback(self, msg: PointStamped):

        self.get_logger().info(f"📌 수신된 msg.header.stamp: {msg.header.stamp}")

        self.get_logger().info(f"📌 현재 노드 시각: {self.get_clock().now().to_msg()}")



        try:

            # ✅ 최신 시간 기준으로 변환 시도

            tf = self.tf_buffer.lookup_transform(

                'map',

                msg.header.frame_id,

                rclpy.time.Time(),  # 최신 시각

                timeout=rclpy.duration.Duration(seconds=0.5)

            )

            transformed_point = do_transform_point(msg, tf)



            self.get_logger().info(

                f"[{msg.header.frame_id} → map] (x={transformed_point.point.x:.2f}, "

                f"y={transformed_point.point.y:.2f}, z={transformed_point.point.z:.2f})"

            )

        except Exception as e:

            self.get_logger().warn(f"TF 변환 실패: {e}")



def main():

    rclpy.init()

    node = TfPointTransform()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()



if __name__ == '__main__':

    main()