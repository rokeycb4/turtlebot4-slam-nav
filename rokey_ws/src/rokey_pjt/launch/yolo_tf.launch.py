import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'rokey_pjt' 


    node_executable = 'yolo_tf' # setup.py entry_points

    # 로봇이 TF를 게시하는 네임스페이스 
    robot_namespace = '/robot3'

    return LaunchDescription([
        Node(
            package=package_name,
            executable=node_executable,
            name='yolo_tf_node',
            output='screen',
            # TF 토픽 리매핑 설정
            remappings=[
                ('/tf', f'{robot_namespace}/tf'),
                ('/tf_static', f'{robot_namespace}/tf_static'),
            ]
        )
    ])