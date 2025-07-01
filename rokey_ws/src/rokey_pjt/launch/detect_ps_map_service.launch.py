# detect_ps_map_service.launch.py
# ros2 launch rokey_pjt detect_ps_map_service.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'rokey_pjt'

    node_executable = 'detect_ps_front_service'  # setup.py entry_points 이름

    robot_namespace = '/robot3'

    return LaunchDescription([
        Node(
            package=package_name,
            executable=node_executable,
            name='detect_ps_front_service_node',
            output='screen',
            remappings=[
                ('/tf', f'{robot_namespace}/tf'),
                ('/tf_static', f'{robot_namespace}/tf_static'),
            ]
        )
    ])
