import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 현재 노드가 포함된 패키지 이름 (확인 필요)
    # 예시: 'my_robot_pkg' 또는 'auto_parking_pkg'
    package_name = 'rokey_pjt' # 여기에 move_object_front.py 파일이 있는 패키지 이름을 입력하세요.

    node_executable = 'move_object_front' # setup.py의 entry_points에 정의된 실행 파일 이름

    # 로봇이 TF를 게시하는 네임스페이스
    robot_namespace = '/robot3'

    return LaunchDescription([
        Node(
            package=package_name,
            executable=node_executable,
            name='auto_parking_node', # move_object_front.py 코드에서 정의된 노드 이름
            output='screen',
            # TF 토픽 리매핑 설정
            remappings=[
                ('/tf', f'{robot_namespace}/tf'),
                ('/tf_static', f'{robot_namespace}/tf_static'),
                # 추가적으로 노드에서 사용하는 토픽이 있다면 여기에 리매핑을 추가할 수 있습니다.
                # 예시:
                # ('/robot3/oakd/stereo/camera_info', f'{robot_namespace}/oakd/stereo/camera_info'),
                # ('/detect/object_info', f'{robot_namespace}/detect/object_info'),
                # ('/robot3/cmd_vel', f'{robot_namespace}/cmd_vel'),
            ]
        )
    ])