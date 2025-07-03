import os
from glob import glob
from setuptools import setup

package_name = 'rokey_pjt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # ✅ 고정: 안전하고 확실함
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),

        # # srv 폴더 내의 모든 '.srv' 파일 포함
        # (os.path.join('share', package_name, 'srv'),
        #     glob(os.path.join('srv', '*.srv'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fred',
    maintainer_email='a93450311@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_checker = rokey_pjt.depth_checker:main',
            'depth_checker_mouse = rokey_pjt.depth_checker_mouse:main',
            'tf_trans = rokey_pjt.tf_point_transform:main',
            'yolo_tf = rokey_pjt.yolo_tf:main',
            'yolo_detect = rokey_pjt.yolo_detect:main',
            'detect_pos = rokey_pjt.object_position:main',
            'move_forward = rokey_pjt.move_forward:main',
            'move_object_front = rokey_pjt.move_object_front:main',
            'carplate_ocr = rokey_pjt.carplate_ocr:main',
            'detect_car_info = rokey_pjt.detect_car_info:main',
            'detect_car_info2 = rokey_pjt.detect_car_info:main',


            'sc_follow_waypoints = rokey_pjt.sc_follow_waypoints:main',
            'detect_ps_map = rokey_pjt.detect_ps_map:main',
            'detect_ps_front_service = rokey_pjt.detect_ps_front_service:main',
        ],
    },
)
