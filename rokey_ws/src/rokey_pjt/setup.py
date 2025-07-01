import os # os 모듈 임포트
from glob import glob # glob 모듈 임포트
from setuptools import find_packages, setup

package_name = 'rokey_pjt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 🚀 수정된 부분: '.launch.py' 파일들을 정확하게 포함하도록 glob 패턴 수정
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
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

        'detect_ps_map = rokey_pjt.detect_ps_map:main',
        'detect_ps_front = rokey_pjt.detect_ps_front:main',

        ],
    },
)