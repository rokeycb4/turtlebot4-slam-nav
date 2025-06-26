from setuptools import find_packages, setup

package_name = 'yolov8_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'image_pub = 9_1_image_publisher:main',
        'image_sub = 9_2_image_subscriber:main',
        'data_pub = 9_3_data_publisher:main',
        'data_sub = 9_4_data_subscriber:main',
        'yolo_pub = 9_5_yolo_publisher:main',
        'yolo_sub = 9_6_yolo_subscriber:main',
        'opencv_image_capture = opencv_image_capture:main',
        'yolo_live_detect = yolov8_ros.yolo_live_detect_node:main',
        ],
    },
)
