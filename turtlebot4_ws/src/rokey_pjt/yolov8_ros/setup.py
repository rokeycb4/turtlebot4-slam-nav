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
    maintainer='kiwi',
    maintainer_email='mingureion@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_node1 = yolov8_ros.capture_node1:main',
            'test_yolo = yolov8_ros.test_yolo:main',

            'yolo_bbox_depth_checker = yolov8_ros.yolo_bbox_depth_checker:main',
            'yolo_bbox_depth_checker2 = yolov8_ros.yolo_bbox_depth_checker2:main',


        ],
    },
)
