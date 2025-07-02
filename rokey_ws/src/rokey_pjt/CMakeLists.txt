cmake_minimum_required(VERSION 3.8)
project(rokey_pjt)

# 기본
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# .srv → Python 모듈
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetMapPose.srv"
  DEPENDENCIES geometry_msgs
)

# Python 모듈 설치
ament_python_install_package(${PROJECT_NAME})

# ament export
ament_package()
