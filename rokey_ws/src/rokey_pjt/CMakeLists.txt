cmake_minimum_required(VERSION 3.8)
project(rokey_pjt)

# 필수 의존성
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 인터페이스 자동 생성 (.srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetMapPose.srv"
)

# Python 설치
ament_python_install_package(${PROJECT_NAME})

# ament 패키지로 선언
ament_package()
