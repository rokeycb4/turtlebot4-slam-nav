cmake_minimum_required(VERSION 3.8)
project(rokey_pjt)

# 필수 의존성
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)  # ✅ 이 줄 추가

# 인터페이스 자동 생성 (.srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetMapPose.srv"
  DEPENDENCIES geometry_msgs  # ✅ 이 줄 추가
)

# Python 설치
ament_python_install_package(${PROJECT_NAME})

# ament 패키지로 선언
ament_package()
