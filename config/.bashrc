# ------------------------------
#  기본 Bash 설정
# ------------------------------
# 색상 프롬프트 설정 (일반적인 색: 녹색 사용자, 시안 호스트, 흰색 경로)
PS1='\[\e[0;32m\]\u\[\e[0m\]@\[\e[0;36m\]\h\[\e[0m\]:\[\e[0;37m\]\w\[\e[0m\]\$ '

# ------------------------------
#  ROS 2 Humble 환경 설정
# ------------------------------
echo -e "\e[32mROS2 humble is activated!\e[0m"
source /opt/ros/humble/setup.bash

# TurtleBot4 설정
echo -e "\e[36mturtlebot4 is activated\e[0m"
echo -e "\e[36m/etc/turtlebot4_discovery/setup.bash\e[0m"
source /etc/turtlebot4_discovery/setup.bash

# 필요 시 워크스페이스 설정도 여기에 추가
# echo "turtlebot4_ws is activated!"
# source ~/turtlebot4_ws/install/setup.bash

# ------------------------------
#  ROS 2 커맨드 함수들
# ------------------------------

undock() {
  if [ -z "$1" ]; then
    echo -e "\e[31mUsage: undock <namespace>\e[0m"
    return 1
  fi
  ros2 action send_goal /robot$1/undock irobot_create_msgs/action/Undock "{}"
}

dock() {
  if [ -z "$1" ]; then
    echo -e "\e[31mUsage: dock <namespace>\e[0m"
    return 1
  fi
  ros2 action send_goal /robot$1/dock irobot_create_msgs/action/Dock "{}"
}

localize() {
  if [ -z "$1" ]; then
    echo -e "\e[31mUsage: ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot4 map:=$HOME/first_map.yaml\e[0m"
    return 1
  fi
  ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot$1 map:=$HOME/first_map.yaml
}

nav() {
  if [ -z "$1" ]; then
    echo -e "\e[31mUsage: ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot4\e[0m"
    return 1
  fi
  ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot$1
}

rv() {
  if [ -z "$1" ]; then
    echo -e "\e[31mUsage: ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot4\e[0m"
    return 1
  fi
  ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot$1
}

# ------------------------------
#  Alias
# ------------------------------
alias ssh4='ssh ubuntu@172.30.1.38'
alias ssh0='ssh ubuntu@172.30.1.72'
alias rrestart='ros2 daemon stop && sleep 1 && ros2 daemon start'



#추가
alias gb='gedit ~/.bashrc'
alias nl='ros2 node list'
alias tl='ros2 topic list'

alias cbs='colcon build --symlink-install --packages-select'
alias cbp='colcon build --symlink-install --parallel-workers 2'
alias cb='colcon build --symlink-install'                    
alias sis='source install/setup.bash'
alias rmall='rm -rf install build log'


#------------------------------------ 작업 디렉터리로 이동
# ROS2 작업 디렉토리
#export CURRENT_WS=~/github_package/rokeypj_ws
#cd $CURRENT_WS
#source $CURRENT_WS/install/setup.bash