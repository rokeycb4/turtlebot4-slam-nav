
## 의존성 + 종속성 관리
```bash
sudo rosdep init
rosdep update

rosdep install --from-path src -yi --rosdistro humble

```

## USER PC Network Setup
```bash
wget -qO - https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/turtlebot4_discovery/configure_discovery.sh | bash <(cat) </dev/tty


source .bashrc
192.168.0.4
192.168.103.3

ros2 daemon stop
ros2 daemon start

nano /etc/turtlebot4_discovery/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot3/cmd_vel  # 테스트

```



### 로봇 이동
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot3/cmd_vel

```


## SSH 접속
```bash
cd /opt/ros/humble/share/turtlebot4_bringup/config

# 원본백업
sudo cp turtlebot4.yaml turtlebot4_origin.yaml

#설정
sudo nano turtlebot4.yaml
sudo nano oakd_pro.yaml

sudo reboot
```

## 네트워크/CPU 부하 확인  
```bash
ssh ubuntu@192.168.103.3
nmcli device status
sudo iftop -i wlan0 
htop
gnome-system-monitor
```


## Robot_TF Transform
```bash
## 깃 클론

#sudo apt update
#sudo apt install ros-humble-irobot-create-control

ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot3 map:=$HOME/github_package/turtlebot4-slam-nav/turtlebot4_ws/maps/first_map.yaml


ros2 launch turtlebot4_navigation localization.launch.py \
  namespace:=/robot3 \
  map:=$HOME/github_package/turtlebot4-slam-nav/turtlebot4_ws/maps/first_map.yaml \
  params_file:=$HOME/github_package/turtlebot4-slam-nav/turtlebot4_ws/configs/local2.yaml


ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3 params_file:=$HOME/github_package/turtlebot4-slam-nav/turtlebot4_ws/configs/nav2_net.yaml

ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot3
ros2 run rokey_pjt tf_trans --ros-args -r __ns:=/robot3 -r /tf:=/robot3/tf -r /tf_static:=/robot3/tf_static
```

```bash
# 홈위치

ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot3 map:=$HOME/rokey_ws/maps/first_map1.yaml

ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3

ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot3

ros2 run rokey_pjt tf_trans --ros-args -r __ns:=/robot3 -r /tf:=/robot3/tf -r /tf_static:=/robot3/tf_static
```

## depth + TF transform
```bash

# 프레임 확인
ros2 launch turtlebot4_viz view_model.launch.py description:=true model:=standard

ros2 run rokey_pjt yolo_detect
#ros2 run rokey_pjt yolo_tf
ros2 launch rokey_pjt yolo_tf.launch.py


```

## 검출 좌표로 이동
```bash
ros2 run rokey_pjt yolo_detect

# ros2 run rokey_pjt move_object_front
ros2 launch rokey_pjt move_object_front.launch.py

ros2 launch rokey_pjt move_object_front.launch.py
ros2 launch rokey_pjt detect_ps_map.launch.py
```


## ocr
```bash
# sudo apt install fonts-noto-cjk

# ros2 run rokey_pjt carplate_ocr
ros2 run rokey_pjt detect_car_info

```


## 네비게이션 테스트
```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot3 map:=$HOME/rokey_ws/maps/first_map1.yaml

ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3

ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot3

ros2 run rokey_pjt navi_test --ros-args -r __ns:=/robot3
# ros2 topic pub /robot3/parking/location std_msgs/msg/String "data: 'A-1'" -1

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot3/cmd_vel  # 테스트

```


## 입차 최종
```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot2 map:=$HOME/rokey_ws/maps/first_map.yaml
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot2
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot2

ros2 run rokey_pjt sc_follow_waypoints2

ros2 run rokey_pjt yolo_detect


```

## 출차 최종
```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot3 map:=$HOME/rokey_ws/maps/first_map1.yaml
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot3

ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3 params_file:=$HOME/turtlebot4_ws/configs/nav2_net.yaml

# ros2 run rokey_pjt yolo_detect
ros2 run rokey_pjt sc_follow_waypoints --ros-args -r __ns:=/robot3


ros2 run rokey_pjt yolo_detect
ros2 run rokey_pjt beep


```