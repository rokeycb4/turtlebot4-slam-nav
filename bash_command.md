
## 의존성 + 종속성 관리
```bash
sudo rosdep init
rosdep update

rosdep install --from-path src -yi --rosdistro humble

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

ros2 launch turtlebot4_navigation localization.launch.py \
  namespace:=/robot3 \
  map:=$HOME/rokey_ws/maps/first_map1.yaml \
  params_file:=$HOME/rokey_ws/configs/local2.yaml

ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3

ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot3

ros2 run rokey_pjt tf_trans --ros-args -r __ns:=/robot3 -r /tf:=/robot3/tf -r /tf_static:=/robot3/tf_static
```

## depth + TF transform
```bash

ros2 run rokey_pjt yolo_detectros2 run rokey_pjt yolo_tf


```

