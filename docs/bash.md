
## Robot_TF Transform
```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot3 map:=$HOME/github_package/turtlebot4-slam-nav/turtlebot4_ws/map

ros2 launch turtlebot4_navigation localization.launch.py \
  namespace:=/robot3 \
  map:=$HOME/github_package/turtlebot4-slam-nav/turtlebot4_ws/maps/first_map.yaml \
  params_file:=$HOME/github_package/turtlebot4-slam-nav/turtlebot4_ws/configs/local2.yaml


ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3 params_file:=$HOME/github_package/turtlebot4-slam-nav/turtlebot4_ws/configs/nav2_net.yaml
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot3 params_file:=$HOME/github_package/turtlebot4-slam-nav/turtlebot4_ws/configs/nav2_net2.yaml

ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot3
ros2 run rokey_pjt tf_trans --ros-args -r __ns:=/robot3 -r /tf:=/robot3/tf -r /tf_static:=/robot3/tf_static
```