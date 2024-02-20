# ackermann_vehicle

tested in ubuntu 20.04, ROS noetic
## 1. installation

```bash
cd catkin_ws/src
git clone https://github.com/0-keun/ackermann_vehicle.git
sudo apt install ros-noetic-ackermann-msgs
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## 2. run in gazebo
```bash
roslaunch ackermann_vehicle_gazebo ackermann_vehicle_noetic.launch
```

publish /cmd_vel -> 동작

테스트 해보고 싶은 경우
```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.2}}'
```
## 3. 수정

- 맵 수정 시
	arkermann_vehicle_gazebo/worlds 에 world 파일 추가 후, ackermann_vehicle_noetic.launch에서 world 파일 선언된 부분 수정
